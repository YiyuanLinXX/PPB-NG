#include "ppbng_thermal/thermal_adapter.hpp"
#include <sstream>
#include <stdexcept>

namespace ppbng_thermal
{
bool StopToken::stop_requested() const noexcept {return flag_ && flag_->load();}
StopSource::StopSource() : flag_(std::make_shared<std::atomic_bool>(false)) {}
void StopSource::request_stop() noexcept {flag_->store(true);}

namespace
{
void hash_bytes(std::uint64_t & hash, const std::string & value) noexcept
{
  for (const unsigned char byte : value) {hash ^= byte; hash *= 1099511628211ULL;}
}
template<typename T> void hash_value(std::uint64_t & hash, const T value) noexcept
{hash_bytes(hash, std::to_string(value)); hash_bytes(hash, "|");}
}

std::uint64_t configuration_hash(const ThermalConfiguration & c) noexcept
{
  std::uint64_t hash = 14695981039346656037ULL;
  hash_value(hash, c.width); hash_value(hash, c.transport_height); hash_value(hash, c.image_height);
  hash_value(hash, c.row_stride_bytes); hash_value(hash, c.payload_bytes);
  hash_bytes(hash, c.pixel_format); hash_bytes(hash, "|"); hash_bytes(hash, c.ir_format);
  hash_bytes(hash, "|"); hash_bytes(hash, c.frame_sync_source); hash_bytes(hash, "|");
  hash_bytes(hash, c.frame_sync_mode); hash_bytes(hash, "|"); hash_bytes(hash, c.frame_sync_polarity);
  hash_value(hash, c.require_ready); hash_value(hash, c.require_fpa_cold);
  return hash;
}

Status validate_a6701_contract(const ThermalReadback & r)
{
  if (r.access != AccessMode::control) return {ErrorCode::access_denied, "control access required"};
  if (r.require_ready && !r.ready) return {ErrorCode::not_ready, "camera electronics not ready"};
  if (r.require_fpa_cold && !r.fpa_cold) return {ErrorCode::not_cold, "FPA not cold"};
  if (r.width != 640 || r.transport_height != 513 || r.image_height != 512 ||
    r.row_stride_bytes != 1280 || r.payload_bytes != 656640 || r.pixel_format != "Mono16" ||
    r.ir_format != "Radiometric" || r.frame_sync_source != "External" ||
    (r.frame_sync_mode != "Integration" && r.frame_sync_mode != "Readout") ||
    (r.frame_sync_polarity != "ActiveHigh" && r.frame_sync_polarity != "ActiveLow"))
    return {ErrorCode::invalid_configuration, "A6701 640x513 header-first contract mismatch"};
  return {};
}

FrameLease::FrameLease(FrameInfo info, std::shared_ptr<std::vector<std::byte>> storage,
  std::function<void()> release) : info_(info), storage_(std::move(storage)), release_(std::move(release)) {}
FrameLease::~FrameLease() {release();}
FrameLease::FrameLease(FrameLease && other) noexcept : info_(other.info_),
  storage_(std::move(other.storage_)), release_(std::move(other.release_))
{other.release_ = {}; other.storage_.reset();}
FrameLease & FrameLease::operator=(FrameLease && other) noexcept
{if (this != &other) {release(); info_ = other.info_; storage_ = std::move(other.storage_); release_ = std::move(other.release_); other.release_ = {}; other.storage_.reset();} return *this;}
void FrameLease::release() noexcept
{if (release_) {auto callback = std::move(release_); storage_.reset(); callback();} else {storage_.reset();}}

BoundedFrameQueue::BoundedFrameQueue(const std::size_t capacity) : capacity_(capacity)
{if (capacity == 0) throw std::invalid_argument("queue capacity must be positive");}
bool BoundedFrameQueue::try_push(FrameLease frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.size() >= capacity_) {++statistics_.backpressure_events; ++statistics_.dropped_newest; return false;}
  queue_.push_back(std::move(frame)); ++statistics_.pushed; condition_.notify_one(); return true;
}
FrameResult BoundedFrameQueue::pop(const std::chrono::milliseconds timeout, const StopToken & stop)
{
  if (timeout.count() <= 0) return {{ErrorCode::timeout, "finite positive timeout required"}, {}};
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (queue_.empty() && !stop.stop_requested()) {
    if (condition_.wait_until(lock, deadline) == std::cv_status::timeout) break;
  }
  if (stop.stop_requested()) return {{ErrorCode::cancelled, "stop requested"}, {}};
  if (queue_.empty()) return {{ErrorCode::timeout, "queue pop timed out"}, {}};
  FrameLease frame = std::move(queue_.front()); queue_.pop_front(); ++statistics_.popped;
  return {{}, std::move(frame)};
}
QueueStatistics BoundedFrameQueue::statistics() const {std::lock_guard<std::mutex> lock(mutex_); return statistics_;}
std::size_t BoundedFrameQueue::size() const {std::lock_guard<std::mutex> lock(mutex_); return queue_.size();}
}  // namespace ppbng_thermal
