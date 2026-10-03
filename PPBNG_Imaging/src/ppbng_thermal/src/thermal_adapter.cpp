#include "ppbng_thermal/thermal_adapter.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
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

const ThermalMetadataValue * find_metadata_value(
  const ThermalCalibrationSnapshot & snapshot, const std::string & key) noexcept
{
  const auto found = std::find_if(
    snapshot.values.begin(), snapshot.values.end(),
    [&key](const ThermalMetadataValue & value) {return value.key == key;});
  return found == snapshot.values.end() ? nullptr : &*found;
}

std::uint64_t calibration_snapshot_hash(const ThermalCalibrationSnapshot & snapshot) noexcept
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto & value : snapshot.values) {
    // These nodes describe the instantaneous camera/FFC state, not the selected
    // calibration or the object/environment correction inputs.  Excluding them
    // makes the hash a session-configuration lock rather than a temperature
    // sensor sample; otherwise a legitimate recovery after the FPA warms by a
    // fraction of a degree would be rejected as a configuration change.
    if (value.key == "DeviceTemperature" || value.key == "FPACold" ||
      value.key == "Ready" || value.key == "CorrectionAutoInProgress")
    {
      continue;
    }
    hash_bytes(hash, value.key); hash_bytes(hash, "=");
    hash_bytes(hash, value.available ? value.value : "<UNAVAILABLE>"); hash_bytes(hash, "|");
  }
  return hash;
}

Status validate_a6701_calibration_snapshot(const ThermalCalibrationSnapshot & snapshot)
{
  const auto required = [&snapshot](const std::string & key) -> const ThermalMetadataValue * {
    const auto * value = find_metadata_value(snapshot, key);
    return value && value->available && !value->value.empty() && value->value != "UNAVAILABLE" ?
      value : nullptr;
  };
  const char * fixed_required[] = {
    "CameraModel", "ActivePreset", "CalibrationIsFactoryCalibrated",
    "CalibrationQueryIndex", "CalibrationQueryTag", "CalibrationQueryMinCounts", "CalibrationQueryMaxCounts",
    "CalibrationQueryMinTemp", "CalibrationQueryMaxTemp", "CalibrationQueryOrder",
    "CalibrationQueryR", "CalibrationQueryB", "CalibrationQueryF"};
  for (const char * key : fixed_required) {
    if (!required(key)) {
      return {ErrorCode::invalid_configuration,
        std::string("required A6701 calibration metadata unavailable: ") + key};
    }
  }
  if (required("CameraModel")->value != "A6701") {
    return {ErrorCode::invalid_configuration, "calibration metadata CameraModel is not A6701"};
  }
  if (!required("CalibrationQueryName") && !required("CalibrationQueryTag") &&
    !required("CalibrationQueryLens"))
  {
    return {ErrorCode::invalid_configuration,
      "calibration identity unavailable (name, tag, and lens are all missing)"};
  }
  auto lower = [](std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return value;
  };
  const auto factory = lower(required("CalibrationIsFactoryCalibrated")->value);
  if (factory != "true" && factory != "1" && factory != "yes" && factory != "on") {
    return {ErrorCode::invalid_configuration, "active A6701 calibration is not factory calibrated"};
  }
  const auto number = [&required](const char * key, double & result) {
    const auto * value = required(key);
    if (!value) return false;
    try {
      std::size_t used = 0;
      result = std::stod(value->value, &used);
      return used == value->value.size() && std::isfinite(result);
    } catch (...) {return false;}
  };
  double minimum_counts = 0.0, maximum_counts = 0.0, minimum_temp = 0.0, maximum_temp = 0.0;
  double planck_r = 0.0, planck_b = 0.0, planck_f = 0.0, order_value = 0.0;
  double active_preset = 0.0, query_index = 0.0;
  if (!number("CalibrationQueryMinCounts", minimum_counts) ||
    !number("CalibrationQueryMaxCounts", maximum_counts) || minimum_counts >= maximum_counts ||
    !number("CalibrationQueryMinTemp", minimum_temp) ||
    !number("CalibrationQueryMaxTemp", maximum_temp) || minimum_temp >= maximum_temp ||
    !number("CalibrationQueryR", planck_r) || planck_r <= 0.0 ||
    !number("CalibrationQueryB", planck_b) || planck_b <= 0.0 ||
    !number("CalibrationQueryF", planck_f) || !number("CalibrationQueryOrder", order_value) ||
    !number("ActivePreset", active_preset) || !number("CalibrationQueryIndex", query_index))
  {
    return {ErrorCode::invalid_configuration,
      "A6701 calibration range, Planck parameters, or polynomial order are invalid"};
  }
  if (active_preset != query_index) {
    return {ErrorCode::readback_mismatch,
      "active A6701 preset does not match queried calibration index"};
  }
  const auto order = static_cast<int>(order_value);
  if (order_value != static_cast<double>(order) || order < 0 || order > 6) {
    return {ErrorCode::invalid_configuration, "A6701 calibration polynomial order is invalid"};
  }
  for (int index = 0; index <= order; ++index) {
    const auto key = std::string("CalibrationQueryCoeff") + std::to_string(index);
    double coefficient = 0.0;
    if (!number(key.c_str(), coefficient)) {
      return {ErrorCode::invalid_configuration, "required A6701 coefficient unavailable: " + key};
    }
  }
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
