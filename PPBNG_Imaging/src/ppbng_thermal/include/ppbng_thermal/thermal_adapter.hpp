#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ppbng_thermal
{

class StopToken
{
public:
  bool stop_requested() const noexcept;
private:
  friend class StopSource;
  explicit StopToken(std::shared_ptr<std::atomic_bool> flag) : flag_(std::move(flag)) {}
  std::shared_ptr<std::atomic_bool> flag_;
};

class StopSource
{
public:
  StopSource();
  StopToken token() const {return StopToken(flag_);}
  void request_stop() noexcept;
private:
  std::shared_ptr<std::atomic_bool> flag_;
};

enum class ErrorCode {none, invalid_state, timeout, cancelled, not_found, access_denied,
    not_ready, not_cold, invalid_configuration, readback_mismatch, incomplete_frame, io_error};
struct Status {ErrorCode code{ErrorCode::none}; std::string detail; bool ok() const noexcept {return code == ErrorCode::none;}};
enum class AccessMode {none, read_only, control};
enum class FrameSyncEvidence {unverified, verified_fssi, verified_fssr};
enum class LifecycleState {idle, discovered, open, configured, armed, streaming, stopped, faulted};

struct ThermalConfiguration
{
  std::size_t width{640};
  std::size_t transport_height{513};
  std::size_t image_height{512};
  std::size_t row_stride_bytes{1280};
  std::size_t payload_bytes{656640};
  std::string pixel_format{"Mono16"};
  std::string ir_format{"Radiometric"};
  std::string frame_sync_source{"External"};
  std::string frame_sync_mode{"Integration"};
  std::string frame_sync_polarity{"ActiveHigh"};
  bool require_ready{true};
  bool require_fpa_cold{true};
};

struct ThermalReadback : ThermalConfiguration
{
  bool ready{false};
  bool fpa_cold{false};
  AccessMode access{AccessMode::none};
  FrameSyncEvidence frame_sync_evidence{FrameSyncEvidence::unverified};
  bool correction_digital_enabled_valid{false};
  bool correction_digital_enabled{false};
  bool correction_video_enabled_valid{false};
  bool correction_video_enabled{false};
  bool correction_auto_enabled_valid{false};
  bool correction_auto_enabled{false};
  bool correction_auto_use_delta_time_valid{false};
  bool correction_auto_use_delta_time{false};
  bool correction_auto_use_delta_temp_valid{false};
  bool correction_auto_use_delta_temp{false};
  bool flag_present_valid{false};
  bool flag_present{false};
};

std::uint64_t configuration_hash(const ThermalConfiguration & configuration) noexcept;
Status validate_a6701_contract(const ThermalReadback & readback);

struct ThermalMetadataValue
{
  std::string key;
  std::string value;
  bool available{false};
};

struct ThermalCalibrationSnapshot
{
  std::vector<ThermalMetadataValue> values;
};

const ThermalMetadataValue * find_metadata_value(
  const ThermalCalibrationSnapshot & snapshot, const std::string & key) noexcept;
std::uint64_t calibration_snapshot_hash(const ThermalCalibrationSnapshot & snapshot) noexcept;
Status validate_a6701_calibration_snapshot(const ThermalCalibrationSnapshot & snapshot);

struct FrameInfo
{
  std::uint64_t frame_id{0};
  std::uint64_t segment_index{0};
  std::size_t payload_bytes{0};
  bool complete{true};
  // Spinnaker 4.4 Image::GetTimeStamp() is documented in nanoseconds.
  std::uint64_t camera_timestamp_ns{0};
  // Host-side monotonic timestamp captured immediately after image receipt.
  std::chrono::steady_clock::time_point host_receive_monotonic{};
  bool nuc_status_valid{false};
  bool nuc_active{false};
  bool correction_auto_in_progress{false};
  std::string correction_status;
  std::string correction_status_text;
  std::string flag_state;
};

class FrameLease
{
public:
  FrameLease() = default;
  FrameLease(FrameInfo info, std::shared_ptr<std::vector<std::byte>> storage,
    std::function<void()> release);
  ~FrameLease();
  FrameLease(FrameLease && other) noexcept;
  FrameLease & operator=(FrameLease && other) noexcept;
  FrameLease(const FrameLease &) = delete;
  FrameLease & operator=(const FrameLease &) = delete;
  void release() noexcept;
  explicit operator bool() const noexcept {return static_cast<bool>(storage_);}
  const FrameInfo & info() const noexcept {return info_;}
  const std::byte * data() const noexcept {return storage_ ? storage_->data() : nullptr;}
  std::size_t size() const noexcept {return storage_ ? storage_->size() : 0;}
private:
  FrameInfo info_{};
  std::shared_ptr<std::vector<std::byte>> storage_;
  std::function<void()> release_;
};

struct FrameResult {Status status; FrameLease frame;};

class IThermalBackend
{
public:
  virtual ~IThermalBackend() = default;
  virtual Status discover(std::chrono::milliseconds timeout, const StopToken & stop,
    std::vector<std::string> & device_ids) = 0;
  virtual Status open(const std::string & device_id, std::chrono::milliseconds timeout,
    const StopToken & stop) = 0;
  virtual Status configure(const ThermalConfiguration & configuration,
    std::chrono::milliseconds timeout, const StopToken & stop) = 0;
  virtual Status readback(std::chrono::milliseconds timeout, const StopToken & stop,
    ThermalReadback & readback) = 0;
  virtual Status calibration_snapshot(std::chrono::milliseconds timeout, const StopToken & stop,
    ThermalCalibrationSnapshot & snapshot) = 0;
  virtual Status arm(std::chrono::milliseconds timeout, const StopToken & stop) = 0;
  virtual FrameResult next_frame(std::chrono::milliseconds timeout, const StopToken & stop) = 0;
  virtual Status recover(std::chrono::milliseconds timeout, const StopToken & stop) = 0;
  virtual Status stop(std::chrono::milliseconds timeout) noexcept = 0;
  virtual LifecycleState state() const noexcept = 0;
};

struct QueueStatistics {std::uint64_t pushed{0}; std::uint64_t popped{0};
    std::uint64_t backpressure_events{0}; std::uint64_t dropped_newest{0};};

class BoundedFrameQueue
{
public:
  explicit BoundedFrameQueue(std::size_t capacity);
  bool try_push(FrameLease frame);
  FrameResult pop(std::chrono::milliseconds timeout, const StopToken & stop);
  QueueStatistics statistics() const;
  std::size_t size() const;
private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<FrameLease> queue_;
  QueueStatistics statistics_;
};

}  // namespace ppbng_thermal
