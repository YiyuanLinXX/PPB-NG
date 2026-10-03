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

namespace ppbng_rgb
{
class StopToken {public: bool stop_requested() const noexcept; private: friend class StopSource; explicit StopToken(std::shared_ptr<std::atomic_bool> f):flag_(std::move(f)){} std::shared_ptr<std::atomic_bool> flag_;};
class StopSource {public: StopSource(); StopToken token() const{return StopToken(flag_);} void request_stop() noexcept; private: std::shared_ptr<std::atomic_bool> flag_;};
enum class ErrorCode {none,invalid_state,timeout,cancelled,not_found,access_denied,invalid_configuration,readback_mismatch,incomplete_frame};
struct Status {ErrorCode code{ErrorCode::none}; std::string detail; bool ok()const noexcept{return code==ErrorCode::none;}};
enum class LifecycleState {idle,discovered,open,configured,armed,streaming,stopped,faulted};

struct RgbConfiguration
{
  std::size_t width{4096}; std::size_t height{3000}; std::size_t row_stride_bytes{4096};
  std::size_t payload_bytes{12'288'000}; std::string pixel_format{"BayerRG8"};
  std::string acquisition_mode{"Continuous"}; std::string trigger_selector{"FrameStart"};
  std::string trigger_source{"Line0"}; std::string trigger_activation{"RisingEdge"};
  std::string exposure_auto{"Continuous"}; std::string gain_auto{"Continuous"};
  std::string balance_white_auto{"Once"}; std::string balance_white_auto_profile{"Outdoor"};
  std::string auto_exposure_control_priority{"Gain"};
  double auto_exposure_time_upper_limit_us{5000.0};
  double auto_exposure_gain_upper_limit_db{12.0};
  bool chunk_data_enabled{true};
  bool hardware_trigger{true};
};
struct RgbReadback : RgbConfiguration {bool control_access{true};};
std::uint64_t configuration_hash(const RgbConfiguration &) noexcept;
Status validate_raw_bayer_contract(const RgbReadback &);
Status select_unique_device_id(
  const std::string & expected_id,
  const std::vector<std::string> & discovered_ids,
  std::string & selected_id);

struct FrameInfo {
  std::uint64_t frame_id{0}; std::uint64_t segment_index{0};
  std::size_t payload_bytes{0}; bool complete{true}; std::string pixel_format;
  // Spinnaker SDK 4.4 documents Image::GetTimeStamp() in nanoseconds.
  std::uint64_t camera_timestamp_ns{0};
  // std::chrono::steady_clock at completion of GetNextImage(); local epoch only.
  std::uint64_t host_receive_monotonic_ns{0};
  bool chunk_data_valid{false};
  bool chunk_frame_id_valid{false}; std::uint64_t chunk_frame_id{0};
  bool chunk_timestamp_valid{false}; std::uint64_t chunk_timestamp{0};
  bool exposure_time_valid{false}; double exposure_time_us{0.0};
  bool gain_valid{false}; double gain_db{0.0};
  bool black_level_valid{false}; double black_level{0.0};
  // BFS-U3-123S6C does not provide white-balance ratios as image chunks.
  // These two values are immediate post-frame node-map readbacks.
  bool white_balance_red_valid{false}; double white_balance_red{0.0};
  bool white_balance_blue_valid{false}; double white_balance_blue{0.0};
  std::string exposure_auto; std::string gain_auto; std::string balance_white_auto;
};
class FrameLease
{
public:
  FrameLease()=default; FrameLease(FrameInfo,std::shared_ptr<std::vector<std::byte>>,std::function<void()>); ~FrameLease();
  FrameLease(FrameLease&&)noexcept; FrameLease&operator=(FrameLease&&)noexcept; FrameLease(const FrameLease&)=delete; FrameLease&operator=(const FrameLease&)=delete;
  void release()noexcept; explicit operator bool()const noexcept{return static_cast<bool>(storage_);} const FrameInfo&info()const noexcept{return info_;} const std::byte*data()const noexcept{return storage_?storage_->data():nullptr;} std::size_t size()const noexcept{return storage_?storage_->size():0;}
private: FrameInfo info_{}; std::shared_ptr<std::vector<std::byte>> storage_; std::function<void()> release_;
};
struct FrameResult {Status status; FrameLease frame;};
class IRgbBackend
{
public: virtual~IRgbBackend()=default;
  virtual Status discover(std::chrono::milliseconds,const StopToken&,std::vector<std::string>&)=0;
  virtual Status open(const std::string&,std::chrono::milliseconds,const StopToken&)=0;
  virtual Status configure(const RgbConfiguration&,std::chrono::milliseconds,const StopToken&)=0;
  virtual Status readback(std::chrono::milliseconds,const StopToken&,RgbReadback&)=0;
  virtual Status arm(std::chrono::milliseconds,const StopToken&)=0;
  virtual FrameResult next_frame(std::chrono::milliseconds,const StopToken&)=0;
  virtual Status recover(std::chrono::milliseconds,const StopToken&)=0;
  virtual Status stop(std::chrono::milliseconds)noexcept=0;
  virtual LifecycleState state()const noexcept=0;
};
struct QueueStatistics {std::uint64_t pushed{0},popped{0},backpressure_events{0},dropped_newest{0};};
class BoundedFrameQueue
{
public: explicit BoundedFrameQueue(std::size_t); bool try_push(FrameLease); FrameResult pop(std::chrono::milliseconds,const StopToken&); QueueStatistics statistics()const;
private: const std::size_t capacity_; mutable std::mutex mutex_; std::condition_variable condition_; std::deque<FrameLease> queue_; QueueStatistics statistics_;
};
} // namespace ppbng_rgb
