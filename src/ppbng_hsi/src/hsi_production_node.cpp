#include "ppbng_hsi/production_activation_gate.hpp"
#include "ppbng_hsi/specsensor_adapter.hpp"
#include "ppbng_hsi/hsi_format.hpp"
#include "ppbng_hsi/envi_segment_writer.hpp"
#include "ppbng_hsi/session_binding.hpp"
#include "ppbng_hsi/pending_trigger_matcher.hpp"
#include "ppbng_hsi/fault_policy.hpp"
#include "ppbng_hsi/bounded_recovery.hpp"
#include "ppbng_hsi/sample_stamp_mapper.hpp"
#include "ppbng_hsi/hsi_production_node_factory.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <memory>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifdef SEVERITY_ERROR
#undef SEVERITY_ERROR
#endif
#endif

#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/msg/sample_stamp.hpp>
#include <ppbng_interfaces/msg/trigger_event.hpp>
#include <ppbng_interfaces/srv/prepare_device.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ppbng_hsi
{

namespace
{

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          constexpr char digits[] = "0123456789abcdef";
          output << "\\u00" << digits[(character >> 4U) & 0x0fU] <<
            digits[character & 0x0fU];
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  return output.str();
}

// SpecSensor supports more than one open camera in one process, but device
// initialization is a multi-call transaction.  Serializing individual SDK
// calls is insufficient because two nodes can otherwise interleave profile,
// calibration-pack and Initialize operations.  Keep the complete production
// start transaction atomic while preserving independent acquisition callbacks
// and writers after both cameras are ready.
std::mutex g_production_start_mutex;

class ProductionStartTransactionLock
{
public:
  ProductionStartTransactionLock()
  : local_lock_(g_production_start_mutex)
  {
#ifdef _WIN32
    process_lock_ = CreateMutexW(
      nullptr, FALSE, L"Local\\PPBNG_SpecSensor_ProductionStart_v1");
    if (process_lock_ == nullptr) {
      detail_ = "CreateMutexW failed while serializing SpecSensor initialization";
      return;
    }
    const auto wait_result = WaitForSingleObject(process_lock_, 60'000U);
    acquired_ = wait_result == WAIT_OBJECT_0 || wait_result == WAIT_ABANDONED;
    if (!acquired_) {
      detail_ = wait_result == WAIT_TIMEOUT ?
        "timed out waiting for another SpecSensor process to finish initialization" :
        "WaitForSingleObject failed while serializing SpecSensor initialization";
    }
#else
    acquired_ = true;
#endif
  }

  ~ProductionStartTransactionLock()
  {
#ifdef _WIN32
    if (acquired_) {
      (void)ReleaseMutex(process_lock_);
    }
    if (process_lock_ != nullptr) {
      (void)CloseHandle(process_lock_);
    }
#endif
  }

  ProductionStartTransactionLock(const ProductionStartTransactionLock &) = delete;
  ProductionStartTransactionLock & operator=(const ProductionStartTransactionLock &) = delete;

  bool acquired() const noexcept {return acquired_;}
  const std::string & detail() const noexcept {return detail_;}

private:
  std::unique_lock<std::mutex> local_lock_;
  bool acquired_{false};
  std::string detail_;
#ifdef _WIN32
  HANDLE process_lock_{nullptr};
#endif
};

}  // namespace

class HsiProductionNode final : public rclcpp::Node
{
public:
  explicit HsiProductionNode(const rclcpp::NodeOptions & options)
  : Node("hsi_production", options),
    gate_(declare_parameter<bool>("hardware_enabled", false))
  {
    const auto kind_text = declare_parameter<std::string>("camera_kind", "");
    options_.kind = kind_text == "fx10e" ? CameraKind::fx10e : CameraKind::swir;
    options_.transport = options_.kind == CameraKind::fx10e ?
      SpecSensorTransport::pleora_gige : SpecSensorTransport::ni_camera_link;
    options_.device_index = static_cast<int>(declare_parameter<std::int64_t>("device_index", -1));
    options_.license_path = widen_ascii(declare_parameter<std::string>("license_path", ""));
    options_.expected_profile_name = declare_parameter<std::string>("expected_profile_name", "");
    options_.expected_sensor_serial = declare_parameter<std::string>("expected_sensor_serial", "");
    options_.calibration_pack_path = widen_ascii(
      declare_parameter<std::string>("calibration_pack_path", ""));
    options_.grabber_channel = widen_ascii(
      declare_parameter<std::string>("grabber_channel", ""));
    options_.ni_grabber_channel = widen_ascii(
      declare_parameter<std::string>("ni_grabber_channel", ""));
    options_.ni_camera_file_path = widen_ascii(
      declare_parameter<std::string>("ni_camera_file_path", ""));
    options_.ni_camera_serial_port = widen_ascii(
      declare_parameter<std::string>("ni_camera_serial_port", ""));
    options_.pleora_packet_size = positive_u32(
      declare_parameter<std::int64_t>("pleora_packet_size", 0));
    options_.initialization_timeout_ms = positive_u32(
      declare_parameter<std::int64_t>("initialization_timeout_ms", 5000));
    options_.callback_queue_capacity = positive_size(
      declare_parameter<std::int64_t>("callback_queue_capacity", 128));
    options_.maximum_frame_bytes = positive_size(
      declare_parameter<std::int64_t>("maximum_frame_bytes", 8 * 1024 * 1024));

    config_.kind = options_.kind;
    config_.device_id = declare_parameter<std::string>("device_id", "");
    config_.trigger_channel = declare_parameter<std::string>("trigger_channel", "");
    config_.trigger_mode = declare_parameter<std::string>("trigger_mode", "External");
    config_.spatial_samples = positive_u32(
      declare_parameter<std::int64_t>("spatial_samples", 0));
    config_.spectral_bands = positive_u32(
      declare_parameter<std::int64_t>("spectral_bands", 0));
    config_.line_rate_hz = declare_parameter<double>("line_rate_hz", 0.0);
    config_.exposure_us = declare_parameter<double>("exposure_us", 0.0);
    dark_line_count_ = positive_size(declare_parameter<std::int64_t>("dark_line_count", 0));
    writer_options_.session_directory.clear();
    session_binding_.set_allowed_output_root(
      declare_parameter<std::string>("allowed_output_root", ""));
    writer_options_.stream_stem = kind_text;
    writer_options_.maximum_segment_bytes = static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("maximum_segment_bytes", 8LL * 1024LL * 1024LL * 1024LL)));
    writer_options_.flush_every_lines = static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("flush_every_lines", 120)));
    const bool association_evidence_confirmed = declare_parameter<bool>(
      "association_evidence_confirmed", false);
    internal_frame_timeout_ms_ = static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("trigger_match_timeout_ms", 100)));
    internal_first_frame_timeout_ms_ = static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("internal_first_frame_timeout_ms", 10'000)));
    trigger_matcher_ = std::make_unique<PendingTriggerMatcher>(positive_size(
      declare_parameter<std::int64_t>("pending_trigger_capacity", 64)),
      internal_frame_timeout_ms_ * 1'000'000ULL,
      association_evidence_confirmed);
    camera_kind_valid_ = kind_text == "fx10e" || kind_text == "swir";
    recovery_ = BoundedRecovery({positive_u32(declare_parameter<std::int64_t>(
      "recovery_max_attempts", 5)), static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("recovery_initial_backoff_ms", 250))),
      static_cast<std::uint64_t>(positive_size(declare_parameter<std::int64_t>(
      "recovery_max_backoff_ms", 4000)))});
    recovery_timeout_threshold_ = positive_u32(declare_parameter<std::int64_t>(
      "recovery_consecutive_frame_timeouts", 3));
    fail_fast_on_sample_fault_ = declare_parameter<bool>(
      "fail_fast_on_sample_fault", true);

    status_publisher_ = create_publisher<ppbng_interfaces::msg::DeviceStatus>("status", 10);
    fault_publisher_ = create_publisher<ppbng_interfaces::msg::FaultEvent>("fault_event", 10);
    sample_stamp_publisher_ = create_publisher<ppbng_interfaces::msg::SampleStamp>(
      "sample_stamp", 100);
    trigger_subscription_ = create_subscription<ppbng_interfaces::msg::TriggerEvent>(
      "trigger", 32,
      [this](const ppbng_interfaces::msg::TriggerEvent & message) {handle_trigger(message);});
    match_timer_ = create_wall_timer(std::chrono::milliseconds(1),
      [this]() {match_pending_trigger();});
    arm_service_ = make_service("arm", [this]() {return arm();});
    prepare_service_ = create_service<ppbng_interfaces::srv::PrepareDevice>("prepare",
      [this](const std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Response> response) {
        const auto result = session_binding_.prepare(request->request_id, request->session_id,
          request->session_directory, gate_.state() == ProductionGateState::started,
          gate_.state() == ProductionGateState::inert);
        if (result.accepted) {writer_options_.session_directory = session_binding_.directory();}
        response->accepted = result.accepted;
        response->duplicate_request = result.duplicate;
        response->message = result.detail;
        publish_status(result.detail);
      });
    start_service_ = make_service("start", [this]() {return start();});
    dark_service_ = make_service("begin_dark", [this]() {return begin_dark();});
    sample_service_ = make_service("start_sample", [this]() {return start_sample();});
    stop_service_ = make_service("stop", [this]() {return stop();});
    publish_status("inert; no SDK loaded and no hardware opened");
  }

private:
  using TriggerService = std_srvs::srv::Trigger;

  rclcpp::Service<TriggerService>::SharedPtr make_service(
    const std::string & name, std::function<OperationResult()> callback)
  {
    using ServiceCallback = std::function<void(
      std::shared_ptr<TriggerService::Request>,
      std::shared_ptr<TriggerService::Response>)>;
    ServiceCallback service_callback =
      [callback = std::move(callback)](
        const std::shared_ptr<TriggerService::Request>,
        std::shared_ptr<TriggerService::Response> response) mutable {
        const auto result = callback();
        response->success = result.success;
        response->message = result.message;
      };
    return create_service<TriggerService>(name, std::move(service_callback));
  }

  static std::size_t positive_size(const std::int64_t value) noexcept
  {
    return value > 0 ? static_cast<std::size_t>(value) : 0U;
  }


  static std::uint32_t positive_u32(const std::int64_t value) noexcept
  {
    return value > 0 && static_cast<std::uint64_t>(value) <=
      (std::numeric_limits<std::uint32_t>::max)() ? static_cast<std::uint32_t>(value) : 0U;
  }

  static std::wstring widen_ascii(const std::string & value)
  {
    return std::wstring(value.begin(), value.end());
  }

  OperationResult arm()
  {
    const auto options = validate_specsensor_options(options_);
    const auto configuration = HsiFormat::validate_config(config_);
    const bool payload_bound_valid = configuration.success &&
      HsiFormat::payload_bytes_per_line(config_) == options_.maximum_frame_bytes;
    const bool valid = camera_kind_valid_ && specsensor_backend_compiled() && options.success &&
      configuration.success && payload_bound_valid && dark_line_count_ > 0U &&
      writer_configuration_valid();
    const bool fully_valid = valid && trigger_matcher_ && trigger_matcher_->configured() &&
      recovery_timeout_threshold_ > 0U && internal_frame_timeout_ms_ > 0U &&
      internal_first_frame_timeout_ms_ >= internal_frame_timeout_ms_;
    const auto result = gate_.arm(fully_valid);
    publish_status(result.detail);
    return {result.accepted, result.detail};
  }

  OperationResult start()
  {
    const ProductionStartTransactionLock start_lock;
    if (!start_lock.acquired()) {
      const OperationResult result{false, start_lock.detail()};
      publish_status(result.message);
      return result;
    }
    const auto authorization = gate_.start();
    if (!authorization.accepted) {
      publish_status(authorization.detail);
      return {false, authorization.detail};
    }
    const auto diagnostic_result = open_diagnostics();
    if (!diagnostic_result.success) {
      gate_.start_failed();
      publish_status(diagnostic_result.message);
      return diagnostic_result;
    }
    auto [writer_result, writer] = EnviSegmentWriter::create(writer_options_, config_);
    if (!writer_result.success) {
      gate_.start_failed();
      publish_fault(ProductionFaultKind::storage_create, writer_result.message);
      publish_status(writer_result.message);
      close_diagnostics();
      return writer_result;
    }
    writer_ = std::move(writer);
    recovery_.reset();
    capture_phase_ = CapturePhase::none;
    adapter_ = std::make_unique<SpecSensorHsiAdapter>(options_);
    observed_callback_overflows_ = 0U;
    observed_invalid_callback_frames_ = 0U;
    callback_queue_depth_ = 0U;
    callback_queue_high_watermark_ = 0U;
    writer_last_latency_us_ = 0U;
    writer_max_latency_us_ = 0U;
    auto result = adapter_->connect();
    if (result.success) {result = adapter_->configure(config_);}
    if (result.success) {result = adapter_->close_shutter();}
    if (result.success) {
      const auto readback = adapter_->readback();
      result.message = "SpecSensor ready with shutter closed: profile='" +
        readback.profile_name + "', serial='" + readback.sensor_serial +
        "', geometry=" + std::to_string(readback.width) + "x" +
        std::to_string(readback.height) + "x" + std::to_string(readback.byte_depth) +
        " bytes, frame_bytes=" + std::to_string(readback.frame_bytes) +
        ", packet_size=" + std::to_string(readback.pleora_packet_size) +
        ", calibration_pack_loaded=" +
        std::string(readback.calibration_pack_loaded ? "true" : "false");
    }
    if (!result.success) {
      const auto closed = writer_->close();
      if (!closed.success) {publish_fault(ProductionFaultKind::storage_flush_close, closed.message);}
      writer_.reset();
      gate_.start_failed();
      publish_fault(ProductionFaultKind::device_transport, result.message);
      adapter_.reset();
    }
    publish_status(result.message);
    if (!result.success) {close_diagnostics();}
    return result;
  }

  OperationResult begin_dark()
  {
    if (!adapter_ || gate_.state() != ProductionGateState::started) {
      return {false, "begin_dark requires a successful arm and start"};
    }
    trigger_matcher_->clear();
    const auto result = adapter_->begin_dark_capture(dark_line_count_);
    if (result.success) {
      capture_phase_ = CapturePhase::dark;
      arm_internal_frame_deadline();
    }
    if (!result.success) {publish_fault(ProductionFaultKind::device_transport, result.message);}
    publish_status(result.message);
    return result;
  }

  OperationResult start_sample()
  {
    if (!adapter_ || gate_.state() != ProductionGateState::started) {
      return {false, "start_sample requires a successful arm and start"};
    }
    auto result = adapter_->open_shutter();
    if (result.success) {result = adapter_->start_streaming();}
    if (result.success) {
      capture_phase_ = CapturePhase::sample;
      arm_internal_frame_deadline();
    }
    if (!result.success) {publish_fault(ProductionFaultKind::device_transport, result.message);}
    publish_status(result.message);
    return result;
  }

  OperationResult stop()
  {
    recovery_.stop();
    const ProductionStartTransactionLock lifecycle_lock;
    if (!lifecycle_lock.acquired()) {
      const OperationResult result{false, lifecycle_lock.detail()};
      publish_status(result.message);
      return result;
    }
    OperationResult result{true, "node returned to inert state"};
    if (adapter_) {
      result = adapter_->stop_streaming();
      if (!result.success) {publish_fault(ProductionFaultKind::device_transport, result.message);}
      if (result.success && (adapter_->state() == HsiState::ready ||
        adapter_->state() == HsiState::configured))
      {
        result = adapter_->close_shutter();
        if (!result.success) {
          publish_fault(ProductionFaultKind::device_transport,
            "failed to close HSI shutter during stop: " + result.message);
        }
      }
      adapter_.reset();
      if (result.success) {
        result.message =
          "node returned to inert state; shutter closed; SpecSensor handle and SDK released";
      }
    }
    if (writer_) {
      const auto storage_result = writer_->close();
      if (!storage_result.success) {result = storage_result;publish_fault(ProductionFaultKind::storage_flush_close, storage_result.message);}
      writer_.reset();
    }
    trigger_matcher_->clear();
    capture_phase_ = CapturePhase::none;
    internal_frame_deadline_ms_ = 0U;
    gate_.stop();
    publish_status(result.message);
    close_diagnostics();
    return result;
  }

  void handle_trigger(const ppbng_interfaces::msg::TriggerEvent & message)
  {
    if (!adapter_ || config_.trigger_mode != "External" ||
      message.channel != config_.trigger_channel) {return;}
    if (recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting)
    {
      ++samples_dropped_;
      return;
    }
    TriggerEvent trigger;
    trigger.channel = message.channel;
    trigger.channel_sequence = message.channel_sequence;
    trigger.pps_sequence = message.pps_sequence;
    trigger.offset_ticks = message.offset_ticks;
    trigger.ticks_per_second = message.ticks_per_second;
    trigger.utc_time_ns = static_cast<std::int64_t>(message.time_quality.utc_time.sec) *
      1'000'000'000LL + message.time_quality.utc_time.nanosec;
    trigger.time_status = message.time_quality.status == 2U ? TimeStatus::locked :
      (message.time_quality.status == 1U ? TimeStatus::holdover : TimeStatus::unsynced);
    trigger.uncertainty_ns = message.time_quality.uncertainty_ns;
    const auto now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    const auto queued = trigger_matcher_->enqueue(trigger, now_ns);
    if (!queued.accepted) {
      ++samples_dropped_;
      publish_status(queued.detail);
    }
  }

  void match_pending_trigger()
  {
    const auto now_ms = monotonic_ms();
    if (recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting)
    {
      recovery_tick(now_ms);
      return;
    }
    if (recovery_.state() == RecoveryState::exhausted ||
      recovery_.state() == RecoveryState::fatal || recovery_.state() == RecoveryState::stopped)
    {
      return;
    }
    if (!adapter_ || (adapter_->state() != HsiState::streaming &&
      adapter_->state() != HsiState::dark_collecting)) {return;}
    if (!observe_callback_queue_health()) {return;}
    if (config_.trigger_mode == "Internal") {
      // Drain a bounded batch per timer tick. At the configured maximum of
      // 120 lines/s this remains comfortably ahead while never monopolizing
      // the ROS executor if a backlog appears after scheduling latency.
      for (std::size_t index = 0U; index < 32U; ++index) {
        auto internal = adapter_->poll_internal();
        if (internal.status == LineStatus::not_ready) {
          if (internal_frame_deadline_ms_ == 0U) {arm_internal_frame_deadline();}
          if (now_ms >= internal_frame_deadline_ms_) {
            ++samples_lost_;
            ++consecutive_frame_timeouts_;
            internal_frame_deadline_ms_ = now_ms + internal_frame_timeout_ms_;
            if (consecutive_frame_timeouts_ >= recovery_timeout_threshold_) {
              begin_recovery(RecoveryFaultClass::timeout,
                "no internally timed HSI frame arrived before the bounded deadline");
              return;
            }
            publish_status("internally timed HSI frame deadline missed; waiting within bounded policy");
          }
          break;
        }
        if (internal.status == LineStatus::produced) {
          consecutive_frame_timeouts_ = 0U;
          if (!persist_line(std::move(*internal.line), internal.message)) {return;}
          // CRC and synchronous disk persistence may make a bounded drain
          // batch span hundreds of milliseconds. Anchor the watchdog to the
          // time the frame was actually persisted, not the stale timer-entry
          // timestamp, or healthy high-payload FX10e traffic can self-trigger
          // false recovery under dual-camera I/O load.
          internal_frame_deadline_ms_ = monotonic_ms() + internal_frame_timeout_ms_;
          continue;
        }
        if (internal.status == LineStatus::disconnected) {
          ++samples_dropped_;
          begin_recovery(RecoveryFaultClass::transport, internal.message);
        } else {
          ++samples_dropped_;
          (void)recovery_.on_fault(RecoveryFaultClass::integrity, now_ms);
          gate_.start_failed();
          publish_fault(ProductionFaultKind::integrity, internal.message);
          publish_status(internal.message);
        }
        return;
      }
      return;
    }
    const auto now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    auto result = trigger_matcher_->poll(*adapter_, now_ns);
    if (result.status == TriggerMatchStatus::idle ||
      result.status == TriggerMatchStatus::waiting_for_frame) {return;}
    if (result.status == TriggerMatchStatus::expired) {
      ++samples_lost_;
      ++consecutive_frame_timeouts_;
      if (consecutive_frame_timeouts_ >= recovery_timeout_threshold_) {
        begin_recovery(RecoveryFaultClass::timeout, result.detail);
      }
      publish_status(result.detail);
      return;
    }
    if (result.status == TriggerMatchStatus::produced) {
      consecutive_frame_timeouts_ = 0U;
      (void)persist_line(std::move(*result.line), result.detail);
      return;
    }
    if (result.status == TriggerMatchStatus::disconnected) {
      ++samples_dropped_;
      begin_recovery(RecoveryFaultClass::transport, result.detail);
      return;
    }
    if (result.status == TriggerMatchStatus::fault) {
      ++samples_dropped_;
      (void)recovery_.on_fault(RecoveryFaultClass::integrity, now_ms);
      gate_.start_failed();
      publish_fault(ProductionFaultKind::integrity, result.detail);
    }
    publish_status(result.detail);
  }

  bool persist_line(LineRecord line, const std::string & detail)
  {
    const auto capture_kind = line.index.capture_kind;
    const auto writer_begin = std::chrono::steady_clock::now();
    const auto persisted = writer_ ? writer_->append(line) :
      OperationResult{false, "HSI writer is not open"};
    writer_last_latency_us_ = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - writer_begin).count());
    writer_max_latency_us_ = (std::max)(writer_max_latency_us_, writer_last_latency_us_);
    if (!persisted.success) {
      ++samples_dropped_;
      storage_fatal(ProductionFaultKind::storage_write, persisted.message);
      return false;
    }
    ++samples_received_;
    last_sequence_ = line.index.camera_line_sequence;
    if (capture_kind == CaptureKind::sample) {
      const auto stamp = make_sample_stamp(line, session_binding_.session_id());
      if (stamp) {sample_stamp_publisher_->publish(*stamp);}
    }
    if (line.index.association_status != AssociationStatus::matched) {
      ++association_unverified_count_;
    }
    if (capture_kind == CaptureKind::dark && adapter_->state() == HsiState::shutter_closed) {
      capture_phase_ = CapturePhase::none;
      trigger_matcher_->clear();
      publish_status("dark_complete");
    } else if (line.index.sequence_gap_before) {
      publish_status("HSI SDK frame-number gap detected; raw evidence preserved");
    } else if (config_.trigger_mode == "Internal") {
      // Internal timing has no per-line hardware trigger association. Avoid a
      // warning per line; DeviceStatus and SampleStamp still mark it UNSYNCED.
    } else if (line.index.association_status == AssociationStatus::unmatched) {
      publish_status("UNMATCHED frame/trigger association; new source segment opened");
    } else if (line.index.association_status == AssociationStatus::unverified) {
      publish_status("UNVERIFIED frame/trigger association; waiting for delta confirmation");
    } else if (line.index.association_status == AssociationStatus::consistent_unverified) {
      publish_status("CONSISTENT_UNVERIFIED counters; missed-trigger FIFO ambiguity remains");
    } else {
      publish_status(detail);
    }
    return true;
  }

  enum class CapturePhase {none, dark, sample};

  static std::uint64_t monotonic_ms() noexcept
  {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void arm_internal_frame_deadline() noexcept
  {
    internal_frame_deadline_ms_ = config_.trigger_mode == "Internal" ?
      monotonic_ms() + internal_first_frame_timeout_ms_ : 0U;
  }

  void storage_fatal(const ProductionFaultKind kind, const std::string & detail)
  {
    (void)recovery_.on_fault(RecoveryFaultClass::storage, monotonic_ms());
    if (adapter_) (void)adapter_->stop_streaming();
    gate_.start_failed();
    publish_fault(kind, detail);
    publish_status("fatal HSI storage/integrity fault; recovery prohibited: " + detail);
  }

  void begin_recovery(const RecoveryFaultClass kind, const std::string & detail)
  {
    if (recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting) return;
    const auto finalized = writer_ ? writer_->finalize_current_segment() :
      OperationResult{false, "HSI writer is not open during recovery boundary"};
    if (!finalized.success) {
      storage_fatal(ProductionFaultKind::storage_flush_close, finalized.message);
      return;
    }
    if (capture_phase_ == CapturePhase::sample && fail_fast_on_sample_fault_) {
      (void)recovery_.on_fault(RecoveryFaultClass::integrity, monotonic_ms());
      OperationResult stopped{true, "sample acquisition stopped"};
      if (adapter_) {
        stopped = adapter_->stop_streaming();
        if (stopped.success && (adapter_->state() == HsiState::ready ||
          adapter_->state() == HsiState::configured))
        {
          stopped = adapter_->close_shutter();
        }
      }
      capture_phase_ = CapturePhase::none;
      internal_frame_deadline_ms_ = 0U;
      gate_.start_failed();
      const auto fault_kind = kind == RecoveryFaultClass::timeout ?
        ProductionFaultKind::device_timeout : ProductionFaultKind::device_transport;
      const std::string stopped_detail =
        "SAMPLE FAIL-FAST: continuity fault stopped acquisition without reconnecting or "
        "opening a new segment: " + detail + "; safe_stop=" + stopped.message;
      publish_fault(fault_kind, stopped_detail);
      publish_status(stopped_detail);
      return;
    }
    trigger_matcher_->start_new_segment();
    consecutive_frame_timeouts_ = 0U;
    if (!recovery_.on_fault(kind, monotonic_ms())) return;
    publish_fault(kind == RecoveryFaultClass::timeout ? ProductionFaultKind::device_timeout :
      ProductionFaultKind::device_transport, detail);
    publish_status("HSI recovery scheduled; writer/sidecars finalized and association reset: " + detail);
  }

  void recovery_tick(const std::uint64_t now_ms)
  {
    if (!recovery_.take_attempt(now_ms)) return;
    const ProductionStartTransactionLock lifecycle_lock;
    auto result = lifecycle_lock.acquired() ? adapter_->recover() :
      OperationResult{false, lifecycle_lock.detail()};
    // recover() reselects the frozen device index, verifies exact sensor serial,
    // reapplies configuration/readback, and returns with the shutter open/ready.
    if (result.success && capture_phase_ == CapturePhase::dark) {
      result = adapter_->close_shutter();
      if (result.success) result = adapter_->begin_dark_capture(dark_line_count_);
    } else if (result.success && capture_phase_ == CapturePhase::sample) {
      result = adapter_->start_streaming();
    } else if (result.success) {
      result = adapter_->close_shutter();
    }
    recovery_.finish_attempt(result.success, now_ms);
    if (result.success) {
      ++successful_reconnects_;
      arm_internal_frame_deadline();
      publish_status("HSI recovered exact identity/configuration; new segment and association epoch active");
    } else if (recovery_.state() == RecoveryState::exhausted) {
      gate_.start_failed();
      publish_fault(ProductionFaultKind::device_transport,
        "HSI recovery exhausted after " + std::to_string(recovery_.attempts()) +
        " attempts: " + result.message);
      publish_status("PERSISTENT HSI FAULT: bounded recovery attempts exhausted");
    } else {
      publish_status("HSI recovery attempt failed; next bounded retry scheduled: " + result.message);
    }
  }

  bool observe_callback_queue_health()
  {
    const auto stats = adapter_->queue_stats();
    callback_queue_depth_ = stats.depth;
    callback_queue_high_watermark_ = (std::max)(callback_queue_high_watermark_, stats.depth);
    const auto overflow_delta = stats.overflow_count >= observed_callback_overflows_ ?
      stats.overflow_count - observed_callback_overflows_ : stats.overflow_count;
    const auto invalid_delta = stats.invalid_frame_count >= observed_invalid_callback_frames_ ?
      stats.invalid_frame_count - observed_invalid_callback_frames_ : stats.invalid_frame_count;
    observed_callback_overflows_ = stats.overflow_count;
    observed_invalid_callback_frames_ = stats.invalid_frame_count;
    if (overflow_delta == 0U && invalid_delta == 0U) {return true;}

    const auto lost = overflow_delta > (std::numeric_limits<std::uint64_t>::max)() - invalid_delta ?
      (std::numeric_limits<std::uint64_t>::max)() : overflow_delta + invalid_delta;
    samples_dropped_ = samples_dropped_ > (std::numeric_limits<std::uint64_t>::max)() - lost ?
      (std::numeric_limits<std::uint64_t>::max)() : samples_dropped_ + lost;
    const std::string detail = "SpecSensor callback queue integrity boundary: capacity=" +
      std::to_string(stats.capacity) + ", depth=" + std::to_string(stats.depth) +
      ", high_watermark=" + std::to_string(callback_queue_high_watermark_) +
      ", new_overflows=" + std::to_string(overflow_delta) +
      ", new_invalid_frames=" + std::to_string(invalid_delta) +
      ", writer_last_latency_us=" + std::to_string(writer_last_latency_us_) +
      ", writer_max_latency_us=" + std::to_string(writer_max_latency_us_);
    begin_recovery(RecoveryFaultClass::transport, detail);
    return false;
  }

  void publish_status(const std::string & detail)
  {
    ppbng_interfaces::msg::DeviceStatus status;
    status.status_time = now();
    status.status_host_monotonic_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    status.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    status.device_id = config_.device_id;
    status.required = true;
    status.segment_id = adapter_ ? adapter_->segment_id() : 0U;
    status.lifecycle_state = recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting ? 6U :
      (adapter_ ? lifecycle_for_state(adapter_->state()) : 0U);
    status.health = gate_.state() == ProductionGateState::fault ||
      recovery_.state() == RecoveryState::exhausted || recovery_.state() == RecoveryState::fatal ?
      3U : (recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting ? 2U :
      (samples_lost_ != 0U || samples_dropped_ != 0U || association_unverified_count_ != 0U ?
      2U : (adapter_ ? 1U : 0U)));
    status.reconnect_attempts = recovery_.attempts();
    status.last_sample_valid = samples_received_ != 0U;
    status.last_sample_sequence = last_sequence_;
    status.samples_received = samples_received_;
    status.samples_dropped = samples_dropped_;
    status.samples_lost = samples_lost_;
    status.samples_incomplete = association_unverified_count_ +
      (trigger_matcher_ ? trigger_matcher_->pending() : 0U) + callback_queue_depth_;
    if (adapter_) {
      const auto readback = adapter_->readback();
      if (readback.valid) {
        status.actual_setting_keys = {"frame_rate_hz", "exposure_us", "trigger_mode",
          "width", "height", "byte_depth", "frame_bytes", "sensor_serial"};
        status.actual_setting_values = {std::to_string(readback.frame_rate_hz),
          std::to_string(readback.exposure_us), readback.trigger_mode,
          std::to_string(readback.width), std::to_string(readback.height),
          std::to_string(readback.byte_depth), std::to_string(readback.frame_bytes),
          readback.sensor_serial};
      }
    }
    status.detail = detail;
    status_publisher_->publish(status);
    append_diagnostic("status", status.status_host_monotonic_ns, "", 0U, detail);
  }

  void publish_fault(const ProductionFaultKind kind, const std::string & detail)
  {
    const auto policy = fault_policy(kind);
    const auto host_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
    ppbng_interfaces::msg::FaultEvent event;
    event.event_id = config_.device_id + "-" + policy.code + "-" + std::to_string(++fault_sequence_);
    event.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    event.source_id = config_.device_id;
    event.fault_code = policy.code;
    event.severity = policy.severity;
    event.first_host_monotonic_ns = host_ns;
    event.last_host_monotonic_ns = host_ns;
    event.occurrence_count = 1U;
    event.latched = true;
    event.acknowledged = false;
    event.causes_global_stop = policy.causes_global_stop;
    event.detail = detail;
    fault_publisher_->publish(event);
    append_diagnostic("fault", host_ns, policy.code, policy.severity, detail);
  }

  OperationResult open_diagnostics()
  {
    const auto path = writer_options_.session_directory /
      (writer_options_.stream_stem + "_events.ndjson");
#ifdef _WIN32
    const auto handle = CreateFileW(
      path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
      FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
      return {false, "diagnostic event target already exists or cannot be created exclusively"};
    }
    CloseHandle(handle);
#else
    std::error_code error;
    if (std::filesystem::exists(path, error) || error) {
      return {false, "diagnostic event target already exists or cannot be inspected"};
    }
    {std::ofstream reserved(path, std::ios::binary | std::ios::out | std::ios::trunc);}
#endif
    diagnostics_.open(path, std::ios::binary | std::ios::out | std::ios::app);
    if (!diagnostics_) {
      return {false, "failed to create mandatory HSI diagnostic event log"};
    }
    return {true, "mandatory HSI diagnostic event log opened"};
  }

  void close_diagnostics()
  {
    if (!diagnostics_.is_open()) {return;}
    diagnostics_.flush();
    diagnostics_.close();
  }

  void append_diagnostic(
    const char * type, const std::uint64_t host_ns, const std::string & fault_code,
    const std::uint8_t severity, const std::string & detail)
  {
    if (!diagnostics_.is_open()) {return;}
    const auto stats = adapter_ ? adapter_->queue_stats() : SpecSensorQueueStats{};
    diagnostics_ << "{\"schema\":1,\"type\":\"" << type <<
      "\",\"session_id\":\"" << json_escape(
        session_binding_.bound() ? session_binding_.session_id() : "") <<
      "\",\"device_id\":\"" << json_escape(config_.device_id) <<
      "\",\"host_monotonic_ns\":" << host_ns <<
      ",\"capture_phase\":\"" <<
      (capture_phase_ == CapturePhase::dark ? "dark" :
      (capture_phase_ == CapturePhase::sample ? "sample" : "none")) <<
      "\",\"segment_id\":" << (adapter_ ? adapter_->segment_id() : 0U) <<
      ",\"queue_capacity\":" << stats.capacity <<
      ",\"queue_depth\":" << stats.depth <<
      ",\"queue_high_watermark\":" << callback_queue_high_watermark_ <<
      ",\"writer_last_latency_us\":" << writer_last_latency_us_ <<
      ",\"writer_max_latency_us\":" << writer_max_latency_us_ <<
      ",\"overflow_count\":" << stats.overflow_count <<
      ",\"invalid_frame_count\":" << stats.invalid_frame_count <<
      ",\"samples_received\":" << samples_received_ <<
      ",\"samples_dropped\":" << samples_dropped_ <<
      ",\"samples_lost\":" << samples_lost_ <<
      ",\"fault_code\":\"" << json_escape(fault_code) <<
      "\",\"severity\":" << static_cast<unsigned>(severity) <<
      ",\"detail\":\"" << json_escape(detail) << "\"}\n";
    diagnostics_.flush();
  }

  static std::uint8_t lifecycle_for_state(const HsiState state) noexcept
  {
    switch (state) {
      case HsiState::disconnected: return 0U;
      case HsiState::connected: return 2U;
      case HsiState::configured:
      case HsiState::shutter_closed: return 3U;
      case HsiState::dark_collecting:
      case HsiState::ready: return 4U;
      case HsiState::streaming: return 5U;
      case HsiState::recovering: return 6U;
      case HsiState::fault: return 0U;
    }
    return 0U;
  }

  bool writer_configuration_valid() const
  {
    std::error_code ec;
    return session_binding_.bound() && !writer_options_.session_directory.empty() &&
      std::filesystem::is_directory(writer_options_.session_directory, ec) &&
      writer_options_.maximum_segment_bytes > 0U && writer_options_.flush_every_lines > 0U;
  }

  ProductionActivationGate gate_;
  SpecSensorBackendOptions options_;
  HsiConfig config_;
  bool camera_kind_valid_{false};
  bool fail_fast_on_sample_fault_{true};
  std::size_t dark_line_count_{0U};
  std::unique_ptr<SpecSensorHsiAdapter> adapter_;
  EnviWriterOptions writer_options_;
  SessionBinding session_binding_;
  std::unique_ptr<EnviSegmentWriter> writer_;
  std::unique_ptr<PendingTriggerMatcher> trigger_matcher_;
  BoundedRecovery recovery_;
  CapturePhase capture_phase_{CapturePhase::none};
  std::uint32_t recovery_timeout_threshold_{3U};
  std::uint64_t internal_frame_timeout_ms_{0U};
  std::uint64_t internal_first_frame_timeout_ms_{0U};
  std::uint64_t internal_frame_deadline_ms_{0U};
  std::uint32_t consecutive_frame_timeouts_{0U};
  std::uint32_t successful_reconnects_{0U};
  std::uint64_t samples_received_{0U};
  std::uint64_t samples_dropped_{0U};
  std::uint64_t samples_lost_{0U};
  std::uint64_t association_unverified_count_{0U};
  std::uint64_t observed_callback_overflows_{0U};
  std::uint64_t observed_invalid_callback_frames_{0U};
  std::size_t callback_queue_depth_{0U};
  std::size_t callback_queue_high_watermark_{0U};
  std::uint64_t writer_last_latency_us_{0U};
  std::uint64_t writer_max_latency_us_{0U};
  std::uint64_t last_sequence_{0U};
  std::uint64_t fault_sequence_{0U};
  rclcpp::Publisher<ppbng_interfaces::msg::DeviceStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::SampleStamp>::SharedPtr sample_stamp_publisher_;
  rclcpp::Subscription<ppbng_interfaces::msg::TriggerEvent>::SharedPtr trigger_subscription_;
  rclcpp::TimerBase::SharedPtr match_timer_;
  rclcpp::Service<TriggerService>::SharedPtr arm_service_;
  rclcpp::Service<ppbng_interfaces::srv::PrepareDevice>::SharedPtr prepare_service_;
  rclcpp::Service<TriggerService>::SharedPtr start_service_;
  rclcpp::Service<TriggerService>::SharedPtr dark_service_;
  rclcpp::Service<TriggerService>::SharedPtr sample_service_;
  rclcpp::Service<TriggerService>::SharedPtr stop_service_;
  std::ofstream diagnostics_;
};

}  // namespace ppbng_hsi

std::shared_ptr<rclcpp::Node> ppbng_hsi::make_hsi_production_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<HsiProductionNode>(options);
}
