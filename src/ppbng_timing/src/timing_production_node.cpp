#include "ppbng_timing/production_control.hpp"
#include "ppbng_timing/timing_event_writer.hpp"

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>

#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/msg/pps_anchor.hpp>
#include <ppbng_interfaces/msg/trigger_event.hpp>
#include <ppbng_interfaces/srv/prepare_device.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ppbng_timing {
namespace {

std::uint32_t u32_parameter(std::int64_t value) {
  return value > 0 && static_cast<std::uint64_t>(value) <=
                          (std::numeric_limits<std::uint32_t>::max)()
             ? static_cast<std::uint32_t>(value)
             : 0U;
}

std::uint16_t u16_parameter(std::int64_t value) {
  return value > 0 && value <= (std::numeric_limits<std::uint16_t>::max)()
             ? static_cast<std::uint16_t>(value)
             : 0U;
}

std::uint32_t nonnegative_u32_parameter(std::int64_t value, const std::string& name) {
  if (value < 0 || static_cast<std::uint64_t>(value) >
                       (std::numeric_limits<std::uint32_t>::max)())
    throw std::invalid_argument(name + " must fit uint32 and be nonnegative");
  return static_cast<std::uint32_t>(value);
}

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string channel_name(Channel channel) {
  switch (channel) {
    case Channel::kFx10e: return "fx10e";
    case Channel::kSwir: return "swir";
    case Channel::kRgb: return "rgb";
    case Channel::kThermal: return "thermal";
  }
  return "invalid";
}

std::uint8_t quality(TimeLock lock) {
  switch (lock) {
    case TimeLock::kLocked: return 2U;
    case TimeLock::kHoldover: return 1U;
    case TimeLock::kUnsynced: return 0U;
  }
  return 0U;
}

}  // namespace

class TimingProductionNode final : public rclcpp::Node {
 public:
  TimingProductionNode() : Node("timing_production") {
    ProductionConfiguration config;
    config.hardware_enabled = declare_parameter<bool>("hardware_enabled", false);
    config.trusted_usb_identity_mapping =
        declare_parameter<bool>("trusted_usb_identity_mapping", false);
    config.identity.com_path = declare_parameter<std::string>("com_path", "");
    config.identity.baud_rate = u32_parameter(declare_parameter<std::int64_t>("baud_rate", 0));
    config.identity.usb_vid = u16_parameter(declare_parameter<std::int64_t>("usb_vid", 0));
    config.identity.usb_pid = u16_parameter(declare_parameter<std::int64_t>("usb_pid", 0));
    config.identity.usb_serial = declare_parameter<std::string>("usb_serial", "");
    const auto expected_boot = declare_parameter<std::int64_t>("expected_boot_id", 0);
    if (expected_boot > 0 && static_cast<std::uint64_t>(expected_boot) <=
                                 (std::numeric_limits<std::uint32_t>::max)())
      config.identity.expected_boot_id = static_cast<std::uint32_t>(expected_boot);
    config.command_timeout = std::chrono::milliseconds(
        declare_parameter<std::int64_t>("command_timeout_ms", 500));
    config.poll_timeout = std::chrono::milliseconds(
        declare_parameter<std::int64_t>("poll_timeout_ms", 5));
    const auto after_pps = declare_parameter<std::int64_t>("arm_after_pps_sequence", 0);
    if (after_pps < 0) throw std::invalid_argument("arm_after_pps_sequence must be nonnegative");
    config.arm_after_pps_sequence = static_cast<std::uint64_t>(after_pps);
    config.schedule.schedule_id = u32_parameter(
        declare_parameter<std::int64_t>("schedule_id", 0));
    config.schedule.ticks_per_second = u32_parameter(
        declare_parameter<std::int64_t>("ticks_per_second", 0));
    config.schedule.channels = {
      read_channel("fx10e", Channel::kFx10e), read_channel("swir", Channel::kSwir),
      read_channel("rgb", Channel::kRgb), read_channel("thermal", Channel::kThermal)};
    device_id_ = declare_parameter<std::string>("device_id", "timing_controller");
    session_binding_ = std::make_unique<TimingSessionBinding>(
        declare_parameter<std::string>("allowed_output_root", ""));
    flush_every_events_ = u32_parameter(
        declare_parameter<std::int64_t>("flush_every_events", 128));

    control_ = std::make_unique<ProductionControl>(
        std::move(config), [] {
          return std::make_unique<TimingControllerClient>(
              std::make_unique<WindowsControllerSerialTransport>());
        });
    trigger_publisher_ = create_publisher<ppbng_interfaces::msg::TriggerEvent>(
        "trigger_event", 256);
    pps_publisher_ = create_publisher<ppbng_interfaces::msg::PpsAnchor>(
        "pps_anchor", 64);
    status_publisher_ = create_publisher<ppbng_interfaces::msg::DeviceStatus>(
        "device_status", 10);
    fault_publisher_ = create_publisher<ppbng_interfaces::msg::FaultEvent>(
        "fault_event", 10);
    prepare_service_ = create_service<ppbng_interfaces::srv::PrepareDevice>("prepare",
      [this](const std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Request> request,
             std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Response> response) {
        const auto result = session_binding_->prepare(request->request_id, request->session_id,
            request->session_directory, control_->state() != ProductionState::inert);
        response->accepted = result.accepted;
        response->duplicate_request = result.duplicate;
        response->message = result.detail;
        publish_status(result.detail);
      });
    arm_service_ = service("arm", [this] {
      if (storage_faulted_) return ProductionResult{false, "storage fault is latched"};
      if (!session_binding_->bound())
        return ProductionResult{false, "successful PrepareDevice is required before arm"};
      if (flush_every_events_ == 0U)
        return ProductionResult{false, "flush_every_events must be positive"};
      return control_->prepare();
    });
    start_service_ = service("start", [this] { return start_with_writer(); });
    arm_next_pps_service_ = service("arm_next_pps", [this] { return control_->arm_next_pps(); });
    start_without_pps_service_ = service(
        "arm_immediate", [this] { return control_->start_without_pps(); });
    disarm_keep_config_service_ = service(
        "disarm_keep_config", [this] { return control_->disarm_keep_configuration(); });
    stop_service_ = service("stop", [this] { return stop_with_writer(); });
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { poll(); });
    publish_status("inert; no COM port opened; parameters are captured for preflight");
  }

 private:
  ChannelSchedule read_channel(const std::string& name, Channel channel) {
    ChannelSchedule value;
    value.channel = channel;
    value.enabled = declare_parameter<bool>(name + ".enabled", false);
    value.rate_numerator_hz = u32_parameter(
        declare_parameter<std::int64_t>(name + ".rate_numerator_hz", 0));
    value.rate_denominator = u32_parameter(
        declare_parameter<std::int64_t>(name + ".rate_denominator", 1));
    value.pulse_width_ticks = u32_parameter(
        declare_parameter<std::int64_t>(name + ".pulse_width_ticks", 0));
    value.phase_ticks = nonnegative_u32_parameter(
        declare_parameter<std::int64_t>(name + ".phase_ticks", 0), name + ".phase_ticks");
    return value;
  }

  using TriggerService = std_srvs::srv::Trigger;
  rclcpp::Service<TriggerService>::SharedPtr service(
      const std::string& name, std::function<ProductionResult()> operation) {
    return create_service<TriggerService>(name,
      [this, operation = std::move(operation)](
          const std::shared_ptr<TriggerService::Request>,
          std::shared_ptr<TriggerService::Response> response) {
        const auto result = operation();
        response->success = result.success;
        response->message = result.detail;
        publish_status(result.detail);
        if (!result.success && control_->state() == ProductionState::fault)
          publish_fault("TIMING_CONTROLLER_FATAL", result.detail);
      });
  }

  ProductionResult start_with_writer() {
    if (storage_faulted_ || !session_binding_->bound())
      return {false, "healthy prepared dataset binding is required"};
    auto started = control_->start();
    if (!started.success) return started;
    auto created = TimingEventWriter::create(session_binding_->directory());
    if (!created.first.ok()) {
      const auto stopped = control_->stop();
      storage_faulted_ = true;
      return {false, "FATAL timing log exclusive-create failed: " + created.first.detail +
          (stopped.success ? "" : "; controller stop unconfirmed: " + stopped.detail)};
    }
    writer_ = std::move(created.second);
    events_since_flush_ = 0U;
    return {true, "controller frozen and append-only timing log exclusively created"};
  }

  ProductionResult stop_with_writer() {
    ProductionResult storage_result{true, "timing log closed"};
    if (writer_) {
      const auto flushed = writer_->flush();
      if (!flushed.ok()) {
        storage_faulted_ = true;
        storage_result = {false, "FATAL timing log flush failed: " + flushed.detail};
      }
      writer_.reset();
    }
    const auto stopped = control_->stop();
    if (!storage_result.success)
      return {false, storage_result.detail +
          (stopped.success ? "" : "; controller stop unconfirmed: " + stopped.detail)};
    return stopped;
  }

  void poll() {
    if (control_->state() != ProductionState::started &&
        control_->state() != ProductionState::armed) return;
    auto result = control_->poll();
    for (const auto& event : result.events) {
      TimingLogStatus stored;
      if (const auto* trigger = std::get_if<ReceivedTriggerEvent>(&event)) {
        stored = writer_ ? writer_->append_trigger(
            trigger->trigger, trigger->host_receive_monotonic_ns)
                         : TimingLogStatus{TimingLogCode::write_failed, "timing writer missing"};
        if (stored.ok()) publish_trigger(trigger->trigger);
      }
      if (const auto* anchor = std::get_if<ReceivedPpsAnchor>(&event)) {
        stored = writer_ ? writer_->append_pps(
            anchor->anchor, anchor->host_receive_monotonic_ns)
                         : TimingLogStatus{TimingLogCode::write_failed, "timing writer missing"};
        if (stored.ok()) publish_pps(anchor->anchor, anchor->host_receive_monotonic_ns);
      }
      if (const auto* status = std::get_if<StatusReport>(&event))
        publish_status("controller state=" + std::to_string(static_cast<int>(status->state)));
      if (!stored.ok()) {
        storage_fatal(stored.detail);
        return;
      }
      if ((std::holds_alternative<ReceivedTriggerEvent>(event) ||
           std::holds_alternative<ReceivedPpsAnchor>(event)) &&
          ++events_since_flush_ >= flush_every_events_) {
        const auto flushed = writer_->flush();
        if (!flushed.ok()) { storage_fatal(flushed.detail); return; }
        events_since_flush_ = 0U;
      }
    }
    if (!result.result.success) {
      publish_status(result.result.detail);
      publish_fault("TIMING_CONTROLLER_FATAL", result.result.detail);
    }
  }

  void storage_fatal(const std::string& detail) {
    storage_faulted_ = true;
    const auto stopped = control_->stop();
    writer_.reset();
    const auto full = "FATAL timing persistence failure: " + detail +
        (stopped.success ? "" : "; output stop unconfirmed: " + stopped.detail);
    publish_status(full);
    publish_fault("TIMING_PERSISTENCE_FATAL", full);
  }

  void publish_pps(const PpsAnchor& anchor, std::uint64_t host_receive_ns) {
    ppbng_interfaces::msg::PpsAnchor message;
    message.header.stamp = now();
    message.boot_id = anchor.boot_id;
    message.pps_sequence = anchor.pps_sequence;
    message.captured_tick = anchor.captured_tick;
    message.host_receive_monotonic_ns = host_receive_ns;
    message.lock = static_cast<std::uint8_t>(anchor.lock);
    message.utc_valid = anchor.utc_second != (std::numeric_limits<std::int64_t>::min)();
    message.utc_second = message.utc_valid ? anchor.utc_second :
        (std::numeric_limits<std::int64_t>::min)();
    pps_publisher_->publish(message);
  }

  void publish_trigger(const TriggerEvent& event) {
    ppbng_interfaces::msg::TriggerEvent message;
    message.header.stamp = now();  // host publication time, not the hardware UTC edge
    message.channel = channel_name(event.channel);
    message.channel_sequence = event.channel_sequence;
    message.pps_sequence = event.pps_sequence;
    message.offset_ticks = event.offset_ticks;
    message.ticks_per_second = event.ticks_per_second;
    message.time_quality.pps_sequence = event.pps_sequence;
    message.time_quality.hardware_tick = event.offset_ticks;
    message.time_quality.status = quality(event.lock);
    message.time_quality.detail = "UTC unresolved here; use PPS/GNSS association";
    trigger_publisher_->publish(message);
    ++events_published_;
  }

  void publish_status(const std::string& detail) {
    ppbng_interfaces::msg::DeviceStatus message;
    message.status_time = now();
    message.status_host_monotonic_ns = monotonic_ns();
    message.session_id = session_binding_->bound() ? session_binding_->session_id() : "";
    message.device_id = device_id_;
    message.required = true;
    message.lifecycle_state = control_->state() == ProductionState::armed ? 4U :
        (control_->state() == ProductionState::started ? 3U : 0U);
    message.health = storage_faulted_ || control_->state() == ProductionState::fault ? 3U :
        (control_->state() == ProductionState::started ||
         control_->state() == ProductionState::armed ? 1U : 0U);
    message.last_sample_valid = events_published_ != 0U;
    message.last_sample_sequence = events_published_;
    message.samples_received = events_published_;
    message.detail = detail;
    status_publisher_->publish(message);
  }

  void publish_fault(const std::string& code, const std::string& detail) {
    const auto host_time = monotonic_ns();
    ppbng_interfaces::msg::FaultEvent message;
    message.event_id = device_id_ + "-" + std::to_string(++fault_count_);
    message.source_id = device_id_;
    message.fault_code = code;
    message.severity = 3U;
    message.first_host_monotonic_ns = host_time;
    message.last_host_monotonic_ns = host_time;
    message.occurrence_count = 1U;
    message.latched = true;
    message.causes_global_stop = true;
    message.detail = detail;
    fault_publisher_->publish(message);
    RCLCPP_FATAL(get_logger(), "%s", detail.c_str());
  }

  std::string device_id_;
  std::unique_ptr<ProductionControl> control_;
  std::unique_ptr<TimingSessionBinding> session_binding_;
  std::unique_ptr<TimingEventWriter> writer_;
  bool storage_faulted_{};
  std::uint32_t flush_every_events_{};
  std::uint32_t events_since_flush_{};
  std::uint64_t events_published_{};
  std::uint64_t fault_count_{};
  rclcpp::Publisher<ppbng_interfaces::msg::TriggerEvent>::SharedPtr trigger_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::PpsAnchor>::SharedPtr pps_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::DeviceStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_publisher_;
  rclcpp::Service<TriggerService>::SharedPtr arm_service_;
  rclcpp::Service<TriggerService>::SharedPtr start_service_;
  rclcpp::Service<TriggerService>::SharedPtr arm_next_pps_service_;
  rclcpp::Service<TriggerService>::SharedPtr start_without_pps_service_;
  rclcpp::Service<TriggerService>::SharedPtr disarm_keep_config_service_;
  rclcpp::Service<TriggerService>::SharedPtr stop_service_;
  rclcpp::Service<ppbng_interfaces::srv::PrepareDevice>::SharedPtr prepare_service_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace ppbng_timing

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppbng_timing::TimingProductionNode>());
  rclcpp::shutdown();
  return 0;
}
