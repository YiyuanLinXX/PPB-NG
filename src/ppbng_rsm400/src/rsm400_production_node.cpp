#include "ppbng_rsm400/command_transport.hpp"
#include "ppbng_rsm400/mcp2_protocol.hpp"
#include "ppbng_rsm400/production_activation_gate.hpp"
#include "ppbng_rsm400/readiness_gate.hpp"
#include "ppbng_rsm400/session_binding.hpp"
#include "ppbng_rsm400/telemetry_log_writer.hpp"
#include "ppbng_rsm400/win32_serial_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <limits>
#include <string>

#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/msg/rsm_telemetry.hpp>
#include <ppbng_interfaces/srv/prepare_device.hpp>
#include <ppbng_interfaces/srv/reset_rsm_faults.hpp>
#include <ppbng_interfaces/srv/set_rsm_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ppbng_rsm400
{
using namespace std::chrono_literals;

class Rsm400ProductionNode final : public rclcpp::Node
{
public:
  Rsm400ProductionNode()
  : Node("rsm400_production"),
    gate_(declare_parameter<bool>("hardware_enabled", false))
  {
    com_port_ = declare_parameter<std::string>("com_port", "");
    device_id_ = declare_parameter<std::string>("device_id", "rsm400");
    allow_control_ = declare_parameter<bool>("allow_control", false);
    features_confirmed_ = declare_parameter<bool>("features_confirmed", false);
    of002_available_ = declare_parameter<bool>("of002_available", false);
    of005_available_ = declare_parameter<bool>("of005_available", false);
    readiness_configuration_.require_ready_telemetry =
      declare_parameter<bool>("require_ready_telemetry", false);
    readiness_configuration_.stab_motion_status_confirmed =
      declare_parameter<bool>("stab_motion_status_confirmed", false);
    const auto expected_stab = declare_parameter<std::int64_t>(
      "expected_stab_motion_status", -1);
    readiness_configuration_.expected_stab_motion_status = expected_stab >= 0 && expected_stab <= 9 ?
      static_cast<int>(expected_stab) : -1;
    const auto maximum_error = declare_parameter<std::int64_t>("maximum_ready_error_level", -1);
    readiness_configuration_.maximum_error_level = maximum_error >= 0 && maximum_error <= 9 ?
      static_cast<int>(maximum_error) : -1;
    const auto ready_timeout = declare_parameter<std::int64_t>("ready_timeout_ms", 0);
    ready_timeout_ms_ = ready_timeout > 0 && ready_timeout <= 10'000 ?
      static_cast<int>(ready_timeout) : 0;
    const auto timeout_value = declare_parameter<std::int64_t>("io_timeout_ms", 20);
    io_timeout_ms_ = timeout_value > 0 &&
      timeout_value <= (std::numeric_limits<int>::max)() ? static_cast<int>(timeout_value) : 0;
    session_binding_.set_allowed_output_root(
      declare_parameter<std::string>("allowed_output_root", ""));
    writer_options_.stem = declare_parameter<std::string>("storage_stem", "rsm400");
    const auto flush_records = declare_parameter<std::int64_t>("flush_every_records", 10);
    writer_options_.flush_every_records = flush_records > 0 ?
      static_cast<std::uint64_t>(flush_records) : 0U;

    telemetry_publisher_ = create_publisher<ppbng_interfaces::msg::RsmTelemetry>(
      "~/telemetry", rclcpp::QoS(100).reliable());
    status_publisher_ = create_publisher<ppbng_interfaces::msg::DeviceStatus>(
      "~/status", rclcpp::QoS(10).reliable());
    fault_publisher_ = create_publisher<ppbng_interfaces::msg::FaultEvent>(
      "~/fault_event", rclcpp::QoS(10).reliable());

    prepare_service_ = create_service<ppbng_interfaces::srv::PrepareDevice>(
      "~/prepare", [this](const std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Response> response) {
        const auto result = session_binding_.prepare(request->request_id, request->session_id,
          request->session_directory, gate_.state() == ProductionGateState::started,
          gate_.state() == ProductionGateState::armed);
        if (result.accepted) writer_options_.segments_directory =
          session_binding_.segments_directory();
        response->accepted = result.accepted;
        response->duplicate_request = result.duplicate;
        response->message = result.detail;
        publish_status(result.detail);
      });

    arm_service_ = create_service<std_srvs::srv::Trigger>(
      "~/arm", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const bool storage_valid = session_binding_.bound() &&
          !writer_options_.segments_directory.empty() && !writer_options_.stem.empty() &&
          writer_options_.flush_every_records > 0U;
        ReadinessGate readiness(readiness_configuration_);
        const bool readiness_valid = !readiness.configuration_result().failed() &&
          ready_timeout_ms_ > 0;
        const auto result = gate_.arm(storage_valid && io_timeout_ms_ > 0 && readiness_valid ?
          com_port_ : std::string{});
        response->success = result.accepted;
        response->message = readiness_valid ? result.detail :
          (ready_timeout_ms_ == 0 ? "ready_timeout_ms must be within 1..10000" :
          readiness.configuration_result().detail);
        publish_status(result.detail);
      });
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start(*response);});
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {stop(*response);});
    stabilize_service_ = create_service<std_srvs::srv::Trigger>(
      "~/activate_stabilization",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        execute(ControlRequest{ControlKind::activate_horizon_stabilization}, *response);
      });
    fast_level_service_ = create_service<std_srvs::srv::Trigger>(
      "~/fast_level", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        execute(ControlRequest{ControlKind::trigger_fast_level}, *response);
      });
    reset_service_ = create_service<ppbng_interfaces::srv::ResetRsmFaults>(
      "~/reset_faults", [this](
        const std::shared_ptr<ppbng_interfaces::srv::ResetRsmFaults::Request>,
        std::shared_ptr<ppbng_interfaces::srv::ResetRsmFaults::Response> response) {
        std_srvs::srv::Trigger::Response common;
        execute(ControlRequest{ControlKind::reset_errors}, common);
        response->accepted = common.success;
        response->message = common.message;
      });
    target_service_ = create_service<ppbng_interfaces::srv::SetRsmTarget>(
      "~/set_target", [this](
        const std::shared_ptr<ppbng_interfaces::srv::SetRsmTarget::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::SetRsmTarget::Response> response) {
        if (!std::isfinite(request->roll_deg) || !std::isfinite(request->pitch_deg) ||
          std::abs(request->roll_deg) > 30.0 || std::abs(request->pitch_deg) > 30.0)
        {
          response->accepted = false;
          response->message = "roll and pitch targets must be finite and within +/-30 degrees";
          return;
        }
        ControlRequest command;
        command.kind = ControlKind::set_leveling_target;
        command.roll_centidegrees = static_cast<int>(std::llround(request->roll_deg * 100.0));
        command.pitch_centidegrees = static_cast<int>(std::llround(request->pitch_deg * 100.0));
        std_srvs::srv::Trigger::Response common;
        execute(command, common);
        response->accepted = common.success;
        response->applied_roll_deg = common.success ? command.roll_centidegrees / 100.0 : 0.0;
        response->applied_pitch_deg = common.success ? command.pitch_centidegrees / 100.0 : 0.0;
        response->message = common.message;
      });

    poll_timer_ = create_wall_timer(10ms, [this]() {poll();});
    status_timer_ = create_wall_timer(500ms, [this]() {publish_status(last_detail_);});
    publish_status("inert; COM port is closed and controls are disabled by default");
  }

  ~Rsm400ProductionNode() override
  {
    close_transport();
    if (writer_) (void)writer_->close();
  }

private:
  static std::uint64_t monotonic_ns() noexcept
  {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  FeatureAvailability feature(const bool available) const noexcept
  {
    if (!features_confirmed_) {return FeatureAvailability::unknown;}
    return available ? FeatureAvailability::available : FeatureAvailability::unavailable;
  }

  void start(std_srvs::srv::Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    const auto authorization = gate_.start();
    if (!authorization.accepted) {
      response.message = authorization.detail;
      return;
    }
    auto [writer_result, writer] = TelemetryLogWriter::create(writer_options_);
    if (!writer_result.success) {
      gate_.start_failed();
      response.message = writer_result.detail;
      last_detail_ = response.message;
      return;
    }
    writer_ = std::move(writer);
    transport_ = std::make_unique<Win32SerialTransport>();
    const auto opened = transport_->open_port(com_port_);
    if (!opened.ok()) {
      gate_.start_failed();
      transport_.reset();
      (void)writer_->close();
      writer_.reset();
      response.message = opened.detail;
      last_detail_ = response.message;
      return;
    }
    ++segment_id_;
    ReadinessGate readiness(readiness_configuration_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(ready_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      const auto budget = std::max(1, std::min(io_timeout_ms_,
        static_cast<int>(std::max<std::int64_t>(1, remaining.count()))));
      std::array<char, 512> bytes{};
      const auto received = transport_->read_some(
        bytes.data(), bytes.size(), std::chrono::milliseconds(budget));
      if (received.code == IoCode::timeout) {continue;}
      if (received.code != IoCode::ok || received.bytes == 0U || received.bytes > bytes.size()) {
        fail_start_locked(response, received.detail.empty() ?
          "RSM400 receive failed during readiness check" : received.detail);
        return;
      }
      for (const auto & raw : decoder_.feed(std::string_view(bytes.data(), received.bytes))) {
        try {
          const auto frame = parse_frame(raw);
          const auto telemetry = decode_telemetry(frame);
          if (!publish_telemetry_locked(frame)) {
            const auto detail = last_detail_;
            fail_start_locked(response, detail);
            return;
          }
          const auto observed = readiness.observe(telemetry);
          if (observed.failed()) {
            fail_start_locked(response, observed.detail);
            return;
          }
          if (observed.ready()) {goto readiness_complete;}
        } catch (const std::exception & error) {
          ++parse_errors_;
          last_detail_ = std::string("RSM400 readiness frame rejected: ") + error.what();
        }
      }
    }
    fail_start_locked(response, readiness.timeout().detail);
    return;

readiness_complete:
    CommandClientOptions options;
    options.allow_control = allow_control_;
    options.features.of002_leveling_offset = feature(of002_available_);
    options.features.of005_status_analysis = feature(of005_available_);
    client_ = std::make_unique<CommandClient>(*transport_, options);
    response.success = true;
    response.message = allow_control_ ?
      "RSM400 ready telemetry confirmed; explicit gated controls enabled" :
      "RSM400 ready telemetry confirmed in observe-only mode";
    last_detail_ = response.message;
  }

  void fail_start_locked(std_srvs::srv::Trigger::Response & response, std::string detail)
  {
    client_.reset();
    if (transport_) {transport_->close();}
    transport_.reset();
    decoder_.reset();
    if (writer_) {
      const auto closed = writer_->close();
      if (!closed.success) {detail += "; writer close failed: " + closed.detail;}
      writer_.reset();
    }
    gate_.start_failed();
    response.success = false;
    response.message = "RSM400 readiness failed closed; port/writer closed: " + detail;
    last_detail_ = response.message;
  }

  void stop(std_srvs::srv::Trigger::Response & response)
  {
    close_transport();
    if (writer_) {
      const auto result = writer_->close();
      if (!result.success) {
        response.success = false;
        response.message = result.detail;
      }
      writer_.reset();
    }
    gate_.stop();
    if (response.message.empty()) {
      response.success = true;
      response.message = "RSM400 port closed; node returned to inert state";
    }
    last_detail_ = response.message;
    publish_status(response.message);
  }

  void close_transport() noexcept
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    client_.reset();
    if (transport_) {transport_->close();}
    transport_.reset();
    decoder_.reset();
  }

  void execute(const ControlRequest & request, std_srvs::srv::Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (gate_.state() != ProductionGateState::started || !client_) {
      response.message = "RSM400 control requires an explicitly started port";
      return;
    }
    const auto result = client_->execute(request, std::chrono::milliseconds(io_timeout_ms_));
    response.success = result.ok();
    response.message = result.detail;
    last_detail_ = result.detail;
    for (const auto & frame : result.unsolicited_frames) {
      if (!publish_telemetry_locked(frame)) break;
    }
  }

  void poll()
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (gate_.state() != ProductionGateState::started || !transport_) {return;}
    std::array<char, 512> bytes{};
    const auto result = transport_->read_some(
      bytes.data(), bytes.size(), std::chrono::milliseconds(io_timeout_ms_));
    if (result.code == IoCode::timeout) {return;}
    if (result.code != IoCode::ok) {
      gate_.start_failed();
      last_detail_ = result.detail.empty() ? "RSM400 serial receive failed" : result.detail;
      return;
    }
    for (const auto & raw : decoder_.feed(std::string_view(bytes.data(), result.bytes))) {
      try {
        if (!publish_telemetry_locked(parse_frame(raw))) break;
      } catch (const std::exception & error) {
        ++parse_errors_;
        last_detail_ = std::string("RSM400 frame rejected: ") + error.what();
      }
    }
  }

  bool publish_telemetry_locked(const Frame & frame)
  {
    const auto decoded = decode_telemetry(frame);
    ppbng_interfaces::msg::RsmTelemetry message;
    message.status_time = now();
    message.host_receive_monotonic_ns = monotonic_ns();
    message.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    message.device_id = device_id_;
    message.segment_id = segment_id_;
    message.sample_sequence = samples_received_ + 1U;
    message.roll_valid = decoded.roll_deg.has_value();
    message.roll_deg = decoded.roll_deg.value_or(0.0);
    message.pitch_valid = decoded.pitch_deg.has_value();
    message.pitch_deg = decoded.pitch_deg.value_or(0.0);
    message.yaw_valid = decoded.yaw_deg.has_value();
    message.yaw_deg = decoded.yaw_deg.value_or(0.0);
    message.timer_valid = decoded.timer_ms.has_value();
    message.mount_timer_ms = decoded.timer_ms.value_or(0);
    message.general_status_valid = decoded.general_status.has_value();
    if (decoded.general_status) {
      message.control_source = decoded.general_status->control_source;
      message.major_status = decoded.general_status->major_status;
      message.motion_status = decoded.general_status->motion_status;
      message.built_in_test_status = decoded.general_status->built_in_test_status;
      message.error_level = decoded.general_status->error_level;
    }
    message.error_groups_valid = decoded.error_groups.has_value();
    if (decoded.error_groups) {message.error_bits = decoded.error_groups->error_bits;}
    message.raw_frame = frame.raw;
    message.validation_detail = "MCP2 checksum and documented fields validated";
    const auto stored = writer_ ? writer_->append(frame, decoded,
      static_cast<std::uint64_t>(message.status_time.sec) * 1'000'000'000ULL +
      message.status_time.nanosec, message.host_receive_monotonic_ns, segment_id_) :
      TelemetryLogResult{false, "RSM telemetry writer is not open"};
    if (!stored.success) {
      client_.reset();
      if (transport_) transport_->close();
      transport_.reset();
      decoder_.reset();
      gate_.start_failed();
      last_detail_ = "RSM storage fault; port closed: " + stored.detail;
      publish_fault(last_detail_);
      return false;
    }
    telemetry_publisher_->publish(message);
    ++samples_received_;
    last_sample_monotonic_ns_ = message.host_receive_monotonic_ns;
    last_detail_ = message.validation_detail;
    return true;
  }

  void publish_fault(const std::string & detail)
  {
    const auto host = monotonic_ns();
    ppbng_interfaces::msg::FaultEvent fault;
    fault.event_id = device_id_ + "-storage-" + std::to_string(++fault_count_);
    fault.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    fault.source_id = device_id_;
    fault.fault_code = "RSM_STORAGE_FATAL";
    fault.severity = fault.SEVERITY_FATAL;
    fault.first_host_monotonic_ns = host;
    fault.last_host_monotonic_ns = host;
    fault.occurrence_count = 1U;
    fault.latched = true;
    fault.causes_global_stop = true;
    fault.detail = detail;
    fault_publisher_->publish(fault);
  }

  void publish_status(const std::string & detail)
  {
    ppbng_interfaces::msg::DeviceStatus status;
    status.status_time = now();
    status.status_host_monotonic_ns = monotonic_ns();
    status.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    status.device_id = device_id_;
    status.required = true;
    status.lifecycle_state = gate_.state() == ProductionGateState::started ? 5U :
      (gate_.state() == ProductionGateState::armed ? 4U : 0U);
    status.health = gate_.state() == ProductionGateState::fault ? 3U :
      (gate_.state() == ProductionGateState::started ? 1U : 0U);
    status.segment_id = segment_id_;
    status.last_sample_valid = samples_received_ != 0U;
    status.last_sample_sequence = samples_received_;
    status.last_sample_host_monotonic_ns = last_sample_monotonic_ns_;
    status.samples_received = samples_received_;
    status.samples_incomplete = parse_errors_;
    status.detail = detail;
    status_publisher_->publish(status);
  }

  ProductionActivationGate gate_;
  std::string com_port_;
  std::string device_id_;
  bool allow_control_{false};
  bool features_confirmed_{false};
  bool of002_available_{false};
  bool of005_available_{false};
  ReadinessConfiguration readiness_configuration_;
  int io_timeout_ms_{20};
  int ready_timeout_ms_{0};
  std::mutex io_mutex_;
  std::unique_ptr<Win32SerialTransport> transport_;
  std::unique_ptr<CommandClient> client_;
  SessionBinding session_binding_;
  TelemetryLogOptions writer_options_;
  std::unique_ptr<TelemetryLogWriter> writer_;
  StreamDecoder decoder_;
  std::uint32_t segment_id_{0};
  std::uint64_t samples_received_{0};
  std::uint64_t parse_errors_{0};
  std::uint64_t last_sample_monotonic_ns_{0};
  std::uint64_t fault_count_{0};
  std::string last_detail_{"inert"};

  rclcpp::Publisher<ppbng_interfaces::msg::RsmTelemetry>::SharedPtr telemetry_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::DeviceStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_publisher_;
  rclcpp::Service<ppbng_interfaces::srv::PrepareDevice>::SharedPtr prepare_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stabilize_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fast_level_service_;
  rclcpp::Service<ppbng_interfaces::srv::ResetRsmFaults>::SharedPtr reset_service_;
  rclcpp::Service<ppbng_interfaces::srv::SetRsmTarget>::SharedPtr target_service_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace ppbng_rsm400

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppbng_rsm400::Rsm400ProductionNode>());
  rclcpp::shutdown();
  return 0;
}
