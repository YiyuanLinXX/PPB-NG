#include "ppbng_gnss/production_activation_gate.hpp"
#include "ppbng_gnss/um982_receiver.hpp"
#include "ppbng_gnss/gnss_log_writer.hpp"
#include "ppbng_gnss/session_binding.hpp"
#include "ppbng_gnss/fault_policy.hpp"
#include "ppbng_gnss/bounded_recovery.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <filesystem>
#include <limits>
#include <string>

#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/gnss_observation.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/srv/prepare_device.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ppbng_gnss
{
using namespace std::chrono_literals;

class Um982ProductionNode final : public rclcpp::Node
{
public:
  Um982ProductionNode()
  : Node("um982_production"),
    gate_(declare_parameter<bool>("hardware_enabled", false))
  {
    configuration_.com_path = declare_parameter<std::string>("com_path", "");
    configuration_.baud_rate = positive_u32(
      declare_parameter<std::int64_t>("baud_rate", 115200));
    configuration_.read_chunk_bytes = positive_size(
      declare_parameter<std::int64_t>("read_chunk_bytes", 4096));
    configuration_.max_line_bytes = positive_size(
      declare_parameter<std::int64_t>("max_line_bytes", 65536));
    device_id_ = declare_parameter<std::string>("device_id", "um982_receive_only");
    poll_timeout_ms_ = static_cast<int>(declare_parameter<std::int64_t>("poll_timeout_ms", 5));
    writer_options_.session_directory.clear();
    session_binding_.set_allowed_output_root(
      declare_parameter<std::string>("allowed_output_root", ""));
    writer_options_.stream_stem = declare_parameter<std::string>("storage_stem", "um982");
    base_storage_stem_ = writer_options_.stream_stem;
    writer_options_.flush_every_sentences = static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("flush_every_sentences", 10)));
    recovery_ = BoundedRecovery({positive_u32(declare_parameter<std::int64_t>(
      "recovery_max_attempts", 5)), static_cast<std::uint64_t>(positive_size(
      declare_parameter<std::int64_t>("recovery_initial_backoff_ms", 250))),
      static_cast<std::uint64_t>(positive_size(declare_parameter<std::int64_t>(
      "recovery_max_backoff_ms", 4000)))});
    recovery_timeout_threshold_ = positive_u32(declare_parameter<std::int64_t>(
      "recovery_consecutive_receive_timeouts", 200));

    raw_publisher_ = create_publisher<std_msgs::msg::String>("validated_sentence", 100);
    observation_publisher_ = create_publisher<ppbng_interfaces::msg::GnssObservation>(
      "observation", 100);
    status_publisher_ = create_publisher<ppbng_interfaces::msg::DeviceStatus>("status", 10);
    fault_publisher_ = create_publisher<ppbng_interfaces::msg::FaultEvent>("fault_event", 10);
    arm_service_ = create_service<std_srvs::srv::Trigger>("arm",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto result = gate_.arm(configuration_);
        if (result.accepted && (!writer_configuration_valid() || recovery_timeout_threshold_ == 0U)) {
          gate_.stop();
          result = {false, "session/writer settings and recovery_consecutive_receive_timeouts must be valid"};
        }
        response->success = result.accepted;
        response->message = result.detail;
        publish_status(result.detail);
      });
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
    start_service_ = create_service<std_srvs::srv::Trigger>("start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start(*response);});
    stop_service_ = create_service<std_srvs::srv::Trigger>("stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {stop(*response);});
    poll_timer_ = create_wall_timer(10ms, [this]() {poll();});
    publish_status("inert; COM port has not been opened");
  }

private:
  static std::size_t positive_size(const std::int64_t value) noexcept
  {
    return value > 0 ? static_cast<std::size_t>(value) : 0U;
  }

  static std::uint32_t positive_u32(const std::int64_t value) noexcept
  {
    return value > 0 && static_cast<std::uint64_t>(value) <=
      (std::numeric_limits<std::uint32_t>::max)() ? static_cast<std::uint32_t>(value) : 0U;
  }

  void start(std_srvs::srv::Trigger::Response & response)
  {
    const auto authorization = gate_.start();
    if (!authorization.accepted) {
      response.success = false;
      response.message = authorization.detail;
      publish_status(response.message);
      return;
    }
    stop_source_ = std::make_unique<ReceiveStopSource>();
    recovery_.reset();
    consecutive_receive_timeouts_ = 0U;
    storage_segment_id_ = 0U;
    receiver_ = std::make_unique<Um982Receiver>(
      std::make_unique<WindowsReceiveOnlySerial>());
    const auto result = receiver_->connect(
      configuration_, std::chrono::milliseconds(poll_timeout_ms_), stop_source_->token());
    response.success = result.code == ByteReadCode::data;
    response.message = response.success ? "receive-only COM port opened" : result.detail;
    if (!response.success) {publish_fault(ProductionFaultKind::device_transport, result.detail);}
    if (response.success) {
      auto [storage_result, writer] = GnssLogWriter::create(segment_writer_options());
      response.success = storage_result.success;
      response.message = storage_result.detail;
      writer_ = std::move(writer);
      if (!storage_result.success) {publish_fault(ProductionFaultKind::storage_create, storage_result.detail);}
    }
    if (!response.success) {
      if (receiver_) {receiver_->stop();}
      receiver_.reset();
      stop_source_.reset();
      gate_.start_failed();
    }
    publish_status(response.message);
  }

  void stop(std_srvs::srv::Trigger::Response & response)
  {
    std::string finalize_failure;
    recovery_.stop();
    consecutive_receive_timeouts_ = 0U;
    if (stop_source_) {stop_source_->request_stop();}
    if (receiver_) {receiver_->stop();}
    receiver_.reset();
    stop_source_.reset();
    if (writer_) {
      const auto result = writer_->close();
      if (!result.success) {++fault_count_;finalize_failure=result.detail;publish_fault(ProductionFaultKind::storage_flush_close, result.detail);}
      writer_.reset();
    }
    gate_.stop();
    response.success = finalize_failure.empty();
    response.message = response.success ? "receive-only port closed; node is inert" :
      "receive-only port closed but GNSS storage finalization failed: " + finalize_failure;
    publish_status(response.message);
  }

  void poll()
  {
    if (gate_.state() != ProductionGateState::started || !receiver_ || !stop_source_) {
      return;
    }
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
    const auto result = receiver_->poll(
      std::chrono::milliseconds(poll_timeout_ms_), stop_source_->token());
    if (result.code == ByteReadCode::disconnected || result.code == ByteReadCode::io_error) {
      ++fault_count_;
      begin_recovery(result.code == ByteReadCode::io_error ? RecoveryFaultClass::transport :
        RecoveryFaultClass::transport, result.detail);
      return;
    }
    if (result.code == ByteReadCode::timeout) {
      ++consecutive_receive_timeouts_;
      if (consecutive_receive_timeouts_ >= recovery_timeout_threshold_) {
        begin_recovery(RecoveryFaultClass::timeout,
          "GNSS receive remained silent for the configured bounded timeout window");
        consecutive_receive_timeouts_ = 0U;
      }
      return;
    }
    if (result.code == ByteReadCode::cancelled) return;
    consecutive_receive_timeouts_ = 0U;
    for (const auto & sentence : result.sentences) {
      const auto monotonic_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
      const auto persisted = writer_ ? writer_->append(
        sentence, monotonic_ns, receiver_->connection_epoch()) :
        GnssLogResult{false, "GNSS writer is not open"};
      if (!persisted.success) {
        ++fault_count_;
        GnssLogResult closed{true, {}};
        if (writer_) closed = writer_->close();
        writer_.reset();
        if (!closed.success) {
          publish_fault(ProductionFaultKind::storage_flush_close, closed.detail);
        }
        storage_fatal(ProductionFaultKind::storage_write, persisted.detail);
        return;
      }
      if (sentence.kind == SentenceKind::gga || sentence.kind == SentenceKind::uniheadinga) {
        std_msgs::msg::String message;
        message.data = sentence.raw_line;
        raw_publisher_->publish(message);
        publish_observation(sentence, monotonic_ns, receiver_->connection_epoch());
        ++samples_received_;
      } else if (sentence.kind == SentenceKind::parse_error) {
        ++parse_error_count_;
      }
    }
    if (!result.sentences.empty()) {publish_status("validated receive-only GNSS data received");}
  }

  static std::uint64_t monotonic_ms() noexcept
  {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  GnssLogWriterOptions segment_writer_options() const
  {
    auto value = writer_options_;
    value.stream_stem = storage_segment_id_ == 0U ? base_storage_stem_ :
      base_storage_stem_ + "_segment_" + std::to_string(storage_segment_id_);
    return value;
  }

  void storage_fatal(const ProductionFaultKind kind, const std::string & detail)
  {
    (void)recovery_.on_fault(RecoveryFaultClass::storage, monotonic_ms());
    if (stop_source_) stop_source_->request_stop();
    if (receiver_) receiver_->stop();
    gate_.start_failed();
    publish_fault(kind, detail);
    publish_status("fatal GNSS storage/integrity fault; recovery prohibited: " + detail);
  }

  void begin_recovery(const RecoveryFaultClass kind, const std::string & detail)
  {
    if (recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting) return;
    if (writer_) {
      const auto finalized = writer_->close();
      writer_.reset();
      if (!finalized.success) {
        storage_fatal(ProductionFaultKind::storage_flush_close, finalized.detail);
        return;
      }
    }
    if (!recovery_.on_fault(kind, monotonic_ms())) return;
    publish_fault(kind == RecoveryFaultClass::timeout ? ProductionFaultKind::device_timeout :
      ProductionFaultKind::device_transport, detail);
    publish_status("GNSS transport recovery scheduled with bounded exponential backoff: " + detail);
  }

  void recovery_tick(const std::uint64_t now_ms)
  {
    if (!recovery_.take_attempt(now_ms)) return;
    const auto result = receiver_->recover(std::chrono::milliseconds(poll_timeout_ms_),
      stop_source_->token());
    if (result.code == ByteReadCode::data) {
      ++storage_segment_id_;
      auto [created, writer] = GnssLogWriter::create(segment_writer_options());
      if (!created.success) {
        recovery_.finish_attempt(false, now_ms);
        storage_fatal(ProductionFaultKind::storage_create, created.detail);
        return;
      }
      writer_ = std::move(writer);
      recovery_.finish_attempt(true, now_ms);
      consecutive_receive_timeouts_ = 0U;
      ++successful_reconnects_;
      publish_status("GNSS recovered exact configured COM path; decoder/time epoch reset; new segment " +
        std::to_string(storage_segment_id_));
      return;
    }
    recovery_.finish_attempt(false, now_ms);
    if (recovery_.state() == RecoveryState::exhausted) {
      gate_.start_failed();
      receiver_->stop();
      publish_fault(ProductionFaultKind::device_transport,
        "GNSS recovery exhausted after " + std::to_string(recovery_.attempts()) +
        " attempts: " + result.detail);
      publish_status("PERSISTENT GNSS FAULT: bounded recovery attempts exhausted");
    } else {
      publish_status("GNSS recovery attempt failed; next bounded retry scheduled: " + result.detail);
    }
  }

  void publish_observation(const ReceivedSentence & sentence, const std::uint64_t monotonic_ns,
    const std::uint64_t connection_epoch)
  {
    ppbng_interfaces::msg::GnssObservation message;
    message.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    message.device_id = device_id_;
    message.sample_sequence = samples_received_ + 1U;
    message.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
      sentence.host_receive_time.time_since_epoch()).count());
    message.host_receive_monotonic_ns = monotonic_ns;
    message.connection_epoch = connection_epoch;
    if (sentence.gga) {
      const auto & fix = *sentence.gga;
      message.kind = message.KIND_GGA;
      message.gga_valid = true;
      message.gga_utc_hhmmss = fix.utc_hhmmss;
      message.latitude_deg = fix.latitude_deg;
      message.longitude_deg = fix.longitude_deg;
      message.altitude_m = fix.altitude_m;
      message.fix_quality = fix.quality;
      message.satellites_used = fix.satellites_used;
      message.hdop = fix.hdop;
      message.differential_age_valid = fix.has_differential_age;
      message.differential_age_sec = fix.differential_age_sec;
    }
    if (sentence.heading) {
      const auto & heading = *sentence.heading;
      message.kind = message.KIND_UNIHEADINGA;
      message.time_reference = heading.receiver_time.time_reference;
      message.time_status = heading.receiver_time.time_status;
      message.receiver_week = heading.receiver_time.week;
      message.receiver_milliseconds_of_week = heading.receiver_time.milliseconds_of_week;
      message.receiver_leap_seconds = heading.receiver_time.leap_seconds;
      message.receiver_output_delay_ms = heading.receiver_time.output_delay_ms;
      if (heading.receiver_time.utc_unix_nanoseconds) {
        const auto utc_ns = *heading.receiver_time.utc_unix_nanoseconds;
        message.receiver_utc_valid = true;
        message.receiver_utc.sec = static_cast<std::int32_t>(utc_ns / 1'000'000'000LL);
        message.receiver_utc.nanosec = static_cast<std::uint32_t>(utc_ns % 1'000'000'000LL);
      }
      message.heading_valid = true;
      message.heading_solution_status = heading.solution_status;
      message.heading_position_type = heading.position_type;
      message.baseline_m = heading.baseline_m;
      message.heading_deg = heading.heading_deg;
      message.pitch_deg = heading.pitch_deg;
      message.heading_stddev_deg = heading.heading_stddev_deg;
      message.pitch_stddev_deg = heading.pitch_stddev_deg;
      message.heading_satellites_tracked = heading.satellites_tracked;
      message.heading_satellites_used = heading.satellites_used;
    }
    observation_publisher_->publish(message);
  }

  void publish_status(const std::string & detail)
  {
    ppbng_interfaces::msg::DeviceStatus status;
    status.status_time = now();
    status.status_host_monotonic_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    status.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    status.device_id = device_id_;
    status.required = true;
    status.lifecycle_state = recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting ? 6U :
      (receiver_ && receiver_->state() == ReceiverState::connected ? 5U : 0U);
    status.health = gate_.state() == ProductionGateState::fault ||
      recovery_.state() == RecoveryState::exhausted || recovery_.state() == RecoveryState::fatal ?
      3U : (recovery_.state() == RecoveryState::waiting ||
      recovery_.state() == RecoveryState::attempting ? 2U : (receiver_ ? 1U : 0U));
    status.segment_id = storage_segment_id_;
    status.reconnect_attempts = recovery_.attempts();
    status.last_sample_valid = samples_received_ != 0U;
    status.last_sample_sequence = samples_received_;
    status.samples_received = samples_received_;
    status.samples_incomplete = parse_error_count_;
    status.detail = detail;
    status_publisher_->publish(status);
  }

  void publish_fault(const ProductionFaultKind kind, const std::string & detail)
  {
    const auto policy = fault_policy(kind);
    const auto host_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
    ppbng_interfaces::msg::FaultEvent event;
    event.event_id = device_id_ + "-" + policy.code + "-" + std::to_string(++fault_sequence_);
    event.session_id = session_binding_.bound() ? session_binding_.session_id() : "";
    event.source_id = device_id_;
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
  }

  bool writer_configuration_valid() const
  {
    std::error_code ec;
    return session_binding_.bound() && !writer_options_.session_directory.empty() &&
      std::filesystem::is_directory(writer_options_.session_directory, ec) &&
      !writer_options_.stream_stem.empty() && writer_options_.flush_every_sentences > 0U;
  }

  ProductionActivationGate gate_;
  SerialReceiveConfiguration configuration_;
  std::string device_id_;
  int poll_timeout_ms_{5};
  std::unique_ptr<ReceiveStopSource> stop_source_;
  std::unique_ptr<Um982Receiver> receiver_;
  GnssLogWriterOptions writer_options_;
  SessionBinding session_binding_;
  std::unique_ptr<GnssLogWriter> writer_;
  BoundedRecovery recovery_;
  std::uint32_t recovery_timeout_threshold_{200U};
  std::uint32_t consecutive_receive_timeouts_{};
  std::string base_storage_stem_;
  std::uint32_t storage_segment_id_{0U};
  std::uint32_t successful_reconnects_{0U};
  std::uint64_t samples_received_{0U};
  std::uint64_t parse_error_count_{0U};
  std::uint64_t fault_count_{0U};
  std::uint64_t fault_sequence_{0U};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::GnssObservation>::SharedPtr observation_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::DeviceStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<ppbng_interfaces::srv::PrepareDevice>::SharedPtr prepare_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace ppbng_gnss

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppbng_gnss::Um982ProductionNode>());
  rclcpp::shutdown();
  return 0;
}
