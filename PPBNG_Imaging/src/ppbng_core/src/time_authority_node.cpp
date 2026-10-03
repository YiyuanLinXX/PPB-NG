#include "ppbng_core/time_authority_correlator.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <ppbng_interfaces/msg/gnss_observation.hpp>
#include <ppbng_interfaces/msg/pps_anchor.hpp>
#include <ppbng_interfaces/msg/trigger_event.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ppbng_core {
namespace {
std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::size_t positive_size(std::int64_t value, const char* name) {
  if (value <= 0) throw std::invalid_argument(std::string(name) + " must be positive");
  return static_cast<std::size_t>(value);
}

std::uint64_t nonnegative_u64(std::int64_t value, const char* name) {
  if (value < 0) throw std::invalid_argument(std::string(name) + " must be nonnegative");
  return static_cast<std::uint64_t>(value);
}
}  // namespace

class TimeAuthorityNode final : public rclcpp::Node {
 public:
  TimeAuthorityNode() : Node("time_authority"), correlator_(configuration()) {
    const auto pps_topic = declare_parameter<std::string>(
      "pps_topic", "/timing/pps_anchor");
    const auto raw_trigger_topic = declare_parameter<std::string>(
      "raw_trigger_topic", "/timing/trigger_event");
    const auto gnss_topic = declare_parameter<std::string>(
      "gnss_topic", "/gnss/observation");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/acquisition/trigger");
    output_ = create_publisher<ppbng_interfaces::msg::TriggerEvent>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1024)).reliable());
    pps_ = create_subscription<ppbng_interfaces::msg::PpsAnchor>(
      pps_topic, rclcpp::QoS(rclcpp::KeepLast(64)).reliable(),
      [this](const ppbng_interfaces::msg::PpsAnchor& value) {
        publish(correlator_.add_pps({value.boot_id, value.pps_sequence, value.captured_tick,
          value.host_receive_monotonic_ns, value.lock}));
      });
    trigger_ = create_subscription<ppbng_interfaces::msg::TriggerEvent>(
      raw_trigger_topic, rclcpp::QoS(rclcpp::KeepLast(1024)).reliable(),
      [this](const ppbng_interfaces::msg::TriggerEvent& value) {
        publish(correlator_.add_trigger({value.channel, value.channel_sequence,
          value.pps_sequence, value.offset_ticks, value.ticks_per_second, monotonic_ns(),
          value.time_quality.status}));
      });
    gnss_ = create_subscription<ppbng_interfaces::msg::GnssObservation>(
      gnss_topic, rclcpp::QoS(rclcpp::KeepLast(64)).reliable(),
      [this](const ppbng_interfaces::msg::GnssObservation& value) {
        // Only an exact integer-second epoch from a validated UNIHEADINGA/FINE header
        // may label a PPS. GGA text and fractional epochs are never rounded into evidence.
        if (value.kind != value.KIND_UNIHEADINGA || value.time_status != "FINE" ||
            !value.receiver_utc_valid || value.receiver_utc.nanosec != 0U) return;
        publish(correlator_.add_gnss({value.host_receive_monotonic_ns,
          value.connection_epoch, value.receiver_utc.sec,
          value.receiver_output_delay_ms, true, true}));
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
      [this] { publish(correlator_.advance(monotonic_ns())); });
  }

 private:
  TimeAuthorityConfiguration configuration() {
    TimeAuthorityConfiguration value;
    value.maximum_pps_queue = positive_size(
      declare_parameter<std::int64_t>("maximum_pps_queue", 16), "maximum_pps_queue");
    value.maximum_gnss_queue = positive_size(
      declare_parameter<std::int64_t>("maximum_gnss_queue", 16), "maximum_gnss_queue");
    value.maximum_pending_triggers = positive_size(
      declare_parameter<std::int64_t>("maximum_pending_triggers", 1024),
      "maximum_pending_triggers");
    value.maximum_host_skew_ns = nonnegative_u64(
      declare_parameter<std::int64_t>("maximum_host_skew_ns", 250'000'000),
      "maximum_host_skew_ns");
    value.expected_system_offset_ns = declare_parameter<std::int64_t>(
      "expected_system_offset_ns", 0);
    value.trigger_wait_timeout_ns = nonnegative_u64(
      declare_parameter<std::int64_t>("trigger_wait_timeout_ns", 500'000'000),
      "trigger_wait_timeout_ns");
    value.unverified_uncertainty_ns = nonnegative_u64(
      declare_parameter<std::int64_t>("unverified_uncertainty_ns", 250'000'000),
      "unverified_uncertainty_ns");
    value.confirmed_uncertainty_ns = nonnegative_u64(
      declare_parameter<std::int64_t>("confirmed_uncertainty_ns", 1'000'000),
      "confirmed_uncertainty_ns");
    value.pps_gnss_evidence_confirmed = declare_parameter<bool>(
      "pps_gnss_evidence_confirmed", false);
    value.pps_required = declare_parameter<bool>("pps_required", true);
    return value;
  }

  void publish(const std::vector<CorrelatedTrigger>& values) {
    for (const auto& value : values) {
      ppbng_interfaces::msg::TriggerEvent message;
      message.header.stamp = now();
      message.channel = value.trigger.channel;
      message.channel_sequence = value.trigger.channel_sequence;
      message.pps_sequence = value.trigger.pps_sequence;
      message.offset_ticks = value.trigger.offset_ticks;
      message.ticks_per_second = value.trigger.ticks_per_second;
      message.time_quality.pps_sequence = value.trigger.pps_sequence;
      message.time_quality.hardware_tick = value.trigger.offset_ticks;
      message.time_quality.uncertainty_ns = value.uncertainty_nanoseconds;
      message.time_quality.status = static_cast<std::uint8_t>(value.status);
      message.time_quality.detail = value.detail;
      if (value.status != TimeStatus::unsynced) {
        message.time_quality.utc_time.sec = static_cast<std::int32_t>(
          value.utc_nanoseconds / 1'000'000'000LL);
        message.time_quality.utc_time.nanosec = static_cast<std::uint32_t>(
          value.utc_nanoseconds % 1'000'000'000LL);
      }
      output_->publish(message);
    }
  }

  TimeAuthorityCorrelator correlator_;
  rclcpp::Publisher<ppbng_interfaces::msg::TriggerEvent>::SharedPtr output_;
  rclcpp::Subscription<ppbng_interfaces::msg::PpsAnchor>::SharedPtr pps_;
  rclcpp::Subscription<ppbng_interfaces::msg::TriggerEvent>::SharedPtr trigger_;
  rclcpp::Subscription<ppbng_interfaces::msg::GnssObservation>::SharedPtr gnss_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ppbng_core

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppbng_core::TimeAuthorityNode>());
  rclcpp::shutdown();
  return 0;
}
