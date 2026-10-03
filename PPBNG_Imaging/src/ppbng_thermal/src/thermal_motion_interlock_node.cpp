#include "ppbng_thermal/thermal_motion_permission.hpp"
#include "ppbng_thermal/acquisition_motion_permission.hpp"
#include <ppbng_interfaces/msg/acquisition_status.hpp>
#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/msg/sample_stamp.hpp>
#include <ppbng_interfaces/msg/frame_metadata.hpp>

#include <ppbng_interfaces/msg/thermal_nuc_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace ppbng_thermal
{

class ThermalMotionInterlockNode final : public rclcpp::Node
{
public:
  ThermalMotionInterlockNode()
  : Node("thermal_motion_interlock")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/thermal/nuc_state");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/ppbng/safety/thermal_motion_permitted");
    const auto state_timeout_ms = declare_parameter<int>("state_timeout_ms", 1250);
    const auto publish_period_ms = declare_parameter<int>("publish_period_ms", 100);
    if (input_topic.empty() || output_topic.empty() || state_timeout_ms <= 0 ||
      publish_period_ms <= 0)
    {
      throw std::invalid_argument("thermal motion interlock parameters must be nonempty/positive");
    }

    timeout_ns_ = static_cast<std::uint64_t>(state_timeout_ms) * 1'000'000ULL;
    const bool require_health = declare_parameter<bool>("require_acquisition_health", false);
    const auto health_timeout = declare_parameter<int>("health_timeout_ms", 3000);
    const auto required = declare_parameter<std::vector<std::string>>("required_device_ids", {});
    if (health_timeout <= 0 || (require_health && required.empty()))
      throw std::invalid_argument("production interlock requires devices and a positive timeout");
    health_timeout_ns_ = static_cast<std::uint64_t>(health_timeout) * 1'000'000ULL;
    if (require_health) {
      health_ = std::make_unique<AcquisitionMotionPermission>(required);
      health_fault_publisher_ = create_publisher<ppbng_interfaces::msg::FaultEvent>("/acquisition/fault_event", 10);
      acquisition_subscription_ = create_subscription<ppbng_interfaces::msg::AcquisitionStatus>(
        "/acquisition/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const ppbng_interfaces::msg::AcquisitionStatus& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (m.session_id != health_->session()) permission_.observe(false, false, steady_now_ns());
          health_->manager(m.session_id, m.state == m.RECORDING, m.degraded || m.state == m.FAULT, steady_now_ns());
        });
      device_subscription_ = create_subscription<ppbng_interfaces::msg::DeviceStatus>(
        "/acquisition/device_status", 100, [this](const ppbng_interfaces::msg::DeviceStatus& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (m.health == m.HEALTH_DEGRADED || m.health == m.HEALTH_FAULT || m.lifecycle_state == m.LIFECYCLE_RECOVERING)
            health_->fault(m.session_id, m.device_id + ": " + m.detail);
          if ((m.device_id == "gnss" || m.device_id == "rsm400" || m.device_id == "timing") &&
              m.lifecycle_state == m.LIFECYCLE_STREAMING && m.samples_received > 0)
            health_->sample(m.session_id, m.device_id, m.samples_received, steady_now_ns());
        });
      fault_subscription_ = create_subscription<ppbng_interfaces::msg::FaultEvent>(
        "/acquisition/fault_event", 100, [this](const ppbng_interfaces::msg::FaultEvent& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (m.severity >= m.SEVERITY_WARNING)
            health_->fault(m.session_id, m.source_id + ": " + m.fault_code);
        });
      sample_subscription_ = create_subscription<ppbng_interfaces::msg::SampleStamp>(
        "/acquisition/sample_stamp", 100, [this](const ppbng_interfaces::msg::SampleStamp& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          health_->sample(m.session_id, m.device_id, m.sample_sequence, steady_now_ns());
        });
      const auto observe_frame = [this](const ppbng_interfaces::msg::FrameMetadata& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        health_->sample(m.stamp.session_id, m.stamp.device_id, m.stamp.sample_sequence, steady_now_ns());
        if (!m.complete) health_->fault(m.stamp.session_id, m.stamp.device_id + ": incomplete image");
      };
      rgb_subscription_ = create_subscription<ppbng_interfaces::msg::FrameMetadata>("/rgb/frame_metadata", 32, observe_frame);
      thermal_subscription_ = create_subscription<ppbng_interfaces::msg::FrameMetadata>("/thermal/frame_metadata", 32, observe_frame);
    }
    const auto safety_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<std_msgs::msg::Bool>(output_topic, safety_qos);
    subscription_ = create_subscription<ppbng_interfaces::msg::ThermalNucState>(
      input_topic, safety_qos,
      [this](const ppbng_interfaces::msg::ThermalNucState & message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (health_ && message.session_id != health_->session()) return;
        permission_.observe(message.valid, message.active, steady_now_ns());
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms), [this]() {publish_permission();});

    RCLCPP_WARN(
      get_logger(),
      "Motion permission starts FALSE and remains fail-safe until fresh, valid A6701 NUC state arrives");
  }

private:
  static std::uint64_t steady_now_ns() noexcept
  {
    return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void publish_permission()
  {
    bool permitted = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      permitted = permission_.permitted(steady_now_ns(), timeout_ns_);
      if (health_) {
        const bool healthy = health_->permitted(steady_now_ns(), health_timeout_ns_);
        permitted = permitted && healthy;
        if (!health_->reason().empty() && health_->reason() != last_fault_reason_) {
          last_fault_reason_ = health_->reason();
          RCLCPP_ERROR(get_logger(), "LATCHED ROBOT STOP: %s; restart acquisition after inspection", last_fault_reason_.c_str());
          ppbng_interfaces::msg::FaultEvent event;
          event.session_id = health_->session(); event.source_id = "motion_interlock";
          event.fault_code = "MOTION_INTERLOCK_LATCHED"; event.severity = event.SEVERITY_ERROR;
          event.latched = true; event.causes_global_stop = false; event.occurrence_count = 1;
          event.first_host_monotonic_ns = steady_now_ns(); event.last_host_monotonic_ns = event.first_host_monotonic_ns;
          event.event_id = event.session_id + "-motion-" + std::to_string(event.first_host_monotonic_ns);
          event.detail = last_fault_reason_; health_fault_publisher_->publish(event);
        }
        if (health_->reason().empty()) last_fault_reason_.clear();
      }
    }
    std_msgs::msg::Bool message;
    message.data = permitted;
    publisher_->publish(message);
    if (!last_published_.has_value() || *last_published_ != permitted) {
      if (permitted) {
        RCLCPP_INFO(get_logger(), "All configured motion gates permit movement");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Motion prohibited: NUC active/invalid/stale, acquisition not recording, or sensor health gate denied");
      }
      last_published_ = permitted;
    }
  }

  std::mutex mutex_;
  ThermalMotionPermission permission_;
  std::unique_ptr<AcquisitionMotionPermission> health_;
  rclcpp::Publisher<ppbng_interfaces::msg::FaultEvent>::SharedPtr health_fault_publisher_;
  std::uint64_t health_timeout_ns_{0};
  std::string last_fault_reason_;
  rclcpp::Subscription<ppbng_interfaces::msg::AcquisitionStatus>::SharedPtr acquisition_subscription_;
  rclcpp::Subscription<ppbng_interfaces::msg::DeviceStatus>::SharedPtr device_subscription_;
  rclcpp::Subscription<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_subscription_;
  rclcpp::Subscription<ppbng_interfaces::msg::SampleStamp>::SharedPtr sample_subscription_;
  rclcpp::Subscription<ppbng_interfaces::msg::FrameMetadata>::SharedPtr rgb_subscription_, thermal_subscription_;
  std::uint64_t timeout_ns_{0U};
  std::optional<bool> last_published_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Subscription<ppbng_interfaces::msg::ThermalNucState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ppbng_thermal

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ppbng_thermal::ThermalMotionInterlockNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("thermal_motion_interlock"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
