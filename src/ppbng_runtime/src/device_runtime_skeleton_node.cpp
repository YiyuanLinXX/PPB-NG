#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ppbng_runtime/hardware_safety_gate.hpp"

namespace ppbng_runtime
{

class DeviceRuntimeSkeletonNode final : public rclcpp::Node
{
public:
  DeviceRuntimeSkeletonNode()
  : Node("device_runtime_skeleton")
  {
    const auto role = declare_parameter<std::string>("device_role", "");
    HardwareSafetyInputs inputs;
    inputs.hardware_enabled = declare_parameter<bool>("hardware_enabled", false);
    inputs.launch_authorized = declare_parameter<bool>("launch_authorized", false);
    inputs.machine_config_path = std::filesystem::u8path(
      declare_parameter<std::string>("machine_config_path", ""));
    inputs.machine_id = declare_parameter<std::string>("machine_id", "");

    const std::set<std::string> supported_roles{
      "rgb", "thermal", "hsi", "gnss", "rsm400", "timing"};
    if (supported_roles.count(role) == 0U) {
      throw std::runtime_error(
              "device_role must be one of rgb, thermal, hsi, gnss, rsm400, or timing");
    }
    const auto decision = evaluate_hardware_safety(inputs);
    if (!decision.allowed) {
      throw std::runtime_error(role + " runtime refused: " + decision.reason);
    }

    // This executable deliberately has no vendor SDK dependency and performs no discovery or I/O.
    RCLCPP_WARN(
      get_logger(),
      "%s runtime safety gate passed, but its hardware backend is intentionally not linked; no device was opened",
      role.c_str());
  }
};

}  // namespace ppbng_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ppbng_runtime::DeviceRuntimeSkeletonNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ppbng_runtime_device"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
