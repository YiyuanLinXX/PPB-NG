#include "ppbng_runtime/hardware_safety_gate.hpp"

#include <cctype>
#include <fstream>

namespace ppbng_runtime
{
namespace
{
bool is_blank(const std::string & value)
{
  for (const unsigned char character : value) {
    if (!std::isspace(character)) {
      return false;
    }
  }
  return true;
}
}  // namespace

HardwareSafetyDecision evaluate_hardware_safety(const HardwareSafetyInputs & inputs)
{
  if (!inputs.hardware_enabled) {
    return {false, "hardware_enabled is false (safe default)"};
  }
  if (!inputs.launch_authorized) {
    return {false, "hardware mode requires explicit authorization from the production launch"};
  }
  if (inputs.machine_config_path.empty()) {
    return {false, "machine_config_path is empty"};
  }
  if (inputs.machine_id.empty() || is_blank(inputs.machine_id)) {
    return {false, "machine_id is empty"};
  }

  std::error_code error;
  if (!std::filesystem::is_regular_file(inputs.machine_config_path, error) || error) {
    return {false, "machine configuration is not a readable regular file"};
  }
  const auto size = std::filesystem::file_size(inputs.machine_config_path, error);
  if (error || size == 0U) {
    return {false, "machine configuration file is empty or unreadable"};
  }

  std::ifstream stream(inputs.machine_config_path, std::ios::binary);
  if (!stream.good()) {
    return {false, "machine configuration file cannot be opened for reading"};
  }
  return {true, "hardware safety prerequisites satisfied; device backends remain disabled"};
}

}  // namespace ppbng_runtime
