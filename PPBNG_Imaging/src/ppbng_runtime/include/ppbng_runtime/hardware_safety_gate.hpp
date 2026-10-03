#pragma once

#include <filesystem>
#include <string>

namespace ppbng_runtime
{

struct HardwareSafetyInputs
{
  bool hardware_enabled{false};
  bool launch_authorized{false};
  std::filesystem::path machine_config_path;
  std::string machine_id;
};

struct HardwareSafetyDecision
{
  bool allowed{false};
  std::string reason;
};

HardwareSafetyDecision evaluate_hardware_safety(const HardwareSafetyInputs & inputs);

}  // namespace ppbng_runtime
