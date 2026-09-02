#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "ppbng_runtime/hardware_safety_gate.hpp"

namespace
{
class TemporaryConfig
{
public:
  explicit TemporaryConfig(const std::string & contents)
  {
    path_ = std::filesystem::temp_directory_path() /
      ("ppbng_gate_" + std::to_string(++counter_) + ".yaml");
    std::ofstream(path_, std::ios::binary) << contents;
  }

  ~TemporaryConfig()
  {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::filesystem::path & path() const {return path_;}

private:
  static inline unsigned int counter_{0U};
  std::filesystem::path path_;
};
}  // namespace

TEST(HardwareSafetyGate, DefaultsToRefused)
{
  const auto decision = ppbng_runtime::evaluate_hardware_safety({});
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reason.find("hardware_enabled"), std::string::npos);
}

TEST(HardwareSafetyGate, RequiresExplicitLaunchAuthorization)
{
  TemporaryConfig config("configured: true\n");
  ppbng_runtime::HardwareSafetyInputs inputs;
  inputs.hardware_enabled = true;
  inputs.machine_config_path = config.path();
  inputs.machine_id = "ppbng-industrial-pc";
  EXPECT_FALSE(ppbng_runtime::evaluate_hardware_safety(inputs).allowed);
}

TEST(HardwareSafetyGate, RequiresNonemptyMachineIdentity)
{
  TemporaryConfig config("configured: true\n");
  ppbng_runtime::HardwareSafetyInputs inputs;
  inputs.hardware_enabled = true;
  inputs.launch_authorized = true;
  inputs.machine_config_path = config.path();
  EXPECT_FALSE(ppbng_runtime::evaluate_hardware_safety(inputs).allowed);
}

TEST(HardwareSafetyGate, RejectsMissingAndEmptyConfiguration)
{
  ppbng_runtime::HardwareSafetyInputs inputs;
  inputs.hardware_enabled = true;
  inputs.launch_authorized = true;
  inputs.machine_id = "ppbng-industrial-pc";
  inputs.machine_config_path = "definitely_missing_ppbng_config.yaml";
  EXPECT_FALSE(ppbng_runtime::evaluate_hardware_safety(inputs).allowed);

  TemporaryConfig empty_config("");
  inputs.machine_config_path = empty_config.path();
  EXPECT_FALSE(ppbng_runtime::evaluate_hardware_safety(inputs).allowed);
}

TEST(HardwareSafetyGate, AllowsOnlyCompletePrerequisites)
{
  TemporaryConfig config("configured: true\nmachine_id: ppbng-industrial-pc\n");
  ppbng_runtime::HardwareSafetyInputs inputs;
  inputs.hardware_enabled = true;
  inputs.launch_authorized = true;
  inputs.machine_config_path = config.path();
  inputs.machine_id = "ppbng-industrial-pc";
  EXPECT_TRUE(ppbng_runtime::evaluate_hardware_safety(inputs).allowed);
}
