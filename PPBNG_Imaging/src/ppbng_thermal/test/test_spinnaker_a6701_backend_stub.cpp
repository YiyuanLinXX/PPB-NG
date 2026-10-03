#include "ppbng_thermal/spinnaker_a6701_backend.hpp"

#include <gtest/gtest.h>

TEST(SpinnakerA6701BackendStub, FailsClosedWithoutTouchingHardware)
{
  ppbng_thermal::SpinnakerA6701Backend backend({"serial", "A6701"});
  ppbng_thermal::StopSource stop;
  std::vector<std::string> ids{"must be cleared"};
  const auto status = backend.discover(std::chrono::milliseconds(1), stop.token(), ids);
  EXPECT_EQ(status.code, ppbng_thermal::ErrorCode::invalid_state);
  EXPECT_NE(status.detail.find("SDK_NOT_BUILT"), std::string::npos);
  EXPECT_TRUE(ids.empty());
  EXPECT_EQ(backend.state(), ppbng_thermal::LifecycleState::idle);
}
