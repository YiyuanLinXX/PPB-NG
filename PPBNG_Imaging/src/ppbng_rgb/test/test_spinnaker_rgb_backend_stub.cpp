#include "ppbng_rgb/spinnaker_rgb_backend.hpp"

#include <gtest/gtest.h>

TEST(SpinnakerRgbBackendStub, FailsClosedWithoutTouchingHardware)
{
  ppbng_rgb::SpinnakerRgbBackend backend;
  ppbng_rgb::StopSource stop;
  std::vector<std::string> ids{"must be cleared"};
  const auto status = backend.discover(std::chrono::milliseconds(1), stop.token(), ids);
  EXPECT_EQ(status.code, ppbng_rgb::ErrorCode::invalid_state);
  EXPECT_NE(status.detail.find("SDK_NOT_BUILT"), std::string::npos);
  EXPECT_TRUE(ids.empty());
  EXPECT_EQ(backend.state(), ppbng_rgb::LifecycleState::idle);
}
