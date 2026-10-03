#include "ppbng_thermal/thermal_motion_permission.hpp"

#include <gtest/gtest.h>

using ppbng_thermal::ThermalMotionPermission;

TEST(ThermalMotionPermission, FailsClosedBeforeFirstObservation)
{
  ThermalMotionPermission state;
  EXPECT_FALSE(state.permitted(1'000U, 500U));
}

TEST(ThermalMotionPermission, RequiresFreshValidIdleState)
{
  ThermalMotionPermission state;
  state.observe(true, false, 1'000U);
  EXPECT_TRUE(state.permitted(1'500U, 500U));
  EXPECT_FALSE(state.permitted(1'501U, 500U));

  state.observe(true, true, 2'000U);
  EXPECT_FALSE(state.permitted(2'001U, 500U));

  state.observe(false, false, 3'000U);
  EXPECT_FALSE(state.permitted(3'001U, 500U));
}

TEST(ThermalMotionPermission, RejectsClockRegression)
{
  ThermalMotionPermission state;
  state.observe(true, false, 2'000U);
  EXPECT_FALSE(state.permitted(1'999U, 500U));
}
