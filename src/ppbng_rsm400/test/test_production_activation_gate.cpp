#include <gtest/gtest.h>

#include "ppbng_rsm400/production_activation_gate.hpp"

namespace ppbng_rsm400
{

TEST(RsmProductionGate, DefaultsToInertAndRefusesHardware)
{
  ProductionActivationGate gate(false);
  EXPECT_FALSE(gate.arm("COM9").accepted);
  EXPECT_FALSE(gate.start().accepted);
  EXPECT_EQ(gate.state(), ProductionGateState::inert);
}

TEST(RsmProductionGate, ArmDoesNotStartAndStartRequiresArm)
{
  ProductionActivationGate gate(true);
  EXPECT_FALSE(gate.start().accepted);
  EXPECT_FALSE(gate.arm("COM_REQUIRED").accepted);
  EXPECT_TRUE(gate.arm("COM9").accepted);
  EXPECT_EQ(gate.state(), ProductionGateState::armed);
  EXPECT_TRUE(gate.start().accepted);
  EXPECT_EQ(gate.state(), ProductionGateState::started);
  gate.stop();
  EXPECT_EQ(gate.state(), ProductionGateState::inert);
}

TEST(RsmProductionGate, FailedStartLatchesFaultUntilStop)
{
  ProductionActivationGate gate(true);
  ASSERT_TRUE(gate.arm("COM9").accepted);
  ASSERT_TRUE(gate.start().accepted);
  gate.start_failed();
  EXPECT_EQ(gate.state(), ProductionGateState::fault);
  EXPECT_FALSE(gate.start().accepted);
  gate.stop();
  EXPECT_EQ(gate.state(), ProductionGateState::inert);
}

}  // namespace ppbng_rsm400
