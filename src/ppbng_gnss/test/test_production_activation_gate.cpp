#include "ppbng_gnss/production_activation_gate.hpp"

#include <gtest/gtest.h>

namespace
{
ppbng_gnss::SerialReceiveConfiguration valid_configuration()
{
  return {"COM12", 115200U, 4096U, 65536U};
}
}  // namespace

TEST(GnssProductionGate, IsInertAndFailClosedByDefault)
{
  ppbng_gnss::ProductionActivationGate gate(false);
  EXPECT_EQ(gate.state(), ppbng_gnss::ProductionGateState::inert);
  EXPECT_FALSE(gate.arm(valid_configuration()).accepted);
  EXPECT_FALSE(gate.start().accepted);
}

TEST(GnssProductionGate, RequiresArmBeforeStart)
{
  ppbng_gnss::ProductionActivationGate gate(true);
  EXPECT_FALSE(gate.start().accepted);
  EXPECT_TRUE(gate.arm(valid_configuration()).accepted);
  EXPECT_EQ(gate.state(), ppbng_gnss::ProductionGateState::armed);
  EXPECT_TRUE(gate.start().accepted);
  EXPECT_EQ(gate.state(), ppbng_gnss::ProductionGateState::started);
}

TEST(GnssProductionGate, ValidatesExplicitBoundedReceiveOnlyConfiguration)
{
  auto configuration = valid_configuration();
  EXPECT_TRUE(ppbng_gnss::validate_receive_only_configuration(configuration).accepted);
  configuration.com_path = R"(\\.\COM27)";
  EXPECT_TRUE(ppbng_gnss::validate_receive_only_configuration(configuration).accepted);
  configuration.com_path = "ttyUSB0";
  EXPECT_FALSE(ppbng_gnss::validate_receive_only_configuration(configuration).accepted);
  configuration = valid_configuration();
  configuration.read_chunk_bytes = configuration.max_line_bytes + 1U;
  EXPECT_FALSE(ppbng_gnss::validate_receive_only_configuration(configuration).accepted);
}

TEST(GnssProductionGate, FailedOpenLatchesUntilExplicitStop)
{
  ppbng_gnss::ProductionActivationGate gate(true);
  ASSERT_TRUE(gate.arm(valid_configuration()).accepted);
  ASSERT_TRUE(gate.start().accepted);
  gate.start_failed();
  EXPECT_EQ(gate.state(), ppbng_gnss::ProductionGateState::fault);
  gate.stop();
  EXPECT_EQ(gate.state(), ppbng_gnss::ProductionGateState::inert);
}
