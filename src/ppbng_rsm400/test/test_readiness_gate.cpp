#include "ppbng_rsm400/readiness_gate.hpp"

#include <gtest/gtest.h>

namespace
{
ppbng_rsm400::ReadinessConfiguration confirmed()
{
  return {true, true, 4, 0};
}

ppbng_rsm400::Telemetry complete(const int motion, const int error)
{
  ppbng_rsm400::Telemetry value;
  value.roll_deg = 0.2;
  value.pitch_deg = -0.3;
  value.general_status = ppbng_rsm400::GeneralMountStatus{2, 4, motion, 0, error, ""};
  return value;
}
}  // namespace

TEST(ReadinessGate, TimesOutWhenRequiredFieldsNeverArrive)
{
  ppbng_rsm400::ReadinessGate gate(confirmed());
  ppbng_rsm400::Telemetry partial;
  partial.roll_deg = 1.0;
  EXPECT_EQ(gate.observe(partial).state, ppbng_rsm400::ReadinessState::waiting);
  const auto result = gate.timeout();
  EXPECT_TRUE(result.failed());
  EXPECT_NE(result.detail.find("pitch"), std::string::npos);
}

TEST(ReadinessGate, FailsImmediatelyOnUnacceptableErrorLevel)
{
  ppbng_rsm400::ReadinessGate gate(confirmed());
  const auto result = gate.observe(complete(4, 1));
  EXPECT_TRUE(result.failed());
  EXPECT_NE(result.detail.find("error_level"), std::string::npos);
}

TEST(ReadinessGate, NonStabStatusCannotBecomeReadyAndFailsAtTimeout)
{
  ppbng_rsm400::ReadinessGate gate(confirmed());
  const auto pending = gate.observe(complete(3, 0));
  EXPECT_EQ(pending.state, ppbng_rsm400::ReadinessState::waiting);
  EXPECT_NE(pending.detail.find("does not equal"), std::string::npos);
  EXPECT_TRUE(gate.timeout().failed());
}

TEST(ReadinessGate, AggregatesValidatedFramesAndAcceptsExactConfirmedStab)
{
  ppbng_rsm400::ReadinessGate gate(confirmed());
  ppbng_rsm400::Telemetry attitude;
  attitude.roll_deg = 0.1;
  attitude.pitch_deg = -0.1;
  EXPECT_EQ(gate.observe(attitude).state, ppbng_rsm400::ReadinessState::waiting);
  ppbng_rsm400::Telemetry status;
  status.general_status = ppbng_rsm400::GeneralMountStatus{2, 4, 4, 0, 0, ""};
  EXPECT_TRUE(gate.observe(status).ready());
}

TEST(ReadinessGate, UnconfirmedConfigurationFailsClosed)
{
  ppbng_rsm400::ReadinessGate gate({true, false, 4, 0});
  EXPECT_TRUE(gate.configuration_result().failed());
  EXPECT_TRUE(gate.observe(complete(4, 0)).failed());
}
