#include "ppbng_core/time_authority_correlator.hpp"

#include <gtest/gtest.h>

namespace {
using namespace ppbng_core;

AuthorityGnssEvidence gnss(std::uint64_t host_ns, std::int64_t utc,
                           std::uint16_t delay_ms = 0, std::uint64_t epoch = 1) {
  return {host_ns, epoch, utc, delay_ms, true, true};
}
AuthorityPpsAnchor pps(std::uint64_t sequence, std::uint64_t host_ns,
                       std::uint8_t lock = 1U) {
  return {7U, sequence, sequence * 1'000'000U, host_ns, lock};
}
AuthorityTrigger trigger(std::uint64_t sequence, std::uint64_t arrival) {
  return {"fx10e", sequence, sequence, 250U, 1'000U, arrival, 2U};
}

TEST(TimeAuthorityCorrelator, UsesOutputDelayAndSystemOffsetButDefaultsToUnverifiedHoldover) {
  TimeAuthorityConfiguration config;
  config.expected_system_offset_ns = 5'000'000;
  TimeAuthorityCorrelator subject(config);
  EXPECT_TRUE(subject.add_trigger(trigger(10, 10'010'000'000ULL)).empty());
  EXPECT_TRUE(subject.add_gnss(gnss(10'105'000'000ULL, 1'700'000'000, 100)).empty());
  const auto output = subject.add_pps(pps(10, 10'000'000'000ULL));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].status, TimeStatus::holdover);
  EXPECT_EQ(output[0].utc_nanoseconds, 1'700'000'000'250'000'000LL);
  EXPECT_GE(output[0].uncertainty_nanoseconds, config.maximum_host_skew_ns);
  EXPECT_NE(output[0].detail.find("UNVERIFIED"), std::string::npos);
}

TEST(TimeAuthorityCorrelator, CallsLockedOnlyAfterExplicitEvidenceConfirmation) {
  TimeAuthorityConfiguration config;
  config.pps_gnss_evidence_confirmed = true;
  TimeAuthorityCorrelator subject(config);
  subject.add_gnss(gnss(20'000'000'000ULL, 1'700'000'001));
  subject.add_pps(pps(20, 20'000'000'000ULL));
  const auto output = subject.add_trigger(trigger(20, 20'001'000'000ULL));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].status, TimeStatus::locked);
  EXPECT_EQ(output[0].uncertainty_nanoseconds, config.confirmed_uncertainty_ns);
}

TEST(TimeAuthorityCorrelator, OptionalPpsPublishesTriggerUnsyncedWithoutWaiting) {
  TimeAuthorityConfiguration config;
  config.pps_required = false;
  TimeAuthorityCorrelator subject(config);
  auto value = trigger(1U, 1'000U);
  value.pps_sequence = 0U;
  value.controller_time_status = 0U;
  const auto output = subject.add_trigger(value);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().status, TimeStatus::unsynced);
  EXPECT_EQ(output.front().trigger.channel_sequence, 1U);
  EXPECT_EQ(output.front().trigger.offset_ticks, 250U);
  EXPECT_EQ(subject.pending_trigger_count(), 0U);
  EXPECT_NE(output.front().detail.find("PPS disabled"), std::string::npos);
}

TEST(TimeAuthorityCorrelator, HoldoverMayCarryMoreThanOneSecondOfControllerTicks) {
  TimeAuthorityConfiguration config;
  config.pps_gnss_evidence_confirmed = true;
  TimeAuthorityCorrelator subject(config);
  subject.add_gnss(gnss(20'000'000'000ULL, 1'700'000'001));
  subject.add_pps(pps(20, 20'000'000'000ULL));
  auto value = trigger(20, 22'001'000'000ULL);
  value.offset_ticks = 2'250U;
  value.controller_time_status = 1U;
  const auto output = subject.add_trigger(value);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().status, TimeStatus::holdover);
  EXPECT_EQ(output.front().utc_nanoseconds, 1'700'000'003'250'000'000LL);
}

TEST(TimeAuthorityCorrelator, RejectsAmbiguousEquidistantCandidates) {
  TimeAuthorityCorrelator subject;
  subject.add_gnss(gnss(9'900'000'000ULL, 99));
  subject.add_gnss(gnss(10'100'000'000ULL, 100));
  EXPECT_TRUE(subject.add_pps(pps(10, 10'000'000'000ULL)).empty());
  EXPECT_EQ(subject.statistics().ambiguous_candidates, 1U);
  EXPECT_EQ(subject.statistics().anchors_correlated, 0U);
}

TEST(TimeAuthorityCorrelator, MissedAnchorAndTriggerTimeoutAreReportedWithoutDroppingTrigger) {
  TimeAuthorityConfiguration config;
  config.maximum_host_skew_ns = 100U;
  config.trigger_wait_timeout_ns = 500U;
  config.unverified_uncertainty_ns = 100U;
  TimeAuthorityCorrelator subject(config);
  subject.add_pps(pps(1, 1'000U));
  subject.add_trigger(trigger(1, 1'100U));
  const auto output = subject.advance(1'700U);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].status, TimeStatus::unsynced);
  EXPECT_EQ(output[0].trigger.pps_sequence, 1U);
  EXPECT_EQ(output[0].uncertainty_nanoseconds,
            (std::numeric_limits<std::uint64_t>::max)());
  EXPECT_EQ(subject.statistics().trigger_timeouts, 1U);
  EXPECT_EQ(subject.statistics().anchors_missed, 1U);
}

TEST(TimeAuthorityCorrelator, RejectsReorderedPpsAndGnssUtc) {
  TimeAuthorityCorrelator subject;
  subject.add_pps(pps(2, 2'000U));
  subject.add_pps(pps(1, 3'000U));
  subject.add_gnss(gnss(4'000U, 101));
  subject.add_gnss(gnss(5'000U, 100));
  EXPECT_EQ(subject.statistics().reordered_inputs, 2U);
}

TEST(TimeAuthorityCorrelator, QueueOverflowPublishesOldestTriggerUnsyncedAndKeepsSequence) {
  TimeAuthorityConfiguration config;
  config.maximum_pending_triggers = 1U;
  TimeAuthorityCorrelator subject(config);
  EXPECT_TRUE(subject.add_trigger(trigger(1, 1'000U)).empty());
  const auto output = subject.add_trigger(trigger(2, 2'000U));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].trigger.pps_sequence, 1U);
  EXPECT_EQ(output[0].status, TimeStatus::unsynced);
  EXPECT_EQ(subject.pending_trigger_count(), 1U);
  EXPECT_EQ(subject.statistics().queue_overflows, 1U);
}

TEST(TimeAuthorityCorrelator, BoundedEvidenceQueuesCountOverflow) {
  TimeAuthorityConfiguration config;
  config.maximum_pps_queue = 1U;
  config.maximum_gnss_queue = 1U;
  config.maximum_host_skew_ns = 1U;
  TimeAuthorityCorrelator subject(config);
  subject.add_pps(pps(1, 1'000U));
  subject.add_pps(pps(2, 2'000U));
  subject.add_gnss(gnss(9'000U, 100));
  subject.add_gnss(gnss(10'000U, 101));
  EXPECT_EQ(subject.statistics().queue_overflows, 2U);
  EXPECT_EQ(subject.statistics().anchors_missed, 1U);
}

TEST(TimeAuthorityCorrelator, RejectsInvalidFractionAndNeverRoundsIt) {
  TimeAuthorityCorrelator subject;
  auto invalid = gnss(1'000U, 100);
  invalid.fine = false;
  subject.add_gnss(invalid);
  EXPECT_EQ(subject.statistics().invalid_inputs, 1U);
  EXPECT_EQ(subject.statistics().anchors_correlated, 0U);
}

TEST(TimeAuthorityCorrelator, ControllerRebootCannotReuseOldGnssEvidence) {
  TimeAuthorityCorrelator subject;
  subject.add_gnss(gnss(10'000U, 100));
  subject.add_pps(pps(1, 10'000U));
  auto rebooted = pps(1, 10'001U);
  rebooted.boot_id = 8U;
  subject.add_pps(rebooted);
  const auto output = subject.add_trigger(trigger(1, 10'002U));
  EXPECT_TRUE(output.empty());
  const auto timed_out = subject.advance(1'000'000'000ULL);
  ASSERT_EQ(timed_out.size(), 1U);
  EXPECT_EQ(timed_out.front().status, TimeStatus::unsynced);
}

TEST(TimeAuthorityCorrelator, GnssReconnectFlushesPendingOldEpochTrigger)
{
  TimeAuthorityCorrelator subject;
  subject.add_pps(pps(5, 50'000U));
  subject.add_trigger(trigger(5, 50'001U));
  subject.add_gnss(gnss(1'000'000'000U, 100, 0, 1));
  const auto output = subject.add_gnss(gnss(50'000U, 200, 0, 2));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().trigger.pps_sequence, 5U);
  EXPECT_EQ(output.front().status, TimeStatus::unsynced);
  EXPECT_NE(output.front().detail.find("epoch changed"), std::string::npos);
}
}  // namespace
