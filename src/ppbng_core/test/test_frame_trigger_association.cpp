#include "ppbng_core/frame_trigger_association.hpp"

#include <gtest/gtest.h>

namespace
{
ppbng_core::TriggerObservation trigger(const std::uint64_t sequence)
{
  ppbng_core::TriggerObservation value;
  value.event_id = 1000U + sequence;
  value.channel_sequence = sequence;
  value.pps_sequence = 9;
  value.utc_time_ns = static_cast<std::int64_t>(sequence * 500'000'000U);
  value.time_quality = ppbng_core::FrameTimeQuality::locked;
  return value;
}

ppbng_core::CameraFrameObservation frame(const std::uint64_t id)
{
  ppbng_core::CameraFrameObservation value;
  value.frame_id = id;
  value.complete = true;
  return value;
}
}  // namespace

TEST(FrameTriggerAssociation, RejectsObservationsBeforeSegment)
{
  ppbng_core::FrameTriggerAssociator associator;
  EXPECT_FALSE(associator.observe_trigger(trigger(1)).accepted);
  EXPECT_FALSE(associator.observe_frame(frame(20)).accepted);
}

TEST(FrameTriggerAssociation, MatchesBySequenceNotArrivalTime)
{
  ppbng_core::FrameTriggerAssociator associator;
  associator.begin_segment(3, 50);
  EXPECT_TRUE(associator.observe_frame(frame(700)).matches.empty());
  const auto first = associator.observe_trigger(trigger(50));
  ASSERT_EQ(first.matches.size(), 1U);
  EXPECT_EQ(first.matches.front().frame.frame_id, 700U);
  EXPECT_EQ(first.matches.front().trigger.channel_sequence, 50U);

  EXPECT_TRUE(associator.observe_trigger(trigger(51)).matches.empty());
  const auto second = associator.observe_frame(frame(701));
  ASSERT_EQ(second.matches.size(), 1U);
  EXPECT_EQ(second.matches.front().trigger.channel_sequence, 51U);
}

TEST(FrameTriggerAssociation, DoesNotShiftAfterMissingFrame)
{
  ppbng_core::FrameTriggerAssociator associator;
  associator.begin_segment(1, 10);
  associator.observe_trigger(trigger(10));
  ASSERT_EQ(associator.observe_frame(frame(100)).matches.size(), 1U);
  associator.observe_trigger(trigger(11));
  associator.observe_trigger(trigger(12));
  const auto update = associator.observe_frame(frame(102));
  ASSERT_EQ(update.matches.size(), 1U);
  EXPECT_EQ(update.matches.front().trigger.channel_sequence, 12U);
  EXPECT_EQ(update.matches.front().frame_gap_before, 1U);
  EXPECT_EQ(associator.summary().pending_trigger_count, 1U);
}

TEST(FrameTriggerAssociation, HoldsFrameUntilDelayedTriggerArrives)
{
  ppbng_core::FrameTriggerAssociator associator;
  associator.begin_segment(8, 200);
  associator.observe_frame(frame(9000));
  associator.observe_frame(frame(9001));
  const auto later = associator.observe_trigger(trigger(201));
  ASSERT_EQ(later.matches.size(), 1U);
  EXPECT_EQ(later.matches.front().frame.frame_id, 9001U);
  EXPECT_EQ(associator.summary().pending_frame_count, 1U);
}

TEST(FrameTriggerAssociation, RejectsCounterRegressionAndRequiresNewSegment)
{
  ppbng_core::FrameTriggerAssociator associator;
  associator.begin_segment(4, 1);
  EXPECT_TRUE(associator.observe_frame(frame(100)).accepted);
  EXPECT_FALSE(associator.observe_frame(frame(99)).accepted);
  associator.begin_segment(5, 80);
  EXPECT_TRUE(associator.observe_frame(frame(1)).accepted);
}

TEST(FrameTriggerAssociation, EnforcesBoundedPendingQueues)
{
  ppbng_core::FrameTriggerAssociator associator(2);
  associator.begin_segment(1, 10);
  EXPECT_TRUE(associator.observe_trigger(trigger(10)).accepted);
  EXPECT_TRUE(associator.observe_trigger(trigger(11)).accepted);
  EXPECT_FALSE(associator.observe_trigger(trigger(12)).accepted);
}
