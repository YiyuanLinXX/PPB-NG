#include "ppbng_hsi/sample_stamp_mapper.hpp"

#include <gtest/gtest.h>

namespace
{
ppbng_hsi::LineRecord line(const ppbng_hsi::CaptureKind capture,
  const ppbng_hsi::AssociationStatus association)
{
  ppbng_hsi::LineRecord value;
  value.device_id = "fx10e";
  value.camera_kind = ppbng_hsi::CameraKind::fx10e;
  value.trigger.channel = "fx10e";
  value.trigger.channel_sequence = 100U;
  value.trigger.pps_sequence = 7U;
  value.trigger.offset_ticks = 1234U;
  value.trigger.ticks_per_second = 1'000'000U;
  value.index.capture_kind = capture;
  value.index.association_status = association;
  value.index.segment_id = 4U;
  value.index.segment_line_index = 22U;
  value.index.camera_line_sequence = 900U;
  value.index.trigger_sequence = 100U;
  value.index.pps_sequence = 7U;
  value.index.utc_time_ns = 1'700'000'000'123'456'789LL;
  value.index.time_status = ppbng_hsi::TimeStatus::locked;
  value.index.uncertainty_ns = 88U;
  value.index.host_receive_monotonic_ns = 555'000U;
  return value;
}
}  // namespace

TEST(HsiSampleStampMapper, PublishesPersistedSceneIdentityAndCallbackArrival)
{
  const auto stamp = ppbng_hsi::make_sample_stamp(
    line(ppbng_hsi::CaptureKind::sample, ppbng_hsi::AssociationStatus::matched), "session-a");
  ASSERT_TRUE(stamp);
  EXPECT_EQ(stamp->session_id, "session-a");
  EXPECT_EQ(stamp->device_id, "fx10e");
  EXPECT_EQ(stamp->segment_id, 4U);
  EXPECT_EQ(stamp->sample_sequence, 22U);
  EXPECT_EQ(stamp->trigger_sequence, 100U);
  EXPECT_EQ(stamp->pps_sequence, 7U);
  EXPECT_EQ(stamp->controller_tick, 1234U);
  EXPECT_EQ(stamp->controller_ticks_per_second, 1'000'000U);
  EXPECT_EQ(stamp->camera_frame_id, 900U);
  EXPECT_TRUE(stamp->camera_frame_id_valid);
  EXPECT_FALSE(stamp->camera_timestamp_valid);
  EXPECT_EQ(stamp->host_receive_monotonic_ns, 555'000U);
  EXPECT_EQ(stamp->time_quality.status, stamp->time_quality.LOCKED);
  EXPECT_EQ(stamp->time_quality.utc_time.sec, 1'700'000'000);
  EXPECT_EQ(stamp->time_quality.utc_time.nanosec, 123'456'789U);
  EXPECT_NE(stamp->time_quality.detail.find("association=MATCHED"), std::string::npos);
}

TEST(HsiSampleStampMapper, DarkIsNeverPublishedAsSceneSample)
{
  EXPECT_FALSE(ppbng_hsi::make_sample_stamp(
    line(ppbng_hsi::CaptureKind::dark, ppbng_hsi::AssociationStatus::matched), "session-a"));
}

TEST(HsiSampleStampMapper, UnverifiedAssociationPreservesRawEvidenceButCannotClaimLock)
{
  const auto stamp = ppbng_hsi::make_sample_stamp(line(ppbng_hsi::CaptureKind::sample,
    ppbng_hsi::AssociationStatus::consistent_unverified), "session-a");
  ASSERT_TRUE(stamp);
  EXPECT_EQ(stamp->time_quality.status, stamp->time_quality.UNSYNCED);
  EXPECT_EQ(stamp->time_quality.utc_time.sec, 1'700'000'000);
  EXPECT_EQ(stamp->time_quality.utc_time.nanosec, 123'456'789U);
  EXPECT_EQ(stamp->time_quality.uncertainty_ns, 88U);
  EXPECT_NE(stamp->time_quality.detail.find("raw_trigger_time_status=LOCKED"), std::string::npos);
  EXPECT_NE(stamp->time_quality.detail.find("canonical_status_forced_UNSYNCED"),
    std::string::npos);
}

TEST(HsiSampleStampMapper, InternalContinuousLineUsesCallbackTimeWithoutInventingUtc)
{
  auto value = line(ppbng_hsi::CaptureKind::sample,
      ppbng_hsi::AssociationStatus::unverified);
  value.trigger = {};
  value.trigger.channel = "internal:fx10e";
  value.index.trigger_sequence = 0U;
  value.index.pps_sequence = 0U;
  value.index.utc_time_ns = 0;
  value.index.time_status = ppbng_hsi::TimeStatus::unsynced;
  value.index.uncertainty_ns = 0U;

  const auto stamp = ppbng_hsi::make_sample_stamp(value, "session-a");
  ASSERT_TRUE(stamp);
  EXPECT_EQ(stamp->trigger_channel, "internal:fx10e");
  EXPECT_EQ(stamp->trigger_sequence, 0U);
  EXPECT_EQ(stamp->pps_sequence, 0U);
  EXPECT_EQ(stamp->controller_ticks_per_second, 0U);
  EXPECT_EQ(stamp->time_quality.status, stamp->time_quality.UNSYNCED);
  EXPECT_EQ(stamp->time_quality.utc_time.sec, 0);
  EXPECT_EQ(stamp->time_quality.utc_time.nanosec, 0U);
  EXPECT_EQ(stamp->host_receive_monotonic_ns, 555'000U);
  EXPECT_NE(stamp->time_quality.detail.find("raw_trigger_utc=unavailable_or_out_of_range"),
    std::string::npos);
}
