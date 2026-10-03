#include "ppbng_core/rsm_attitude_association.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace {

ppbng_core::RsmAttitudeSample sample(
    std::uint64_t id, std::int64_t time, double roll, double pitch,
    double yaw, bool yaw_valid = true, bool status_valid = true) {
  return ppbng_core::RsmAttitudeSample{
      id, time, roll, pitch, yaw, status_valid, true, true, yaw_valid,
      static_cast<std::uint32_t>(100U + id)};
}

TEST(RsmAttitudeAssociation, RequiresUsableConfiguration) {
  EXPECT_THROW(ppbng_core::RsmAttitudeAssociator(0U), std::invalid_argument);
  EXPECT_THROW(ppbng_core::RsmAttitudeAssociator(1U, 1U), std::invalid_argument);
}

TEST(RsmAttitudeAssociation, StrictlyBracketsAndRetainsProvenance) {
  ppbng_core::RsmAttitudeAssociator associator(1'000U);
  ASSERT_TRUE(associator.add_sample(sample(10U, 1'000, -2.0, 4.0, 100.0)));
  ASSERT_TRUE(associator.add_sample(sample(11U, 2'000, 2.0, 8.0, 120.0)));

  const auto result = associator.associate(1'250);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->roll_deg, -1.0);
  EXPECT_DOUBLE_EQ(result->pitch_deg, 5.0);
  ASSERT_TRUE(result->yaw_deg.has_value());
  EXPECT_DOUBLE_EQ(*result->yaw_deg, 105.0);
  EXPECT_FALSE(result->yaw_is_platform_heading);
  EXPECT_EQ(result->before_sample_id, 10U);
  EXPECT_EQ(result->after_sample_id, 11U);
  EXPECT_EQ(result->before_age_nanoseconds, 250U);
  EXPECT_EQ(result->after_age_nanoseconds, 750U);
  EXPECT_EQ(result->before_raw_status, 110U);
  EXPECT_EQ(result->after_raw_status, 111U);
}

TEST(RsmAttitudeAssociation, InterpolatesYawAcrossZeroByShortestArc) {
  ppbng_core::RsmAttitudeAssociator associator(1'000U);
  ASSERT_TRUE(associator.add_sample(sample(1U, 1'000, 0.0, 0.0, 359.0)));
  ASSERT_TRUE(associator.add_sample(sample(2U, 2'000, 0.0, 0.0, 1.0)));

  const auto midpoint = associator.associate(1'500);
  ASSERT_TRUE(midpoint.has_value());
  ASSERT_TRUE(midpoint->yaw_deg.has_value());
  EXPECT_NEAR(*midpoint->yaw_deg, 0.0, 1e-12);
  EXPECT_FALSE(midpoint->yaw_is_platform_heading);
}

TEST(RsmAttitudeAssociation, MissingYawDoesNotDiscardValidRollPitch) {
  ppbng_core::RsmAttitudeAssociator associator(1'000U);
  ASSERT_TRUE(associator.add_sample(sample(1U, 1'000, 0.0, 2.0, 0.0, false)));
  ASSERT_TRUE(associator.add_sample(sample(2U, 2'000, 2.0, 4.0, 20.0, true)));

  const auto result = associator.associate(1'500);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->roll_deg, 1.0);
  EXPECT_DOUBLE_EQ(result->pitch_deg, 3.0);
  EXPECT_FALSE(result->yaw_deg.has_value());
  EXPECT_FALSE(result->yaw_is_platform_heading);
}

TEST(RsmAttitudeAssociation, RefusesExtrapolationAndStaleBrackets) {
  ppbng_core::RsmAttitudeAssociator associator(100U);
  ASSERT_TRUE(associator.add_sample(sample(1U, 1'000, 0.0, 0.0, 0.0)));
  ASSERT_TRUE(associator.add_sample(sample(2U, 2'000, 1.0, 1.0, 1.0)));

  EXPECT_FALSE(associator.associate(999).has_value());
  EXPECT_FALSE(associator.associate(2'001).has_value());
  EXPECT_FALSE(associator.associate(1'500).has_value());
}

TEST(RsmAttitudeAssociation, RejectsOutOfOrderDuplicateTimeAndSequence) {
  ppbng_core::RsmAttitudeAssociator associator(1'000U);
  ASSERT_TRUE(associator.add_sample(sample(5U, 1'000, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(associator.add_sample(sample(6U, 1'000, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(associator.add_sample(sample(6U, 999, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(associator.add_sample(sample(5U, 2'000, 0.0, 0.0, 0.0)));
  EXPECT_EQ(associator.sample_count(), 1U);
  EXPECT_TRUE(associator.add_sample(sample(6U, 2'000, 0.0, 0.0, 0.0)));
}

TEST(RsmAttitudeAssociation, DoesNotBridgeInvalidStatusSample) {
  ppbng_core::RsmAttitudeAssociator associator(2'000U);
  ASSERT_TRUE(associator.add_sample(sample(1U, 1'000, 0.0, 0.0, 0.0)));
  ASSERT_TRUE(associator.add_sample(sample(2U, 2'000, 1.0, 1.0, 1.0, true, false)));
  ASSERT_TRUE(associator.add_sample(sample(3U, 3'000, 2.0, 2.0, 2.0)));
  EXPECT_FALSE(associator.associate(1'500).has_value());
  EXPECT_FALSE(associator.associate(2'500).has_value());
}

TEST(RsmAttitudeAssociation, RejectsNonFiniteValidAngles) {
  ppbng_core::RsmAttitudeAssociator associator(1'000U);
  auto invalid = sample(1U, 1'000, 0.0, 0.0, 0.0);
  invalid.roll_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(associator.add_sample(invalid));

  auto missing_yaw = sample(1U, 1'000, 0.0, 0.0, 0.0, false);
  missing_yaw.yaw_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(associator.add_sample(missing_yaw));
}

}  // namespace
