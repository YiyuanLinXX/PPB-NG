#include "ppbng_core/gnss_association.hpp"

#include <cmath>
#include <gtest/gtest.h>

using ppbng_core::GnssObservation;
using ppbng_core::associate_gnss;

namespace
{
GnssObservation sample(
  const std::uint64_t sequence, const std::int64_t time, const double latitude,
  const double longitude, const double altitude, const double heading)
{
  GnssObservation value;
  value.sequence = sequence;
  value.utc_nanoseconds = time;
  value.latitude_degrees = latitude;
  value.longitude_degrees = longitude;
  value.altitude_meters = altitude;
  value.heading_degrees = heading;
  value.pitch_degrees = 0.0;
  value.fix_quality = 4U;
  value.position_valid = true;
  value.heading_valid = true;
  return value;
}
}

TEST(GnssAssociation, InterpolatesPositionAndCircularHeading)
{
  const auto before = sample(10, 1'000'000'000, 40.0, -75.0, 100.0, 359.0);
  const auto after = sample(11, 1'100'000'000, 40.0001, -74.9999, 102.0, 1.0);
  const auto result = associate_gnss(before, after, 1'050'000'000, 200'000'000, 100'000'000);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->interpolation_fraction, 0.5);
  EXPECT_NEAR(result->latitude_degrees, 40.00005, 1e-8);
  EXPECT_NEAR(result->longitude_degrees, -74.99995, 1e-8);
  EXPECT_NEAR(result->altitude_meters, 101.0, 0.01);
  EXPECT_NEAR(result->heading_degrees, 0.0, 1e-12);
}

TEST(GnssAssociation, HandlesLongitudeWrapInCartesianSpace)
{
  const auto before = sample(1, 0, 0.0, 179.999, 0.0, 90.0);
  const auto after = sample(2, 100'000'000, 0.0, -179.999, 0.0, 90.0);
  const auto result = associate_gnss(before, after, 50'000'000, 200'000'000, 100'000'000);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(std::abs(result->longitude_degrees), 180.0, 1e-9);
}

TEST(GnssAssociation, RejectsExtrapolationAndStaleBrackets)
{
  const auto before = sample(1, 100, 40.0, -75.0, 0.0, 0.0);
  const auto after = sample(2, 200, 40.0, -75.0, 0.0, 0.0);
  EXPECT_FALSE(associate_gnss(before, after, 99, 200, 200).has_value());
  EXPECT_FALSE(associate_gnss(before, after, 201, 200, 200).has_value());
  EXPECT_FALSE(associate_gnss(before, after, 150, 99, 200).has_value());
  EXPECT_FALSE(associate_gnss(before, after, 150, 200, 49).has_value());
}

TEST(GnssAssociation, PreservesPositionWhenHeadingIsUnavailable)
{
  auto before = sample(1, 100, 40.0, -75.0, 0.0, 10.0);
  auto after = sample(2, 200, 40.0, -75.0, 0.0, 20.0);
  after.heading_valid = false;
  const auto result = associate_gnss(before, after, 150, 200, 100);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->heading_valid);
}

TEST(GnssAssociation, RejectsInvalidPositionAndDegenerateTime)
{
  auto before = sample(1, 100, 40.0, -75.0, 0.0, 10.0);
  auto after = sample(2, 100, 40.0, -75.0, 0.0, 20.0);
  EXPECT_FALSE(associate_gnss(before, after, 100, 200, 100).has_value());
  after.utc_nanoseconds = 200;
  before.position_valid = false;
  EXPECT_FALSE(associate_gnss(before, after, 150, 200, 100).has_value());
}
