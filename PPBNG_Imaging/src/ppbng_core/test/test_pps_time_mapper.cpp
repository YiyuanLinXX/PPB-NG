#include "ppbng_core/pps_time_mapper.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

using ppbng_core::PpsTimeMapper;
using ppbng_core::TimeStatus;

TEST(PpsTimeMapper, RejectsZeroAnchorCapacity)
{
  EXPECT_THROW(PpsTimeMapper mapper(0), std::invalid_argument);
}

TEST(PpsTimeMapper, MapsSubsecondHardwareTicks)
{
  PpsTimeMapper mapper;
  ASSERT_TRUE(mapper.add_anchor(42, 1'700'000'000'000'000'000LL));

  const auto mapped = mapper.map_event(42, 24'000'000, 48'000'000, true, 100);
  ASSERT_TRUE(mapped.has_value());
  EXPECT_EQ(mapped->utc_nanoseconds, 1'700'000'000'500'000'000LL);
  EXPECT_EQ(mapped->status, TimeStatus::locked);
  EXPECT_EQ(mapped->uncertainty_nanoseconds, 100U);
}

TEST(PpsTimeMapper, MapsNonBinaryFractionWithoutUtcPrecisionLoss)
{
  PpsTimeMapper mapper;
  constexpr auto utc = 1'700'000'000'000'000'001LL;
  ASSERT_TRUE(mapper.add_anchor(3, utc));

  const auto mapped = mapper.map_event(3, 1, 3, true);
  ASSERT_TRUE(mapped.has_value());
  EXPECT_EQ(mapped->utc_nanoseconds, utc + 333'333'333LL);
}

TEST(PpsTimeMapper, MarksControllerHoldover)
{
  PpsTimeMapper mapper;
  ASSERT_TRUE(mapper.add_anchor(7, 1'000'000'000LL));

  const auto mapped = mapper.map_event(7, 12, 120, false);
  ASSERT_TRUE(mapped.has_value());
  EXPECT_EQ(mapped->utc_nanoseconds, 1'100'000'000LL);
  EXPECT_EQ(mapped->status, TimeStatus::holdover);
}

TEST(PpsTimeMapper, ExtrapolatesBoundedHoldoverFromLatestAnchor)
{
  PpsTimeMapper mapper;
  ASSERT_TRUE(mapper.add_anchor(10, 1'700'000'000'000'000'000LL));

  const auto mapped = mapper.map_event(12, 12, 48, false, 100, 3, 50'000);
  ASSERT_TRUE(mapped.has_value());
  EXPECT_EQ(mapped->utc_nanoseconds, 1'700'000'002'250'000'000LL);
  EXPECT_EQ(mapped->uncertainty_nanoseconds, 112'600U);
  EXPECT_EQ(mapped->status, TimeStatus::holdover);
}

TEST(PpsTimeMapper, RejectsHoldoverBeyondConfiguredLimit)
{
  PpsTimeMapper mapper;
  ASSERT_TRUE(mapper.add_anchor(10, 0));

  EXPECT_FALSE(mapper.map_event(14, 0, 48'000'000, false, 0, 3, 50'000).has_value());
  EXPECT_FALSE(mapper.map_event(11, 0, 48'000'000, true, 0, 3, 50'000).has_value());
}

TEST(PpsTimeMapper, RejectsUnknownOrInvalidEvents)
{
  PpsTimeMapper mapper;
  ASSERT_TRUE(mapper.add_anchor(1, 0));

  EXPECT_FALSE(mapper.map_event(2, 0, 48'000'000, true).has_value());
  EXPECT_FALSE(mapper.map_event(1, 0, 0, true).has_value());
  EXPECT_FALSE(mapper.map_event(1, 48'000'000, 48'000'000, true).has_value());
}

TEST(PpsTimeMapper, RejectsConflictingDuplicateAnchor)
{
  PpsTimeMapper mapper;
  ASSERT_TRUE(mapper.add_anchor(5, 123));
  EXPECT_TRUE(mapper.add_anchor(5, 123));
  EXPECT_FALSE(mapper.add_anchor(5, 124));
}

TEST(PpsTimeMapper, EvictsOldestSequence)
{
  PpsTimeMapper mapper(2);
  ASSERT_TRUE(mapper.add_anchor(10, 10));
  ASSERT_TRUE(mapper.add_anchor(12, 12));
  ASSERT_TRUE(mapper.add_anchor(11, 11));

  EXPECT_EQ(mapper.anchor_count(), 2U);
  EXPECT_FALSE(mapper.map_event(10, 0, 1, true).has_value());
  EXPECT_TRUE(mapper.map_event(11, 0, 1, true).has_value());
  EXPECT_TRUE(mapper.map_event(12, 0, 1, true).has_value());
}
