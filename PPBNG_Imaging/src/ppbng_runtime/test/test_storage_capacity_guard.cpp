#include "ppbng_runtime/storage_capacity_guard.hpp"

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace ppbng_runtime
{

TEST(StorageCapacityGuard, RequiresWholeRemainingTaskPlusHeadroomAndReserve)
{
  const StorageCapacityPolicy policy{100U, 7200U, 1000U, 1500U};
  const auto decision = evaluate_storage_capacity(policy, 829000U, 0U);
  ASSERT_TRUE(decision.valid);
  EXPECT_TRUE(decision.sufficient);
  EXPECT_EQ(decision.remaining_stream_bytes, 828000U);
  EXPECT_EQ(decision.required_available_bytes, 829000U);
  EXPECT_FALSE(evaluate_storage_capacity(policy, 828999U, 0U).sufficient);
}

TEST(StorageCapacityGuard, RemainingRequirementFallsWithElapsedTimeButKeepsReserve)
{
  const StorageCapacityPolicy policy{100U, 10U, 500U, 0U};
  EXPECT_EQ(evaluate_storage_capacity(policy, 1000U, 5U).required_available_bytes, 1000U);
  const auto finished = evaluate_storage_capacity(policy, 500U, 12U);
  EXPECT_TRUE(finished.valid);
  EXPECT_TRUE(finished.sufficient);
  EXPECT_EQ(finished.remaining_stream_bytes, 0U);
  EXPECT_EQ(finished.required_available_bytes, 500U);
}

TEST(StorageCapacityGuard, RejectsZeroInputsExcessiveHeadroomAndArithmeticOverflow)
{
  EXPECT_FALSE(evaluate_storage_capacity({}, 0U, 0U).valid);
  EXPECT_FALSE(evaluate_storage_capacity({1U, 1U, 0U, 10001U}, 0U, 0U).valid);
  const StorageCapacityPolicy multiply_overflow{
    (std::numeric_limits<std::uint64_t>::max)(), 2U, 0U, 0U};
  EXPECT_FALSE(evaluate_storage_capacity(multiply_overflow, 0U, 0U).valid);
  const StorageCapacityPolicy reserve_overflow{
    1U, 1U, (std::numeric_limits<std::uint64_t>::max)(), 0U};
  EXPECT_FALSE(evaluate_storage_capacity(reserve_overflow, 0U, 0U).valid);
}

TEST(StorageCapacityGuard, RoundsFractionalHeadroomUp)
{
  const StorageCapacityPolicy policy{1U, 1U, 0U, 1U};
  const auto decision = evaluate_storage_capacity(policy, 2U, 0U);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.remaining_stream_bytes, 2U);
  EXPECT_TRUE(decision.sufficient);
}

TEST(StorageCapacityGuard, DefaultTwoHourProfileRequiresDocumentedBudget)
{
  // 120 Hz FX10e + 120 Hz SWIR + 2 Hz RGB + 2 Hz full 640x513 thermal transport.
  const StorageCapacityPolicy policy{
    162'531'840U, 7'200U, 107'374'182'400U, 2'500U};
  const auto decision = evaluate_storage_capacity(policy, 1'570'160'742'400U, 0U);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.remaining_stream_bytes, 1'462'786'560'000U);
  EXPECT_EQ(decision.required_available_bytes, 1'570'160'742'400U);
  EXPECT_TRUE(decision.sufficient);
  EXPECT_FALSE(evaluate_storage_capacity(
      policy, 1'570'160'742'399U, 0U).sufficient);
}

}  // namespace ppbng_runtime
