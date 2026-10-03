#include "ppbng_hsi/bounded_recovery.hpp"
#include <gtest/gtest.h>

TEST(HsiBoundedRecovery, RetriesOnlyTransportAndTimeoutWithExponentialBounds) {
  using namespace ppbng_hsi;
  BoundedRecovery recovery({3U, 10U, 25U});
  ASSERT_TRUE(recovery.on_fault(RecoveryFaultClass::transport, 100U));
  EXPECT_FALSE(recovery.take_attempt(109U));
  ASSERT_TRUE(recovery.take_attempt(110U));
  recovery.finish_attempt(false, 110U);
  EXPECT_EQ(recovery.due_ms(), 130U);
  ASSERT_TRUE(recovery.take_attempt(130U));
  recovery.finish_attempt(false, 130U);
  EXPECT_EQ(recovery.due_ms(), 155U);
  ASSERT_TRUE(recovery.take_attempt(155U));
  recovery.finish_attempt(false, 155U);
  EXPECT_EQ(recovery.state(), RecoveryState::exhausted);
}

TEST(HsiBoundedRecovery, StorageIntegrityAndTimingAreFatalWithoutAttempt) {
  using namespace ppbng_hsi;
  for (const auto fault : {RecoveryFaultClass::storage, RecoveryFaultClass::integrity,
                           RecoveryFaultClass::timing}) {
    BoundedRecovery recovery;
    EXPECT_FALSE(recovery.on_fault(fault, 0U));
    EXPECT_EQ(recovery.state(), RecoveryState::fatal);
    EXPECT_FALSE(recovery.take_attempt(UINT64_MAX));
  }
}

TEST(HsiBoundedRecovery, StopInterruptsBackoffAndSuccessfulAttemptCanReset) {
  using namespace ppbng_hsi;
  BoundedRecovery recovery({2U, 1000U, 2000U});
  recovery.on_fault(RecoveryFaultClass::timeout, 0U);
  recovery.stop();
  EXPECT_FALSE(recovery.take_attempt(5000U));
  EXPECT_EQ(recovery.state(), RecoveryState::stopped);
  recovery.reset();
  recovery.on_fault(RecoveryFaultClass::transport, 0U);
  ASSERT_TRUE(recovery.take_attempt(1000U));
  recovery.finish_attempt(true, 1000U);
  EXPECT_EQ(recovery.state(), RecoveryState::recovered);
}
