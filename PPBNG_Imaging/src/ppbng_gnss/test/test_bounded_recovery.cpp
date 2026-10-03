#include "ppbng_gnss/bounded_recovery.hpp"
#include <gtest/gtest.h>

TEST(GnssBoundedRecovery, RetriesOnlyTransportAndTimeoutWithExponentialBounds) {
  using namespace ppbng_gnss;
  BoundedRecovery recovery({3U, 10U, 25U});
  ASSERT_TRUE(recovery.on_fault(RecoveryFaultClass::timeout, 100U));
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

TEST(GnssBoundedRecovery, NonTransportFaultsNeverRetryAndStopCancelsWait) {
  using namespace ppbng_gnss;
  BoundedRecovery fatal;
  EXPECT_FALSE(fatal.on_fault(RecoveryFaultClass::storage, 0U));
  EXPECT_EQ(fatal.state(), RecoveryState::fatal);
  BoundedRecovery stopped;
  stopped.on_fault(RecoveryFaultClass::transport, 0U);
  stopped.stop();
  EXPECT_FALSE(stopped.take_attempt(UINT64_MAX));
  EXPECT_EQ(stopped.state(), RecoveryState::stopped);
}
