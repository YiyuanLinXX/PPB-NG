#include "ppbng_rgb/recovery_supervisor.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ppbng_rgb {

RecoverySupervisor::RecoverySupervisor(RecoveryPolicy policy) : policy_(policy) {
  if (policy_.consecutive_timeout_threshold == 0 || policy_.maximum_attempts == 0 ||
      policy_.initial_backoff.count() <= 0 || policy_.maximum_backoff < policy_.initial_backoff) {
    throw std::invalid_argument("invalid recovery policy");
  }
}

bool RecoverySupervisor::observe_timeout() {
  if (consecutive_timeouts_ != std::numeric_limits<std::size_t>::max()) ++consecutive_timeouts_;
  return consecutive_timeouts_ >= policy_.consecutive_timeout_threshold;
}

void RecoverySupervisor::observe_frame() { consecutive_timeouts_ = 0; }

void RecoverySupervisor::begin_episode() {
  consecutive_timeouts_ = 0;
  attempts_ = 0;
}

RecoveryAttempt RecoverySupervisor::next_attempt() {
  if (attempts_ >= policy_.maximum_attempts) {
    return {false, true, attempts_, std::chrono::milliseconds{0}, "recovery attempts exhausted"};
  }
  ++attempts_;
  auto delay = policy_.initial_backoff;
  for (std::size_t i = 1; i < attempts_ && delay < policy_.maximum_backoff; ++i) {
    delay = std::min(policy_.maximum_backoff, delay * 2);
  }
  return {true, false, attempts_, delay, "recovery attempt permitted"};
}

void RecoverySupervisor::recovered() {
  attempts_ = 0;
  consecutive_timeouts_ = 0;
}

std::size_t RecoverySupervisor::consecutive_timeouts() const noexcept {
  return consecutive_timeouts_;
}

}  // namespace ppbng_rgb
