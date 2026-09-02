#pragma once

#include <chrono>
#include <cstddef>
#include <string>

namespace ppbng_rgb {

struct RecoveryPolicy {
  std::size_t consecutive_timeout_threshold{5};
  std::size_t maximum_attempts{20};
  std::chrono::milliseconds initial_backoff{250};
  std::chrono::milliseconds maximum_backoff{5000};
};

struct RecoveryAttempt {
  bool allowed{false};
  bool latched{false};
  std::size_t number{0};
  std::chrono::milliseconds backoff{0};
  std::string detail;
};

// Pure, hardware-free policy state machine. The node owns all I/O and waits.
class RecoverySupervisor {
public:
  explicit RecoverySupervisor(RecoveryPolicy policy);
  bool observe_timeout();
  void observe_frame();
  void begin_episode();
  RecoveryAttempt next_attempt();
  void recovered();
  std::size_t consecutive_timeouts() const noexcept;

private:
  RecoveryPolicy policy_;
  std::size_t consecutive_timeouts_{0};
  std::size_t attempts_{0};
};

}  // namespace ppbng_rgb
