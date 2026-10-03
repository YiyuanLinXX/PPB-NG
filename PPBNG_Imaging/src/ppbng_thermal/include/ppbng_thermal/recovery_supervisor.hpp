#pragma once

#include <chrono>
#include <cstddef>
#include <string>

namespace ppbng_thermal {

enum class StartupTimeoutAction { wait, count_runtime_timeout, missing_trigger };

// Worker-thread-only startup gate. No sleeps and no hardware access.
// A trigger arriving during GetNextImage must not count that partial wait.
class StartupTriggerWait {
public:
  using Clock = std::chrono::steady_clock;
  explicit StartupTriggerWait(std::chrono::milliseconds limit,
    Clock::time_point start = Clock::now()) : deadline_(start + limit) {}
  StartupTimeoutAction timeout(bool trigger_seen, Clock::time_point now = Clock::now()) {
    if (!waiting_) return StartupTimeoutAction::count_runtime_timeout;
    if (trigger_seen) {
      waiting_ = false;
      return StartupTimeoutAction::wait;
    }
    if (now >= deadline_) {
      waiting_ = false;
      return StartupTimeoutAction::missing_trigger;
    }
    return StartupTimeoutAction::wait;
  }
  void observe_frame() noexcept { waiting_ = false; }
private:
  Clock::time_point deadline_;
  bool waiting_{true};
};

struct RecoveryPolicy {
  std::size_t consecutive_timeout_threshold{5};
  std::size_t maximum_attempts{20};
  std::chrono::milliseconds initial_backoff{250};
  std::chrono::milliseconds maximum_backoff{5000};
};
struct RecoveryAttempt { bool allowed{false}; bool latched{false}; std::size_t number{0}; std::chrono::milliseconds backoff{0}; std::string detail; };
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
  std::size_t consecutive_timeouts_{0}, attempts_{0};
};
}  // namespace ppbng_thermal
