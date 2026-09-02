#pragma once
#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace ppbng_hsi {
enum class RecoveryFaultClass {transport, timeout, storage, integrity, timing, cancelled};
enum class RecoveryState {idle, waiting, attempting, recovered, exhausted, fatal, stopped};
struct RecoveryConfiguration {
  std::uint32_t maximum_attempts{5U};
  std::uint64_t initial_backoff_ms{250U};
  std::uint64_t maximum_backoff_ms{4000U};
};
class BoundedRecovery {
 public:
  explicit BoundedRecovery(RecoveryConfiguration value = {}) : configuration_(value) {
    if (value.maximum_attempts == 0U || value.initial_backoff_ms == 0U ||
        value.maximum_backoff_ms < value.initial_backoff_ms)
      throw std::invalid_argument("invalid bounded recovery configuration");
  }
  bool on_fault(RecoveryFaultClass kind, std::uint64_t now_ms) noexcept {
    if (state_ == RecoveryState::stopped) return false;
    if (kind != RecoveryFaultClass::transport && kind != RecoveryFaultClass::timeout) {
      state_ = RecoveryState::fatal; return false;
    }
    if (state_ == RecoveryState::waiting || state_ == RecoveryState::attempting) return true;
    attempts_ = 0U; next_backoff_ms_ = configuration_.initial_backoff_ms;
    due_ms_ = saturating_add(now_ms, next_backoff_ms_);
    state_ = RecoveryState::waiting; return true;
  }
  bool take_attempt(std::uint64_t now_ms) noexcept {
    if (state_ != RecoveryState::waiting || now_ms < due_ms_) return false;
    state_ = RecoveryState::attempting; ++attempts_; return true;
  }
  void finish_attempt(bool success, std::uint64_t now_ms) noexcept {
    if (state_ != RecoveryState::attempting) return;
    if (success) {state_ = RecoveryState::recovered; return;}
    if (attempts_ >= configuration_.maximum_attempts) {
      state_ = RecoveryState::exhausted; return;
    }
    next_backoff_ms_ = std::min(configuration_.maximum_backoff_ms,
      next_backoff_ms_ > configuration_.maximum_backoff_ms / 2U ?
      configuration_.maximum_backoff_ms : next_backoff_ms_ * 2U);
    due_ms_ = saturating_add(now_ms, next_backoff_ms_);
    state_ = RecoveryState::waiting;
  }
  void reset() noexcept {state_ = RecoveryState::idle; attempts_ = 0U; due_ms_ = 0U;}
  void stop() noexcept {state_ = RecoveryState::stopped;}
  RecoveryState state() const noexcept {return state_;}
  std::uint32_t attempts() const noexcept {return attempts_;}
  std::uint64_t due_ms() const noexcept {return due_ms_;}
 private:
  static std::uint64_t saturating_add(std::uint64_t a, std::uint64_t b) noexcept {
    return a > UINT64_MAX - b ? UINT64_MAX : a + b;
  }
  RecoveryConfiguration configuration_;
  RecoveryState state_{RecoveryState::idle};
  std::uint32_t attempts_{};
  std::uint64_t due_ms_{};
  std::uint64_t next_backoff_ms_{};
};
}  // namespace ppbng_hsi
