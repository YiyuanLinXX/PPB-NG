#pragma once

#include "ppbng_timing/protocol.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace ppbng_timing {

enum class SequenceObservation {
  kFirst,
  kInOrder,
  kGap,
  kDuplicate,
  kRegression,
  kControllerRestart,
};

class EventSequenceTracker {
 public:
  SequenceObservation observe(std::uint32_t boot_id, std::uint64_t event_sequence);
  void reset() noexcept;

 private:
  std::optional<std::uint32_t> boot_id_;
  std::optional<std::uint64_t> event_sequence_;
};

class TimeLockTracker {
 public:
  TimeLockTracker(std::uint32_t ticks_per_second, std::uint32_t maximum_holdover_seconds);
  void observe_pps(std::uint64_t pps_sequence, std::uint64_t captured_tick);
  TimeLock quality_at(std::uint64_t current_tick) const;
  std::uint64_t last_pps_sequence() const noexcept;
  std::uint64_t last_pps_tick() const noexcept;

 private:
  std::uint32_t ticks_per_second_;
  std::uint32_t maximum_holdover_seconds_;
  std::optional<std::uint64_t> last_pps_sequence_;
  std::optional<std::uint64_t> last_pps_tick_;
};

// Deterministic in-memory state model for host tests. It performs no I/O.
class ControllerModel {
 public:
  ControllerModel(
      std::uint32_t boot_id, std::uint32_t ticks_per_second,
      std::uint32_t maximum_holdover_seconds = 5U);

  void configure(const ScheduleConfig& config);
  void freeze(std::uint32_t schedule_id);
  void arm_next_whole_second(const ArmRequest& request);
  void disarm();
  void observe_pps(std::uint64_t pps_sequence, std::int64_t utc_second, std::uint64_t tick);
  void advance_to(std::uint64_t tick);
  TriggerEvent emit_trigger(Channel channel, std::uint64_t channel_sequence);
  std::array<TriggerEvent, 2> emit_snapshot_pair(std::uint64_t snapshot_sequence);
  StatusReport status() const;

 private:
  std::uint32_t boot_id_;
  std::uint32_t ticks_per_second_;
  ControllerState state_{ControllerState::kIdle};
  std::optional<ScheduleConfig> schedule_;
  std::optional<ArmRequest> arm_request_;
  TimeLockTracker time_lock_;
  std::uint64_t current_tick_{};
  std::uint64_t next_event_sequence_{};
  std::uint32_t missed_pps_count_{};
};

}  // namespace ppbng_timing
