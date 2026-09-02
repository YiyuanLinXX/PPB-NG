#include "ppbng_timing/controller_model.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ppbng_timing {

SequenceObservation EventSequenceTracker::observe(
    std::uint32_t boot_id, std::uint64_t event_sequence) {
  if (!boot_id_.has_value()) {
    boot_id_ = boot_id;
    event_sequence_ = event_sequence;
    return SequenceObservation::kFirst;
  }
  if (*boot_id_ != boot_id) {
    boot_id_ = boot_id;
    event_sequence_ = event_sequence;
    return SequenceObservation::kControllerRestart;
  }
  if (event_sequence == *event_sequence_) {
    return SequenceObservation::kDuplicate;
  }
  if (event_sequence < *event_sequence_) {
    event_sequence_ = event_sequence;
    return SequenceObservation::kRegression;
  }
  const auto observation =
      event_sequence == *event_sequence_ + 1U
          ? SequenceObservation::kInOrder
          : SequenceObservation::kGap;
  event_sequence_ = event_sequence;
  return observation;
}

void EventSequenceTracker::reset() noexcept {
  boot_id_.reset();
  event_sequence_.reset();
}

TimeLockTracker::TimeLockTracker(
    std::uint32_t ticks_per_second, std::uint32_t maximum_holdover_seconds)
    : ticks_per_second_(ticks_per_second),
      maximum_holdover_seconds_(maximum_holdover_seconds) {
  if (ticks_per_second_ == 0U || maximum_holdover_seconds_ == 0U) {
    throw std::invalid_argument("time-lock tracker rates must be positive");
  }
}

void TimeLockTracker::observe_pps(
    std::uint64_t pps_sequence, std::uint64_t captured_tick) {
  if (last_pps_sequence_.has_value() && pps_sequence <= *last_pps_sequence_) {
    throw std::invalid_argument("PPS sequence must increase");
  }
  if (last_pps_tick_.has_value() && captured_tick <= *last_pps_tick_) {
    throw std::invalid_argument("PPS captured tick must increase");
  }
  last_pps_sequence_ = pps_sequence;
  last_pps_tick_ = captured_tick;
}

TimeLock TimeLockTracker::quality_at(std::uint64_t current_tick) const {
  if (!last_pps_tick_.has_value() || current_tick < *last_pps_tick_) {
    return TimeLock::kUnsynced;
  }
  const auto elapsed = current_tick - *last_pps_tick_;
  if (elapsed <= ticks_per_second_) {
    return TimeLock::kLocked;
  }
  const auto maximum_holdover_ticks =
      static_cast<std::uint64_t>(ticks_per_second_) * maximum_holdover_seconds_;
  return elapsed <= maximum_holdover_ticks ? TimeLock::kHoldover : TimeLock::kUnsynced;
}

std::uint64_t TimeLockTracker::last_pps_sequence() const noexcept {
  return last_pps_sequence_.value_or(0U);
}
std::uint64_t TimeLockTracker::last_pps_tick() const noexcept {
  return last_pps_tick_.value_or(0U);
}

ControllerModel::ControllerModel(
    std::uint32_t boot_id, std::uint32_t ticks_per_second,
    std::uint32_t maximum_holdover_seconds)
    : boot_id_(boot_id),
      ticks_per_second_(ticks_per_second),
      time_lock_(ticks_per_second, maximum_holdover_seconds) {
  if (boot_id_ == 0U || ticks_per_second_ == 0U) {
    throw std::invalid_argument("controller boot ID and tick rate must be positive");
  }
}

void ControllerModel::configure(const ScheduleConfig& config) {
  if (state_ != ControllerState::kIdle && state_ != ControllerState::kConfigured) {
    throw std::logic_error("timing configuration is frozen for the active task");
  }
  validate_schedule(config);
  if (config.ticks_per_second != ticks_per_second_) {
    throw std::invalid_argument("schedule tick rate does not match controller tick rate");
  }
  schedule_ = config;
  state_ = ControllerState::kConfigured;
}

void ControllerModel::freeze(std::uint32_t schedule_id) {
  if (state_ != ControllerState::kConfigured || !schedule_.has_value() ||
      schedule_->schedule_id != schedule_id) {
    throw std::logic_error("only the configured schedule can be frozen");
  }
  state_ = ControllerState::kFrozen;
}

void ControllerModel::arm_next_whole_second(const ArmRequest& request) {
  if (state_ != ControllerState::kFrozen || !schedule_.has_value() ||
      request.schedule_id != schedule_->schedule_id) {
    throw std::logic_error("controller must have the requested frozen schedule before arm");
  }
  arm_request_ = request;
  state_ = ControllerState::kWaitingForPps;
}

void ControllerModel::disarm() {
  state_ = ControllerState::kIdle;
  schedule_.reset();
  arm_request_.reset();
}

void ControllerModel::observe_pps(
    std::uint64_t pps_sequence, std::int64_t utc_second, std::uint64_t tick) {
  (void)utc_second;
  time_lock_.observe_pps(pps_sequence, tick);
  current_tick_ = tick;
  missed_pps_count_ = 0U;
  if (state_ == ControllerState::kWaitingForPps && arm_request_.has_value() &&
      pps_sequence > arm_request_->after_pps_sequence) {
    state_ = ControllerState::kRunning;
  }
}

void ControllerModel::advance_to(std::uint64_t tick) {
  if (tick < current_tick_) {
    throw std::invalid_argument("controller tick cannot move backwards");
  }
  current_tick_ = tick;
  if (time_lock_.last_pps_tick() > 0U && current_tick_ > time_lock_.last_pps_tick()) {
    const auto elapsed = current_tick_ - time_lock_.last_pps_tick();
    missed_pps_count_ = elapsed > ticks_per_second_
                            ? static_cast<std::uint32_t>(
                                  std::min<std::uint64_t>(
                                      elapsed / ticks_per_second_ - 1U,
                                      std::numeric_limits<std::uint32_t>::max()))
                            : 0U;
  }
}

TriggerEvent ControllerModel::emit_trigger(
    Channel channel, std::uint64_t channel_sequence) {
  if (state_ != ControllerState::kRunning || !schedule_.has_value()) {
    throw std::logic_error("triggers can only be emitted while running");
  }
  const auto configured = std::find_if(
      schedule_->channels.begin(), schedule_->channels.end(),
      [channel](const ChannelSchedule& item) {
        return item.channel == channel && item.enabled;
      });
  if (configured == schedule_->channels.end()) {
    throw std::invalid_argument("requested trigger channel is not enabled");
  }
  const auto lock = time_lock_.quality_at(current_tick_);
  const auto offset = current_tick_ >= time_lock_.last_pps_tick()
                          ? current_tick_ - time_lock_.last_pps_tick()
                          : 0U;
  return TriggerEvent{
      boot_id_, next_event_sequence_++, channel, channel_sequence,
      time_lock_.last_pps_sequence(), offset, ticks_per_second_, lock};
}

std::array<TriggerEvent, 2> ControllerModel::emit_snapshot_pair(
    std::uint64_t snapshot_sequence) {
  const auto rgb = emit_trigger(Channel::kRgb, snapshot_sequence);
  const auto thermal = emit_trigger(Channel::kThermal, snapshot_sequence);
  if (rgb.pps_sequence != thermal.pps_sequence ||
      rgb.offset_ticks != thermal.offset_ticks || rgb.lock != thermal.lock) {
    throw std::logic_error("shared snapshot event pair diverged");
  }
  return {rgb, thermal};
}

StatusReport ControllerModel::status() const {
  return StatusReport{
      boot_id_, state_, schedule_.has_value() ? schedule_->schedule_id : 0U,
      0U, time_lock_.last_pps_sequence(), current_tick_,
      time_lock_.quality_at(current_tick_), missed_pps_count_, 0U};
}

}  // namespace ppbng_timing
