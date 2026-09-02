#include "ppbng_timing_firmware/firmware_core.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ppbng_timing_firmware {
namespace {
using namespace ppbng_timing;

class CriticalGuard {
 public:
  explicit CriticalGuard(ITimingHardware& hardware) noexcept
      : hardware_(hardware), token_(hardware_.enter_timing_critical()) {}
  ~CriticalGuard() { hardware_.exit_timing_critical(token_); }
 private:
  ITimingHardware& hardware_;
  ITimingHardware::CriticalToken token_;
};

const ChannelSchedule* find_channel(const ScheduleConfig& schedule, Channel channel) {
  const auto found = std::find_if(schedule.channels.begin(), schedule.channels.end(),
      [channel](const ChannelSchedule& candidate) { return candidate.channel == channel; });
  return found == schedule.channels.end() ? nullptr : &*found;
}

}  // namespace

FirmwareCore::FirmwareCore(ITimingHardware& hardware, std::uint32_t boot_id,
                           std::uint32_t hardware_ticks_per_second,
                           SafetyPolicy policy) noexcept
    : hardware_(hardware), policy_(policy),
      hardware_ticks_per_second_(hardware_ticks_per_second) {
  reset(boot_id);
}

void FirmwareCore::reset(std::uint32_t new_boot_id) noexcept {
  CriticalGuard critical(hardware_);
  hardware_.set_output_gate(false);
  for (auto output : {PhysicalOutput::fx10e, PhysicalOutput::swir,
                      PhysicalOutput::snapshot})
    hardware_.cancel_output_compare(output);
  hardware_.configure_pps_input_capture(CaptureEdge::rising);
  boot_id_ = new_boot_id;
  state_ = ControllerState::kIdle;
  error_flags_ = new_boot_id == 0U || hardware_ticks_per_second_ == 0U
                     ? kErrorProtocol : kErrorNone;
  if (error_flags_ != 0U) state_ = ControllerState::kFault;
  output_gate_enabled_ = false;
  schedule_.reset();
  oscillators_ = {};
  pps_sequence_ = 0U;
  last_pps_tick_ = 0U;
  have_pps_ = false;
  global_event_sequence_ = 0U;
  last_host_activity_tick_ = 0U;
  last_observed_tick_ = 0U;
  arm_request_ = {};
  synchronous_start_armed_ = false;
  pps_watchdog_required_ = false;
  events_.clear();
  cached_request_.reset();
  cached_response_.reset();
}

void FirmwareCore::disable_outputs_isr() noexcept {
  hardware_.set_output_gate(false);
  output_gate_enabled_ = false;
  synchronous_start_armed_ = false;
  for (auto output : {PhysicalOutput::fx10e, PhysicalOutput::swir,
                      PhysicalOutput::snapshot})
    hardware_.cancel_output_compare(output);
}

void FirmwareCore::force_fault_isr(std::uint32_t flag) noexcept {
  error_flags_ |= flag;
  state_ = ControllerState::kFault;
  disable_outputs_isr();
}

bool FirmwareCore::same_as_cached(const Packet& request) const noexcept {
  return cached_request_ && cached_request_->protocol_version == request.protocol_version &&
         cached_request_->type == request.type && cached_request_->sequence == request.sequence &&
         cached_request_->payload == request.payload;
}

void FirmwareCore::cache(const Packet& request, const Packet& response) {
  cached_request_ = request;
  cached_response_ = response;
}

Packet FirmwareCore::make_ack(const Packet& request, ControllerState state,
                              const char* detail) const {
  return {kProtocolVersion, MessageType::kAck, request.sequence,
          encode(Ack{request.sequence, 0U, state, detail})};
}

Packet FirmwareCore::make_error(const Packet& request, std::uint16_t code,
                                const char* detail) const {
  return {kProtocolVersion, MessageType::kError, request.sequence,
          encode(ErrorReport{request.sequence, code, 3U, detail})};
}

void FirmwareCore::configure_oscillators(const ScheduleConfig& schedule) noexcept {
  const auto configure = [&schedule](Oscillator& oscillator, PhysicalOutput output,
                                     Channel channel) {
    const auto* source = find_channel(schedule, channel);
    oscillator = {};
    oscillator.output = output;
    oscillator.logical_channel = channel;
    if (!source) return;
    oscillator.enabled = source->enabled;
    oscillator.numerator = source->rate_numerator_hz;
    oscillator.denominator = source->rate_denominator;
    oscillator.width = source->pulse_width_ticks;
    oscillator.phase = source->phase_ticks;
    if (source->enabled) {
      const std::uint64_t scaled =
          static_cast<std::uint64_t>(schedule.ticks_per_second) * source->rate_denominator;
      oscillator.base_interval = scaled / source->rate_numerator_hz;
      oscillator.interval_remainder = scaled % source->rate_numerator_hz;
    }
  };
  configure(oscillators_[0], PhysicalOutput::fx10e, Channel::kFx10e);
  configure(oscillators_[1], PhysicalOutput::swir, Channel::kSwir);
  configure(oscillators_[2], PhysicalOutput::snapshot, Channel::kRgb);
}

Packet FirmwareCore::handle_command(const Packet& request, std::uint64_t current_tick) {
  if (request.protocol_version != kProtocolVersion) {
    { CriticalGuard critical(hardware_); force_fault_isr(kErrorProtocol); }
    return make_error(request, 1U, "unsupported protocol version");
  }
  bool tick_regression = false;
  {
    CriticalGuard critical(hardware_);
    if (current_tick < last_observed_tick_) {
      force_fault_isr(kErrorCounterOrTimeRegression);
      tick_regression = true;
    } else {
      last_observed_tick_ = current_tick;
      last_host_activity_tick_ = current_tick;
    }
  }
  if (tick_regression) return make_error(request, 2U, "host tick regression");

  // Command cache is owned by the single USB task. It is never touched by an ISR.
  if (cached_request_ && request.sequence == cached_request_->sequence) {
    if (same_as_cached(request)) return *cached_response_;
    { CriticalGuard critical(hardware_); force_fault_isr(kErrorProtocol); }
    return make_error(request, 3U, "sequence reused with different command");
  }
  if (cached_request_ && request.sequence < cached_request_->sequence) {
    { CriticalGuard critical(hardware_); force_fault_isr(kErrorProtocol); }
    return make_error(request, 4U, "command sequence regression");
  }

  Packet response;
  try {
    switch (request.type) {
      case MessageType::kGetVersion:
        if (!request.payload.empty()) throw std::invalid_argument("GetVersion payload must be empty");
        {
          std::uint32_t boot_snapshot;
          { CriticalGuard critical(hardware_); boot_snapshot = boot_id_; }
          response = {kProtocolVersion, MessageType::kVersionReport, request.sequence,
                      encode(VersionInfo{1U, 0U, 1U, boot_snapshot,
                                         kCapabilityStartAtCurrentTick,
                                         hardware_ticks_per_second_})};
        }
        break;
      case MessageType::kConfigureSchedule: {
        auto candidate = decode_schedule_config(request.payload);
        validate_schedule(candidate);
        if (candidate.ticks_per_second != hardware_ticks_per_second_)
          throw std::invalid_argument("schedule tick rate differs from hardware timer");
        {
          CriticalGuard critical(hardware_);
          if (state_ != ControllerState::kIdle && state_ != ControllerState::kConfigured)
            throw std::invalid_argument("configuration is not mutable in this state");
          schedule_ = std::move(candidate);
          configure_oscillators(*schedule_);
          state_ = ControllerState::kConfigured;
        }
        response = make_ack(request, ControllerState::kConfigured, "configured");
        break;
      }
      case MessageType::kFreezeConfiguration: {
        {
          CriticalGuard critical(hardware_);
          if (!request.payload.empty() || state_ != ControllerState::kConfigured || !schedule_)
            throw std::invalid_argument("Freeze requires configured state and empty payload");
          state_ = ControllerState::kFrozen;
        }
        response = make_ack(request, ControllerState::kFrozen, "frozen");
        break;
      }
      case MessageType::kArmNextWholeSecond: {
        const auto requested_arm = decode_arm_request(request.payload);
        if (policy_.maximum_host_silence_ticks == 0U ||
            policy_.maximum_pps_silence_ticks == 0U)
          throw std::invalid_argument("fail-closed watchdog policy is not configured");
        {
          CriticalGuard critical(hardware_);
          if (state_ != ControllerState::kFrozen || !schedule_)
            throw std::invalid_argument("Arm requires frozen configuration");
          arm_request_ = requested_arm;
          pps_watchdog_required_ = true;
          if (arm_request_.schedule_id != schedule_->schedule_id)
            throw std::invalid_argument("Arm schedule ID mismatch");
          if (!prepare_synchronous_start_locked())
            throw std::invalid_argument("hardware cannot preload safe PPS-synchronous outputs");
          if (pps_sequence_ >= arm_request_.after_pps_sequence) {
            if (!hardware_.arm_output_gate_on_next_pps())
              throw std::invalid_argument("hardware cannot arm PPS-synchronous output gate");
            synchronous_start_armed_ = true;
          }
          state_ = ControllerState::kWaitingForPps;
        }
        response = make_ack(request, ControllerState::kWaitingForPps, "waiting for PPS");
        break;
      }
      case MessageType::kStartAtCurrentTick: {
        const auto requested_start = decode_start_at_current_tick_request(request.payload);
        if (policy_.maximum_host_silence_ticks == 0U)
          throw std::invalid_argument("fail-closed host watchdog policy is not configured");
        {
          CriticalGuard critical(hardware_);
          if (state_ != ControllerState::kFrozen || !schedule_)
            throw std::invalid_argument("StartAtCurrentTick requires frozen configuration");
          if (requested_start.schedule_id != schedule_->schedule_id)
            throw std::invalid_argument("StartAtCurrentTick schedule ID mismatch");
          pps_watchdog_required_ = false;
          if (!start_at_current_tick_locked(current_tick))
            throw std::invalid_argument("hardware cannot safely start from current tick");
          state_ = ControllerState::kRunning;
        }
        response = make_ack(request, ControllerState::kRunning,
                            "running from controller tick without PPS");
        break;
      }
      case MessageType::kDisarm: {
        ControllerState result;
        {
          CriticalGuard critical(hardware_);
          if (!request.payload.empty()) throw std::invalid_argument("Disarm payload must be empty");
          disable_outputs_isr();
          state_ = error_flags_ == 0U ? ControllerState::kIdle : ControllerState::kFault;
          if (error_flags_ == 0U) schedule_.reset();
          result = state_;
        }
        response = make_ack(request, result, "outputs gated inactive");
        break;
      }
      case MessageType::kDisarmKeepConfiguration: {
        ControllerState result;
        {
          CriticalGuard critical(hardware_);
          if (!request.payload.empty() || !schedule_)
            throw std::invalid_argument("keep-config Disarm requires a frozen schedule");
          disable_outputs_isr();
          state_ = error_flags_ == 0U ? ControllerState::kFrozen : ControllerState::kFault;
          result = state_;
        }
        response = make_ack(request, result, "outputs gated; configuration remains frozen");
        break;
      }
      case MessageType::kStatus: {
        if (!request.payload.empty()) throw std::invalid_argument("Status payload must be empty");
        StatusReport snapshot;
        { CriticalGuard critical(hardware_); snapshot = status_locked(current_tick); }
        response = {kProtocolVersion, MessageType::kStatus, request.sequence,
                    encode(snapshot)};
        break;
      }
      default:
        throw std::invalid_argument("unsupported host command type");
    }
  } catch (const std::exception& error) {
    {
      CriticalGuard critical(hardware_);
      force_fault_isr(request.type == MessageType::kConfigureSchedule
                          ? kErrorInvalidSchedule : kErrorProtocol);
    }
    response = make_error(request, 5U, error.what());
  }
  cache(request, response);
  return response;
}

bool FirmwareCore::prepare_synchronous_start_locked() noexcept {
  for (auto& oscillator : oscillators_) {
    oscillator.remainder_accumulator = 0U;
    oscillator.channel_sequence = 0U;
    if (!oscillator.enabled) continue;
    if (oscillator.base_interval < hardware_.minimum_compare_lead_ticks())
      return false;
    if (!hardware_.prepare_pps_synchronous_output(
            oscillator.output, oscillator.phase, oscillator.width))
      return false;
  }
  return true;
}

bool FirmwareCore::start_at_current_tick_locked(std::uint64_t current_tick) noexcept {
  const auto lead = std::max<std::uint64_t>(
      1U, static_cast<std::uint64_t>(hardware_.minimum_compare_lead_ticks()));
  if (current_tick > (std::numeric_limits<std::uint64_t>::max)() - lead)
    return false;
  const auto epoch = current_tick + lead;
  for (auto& oscillator : oscillators_) {
    oscillator.remainder_accumulator = 0U;
    oscillator.channel_sequence = 0U;
    if (!oscillator.enabled) continue;
    if (oscillator.base_interval < hardware_.minimum_compare_lead_ticks() ||
        epoch > (std::numeric_limits<std::uint64_t>::max)() - oscillator.phase)
      return false;
    oscillator.next_tick = epoch + oscillator.phase;
    if (!hardware_.configure_output_compare(
            oscillator.output, oscillator.next_tick, oscillator.width))
      return false;
  }
  hardware_.set_output_gate(true);
  output_gate_enabled_ = true;
  synchronous_start_armed_ = false;
  return true;
}

bool FirmwareCore::schedule_next_isr(Oscillator& oscillator,
                                     std::uint64_t from_tick) noexcept {
  std::uint64_t interval = oscillator.base_interval;
  oscillator.remainder_accumulator += oscillator.interval_remainder;
  if (oscillator.remainder_accumulator >= oscillator.numerator) {
    ++interval;
    oscillator.remainder_accumulator -= oscillator.numerator;
  }
  if (interval == 0U || from_tick > (std::numeric_limits<std::uint64_t>::max)() - interval)
    return false;
  oscillator.next_tick = from_tick + interval;
  return hardware_.configure_output_compare(oscillator.output, oscillator.next_tick,
                                             oscillator.width);
}

bool FirmwareCore::push_event_isr(const FirmwareEvent& event) noexcept {
  if (events_.push_isr(event)) return true;
  force_fault_isr(kErrorEventOverflow);
  return false;
}

TimeLock FirmwareCore::time_quality(std::uint64_t tick) const noexcept {
  if (!have_pps_ || !schedule_ || tick < last_pps_tick_) return TimeLock::kUnsynced;
  return tick - last_pps_tick_ <= schedule_->ticks_per_second
             ? TimeLock::kLocked : TimeLock::kHoldover;
}

void FirmwareCore::emit_trigger_isr(Oscillator& oscillator, std::uint64_t tick) noexcept {
  if (have_pps_ && tick < last_pps_tick_) {
    force_fault_isr(kErrorCounterOrTimeRegression);
    return;
  }
  ++oscillator.channel_sequence;
  const auto emit = [this, &oscillator, tick](Channel channel) {
    FirmwareEvent event;
    event.kind = EventKind::trigger;
    event.trigger = TriggerEvent{boot_id_, ++global_event_sequence_, channel,
        oscillator.channel_sequence, have_pps_ ? pps_sequence_ : 0U,
        have_pps_ ? tick - last_pps_tick_ : tick,
        schedule_->ticks_per_second, time_quality(tick)};
    return push_event_isr(event);
  };
  if (oscillator.output == PhysicalOutput::snapshot) {
    if (!emit(Channel::kRgb)) return;
    (void)emit(Channel::kThermal);
  } else {
    (void)emit(oscillator.logical_channel);
  }
}

void FirmwareCore::on_pps_capture_isr(std::uint64_t captured_tick) noexcept {
  CriticalGuard critical(hardware_);
  if (captured_tick < last_observed_tick_ || (have_pps_ && captured_tick <= last_pps_tick_)) {
    force_fault_isr(kErrorCounterOrTimeRegression);
    return;
  }
  last_observed_tick_ = captured_tick;
  last_pps_tick_ = captured_tick;
  have_pps_ = true;
  ++pps_sequence_;
  FirmwareEvent event;
  event.kind = EventKind::pps;
  event.pps = PpsAnchor{boot_id_, pps_sequence_,
                        (std::numeric_limits<std::int64_t>::min)(), captured_tick,
                        TimeLock::kLocked};
  if (!push_event_isr(event)) return;
  if (state_ == ControllerState::kWaitingForPps) {
    if (synchronous_start_armed_ && pps_sequence_ > arm_request_.after_pps_sequence) {
      for (auto& oscillator : oscillators_) {
        if (!oscillator.enabled) continue;
        if (captured_tick > (std::numeric_limits<std::uint64_t>::max)() - oscillator.phase) {
          force_fault_isr(kErrorHardwareSchedule);
          return;
        }
        oscillator.next_tick = captured_tick + oscillator.phase;
      }
      // The HAL gate changed on the PPS hardware path, not at this ISR statement.
      output_gate_enabled_ = true;
      synchronous_start_armed_ = false;
      state_ = ControllerState::kRunning;
    } else if (!synchronous_start_armed_ &&
               pps_sequence_ >= arm_request_.after_pps_sequence) {
      if (!hardware_.arm_output_gate_on_next_pps()) {
        force_fault_isr(kErrorHardwareSchedule);
        return;
      }
      synchronous_start_armed_ = true;
    }
  }
}

void FirmwareCore::on_output_compare_isr(PhysicalOutput output,
                                         std::uint64_t captured_tick) noexcept {
  CriticalGuard critical(hardware_);
  if (state_ != ControllerState::kRunning || !output_gate_enabled_) return;
  auto found = std::find_if(oscillators_.begin(), oscillators_.end(),
      [output](const Oscillator& oscillator) { return oscillator.output == output; });
  if (found == oscillators_.end() || !found->enabled || captured_tick != found->next_tick ||
      captured_tick < last_observed_tick_) {
    force_fault_isr(kErrorCounterOrTimeRegression);
    return;
  }
  last_observed_tick_ = captured_tick;
  emit_trigger_isr(*found, captured_tick);
  if (state_ == ControllerState::kRunning && !schedule_next_isr(*found, captured_tick))
    force_fault_isr(kErrorHardwareSchedule);
}

void FirmwareCore::service_watchdogs_isr(std::uint64_t current_tick) noexcept {
  CriticalGuard critical(hardware_);
  if (current_tick < last_observed_tick_) {
    force_fault_isr(kErrorCounterOrTimeRegression);
    return;
  }
  last_observed_tick_ = current_tick;
  if (state_ != ControllerState::kWaitingForPps && state_ != ControllerState::kRunning) return;
  if (policy_.maximum_host_silence_ticks == 0U ||
      current_tick - last_host_activity_tick_ > policy_.maximum_host_silence_ticks) {
    force_fault_isr(kErrorHostWatchdog);
    return;
  }
  if (pps_watchdog_required_) {
    const auto pps_reference = have_pps_ ? last_pps_tick_ : last_host_activity_tick_;
    if (policy_.maximum_pps_silence_ticks == 0U ||
        current_tick - pps_reference > policy_.maximum_pps_silence_ticks)
      force_fault_isr(kErrorPpsWatchdog);
  }
}

StatusReport FirmwareCore::status_locked(std::uint64_t current_tick) const noexcept {
  std::uint32_t missed = 0U;
  if (have_pps_ && schedule_ && current_tick > last_pps_tick_) {
    const auto elapsed_seconds = (current_tick - last_pps_tick_) / schedule_->ticks_per_second;
    missed = elapsed_seconds > (std::numeric_limits<std::uint32_t>::max)()
                 ? (std::numeric_limits<std::uint32_t>::max)()
                 : static_cast<std::uint32_t>(elapsed_seconds);
  }
  return {boot_id_, state_, schedule_ ? schedule_->schedule_id : 0U,
          cached_request_ ? cached_request_->sequence : 0U, pps_sequence_, current_tick,
          time_quality(current_tick), missed, error_flags_};
}

bool FirmwareCore::pop_event(FirmwareEvent& event) noexcept {
  CriticalGuard critical(hardware_);
  return events_.pop(event);
}

StatusReport FirmwareCore::status(std::uint64_t current_tick) const noexcept {
  CriticalGuard critical(hardware_);
  return status_locked(current_tick);
}

ControllerState FirmwareCore::state() const noexcept {
  CriticalGuard critical(hardware_);
  return state_;
}

bool FirmwareCore::output_gate_enabled() const noexcept {
  CriticalGuard critical(hardware_);
  return output_gate_enabled_;
}

std::uint32_t FirmwareCore::error_flags() const noexcept {
  CriticalGuard critical(hardware_);
  return error_flags_;
}

std::uint32_t FirmwareCore::boot_id() const noexcept {
  CriticalGuard critical(hardware_);
  return boot_id_;
}

}  // namespace ppbng_timing_firmware
