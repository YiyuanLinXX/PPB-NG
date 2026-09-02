#include "ppbng_timing/production_control.hpp"

#include <stdexcept>
#include <utility>

namespace ppbng_timing {

ProductionControl::ProductionControl(ProductionConfiguration configuration,
                                     HostClientFactory factory)
    : configuration_(std::move(configuration)), factory_(std::move(factory)) {}

ProductionControl::~ProductionControl() {
  if (client_ && client_->is_transport_open()) {
    (void)try_disarm();
    client_->close();
  }
}

ProductionResult ProductionControl::prepare() {
  if (state_ != ProductionState::inert) return {false, "timing task is already prepared"};
  if (!configuration_.hardware_enabled)
    return {false, "hardware_enabled=false; timing controller remains inert"};
  if (!configuration_.trusted_usb_identity_mapping)
    return {false, "trusted COM to USB VID/PID/serial mapping is required"};
  if (configuration_.identity.com_path.empty() ||
      configuration_.identity.baud_rate == 0U ||
      configuration_.identity.usb_vid == 0U ||
      configuration_.identity.usb_pid == 0U ||
      configuration_.identity.usb_serial.empty())
    return {false, "COM path, baud, USB VID/PID and serial are required"};
  if (configuration_.command_timeout.count() <= 0 ||
      configuration_.poll_timeout.count() <= 0)
    return {false, "finite positive controller timeouts are required"};
  if (!factory_) return {false, "host client factory is required"};
  try {
    validate_schedule(configuration_.schedule);
  } catch (const std::exception& error) {
    return {false, std::string("invalid timing schedule: ") + error.what()};
  }
  // The OS mapping was resolved and attested outside the bare COM API.
  configuration_.identity.allow_unverified_os_usb_identity = true;
  state_ = ProductionState::prepared;
  return {true, "configuration validated and locked; no COM port opened"};
}

ProductionResult ProductionControl::start() {
  if (state_ != ProductionState::prepared)
    return {false, "prepare must succeed before start"};
  client_ = factory_();
  if (!client_) return latch_fault("host client factory returned null", false);
  auto status = client_->connect(configuration_.identity, configuration_.command_timeout,
                                 stop_source_.token());
  if (status.ok()) status = client_->configure(configuration_.schedule,
                                                configuration_.command_timeout,
                                                stop_source_.token());
  if (status.ok()) status = client_->freeze(configuration_.command_timeout,
                                             stop_source_.token());
  if (!status.ok()) {
    client_->close();
    client_.reset();
    return latch_fault("start failed before outputs were armed: " + status.detail, false);
  }
  state_ = ProductionState::started;
  return {true, "controller connected, configured and frozen; outputs remain disarmed"};
}

ProductionResult ProductionControl::arm_next_pps() {
  if (state_ != ProductionState::started || !client_)
    return {false, "successful start is required before arm_next_pps"};
  const auto status = client_->arm(
      {configuration_.schedule.schedule_id, configuration_.arm_after_pps_sequence},
      configuration_.command_timeout, stop_source_.token());
  if (!status.ok()) return latch_fault("arm_next_pps failed: " + status.detail, true);
  state_ = ProductionState::armed;
  return {true, "outputs armed for the next accepted PPS"};
}

ProductionResult ProductionControl::start_without_pps() {
  if (state_ != ProductionState::started || !client_)
    return {false, "successful start is required before start_without_pps"};
  const auto status = client_->start_at_current_tick(
      {configuration_.schedule.schedule_id}, configuration_.command_timeout,
      stop_source_.token());
  if (!status.ok())
    return latch_fault("start_without_pps failed: " + status.detail, true);
  state_ = ProductionState::armed;
  return {true, "outputs started from controller tick without PPS; time is UNSYNCED"};
}

ProductionResult ProductionControl::disarm_keep_configuration() {
  if ((state_ != ProductionState::armed && state_ != ProductionState::fault) || !client_)
    return {false, "armed timing output is required"};
  const auto status = client_->disarm_keep_configuration(
      configuration_.command_timeout, stop_source_.token());
  if (!status.ok())
    return latch_fault("FATAL: keep-config disarm could not be confirmed: " +
                       status.detail, false);
  state_ = ProductionState::started;
  return {true, "outputs disarmed; connection and frozen configuration retained"};
}

ProductionResult ProductionControl::try_disarm() {
  ++disarm_attempts_;
  const auto status = client_->disarm(configuration_.command_timeout, stop_source_.token());
  return {status.ok(), status.ok() ? "controller output disarm confirmed" : status.detail};
}

ProductionResult ProductionControl::stop() {
  if (!client_) {
    state_ = ProductionState::inert;
    return {true, "timing controller is inert"};
  }
  const auto disarmed = try_disarm();
  if (!disarmed.success) {
    state_ = ProductionState::fault;
    return {false, "FATAL: output disarm could not be confirmed: " + disarmed.detail};
  }
  client_->close();
  client_.reset();
  state_ = ProductionState::inert;
  return {true, "controller disarmed, port closed and configuration unlocked"};
}

ProductionResult ProductionControl::latch_fault(std::string detail, bool attempt_disarm) {
  state_ = ProductionState::fault;
  if (attempt_disarm && client_ && client_->is_transport_open()) {
    const auto result = try_disarm();
    detail += result.success ? "; emergency disarm confirmed"
                             : "; FATAL: emergency disarm unconfirmed: " + result.detail;
  }
  return {false, std::move(detail)};
}

ProductionPollResult ProductionControl::poll() {
  if ((state_ != ProductionState::started && state_ != ProductionState::armed) || !client_)
    return {{false, "controller is not started"}, {}};
  auto polled = client_->poll_events(configuration_.poll_timeout, stop_source_.token());
  if (polled.status.ok()) return {{true, "controller events received"}, std::move(polled.events)};
  if (polled.status.code == HostClientCode::timeout)
    return {{true, "no controller event before poll deadline"}, {}};
  auto fault = latch_fault("timing controller runtime fault: " + polled.status.detail, true);
  return {std::move(fault), std::move(polled.events)};
}

}  // namespace ppbng_timing
