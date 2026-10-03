#include "ppbng_rsm400/readiness_gate.hpp"

#include <cmath>
#include <sstream>

namespace ppbng_rsm400
{

ReadinessGate::ReadinessGate(ReadinessConfiguration configuration)
: configuration_(configuration)
{
  const auto configured = configuration_result();
  if (configured.failed()) {
    state_ = configured.state;
    detail_ = configured.detail;
  }
}

ReadinessResult ReadinessGate::configuration_result() const
{
  if (!configuration_.require_ready_telemetry) {
    return {ReadinessState::failed, "require_ready_telemetry must be explicitly true"};
  }
  if (!configuration_.stab_major_status_confirmed) {
    return {ReadinessState::failed,
      "expected STAB major status has not been confirmed from the ICD"};
  }
  if (configuration_.expected_stab_major_status < 0 ||
    configuration_.expected_stab_major_status > 9)
  {
    return {ReadinessState::failed, "expected_stab_major_status must be a confirmed digit 0..9"};
  }
  if (configuration_.maximum_error_level < 0 || configuration_.maximum_error_level > 9) {
    return {ReadinessState::failed, "maximum_ready_error_level must be within 0..9"};
  }
  return {ReadinessState::waiting, "waiting for validated RSM400 readiness telemetry"};
}

ReadinessResult ReadinessGate::observe(const Telemetry & telemetry)
{
  if (state_ != ReadinessState::waiting) {return {state_, detail_};}
  if (telemetry.roll_deg) {
    if (!std::isfinite(*telemetry.roll_deg)) {
      state_ = ReadinessState::failed;
      detail_ = "readiness telemetry contains non-finite roll";
      return {state_, detail_};
    }
    roll_valid_ = true;
  }
  if (telemetry.pitch_deg) {
    if (!std::isfinite(*telemetry.pitch_deg)) {
      state_ = ReadinessState::failed;
      detail_ = "readiness telemetry contains non-finite pitch";
      return {state_, detail_};
    }
    pitch_valid_ = true;
  }
  if (telemetry.general_status) {
    general_status_valid_ = true;
    major_status_ = telemetry.general_status->major_status;
    error_level_ = telemetry.general_status->error_level;
    if (error_level_ > configuration_.maximum_error_level) {
      state_ = ReadinessState::failed;
      detail_ = "RSM400 error_level exceeds the configured readiness maximum";
      return {state_, detail_};
    }
  }
  return evaluate();
}

ReadinessResult ReadinessGate::evaluate()
{
  if (!roll_valid_ || !pitch_valid_ || !general_status_valid_) {
    detail_ = "waiting for validated roll, pitch, and general status";
    return {state_, detail_};
  }
  if (major_status_ != configuration_.expected_stab_major_status) {
    std::ostringstream detail;
    detail << "major_status " << major_status_ << " does not equal confirmed STAB value " <<
      configuration_.expected_stab_major_status;
    detail_ = detail.str();
    return {state_, detail_};
  }
  state_ = ReadinessState::ready;
  detail_ = "validated roll, pitch, acceptable error level, and confirmed STAB status received";
  return {state_, detail_};
}

ReadinessResult ReadinessGate::timeout()
{
  if (state_ == ReadinessState::ready || state_ == ReadinessState::failed) {
    return {state_, detail_};
  }
  const auto pending = evaluate();
  state_ = ReadinessState::failed;
  detail_ = "RSM400 readiness timeout: " + pending.detail;
  return {state_, detail_};
}

ReadinessState ReadinessGate::state() const noexcept {return state_;}

}  // namespace ppbng_rsm400
