#include "ppbng_orchestrator/acquisition_state_machine.hpp"

#include <atomic>
#include <utility>

namespace ppbng_orchestrator
{

AcquisitionStateMachine::AcquisitionStateMachine(SessionIdFactory session_id_factory)
: session_id_factory_(std::move(session_id_factory))
{
  if (!session_id_factory_) {
    session_id_factory_ = &AcquisitionStateMachine::default_session_id;
  }
}

CommandResult AcquisitionStateMachine::start(
  const std::string & request_id,
  const std::string & dataset_name,
  const bool force_degraded)
{
  return execute_command(
    request_id, CommandKind::start, {dataset_name, force_degraded ? "true" : "false"},
    [this, &dataset_name, force_degraded]() {
      if (dataset_name.empty()) {
        return command_result(false, "dataset_name must not be empty");
      }
      if (state_ != AcquisitionState::idle && state_ != AcquisitionState::finalized) {
        return command_result(false, "start is only valid in IDLE or FINALIZED");
      }

      const std::string new_session_id = session_id_factory_();
      if (new_session_id.empty()) {
        return command_result(false, "session ID factory returned an empty ID");
      }

      session_id_ = new_session_id;
      dataset_name_ = dataset_name;
      force_degraded_ = force_degraded;
      has_global_fault_ = false;
      health_ = Health::ok;
      health_detail_.clear();
      state_ = AcquisitionState::preflight;
      return command_result(true, "session created; preflight required");
    });
}

CommandResult AcquisitionStateMachine::confirm_dark_ready(const std::string & request_id)
{
  return execute_command(
    request_id, CommandKind::confirm_dark, {}, [this]() {
      if (state_ != AcquisitionState::waiting_for_dark) {
        return command_result(false, "dark readiness is only valid in WAITING_FOR_DARK");
      }
      state_ = AcquisitionState::capturing_dark;
      return command_result(true, "dark capture authorized");
    });
}

CommandResult AcquisitionStateMachine::confirm_sample_ready(const std::string & request_id)
{
  return execute_command(
    request_id, CommandKind::confirm_sample, {}, [this]() {
      if (state_ != AcquisitionState::waiting_for_sample) {
        return command_result(false, "sample readiness is only valid in WAITING_FOR_SAMPLE");
      }
      state_ = AcquisitionState::waiting_for_pps;
      return command_result(true, "sample ready; waiting for start PPS");
    });
}

CommandResult AcquisitionStateMachine::stop(
  const std::string & request_id, const std::string & reason)
{
  return execute_command(
    request_id, CommandKind::stop, {reason}, [this]() {
      if (!has_active_session() || state_ == AcquisitionState::finalized) {
        return command_result(false, "there is no active session to stop");
      }
      if (state_ == AcquisitionState::stopping) {
        return command_result(true, "controlled stop is already in progress");
      }
      state_ = AcquisitionState::stopping;
      return command_result(true, "controlled stop requested");
    });
}

CommandResult AcquisitionStateMachine::abort(
  const std::string & request_id, const std::string & reason)
{
  return execute_command(
    request_id, CommandKind::abort, {reason}, [this]() {
      if (!has_active_session() || state_ == AcquisitionState::finalized) {
        return command_result(false, "there is no active session to abort");
      }
      if (state_ == AcquisitionState::stopping) {
        return command_result(true, "controlled stop is already in progress");
      }
      state_ = AcquisitionState::stopping;
      return command_result(true, "abort accepted; controlled stop requested");
    });
}

TransitionResult AcquisitionStateMachine::complete_preflight(
  const bool acceptable, const bool degraded, const std::string & detail)
{
  const auto previous = state_;
  if (state_ != AcquisitionState::preflight) {
    return transition_result(false, previous, "preflight completion is only valid in PREFLIGHT");
  }
  if (!acceptable && !force_degraded_) {
    health_ = Health::fault;
    health_detail_ = detail;
    return transition_result(false, previous, "preflight rejected; force_degraded was not set");
  }

  if (!acceptable || degraded) {
    health_ = Health::degraded;
    health_detail_ = detail;
  } else {
    health_ = Health::ok;
    health_detail_ = detail;
  }
  state_ = AcquisitionState::waiting_for_dark;
  return transition_result(true, previous, "preflight completed");
}

TransitionResult AcquisitionStateMachine::complete_dark_capture(
  const bool successful, const std::string & detail)
{
  const auto previous = state_;
  if (state_ != AcquisitionState::capturing_dark) {
    return transition_result(false, previous, "dark completion is only valid in CAPTURING_DARK");
  }
  if (!successful) {
    has_global_fault_ = true;
    global_fault_ = GlobalFault::dark_capture_failure;
    health_ = Health::fault;
    health_detail_ = detail;
    state_ = AcquisitionState::fault;
    return transition_result(true, previous, "dark capture failed; controlled stop required");
  }
  state_ = AcquisitionState::waiting_for_sample;
  return transition_result(true, previous, "dark capture completed");
}

TransitionResult AcquisitionStateMachine::reach_start_pps()
{
  const auto previous = state_;
  if (state_ != AcquisitionState::waiting_for_pps) {
    return transition_result(false, previous, "start PPS is only valid in WAITING_FOR_PPS");
  }
  state_ = AcquisitionState::recording;
  return transition_result(true, previous, "recording started on PPS boundary");
}

TransitionResult AcquisitionStateMachine::report_global_fault(
  const GlobalFault fault, const std::string & detail)
{
  const auto previous = state_;
  has_global_fault_ = true;
  global_fault_ = fault;
  health_ = Health::fault;
  health_detail_ = detail;

  if (state_ == AcquisitionState::stopping) {
    return transition_result(true, previous, "global fault recorded while controlled stop continues");
  }
  state_ = AcquisitionState::fault;
  return transition_result(true, previous, "global fault recorded; controlled stop required");
}

TransitionResult AcquisitionStateMachine::begin_fault_stop()
{
  const auto previous = state_;
  if (state_ != AcquisitionState::fault || !has_active_session()) {
    return transition_result(false, previous, "fault stop requires FAULT with an active session");
  }
  state_ = AcquisitionState::stopping;
  return transition_result(true, previous, "controlled fault stop started");
}

TransitionResult AcquisitionStateMachine::complete_stop(
  const bool successful, const std::string & detail)
{
  const auto previous = state_;
  if (state_ != AcquisitionState::stopping) {
    return transition_result(false, previous, "stop completion is only valid in STOPPING");
  }
  if (!successful) {
    has_global_fault_ = true;
    global_fault_ = GlobalFault::finalization_failure;
    health_ = Health::fault;
    health_detail_ = detail;
    state_ = AcquisitionState::fault;
    return transition_result(true, previous, "finalization failed");
  }
  state_ = AcquisitionState::finalized;
  return transition_result(true, previous, "session finalized");
}

void AcquisitionStateMachine::mark_degraded(const std::string & detail)
{
  if (health_ != Health::fault) {
    health_ = Health::degraded;
    health_detail_ = detail;
  }
}

void AcquisitionStateMachine::clear_degraded(const std::string & detail)
{
  if (health_ == Health::degraded) {
    health_ = Health::ok;
    health_detail_ = detail;
  }
}

AcquisitionState AcquisitionStateMachine::state() const noexcept {return state_;}
Health AcquisitionStateMachine::health() const noexcept {return health_;}
bool AcquisitionStateMachine::degraded() const noexcept {return health_ == Health::degraded;}
const std::string & AcquisitionStateMachine::health_detail() const noexcept {return health_detail_;}
const std::string & AcquisitionStateMachine::session_id() const noexcept {return session_id_;}
const std::string & AcquisitionStateMachine::dataset_name() const noexcept {return dataset_name_;}
bool AcquisitionStateMachine::force_degraded() const noexcept {return force_degraded_;}
bool AcquisitionStateMachine::has_global_fault() const noexcept {return has_global_fault_;}
GlobalFault AcquisitionStateMachine::global_fault() const noexcept {return global_fault_;}

CommandResult AcquisitionStateMachine::execute_command(
  const std::string & request_id,
  const CommandKind kind,
  std::vector<std::string> fields,
  const std::function<CommandResult()> & action)
{
  if (request_id.empty()) {
    return command_result(false, "request_id must not be empty");
  }

  const auto existing = requests_.find(request_id);
  if (existing != requests_.end()) {
    if (existing->second.kind != kind || existing->second.fields != fields) {
      return command_result(false, "request_id was already used with a different command or payload");
    }
    CommandResult replay = existing->second.result;
    replay.duplicate_request = true;
    return replay;
  }

  CommandResult result = action();
  requests_.emplace(request_id, StoredRequest{kind, std::move(fields), result});
  return result;
}

CommandResult AcquisitionStateMachine::command_result(
  const bool accepted, const std::string & message) const
{
  return CommandResult{accepted, false, state_, session_id_, message};
}

TransitionResult AcquisitionStateMachine::transition_result(
  const bool accepted, const AcquisitionState previous, const std::string & message) const
{
  return TransitionResult{accepted, previous, state_, message};
}

bool AcquisitionStateMachine::has_active_session() const noexcept
{
  return !session_id_.empty() && state_ != AcquisitionState::idle;
}

std::string AcquisitionStateMachine::default_session_id()
{
  static std::atomic<std::uint64_t> counter{0};
  return "session-" + std::to_string(++counter);
}

}  // namespace ppbng_orchestrator

