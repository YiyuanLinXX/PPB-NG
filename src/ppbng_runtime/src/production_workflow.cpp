#include "ppbng_runtime/production_workflow.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace ppbng_runtime
{
namespace
{
using Op = DeviceOperation;
DeviceAction action(const char * device, const Op operation) {return {device, operation};}
}  // namespace

ProductionWorkflow::ProductionWorkflow(const bool pps_required) noexcept
: pps_required_(pps_required)
{
}

WorkflowResult ProductionWorkflow::reject(std::string detail) const
{
  return {false, std::move(detail), {}};
}

WorkflowResult ProductionWorkflow::issue(
  std::vector<DeviceAction> actions, const Continuation continuation, std::string detail)
{
  if (!pending_.actions.empty()) {return reject("a device batch is already pending");}
  // Once an arm request is issued, loss of its response leaves the physical
  // output state unknown.  Treat it as possibly armed until a confirmed
  // disarm/stop, so every fault and timeout path begins with hard-disarm.
  if (std::any_of(actions.begin(), actions.end(), [](const DeviceAction & item) {
      return item.device == "timing" &&
             (item.operation == DeviceOperation::arm_next_pps ||
             item.operation == DeviceOperation::arm_immediate);
    }))
  {
    timing_disarmed_ = false;
  }
  pending_.id = next_batch_id_++;
  pending_.actions = std::move(actions);
  continuation_ = continuation;
  detail_ = std::move(detail);
  return {true, detail_, pending_};
}

WorkflowResult ProductionWorkflow::begin(std::string session_id)
{
  if (state_ != ProductionWorkflowState::inert || !pending_.actions.empty()) {
    return reject("production workflow is not inert");
  }
  if (session_id.empty()) {return reject("session_id must not be empty");}
  session_id_ = std::move(session_id);
  dark_complete_.clear();
  dark_capture_requested_ = false;
  started_devices_.clear();
  timing_disarmed_ = false;
  timing_stop_required_before_streams_ = false;
  fault_stop_ = false;
  state_ = ProductionWorkflowState::preflight;
  return issue({action("fx10e", Op::prepare), action("swir", Op::prepare),
    action("rgb", Op::prepare), action("thermal", Op::prepare), action("gnss", Op::prepare),
    action("rsm400", Op::prepare), action("timing", Op::prepare), action("context", Op::prepare)}, Continuation::after_prepare,
    "bind inert storage devices to the existing session");
}

bool ProductionWorkflow::outcomes_match(
  const std::vector<ActionOutcome> & outcomes, std::string & error) const
{
  if (outcomes.size() != pending_.actions.size()) {
    error = "outcome count differs from pending action count";
    return false;
  }
  std::multiset<std::pair<std::string, DeviceOperation>> expected;
  std::multiset<std::pair<std::string, DeviceOperation>> actual;
  for (const auto & item : pending_.actions) {expected.emplace(item.device, item.operation);}
  for (const auto & item : outcomes) {actual.emplace(item.device, item.operation);}
  if (actual != expected) {
    error = "outcomes do not exactly identify the pending actions";
    return false;
  }
  const auto failed = std::find_if(outcomes.begin(), outcomes.end(),
    [](const ActionOutcome & value) {return !value.success;});
  if (failed != outcomes.end()) {
    error = failed->device + " operation failed: " + failed->detail;
    return false;
  }
  return true;
}

void ProductionWorkflow::remember_possibly_started_pending_actions()
{
  for (const auto & item : pending_.actions) {
    if (item.operation == DeviceOperation::start) {
      started_devices_.insert(item.device);
    }
  }
}

WorkflowResult ProductionWorkflow::timeout_pending_batch(
  const std::uint64_t batch_id, std::string detail)
{
  if (pending_.actions.empty() || pending_.id != batch_id) {
    return reject("batch_id is not the currently pending batch");
  }
  remember_possibly_started_pending_actions();
  std::vector<ActionOutcome> outcomes;
  outcomes.reserve(pending_.actions.size());
  for (const auto & item : pending_.actions) {
    outcomes.push_back({item.device, item.operation, false, detail});
  }
  return complete_batch(batch_id, outcomes);
}

WorkflowResult ProductionWorkflow::complete_batch(
  const std::uint64_t batch_id, const std::vector<ActionOutcome> & outcomes)
{
  if (pending_.actions.empty() || pending_.id != batch_id) {
    return reject("batch_id is not the currently pending batch");
  }
  std::string error;
  if (!outcomes_match(outcomes, error)) {
    // If the responses do identify this batch, remember every transport that
    // may already have opened so a partial parallel start is never abandoned.
    for (const auto & outcome : outcomes) {
      const auto expected = std::find_if(pending_.actions.begin(), pending_.actions.end(),
        [&outcome](const DeviceAction & value) {
          return value.device == outcome.device && value.operation == outcome.operation;
        });
      if (expected != pending_.actions.end() && outcome.success &&
        outcome.operation == DeviceOperation::start)
      {
        started_devices_.insert(outcome.device);
        if (outcome.device == "timing") timing_disarmed_ = true;
      }
    }
    if (state_ == ProductionWorkflowState::stopping) {
      // A failed or timed-out stop response must not strand transports that
      // have not yet had their own stop request.  Every action in this batch
      // was attempted once; retire it from the logical open set, latch the
      // terminal fault, and continue the remaining best-effort stop order.
      // This deliberately avoids an unbounded retry loop when a device has
      // disappeared while still allowing background sources and the context
      // writer to checkpoint and close.
      for (const auto & item : pending_.actions) {
        if (item.operation == DeviceOperation::stop) {
          started_devices_.erase(item.device);
        } else if (item.operation == DeviceOperation::disarm_keep_configuration &&
          item.device == "timing")
        {
          // The physical output state is unknown.  Before touching camera
          // streams, issue the timing node's full stop, which performs a
          // second independent disarm attempt and closes only on confirmation.
          timing_stop_required_before_streams_ = true;
        }
      }
      pending_ = {};
      continuation_ = Continuation::none;
      fault_stop_ = true;
      const auto stop_error = "ordered stop failed: " + error;
      auto cleanup = issue_next_stop_batch();
      cleanup.accepted = false;
      cleanup.detail = cleanup.batch.actions.empty() ? stop_error :
        stop_error + "; continuing best-effort cleanup";
      detail_ = cleanup.detail;
      return cleanup;
    }
    pending_ = {};
    continuation_ = Continuation::none;
    fault_stop_ = true;
    detail_ = error;
    if (!started_devices_.empty()) {
      state_ = ProductionWorkflowState::stopping;
      auto cleanup = issue_next_stop_batch();
      cleanup.accepted = false;
      cleanup.detail = error + "; cleanup started";
      return cleanup;
    }
    state_ = ProductionWorkflowState::fault;
    return {false, detail_, {}};
  }
  for (const auto & outcome : outcomes) {
    if (outcome.operation == DeviceOperation::start) {
      started_devices_.insert(outcome.device);
      // Timing start configures and freezes the controller but explicitly
      // leaves all physical outputs disarmed.
      if (outcome.device == "timing") timing_disarmed_ = true;
    } else if (outcome.operation == DeviceOperation::stop) {
      started_devices_.erase(outcome.device);
    }
  }
  pending_ = {};
  return continue_after_batch();
}

WorkflowResult ProductionWorkflow::continue_after_batch()
{
  const auto completed = continuation_;
  continuation_ = Continuation::none;
  switch (completed) {
    case Continuation::after_prepare:
      return issue({action("timing", Op::arm), action("gnss", Op::arm),
        action("rsm400", Op::arm), action("fx10e", Op::arm), action("swir", Op::arm),
        action("rgb", Op::arm), action("thermal", Op::arm), action("context", Op::arm)}, Continuation::after_arm,
        "validate and lock all task configuration");
    case Continuation::after_arm:
      return issue({action("context", Op::start)}, Continuation::after_context_start,
        "start frame context recorder before every data source");
    case Continuation::after_context_start:
      return issue({action("timing", Op::start), action("gnss", Op::start),
        action("rsm400", Op::start), action("fx10e", Op::start), action("swir", Op::start)},
        Continuation::after_background_start,
        "open background devices and HSI with shutters closed; trigger outputs remain disarmed");
    case Continuation::after_background_start:
      state_ = ProductionWorkflowState::awaiting_dark_cover;
      detail_ = "ready for the user to cover both HSI cameras and confirm dark capture";
      return {true, detail_, {}};
    case Continuation::after_dark_begin:
      return issue({action("timing", pps_required_ ? Op::arm_next_pps : Op::arm_immediate)},
        Continuation::after_dark_arm,
        pps_required_ ? "arm dark triggers on the next PPS" :
        "arm dark triggers immediately without PPS gating");
    case Continuation::after_dark_arm:
      timing_disarmed_ = false;
      state_ = ProductionWorkflowState::capturing_dark;
      if (dark_complete_.size() == 2U) {
        dark_capture_requested_ = false;
        return issue({action("timing", Op::disarm_keep_configuration)},
          Continuation::after_dark_disarm,
          "both dark streams completed before timing arm reply; hard-disarm outputs");
      }
      detail_ = "dark capture is running; waiting for both HSI completion signals";
      return {true, detail_, {}};
    case Continuation::after_dark_disarm:
      timing_disarmed_ = true;
      state_ = ProductionWorkflowState::awaiting_sample_confirmation;
      detail_ = "dark capture complete and outputs disarmed; ready for sample confirmation";
      return {true, detail_, {}};
    case Continuation::after_sample_start:
      return issue({action("timing", pps_required_ ? Op::arm_next_pps : Op::arm_immediate)},
        Continuation::after_sample_arm,
        pps_required_ ? "arm sample triggers on the next PPS" :
        "arm sample triggers immediately without PPS gating");
    case Continuation::after_sample_arm:
      timing_disarmed_ = false;
      if (!pps_required_) {
        state_ = ProductionWorkflowState::recording;
        detail_ = "recording after immediate sample trigger arm; PPS is optional";
        return {true, detail_, {}};
      }
      state_ = ProductionWorkflowState::waiting_for_sample_pps;
      detail_ = "sample devices ready; waiting for the first PPS-gated trigger";
      return {true, detail_, {}};
    case Continuation::after_stop_disarm:
      timing_disarmed_ = true;
      return issue_next_stop_batch();
    case Continuation::after_stop_streams:
      return issue_next_stop_batch();
    case Continuation::after_stop_background:
      if (started_devices_.count("context") != 0U) {
        return issue({action("context", Op::stop)}, Continuation::after_stop_context,
          "flush and finalize frame context after all producers stop");
      }
      [[fallthrough]];
    case Continuation::after_stop_context:
      state_ = fault_stop_ ? ProductionWorkflowState::fault : ProductionWorkflowState::inert;
      detail_ = fault_stop_ ? "fault stop completed; operator acknowledgement required" :
        "ordered production stop completed";
      if (!fault_stop_) session_id_.clear();
      return {true, detail_, {}};
    case Continuation::none:
      break;
  }
  state_ = ProductionWorkflowState::fault;
  detail_ = "invalid workflow continuation";
  return {false, detail_, {}};
}

WorkflowResult ProductionWorkflow::confirm_dark_cover()
{
  if (state_ != ProductionWorkflowState::awaiting_dark_cover || !pending_.actions.empty()) {
    return reject("dark confirmation is not valid in the current state");
  }
  dark_complete_.clear();
  dark_capture_requested_ = true;
  return issue({action("fx10e", Op::begin_dark), action("swir", Op::begin_dark)},
    Continuation::after_dark_begin, "start both closed-shutter dark streams");
}

WorkflowResult ProductionWorkflow::observe_dark_complete(const std::string & device)
{
  if (!dark_capture_requested_ || (device != "fx10e" && device != "swir"))
  {
    return reject("dark completion is not valid for this state/device");
  }
  dark_complete_.insert(device);
  if (state_ != ProductionWorkflowState::capturing_dark || !pending_.actions.empty() ||
    dark_complete_.size() != 2U)
  {
    detail_ = dark_complete_.size() == 2U ?
      "both HSI dark streams complete; waiting for the outstanding timing-arm reply" :
      "one HSI dark stream complete; waiting for the other";
    return {true, detail_, {}};
  }
  dark_capture_requested_ = false;
  return issue({action("timing", Op::disarm_keep_configuration)},
    Continuation::after_dark_disarm,
    "both dark streams complete; hard-disarm outputs while preserving the frozen schedule");
}

WorkflowResult ProductionWorkflow::confirm_sample_ready()
{
  if (state_ != ProductionWorkflowState::awaiting_sample_confirmation ||
    !pending_.actions.empty())
  {
    return reject("sample confirmation is not valid in the current state");
  }
  return issue({action("fx10e", Op::start_sample), action("swir", Op::start_sample),
    action("rgb", Op::start), action("thermal", Op::start)},
    Continuation::after_sample_start,
    "open HSI shutters and start snapshot streams while outputs remain disarmed");
}

WorkflowResult ProductionWorkflow::observe_sample_pps()
{
  if (state_ != ProductionWorkflowState::waiting_for_sample_pps || !pending_.actions.empty()) {
    return reject("sample PPS is not valid in the current state");
  }
  state_ = ProductionWorkflowState::recording;
  detail_ = "recording from the first PPS-gated sample trigger";
  return {true, detail_, {}};
}

WorkflowResult ProductionWorkflow::begin_ordered_stop(std::string reason)
{
  if (!pending_.actions.empty()) {return reject("cannot start stop while a batch is pending");}
  state_ = ProductionWorkflowState::stopping;
  detail_ = std::move(reason);
  return issue_next_stop_batch();
}

WorkflowResult ProductionWorkflow::issue_next_stop_batch()
{
  if (timing_stop_required_before_streams_ && started_devices_.count("timing") != 0U) {
    timing_stop_required_before_streams_ = false;
    return issue({action("timing", Op::stop)}, Continuation::after_stop_disarm,
      "keep-config disarm was unconfirmed; retry full timing disarm/stop before streams");
  }
  if (started_devices_.count("timing") != 0U && !timing_disarmed_) {
    return issue({action("timing", Op::disarm_keep_configuration)},
      Continuation::after_stop_disarm, "hard-disarm trigger outputs before stopping any stream");
  }
  std::vector<DeviceAction> cameras;
  for (const auto * device : {"rgb", "thermal", "fx10e", "swir"}) {
    if (started_devices_.count(device) != 0U) cameras.push_back(action(device, Op::stop));
  }
  if (!cameras.empty()) {
    return issue(std::move(cameras), Continuation::after_stop_streams,
      "drain and close every stream that successfully started");
  }
  std::vector<DeviceAction> background;
  for (const auto * device : {"gnss", "rsm400", "timing"}) {
    if (started_devices_.count(device) != 0U) background.push_back(action(device, Op::stop));
  }
  if (!background.empty()) {
    return issue(std::move(background), Continuation::after_stop_background,
      "checkpoint background streams and close transports");
  }
  if (started_devices_.count("context") != 0U) {
    return issue({action("context", Op::stop)}, Continuation::after_stop_context,
      "flush and finalize frame context after all producers stop");
  }
  state_ = fault_stop_ ? ProductionWorkflowState::fault : ProductionWorkflowState::inert;
  detail_ = fault_stop_ ? "fault stop completed; operator acknowledgement required" :
    "ordered production stop completed";
  if (!fault_stop_) session_id_.clear();
  return {true, detail_, {}};
}

WorkflowResult ProductionWorkflow::request_stop(std::string reason)
{
  if (state_ == ProductionWorkflowState::inert || state_ == ProductionWorkflowState::fault ||
    state_ == ProductionWorkflowState::stopping)
  {
    return reject("ordered stop is not valid in the current state");
  }
  return begin_ordered_stop(reason.empty() ? "operator requested stop" : std::move(reason));
}

WorkflowResult ProductionWorkflow::report_global_fault(std::string detail)
{
  if (state_ == ProductionWorkflowState::inert || state_ == ProductionWorkflowState::stopping) {
    return reject("global fault is not actionable in the current state");
  }
  fault_stop_ = true;
  if (!pending_.actions.empty()) {
    remember_possibly_started_pending_actions();
    pending_ = {};
    continuation_ = Continuation::none;
  }
  if (started_devices_.empty()) {
    state_ = ProductionWorkflowState::fault;
    detail_ = "global fault before any device transport opened: " + std::move(detail);
    return {false, detail_, {}};
  }
  return begin_ordered_stop("global fault: " + std::move(detail));
}

ProductionWorkflowState ProductionWorkflow::state() const noexcept {return state_;}
const std::string & ProductionWorkflow::session_id() const noexcept {return session_id_;}
const std::string & ProductionWorkflow::detail() const noexcept {return detail_;}
const WorkflowBatch & ProductionWorkflow::pending_batch() const noexcept {return pending_;}

}  // namespace ppbng_runtime
