#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace ppbng_runtime
{

enum class ProductionWorkflowState : std::uint8_t
{
  inert,
  preflight,
  awaiting_dark_cover,
  capturing_dark,
  awaiting_sample_confirmation,
  waiting_for_sample_pps,
  recording,
  stopping,
  fault,
};

enum class DeviceOperation : std::uint8_t
{
  prepare,
  arm,
  start,
  begin_dark,
  start_sample,
  arm_next_pps,
  disarm_keep_configuration,
  stop,
  arm_immediate,
};

struct DeviceAction
{
  std::string device;
  DeviceOperation operation{DeviceOperation::prepare};
};

struct WorkflowBatch
{
  std::uint64_t id{};
  std::vector<DeviceAction> actions;
};

struct ActionOutcome
{
  std::string device;
  DeviceOperation operation{DeviceOperation::prepare};
  bool success{};
  std::string detail;
};

struct WorkflowResult
{
  bool accepted{};
  std::string detail;
  WorkflowBatch batch;
};

// A transport-free, deterministic description of the production ordering.
// It performs no ROS, file, SDK, serial, or hardware operations.
class ProductionWorkflow
{
public:
  explicit ProductionWorkflow(bool pps_required = true) noexcept;

  WorkflowResult begin(std::string session_id);
  WorkflowResult complete_batch(std::uint64_t batch_id, const std::vector<ActionOutcome> & outcomes);
  // A timed-out start response is treated as "possibly opened". This makes the
  // subsequent cleanup conservative even when the transport reply was lost.
  WorkflowResult timeout_pending_batch(std::uint64_t batch_id, std::string detail);
  WorkflowResult confirm_dark_cover();
  WorkflowResult observe_dark_complete(const std::string & device);
  WorkflowResult confirm_sample_ready();
  WorkflowResult observe_sample_pps();
  WorkflowResult request_stop(std::string reason);
  WorkflowResult report_global_fault(std::string detail);

  [[nodiscard]] ProductionWorkflowState state() const noexcept;
  [[nodiscard]] const std::string & session_id() const noexcept;
  [[nodiscard]] const std::string & detail() const noexcept;
  [[nodiscard]] const WorkflowBatch & pending_batch() const noexcept;

private:
  enum class Continuation : std::uint8_t
  {
    none,
    after_prepare,
    after_arm,
    after_context_start,
    after_background_start,
    after_dark_begin,
    after_dark_arm,
    after_dark_disarm,
    after_sample_start,
    after_sample_arm,
    after_stop_disarm,
    after_stop_streams,
    after_stop_background,
    after_stop_context,
  };

  WorkflowResult issue(std::vector<DeviceAction> actions, Continuation continuation,
    std::string detail);
  WorkflowResult reject(std::string detail) const;
  WorkflowResult continue_after_batch();
  WorkflowResult begin_ordered_stop(std::string reason);
  WorkflowResult issue_next_stop_batch();
  bool outcomes_match(const std::vector<ActionOutcome> & outcomes, std::string & error) const;
  void remember_possibly_started_pending_actions();

  ProductionWorkflowState state_{ProductionWorkflowState::inert};
  std::string session_id_;
  std::string detail_{"inert"};
  std::uint64_t next_batch_id_{1U};
  WorkflowBatch pending_;
  Continuation continuation_{Continuation::none};
  std::unordered_set<std::string> dark_complete_;
  bool dark_capture_requested_{};
  std::unordered_set<std::string> started_devices_;
  bool timing_disarmed_{};
  bool timing_stop_required_before_streams_{};
  bool fault_stop_{};
  bool pps_required_{true};
};

}  // namespace ppbng_runtime
