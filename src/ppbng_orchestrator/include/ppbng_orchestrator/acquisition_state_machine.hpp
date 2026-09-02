#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ppbng_orchestrator
{

enum class AcquisitionState : std::uint8_t
{
  idle = 0,
  preflight = 1,
  waiting_for_dark = 2,
  capturing_dark = 3,
  waiting_for_sample = 4,
  waiting_for_pps = 5,
  recording = 6,
  stopping = 7,
  finalized = 8,
  fault = 9,
};

enum class Health : std::uint8_t
{
  unknown = 0,
  ok = 1,
  degraded = 2,
  fault = 3,
};

enum class GlobalFault : std::uint8_t
{
  disk_write_failure = 0,
  insufficient_disk_space = 1,
  trigger_controller_failure = 2,
  finalization_failure = 3,
  dark_capture_failure = 4,
};

struct CommandResult
{
  bool accepted{false};
  bool duplicate_request{false};
  AcquisitionState resulting_state{AcquisitionState::idle};
  std::string session_id;
  std::string message;
};

struct TransitionResult
{
  bool accepted{false};
  AcquisitionState previous_state{AcquisitionState::idle};
  AcquisitionState resulting_state{AcquisitionState::idle};
  std::string message;
};

class AcquisitionStateMachine
{
public:
  using SessionIdFactory = std::function<std::string()>;

  explicit AcquisitionStateMachine(SessionIdFactory session_id_factory = {});

  CommandResult start(
    const std::string & request_id,
    const std::string & dataset_name,
    bool force_degraded);
  CommandResult confirm_dark_ready(const std::string & request_id);
  CommandResult confirm_sample_ready(const std::string & request_id);
  CommandResult stop(const std::string & request_id, const std::string & reason);
  CommandResult abort(const std::string & request_id, const std::string & reason);

  TransitionResult complete_preflight(bool acceptable, bool degraded, const std::string & detail);
  TransitionResult complete_dark_capture(bool successful, const std::string & detail);
  TransitionResult reach_start_pps();
  TransitionResult report_global_fault(GlobalFault fault, const std::string & detail);
  TransitionResult begin_fault_stop();
  TransitionResult complete_stop(bool successful, const std::string & detail);

  void mark_degraded(const std::string & detail);
  void clear_degraded(const std::string & detail = {});

  [[nodiscard]] AcquisitionState state() const noexcept;
  [[nodiscard]] Health health() const noexcept;
  [[nodiscard]] bool degraded() const noexcept;
  [[nodiscard]] const std::string & health_detail() const noexcept;
  [[nodiscard]] const std::string & session_id() const noexcept;
  [[nodiscard]] const std::string & dataset_name() const noexcept;
  [[nodiscard]] bool force_degraded() const noexcept;
  [[nodiscard]] bool has_global_fault() const noexcept;
  [[nodiscard]] GlobalFault global_fault() const noexcept;

private:
  enum class CommandKind : std::uint8_t
  {
    start,
    confirm_dark,
    confirm_sample,
    stop,
    abort,
  };

  struct StoredRequest
  {
    CommandKind kind;
    std::vector<std::string> fields;
    CommandResult result;
  };

  CommandResult execute_command(
    const std::string & request_id,
    CommandKind kind,
    std::vector<std::string> fields,
    const std::function<CommandResult()> & action);
  [[nodiscard]] CommandResult command_result(bool accepted, const std::string & message) const;
  [[nodiscard]] TransitionResult transition_result(
    bool accepted,
    AcquisitionState previous,
    const std::string & message) const;
  [[nodiscard]] bool has_active_session() const noexcept;
  [[nodiscard]] static std::string default_session_id();

  AcquisitionState state_{AcquisitionState::idle};
  Health health_{Health::unknown};
  std::string health_detail_;
  std::string session_id_;
  std::string dataset_name_;
  bool force_degraded_{false};
  bool has_global_fault_{false};
  GlobalFault global_fault_{GlobalFault::disk_write_failure};
  SessionIdFactory session_id_factory_;
  std::unordered_map<std::string, StoredRequest> requests_;
};

}  // namespace ppbng_orchestrator

