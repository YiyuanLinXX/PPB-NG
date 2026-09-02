#include "ppbng_orchestrator/acquisition_state_machine.hpp"

#include <string>

#include <gtest/gtest.h>

using ppbng_orchestrator::AcquisitionState;
using ppbng_orchestrator::AcquisitionStateMachine;
using ppbng_orchestrator::GlobalFault;
using ppbng_orchestrator::Health;

namespace
{
AcquisitionStateMachine make_machine()
{
  return AcquisitionStateMachine([]() {return "session-test";});
}

void advance_to_recording(AcquisitionStateMachine & machine)
{
  ASSERT_TRUE(machine.start("start-1", "plot-a", false).accepted);
  ASSERT_TRUE(machine.complete_preflight(true, false, "ready").accepted);
  ASSERT_TRUE(machine.confirm_dark_ready("dark-1").accepted);
  ASSERT_TRUE(machine.complete_dark_capture(true, "dark saved").accepted);
  ASSERT_TRUE(machine.confirm_sample_ready("sample-1").accepted);
  ASSERT_TRUE(machine.reach_start_pps().accepted);
  ASSERT_EQ(machine.state(), AcquisitionState::recording);
}
}  // namespace

TEST(AcquisitionStateMachine, CompletesNormalWorkflow)
{
  auto machine = make_machine();
  EXPECT_EQ(machine.state(), AcquisitionState::idle);
  EXPECT_EQ(machine.health(), Health::unknown);

  const auto started = machine.start("start-1", "plot-a", false);
  EXPECT_TRUE(started.accepted);
  EXPECT_EQ(started.session_id, "session-test");
  EXPECT_EQ(machine.state(), AcquisitionState::preflight);

  EXPECT_TRUE(machine.complete_preflight(true, false, "ready").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::waiting_for_dark);
  EXPECT_TRUE(machine.confirm_dark_ready("dark-1").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::capturing_dark);
  EXPECT_TRUE(machine.complete_dark_capture(true, "saved").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::waiting_for_sample);
  EXPECT_TRUE(machine.confirm_sample_ready("sample-1").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::waiting_for_pps);
  EXPECT_TRUE(machine.reach_start_pps().accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::recording);
  EXPECT_TRUE(machine.stop("stop-1", "operator stop").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::stopping);
  EXPECT_TRUE(machine.complete_stop(true, "flushed").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::finalized);
}

TEST(AcquisitionStateMachine, HealthIsOrthogonalToTaskState)
{
  auto machine = make_machine();
  advance_to_recording(machine);

  machine.mark_degraded("GNSS RTK not fixed");
  EXPECT_EQ(machine.state(), AcquisitionState::recording);
  EXPECT_EQ(machine.health(), Health::degraded);
  EXPECT_TRUE(machine.degraded());
  EXPECT_EQ(machine.health_detail(), "GNSS RTK not fixed");

  machine.clear_degraded("GNSS recovered");
  EXPECT_EQ(machine.state(), AcquisitionState::recording);
  EXPECT_EQ(machine.health(), Health::ok);
}

TEST(AcquisitionStateMachine, ReplaysIdenticalStartWithoutCreatingAnotherSession)
{
  int factory_calls = 0;
  AcquisitionStateMachine machine([&factory_calls]() {
    return "session-" + std::to_string(++factory_calls);
  });

  const auto first = machine.start("request-a", "plot-a", false);
  ASSERT_TRUE(first.accepted);
  const auto replay = machine.start("request-a", "plot-a", false);
  EXPECT_TRUE(replay.accepted);
  EXPECT_TRUE(replay.duplicate_request);
  EXPECT_EQ(replay.resulting_state, AcquisitionState::preflight);
  EXPECT_EQ(replay.session_id, first.session_id);
  EXPECT_EQ(factory_calls, 1);
}

TEST(AcquisitionStateMachine, RejectsRequestIdReuseWithDifferentPayloadOrCommand)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("same-id", "plot-a", false).accepted);

  const auto changed = machine.start("same-id", "plot-b", false);
  EXPECT_FALSE(changed.accepted);
  EXPECT_FALSE(changed.duplicate_request);

  const auto other_command = machine.stop("same-id", "stop");
  EXPECT_FALSE(other_command.accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::preflight);
}

TEST(AcquisitionStateMachine, ReplaysRejectedCommandAsRejected)
{
  auto machine = make_machine();
  const auto first = machine.confirm_dark_ready("too-early");
  ASSERT_FALSE(first.accepted);

  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  ASSERT_TRUE(machine.complete_preflight(true, false, "ready").accepted);
  const auto replay = machine.confirm_dark_ready("too-early");
  EXPECT_FALSE(replay.accepted);
  EXPECT_TRUE(replay.duplicate_request);
  EXPECT_EQ(replay.resulting_state, AcquisitionState::idle);
  EXPECT_EQ(machine.state(), AcquisitionState::waiting_for_dark);
}

TEST(AcquisitionStateMachine, RejectsEmptyRequestIdsAndDatasetNames)
{
  auto machine = make_machine();
  EXPECT_FALSE(machine.start("", "plot", false).accepted);
  EXPECT_FALSE(machine.start("start", "", false).accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::idle);
}

TEST(AcquisitionStateMachine, RejectsWorkflowCommandsOutOfOrder)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  EXPECT_FALSE(machine.confirm_dark_ready("dark-early").accepted);
  EXPECT_FALSE(machine.confirm_sample_ready("sample-early").accepted);
  EXPECT_FALSE(machine.reach_start_pps().accepted);
  EXPECT_FALSE(machine.complete_dark_capture(true, "wrong state").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::preflight);
}

TEST(AcquisitionStateMachine, RequiresForceForUnacceptablePreflight)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  const auto rejected = machine.complete_preflight(false, true, "RSM degraded");
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::preflight);
  EXPECT_EQ(machine.health(), Health::fault);
}

TEST(AcquisitionStateMachine, AllowsForcedDegradedPreflight)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", true).accepted);
  const auto accepted = machine.complete_preflight(false, true, "RSM degraded");
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::waiting_for_dark);
  EXPECT_EQ(machine.health(), Health::degraded);
}

TEST(AcquisitionStateMachine, DiskFaultUsesControlledStopPath)
{
  auto machine = make_machine();
  advance_to_recording(machine);

  const auto fault = machine.report_global_fault(GlobalFault::disk_write_failure, "write failed");
  EXPECT_TRUE(fault.accepted);
  EXPECT_EQ(fault.previous_state, AcquisitionState::recording);
  EXPECT_EQ(machine.state(), AcquisitionState::fault);
  EXPECT_EQ(machine.health(), Health::fault);
  EXPECT_TRUE(machine.has_global_fault());
  EXPECT_EQ(machine.global_fault(), GlobalFault::disk_write_failure);

  EXPECT_TRUE(machine.begin_fault_stop().accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::stopping);
  EXPECT_TRUE(machine.complete_stop(true, "best-effort flush complete").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::finalized);
  EXPECT_EQ(machine.health(), Health::fault);
}

TEST(AcquisitionStateMachine, TriggerFaultBeforeRecordingAlsoStopsSafely)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  ASSERT_TRUE(machine.complete_preflight(true, false, "ready").accepted);

  machine.report_global_fault(GlobalFault::trigger_controller_failure, "controller missing");
  EXPECT_EQ(machine.state(), AcquisitionState::fault);
  EXPECT_TRUE(machine.begin_fault_stop().accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::stopping);
}

TEST(AcquisitionStateMachine, FaultDuringStopDoesNotLeaveStopping)
{
  auto machine = make_machine();
  advance_to_recording(machine);
  ASSERT_TRUE(machine.stop("stop", "operator").accepted);

  const auto fault = machine.report_global_fault(
    GlobalFault::insufficient_disk_space, "space exhausted during drain");
  EXPECT_TRUE(fault.accepted);
  EXPECT_EQ(fault.previous_state, AcquisitionState::stopping);
  EXPECT_EQ(machine.state(), AcquisitionState::stopping);
  EXPECT_EQ(machine.health(), Health::fault);
}

TEST(AcquisitionStateMachine, AbortAndStopAreControlledAndIdempotent)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  const auto aborted = machine.abort("abort", "operator cancelled setup");
  EXPECT_TRUE(aborted.accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::stopping);

  const auto replay = machine.abort("abort", "operator cancelled setup");
  EXPECT_TRUE(replay.accepted);
  EXPECT_TRUE(replay.duplicate_request);

  const auto additional_stop = machine.stop("stop", "also stop");
  EXPECT_TRUE(additional_stop.accepted);
  EXPECT_FALSE(additional_stop.duplicate_request);
  EXPECT_EQ(machine.state(), AcquisitionState::stopping);
}

TEST(AcquisitionStateMachine, DarkCaptureFailureRequiresControlledStop)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  ASSERT_TRUE(machine.complete_preflight(true, false, "ready").accepted);
  ASSERT_TRUE(machine.confirm_dark_ready("dark").accepted);

  EXPECT_TRUE(machine.complete_dark_capture(false, "shutter timeout").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::fault);
  EXPECT_EQ(machine.global_fault(), GlobalFault::dark_capture_failure);
  EXPECT_TRUE(machine.begin_fault_stop().accepted);
}

TEST(AcquisitionStateMachine, FailedFinalizationReturnsToFaultPath)
{
  auto machine = make_machine();
  advance_to_recording(machine);
  ASSERT_TRUE(machine.stop("stop", "operator").accepted);

  EXPECT_TRUE(machine.complete_stop(false, "index flush failed").accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::fault);
  EXPECT_EQ(machine.global_fault(), GlobalFault::finalization_failure);
  EXPECT_TRUE(machine.begin_fault_stop().accepted);
}

TEST(AcquisitionStateMachine, CanStartNewSessionAfterFinalization)
{
  int factory_calls = 0;
  AcquisitionStateMachine machine([&factory_calls]() {
    return "session-" + std::to_string(++factory_calls);
  });
  advance_to_recording(machine);
  ASSERT_TRUE(machine.stop("stop", "done").accepted);
  ASSERT_TRUE(machine.complete_stop(true, "done").accepted);

  const auto next = machine.start("start-2", "plot-b", false);
  EXPECT_TRUE(next.accepted);
  EXPECT_EQ(next.session_id, "session-2");
  EXPECT_EQ(machine.dataset_name(), "plot-b");
  EXPECT_EQ(machine.state(), AcquisitionState::preflight);
  EXPECT_EQ(machine.health(), Health::ok);
}

TEST(AcquisitionStateMachine, EmptySessionFactoryResultRejectsStart)
{
  AcquisitionStateMachine machine([]() {return std::string{};});
  EXPECT_FALSE(machine.start("start", "plot", false).accepted);
  EXPECT_EQ(machine.state(), AcquisitionState::idle);
}

TEST(AcquisitionStateMachine, FaultHealthCannotBeClearedAsOrdinaryDegradation)
{
  auto machine = make_machine();
  ASSERT_TRUE(machine.start("start", "plot", false).accepted);
  machine.report_global_fault(GlobalFault::trigger_controller_failure, "lost");
  ASSERT_EQ(machine.health(), Health::fault);

  machine.mark_degraded("less severe");
  machine.clear_degraded("recovered");
  EXPECT_EQ(machine.health(), Health::fault);
  EXPECT_EQ(machine.health_detail(), "lost");
}

