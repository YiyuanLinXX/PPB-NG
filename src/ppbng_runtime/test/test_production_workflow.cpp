#include "ppbng_runtime/production_workflow.hpp"

#include <gtest/gtest.h>

namespace
{
using ppbng_runtime::ActionOutcome;
using ppbng_runtime::DeviceOperation;
using ppbng_runtime::ProductionWorkflow;
using ppbng_runtime::ProductionWorkflowState;
using ppbng_runtime::WorkflowResult;

WorkflowResult succeed(ProductionWorkflow & workflow, const WorkflowResult & issued)
{
  std::vector<ActionOutcome> outcomes;
  for (const auto & action : issued.batch.actions) {
    outcomes.push_back({action.device, action.operation, true, "ok"});
  }
  return workflow.complete_batch(issued.batch.id, outcomes);
}

WorkflowResult reach_dark_prompt(ProductionWorkflow & workflow)
{
  auto result = workflow.begin("session-1");
  result = succeed(workflow, result);  // prepare -> arm
  result = succeed(workflow, result);  // arm -> context start
  result = succeed(workflow, result);  // context -> background start
  return succeed(workflow, result);    // background -> prompt
}

WorkflowResult reach_recording(ProductionWorkflow & workflow)
{
  auto result = reach_dark_prompt(workflow);
  result = workflow.confirm_dark_cover();
  result = succeed(workflow, result);  // dark begin -> timing arm
  result = succeed(workflow, result);  // timing arm -> capturing
  workflow.observe_dark_complete("fx10e");
  result = workflow.observe_dark_complete("swir");
  result = succeed(workflow, result);  // disarm -> sample prompt
  result = workflow.confirm_sample_ready();
  result = succeed(workflow, result);  // streams -> timing arm
  result = succeed(workflow, result);  // timing arm -> waiting PPS
  return workflow.observe_sample_pps();
}

WorkflowResult reach_recording_without_pps(ProductionWorkflow & workflow)
{
  auto result = reach_dark_prompt(workflow);
  result = workflow.confirm_dark_cover();
  result = succeed(workflow, result);  // dark begin -> immediate timing arm
  result = succeed(workflow, result);  // immediate timing arm -> capturing
  workflow.observe_dark_complete("fx10e");
  result = workflow.observe_dark_complete("swir");
  result = succeed(workflow, result);  // disarm -> sample prompt
  result = workflow.confirm_sample_ready();
  result = succeed(workflow, result);  // streams -> immediate timing arm
  return succeed(workflow, result);    // immediate timing arm -> recording
}

TEST(ProductionWorkflow, EnforcesDarkThenSampleAndStartsOnlyAtObservedPps)
{
  ProductionWorkflow workflow;
  auto result = reach_dark_prompt(workflow);
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::awaiting_dark_cover);
  result = workflow.confirm_dark_cover();
  ASSERT_EQ(result.batch.actions.size(), 2U);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.at(0).operation, DeviceOperation::arm_next_pps);
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::capturing_dark);
  EXPECT_TRUE(workflow.observe_dark_complete("fx10e").batch.actions.empty());
  result = workflow.observe_dark_complete("swir");
  ASSERT_EQ(result.batch.actions.at(0).operation, DeviceOperation::disarm_keep_configuration);
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::awaiting_sample_confirmation);
  result = workflow.confirm_sample_ready();
  EXPECT_EQ(result.batch.actions.size(), 4U);
  result = succeed(workflow, result);
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::waiting_for_sample_pps);
  EXPECT_EQ(workflow.observe_sample_pps().accepted, true);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::recording);
}

TEST(ProductionWorkflow, StopAlwaysDisarmsBeforeStreamsAndBackground)
{
  ProductionWorkflow workflow;
  ASSERT_TRUE(reach_recording(workflow).accepted);
  auto result = workflow.request_stop("done");
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions[0].device, "timing");
  EXPECT_EQ(result.batch.actions[0].operation, DeviceOperation::disarm_keep_configuration);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 4U);
  EXPECT_EQ(result.batch.actions[0].device, "rgb");
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 3U);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions[0].device, "context");
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::inert);
  EXPECT_TRUE(workflow.session_id().empty());
}

TEST(ProductionWorkflow, OptionalPpsCompletesDarkAndSampleWithImmediateArming)
{
  ProductionWorkflow workflow(false);
  auto result = reach_dark_prompt(workflow);
  ASSERT_TRUE(result.accepted);

  result = workflow.confirm_dark_cover();
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "timing");
  EXPECT_EQ(result.batch.actions.front().operation, DeviceOperation::arm_immediate);
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::capturing_dark);

  EXPECT_TRUE(workflow.observe_dark_complete("fx10e").accepted);
  result = workflow.observe_dark_complete("swir");
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().operation,
    DeviceOperation::disarm_keep_configuration);
  result = succeed(workflow, result);

  result = workflow.confirm_sample_ready();
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().operation, DeviceOperation::arm_immediate);
  result = succeed(workflow, result);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::recording);
  EXPECT_FALSE(workflow.observe_sample_pps().accepted);
}

TEST(ProductionWorkflow, OptionalPpsStopStillDisarmsBeforeEveryProducer)
{
  ProductionWorkflow workflow(false);
  ASSERT_TRUE(reach_recording_without_pps(workflow).accepted);

  auto result = workflow.request_stop("done without PPS");
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "timing");
  EXPECT_EQ(result.batch.actions.front().operation,
    DeviceOperation::disarm_keep_configuration);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 4U);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 3U);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "context");
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::inert);
}

TEST(ProductionWorkflow, OptionalPpsImmediateArmFailureBeginsWithHardDisarm)
{
  ProductionWorkflow workflow(false);
  auto result = reach_dark_prompt(workflow);
  result = workflow.confirm_dark_cover();
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.front().operation, DeviceOperation::arm_immediate);

  const std::vector<ActionOutcome> failed{{"timing", DeviceOperation::arm_immediate,
    false, "controller rejected immediate arm"}};
  result = workflow.complete_batch(result.batch.id, failed);
  EXPECT_FALSE(result.accepted);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "timing");
  EXPECT_EQ(result.batch.actions.front().operation,
    DeviceOperation::disarm_keep_configuration);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::stopping);
}

TEST(ProductionWorkflow, RejectsStaleOrIncompleteBatchWithoutAdvancing)
{
  ProductionWorkflow workflow;
  const auto result = workflow.begin("session-1");
  EXPECT_FALSE(workflow.complete_batch(result.batch.id + 1U, {}).accepted);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::preflight);
  EXPECT_FALSE(workflow.complete_batch(result.batch.id, {}).accepted);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::fault);
}

TEST(ProductionWorkflow, GlobalFaultCancelsPendingAndBeginsWithHardDisarm)
{
  ProductionWorkflow workflow;
  ASSERT_TRUE(workflow.begin("session-1").accepted);
  const auto result = workflow.report_global_fault("disk write failed");
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.batch.actions.empty());
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::fault);
}

TEST(ProductionWorkflow, PartialBackgroundStartFailureCleansOnlyOpenedDevices)
{
  ProductionWorkflow workflow;
  auto result = workflow.begin("session-1");
  result = succeed(workflow, result);
  result = succeed(workflow, result);
  result = succeed(workflow, result);  // context start -> background start
  std::vector<ActionOutcome> outcomes;
  for (const auto & action : result.batch.actions) {
    outcomes.push_back({action.device, action.operation, action.device != "swir", "failed"});
  }
  result = workflow.complete_batch(result.batch.id, outcomes);
  EXPECT_FALSE(result.accepted);
  ASSERT_FALSE(result.batch.actions.empty());
  EXPECT_EQ(result.batch.actions[0].device, "fx10e");
  EXPECT_EQ(result.batch.actions[0].operation, DeviceOperation::stop);
}

TEST(ProductionWorkflow, AnyMandatoryPreflightFailureLatchesFault)
{
  ProductionWorkflow workflow;
  const auto issued = workflow.begin("session-1");
  std::vector<ActionOutcome> outcomes;
  for (const auto & action : issued.batch.actions) {
    outcomes.push_back({action.device, action.operation, action.device != "thermal", "no SDK"});
  }
  EXPECT_FALSE(workflow.complete_batch(issued.batch.id, outcomes).accepted);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::fault);
}

TEST(ProductionWorkflow, TimedOutStartIsTreatedAsPossiblyOpenAndSafelyStopped)
{
  ProductionWorkflow workflow;
  auto result = succeed(workflow, workflow.begin("session"));
  result = succeed(workflow, result);  // arm -> context start
  result = succeed(workflow, result);  // context -> background start
  ASSERT_FALSE(result.batch.actions.empty());
  const auto timed_out = workflow.timeout_pending_batch(result.batch.id, "timeout");
  EXPECT_FALSE(timed_out.accepted);
  ASSERT_FALSE(timed_out.batch.actions.empty());
  EXPECT_EQ(timed_out.batch.actions.front().device, "timing");
  EXPECT_EQ(timed_out.batch.actions.front().operation,
    DeviceOperation::disarm_keep_configuration);
}

TEST(ProductionWorkflow, GlobalFaultDuringStartTreatsUnansweredDevicesAsPossiblyOpen)
{
  ProductionWorkflow workflow;
  auto result = succeed(workflow, workflow.begin("session"));
  result = succeed(workflow, result);  // context start is now outstanding
  ASSERT_FALSE(result.batch.actions.empty());
  result = workflow.report_global_fault("fatal event while replies are outstanding");
  ASSERT_FALSE(result.batch.actions.empty());
  EXPECT_EQ(result.batch.actions.front().device, "context");
  EXPECT_EQ(result.batch.actions.front().operation, DeviceOperation::stop);
}

TEST(ProductionWorkflow, TimedOutContextStartIsTreatedAsPossiblyStartedAndStopped)
{
  ProductionWorkflow workflow;
  auto result = succeed(workflow, workflow.begin("session"));
  result = succeed(workflow, result);  // arm -> context start
  ASSERT_EQ(result.batch.actions.size(), 1U);
  ASSERT_EQ(result.batch.actions.front().device, "context");
  const auto timed_out = workflow.timeout_pending_batch(result.batch.id, "timeout");
  EXPECT_FALSE(timed_out.accepted);
  ASSERT_EQ(timed_out.batch.actions.size(), 1U);
  EXPECT_EQ(timed_out.batch.actions.front().device, "context");
  EXPECT_EQ(timed_out.batch.actions.front().operation, DeviceOperation::stop);
}

TEST(ProductionWorkflow, FailedCameraStopContinuesBackgroundAndContextCleanup)
{
  ProductionWorkflow workflow;
  ASSERT_TRUE(reach_recording(workflow).accepted);
  auto result = workflow.request_stop("done");
  result = succeed(workflow, result);  // disarm -> camera stops
  ASSERT_EQ(result.batch.actions.size(), 4U);

  std::vector<ActionOutcome> camera_outcomes;
  for (const auto & action : result.batch.actions) {
    camera_outcomes.push_back(
      {action.device, action.operation, action.device != "thermal", "stop failed"});
  }
  result = workflow.complete_batch(result.batch.id, camera_outcomes);
  EXPECT_FALSE(result.accepted);
  ASSERT_EQ(result.batch.actions.size(), 3U);
  EXPECT_EQ(result.batch.actions.front().device, "gnss");

  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "context");
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::fault);
  EXPECT_FALSE(workflow.session_id().empty());
}

TEST(ProductionWorkflow, TimedOutDisarmStillAttemptsEveryStopGroup)
{
  ProductionWorkflow workflow;
  ASSERT_TRUE(reach_recording(workflow).accepted);
  auto result = workflow.request_stop("done");
  result = workflow.timeout_pending_batch(result.batch.id, "disarm timeout");
  EXPECT_FALSE(result.accepted);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "timing");
  EXPECT_EQ(result.batch.actions.front().operation, DeviceOperation::stop);

  result = succeed(workflow, result);  // confirmed full timing stop -> cameras
  ASSERT_EQ(result.batch.actions.size(), 4U);
  EXPECT_EQ(result.batch.actions.front().device, "rgb");

  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 2U);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "context");
  result = succeed(workflow, result);
  EXPECT_EQ(workflow.state(), ProductionWorkflowState::fault);
}

TEST(ProductionWorkflow, StopAtDarkPromptDoesNotIssueImpossibleKeepConfigDisarm)
{
  ProductionWorkflow workflow;
  ASSERT_TRUE(reach_dark_prompt(workflow).accepted);
  auto result = workflow.request_stop("operator stopped before dark");
  ASSERT_FALSE(result.batch.actions.empty());
  EXPECT_EQ(result.batch.actions.front().device, "fx10e");
  for (const auto & action : result.batch.actions) {
    EXPECT_EQ(action.operation, DeviceOperation::stop);
    EXPECT_NE(action.device, "timing");
  }
}

TEST(ProductionWorkflow, TimedOutArmIsTreatedAsPossiblyArmedAndDisarmedFirst)
{
  ProductionWorkflow workflow;
  auto result = reach_dark_prompt(workflow);
  result = workflow.confirm_dark_cover();
  result = succeed(workflow, result);  // dark begin -> arm next PPS
  ASSERT_EQ(result.batch.actions.front().operation, DeviceOperation::arm_next_pps);

  result = workflow.timeout_pending_batch(result.batch.id, "arm reply timeout");
  EXPECT_FALSE(result.accepted);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().device, "timing");
  EXPECT_EQ(result.batch.actions.front().operation,
    DeviceOperation::disarm_keep_configuration);
}

TEST(ProductionWorkflow, DarkCompletionBeforeArmReplyIsRemembered)
{
  ProductionWorkflow workflow;
  auto result = reach_dark_prompt(workflow);
  result = workflow.confirm_dark_cover();
  result = succeed(workflow, result);  // timing arm outstanding
  ASSERT_EQ(result.batch.actions.front().operation, DeviceOperation::arm_next_pps);
  EXPECT_TRUE(workflow.observe_dark_complete("fx10e").accepted);
  EXPECT_TRUE(workflow.observe_dark_complete("swir").accepted);
  result = succeed(workflow, result);
  ASSERT_EQ(result.batch.actions.size(), 1U);
  EXPECT_EQ(result.batch.actions.front().operation,
    DeviceOperation::disarm_keep_configuration);
}

}  // namespace
