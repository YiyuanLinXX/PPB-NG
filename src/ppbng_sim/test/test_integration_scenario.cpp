#include "ppbng_sim/integration_scenario.hpp"

#include <algorithm>
#include <cstdint>

#include <gtest/gtest.h>

using ppbng_hsi::CameraKind;
using ppbng_hsi::HsiState;
using ppbng_orchestrator::AcquisitionState;
using ppbng_orchestrator::GlobalFault;
using ppbng_orchestrator::Health;
using ppbng_sim::IntegrationScenario;
using ppbng_sim::SimDevice;
using ppbng_timing::ControllerState;

namespace
{
void start_recording(IntegrationScenario & scenario)
{
  ASSERT_TRUE(scenario.start_and_preflight());
  ASSERT_EQ(scenario.task().state(), AcquisitionState::waiting_for_dark);
  ASSERT_TRUE(scenario.collect_dark_and_wait_for_pps());
  ASSERT_EQ(scenario.task().state(), AcquisitionState::waiting_for_pps);
  ASSERT_TRUE(scenario.start_on_next_pps());
  ASSERT_EQ(scenario.task().state(), AcquisitionState::recording);
  ASSERT_EQ(scenario.timing_state(), ControllerState::kRunning);
}

void expect_global_event_ids_contiguous(const IntegrationScenario & scenario)
{
  const auto & events = scenario.observations();
  ASSERT_FALSE(events.empty());
  for (std::size_t index = 0; index < events.size(); ++index) {
    EXPECT_EQ(events[index].event_id, index);
  }
}
}  // namespace

TEST(IntegrationScenario, CompletesFullStartDarkPpsRecordStopFinalizeWorkflow)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.run_one_second();

  EXPECT_EQ(scenario.trigger_count(SimDevice::fx10e), 120U);
  EXPECT_EQ(scenario.trigger_count(SimDevice::swir), 80U);
  EXPECT_EQ(scenario.trigger_count(SimDevice::rgb), 2U);
  EXPECT_EQ(scenario.trigger_count(SimDevice::thermal), 2U);
  EXPECT_EQ(scenario.produced_count(SimDevice::fx10e), 120U);
  EXPECT_EQ(scenario.produced_count(SimDevice::swir), 80U);
  EXPECT_EQ(scenario.produced_count(SimDevice::rgb), 2U);
  EXPECT_EQ(scenario.produced_count(SimDevice::thermal), 2U);
  expect_global_event_ids_contiguous(scenario);

  EXPECT_TRUE(scenario.normal_stop_and_finalize());
  EXPECT_EQ(scenario.task().state(), AcquisitionState::finalized);
  EXPECT_EQ(scenario.timing_state(), ControllerState::kIdle);
  EXPECT_TRUE(scenario.indices_finalized());
  EXPECT_EQ(scenario.segment_index(SimDevice::fx10e).records().size(), 1U);
  EXPECT_EQ(scenario.segment_index(SimDevice::swir).records().size(), 1U);
  EXPECT_EQ(scenario.segment_index(SimDevice::rgb).records().size(), 1U);
  EXPECT_EQ(scenario.segment_index(SimDevice::thermal).records().size(), 1U);
  EXPECT_EQ(scenario.segment_index(SimDevice::fx10e).records()[0].frame_count, 120U);
}

TEST(IntegrationScenario, DifferentHsiRatesAndTwoHertzSnapshotsRemainStableAcrossSeconds)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.run_one_second();
  scenario.run_one_second();

  EXPECT_EQ(scenario.produced_count(SimDevice::fx10e), 240U);
  EXPECT_EQ(scenario.produced_count(SimDevice::swir), 160U);
  EXPECT_EQ(scenario.produced_count(SimDevice::rgb), 4U);
  EXPECT_EQ(scenario.produced_count(SimDevice::thermal), 4U);
  EXPECT_EQ(scenario.fx10e().segment_id(), 0U);
  EXPECT_EQ(scenario.swir().segment_id(), 0U);
  EXPECT_EQ(scenario.rgb().segment_index(), 0U);
  EXPECT_EQ(scenario.thermal().segment_index(), 0U);
  expect_global_event_ids_contiguous(scenario);
}

TEST(IntegrationScenario, ThermalDropoutStartsNewSegmentWhileAllOtherDevicesContinue)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.run_one_second();
  const auto fx_before = scenario.produced_count(SimDevice::fx10e);
  const auto swir_before = scenario.produced_count(SimDevice::swir);
  const auto rgb_before = scenario.produced_count(SimDevice::rgb);

  scenario.set_thermal_disconnected(true);
  scenario.run_one_second();
  EXPECT_EQ(scenario.trigger_count(SimDevice::thermal), 4U);
  EXPECT_EQ(scenario.produced_count(SimDevice::thermal), 2U);
  EXPECT_GT(scenario.produced_count(SimDevice::fx10e), fx_before);
  EXPECT_GT(scenario.produced_count(SimDevice::swir), swir_before);
  EXPECT_GT(scenario.produced_count(SimDevice::rgb), rgb_before);

  ASSERT_TRUE(scenario.recover_thermal());
  EXPECT_EQ(scenario.thermal().segment_index(), 1U);
  scenario.run_one_second();
  EXPECT_EQ(scenario.produced_count(SimDevice::thermal), 4U);
  EXPECT_EQ(scenario.thermal().segment_index(), 1U);

  const auto recovered = std::find_if(
    scenario.observations().begin(), scenario.observations().end(), [](const auto & event) {
      return event.device == SimDevice::thermal && event.produced && event.segment_id == 1U;
    });
  ASSERT_NE(recovered, scenario.observations().end());
  EXPECT_EQ(recovered->sample_index, 1U);
  EXPECT_EQ(scenario.fx10e().segment_id(), 0U);
  EXPECT_EQ(scenario.swir().segment_id(), 0U);
  EXPECT_EQ(scenario.rgb().segment_index(), 0U);
  expect_global_event_ids_contiguous(scenario);

  ASSERT_TRUE(scenario.normal_stop_and_finalize());
  ASSERT_EQ(scenario.segment_index(SimDevice::thermal).records().size(), 2U);
  EXPECT_EQ(scenario.segment_index(SimDevice::thermal).records()[0].frame_count, 2U);
  EXPECT_EQ(scenario.segment_index(SimDevice::thermal).records()[1].frame_count, 2U);
}

TEST(IntegrationScenario, OneHsiDisconnectsAndOtherContinuesUntilRecovery)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.run_one_second();
  const auto swir_before = scenario.produced_count(SimDevice::swir);
  const auto fx_before = scenario.produced_count(SimDevice::fx10e);

  scenario.disconnect_hsi_on_next_line(CameraKind::fx10e);
  scenario.run_one_second();
  EXPECT_EQ(scenario.produced_count(SimDevice::fx10e), fx_before);
  EXPECT_EQ(scenario.produced_count(SimDevice::swir), swir_before + 80U);
  EXPECT_EQ(scenario.fx10e().state(), HsiState::recovering);
  EXPECT_EQ(scenario.swir().state(), HsiState::streaming);

  ASSERT_TRUE(scenario.recover_hsi(CameraKind::fx10e));
  EXPECT_EQ(scenario.fx10e().segment_id(), 1U);
  scenario.run_one_second();
  EXPECT_EQ(scenario.produced_count(SimDevice::fx10e), fx_before + 120U);
  EXPECT_EQ(scenario.produced_count(SimDevice::swir), swir_before + 160U);
  EXPECT_EQ(scenario.swir().segment_id(), 0U);

  ASSERT_TRUE(scenario.normal_stop_and_finalize());
  EXPECT_EQ(scenario.segment_index(SimDevice::fx10e).records().size(), 2U);
  EXPECT_EQ(scenario.segment_index(SimDevice::swir).records().size(), 1U);
}

TEST(IntegrationScenario, RtkDegradationIsRecordedWithoutStoppingAcquisition)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.set_rtk_fixed(false);
  EXPECT_EQ(scenario.task().state(), AcquisitionState::recording);
  EXPECT_EQ(scenario.task().health(), Health::degraded);
  scenario.run_one_second();

  EXPECT_TRUE(std::all_of(
    scenario.observations().begin(), scenario.observations().end(),
    [](const auto & event) {return !event.rtk_fixed;}));
  EXPECT_EQ(scenario.task().state(), AcquisitionState::recording);
  EXPECT_GT(scenario.produced_count(SimDevice::thermal), 0U);

  scenario.set_rtk_fixed(true);
  EXPECT_EQ(scenario.task().health(), Health::ok);
  scenario.run_one_second();
  EXPECT_EQ(scenario.task().state(), AcquisitionState::recording);
}

TEST(IntegrationScenario, DiskFaultTriggersGlobalControlledStop)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.run_one_second();
  scenario.report_disk_fault("simulated disk write failure");

  EXPECT_EQ(scenario.task().state(), AcquisitionState::fault);
  EXPECT_EQ(scenario.task().health(), Health::fault);
  EXPECT_EQ(scenario.task().global_fault(), GlobalFault::disk_write_failure);
  ASSERT_TRUE(scenario.controlled_stop_after_fault());
  EXPECT_EQ(scenario.task().state(), AcquisitionState::finalized);
  EXPECT_EQ(scenario.task().health(), Health::fault);
  EXPECT_EQ(scenario.timing_state(), ControllerState::kIdle);
  EXPECT_TRUE(scenario.indices_finalized());
}

TEST(IntegrationScenario, TriggerFaultTriggersGlobalControlledStop)
{
  IntegrationScenario scenario;
  start_recording(scenario);
  scenario.run_one_second();
  scenario.report_trigger_fault("simulated timing controller loss");

  EXPECT_EQ(scenario.task().state(), AcquisitionState::fault);
  EXPECT_EQ(scenario.task().global_fault(), GlobalFault::trigger_controller_failure);
  ASSERT_TRUE(scenario.controlled_stop_after_fault());
  EXPECT_EQ(scenario.task().state(), AcquisitionState::finalized);
  EXPECT_EQ(scenario.timing_state(), ControllerState::kIdle);
}

