#include "ppbng_hsi/dual_hsi_coordinator.hpp"
#include "ppbng_hsi/hsi_format.hpp"
#include "ppbng_hsi/mock_hsi_adapter.hpp"
#include "ppbng_hsi/specsensor_adapter.hpp"
#include "ppbng_hsi/production_activation_gate.hpp"

#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

using ppbng_hsi::CameraKind;
using ppbng_hsi::CaptureKind;
using ppbng_hsi::DualHsiCoordinator;
using ppbng_hsi::FailurePoint;
using ppbng_hsi::HsiConfig;
using ppbng_hsi::HsiFormat;
using ppbng_hsi::HsiState;
using ppbng_hsi::LineStatus;
using ppbng_hsi::MockHsiAdapter;
using ppbng_hsi::TimeStatus;
using ppbng_hsi::TriggerEvent;

namespace
{
HsiConfig fx_config()
{
  HsiConfig value;
  value.kind = CameraKind::fx10e;
  value.device_id = "fx10e-serial";
  value.trigger_channel = "fx10e_line";
  value.spatial_samples = 8;
  value.spectral_bands = 4;
  value.line_rate_hz = 120.0;
  value.exposure_us = 2500.0;
  return value;
}

HsiConfig swir_config()
{
  HsiConfig value;
  value.kind = CameraKind::swir;
  value.device_id = "swir-serial";
  value.trigger_channel = "swir_line";
  value.spatial_samples = 6;
  value.spectral_bands = 3;
  value.line_rate_hz = 100.0;
  value.exposure_us = 4000.0;
  return value;
}

TriggerEvent trigger(
  const std::string & channel,
  const std::uint64_t sequence,
  const std::int64_t utc_ns = 1'700'000'000'000'000'000LL)
{
  return {channel, sequence, 42, 24'000'000, 48'000'000, utc_ns,
    TimeStatus::locked, 100};
}

void prepare_ready(MockHsiAdapter & adapter, const HsiConfig & config)
{
  ASSERT_TRUE(adapter.connect().success);
  ASSERT_TRUE(adapter.configure(config).success);
  ASSERT_TRUE(adapter.close_shutter().success);
  ASSERT_TRUE(adapter.begin_dark_capture(1).success);
  ASSERT_EQ(adapter.on_trigger(trigger(config.trigger_channel, 1)).status, LineStatus::produced);
  ASSERT_EQ(adapter.state(), HsiState::shutter_closed);
  ASSERT_TRUE(adapter.open_shutter().success);
  ASSERT_EQ(adapter.state(), HsiState::ready);
}

void prepare_both_streaming(
  MockHsiAdapter & fx, MockHsiAdapter & swir, DualHsiCoordinator & coordinator)
{
  ASSERT_TRUE(coordinator.connect_both().both_succeeded());
  ASSERT_TRUE(coordinator.configure_both(fx_config(), swir_config()).both_succeeded());
  ASSERT_TRUE(coordinator.close_both_shutters().both_succeeded());
  ASSERT_TRUE(coordinator.begin_dark_both(1, 1).both_succeeded());
  ASSERT_EQ(coordinator.dispatch(CameraKind::fx10e, trigger("fx10e_line", 1)).status,
    LineStatus::produced);
  ASSERT_EQ(coordinator.dispatch(CameraKind::swir, trigger("swir_line", 1)).status,
    LineStatus::produced);
  ASSERT_TRUE(coordinator.open_both_shutters().both_succeeded());
  ASSERT_TRUE(coordinator.start_both().both_succeeded());
  ASSERT_EQ(fx.state(), HsiState::streaming);
  ASSERT_EQ(swir.state(), HsiState::streaming);
}
}  // namespace

TEST(HsiFormat, ValidatesIndependentCameraConfigurations)
{
  EXPECT_TRUE(HsiFormat::validate_config(fx_config()).success);
  EXPECT_TRUE(HsiFormat::validate_config(swir_config()).success);

  auto invalid = fx_config();
  invalid.line_rate_hz = 0.0;
  EXPECT_FALSE(HsiFormat::validate_config(invalid).success);
  invalid = fx_config();
  invalid.trigger_channel.clear();
  EXPECT_FALSE(HsiFormat::validate_config(invalid).success);
  invalid = fx_config();
  invalid.spectral_bands = 0;
  EXPECT_FALSE(HsiFormat::validate_config(invalid).success);
}

TEST(HsiInternalTiming, ContinuousFramesDoNotRequireExternalTriggerEvents)
{
  auto config = fx_config();
  config.trigger_mode = "Internal";
  config.trigger_channel.clear();
  ASSERT_TRUE(HsiFormat::validate_config(config).success);

  MockHsiAdapter camera(CameraKind::fx10e);
  ASSERT_TRUE(camera.connect().success);
  ASSERT_TRUE(camera.configure(config).success);
  ASSERT_TRUE(camera.close_shutter().success);
  ASSERT_TRUE(camera.begin_dark_capture(2).success);
  EXPECT_EQ(camera.poll_internal().status, LineStatus::produced);
  EXPECT_EQ(camera.poll_internal().status, LineStatus::produced);
  EXPECT_EQ(camera.state(), HsiState::shutter_closed);
  ASSERT_TRUE(camera.open_shutter().success);
  ASSERT_TRUE(camera.start_streaming().success);
  const auto sample = camera.poll_internal();
  ASSERT_EQ(sample.status, LineStatus::produced);
  EXPECT_EQ(sample.line->index.capture_kind, CaptureKind::sample);
  EXPECT_EQ(sample.line->index.trigger_sequence, 0U);
  EXPECT_EQ(sample.line->index.time_status, TimeStatus::unsynced);
}

TEST(HsiFormat, DescribesEnviBilRawAndSidecars)
{
  const auto layout = HsiFormat::make_envi_layout(fx_config(), 7, 1234, "fx10e");
  EXPECT_EQ(layout.samples, 8U);
  EXPECT_EQ(layout.lines, 1234U);
  EXPECT_EQ(layout.bands, 4U);
  EXPECT_EQ(layout.data_type, 12U);
  EXPECT_EQ(layout.interleave, "bil");
  EXPECT_EQ(layout.byte_order, 0U);
  EXPECT_EQ(layout.bytes_per_line, 64U);
  EXPECT_EQ(layout.raw_filename, "fx10e_segment_7.raw");
  EXPECT_EQ(layout.header_filename, "fx10e_segment_7.hdr");
  EXPECT_EQ(layout.timestamp_filename, "fx10e_segment_7.timestamps.bin");
  EXPECT_EQ(layout.index_filename, "fx10e_segment_7.index.bin");

  const auto header = HsiFormat::render_envi_header(layout);
  EXPECT_NE(header.find("samples = 8"), std::string::npos);
  EXPECT_NE(header.find("lines = 1234"), std::string::npos);
  EXPECT_NE(header.find("bands = 4"), std::string::npos);
  EXPECT_NE(header.find("data type = 12"), std::string::npos);
  EXPECT_NE(header.find("interleave = bil"), std::string::npos);
}

TEST(HsiFormat, ZeroDimensionsHaveNoPayload)
{
  HsiConfig config;
  EXPECT_EQ(HsiFormat::payload_bytes_per_line(config), 0U);
}

TEST(MockHsiAdapter, EnforcesGuidedDarkWorkflow)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  ASSERT_TRUE(fx.connect().success);
  ASSERT_TRUE(fx.configure(fx_config()).success);
  EXPECT_FALSE(fx.start_streaming().success);
  ASSERT_TRUE(fx.close_shutter().success);
  ASSERT_TRUE(fx.begin_dark_capture(2).success);
  EXPECT_EQ(fx.state(), HsiState::dark_collecting);

  const auto dark0 = fx.on_trigger(trigger("fx10e_line", 10));
  ASSERT_EQ(dark0.status, LineStatus::produced);
  ASSERT_TRUE(dark0.line.has_value());
  EXPECT_EQ(dark0.line->index.capture_kind, CaptureKind::dark);
  EXPECT_EQ(dark0.line->index.segment_line_index, 0U);
  EXPECT_EQ(fx.state(), HsiState::dark_collecting);

  const auto dark1 = fx.on_trigger(trigger("fx10e_line", 11));
  ASSERT_EQ(dark1.status, LineStatus::produced);
  EXPECT_EQ(fx.state(), HsiState::shutter_closed);
  EXPECT_FALSE(fx.start_streaming().success);
  EXPECT_TRUE(fx.open_shutter().success);
  EXPECT_TRUE(fx.start_streaming().success);
}

TEST(MockHsiAdapter, LineIndexPreservesTriggerTimingAndRawOffset)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  prepare_ready(fx, fx_config());
  ASSERT_TRUE(fx.start_streaming().success);

  const auto first = fx.on_trigger(trigger("fx10e_line", 2, 123456789));
  ASSERT_EQ(first.status, LineStatus::produced);
  ASSERT_TRUE(first.line.has_value());
  EXPECT_EQ(first.line->index.segment_id, 0U);
  EXPECT_EQ(first.line->index.segment_line_index, 0U);
  EXPECT_EQ(first.line->index.trigger_sequence, 2U);
  EXPECT_EQ(first.line->index.pps_sequence, 42U);
  EXPECT_EQ(first.line->index.utc_time_ns, 123456789);
  EXPECT_EQ(first.line->index.time_status, TimeStatus::locked);
  EXPECT_EQ(first.line->index.uncertainty_ns, 100U);
  EXPECT_EQ(first.line->index.raw_file_offset_bytes, 0U);
  EXPECT_EQ(first.line->index.payload_size_bytes, 64U);
  EXPECT_EQ(first.line->pixels.size(), 32U);

  const auto second = fx.on_trigger(trigger("fx10e_line", 3));
  ASSERT_EQ(second.status, LineStatus::produced);
  EXPECT_EQ(second.line->index.raw_file_offset_bytes, 64U);
}

TEST(MockHsiAdapter, RejectsWrongDuplicateAndOutOfOrderTriggers)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  prepare_ready(fx, fx_config());
  ASSERT_TRUE(fx.start_streaming().success);

  EXPECT_EQ(fx.on_trigger(trigger("wrong", 2)).status, LineStatus::invalid_trigger);
  EXPECT_EQ(fx.on_trigger(trigger("fx10e_line", 2)).status, LineStatus::produced);
  EXPECT_EQ(fx.on_trigger(trigger("fx10e_line", 2)).status, LineStatus::invalid_trigger);
  EXPECT_EQ(fx.on_trigger(trigger("fx10e_line", 1)).status, LineStatus::invalid_trigger);

  auto invalid_ticks = trigger("fx10e_line", 3);
  invalid_ticks.offset_ticks = invalid_ticks.ticks_per_second;
  EXPECT_EQ(fx.on_trigger(invalid_ticks).status, LineStatus::invalid_trigger);
}

TEST(MockHsiAdapter, AcceptsAbsoluteControllerTickWhenPpsIsUnsynced)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  prepare_ready(fx, fx_config());
  ASSERT_TRUE(fx.start_streaming().success);
  auto no_pps = trigger("fx10e_line", 2);
  no_pps.pps_sequence = 0U;
  no_pps.offset_ticks = no_pps.ticks_per_second * 7U + 123U;
  no_pps.utc_time_ns = 0;
  no_pps.time_status = TimeStatus::unsynced;
  const auto result = fx.on_trigger(no_pps);
  ASSERT_EQ(result.status, LineStatus::produced);
  ASSERT_TRUE(result.line);
  EXPECT_EQ(result.line->trigger.offset_ticks, no_pps.offset_ticks);
  EXPECT_EQ(result.line->index.time_status, TimeStatus::unsynced);
}

TEST(MockHsiAdapter, RecordsTriggerSequenceGapsWithoutDroppingTheLaterLine)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  prepare_ready(fx, fx_config());
  ASSERT_TRUE(fx.start_streaming().success);

  const auto line = fx.on_trigger(trigger("fx10e_line", 5));
  ASSERT_EQ(line.status, LineStatus::produced);
  EXPECT_TRUE(line.line->index.sequence_gap_before);
  EXPECT_EQ(line.line->index.missing_trigger_count, 3U);
}

TEST(DualHsiCoordinator, KeepsIndependentRatesConfigurationsAndContexts)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  MockHsiAdapter swir(CameraKind::swir);
  DualHsiCoordinator coordinator(fx, swir);
  ASSERT_TRUE(coordinator.connect_both().both_succeeded());
  ASSERT_TRUE(coordinator.configure_both(fx_config(), swir_config()).both_succeeded());

  EXPECT_DOUBLE_EQ(fx.config().line_rate_hz, 120.0);
  EXPECT_DOUBLE_EQ(swir.config().line_rate_hz, 100.0);
  EXPECT_EQ(fx.config().spatial_samples, 8U);
  EXPECT_EQ(swir.config().spatial_samples, 6U);
  EXPECT_NE(fx.config().trigger_channel, swir.config().trigger_channel);
}

TEST(DualHsiCoordinator, BothCamerasContinuouslyProduceIndependentLines)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  MockHsiAdapter swir(CameraKind::swir);
  DualHsiCoordinator coordinator(fx, swir);
  prepare_both_streaming(fx, swir, coordinator);

  for (std::uint64_t sequence = 2; sequence < 12; ++sequence) {
    const auto fx_line = coordinator.dispatch(
      CameraKind::fx10e, trigger("fx10e_line", sequence));
    const auto swir_line = coordinator.dispatch(
      CameraKind::swir, trigger("swir_line", sequence));
    ASSERT_EQ(fx_line.status, LineStatus::produced);
    ASSERT_EQ(swir_line.status, LineStatus::produced);
    EXPECT_EQ(fx_line.line->index.segment_line_index, sequence - 2);
    EXPECT_EQ(swir_line.line->index.segment_line_index, sequence - 2);
  }
  EXPECT_EQ(fx.state(), HsiState::streaming);
  EXPECT_EQ(swir.state(), HsiState::streaming);
}

TEST(DualHsiCoordinator, OneDisconnectsWhileOtherContinuesAndRecoveryStartsNewSegment)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  MockHsiAdapter swir(CameraKind::swir);
  DualHsiCoordinator coordinator(fx, swir);
  prepare_both_streaming(fx, swir, coordinator);

  fx.disconnect_on_next_line();
  EXPECT_EQ(coordinator.dispatch(CameraKind::fx10e, trigger("fx10e_line", 2)).status,
    LineStatus::disconnected);
  EXPECT_EQ(fx.state(), HsiState::recovering);

  const auto swir_line2 = coordinator.dispatch(
    CameraKind::swir, trigger("swir_line", 2));
  const auto swir_line3 = coordinator.dispatch(
    CameraKind::swir, trigger("swir_line", 3));
  EXPECT_EQ(swir_line2.status, LineStatus::produced);
  EXPECT_EQ(swir_line3.status, LineStatus::produced);
  EXPECT_EQ(swir.state(), HsiState::streaming);
  EXPECT_EQ(swir.segment_id(), 0U);

  ASSERT_TRUE(fx.recover().success);
  ASSERT_EQ(fx.state(), HsiState::ready);
  ASSERT_TRUE(fx.start_streaming().success);
  EXPECT_EQ(fx.segment_id(), 1U);
  const auto recovered = coordinator.dispatch(
    CameraKind::fx10e, trigger("fx10e_line", 3));
  ASSERT_EQ(recovered.status, LineStatus::produced);
  EXPECT_EQ(recovered.line->index.segment_id, 1U);
  EXPECT_EQ(recovered.line->index.segment_line_index, 0U);
  EXPECT_EQ(swir.segment_id(), 0U);
}

TEST(DualHsiCoordinator, ConfigurationFailureIsIsolated)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  MockHsiAdapter swir(CameraKind::swir);
  DualHsiCoordinator coordinator(fx, swir);
  ASSERT_TRUE(coordinator.connect_both().both_succeeded());
  fx.fail_next(FailurePoint::configure);

  const auto result = coordinator.configure_both(fx_config(), swir_config());
  EXPECT_FALSE(result.fx10e.success);
  EXPECT_TRUE(result.swir.success);
  EXPECT_EQ(fx.state(), HsiState::fault);
  EXPECT_EQ(swir.state(), HsiState::configured);
}

TEST(DualHsiCoordinator, ShutterFailureDoesNotPreventOtherCameraDarkWorkflow)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  MockHsiAdapter swir(CameraKind::swir);
  DualHsiCoordinator coordinator(fx, swir);
  ASSERT_TRUE(coordinator.connect_both().both_succeeded());
  ASSERT_TRUE(coordinator.configure_both(fx_config(), swir_config()).both_succeeded());
  fx.fail_next(FailurePoint::close_shutter);

  const auto close = coordinator.close_both_shutters();
  EXPECT_FALSE(close.fx10e.success);
  EXPECT_TRUE(close.swir.success);
  EXPECT_EQ(swir.state(), HsiState::shutter_closed);
  EXPECT_TRUE(swir.begin_dark_capture(1).success);
  EXPECT_EQ(swir.on_trigger(trigger("swir_line", 1)).status, LineStatus::produced);
}

TEST(MockHsiAdapter, AcquisitionFaultIsInjectable)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  prepare_ready(fx, fx_config());
  ASSERT_TRUE(fx.start_streaming().success);
  fx.fail_next(FailurePoint::acquire_line);

  const auto failed = fx.on_trigger(trigger("fx10e_line", 2));
  EXPECT_EQ(failed.status, LineStatus::fault);
  EXPECT_FALSE(failed.line.has_value());
  EXPECT_EQ(fx.state(), HsiState::fault);
}

TEST(MockHsiAdapter, RecoveryFailureIsInjectableAndDoesNotChangeOtherContext)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  prepare_ready(fx, fx_config());
  ASSERT_TRUE(fx.start_streaming().success);
  fx.disconnect_on_next_line();
  ASSERT_EQ(fx.on_trigger(trigger("fx10e_line", 2)).status, LineStatus::disconnected);
  fx.fail_next(FailurePoint::recover);

  EXPECT_FALSE(fx.recover().success);
  EXPECT_EQ(fx.state(), HsiState::fault);
}

TEST(DualHsiCoordinator, RejectsReversedAdapterKinds)
{
  MockHsiAdapter fx(CameraKind::fx10e);
  MockHsiAdapter swir(CameraKind::swir);
  EXPECT_THROW(DualHsiCoordinator coordinator(swir, fx), std::invalid_argument);
}

TEST(SpecSensorContract, HardwareBackendMatchesBuildConfiguration)
{
#if PPBNG_HSI_ENABLE_SPECSENSOR
  EXPECT_TRUE(ppbng_hsi::specsensor_backend_compiled());
#else
  EXPECT_FALSE(ppbng_hsi::specsensor_backend_compiled());
  ppbng_hsi::SpecSensorBackendOptions options;
  ppbng_hsi::SpecSensorHsiAdapter adapter(options);
  EXPECT_FALSE(adapter.connect().success);
  EXPECT_EQ(adapter.state(), HsiState::disconnected);
#endif
}

TEST(SpecSensorContract, RequiresExplicitTransportIdentityAndBounds)
{
  ppbng_hsi::SpecSensorBackendOptions fx;
  fx.kind = CameraKind::fx10e;
  fx.transport = ppbng_hsi::SpecSensorTransport::pleora_gige;
  fx.device_index = 0;
  fx.expected_profile_name = "FX10e with Pleora";
  fx.expected_sensor_serial = "verified-fx10e-serial";
  fx.calibration_pack_path = L"C:/local/fx10e.scp";
  fx.grabber_channel = L"192.168.10.2";
  fx.pleora_packet_size = 8228;
  EXPECT_TRUE(ppbng_hsi::validate_specsensor_options(fx).success);

  auto invalid = fx;
  invalid.transport = ppbng_hsi::SpecSensorTransport::ni_camera_link;
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);
  invalid = fx;
  invalid.device_index = -1;
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);
  invalid = fx;
  invalid.grabber_channel = L"ui";
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);
  invalid = fx;
  invalid.callback_queue_capacity = 0;
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);
  invalid = fx;
  invalid.callback_queue_capacity = 513;
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);
  invalid = fx;
  invalid.callback_queue_capacity = 512;
  invalid.maximum_frame_bytes = 3U * 1024U * 1024U;
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);
  invalid = fx;
  invalid.maximum_frame_bytes = 65U * 1024U * 1024U;
  EXPECT_FALSE(ppbng_hsi::validate_specsensor_options(invalid).success);

  ppbng_hsi::SpecSensorBackendOptions swir = fx;
  swir.kind = CameraKind::swir;
  swir.transport = ppbng_hsi::SpecSensorTransport::ni_camera_link;
  swir.expected_profile_name = "SWIR3 with NI";
  swir.pleora_packet_size = 0;
  swir.ni_grabber_channel = L"img0";
  swir.ni_camera_file_path = L"C:/local/Specim_SWIR3.icd";
  EXPECT_TRUE(ppbng_hsi::validate_specsensor_options(swir).success);
}

TEST(SpecSensorFrameQueue, CallbackPathUsesFixedCapacityAndRejectsOverflow)
{
  ppbng_hsi::BoundedSpecSensorFrameQueue queue(2, 8);
  EXPECT_EQ(queue.capacity(), 2U);
  const std::uint8_t first[] = {1, 2, 3, 4};
  const std::uint8_t second[] = {5, 6, 7, 8};
  EXPECT_TRUE(queue.try_push(first, sizeof(first), 10, 1001U));
  EXPECT_TRUE(queue.try_push(second, sizeof(second), 11, 1002U));
  EXPECT_FALSE(queue.try_push(first, sizeof(first), 12, 1003U));
  EXPECT_EQ(queue.overflow_count(), 1U);

  ppbng_hsi::SpecSensorFrame frame;
  ASSERT_TRUE(queue.try_pop(frame));
  EXPECT_EQ(frame.sdk_frame_number, 10);
  EXPECT_EQ(frame.host_receive_monotonic_ns, 1001U);
  EXPECT_EQ(frame.bytes, std::vector<std::uint8_t>({1, 2, 3, 4}));
  ASSERT_TRUE(queue.try_pop(frame));
  EXPECT_EQ(frame.sdk_frame_number, 11);
  EXPECT_EQ(frame.host_receive_monotonic_ns, 1002U);
  EXPECT_FALSE(queue.try_pop(frame));
}

TEST(SpecSensorFrameQueue, RejectsInvalidCallbackPayloadWithoutAllocationGrowth)
{
  ppbng_hsi::BoundedSpecSensorFrameQueue queue(1, 4);
  const std::uint8_t bytes[] = {1, 2, 3, 4, 5};
  EXPECT_FALSE(queue.try_push(nullptr, 1, 1, 1U));
  EXPECT_FALSE(queue.try_push(bytes, 0, 1, 1U));
  EXPECT_FALSE(queue.try_push(bytes, sizeof(bytes), 1, 1U));
  EXPECT_FALSE(queue.try_push(bytes, 4, -1, 1U));
  EXPECT_EQ(queue.invalid_frame_count(), 4U);
  EXPECT_EQ(queue.size(), 0U);
}

TEST(HsiProductionGate, CannotStartWithoutExplicitHardwareEnableAndArm)
{
  ppbng_hsi::ProductionActivationGate disabled(false);
  EXPECT_FALSE(disabled.arm(true).accepted);
  EXPECT_FALSE(disabled.start().accepted);

  ppbng_hsi::ProductionActivationGate enabled(true);
  EXPECT_FALSE(enabled.start().accepted);
  EXPECT_FALSE(enabled.arm(false).accepted);
  EXPECT_TRUE(enabled.arm(true).accepted);
  EXPECT_EQ(enabled.state(), ppbng_hsi::ProductionGateState::armed);
  EXPECT_TRUE(enabled.start().accepted);
  EXPECT_EQ(enabled.state(), ppbng_hsi::ProductionGateState::started);
}

TEST(HsiProductionGate, FailedStartLatchesFaultUntilExplicitStop)
{
  ppbng_hsi::ProductionActivationGate gate(true);
  ASSERT_TRUE(gate.arm(true).accepted);
  ASSERT_TRUE(gate.start().accepted);
  gate.start_failed();
  EXPECT_EQ(gate.state(), ppbng_hsi::ProductionGateState::fault);
  EXPECT_FALSE(gate.start().accepted);
  gate.stop();
  EXPECT_EQ(gate.state(), ppbng_hsi::ProductionGateState::inert);
}
