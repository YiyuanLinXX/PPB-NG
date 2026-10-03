#include "ppbng_timing_firmware/firmware_core.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

namespace {
using namespace ppbng_timing;
using namespace ppbng_timing_firmware;

class FakeHardware final : public ITimingHardware {
 public:
  CriticalToken enter_timing_critical() noexcept override {
    ++critical_entries; ++critical_depth;
    if (critical_depth > max_critical_depth) max_critical_depth = critical_depth;
    return static_cast<CriticalToken>(critical_depth);
  }
  void exit_timing_critical(CriticalToken) noexcept override { --critical_depth; }
  void configure_pps_input_capture(CaptureEdge edge) noexcept override {
    capture_edge = edge; ++capture_configurations;
  }
  bool configure_output_compare(PhysicalOutput output, std::uint64_t tick,
                                std::uint32_t width) noexcept override {
    if (fail_schedule) return false;
    const auto index = static_cast<std::size_t>(output);
    next[index] = tick; pulse_width[index] = width; ++schedule_calls[index];
    return true;
  }
  bool prepare_pps_synchronous_output(PhysicalOutput output, std::uint32_t phase,
                                      std::uint32_t width) noexcept override {
    if (fail_schedule) return false;
    const auto index = static_cast<std::size_t>(output);
    prepared_phase[index] = phase; pulse_width[index] = width; ++prepare_calls[index];
    return true;
  }
  bool arm_output_gate_on_next_pps() noexcept override {
    if (fail_schedule) return false;
    synchronous_gate_armed = true; ++synchronous_arm_calls; return true;
  }
  std::uint32_t minimum_compare_lead_ticks() const noexcept override {
    return minimum_lead;
  }
  void cancel_output_compare(PhysicalOutput output) noexcept override {
    ++cancel_calls[static_cast<std::size_t>(output)];
  }
  void set_output_gate(bool enabled) noexcept override {
    gate = enabled; ++gate_calls;
  }
  bool gate{};
  bool fail_schedule{};
  CaptureEdge capture_edge{CaptureEdge::falling};
  int capture_configurations{};
  int gate_calls{};
  int critical_entries{};
  int critical_depth{};
  int max_critical_depth{};
  int synchronous_arm_calls{};
  bool synchronous_gate_armed{};
  std::uint32_t minimum_lead{10U};
  std::array<std::uint64_t, 3> next{};
  std::array<std::uint32_t, 3> prepared_phase{};
  std::array<std::uint32_t, 3> pulse_width{};
  std::array<std::uint64_t, 3> schedule_calls{};
  std::array<std::uint64_t, 3> prepare_calls{};
  std::array<std::uint64_t, 3> cancel_calls{};
};

ScheduleConfig schedule(std::uint32_t fx_numerator = 120U) {
  return {17U, 1'000'000U,
      {{Channel::kFx10e, true, fx_numerator, 1U, 100U, 0U},
       {Channel::kSwir, true, 100U, 1U, 100U, 0U},
       {Channel::kRgb, true, 2U, 1U, 100U, 10U},
       {Channel::kThermal, true, 2U, 1U, 100U, 10U}}};
}

Packet command(MessageType type, std::uint32_t sequence,
               std::vector<std::uint8_t> payload = {}) {
  return {kProtocolVersion, type, sequence, std::move(payload)};
}

void configure_freeze_arm(FirmwareCore& core, std::uint64_t tick = 100U) {
  EXPECT_EQ(decode_ack(core.handle_command(
      command(MessageType::kConfigureSchedule, 1U, encode(schedule())), tick).payload).state,
      ControllerState::kConfigured);
  EXPECT_EQ(decode_ack(core.handle_command(
      command(MessageType::kFreezeConfiguration, 2U), tick).payload).state,
      ControllerState::kFrozen);
  EXPECT_EQ(decode_ack(core.handle_command(
      command(MessageType::kArmNextWholeSecond, 3U, encode(ArmRequest{17U, 0U})), tick).payload).state,
      ControllerState::kWaitingForPps);
}

TEST(FirmwareCore, ResetIsInertAndZeroPolicyIsFailClosed) {
  static_assert(std::is_trivially_copyable<FirmwareEvent>::value,
                "ISR event must remain fixed and trivially copyable");
  FakeHardware hardware;
  FirmwareCore core(hardware, 9U, 1'000'000U);
  EXPECT_FALSE(hardware.gate);
  EXPECT_FALSE(core.output_gate_enabled());
  EXPECT_EQ(core.state(), ControllerState::kIdle);
  EXPECT_EQ(hardware.capture_edge, CaptureEdge::rising);

  core.handle_command(command(MessageType::kConfigureSchedule, 1U, encode(schedule())), 1U);
  core.handle_command(command(MessageType::kFreezeConfiguration, 2U), 1U);
  const auto response = core.handle_command(
      command(MessageType::kArmNextWholeSecond, 3U, encode(ArmRequest{17U, 0U})), 1U);
  EXPECT_EQ(response.type, MessageType::kError);
  EXPECT_EQ(core.state(), ControllerState::kFault);
  EXPECT_FALSE(hardware.gate);
}

TEST(FirmwareCore, CommandsAreIdempotentAndWireGoldenCompatible) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 0xAABBCCDDU, 1'000'000U, {2'000'000U, 2'000'000U});
  const std::vector<std::uint8_t> golden_get_version{
      0x50U, 0x54U, 0x01U, 0x01U, 0x00U, 0x00U, 0x78U, 0x56U,
      0x34U, 0x12U, 0x8DU, 0x8CU, 0xD6U, 0x81U};
  const auto parsed = parse_packet(golden_get_version);
  const auto response = core.handle_command(parsed, 10U);
  EXPECT_EQ(decode_version_info(response.payload).boot_id, 0xAABBCCDDU);
  EXPECT_EQ(decode_version_info(response.payload).ticks_per_second, 1'000'000U);
  EXPECT_EQ(parse_packet(serialize_packet(response)).sequence, 0x12345678U);

  FakeHardware second_hardware;
  FirmwareCore second(second_hardware, 5U, 1'000'000U, {2'000'000U, 2'000'000U});
  const auto configure = command(MessageType::kConfigureSchedule, 1U, encode(schedule()));
  const auto first = serialize_packet(second.handle_command(configure, 20U));
  const auto duplicate = serialize_packet(second.handle_command(configure, 21U));
  EXPECT_EQ(first, duplicate);
  EXPECT_EQ(second.state(), ControllerState::kConfigured);
}

TEST(FirmwareCore, ArmsOnlyOnLaterPpsAndEmitsSharedSnapshotPair) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 5U, 1'000'000U, {5'000'000U, 5'000'000U});
  configure_freeze_arm(core);
  EXPECT_FALSE(hardware.gate);
  core.on_pps_capture_isr(1'000'000U);
  EXPECT_TRUE(core.output_gate_enabled());
  EXPECT_EQ(core.state(), ControllerState::kRunning);
  const auto snapshot_index = static_cast<std::size_t>(PhysicalOutput::snapshot);
  EXPECT_EQ(hardware.prepared_phase[snapshot_index], 10U);
  core.on_output_compare_isr(PhysicalOutput::snapshot, 1'000'010U);

  FirmwareEvent event;
  ASSERT_TRUE(core.pop_event(event));  // PPS
  ASSERT_TRUE(core.pop_event(event));
  ASSERT_EQ(event.kind, EventKind::trigger);
  const auto rgb = event.trigger;
  ASSERT_TRUE(core.pop_event(event));
  const auto thermal = event.trigger;
  EXPECT_EQ(rgb.channel, Channel::kRgb);
  EXPECT_EQ(thermal.channel, Channel::kThermal);
  EXPECT_EQ(rgb.channel_sequence, thermal.channel_sequence);
  EXPECT_EQ(rgb.pps_sequence, thermal.pps_sequence);
  EXPECT_EQ(rgb.offset_ticks, thermal.offset_ticks);
}

TEST(FirmwareCore, StartsSafelyWithoutPpsAndLabelsHardwareTicksUnsynced) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 5U, 1'000'000U, {1'000U, 50U});
  core.handle_command(command(MessageType::kConfigureSchedule, 1U,
                              encode(schedule())), 100U);
  core.handle_command(command(MessageType::kFreezeConfiguration, 2U), 100U);
  const auto response = core.handle_command(command(MessageType::kStartAtCurrentTick, 3U,
      encode(StartAtCurrentTickRequest{17U})), 100U);
  ASSERT_EQ(response.type, MessageType::kAck);
  EXPECT_EQ(decode_ack(response.payload).state, ControllerState::kRunning);
  EXPECT_TRUE(core.output_gate_enabled());
  const auto fx = static_cast<std::size_t>(PhysicalOutput::fx10e);
  EXPECT_EQ(hardware.next[fx], 110U);

  core.on_output_compare_isr(PhysicalOutput::fx10e, hardware.next[fx]);
  FirmwareEvent event;
  ASSERT_TRUE(core.pop_event(event));
  ASSERT_EQ(event.kind, EventKind::trigger);
  EXPECT_EQ(event.trigger.pps_sequence, 0U);
  EXPECT_EQ(event.trigger.offset_ticks, 110U);  // absolute controller tick without PPS
  EXPECT_EQ(event.trigger.lock, TimeLock::kUnsynced);

  // The explicitly optional PPS watchdog does not stop this mode. Host liveness
  // remains mandatory and the frozen oscillator continues independently.
  core.service_watchdogs_isr(500U);
  EXPECT_EQ(core.state(), ControllerState::kRunning);
  core.on_pps_capture_isr(1'000U);
  ASSERT_TRUE(core.pop_event(event));
  ASSERT_EQ(event.kind, EventKind::pps);
  EXPECT_EQ(event.pps.pps_sequence, 1U);
  EXPECT_EQ(event.pps.utc_second, (std::numeric_limits<std::int64_t>::min)());
  core.on_output_compare_isr(PhysicalOutput::fx10e, hardware.next[fx]);
  ASSERT_TRUE(core.pop_event(event));
  ASSERT_EQ(event.kind, EventKind::trigger);
  EXPECT_EQ(event.trigger.pps_sequence, 1U);
  EXPECT_EQ(event.trigger.lock, TimeLock::kLocked);
  EXPECT_LT(event.trigger.offset_ticks, event.trigger.ticks_per_second);
}

TEST(FirmwareCore, NoPpsModeRetainsHostWatchdogAndHardGate) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 5U, 1'000'000U, {100U, 0U});
  core.handle_command(command(MessageType::kConfigureSchedule, 1U,
                              encode(schedule())), 10U);
  core.handle_command(command(MessageType::kFreezeConfiguration, 2U), 10U);
  ASSERT_EQ(core.handle_command(command(MessageType::kStartAtCurrentTick, 3U,
      encode(StartAtCurrentTickRequest{17U})), 10U).type, MessageType::kAck);
  core.service_watchdogs_isr(111U);
  EXPECT_EQ(core.state(), ControllerState::kFault);
  EXPECT_NE(core.error_flags() & kErrorHostWatchdog, 0U);
  EXPECT_FALSE(core.output_gate_enabled());
}

TEST(FirmwareCore, FractionalAccumulatorHasNoLongRunDrift) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 5U, 1'000'000U, {200'000'000U, 200'000'000U});
  configure_freeze_arm(core);
  core.on_pps_capture_isr(1'000'000U);
  FirmwareEvent ignored;
  core.pop_event(ignored);
  const auto fx = static_cast<std::size_t>(PhysicalOutput::fx10e);
  const auto start = 1'000'000U;
  std::uint64_t compare_tick = start;
  for (std::uint32_t index = 0U; index < 12'000U; ++index) {
    core.on_output_compare_isr(PhysicalOutput::fx10e, compare_tick);
    ASSERT_EQ(core.state(), ControllerState::kRunning);
    ASSERT_TRUE(core.pop_event(ignored));
    compare_tick = hardware.next[fx];
  }
  EXPECT_EQ(hardware.next[fx], start + 100'000'000U);
  EXPECT_EQ(hardware.schedule_calls[fx], 12'000U);
}

TEST(FirmwareCore, OverflowInvalidScheduleAndWatchdogsHardGateOutputs) {
  {
    FakeHardware hardware;
    FirmwareCore core(hardware, 5U, 1'000'000U, {10'000'000U, 10'000'000U});
    configure_freeze_arm(core);
    core.on_pps_capture_isr(1'000'000U);
    const auto fx = static_cast<std::size_t>(PhysicalOutput::fx10e);
    std::uint64_t compare_tick = 1'000'000U;
    for (std::size_t i = 0; i < FirmwareCore::kEventRingSlots; ++i) {
      if (core.state() == ControllerState::kFault) break;
      core.on_output_compare_isr(PhysicalOutput::fx10e, compare_tick);
      compare_tick = hardware.next[fx];
    }
    EXPECT_EQ(core.state(), ControllerState::kFault);
    EXPECT_NE(core.error_flags() & kErrorEventOverflow, 0U);
    EXPECT_FALSE(hardware.gate);
  }
  {
    FakeHardware hardware;
    FirmwareCore core(hardware, 5U, 1'000'000U, {10U, 10U});
    EXPECT_EQ(core.handle_command(
        command(MessageType::kConfigureSchedule, 1U, encode(schedule())), 1U).type,
        MessageType::kAck);
    // A malformed decoded configuration (wrong timer frequency) is safely rejected.
    auto wrong_tick = schedule(); wrong_tick.ticks_per_second = 999'999U;
    EXPECT_EQ(core.handle_command(
        command(MessageType::kConfigureSchedule, 2U, encode(wrong_tick)), 2U).type,
        MessageType::kError);
    EXPECT_EQ(core.state(), ControllerState::kFault);
    EXPECT_FALSE(hardware.gate);
  }
  {
    FakeHardware hardware;
    FirmwareCore core(hardware, 5U, 1'000'000U, {100U, 100U});
    configure_freeze_arm(core, 10U);
    core.on_pps_capture_isr(20U);
    ASSERT_TRUE(core.output_gate_enabled());
    core.service_watchdogs_isr(111U);
    EXPECT_EQ(core.state(), ControllerState::kFault);
    EXPECT_NE(core.error_flags() & kErrorHostWatchdog, 0U);
    EXPECT_FALSE(hardware.gate);
  }
  {
    FakeHardware hardware;
    FirmwareCore core(hardware, 5U, 1'000'000U, {1'000U, 100U});
    configure_freeze_arm(core, 10U);
    core.service_watchdogs_isr(111U);
    EXPECT_EQ(core.state(), ControllerState::kFault);
    EXPECT_NE(core.error_flags() & kErrorPpsWatchdog, 0U);
    EXPECT_FALSE(core.output_gate_enabled());
  }
}

TEST(FirmwareCore, BootIdChangesOnResetAndDisarmAlwaysGates) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 5U, 1'000'000U, {5'000'000U, 5'000'000U});
  configure_freeze_arm(core);
  core.on_pps_capture_isr(1'000'000U);
  ASSERT_TRUE(core.output_gate_enabled());
  const auto disarm = core.handle_command(command(MessageType::kDisarm, 4U), 1'000'001U);
  EXPECT_EQ(decode_ack(disarm.payload).state, ControllerState::kIdle);
  EXPECT_FALSE(hardware.gate);
  core.reset(6U);
  EXPECT_EQ(core.boot_id(), 6U);
  EXPECT_EQ(core.state(), ControllerState::kIdle);
  EXPECT_FALSE(hardware.gate);
}

TEST(FirmwareCore, KeepConfigDisarmReturnsFrozenAndCanRearm) {
  FakeHardware hardware;
  FirmwareCore core(hardware, 5U, 1'000'000U, {5'000'000U, 5'000'000U});
  configure_freeze_arm(core);
  core.on_pps_capture_isr(1'000'000U);
  ASSERT_EQ(core.state(), ControllerState::kRunning);
  const std::vector<std::uint8_t> golden_keep_config_disarm{
      0x50U, 0x54U, 0x01U, 0x14U, 0x00U, 0x00U, 0x04U,
      0x00U, 0x00U, 0x00U, 0x7DU, 0xF0U, 0x7BU, 0xB5U};
  const auto response = core.handle_command(
      parse_packet(golden_keep_config_disarm), 1'000'001U);
  EXPECT_EQ(response.type, MessageType::kAck);
  EXPECT_EQ(decode_ack(response.payload).state, ControllerState::kFrozen);
  EXPECT_EQ(core.status(1'000'001U).active_schedule_id, 17U);
  EXPECT_FALSE(core.output_gate_enabled());
  EXPECT_EQ(core.handle_command(command(MessageType::kArmNextWholeSecond, 5U,
      encode(ArmRequest{17U, 1U})), 1'000'002U).type, MessageType::kAck);
}

TEST(FirmwareCore, RejectsTimerWithoutEnoughCompareLeadAndSerializesAccess) {
  FakeHardware hardware;
  hardware.minimum_lead = 9'000U;  // 120 Hz interval is about 8,333 ticks.
  FirmwareCore core(hardware, 5U, 1'000'000U, {5'000'000U, 5'000'000U});
  core.handle_command(command(MessageType::kConfigureSchedule, 1U, encode(schedule())), 10U);
  core.handle_command(command(MessageType::kFreezeConfiguration, 2U), 10U);
  EXPECT_EQ(core.handle_command(command(MessageType::kArmNextWholeSecond, 3U,
      encode(ArmRequest{17U, 0U})), 10U).type, MessageType::kError);
  EXPECT_EQ(core.state(), ControllerState::kFault);
  EXPECT_FALSE(core.output_gate_enabled());
  EXPECT_GT(hardware.critical_entries, 0);
  EXPECT_EQ(hardware.max_critical_depth, 1);
  EXPECT_EQ(hardware.critical_depth, 0);
}

}  // namespace
