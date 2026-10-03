#include "ppbng_timing/controller_model.hpp"
#include "ppbng_timing/protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

ppbng_timing::ScheduleConfig valid_schedule() {
  using ppbng_timing::Channel;
  return ppbng_timing::ScheduleConfig{
      17U,
      1'000'000U,
      {
          {Channel::kFx10e, true, 120U, 1U, 1000U, 0U},
          {Channel::kSwir, true, 100U, 1U, 1000U, 0U},
          {Channel::kRgb, true, 2U, 1U, 1000U, 0U},
          {Channel::kThermal, true, 2U, 1U, 1000U, 0U},
      }};
}

TEST(TimingPacket, RoundTripsPacketAndUsesKnownIeeeCrcVector) {
  const std::string canonical = "123456789";
  EXPECT_EQ(
      ppbng_timing::crc32_ieee(
          reinterpret_cast<const std::uint8_t*>(canonical.data()), canonical.size()),
      0xCBF43926U);

  ppbng_timing::Packet packet;
  packet.type = ppbng_timing::MessageType::kAck;
  packet.sequence = 0x12345678U;
  packet.payload = {1U, 2U, 3U, 4U};
  const auto decoded = ppbng_timing::parse_packet(ppbng_timing::serialize_packet(packet));
  EXPECT_EQ(decoded.protocol_version, ppbng_timing::kProtocolVersion);
  EXPECT_EQ(decoded.type, ppbng_timing::MessageType::kAck);
  EXPECT_EQ(decoded.sequence, 0x12345678U);
  EXPECT_EQ(decoded.payload, packet.payload);
}

TEST(TimingPacket, RejectsBadCrcTruncationAndUnsupportedVersion) {
  ppbng_timing::Packet packet;
  packet.type = ppbng_timing::MessageType::kStatus;
  auto bytes = ppbng_timing::serialize_packet(packet);
  bytes.back() ^= 0x80U;
  EXPECT_THROW(ppbng_timing::parse_packet(bytes), std::invalid_argument);
  bytes.pop_back();
  EXPECT_THROW(ppbng_timing::parse_packet(bytes), std::invalid_argument);

  packet.protocol_version = 2U;
  EXPECT_THROW(ppbng_timing::serialize_packet(packet), std::invalid_argument);

  packet.protocol_version = ppbng_timing::kProtocolVersion;
  packet.type = static_cast<ppbng_timing::MessageType>(0xFFU);
  EXPECT_THROW(ppbng_timing::serialize_packet(packet), std::invalid_argument);
}

TEST(TimingStream, HandlesNoiseFragmentsAndCoalescedPackets) {
  ppbng_timing::Packet first;
  first.type = ppbng_timing::MessageType::kGetVersion;
  first.sequence = 1U;
  ppbng_timing::Packet second;
  second.type = ppbng_timing::MessageType::kStatus;
  second.sequence = 2U;
  const auto a = ppbng_timing::serialize_packet(first);
  const auto b = ppbng_timing::serialize_packet(second);

  ppbng_timing::StreamDecoder decoder;
  std::vector<std::uint8_t> prefix{0x00U, 0x01U, a[0], a[1], a[2]};
  EXPECT_TRUE(decoder.feed(prefix).empty());
  std::vector<std::uint8_t> rest(a.begin() + 3, a.end());
  rest.insert(rest.end(), b.begin(), b.end());
  const auto packets = decoder.feed(rest);
  ASSERT_EQ(packets.size(), 2U);
  EXPECT_EQ(packets[0].sequence, 1U);
  EXPECT_EQ(packets[1].sequence, 2U);
  EXPECT_EQ(decoder.discarded_noise_bytes(), 2U);
}

TEST(TimingStream, RecoversAfterBadCrcPacket) {
  ppbng_timing::Packet bad;
  bad.type = ppbng_timing::MessageType::kAck;
  bad.sequence = 8U;
  auto bad_bytes = ppbng_timing::serialize_packet(bad);
  bad_bytes.back() ^= 1U;
  ppbng_timing::Packet good;
  good.type = ppbng_timing::MessageType::kStatus;
  good.sequence = 9U;
  const auto good_bytes = ppbng_timing::serialize_packet(good);
  bad_bytes.insert(bad_bytes.end(), good_bytes.begin(), good_bytes.end());

  ppbng_timing::StreamDecoder decoder;
  const auto packets = decoder.feed(bad_bytes);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].sequence, 9U);
  EXPECT_EQ(decoder.crc_or_format_errors(), 1U);
}

TEST(TimingSchedule, RoundTripsIndependentChannelRates) {
  const auto schedule = valid_schedule();
  const auto decoded = ppbng_timing::decode_schedule_config(ppbng_timing::encode(schedule));
  ASSERT_EQ(decoded.channels.size(), 4U);
  EXPECT_EQ(decoded.channels[0].rate_numerator_hz, 120U);
  EXPECT_EQ(decoded.channels[1].rate_numerator_hz, 100U);
  EXPECT_EQ(decoded.channels[2].channel, ppbng_timing::Channel::kRgb);
  EXPECT_EQ(decoded.channels[3].channel, ppbng_timing::Channel::kThermal);
}

TEST(TimingSchedule, RejectsDuplicateMissingAndImpossibleChannels) {
  auto duplicate = valid_schedule();
  duplicate.channels[3].channel = ppbng_timing::Channel::kRgb;
  EXPECT_THROW(ppbng_timing::validate_schedule(duplicate), std::invalid_argument);

  auto missing = valid_schedule();
  missing.channels.pop_back();
  EXPECT_THROW(ppbng_timing::validate_schedule(missing), std::invalid_argument);

  auto impossible = valid_schedule();
  impossible.channels[0].pulse_width_ticks = 10'000U;
  EXPECT_THROW(ppbng_timing::validate_schedule(impossible), std::invalid_argument);
}

TEST(TimingSchedule, EnforcesSharedRgbThermalPhysicalEdge) {
  auto mismatched_rate = valid_schedule();
  mismatched_rate.channels[3].rate_numerator_hz = 3U;
  EXPECT_THROW(ppbng_timing::validate_schedule(mismatched_rate), std::invalid_argument);

  auto mismatched_phase = valid_schedule();
  mismatched_phase.channels[3].phase_ticks = 1U;
  EXPECT_THROW(ppbng_timing::validate_schedule(mismatched_phase), std::invalid_argument);

  auto mismatched_enable = valid_schedule();
  mismatched_enable.channels[3].enabled = false;
  mismatched_enable.channels[3].rate_numerator_hz = 0U;
  EXPECT_THROW(ppbng_timing::validate_schedule(mismatched_enable), std::invalid_argument);
}

TEST(TimingPayloads, RoundTripsVersionEventsAckErrorAndStatus) {
  const ppbng_timing::VersionInfo version{1U, 2U, 3U, 77U, 0x55U, 1'000'000U};
  const auto version2 = ppbng_timing::decode_version_info(ppbng_timing::encode(version));
  EXPECT_EQ(version2.boot_id, 77U);
  EXPECT_EQ(version2.firmware_patch, 3U);

  const ppbng_timing::PpsAnchor anchor{
      77U, 100U, 1'787'670'000, 50'000'000U, ppbng_timing::TimeLock::kLocked};
  const auto anchor2 = ppbng_timing::decode_pps_anchor(ppbng_timing::encode(anchor));
  EXPECT_EQ(anchor2.utc_second, anchor.utc_second);
  EXPECT_EQ(anchor2.captured_tick, anchor.captured_tick);

  const ppbng_timing::TriggerEvent event{
      77U, 900U, ppbng_timing::Channel::kThermal, 44U, 100U, 500'000U,
      1'000'000U, ppbng_timing::TimeLock::kLocked};
  const auto event2 = ppbng_timing::decode_trigger_event(ppbng_timing::encode(event));
  EXPECT_EQ(event2.event_sequence, 900U);
  EXPECT_EQ(event2.channel_sequence, 44U);

  const ppbng_timing::Ack ack{12U, 0U, ppbng_timing::ControllerState::kFrozen, "frozen"};
  EXPECT_EQ(ppbng_timing::decode_ack(ppbng_timing::encode(ack)).detail, "frozen");
  const ppbng_timing::ErrorReport error{12U, 42U, 2U, "bad schedule"};
  EXPECT_EQ(ppbng_timing::decode_error_report(ppbng_timing::encode(error)).code, 42U);
  const ppbng_timing::StatusReport status{
      77U, ppbng_timing::ControllerState::kRunning, 17U, 22U, 100U,
      50'500'000U, ppbng_timing::TimeLock::kLocked, 0U, 0U};
  EXPECT_EQ(
      ppbng_timing::decode_status_report(ppbng_timing::encode(status)).active_schedule_id,
      17U);
}

TEST(TimingPayloads, RejectsTrailingAndTruncatedPayloads) {
  auto encoded = ppbng_timing::encode(ppbng_timing::ArmRequest{17U, 10U});
  encoded.push_back(0U);
  EXPECT_THROW(ppbng_timing::decode_arm_request(encoded), std::invalid_argument);
  encoded.clear();
  EXPECT_THROW(ppbng_timing::decode_arm_request(encoded), std::invalid_argument);
}

TEST(TimingPayloads, RoundTripsExplicitStartAtCurrentTickRequest) {
  const ppbng_timing::StartAtCurrentTickRequest request{17U};
  const auto decoded = ppbng_timing::decode_start_at_current_tick_request(
      ppbng_timing::encode(request));
  EXPECT_EQ(decoded.schedule_id, 17U);
  auto trailing = ppbng_timing::encode(request);
  trailing.push_back(0U);
  EXPECT_THROW(ppbng_timing::decode_start_at_current_tick_request(trailing),
               std::invalid_argument);
}

TEST(TimingSequence, DetectsGapRegressionDuplicateAndControllerRestart) {
  ppbng_timing::EventSequenceTracker tracker;
  EXPECT_EQ(tracker.observe(1U, 10U), ppbng_timing::SequenceObservation::kFirst);
  EXPECT_EQ(tracker.observe(1U, 11U), ppbng_timing::SequenceObservation::kInOrder);
  EXPECT_EQ(tracker.observe(1U, 13U), ppbng_timing::SequenceObservation::kGap);
  EXPECT_EQ(tracker.observe(1U, 13U), ppbng_timing::SequenceObservation::kDuplicate);
  EXPECT_EQ(tracker.observe(1U, 2U), ppbng_timing::SequenceObservation::kRegression);
  EXPECT_EQ(
      tracker.observe(2U, 0U),
      ppbng_timing::SequenceObservation::kControllerRestart);
}

TEST(TimingModel, FreezesRatesUntilDisarmAndArmsAtNextPps) {
  ppbng_timing::ControllerModel controller(99U, 1'000'000U);
  const auto schedule = valid_schedule();
  controller.configure(schedule);
  controller.freeze(schedule.schedule_id);
  EXPECT_THROW(controller.configure(schedule), std::logic_error);
  controller.arm_next_whole_second({schedule.schedule_id, 10U});
  EXPECT_EQ(controller.status().state, ppbng_timing::ControllerState::kWaitingForPps);
  controller.observe_pps(10U, 1000, 10'000'000U);
  EXPECT_EQ(controller.status().state, ppbng_timing::ControllerState::kWaitingForPps);
  controller.observe_pps(11U, 1001, 11'000'000U);
  EXPECT_EQ(controller.status().state, ppbng_timing::ControllerState::kRunning);
  const auto first_event = controller.emit_trigger(ppbng_timing::Channel::kRgb, 0U);
  controller.disarm();
  controller.configure(schedule);
  EXPECT_EQ(controller.status().state, ppbng_timing::ControllerState::kConfigured);
  controller.freeze(schedule.schedule_id);
  controller.arm_next_whole_second({schedule.schedule_id, 11U});
  controller.observe_pps(12U, 1002, 12'000'000U);
  const auto next_task_event = controller.emit_trigger(ppbng_timing::Channel::kRgb, 0U);
  EXPECT_EQ(next_task_event.event_sequence, first_event.event_sequence + 1U);
}

TEST(TimingModel, ContinuesEventsInPpsHoldoverThenMarksUnsynced) {
  ppbng_timing::ControllerModel controller(99U, 1'000'000U, 5U);
  const auto schedule = valid_schedule();
  controller.configure(schedule);
  controller.freeze(schedule.schedule_id);
  controller.arm_next_whole_second({schedule.schedule_id, 0U});
  controller.observe_pps(1U, 1000, 1'000'000U);

  controller.advance_to(3'000'000U);
  const auto holdover = controller.emit_trigger(ppbng_timing::Channel::kFx10e, 240U);
  EXPECT_EQ(holdover.lock, ppbng_timing::TimeLock::kHoldover);
  EXPECT_EQ(holdover.pps_sequence, 1U);
  EXPECT_EQ(holdover.offset_ticks, 2'000'000U);

  controller.advance_to(7'000'000U);
  const auto unsynced = controller.emit_trigger(ppbng_timing::Channel::kFx10e, 720U);
  EXPECT_EQ(unsynced.lock, ppbng_timing::TimeLock::kUnsynced);
  EXPECT_GT(controller.status().missed_pps_count, 0U);
}

TEST(TimingModel, EmitsPairedLogicalEventsForOneSnapshotEdge) {
  ppbng_timing::ControllerModel controller(99U, 1'000'000U);
  const auto schedule = valid_schedule();
  controller.configure(schedule);
  controller.freeze(schedule.schedule_id);
  controller.arm_next_whole_second({schedule.schedule_id, 0U});
  controller.observe_pps(1U, 1000, 1'000'000U);
  controller.advance_to(1'500'000U);

  const auto pair = controller.emit_snapshot_pair(7U);
  EXPECT_EQ(pair[0].channel, ppbng_timing::Channel::kRgb);
  EXPECT_EQ(pair[1].channel, ppbng_timing::Channel::kThermal);
  EXPECT_EQ(pair[0].channel_sequence, 7U);
  EXPECT_EQ(pair[1].channel_sequence, 7U);
  EXPECT_EQ(pair[0].pps_sequence, pair[1].pps_sequence);
  EXPECT_EQ(pair[0].offset_ticks, pair[1].offset_ticks);
  EXPECT_EQ(pair[0].lock, pair[1].lock);
  EXPECT_EQ(pair[1].event_sequence, pair[0].event_sequence + 1U);
}

TEST(TimingModel, RejectsBackwardPpsAndTick) {
  ppbng_timing::TimeLockTracker lock(1'000U, 3U);
  lock.observe_pps(5U, 5'000U);
  EXPECT_THROW(lock.observe_pps(5U, 6'000U), std::invalid_argument);
  EXPECT_THROW(lock.observe_pps(6U, 4'000U), std::invalid_argument);

  ppbng_timing::ControllerModel controller(1U, 1'000U);
  controller.advance_to(100U);
  EXPECT_THROW(controller.advance_to(99U), std::invalid_argument);
}

}  // namespace
