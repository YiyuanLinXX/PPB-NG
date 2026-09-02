#include "ppbng_rsm400/mcp2_protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr char kRemoteControlExample[] = "VM136H /RC 1/SCA 0/SCD 1/\r\n";
constexpr char kAcknowledgementExample[] = "VM081H /\r\n";
constexpr char kAttitudeExample[] = "VM159@ /AP -400/AR 0/AYN 1910/\r\n";

std::string make_frame(char ack, std::string messages) {
  std::string tail;
  tail.push_back(ack);
  tail += " /" + messages + "\r\n";
  const auto checksum = ppbng_rsm400::calculate_checksum(tail);
  std::string digits = std::to_string(checksum);
  digits.insert(digits.begin(), 3U - digits.size(), '0');
  return "VM" + digits + tail;
}

TEST(Mcp2Protocol, ParsesManufacturerRemoteControlGoldenFrame) {
  const auto frame = ppbng_rsm400::parse_frame(kRemoteControlExample);
  EXPECT_EQ(frame.received_checksum, 136U);
  EXPECT_EQ(frame.connection_status.raw, static_cast<std::uint8_t>('H'));
  EXPECT_TRUE(frame.connection_status.acknowledgement);
  EXPECT_FALSE(frame.connection_status.command_error);
  ASSERT_EQ(frame.messages.size(), 3U);
  EXPECT_EQ(frame.messages[0].command, "RC");
  ASSERT_TRUE(frame.messages[0].argument.has_value());
  EXPECT_EQ(*frame.messages[0].argument, "1");
  EXPECT_EQ(frame.messages[2].raw, "SCD 1");
}

TEST(Mcp2Protocol, ParsesManufacturerEmptyAcknowledgementGoldenFrame) {
  const auto frame = ppbng_rsm400::parse_frame(kAcknowledgementExample);
  EXPECT_EQ(frame.received_checksum, 81U);
  EXPECT_TRUE(frame.connection_status.acknowledgement);
  EXPECT_TRUE(frame.messages.empty());
}

TEST(Mcp2Protocol, ValidatesManufacturerAttitudeGoldenChecksum) {
  const auto frame = ppbng_rsm400::parse_frame(kAttitudeExample);
  EXPECT_EQ(frame.received_checksum, 159U);
  EXPECT_EQ(frame.connection_status.raw, static_cast<std::uint8_t>('@'));
  EXPECT_FALSE(frame.connection_status.acknowledgement);
  ASSERT_EQ(frame.messages.size(), 3U);
  EXPECT_EQ(frame.messages[0].raw, "AP -400");
}

TEST(Mcp2Protocol, RejectsBadChecksumAndTruncatedFrame) {
  std::string bad(kRemoteControlExample);
  bad.replace(2U, 3U, "000");
  EXPECT_THROW(ppbng_rsm400::parse_frame(bad), std::invalid_argument);
  EXPECT_THROW(ppbng_rsm400::parse_frame("VM136H /RC 1/"), std::invalid_argument);
}

TEST(Mcp2Protocol, DecodesConnectionStatusBits) {
  const auto frame = ppbng_rsm400::parse_frame(make_frame('O', "MS 24000/"));
  EXPECT_TRUE(frame.connection_status.acknowledgement);
  EXPECT_TRUE(frame.connection_status.command_error);
  EXPECT_TRUE(frame.connection_status.checksum_error);
  EXPECT_TRUE(frame.connection_status.retransmission_requested);
}

TEST(Mcp2Telemetry, DecodesDocumentedAnglesTimerAndStatus) {
  const auto frame = ppbng_rsm400::parse_frame(
      make_frame('@', "GR 0/GP 183/GY 825/TM 118800017/MS 24000/EMS 01900/"));
  const auto telemetry = ppbng_rsm400::decode_telemetry(frame);
  ASSERT_TRUE(telemetry.roll_deg.has_value());
  ASSERT_TRUE(telemetry.pitch_deg.has_value());
  ASSERT_TRUE(telemetry.yaw_deg.has_value());
  EXPECT_DOUBLE_EQ(*telemetry.roll_deg, 0.0);
  EXPECT_DOUBLE_EQ(*telemetry.pitch_deg, 1.83);
  EXPECT_DOUBLE_EQ(*telemetry.yaw_deg, 8.25);
  ASSERT_TRUE(telemetry.timer_ms.has_value());
  EXPECT_EQ(*telemetry.timer_ms, 118800017);
  ASSERT_TRUE(telemetry.general_status.has_value());
  EXPECT_EQ(telemetry.general_status->control_source, 2);
  EXPECT_EQ(telemetry.general_status->major_status, 4);
  EXPECT_EQ(telemetry.general_status->error_level, 0);
  ASSERT_TRUE(telemetry.extended_status.has_value());
  EXPECT_EQ(telemetry.extended_status->inertial_navigation_support, 9);
  EXPECT_EQ(telemetry.extended_status->raw_argument, "01900");
}

TEST(Mcp2Telemetry, DecodesManufacturerEventAngleExample) {
  const auto frame = ppbng_rsm400::parse_frame(
      make_frame('@', "GRE 0/GPE 183/GYE 825/TME 118804981/"));
  const auto telemetry = ppbng_rsm400::decode_telemetry(frame);
  EXPECT_DOUBLE_EQ(*telemetry.event_roll_deg, 0.0);
  EXPECT_DOUBLE_EQ(*telemetry.event_pitch_deg, 1.83);
  EXPECT_DOUBLE_EQ(*telemetry.event_yaw_deg, 8.25);
  EXPECT_EQ(*telemetry.event_timer_ms, 118804981);
}

TEST(Mcp2Telemetry, DecodesManufacturerErrorGroupExample) {
  const auto frame = ppbng_rsm400::parse_frame(
      make_frame('@', "ERG 103.000.200.002/"));
  const auto telemetry = ppbng_rsm400::decode_telemetry(frame);
  ASSERT_TRUE(telemetry.error_groups.has_value());
  const auto& errors = *telemetry.error_groups;
  EXPECT_EQ(errors.raw_groups[0], "103");
  EXPECT_NE(errors.error_bits[0] & (1U << 3U), 0U);
  EXPECT_NE(errors.error_bits[0] & (1U << 10U), 0U);
  EXPECT_NE(errors.error_bits[0] & (1U << 11U), 0U);
  EXPECT_NE(errors.error_bits[2] & (1U << 2U), 0U);
  EXPECT_NE(errors.error_bits[3] & (1U << 10U), 0U);
}

TEST(Mcp2Telemetry, PreservesUnknownMessagesWithoutGuessing) {
  const auto frame = ppbng_rsm400::parse_frame(
      make_frame('@', "VER GSM4000.090211_A0.606.054.001/NAC 2/"));
  const auto telemetry = ppbng_rsm400::decode_telemetry(frame);
  ASSERT_EQ(telemetry.unhandled_messages.size(), 2U);
  EXPECT_EQ(telemetry.unhandled_messages[0].command, "VER");
  EXPECT_EQ(telemetry.unhandled_messages[0].raw, "VER GSM4000.090211_A0.606.054.001");
}

TEST(Mcp2StreamDecoder, HandlesNoiseFragmentationAndCoalescedFrames) {
  ppbng_rsm400::StreamDecoder decoder;
  EXPECT_TRUE(decoder.feed("noiseV").empty());
  auto first = decoder.feed("M136H /RC 1/SCA 0/SCD 1/\r");
  EXPECT_TRUE(first.empty());
  auto frames = decoder.feed(std::string("\n") + kAcknowledgementExample);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0], kRemoteControlExample);
  EXPECT_EQ(frames[1], kAcknowledgementExample);
  EXPECT_EQ(decoder.discarded_noise_bytes(), 5U);
  EXPECT_EQ(decoder.buffered_bytes(), 0U);
}

TEST(Mcp2StreamDecoder, ResynchronizesAfterTruncatedFrame) {
  ppbng_rsm400::StreamDecoder decoder;
  const auto frames = decoder.feed(std::string("VM999H /BROKEN/") + kAcknowledgementExample);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0], kAcknowledgementExample);
  EXPECT_EQ(decoder.discarded_truncated_frames(), 1U);
}

TEST(Mcp2StreamDecoder, DropsOversizeCandidateAndRecovers) {
  ppbng_rsm400::StreamDecoder decoder;
  const std::string oversize = "VM" + std::string(254U, 'X') + "\r\n";
  auto frames = decoder.feed(oversize + kAcknowledgementExample);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0], kAcknowledgementExample);
  EXPECT_EQ(decoder.discarded_oversize_frames(), 1U);
}

}  // namespace
