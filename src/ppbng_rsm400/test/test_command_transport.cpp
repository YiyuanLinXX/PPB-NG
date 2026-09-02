#include "ppbng_rsm400/command_transport.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <deque>
#include <string>
#include <utility>

namespace {
using namespace ppbng_rsm400;

std::string make_frame(char status, const std::string& messages = {}) {
  std::string tail(1U, status);
  tail += " /" + messages + "\r\n";
  auto digits = std::to_string(calculate_checksum(tail));
  digits.insert(digits.begin(), 3U - digits.size(), '0');
  return "VM" + digits + tail;
}

class FakeTransport final : public IByteTransport {
 public:
  IoResult write_some(std::string_view bytes, std::chrono::milliseconds) override {
    ++write_calls;
    if (write_code != IoCode::ok) { return {write_code, 0U, "scripted write failure"}; }
    const auto count = std::min(max_write, bytes.size());
    written.append(bytes.data(), count);
    return {IoCode::ok, count, {}};
  }

  IoResult read_some(char* destination, std::size_t capacity,
                     std::chrono::milliseconds) override {
    ++read_calls;
    if (reads.empty()) { return {IoCode::timeout, 0U, "scripted timeout"}; }
    auto& item = reads.front();
    if (item.first != IoCode::ok) {
      const auto code = item.first;
      reads.pop_front();
      return {code, 0U, "scripted read failure"};
    }
    const auto count = std::min(capacity, item.second.size());
    std::memcpy(destination, item.second.data(), count);
    item.second.erase(0U, count);
    if (item.second.empty()) { reads.pop_front(); }
    return {IoCode::ok, count, {}};
  }

  std::size_t max_write{1024U};
  IoCode write_code{IoCode::ok};
  std::deque<std::pair<IoCode, std::string>> reads;
  std::string written;
  std::size_t write_calls{};
  std::size_t read_calls{};
};

CommandClientOptions control_options() {
  CommandClientOptions options;
  options.allow_control = true;
  options.features.of002_leveling_offset = FeatureAvailability::available;
  options.features.of005_status_analysis = FeatureAvailability::available;
  return options;
}

TEST(CommandTransport, ObserveOnlyPerformsNoWrite) {
  FakeTransport transport;
  CommandClient client(transport);
  const auto result = client.execute({ControlKind::trigger_fast_level}, std::chrono::milliseconds(10));
  EXPECT_EQ(result.code, TransactionCode::control_disabled);
  EXPECT_EQ(result.local_sequence, 1U);
  EXPECT_EQ(transport.write_calls, 0U);
  EXPECT_EQ(transport.read_calls, 0U);
}

TEST(CommandTransport, HandlesPartialWriteAndReadWithExactHorizonStabCommand) {
  FakeTransport transport;
  transport.max_write = 2U;
  const auto ack = make_frame('H');
  transport.reads.push_back({IoCode::ok, ack.substr(0U, 4U)});
  transport.reads.push_back({IoCode::ok, ack.substr(4U)});
  CommandClient client(transport, control_options());
  const auto result = client.execute({ControlKind::activate_horizon_stabilization},
                                     std::chrono::milliseconds(50));
  EXPECT_TRUE(result.ok()) << result.detail;
  EXPECT_GT(transport.write_calls, 1U);
  EXPECT_EQ(parse_frame(transport.written).messages.at(0).raw, "ST 1");
}

TEST(CommandTransport, BuildsDocumentedOf002CentidegreePair) {
  FakeTransport transport;
  transport.reads.push_back({IoCode::ok, make_frame('H')});
  CommandClient client(transport, control_options());
  ControlRequest request{ControlKind::set_leveling_target, -125, 250};
  const auto result = client.execute(request, std::chrono::milliseconds(50));
  ASSERT_TRUE(result.ok()) << result.detail;
  const auto frame = parse_frame(transport.written);
  ASSERT_EQ(frame.messages.size(), 2U);
  EXPECT_EQ(frame.messages[0].raw, "OFR -125");
  EXPECT_EQ(frame.messages[1].raw, "OFP 250");
}

TEST(CommandTransport, RejectsUnknownOrUnavailableFeatureBeforeWrite) {
  FakeTransport transport;
  CommandClientOptions options;
  options.allow_control = true;
  CommandClient client(transport, options);
  EXPECT_EQ(client.execute({ControlKind::set_leveling_target, 0, 0},
                           std::chrono::milliseconds(10)).code,
            TransactionCode::feature_unknown);
  EXPECT_EQ(transport.write_calls, 0U);

  options.features.of005_status_analysis = FeatureAvailability::unavailable;
  CommandClient second(transport, options);
  EXPECT_EQ(second.execute({ControlKind::reset_errors}, std::chrono::milliseconds(10)).code,
            TransactionCode::feature_unavailable);
  EXPECT_EQ(transport.write_calls, 0U);
}

TEST(CommandTransport, RejectsOutOfRangeOffsetAndUnknownControlKind) {
  FakeTransport transport;
  auto options = control_options();
  CommandClient client(transport, options);
  EXPECT_EQ(client.execute({ControlKind::set_leveling_target, 3001, 0},
                           std::chrono::milliseconds(10)).code,
            TransactionCode::invalid_argument);
  ControlRequest unknown;
  unknown.kind = static_cast<ControlKind>(99);
  EXPECT_EQ(client.execute(unknown, std::chrono::milliseconds(10)).code,
            TransactionCode::invalid_argument);
  EXPECT_EQ(transport.write_calls, 0U);
}

TEST(CommandTransport, ReportsTimeoutAndAcknowledgementMismatch) {
  FakeTransport timeout_transport;
  timeout_transport.reads.push_back({IoCode::timeout, {}});
  CommandClient timeout_client(timeout_transport, control_options());
  EXPECT_EQ(timeout_client.execute({ControlKind::trigger_fast_level},
                                   std::chrono::milliseconds(10)).code,
            TransactionCode::timeout);

  FakeTransport mismatch_transport;
  mismatch_transport.reads.push_back({IoCode::ok, make_frame('H', "MS 24000/")});
  CommandClient mismatch_client(mismatch_transport, control_options());
  EXPECT_EQ(mismatch_client.execute({ControlKind::trigger_fast_level},
                                    std::chrono::milliseconds(10)).code,
            TransactionCode::acknowledgement_mismatch);
}

TEST(CommandTransport, RejectsMountCommandError) {
  FakeTransport transport;
  transport.reads.push_back({IoCode::ok, make_frame('L')});  // ACKN + CMD ERROR.
  CommandClient client(transport, control_options());
  EXPECT_EQ(client.execute({ControlKind::trigger_fast_level},
                           std::chrono::milliseconds(10)).code,
            TransactionCode::command_rejected);
}

TEST(CommandTransport, RetransmitsIdenticalFrameAndKeepsLocalSequence) {
  FakeTransport transport;
  transport.reads.push_back({IoCode::ok, make_frame('A')});  // RETR, no ACKN.
  transport.reads.push_back({IoCode::ok, make_frame('H')});
  CommandClient client(transport, control_options());
  const auto result = client.execute({ControlKind::trigger_fast_level},
                                     std::chrono::milliseconds(50));
  ASSERT_TRUE(result.ok()) << result.detail;
  EXPECT_EQ(result.retransmissions, 1U);
  EXPECT_EQ(result.local_sequence, 1U);
  ASSERT_EQ(transport.written.size() % 2U, 0U);
  EXPECT_EQ(transport.written.substr(0U, transport.written.size() / 2U),
            transport.written.substr(transport.written.size() / 2U));
}

TEST(CommandTransport, PreservesUnsolicitedTelemetryBeforeAck) {
  FakeTransport transport;
  transport.reads.push_back({IoCode::ok, make_frame('@', "GR 12/GP -34/") + make_frame('H')});
  CommandClient client(transport, control_options());
  const auto result = client.execute({ControlKind::trigger_fast_level},
                                     std::chrono::milliseconds(50));
  ASSERT_TRUE(result.ok()) << result.detail;
  ASSERT_EQ(result.unsolicited_frames.size(), 1U);
  EXPECT_EQ(result.unsolicited_frames[0].messages[0].raw, "GR 12");
}

}  // namespace
