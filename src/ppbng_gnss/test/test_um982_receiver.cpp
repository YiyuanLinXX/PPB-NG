#include "ppbng_gnss/um982_receiver.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <utility>

namespace {

constexpr char kGga[] =
    "$GNGGA,123519.00,4250.000000,N,07630.000000,W,4,20,"
    "0.7,100.000,M,-30.000,M,0.8,0001*64";
constexpr char kHeading[] =
    "#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;"
    "SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,"
    "0.1000,0.2000,\"0\",20,16,18,12,0,00,0,0*5cdb44d1";

ppbng_gnss::ByteReadResult data(std::string text) {
  ppbng_gnss::ByteReadResult result;
  result.code = ppbng_gnss::ByteReadCode::data;
  result.bytes.resize(text.size());
  std::memcpy(result.bytes.data(), text.data(), text.size());
  return result;
}

class FakeReceiveOnlySource final : public ppbng_gnss::IReceiveOnlyByteSource {
 public:
  ppbng_gnss::ByteReadResult open(
      const ppbng_gnss::SerialReceiveConfiguration& configuration,
      std::chrono::milliseconds timeout,
      const ppbng_gnss::ReceiveStopToken& stop) override {
    ++open_calls;
    last_configuration = configuration;
    if (stop.stop_requested()) {
      return {ppbng_gnss::ByteReadCode::cancelled, {}, "cancelled"};
    }
    if (timeout.count() <= 0) {
      return {ppbng_gnss::ByteReadCode::timeout, {}, "timeout"};
    }
    open_ = open_succeeds;
    return open_ ? ppbng_gnss::ByteReadResult{ppbng_gnss::ByteReadCode::data, {}, {}}
                 : ppbng_gnss::ByteReadResult{ppbng_gnss::ByteReadCode::disconnected,
                                              {}, "disconnected"};
  }

  ppbng_gnss::ByteReadResult read(
      std::size_t maximum_bytes, std::chrono::milliseconds timeout,
      const ppbng_gnss::ReceiveStopToken& stop) override {
    ++read_calls;
    last_maximum_bytes = maximum_bytes;
    if (stop.stop_requested()) {
      return {ppbng_gnss::ByteReadCode::cancelled, {}, "cancelled"};
    }
    if (timeout.count() <= 0) {
      return {ppbng_gnss::ByteReadCode::timeout, {}, "timeout"};
    }
    if (!open_) {
      return {ppbng_gnss::ByteReadCode::disconnected, {}, "closed"};
    }
    if (reads.empty()) {
      return {ppbng_gnss::ByteReadCode::timeout, {}, "timeout"};
    }
    auto result = std::move(reads.front());
    reads.pop_front();
    return result;
  }

  void close() noexcept override { open_ = false; ++close_calls; }
  bool is_open() const noexcept override { return open_; }

  bool open_succeeds{true};
  bool open_{false};
  int open_calls{};
  int read_calls{};
  int close_calls{};
  std::size_t last_maximum_bytes{};
  ppbng_gnss::SerialReceiveConfiguration last_configuration;
  std::deque<ppbng_gnss::ByteReadResult> reads;
};

ppbng_gnss::SerialReceiveConfiguration configuration() {
  return {"COM17", 115200U, 64U, 4096U};
}

TEST(Um982Receiver, ConstructionNeverOpensTransportAndRequiresExplicitConfiguration) {
  ppbng_gnss::WindowsReceiveOnlySerial windows_transport;
  EXPECT_FALSE(windows_transport.is_open());

  auto fake = std::make_unique<FakeReceiveOnlySource>();
  auto* observed = fake.get();
  ppbng_gnss::Um982Receiver receiver(std::move(fake));
  EXPECT_EQ(observed->open_calls, 0);
  EXPECT_EQ(observed->read_calls, 0);
  EXPECT_EQ(receiver.state(), ppbng_gnss::ReceiverState::idle);

  ppbng_gnss::ReceiveStopSource stop;
  auto invalid = configuration();
  invalid.com_path.clear();
  EXPECT_EQ(receiver.connect(invalid, std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::io_error);
  EXPECT_EQ(observed->open_calls, 0);
}

TEST(Um982Receiver, IncrementallyParsesGgaAndHeadingWithRawLinesAndTimestamp) {
  auto fake = std::make_unique<FakeReceiveOnlySource>();
  auto* observed = fake.get();
  observed->reads.push_back(data(std::string(kGga).substr(0U, 20U)));
  observed->reads.push_back(data(std::string(kGga).substr(20U) + "\r\n" + kHeading + "\n"));
  ppbng_gnss::Um982Receiver receiver(std::move(fake));
  ppbng_gnss::ReceiveStopSource stop;
  ASSERT_EQ(receiver.connect(configuration(), std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::data);
  EXPECT_TRUE(receiver.poll(std::chrono::milliseconds(5), stop.token()).sentences.empty());
  const auto result = receiver.poll(std::chrono::milliseconds(5), stop.token());
  ASSERT_EQ(result.sentences.size(), 2U);
  EXPECT_EQ(result.sentences[0].kind, ppbng_gnss::SentenceKind::gga);
  ASSERT_TRUE(result.sentences[0].gga);
  EXPECT_EQ(result.sentences[0].raw_line, kGga);
  EXPECT_EQ(result.sentences[0].gga->quality, 4);
  EXPECT_NE(result.sentences[0].host_receive_time.time_since_epoch().count(), 0);
  EXPECT_EQ(result.sentences[1].kind, ppbng_gnss::SentenceKind::uniheadinga);
  ASSERT_TRUE(result.sentences[1].heading);
  EXPECT_EQ(result.sentences[1].raw_line, kHeading);
  EXPECT_EQ(observed->last_maximum_bytes, 64U);
}

TEST(Um982Receiver, PreservesMalformedRecognizedLineAndUnknownLine) {
  auto fake = std::make_unique<FakeReceiveOnlySource>();
  auto* observed = fake.get();
  observed->reads.push_back(data("$GNGGA,bad*00\n$GNRMC,ignored*00\n"));
  ppbng_gnss::Um982Receiver receiver(std::move(fake));
  ppbng_gnss::ReceiveStopSource stop;
  ASSERT_EQ(receiver.connect(configuration(), std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::data);
  const auto result = receiver.poll(std::chrono::milliseconds(5), stop.token());
  ASSERT_EQ(result.sentences.size(), 2U);
  EXPECT_EQ(result.sentences[0].kind, ppbng_gnss::SentenceKind::parse_error);
  EXPECT_EQ(result.sentences[0].raw_line, "$GNGGA,bad*00");
  EXPECT_FALSE(result.sentences[0].parse_error.empty());
  EXPECT_EQ(result.sentences[1].kind, ppbng_gnss::SentenceKind::other);
  EXPECT_EQ(result.sentences[1].raw_line, "$GNRMC,ignored*00");
}

TEST(Um982Receiver, DisconnectClearsPartialLineAndRecoveryIncrementsEpoch) {
  auto fake = std::make_unique<FakeReceiveOnlySource>();
  auto* observed = fake.get();
  observed->reads.push_back(data("stale-partial"));
  observed->reads.push_back(
      {ppbng_gnss::ByteReadCode::disconnected, {}, "USB removed"});
  observed->reads.push_back(data("fresh\n"));
  ppbng_gnss::Um982Receiver receiver(std::move(fake));
  ppbng_gnss::ReceiveStopSource stop;
  ASSERT_EQ(receiver.connect(configuration(), std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::data);
  EXPECT_TRUE(receiver.poll(std::chrono::milliseconds(5), stop.token()).sentences.empty());
  EXPECT_EQ(receiver.poll(std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::disconnected);
  EXPECT_EQ(receiver.state(), ppbng_gnss::ReceiverState::disconnected);
  ASSERT_EQ(receiver.recover(std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::data);
  EXPECT_EQ(receiver.connection_epoch(), 1U);
  const auto result = receiver.poll(std::chrono::milliseconds(5), stop.token());
  ASSERT_EQ(result.sentences.size(), 1U);
  EXPECT_EQ(result.sentences[0].raw_line, "fresh");
}

TEST(Um982Receiver, TimeoutAndCancellationRemainNonFaulting) {
  auto fake = std::make_unique<FakeReceiveOnlySource>();
  auto* observed = fake.get();
  observed->reads.push_back({ppbng_gnss::ByteReadCode::timeout, {}, "timeout"});
  ppbng_gnss::Um982Receiver receiver(std::move(fake));
  ppbng_gnss::ReceiveStopSource stop;
  ASSERT_EQ(receiver.connect(configuration(), std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::data);
  EXPECT_EQ(receiver.poll(std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::timeout);
  EXPECT_EQ(receiver.state(), ppbng_gnss::ReceiverState::connected);
  stop.request_stop();
  EXPECT_EQ(receiver.poll(std::chrono::milliseconds(5), stop.token()).code,
            ppbng_gnss::ByteReadCode::cancelled);
  EXPECT_EQ(receiver.state(), ppbng_gnss::ReceiverState::connected);
  EXPECT_EQ(observed->open_calls, 1);
}

}  // namespace
