#include "ppbng_timing/host_client.hpp"

#include <gtest/gtest.h>

#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace {

class FakeControllerTransport final : public ppbng_timing::IControllerTransport {
 public:
  explicit FakeControllerTransport(bool verifies_identity = true)
      : verifies_identity_(verifies_identity) {}

  bool can_verify_usb_identity() const noexcept override { return verifies_identity_; }
  ppbng_timing::TransportResult open(
      const ppbng_timing::ControllerPortIdentity& identity,
      std::chrono::milliseconds timeout,
      const ppbng_timing::HostStopToken& stop) override {
    ++open_calls;
    opened_identity = identity;
    if (stop.stop_requested()) {
      return {ppbng_timing::TransportCode::cancelled, {}, "cancelled"};
    }
    if (timeout.count() <= 0) {
      return {ppbng_timing::TransportCode::timeout, {}, "timeout"};
    }
    open_ = true;
    return {};
  }

  ppbng_timing::TransportResult write(
      const std::vector<std::uint8_t>& bytes, std::chrono::milliseconds,
      const ppbng_timing::HostStopToken&) override {
    writes.push_back(bytes);
    if (!open_) return {ppbng_timing::TransportCode::disconnected, {}, "closed"};
    const auto request = ppbng_timing::parse_packet(bytes);
    if (drop_first_request && writes.size() == 1U) return {};
    respond(request);
    return {};
  }

  ppbng_timing::TransportResult read(
      std::size_t, std::chrono::milliseconds,
      const ppbng_timing::HostStopToken& stop) override {
    if (stop.stop_requested()) {
      return {ppbng_timing::TransportCode::cancelled, {}, "cancelled"};
    }
    if (!open_) return {ppbng_timing::TransportCode::disconnected, {}, "closed"};
    if (reads.empty()) return {ppbng_timing::TransportCode::timeout, {}, "timeout"};
    auto bytes = std::move(reads.front());
    reads.pop_front();
    return {ppbng_timing::TransportCode::ok, std::move(bytes), {}};
  }

  void close() noexcept override { open_ = false; ++close_calls; }
  bool is_open() const noexcept override { return open_; }

  void queue(const ppbng_timing::Packet& packet) {
    reads.push_back(ppbng_timing::serialize_packet(packet));
  }

  void respond(const ppbng_timing::Packet& request) {
    using namespace ppbng_timing;
    Packet response;
    response.sequence = request.sequence;
    if (request.type == MessageType::kGetVersion) {
      response.type = MessageType::kVersionReport;
      response.payload = encode(VersionInfo{1U, 0U, 0U, boot_id, 0x1U, 1'000'000U});
    } else if (request.type == MessageType::kConfigureSchedule) {
      active_schedule = decode_schedule_config(request.payload).schedule_id;
      controller_state = ControllerState::kConfigured;
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, controller_state, "configured"});
    } else if (request.type == MessageType::kFreezeConfiguration) {
      controller_state = ControllerState::kFrozen;
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, controller_state, "frozen"});
    } else if (request.type == MessageType::kArmNextWholeSecond) {
      (void)decode_arm_request(request.payload);
      controller_state = ControllerState::kWaitingForPps;
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, controller_state, "armed"});
    } else if (request.type == MessageType::kStartAtCurrentTick) {
      (void)decode_start_at_current_tick_request(request.payload);
      controller_state = ControllerState::kRunning;
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, controller_state, "running"});
    } else if (request.type == MessageType::kDisarm) {
      controller_state = ControllerState::kIdle;
      active_schedule = 0U;
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, controller_state, "idle"});
    } else if (request.type == MessageType::kDisarmKeepConfiguration) {
      controller_state = ControllerState::kFrozen;
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, controller_state, "frozen"});
    } else if (request.type == MessageType::kStatus) {
      response.type = MessageType::kStatus;
      const auto reported_state = forced_status_state.value_or(controller_state);
      response.payload = encode(StatusReport{forced_status_boot.value_or(boot_id),
          reported_state, active_schedule, request.sequence, 0U, 0U,
          TimeLock::kUnsynced, 0U, forced_error_flags});
    }
    queue(response);
  }

  bool verifies_identity_;
  bool open_{false};
  bool drop_first_request{false};
  int open_calls{};
  int close_calls{};
  std::uint32_t boot_id{77U};
  std::uint32_t active_schedule{};
  std::uint32_t forced_error_flags{};
  ppbng_timing::ControllerState controller_state{ppbng_timing::ControllerState::kIdle};
  std::optional<ppbng_timing::ControllerState> forced_status_state;
  std::optional<std::uint32_t> forced_status_boot;
  ppbng_timing::ControllerPortIdentity opened_identity;
  std::vector<std::vector<std::uint8_t>> writes;
  std::deque<std::vector<std::uint8_t>> reads;
};

ppbng_timing::ControllerPortIdentity identity() {
  return {"COM42", 921600U, 0x1234U, 0x5678U, "TIMING-001", 77U, false};
}

ppbng_timing::ScheduleConfig schedule() {
  using ppbng_timing::Channel;
  return {12U, 1'000'000U,
          {{Channel::kFx10e, true, 120U, 1U, 1000U, 0U},
           {Channel::kSwir, true, 120U, 1U, 1000U, 0U},
           {Channel::kRgb, true, 2U, 1U, 1000U, 0U},
           {Channel::kThermal, true, 2U, 1U, 1000U, 0U}}};
}

void connect_and_arm(ppbng_timing::TimingControllerClient& client,
                     const ppbng_timing::HostStopToken& stop) {
  ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop).ok());
  ASSERT_TRUE(client.configure(schedule(), std::chrono::milliseconds(50), stop).ok());
  ASSERT_TRUE(client.freeze(std::chrono::milliseconds(50), stop).ok());
  ASSERT_TRUE(client.arm({12U, 0U}, std::chrono::milliseconds(50), stop).ok());
}

TEST(TimingHostClient, ConstructionDoesNotOpenAndRejectsUnverifiedOsIdentityBeforeOpen) {
  auto fake = std::make_unique<FakeControllerTransport>(false);
  auto* observed = fake.get();
  ppbng_timing::TimingControllerClient client(std::move(fake));
  EXPECT_EQ(observed->open_calls, 0);
  ppbng_timing::HostStopSource stop;
  EXPECT_EQ(client.connect(identity(), std::chrono::milliseconds(10), stop.token()).code,
            ppbng_timing::HostClientCode::identity_unverified);
  EXPECT_EQ(observed->open_calls, 0);

  ppbng_timing::WindowsControllerSerialTransport windows_transport;
  EXPECT_FALSE(windows_transport.is_open());
  EXPECT_FALSE(windows_transport.can_verify_usb_identity());
}

TEST(TimingHostClient, ExecutesStrictLifecycleWithAckAndStatusReadback) {
  auto fake = std::make_unique<FakeControllerTransport>();
  auto* observed = fake.get();
  ppbng_timing::TimingControllerClient client(std::move(fake));
  ppbng_timing::HostStopSource stop;
  ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
  EXPECT_EQ(client.boot_id(), 77U);
  ASSERT_TRUE(client.configure(schedule(), std::chrono::milliseconds(50), stop.token()).ok());
  EXPECT_EQ(client.state(), ppbng_timing::HostClientState::configured);
  ASSERT_TRUE(client.freeze(std::chrono::milliseconds(50), stop.token()).ok());
  EXPECT_EQ(client.state(), ppbng_timing::HostClientState::frozen);
  ASSERT_TRUE(client.arm({12U, 10U}, std::chrono::milliseconds(50), stop.token()).ok());
  EXPECT_EQ(client.state(), ppbng_timing::HostClientState::waiting_for_pps);
  ASSERT_TRUE(client.disarm(std::chrono::milliseconds(50), stop.token()).ok());
  EXPECT_EQ(client.state(), ppbng_timing::HostClientState::connected);
  EXPECT_EQ(observed->active_schedule, 0U);
}

TEST(TimingHostClient, StartsFrozenScheduleWithoutWaitingForPps) {
  auto fake = std::make_unique<FakeControllerTransport>();
  ppbng_timing::TimingControllerClient client(std::move(fake));
  ppbng_timing::HostStopSource stop;
  ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
  ASSERT_NE(client.capabilities() & ppbng_timing::kCapabilityStartAtCurrentTick, 0U);
  ASSERT_TRUE(client.configure(schedule(), std::chrono::milliseconds(50), stop.token()).ok());
  ASSERT_TRUE(client.freeze(std::chrono::milliseconds(50), stop.token()).ok());
  ASSERT_TRUE(client.start_at_current_tick({12U}, std::chrono::milliseconds(50),
                                           stop.token()).ok());
  EXPECT_EQ(client.state(), ppbng_timing::HostClientState::running);
}

TEST(TimingHostClient, RetriesAnIdenticalSerializedCommandWithSameSequence) {
  auto fake = std::make_unique<FakeControllerTransport>();
  auto* observed = fake.get();
  observed->drop_first_request = true;
  ppbng_timing::TimingControllerClient client(std::move(fake));
  ppbng_timing::HostStopSource stop;
  ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
  ASSERT_GE(observed->writes.size(), 2U);
  EXPECT_EQ(observed->writes[0], observed->writes[1]);
  const auto first = ppbng_timing::parse_packet(observed->writes[0]);
  const auto second = ppbng_timing::parse_packet(observed->writes[1]);
  EXPECT_EQ(first.sequence, second.sequence);
}

TEST(TimingHostClient, StatusReadbackMismatchLatchesFatal) {
  auto fake = std::make_unique<FakeControllerTransport>();
  auto* observed = fake.get();
  ppbng_timing::TimingControllerClient client(std::move(fake));
  ppbng_timing::HostStopSource stop;
  ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
  observed->forced_status_state = ppbng_timing::ControllerState::kFrozen;
  EXPECT_EQ(client.configure(schedule(), std::chrono::milliseconds(50), stop.token()).code,
            ppbng_timing::HostClientCode::fatal);
  EXPECT_EQ(client.state(), ppbng_timing::HostClientState::fault);
}

TEST(TimingHostClient, ParsesPpsAndPairedSnapshotEvents) {
  auto fake = std::make_unique<FakeControllerTransport>();
  auto* observed = fake.get();
  ppbng_timing::TimingControllerClient client(std::move(fake));
  ppbng_timing::HostStopSource stop;
  connect_and_arm(client, stop.token());

  ppbng_timing::Packet pps{ppbng_timing::kProtocolVersion,
      ppbng_timing::MessageType::kPpsAnchor, 100U,
      ppbng_timing::encode(ppbng_timing::PpsAnchor{77U, 1U,
          (std::numeric_limits<std::int64_t>::min)(), 1'000'000U,
          ppbng_timing::TimeLock::kLocked})};
  ppbng_timing::Packet rgb{ppbng_timing::kProtocolVersion,
      ppbng_timing::MessageType::kTriggerEvent, 101U,
      ppbng_timing::encode(ppbng_timing::TriggerEvent{77U, 10U,
          ppbng_timing::Channel::kRgb, 0U, 1U, 500'000U, 1'000'000U,
          ppbng_timing::TimeLock::kLocked})};
  ppbng_timing::Packet thermal{ppbng_timing::kProtocolVersion,
      ppbng_timing::MessageType::kTriggerEvent, 102U,
      ppbng_timing::encode(ppbng_timing::TriggerEvent{77U, 11U,
          ppbng_timing::Channel::kThermal, 0U, 1U, 500'000U, 1'000'000U,
          ppbng_timing::TimeLock::kLocked})};
  auto bytes = ppbng_timing::serialize_packet(pps);
  const auto rgb_bytes = ppbng_timing::serialize_packet(rgb);
  const auto thermal_bytes = ppbng_timing::serialize_packet(thermal);
  bytes.insert(bytes.end(), rgb_bytes.begin(), rgb_bytes.end());
  bytes.insert(bytes.end(), thermal_bytes.begin(), thermal_bytes.end());
  observed->reads.push_back(std::move(bytes));

  const auto result = client.poll_events(std::chrono::milliseconds(50), stop.token());
  EXPECT_TRUE(result.status.ok());
  EXPECT_EQ(result.events.size(), 3U);
}

TEST(TimingHostClient, RebootCounterRegressionAndOverflowAreFatal) {
  ppbng_timing::HostStopSource stop;
  {
    auto fake = std::make_unique<FakeControllerTransport>();
    auto* observed = fake.get();
    ppbng_timing::TimingControllerClient client(std::move(fake));
    ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
    observed->queue({ppbng_timing::kProtocolVersion, ppbng_timing::MessageType::kPpsAnchor,
        1U, ppbng_timing::encode(ppbng_timing::PpsAnchor{78U, 1U, 0, 1U,
            ppbng_timing::TimeLock::kLocked})});
    EXPECT_EQ(client.poll_events(std::chrono::milliseconds(50), stop.token()).status.code,
              ppbng_timing::HostClientCode::fatal);
  }
  {
    auto fake = std::make_unique<FakeControllerTransport>();
    auto* observed = fake.get();
    ppbng_timing::TimingControllerClient client(std::move(fake));
    ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
    observed->queue({ppbng_timing::kProtocolVersion, ppbng_timing::MessageType::kTriggerEvent,
        1U, ppbng_timing::encode(ppbng_timing::TriggerEvent{77U, 9U,
            ppbng_timing::Channel::kFx10e, 2U, 1U, 0U, 1'000'000U,
            ppbng_timing::TimeLock::kLocked})});
    EXPECT_TRUE(client.poll_events(std::chrono::milliseconds(50), stop.token()).status.ok());
    observed->queue({ppbng_timing::kProtocolVersion, ppbng_timing::MessageType::kTriggerEvent,
        2U, ppbng_timing::encode(ppbng_timing::TriggerEvent{77U, 8U,
            ppbng_timing::Channel::kFx10e, 1U, 1U, 1U, 1'000'000U,
            ppbng_timing::TimeLock::kLocked})});
    EXPECT_EQ(client.poll_events(std::chrono::milliseconds(50), stop.token()).status.code,
              ppbng_timing::HostClientCode::fatal);
  }
  {
    auto fake = std::make_unique<FakeControllerTransport>();
    auto* observed = fake.get();
    ppbng_timing::TimingControllerClient client(std::move(fake));
    ASSERT_TRUE(client.connect(identity(), std::chrono::milliseconds(50), stop.token()).ok());
    observed->queue({ppbng_timing::kProtocolVersion, ppbng_timing::MessageType::kStatus,
        3U, ppbng_timing::encode(ppbng_timing::StatusReport{77U,
            ppbng_timing::ControllerState::kRunning, 12U, 0U, 1U, 1U,
            ppbng_timing::TimeLock::kLocked, 0U, 1U})});
    EXPECT_EQ(client.poll_events(std::chrono::milliseconds(50), stop.token()).status.code,
              ppbng_timing::HostClientCode::fatal);
  }
}

TEST(TimingHostClient, ChannelCounterWrapIsFatal) {
  auto fake = std::make_unique<FakeControllerTransport>();
  auto* backend = fake.get();
  ppbng_timing::TimingControllerClient client(std::move(fake));
  ppbng_timing::HostStopSource stop;
  connect_and_arm(client, stop.token());

  const ppbng_timing::TriggerEvent almost_wrapped{
      77U, 1U, ppbng_timing::Channel::kFx10e,
      (std::numeric_limits<std::uint64_t>::max)(), 1U, 0U, 1'000'000U,
      ppbng_timing::TimeLock::kLocked};
  backend->queue({ppbng_timing::kProtocolVersion,
                  ppbng_timing::MessageType::kTriggerEvent, 20U,
                  ppbng_timing::encode(almost_wrapped)});
  EXPECT_TRUE(client.poll_events(std::chrono::milliseconds(20), stop.token()).status.ok());

  const ppbng_timing::TriggerEvent wrapped{
      77U, 2U, ppbng_timing::Channel::kFx10e, 0U, 1U, 1U, 1'000'000U,
      ppbng_timing::TimeLock::kLocked};
  backend->queue({ppbng_timing::kProtocolVersion,
                  ppbng_timing::MessageType::kTriggerEvent, 21U,
                  ppbng_timing::encode(wrapped)});
  EXPECT_EQ(client.poll_events(std::chrono::milliseconds(20), stop.token()).status.code,
            ppbng_timing::HostClientCode::fatal);
}

}  // namespace
