#include "ppbng_timing/production_control.hpp"

#include <gtest/gtest.h>

#include <deque>
#include <memory>

namespace {

class FakeTransport final : public ppbng_timing::IControllerTransport {
 public:
  bool can_verify_usb_identity() const noexcept override { return false; }
  ppbng_timing::TransportResult open(const ppbng_timing::ControllerPortIdentity&,
      std::chrono::milliseconds, const ppbng_timing::HostStopToken&) override {
    ++open_calls; open_ = true; return {};
  }
  ppbng_timing::TransportResult write(const std::vector<std::uint8_t>& bytes,
      std::chrono::milliseconds, const ppbng_timing::HostStopToken&) override {
    ++write_calls;
    const auto request = ppbng_timing::parse_packet(bytes);
    using namespace ppbng_timing;
    Packet response;
    response.sequence = request.sequence;
    if (request.type == MessageType::kGetVersion) {
      response.type = MessageType::kVersionReport;
      response.payload = encode(VersionInfo{1U, 0U, 1U, 55U,
          kCapabilityStartAtCurrentTick, 1'000'000U});
    } else if (request.type == MessageType::kStatus) {
      response.type = MessageType::kStatus;
      response.payload = encode(StatusReport{55U, state, schedule_id, request.sequence,
          0U, 0U, TimeLock::kLocked, 0U, 0U});
    } else {
      if (request.type == MessageType::kConfigureSchedule) {
        schedule_id = decode_schedule_config(request.payload).schedule_id;
        state = ControllerState::kConfigured;
      } else if (request.type == MessageType::kFreezeConfiguration) {
        state = ControllerState::kFrozen;
      } else if (request.type == MessageType::kArmNextWholeSecond) {
        ++arm_commands; state = ControllerState::kWaitingForPps;
      } else if (request.type == MessageType::kStartAtCurrentTick) {
        ++start_without_pps_commands; state = ControllerState::kRunning;
      } else if (request.type == MessageType::kDisarm) {
        ++disarm_commands;
        if (fail_disarm) {
          return {TransportCode::io_error, {}, "injected disarm failure"};
        }
        state = ControllerState::kIdle; schedule_id = 0U;
      } else if (request.type == MessageType::kDisarmKeepConfiguration) {
        ++keep_config_disarm_commands;
        if (fail_keep_config_disarm) {
          return {TransportCode::io_error, {}, "injected keep-config disarm failure"};
        }
        state = ControllerState::kFrozen;
      }
      response.type = MessageType::kAck;
      response.payload = encode(Ack{request.sequence, 0U, state, "ok"});
    }
    queue(response);
    return {};
  }
  ppbng_timing::TransportResult read(std::size_t, std::chrono::milliseconds,
      const ppbng_timing::HostStopToken&) override {
    if (reads.empty()) return {ppbng_timing::TransportCode::timeout, {}, "timeout"};
    auto bytes = std::move(reads.front()); reads.pop_front();
    return {ppbng_timing::TransportCode::ok, std::move(bytes), {}};
  }
  void close() noexcept override { open_ = false; ++close_calls; }
  bool is_open() const noexcept override { return open_; }
  void queue(const ppbng_timing::Packet& packet) {
    reads.push_back(ppbng_timing::serialize_packet(packet));
  }

  bool open_{false};
  int open_calls{};
  int close_calls{};
  int write_calls{};
  int arm_commands{};
  int start_without_pps_commands{};
  int disarm_commands{};
  int keep_config_disarm_commands{};
  bool fail_disarm{false};
  bool fail_keep_config_disarm{false};
  std::uint32_t schedule_id{};
  ppbng_timing::ControllerState state{ppbng_timing::ControllerState::kIdle};
  std::deque<std::vector<std::uint8_t>> reads;
};

ppbng_timing::ProductionConfiguration configuration(bool enabled = true) {
  using namespace ppbng_timing;
  ProductionConfiguration config;
  config.hardware_enabled = enabled;
  config.trusted_usb_identity_mapping = true;
  config.identity = {"COM42", 921600U, 0x1234U, 0x5678U, "TIMING-1", 55U, false};
  config.schedule = {8U, 1'000'000U,
      {{Channel::kFx10e, true, 120U, 1U, 100U, 0U},
       {Channel::kSwir, true, 120U, 1U, 100U, 0U},
       {Channel::kRgb, true, 2U, 1U, 100U, 0U},
       {Channel::kThermal, true, 2U, 1U, 100U, 0U}}};
  config.command_timeout = std::chrono::milliseconds(20);
  config.poll_timeout = std::chrono::milliseconds(1);
  return config;
}

ppbng_timing::HostClientFactory factory(FakeTransport** observed, int* creations) {
  return [observed, creations] {
    ++*creations;
    auto transport = std::make_unique<FakeTransport>();
    *observed = transport.get();
    return std::make_unique<ppbng_timing::TimingControllerClient>(std::move(transport));
  };
}

TEST(TimingProductionControl, ConstructionAndPrepareNeverCreateOrOpenTransport) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl disabled(configuration(false), factory(&observed, &creations));
  EXPECT_EQ(creations, 0);
  EXPECT_FALSE(disabled.prepare().success);
  EXPECT_EQ(creations, 0);

  ppbng_timing::ProductionControl enabled(configuration(), factory(&observed, &creations));
  EXPECT_TRUE(enabled.prepare().success);
  EXPECT_TRUE(enabled.configuration_locked());
  EXPECT_EQ(creations, 0);
  EXPECT_EQ(observed, nullptr);
}

TEST(TimingProductionControl, StartFreezesButDoesNotArmUntilSeparateCommand) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl control(configuration(), factory(&observed, &creations));
  ASSERT_TRUE(control.prepare().success);
  ASSERT_TRUE(control.start().success);
  ASSERT_NE(observed, nullptr);
  EXPECT_EQ(observed->open_calls, 1);
  EXPECT_EQ(observed->arm_commands, 0);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::started);
  ASSERT_TRUE(control.arm_next_pps().success);
  EXPECT_EQ(observed->arm_commands, 1);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::armed);
  ASSERT_TRUE(control.stop().success);
  EXPECT_EQ(control.emergency_disarm_attempts(), 1U);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::inert);
}

TEST(TimingProductionControl, StopKeepsFaultedConnectionWhenDisarmIsUnconfirmed) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl control(configuration(), factory(&observed, &creations));
  ASSERT_TRUE(control.prepare().success);
  ASSERT_TRUE(control.start().success);
  ASSERT_TRUE(control.arm_next_pps().success);
  observed->fail_disarm = true;
  const auto stopped = control.stop();
  EXPECT_FALSE(stopped.success);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::fault);
  EXPECT_TRUE(observed->is_open());
  EXPECT_EQ(observed->disarm_commands, 1);
  observed->fail_disarm = false;
  EXPECT_TRUE(control.stop().success);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::inert);
}

TEST(TimingProductionControl, ExplicitNoPpsStartRunsAndStillDisarmsOnStop) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl control(configuration(), factory(&observed, &creations));
  ASSERT_TRUE(control.prepare().success);
  ASSERT_TRUE(control.start().success);
  const auto started = control.start_without_pps();
  ASSERT_TRUE(started.success) << started.detail;
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::armed);
  EXPECT_EQ(observed->start_without_pps_commands, 1);
  ASSERT_TRUE(control.stop().success);
  EXPECT_EQ(observed->disarm_commands, 1);
}

TEST(TimingProductionControl, DarkCanDisarmAndRearmWithoutClosingOrUnlocking) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl control(configuration(), factory(&observed, &creations));
  ASSERT_TRUE(control.prepare().success);
  ASSERT_TRUE(control.start().success);
  ASSERT_TRUE(control.arm_next_pps().success);
  ASSERT_TRUE(control.disarm_keep_configuration().success);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::started);
  EXPECT_TRUE(control.configuration_locked());
  EXPECT_TRUE(observed->is_open());
  EXPECT_EQ(observed->keep_config_disarm_commands, 1);
  ASSERT_TRUE(control.arm_next_pps().success);
  EXPECT_EQ(observed->arm_commands, 2);
}

TEST(TimingProductionControl, FailedKeepConfigDisarmStaysConnectedAndCanRetry) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl control(configuration(), factory(&observed, &creations));
  ASSERT_TRUE(control.prepare().success);
  ASSERT_TRUE(control.start().success);
  ASSERT_TRUE(control.arm_next_pps().success);
  observed->fail_keep_config_disarm = true;
  EXPECT_FALSE(control.disarm_keep_configuration().success);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::fault);
  EXPECT_TRUE(observed->is_open());
  observed->fail_keep_config_disarm = false;
  EXPECT_TRUE(control.disarm_keep_configuration().success);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::started);
}

TEST(TimingProductionControl, FatalRuntimeEventAttemptsEmergencyDisarm) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  ppbng_timing::ProductionControl control(configuration(), factory(&observed, &creations));
  ASSERT_TRUE(control.prepare().success);
  ASSERT_TRUE(control.start().success);
  ASSERT_TRUE(control.arm_next_pps().success);
  observed->queue({ppbng_timing::kProtocolVersion, ppbng_timing::MessageType::kStatus, 99U,
      ppbng_timing::encode(ppbng_timing::StatusReport{55U,
          ppbng_timing::ControllerState::kFault, 8U, 0U, 0U, 0U,
          ppbng_timing::TimeLock::kLocked, 0U, 1U})});
  const auto result = control.poll();
  EXPECT_FALSE(result.result.success);
  EXPECT_EQ(control.state(), ppbng_timing::ProductionState::fault);
  EXPECT_EQ(control.emergency_disarm_attempts(), 1U);
  EXPECT_EQ(observed->disarm_commands, 1);
}

TEST(TimingProductionControl, RejectsUntrustedIdentityWithoutCreatingTransport) {
  FakeTransport* observed = nullptr;
  int creations = 0;
  auto config = configuration();
  config.trusted_usb_identity_mapping = false;
  ppbng_timing::ProductionControl control(std::move(config), factory(&observed, &creations));
  EXPECT_FALSE(control.prepare().success);
  EXPECT_EQ(creations, 0);
}

}  // namespace
