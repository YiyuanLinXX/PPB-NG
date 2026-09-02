#pragma once

#include "ppbng_timing/host_client.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace ppbng_timing {

enum class ProductionState { inert, prepared, started, armed, fault };

struct ProductionConfiguration {
  bool hardware_enabled{false};
  bool trusted_usb_identity_mapping{false};
  ControllerPortIdentity identity;
  ScheduleConfig schedule;
  std::uint64_t arm_after_pps_sequence{};
  std::chrono::milliseconds command_timeout{500};
  std::chrono::milliseconds poll_timeout{5};
};

struct ProductionResult {
  bool success{false};
  std::string detail;
};

struct ProductionPollResult {
  ProductionResult result;
  std::vector<ControllerEvent> events;
};

using HostClientFactory = std::function<std::unique_ptr<TimingControllerClient>()>;

// Fail-closed orchestration core. Construction and prepare() perform no transport
// I/O. Configuration is copied and frozen by prepare() until a successful stop().
class ProductionControl {
 public:
  ProductionControl(ProductionConfiguration configuration, HostClientFactory factory);
  ~ProductionControl();
  ProductionControl(const ProductionControl&) = delete;
  ProductionControl& operator=(const ProductionControl&) = delete;

  ProductionResult prepare();
  ProductionResult start();
  ProductionResult arm_next_pps();
  ProductionResult start_without_pps();
  ProductionResult disarm_keep_configuration();
  ProductionResult stop();
  ProductionPollResult poll();

  ProductionState state() const noexcept { return state_; }
  bool configuration_locked() const noexcept { return state_ != ProductionState::inert; }
  std::uint64_t emergency_disarm_attempts() const noexcept { return disarm_attempts_; }

 private:
  ProductionResult latch_fault(std::string detail, bool attempt_disarm);
  ProductionResult try_disarm();

  ProductionConfiguration configuration_;
  HostClientFactory factory_;
  ProductionState state_{ProductionState::inert};
  std::unique_ptr<TimingControllerClient> client_;
  HostStopSource stop_source_;
  std::uint64_t disarm_attempts_{};
};

}  // namespace ppbng_timing
