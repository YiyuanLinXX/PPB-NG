#pragma once

#include "ppbng_rsm400/mcp2_protocol.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace ppbng_rsm400 {

enum class IoCode { ok, timeout, closed, error };
struct IoResult {
  IoCode code{IoCode::ok};
  std::size_t bytes{};
  std::string detail;
};

// A synchronous, partial-I/O transport. Implementations must return within the
// supplied timeout. The transaction layer never assumes one write/read is complete.
class IByteTransport {
 public:
  virtual ~IByteTransport() = default;
  virtual IoResult write_some(std::string_view bytes, std::chrono::milliseconds timeout) = 0;
  virtual IoResult read_some(char* destination, std::size_t capacity,
                             std::chrono::milliseconds timeout) = 0;
};

enum class FeatureAvailability { unknown, unavailable, available };
struct ConfirmedFeatures {
  FeatureAvailability of002_leveling_offset{FeatureAvailability::unknown};
  FeatureAvailability of005_status_analysis{FeatureAvailability::unknown};
};

enum class ControlKind {
  activate_horizon_stabilization,
  set_leveling_target,
  trigger_fast_level,
  reset_errors
};

struct ControlRequest {
  ControlKind kind{ControlKind::activate_horizon_stabilization};
  // Used only by set_leveling_target. ICD unit is 0.01 degree, range +/-3000.
  int roll_centidegrees{};
  int pitch_centidegrees{};
};

enum class TransactionCode {
  ok,
  control_disabled,
  feature_unknown,
  feature_unavailable,
  invalid_argument,
  timeout,
  transport_error,
  protocol_error,
  command_rejected,
  acknowledgement_mismatch,
  retransmission_limit
};

struct TransactionResult {
  TransactionCode code{TransactionCode::ok};
  std::uint64_t local_sequence{};
  std::size_t retransmissions{};
  std::string detail;
  std::vector<Frame> unsolicited_frames;
  bool ok() const noexcept { return code == TransactionCode::ok; }
};

struct CommandClientOptions {
  // Safety default: constructing a client can monitor but cannot write controls.
  bool allow_control{false};
  // GAT/MST only: permits configuring periodic telemetry without enabling
  // remote control or any motion-capable command.
  bool allow_observation_commands{false};
  ConfirmedFeatures features{};
  std::size_t maximum_retransmissions{2U};  // ICD says Mount requests at most two.
};

class CommandClient {
 public:
  CommandClient(IByteTransport& transport, CommandClientOptions options = {});
  TransactionResult execute(const ControlRequest& request, std::chrono::milliseconds timeout);
  TransactionResult configure_periodic_telemetry(
      int period_10ms, std::chrono::milliseconds timeout);
  std::uint64_t last_local_sequence() const noexcept;

 private:
  IByteTransport& transport_;
  CommandClientOptions options_;
  mutable std::mutex mutex_;
  std::uint64_t last_sequence_{};
  StreamDecoder decoder_;
  TransactionResult transact_locked(const std::vector<Message>& messages,
                                     std::chrono::milliseconds timeout,
                                     bool accept_data_bearing_acknowledgement);
};

}  // namespace ppbng_rsm400
