#pragma once

#include "ppbng_rsm400/mcp2_protocol.hpp"

#include <string>

namespace ppbng_rsm400
{

struct ReadinessConfiguration
{
  bool require_ready_telemetry{false};
  bool stab_major_status_confirmed{false};
  int expected_stab_major_status{-1};
  int maximum_error_level{-1};
};

enum class ReadinessState {waiting, ready, failed};

struct ReadinessResult
{
  ReadinessState state{ReadinessState::waiting};
  std::string detail;
  [[nodiscard]] bool ready() const noexcept {return state == ReadinessState::ready;}
  [[nodiscard]] bool failed() const noexcept {return state == ReadinessState::failed;}
};

// Pure, transport-free aggregation of checksum-validated MCP telemetry. Roll,
// pitch, and MS may arrive in separate frames. An unacceptable error level is
// fatal immediately; a non-STAB major status remains pending until timeout so
// an already-commanded mount has a bounded opportunity to settle.
class ReadinessGate
{
public:
  explicit ReadinessGate(ReadinessConfiguration configuration);
  [[nodiscard]] ReadinessResult configuration_result() const;
  ReadinessResult observe(const Telemetry & telemetry);
  ReadinessResult timeout();
  [[nodiscard]] ReadinessState state() const noexcept;

private:
  ReadinessResult evaluate();

  ReadinessConfiguration configuration_;
  ReadinessState state_{ReadinessState::waiting};
  std::string detail_;
  bool roll_valid_{false};
  bool pitch_valid_{false};
  bool general_status_valid_{false};
  int major_status_{-1};
  int error_level_{-1};
};

}  // namespace ppbng_rsm400
