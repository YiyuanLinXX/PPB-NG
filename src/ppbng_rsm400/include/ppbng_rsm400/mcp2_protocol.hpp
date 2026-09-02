#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ppbng_rsm400 {

constexpr std::size_t kMcp2MaximumFrameBytes = 255U;
constexpr std::size_t kMcp2MaximumMessages = 10U;

struct ConnectionStatus {
  std::uint8_t raw{};
  bool acknowledgement{};
  bool command_error{};
  bool checksum_error{};
  bool retransmission_requested{};
};

struct Message {
  std::string command;
  std::optional<std::string> argument;
  std::string raw;
};

struct Frame {
  std::uint8_t received_checksum{};
  ConnectionStatus connection_status;
  std::vector<Message> messages;
  std::string raw;
};

// Calculates the MCP 2.0 checksum over bytes 6..n: ACK through LF inclusive.
std::uint8_t calculate_checksum(std::string_view ack_through_lf);
Frame parse_frame(std::string_view frame);

struct GeneralMountStatus {
  int control_source{};
  int major_status{};
  int motion_status{};
  int built_in_test_status{};
  int error_level{};
  std::string raw_argument;
};

struct ExtendedMountStatus {
  int action_unit_mode{};
  int hydraulic_valves_status{};
  int inertial_navigation_support{};
  int reserved_digit_4{};
  int reserved_digit_5{};
  std::string raw_argument;
};

struct ErrorGroups {
  std::array<std::string, 4U> raw_groups;
  std::array<std::uint16_t, 4U> error_bits{};
  std::string raw_argument;
};

struct Telemetry {
  std::optional<double> roll_deg;
  std::optional<double> pitch_deg;
  std::optional<double> yaw_deg;
  std::optional<double> event_roll_deg;
  std::optional<double> event_pitch_deg;
  std::optional<double> event_yaw_deg;
  std::optional<std::int64_t> timer_ms;
  std::optional<std::int64_t> event_timer_ms;
  std::optional<GeneralMountStatus> general_status;
  std::optional<ExtendedMountStatus> extended_status;
  std::optional<ErrorGroups> error_groups;
  std::vector<Message> unhandled_messages;
};

// Decodes only fields explicitly defined by ICD 323403-901-08/02. Unknown
// commands and fields remain available in unhandled_messages without guesses.
Telemetry decode_telemetry(const Frame& frame);

// Transport-independent byte-stream framing. It never opens or writes a port.
// Noise is discarded until "VM"; complete candidates end at CRLF. A new "VM"
// before CRLF resynchronizes a truncated frame.
class StreamDecoder {
 public:
  std::vector<std::string> feed(std::string_view bytes);
  void reset() noexcept;
  std::size_t buffered_bytes() const noexcept;
  std::size_t discarded_noise_bytes() const noexcept;
  std::size_t discarded_oversize_frames() const noexcept;
  std::size_t discarded_truncated_frames() const noexcept;

 private:
  std::string buffer_;
  std::size_t discarded_noise_bytes_{};
  std::size_t discarded_oversize_frames_{};
  std::size_t discarded_truncated_frames_{};
};

}  // namespace ppbng_rsm400
