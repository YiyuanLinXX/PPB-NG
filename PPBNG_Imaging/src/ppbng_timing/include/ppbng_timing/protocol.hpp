#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ppbng_timing {

constexpr std::uint8_t kProtocolVersion = 1U;
constexpr std::size_t kMaximumPayloadBytes = 1024U;
constexpr std::size_t kPacketOverheadBytes = 14U;
constexpr std::uint32_t kCapabilityStartAtCurrentTick = 1U << 0U;

enum class MessageType : std::uint8_t {
  kGetVersion = 0x01,
  kVersionReport = 0x02,
  kConfigureSchedule = 0x10,
  kFreezeConfiguration = 0x11,
  kArmNextWholeSecond = 0x12,
  kDisarm = 0x13,
  kDisarmKeepConfiguration = 0x14,
  kStartAtCurrentTick = 0x15,
  kPpsAnchor = 0x20,
  kTriggerEvent = 0x21,
  kAck = 0x30,
  kError = 0x31,
  kStatus = 0x32,
};

enum class Channel : std::uint8_t {
  kFx10e = 1,
  kSwir = 2,
  kRgb = 3,
  kThermal = 4,
};

enum class TimeLock : std::uint8_t {
  kUnsynced = 0,
  kLocked = 1,
  kHoldover = 2,
};

enum class ControllerState : std::uint8_t {
  kIdle = 0,
  kConfigured = 1,
  kFrozen = 2,
  kWaitingForPps = 3,
  kRunning = 4,
  kFault = 5,
};

struct Packet {
  std::uint8_t protocol_version{kProtocolVersion};
  MessageType type{};
  std::uint32_t sequence{};
  std::vector<std::uint8_t> payload;
};

std::uint32_t crc32_ieee(const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> serialize_packet(const Packet& packet);
Packet parse_packet(const std::vector<std::uint8_t>& bytes);

class StreamDecoder {
 public:
  std::vector<Packet> feed(const std::uint8_t* data, std::size_t size);
  std::vector<Packet> feed(const std::vector<std::uint8_t>& bytes);
  void reset() noexcept;
  std::size_t buffered_bytes() const noexcept;
  std::size_t discarded_noise_bytes() const noexcept;
  std::size_t crc_or_format_errors() const noexcept;

 private:
  std::vector<std::uint8_t> buffer_;
  std::size_t discarded_noise_bytes_{};
  std::size_t crc_or_format_errors_{};
};

struct VersionInfo {
  std::uint16_t firmware_major{};
  std::uint16_t firmware_minor{};
  std::uint16_t firmware_patch{};
  std::uint32_t boot_id{};
  std::uint32_t capabilities{};
  std::uint32_t ticks_per_second{};
};

struct ChannelSchedule {
  Channel channel{};
  bool enabled{};
  std::uint32_t rate_numerator_hz{};
  std::uint32_t rate_denominator{};
  std::uint32_t pulse_width_ticks{};
  std::uint32_t phase_ticks{};
};

struct ScheduleConfig {
  std::uint32_t schedule_id{};
  std::uint32_t ticks_per_second{};
  std::vector<ChannelSchedule> channels;
};

struct ArmRequest {
  std::uint32_t schedule_id{};
  std::uint64_t after_pps_sequence{};
};

struct StartAtCurrentTickRequest {
  std::uint32_t schedule_id{};
};

struct PpsAnchor {
  std::uint32_t boot_id{};
  std::uint64_t pps_sequence{};
  std::int64_t utc_second{};
  std::uint64_t captured_tick{};
  TimeLock lock{};
};

struct TriggerEvent {
  std::uint32_t boot_id{};
  std::uint64_t event_sequence{};
  Channel channel{};
  std::uint64_t channel_sequence{};
  std::uint64_t pps_sequence{};
  std::uint64_t offset_ticks{};
  std::uint32_t ticks_per_second{};
  TimeLock lock{};
};

struct Ack {
  std::uint32_t acknowledged_sequence{};
  std::uint16_t code{};
  ControllerState state{};
  std::string detail;
};

struct ErrorReport {
  std::uint32_t related_sequence{};
  std::uint16_t code{};
  std::uint8_t severity{};
  std::string detail;
};

struct StatusReport {
  std::uint32_t boot_id{};
  ControllerState state{};
  std::uint32_t active_schedule_id{};
  std::uint32_t last_packet_sequence{};
  std::uint64_t last_pps_sequence{};
  std::uint64_t current_tick{};
  TimeLock lock{};
  std::uint32_t missed_pps_count{};
  std::uint32_t error_flags{};
};

void validate_schedule(const ScheduleConfig& config);

std::vector<std::uint8_t> encode(const VersionInfo& value);
std::vector<std::uint8_t> encode(const ScheduleConfig& value);
std::vector<std::uint8_t> encode(const ArmRequest& value);
std::vector<std::uint8_t> encode(const StartAtCurrentTickRequest& value);
std::vector<std::uint8_t> encode(const PpsAnchor& value);
std::vector<std::uint8_t> encode(const TriggerEvent& value);
std::vector<std::uint8_t> encode(const Ack& value);
std::vector<std::uint8_t> encode(const ErrorReport& value);
std::vector<std::uint8_t> encode(const StatusReport& value);

VersionInfo decode_version_info(const std::vector<std::uint8_t>& payload);
ScheduleConfig decode_schedule_config(const std::vector<std::uint8_t>& payload);
ArmRequest decode_arm_request(const std::vector<std::uint8_t>& payload);
StartAtCurrentTickRequest decode_start_at_current_tick_request(
    const std::vector<std::uint8_t>& payload);
PpsAnchor decode_pps_anchor(const std::vector<std::uint8_t>& payload);
TriggerEvent decode_trigger_event(const std::vector<std::uint8_t>& payload);
Ack decode_ack(const std::vector<std::uint8_t>& payload);
ErrorReport decode_error_report(const std::vector<std::uint8_t>& payload);
StatusReport decode_status_report(const std::vector<std::uint8_t>& payload);

}  // namespace ppbng_timing
