#include "ppbng_timing/protocol.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace ppbng_timing {
namespace {

constexpr std::uint8_t kMagic0 = 0x50U;  // P
constexpr std::uint8_t kMagic1 = 0x54U;  // T

class Writer {
 public:
  void u8(std::uint8_t value) { bytes_.push_back(value); }
  void u16(std::uint16_t value) {
    u8(static_cast<std::uint8_t>(value));
    u8(static_cast<std::uint8_t>(value >> 8U));
  }
  void u32(std::uint32_t value) {
    for (unsigned shift = 0U; shift < 32U; shift += 8U) {
      u8(static_cast<std::uint8_t>(value >> shift));
    }
  }
  void u64(std::uint64_t value) {
    for (unsigned shift = 0U; shift < 64U; shift += 8U) {
      u8(static_cast<std::uint8_t>(value >> shift));
    }
  }
  void i64(std::int64_t value) { u64(static_cast<std::uint64_t>(value)); }
  void text(std::string_view value) {
    if (value.size() > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument("protocol text field is too long");
    }
    u16(static_cast<std::uint16_t>(value.size()));
    bytes_.insert(bytes_.end(), value.begin(), value.end());
  }
  void append(const std::vector<std::uint8_t>& value) {
    bytes_.insert(bytes_.end(), value.begin(), value.end());
  }
  std::vector<std::uint8_t> take() { return std::move(bytes_); }

 private:
  std::vector<std::uint8_t> bytes_;
};

class Reader {
 public:
  explicit Reader(const std::vector<std::uint8_t>& bytes) : bytes_(bytes) {}
  std::uint8_t u8() {
    require(1U);
    return bytes_[offset_++];
  }
  std::uint16_t u16() {
    std::uint16_t value = u8();
    value |= static_cast<std::uint16_t>(u8()) << 8U;
    return value;
  }
  std::uint32_t u32() {
    std::uint32_t value = 0U;
    for (unsigned shift = 0U; shift < 32U; shift += 8U) {
      value |= static_cast<std::uint32_t>(u8()) << shift;
    }
    return value;
  }
  std::uint64_t u64() {
    std::uint64_t value = 0U;
    for (unsigned shift = 0U; shift < 64U; shift += 8U) {
      value |= static_cast<std::uint64_t>(u8()) << shift;
    }
    return value;
  }
  std::int64_t i64() { return static_cast<std::int64_t>(u64()); }
  std::string text() {
    const auto size = u16();
    require(size);
    std::string result(bytes_.begin() + static_cast<std::ptrdiff_t>(offset_),
                       bytes_.begin() + static_cast<std::ptrdiff_t>(offset_ + size));
    offset_ += size;
    return result;
  }
  void require_end() const {
    if (offset_ != bytes_.size()) {
      throw std::invalid_argument("unexpected trailing protocol payload bytes");
    }
  }

 private:
  void require(std::size_t count) const {
    if (count > bytes_.size() - std::min(offset_, bytes_.size())) {
      throw std::invalid_argument("truncated protocol payload");
    }
  }
  const std::vector<std::uint8_t>& bytes_;
  std::size_t offset_{};
};

Channel decode_channel(std::uint8_t raw) {
  if (raw < static_cast<std::uint8_t>(Channel::kFx10e) ||
      raw > static_cast<std::uint8_t>(Channel::kThermal)) {
    throw std::invalid_argument("unknown timing channel");
  }
  return static_cast<Channel>(raw);
}

TimeLock decode_lock(std::uint8_t raw) {
  if (raw > static_cast<std::uint8_t>(TimeLock::kHoldover)) {
    throw std::invalid_argument("unknown time-lock value");
  }
  return static_cast<TimeLock>(raw);
}

ControllerState decode_state(std::uint8_t raw) {
  if (raw > static_cast<std::uint8_t>(ControllerState::kFault)) {
    throw std::invalid_argument("unknown controller state");
  }
  return static_cast<ControllerState>(raw);
}

MessageType decode_message_type(std::uint8_t raw) {
  switch (static_cast<MessageType>(raw)) {
    case MessageType::kGetVersion:
    case MessageType::kVersionReport:
    case MessageType::kConfigureSchedule:
    case MessageType::kFreezeConfiguration:
    case MessageType::kArmNextWholeSecond:
    case MessageType::kDisarm:
    case MessageType::kDisarmKeepConfiguration:
    case MessageType::kStartAtCurrentTick:
    case MessageType::kPpsAnchor:
    case MessageType::kTriggerEvent:
    case MessageType::kAck:
    case MessageType::kError:
    case MessageType::kStatus:
      return static_cast<MessageType>(raw);
  }
  throw std::invalid_argument("unknown timing protocol message type");
}

std::uint16_t read_u16_at(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return static_cast<std::uint16_t>(bytes[offset]) |
         (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U);
}

}  // namespace

std::uint32_t crc32_ieee(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 0U; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

std::vector<std::uint8_t> serialize_packet(const Packet& packet) {
  if (packet.protocol_version != kProtocolVersion) {
    throw std::invalid_argument("unsupported timing protocol version");
  }
  decode_message_type(static_cast<std::uint8_t>(packet.type));
  if (packet.payload.size() > kMaximumPayloadBytes) {
    throw std::invalid_argument("timing protocol payload exceeds limit");
  }
  Writer writer;
  writer.u8(kMagic0);
  writer.u8(kMagic1);
  writer.u8(packet.protocol_version);
  writer.u8(static_cast<std::uint8_t>(packet.type));
  writer.u16(static_cast<std::uint16_t>(packet.payload.size()));
  writer.u32(packet.sequence);
  writer.append(packet.payload);
  auto bytes = writer.take();
  const auto crc = crc32_ieee(bytes.data() + 2U, bytes.size() - 2U);
  Writer crc_writer;
  crc_writer.u32(crc);
  const auto crc_bytes = crc_writer.take();
  bytes.insert(bytes.end(), crc_bytes.begin(), crc_bytes.end());
  return bytes;
}

Packet parse_packet(const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() < kPacketOverheadBytes) {
    throw std::invalid_argument("truncated timing protocol packet");
  }
  if (bytes[0] != kMagic0 || bytes[1] != kMagic1) {
    throw std::invalid_argument("invalid timing protocol magic");
  }
  if (bytes[2] != kProtocolVersion) {
    throw std::invalid_argument("unsupported timing protocol version");
  }
  const auto payload_size = read_u16_at(bytes, 4U);
  if (payload_size > kMaximumPayloadBytes ||
      bytes.size() != kPacketOverheadBytes + payload_size) {
    throw std::invalid_argument("invalid timing protocol payload length");
  }
  const auto crc_offset = bytes.size() - 4U;
  const std::uint32_t received_crc =
      static_cast<std::uint32_t>(bytes[crc_offset]) |
      (static_cast<std::uint32_t>(bytes[crc_offset + 1U]) << 8U) |
      (static_cast<std::uint32_t>(bytes[crc_offset + 2U]) << 16U) |
      (static_cast<std::uint32_t>(bytes[crc_offset + 3U]) << 24U);
  const auto calculated_crc = crc32_ieee(bytes.data() + 2U, crc_offset - 2U);
  if (received_crc != calculated_crc) {
    throw std::invalid_argument("timing protocol CRC mismatch");
  }

  Packet packet;
  packet.protocol_version = bytes[2];
  packet.type = decode_message_type(bytes[3]);
  packet.sequence =
      static_cast<std::uint32_t>(bytes[6]) |
      (static_cast<std::uint32_t>(bytes[7]) << 8U) |
      (static_cast<std::uint32_t>(bytes[8]) << 16U) |
      (static_cast<std::uint32_t>(bytes[9]) << 24U);
  packet.payload.assign(bytes.begin() + 10, bytes.begin() + static_cast<std::ptrdiff_t>(crc_offset));
  return packet;
}

std::vector<Packet> StreamDecoder::feed(const std::uint8_t* data, std::size_t size) {
  if (size > 0U && data == nullptr) {
    throw std::invalid_argument("null timing stream input");
  }
  if (size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }
  std::vector<Packet> packets;
  while (true) {
    std::size_t start = buffer_.size();
    for (std::size_t i = 0U; i + 1U < buffer_.size(); ++i) {
      if (buffer_[i] == kMagic0 && buffer_[i + 1U] == kMagic1) {
        start = i;
        break;
      }
    }
    if (start == buffer_.size()) {
      if (!buffer_.empty() && buffer_.back() == kMagic0) {
        discarded_noise_bytes_ += buffer_.size() - 1U;
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        discarded_noise_bytes_ += buffer_.size();
        buffer_.clear();
      }
      break;
    }
    if (start > 0U) {
      discarded_noise_bytes_ += start;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start));
    }
    if (buffer_.size() < 6U) {
      break;
    }
    const auto payload_size = read_u16_at(buffer_, 4U);
    if (payload_size > kMaximumPayloadBytes) {
      ++crc_or_format_errors_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const auto total_size = kPacketOverheadBytes + payload_size;
    if (buffer_.size() < total_size) {
      break;
    }
    std::vector<std::uint8_t> candidate(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total_size));
    try {
      packets.push_back(parse_packet(candidate));
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total_size));
    } catch (const std::invalid_argument&) {
      ++crc_or_format_errors_;
      buffer_.erase(buffer_.begin());
    }
  }
  return packets;
}

std::vector<Packet> StreamDecoder::feed(const std::vector<std::uint8_t>& bytes) {
  return feed(bytes.data(), bytes.size());
}
void StreamDecoder::reset() noexcept { buffer_.clear(); }
std::size_t StreamDecoder::buffered_bytes() const noexcept { return buffer_.size(); }
std::size_t StreamDecoder::discarded_noise_bytes() const noexcept {
  return discarded_noise_bytes_;
}
std::size_t StreamDecoder::crc_or_format_errors() const noexcept {
  return crc_or_format_errors_;
}

void validate_schedule(const ScheduleConfig& config) {
  if (config.schedule_id == 0U) {
    throw std::invalid_argument("schedule ID zero is reserved");
  }
  if (config.ticks_per_second == 0U) {
    throw std::invalid_argument("tick rate must be positive");
  }
  if (config.channels.size() != 4U) {
    throw std::invalid_argument("schedule must define exactly four logical channels");
  }
  std::set<std::uint8_t> seen;
  for (const auto& channel : config.channels) {
    const auto raw_channel = static_cast<std::uint8_t>(channel.channel);
    decode_channel(raw_channel);
    if (!seen.insert(raw_channel).second) {
      throw std::invalid_argument("duplicate timing channel");
    }
    if (channel.rate_denominator == 0U) {
      throw std::invalid_argument("channel rate denominator must be positive");
    }
    if (channel.enabled && channel.rate_numerator_hz == 0U) {
      throw std::invalid_argument("enabled channel rate must be positive");
    }
    if (!channel.enabled && channel.rate_numerator_hz != 0U) {
      throw std::invalid_argument("disabled channel rate must be zero");
    }
    if (channel.pulse_width_ticks == 0U) {
      throw std::invalid_argument("pulse width must be positive");
    }
    if (channel.phase_ticks >= config.ticks_per_second) {
      throw std::invalid_argument("channel phase must be within one PPS interval");
    }
    if (channel.enabled) {
      const long double rate =
          static_cast<long double>(channel.rate_numerator_hz) /
          static_cast<long double>(channel.rate_denominator);
      if (rate > static_cast<long double>(config.ticks_per_second)) {
        throw std::invalid_argument("channel rate exceeds controller tick rate");
      }
      const long double period_ticks =
          static_cast<long double>(config.ticks_per_second) / rate;
      if (static_cast<long double>(channel.pulse_width_ticks) >= period_ticks) {
        throw std::invalid_argument("pulse width must be shorter than trigger period");
      }
    }
  }

  const auto find_channel = [&config](Channel target) -> const ChannelSchedule& {
    return *std::find_if(
        config.channels.begin(), config.channels.end(),
        [target](const ChannelSchedule& item) { return item.channel == target; });
  };
  const auto& rgb = find_channel(Channel::kRgb);
  const auto& thermal = find_channel(Channel::kThermal);
  if (rgb.enabled != thermal.enabled ||
      (rgb.enabled &&
       (rgb.rate_numerator_hz != thermal.rate_numerator_hz ||
        rgb.rate_denominator != thermal.rate_denominator ||
        rgb.pulse_width_ticks != thermal.pulse_width_ticks ||
        rgb.phase_ticks != thermal.phase_ticks))) {
    throw std::invalid_argument(
        "RGB and thermal schedules must match because they share one physical trigger edge");
  }
}

std::vector<std::uint8_t> encode(const VersionInfo& value) {
  Writer w;
  w.u16(value.firmware_major); w.u16(value.firmware_minor); w.u16(value.firmware_patch);
  w.u32(value.boot_id); w.u32(value.capabilities); w.u32(value.ticks_per_second);
  return w.take();
}

std::vector<std::uint8_t> encode(const ScheduleConfig& value) {
  validate_schedule(value);
  Writer w;
  w.u32(value.schedule_id); w.u32(value.ticks_per_second);
  w.u8(static_cast<std::uint8_t>(value.channels.size()));
  for (const auto& channel : value.channels) {
    w.u8(static_cast<std::uint8_t>(channel.channel));
    w.u8(channel.enabled ? 1U : 0U);
    w.u32(channel.rate_numerator_hz); w.u32(channel.rate_denominator);
    w.u32(channel.pulse_width_ticks); w.u32(channel.phase_ticks);
  }
  return w.take();
}

std::vector<std::uint8_t> encode(const ArmRequest& value) {
  Writer w; w.u32(value.schedule_id); w.u64(value.after_pps_sequence); return w.take();
}
std::vector<std::uint8_t> encode(const StartAtCurrentTickRequest& value) {
  Writer w; w.u32(value.schedule_id); return w.take();
}
std::vector<std::uint8_t> encode(const PpsAnchor& value) {
  Writer w; w.u32(value.boot_id); w.u64(value.pps_sequence); w.i64(value.utc_second);
  w.u64(value.captured_tick); w.u8(static_cast<std::uint8_t>(value.lock)); return w.take();
}
std::vector<std::uint8_t> encode(const TriggerEvent& value) {
  Writer w; w.u32(value.boot_id); w.u64(value.event_sequence);
  w.u8(static_cast<std::uint8_t>(value.channel)); w.u64(value.channel_sequence);
  w.u64(value.pps_sequence); w.u64(value.offset_ticks); w.u32(value.ticks_per_second);
  w.u8(static_cast<std::uint8_t>(value.lock)); return w.take();
}
std::vector<std::uint8_t> encode(const Ack& value) {
  Writer w; w.u32(value.acknowledged_sequence); w.u16(value.code);
  w.u8(static_cast<std::uint8_t>(value.state)); w.text(value.detail); return w.take();
}
std::vector<std::uint8_t> encode(const ErrorReport& value) {
  Writer w; w.u32(value.related_sequence); w.u16(value.code); w.u8(value.severity);
  w.text(value.detail); return w.take();
}
std::vector<std::uint8_t> encode(const StatusReport& value) {
  Writer w; w.u32(value.boot_id); w.u8(static_cast<std::uint8_t>(value.state));
  w.u32(value.active_schedule_id); w.u32(value.last_packet_sequence);
  w.u64(value.last_pps_sequence); w.u64(value.current_tick);
  w.u8(static_cast<std::uint8_t>(value.lock)); w.u32(value.missed_pps_count);
  w.u32(value.error_flags); return w.take();
}

VersionInfo decode_version_info(const std::vector<std::uint8_t>& p) {
  Reader r(p); VersionInfo v; v.firmware_major=r.u16(); v.firmware_minor=r.u16();
  v.firmware_patch=r.u16(); v.boot_id=r.u32(); v.capabilities=r.u32();
  v.ticks_per_second=r.u32(); r.require_end(); return v;
}
ScheduleConfig decode_schedule_config(const std::vector<std::uint8_t>& p) {
  Reader r(p); ScheduleConfig v; v.schedule_id=r.u32(); v.ticks_per_second=r.u32();
  const auto count=r.u8();
  for (std::uint8_t i=0U; i<count; ++i) {
    ChannelSchedule c; c.channel=decode_channel(r.u8());
    const auto enabled=r.u8(); if (enabled>1U) throw std::invalid_argument("invalid enabled flag");
    c.enabled=enabled==1U; c.rate_numerator_hz=r.u32(); c.rate_denominator=r.u32();
    c.pulse_width_ticks=r.u32(); c.phase_ticks=r.u32(); v.channels.push_back(c);
  }
  r.require_end(); validate_schedule(v); return v;
}
ArmRequest decode_arm_request(const std::vector<std::uint8_t>& p) {
  Reader r(p); ArmRequest v; v.schedule_id=r.u32(); v.after_pps_sequence=r.u64();
  r.require_end(); return v;
}
StartAtCurrentTickRequest decode_start_at_current_tick_request(
    const std::vector<std::uint8_t>& p) {
  Reader r(p); StartAtCurrentTickRequest v; v.schedule_id=r.u32();
  r.require_end(); return v;
}
PpsAnchor decode_pps_anchor(const std::vector<std::uint8_t>& p) {
  Reader r(p); PpsAnchor v; v.boot_id=r.u32(); v.pps_sequence=r.u64();
  v.utc_second=r.i64(); v.captured_tick=r.u64(); v.lock=decode_lock(r.u8());
  r.require_end(); return v;
}
TriggerEvent decode_trigger_event(const std::vector<std::uint8_t>& p) {
  Reader r(p); TriggerEvent v; v.boot_id=r.u32(); v.event_sequence=r.u64();
  v.channel=decode_channel(r.u8()); v.channel_sequence=r.u64(); v.pps_sequence=r.u64();
  v.offset_ticks=r.u64(); v.ticks_per_second=r.u32(); v.lock=decode_lock(r.u8());
  r.require_end(); return v;
}
Ack decode_ack(const std::vector<std::uint8_t>& p) {
  Reader r(p); Ack v; v.acknowledged_sequence=r.u32(); v.code=r.u16();
  v.state=decode_state(r.u8()); v.detail=r.text(); r.require_end(); return v;
}
ErrorReport decode_error_report(const std::vector<std::uint8_t>& p) {
  Reader r(p); ErrorReport v; v.related_sequence=r.u32(); v.code=r.u16();
  v.severity=r.u8(); v.detail=r.text(); r.require_end(); return v;
}
StatusReport decode_status_report(const std::vector<std::uint8_t>& p) {
  Reader r(p); StatusReport v; v.boot_id=r.u32(); v.state=decode_state(r.u8());
  v.active_schedule_id=r.u32(); v.last_packet_sequence=r.u32();
  v.last_pps_sequence=r.u64(); v.current_tick=r.u64(); v.lock=decode_lock(r.u8());
  v.missed_pps_count=r.u32(); v.error_flags=r.u32(); r.require_end(); return v;
}

}  // namespace ppbng_timing
