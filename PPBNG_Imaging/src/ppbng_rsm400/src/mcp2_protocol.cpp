#include "ppbng_rsm400/mcp2_protocol.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <stdexcept>

namespace ppbng_rsm400 {
namespace {

bool is_ascii_digit(char value) { return value >= '0' && value <= '9'; }

int parse_decimal_digit(char value, const char* field_name) {
  if (!is_ascii_digit(value)) {
    throw std::invalid_argument(std::string(field_name) + " must contain decimal digits");
  }
  return value - '0';
}

std::int64_t parse_integer(std::string_view text, const char* field_name) {
  if (text.empty()) {
    throw std::invalid_argument(std::string("empty ") + field_name);
  }
  std::string owned(text);
  std::size_t parsed = 0U;
  long long value = 0;
  try {
    value = std::stoll(owned, &parsed, 10);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  if (parsed != owned.size()) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  return static_cast<std::int64_t>(value);
}

GeneralMountStatus decode_general_status(std::string_view argument) {
  if (argument.size() != 5U) {
    throw std::invalid_argument("MS status must contain exactly five digits");
  }
  GeneralMountStatus status;
  status.control_source = parse_decimal_digit(argument[0], "MS status");
  status.major_status = parse_decimal_digit(argument[1], "MS status");
  status.motion_status = parse_decimal_digit(argument[2], "MS status");
  status.built_in_test_status = parse_decimal_digit(argument[3], "MS status");
  status.error_level = parse_decimal_digit(argument[4], "MS status");
  status.raw_argument = std::string(argument);
  return status;
}

ExtendedMountStatus decode_extended_status(std::string_view argument) {
  if (argument.size() != 5U) {
    throw std::invalid_argument("EMS status must contain exactly five digits");
  }
  ExtendedMountStatus status;
  status.action_unit_mode = parse_decimal_digit(argument[0], "EMS status");
  status.hydraulic_valves_status = parse_decimal_digit(argument[1], "EMS status");
  status.inertial_navigation_support = parse_decimal_digit(argument[2], "EMS status");
  status.reserved_digit_4 = parse_decimal_digit(argument[3], "EMS status");
  status.reserved_digit_5 = parse_decimal_digit(argument[4], "EMS status");
  status.raw_argument = std::string(argument);
  return status;
}

std::uint16_t decode_error_group(std::string_view group) {
  if (group.size() != 3U) {
    throw std::invalid_argument("ERG group must contain exactly three characters");
  }
  std::uint16_t result = 0U;
  for (std::size_t character_index = 0U; character_index < 3U; ++character_index) {
    const auto byte = static_cast<std::uint8_t>(group[character_index]);
    if ((byte & 0xF0U) != 0x30U) {
      throw std::invalid_argument("ERG characters must be in ASCII range 0x30..0x3F");
    }
    const auto nibble = static_cast<std::uint8_t>(byte & 0x0FU);
    for (std::size_t physical_bit = 0U; physical_bit < 4U; ++physical_bit) {
      if ((nibble & (1U << physical_bit)) != 0U) {
        const auto documented_bit = character_index * 4U + (3U - physical_bit);
        result |= static_cast<std::uint16_t>(1U << documented_bit);
      }
    }
  }
  return result;
}

ErrorGroups decode_error_groups(std::string_view argument) {
  ErrorGroups groups;
  groups.raw_argument = std::string(argument);
  std::size_t begin = 0U;
  for (std::size_t index = 0U; index < groups.raw_groups.size(); ++index) {
    const auto end = index + 1U == groups.raw_groups.size()
                         ? argument.size()
                         : argument.find('.', begin);
    if (end == std::string_view::npos) {
      throw std::invalid_argument("ERG must contain four dot-separated groups");
    }
    const auto group = argument.substr(begin, end - begin);
    groups.raw_groups[index] = std::string(group);
    groups.error_bits[index] = decode_error_group(group);
    begin = end + 1U;
  }
  if (begin != argument.size() + 1U) {
    throw std::invalid_argument("ERG contains unexpected extra groups");
  }
  return groups;
}

bool is_command_character(char value) {
  return value >= 'A' && value <= 'Z';
}

Message parse_message(std::string_view raw) {
  if (raw.empty()) {
    throw std::invalid_argument("empty MCP message");
  }
  const auto space = raw.find(' ');
  const auto command_text = raw.substr(0U, space);
  if (command_text.empty() || command_text.size() > 3U ||
      !std::all_of(command_text.begin(), command_text.end(), is_command_character)) {
    throw std::invalid_argument("MCP command must contain one to three uppercase ASCII letters");
  }
  Message message;
  message.command = std::string(command_text);
  message.raw = std::string(raw);
  if (space != std::string_view::npos) {
    const auto argument = raw.substr(space + 1U);
    if (argument.empty() || argument.front() == ' ') {
      throw std::invalid_argument("invalid MCP argument separator");
    }
    message.argument = std::string(argument);
  }
  return message;
}

}  // namespace

std::uint8_t calculate_checksum(std::string_view ack_through_lf) {
  std::uint16_t sum = 0U;
  for (const unsigned char byte : ack_through_lf) {
    sum = static_cast<std::uint16_t>(sum + static_cast<std::uint8_t>(~byte));
  }
  sum = static_cast<std::uint16_t>((sum & 0x00FFU) + ((sum & 0xFF00U) >> 8U));
  return static_cast<std::uint8_t>(sum & 0x00FFU);
}

Frame parse_frame(std::string_view frame) {
  if (frame.size() < 10U) {
    throw std::invalid_argument("truncated MCP frame");
  }
  if (frame.size() > kMcp2MaximumFrameBytes) {
    throw std::invalid_argument("MCP frame exceeds 255-byte receiver limit");
  }
  if (frame.substr(0U, 2U) != "VM") {
    throw std::invalid_argument("MCP frame must start with VM");
  }
  if (!is_ascii_digit(frame[2]) || !is_ascii_digit(frame[3]) || !is_ascii_digit(frame[4])) {
    throw std::invalid_argument("MCP checksum must be three decimal ASCII digits");
  }
  if ((static_cast<std::uint8_t>(frame[5]) & 0xF0U) != 0x40U) {
    throw std::invalid_argument("invalid MCP connection status byte");
  }
  if (frame[6] != ' ' || frame[7] != '/') {
    throw std::invalid_argument("invalid MCP frame header separator");
  }
  if (frame.substr(frame.size() - 2U) != "\r\n" || frame[frame.size() - 3U] != '/') {
    throw std::invalid_argument("MCP frame must end with slash and CRLF");
  }

  const auto received_value =
      (frame[2] - '0') * 100 + (frame[3] - '0') * 10 + (frame[4] - '0');
  if (received_value > 255) {
    throw std::invalid_argument("MCP checksum value is outside the 8-bit range");
  }
  const auto received = static_cast<std::uint8_t>(received_value);
  const auto calculated = calculate_checksum(frame.substr(5U));
  if (received != calculated) {
    throw std::invalid_argument("MCP checksum mismatch");
  }

  Frame result;
  result.received_checksum = received;
  const auto ack = static_cast<std::uint8_t>(frame[5]);
  result.connection_status = ConnectionStatus{
      ack,
      (ack & 0x08U) != 0U,
      (ack & 0x04U) != 0U,
      (ack & 0x02U) != 0U,
      (ack & 0x01U) != 0U};
  result.raw = std::string(frame);

  const auto message_bytes = frame.substr(8U, frame.size() - 10U);
  std::size_t begin = 0U;
  while (begin < message_bytes.size()) {
    const auto slash = message_bytes.find('/', begin);
    if (slash == std::string_view::npos) {
      throw std::invalid_argument("unterminated MCP message");
    }
    result.messages.push_back(parse_message(message_bytes.substr(begin, slash - begin)));
    if (result.messages.size() > kMcp2MaximumMessages) {
      throw std::invalid_argument("MCP frame contains more than ten messages");
    }
    begin = slash + 1U;
  }
  return result;
}

Telemetry decode_telemetry(const Frame& frame) {
  Telemetry telemetry;
  for (const auto& message : frame.messages) {
    const auto require_argument = [&message]() -> const std::string& {
      if (!message.argument.has_value()) {
        throw std::invalid_argument(message.command + " telemetry message requires an argument");
      }
      return *message.argument;
    };

    if (message.command == "GR") {
      telemetry.roll_deg = static_cast<double>(parse_integer(require_argument(), "GR angle")) / 100.0;
    } else if (message.command == "GP") {
      telemetry.pitch_deg = static_cast<double>(parse_integer(require_argument(), "GP angle")) / 100.0;
    } else if (message.command == "GY") {
      telemetry.yaw_deg = static_cast<double>(parse_integer(require_argument(), "GY angle")) / 100.0;
    } else if (message.command == "GRE") {
      telemetry.event_roll_deg = static_cast<double>(parse_integer(require_argument(), "GRE angle")) / 100.0;
    } else if (message.command == "GPE") {
      telemetry.event_pitch_deg = static_cast<double>(parse_integer(require_argument(), "GPE angle")) / 100.0;
    } else if (message.command == "GYE") {
      telemetry.event_yaw_deg = static_cast<double>(parse_integer(require_argument(), "GYE angle")) / 100.0;
    } else if (message.command == "TM") {
      telemetry.timer_ms = parse_integer(require_argument(), "TM timer");
    } else if (message.command == "TME") {
      telemetry.event_timer_ms = parse_integer(require_argument(), "TME timer");
    } else if (message.command == "MS") {
      telemetry.general_status = decode_general_status(require_argument());
    } else if (message.command == "EMS") {
      telemetry.extended_status = decode_extended_status(require_argument());
    } else if (message.command == "ERG") {
      telemetry.error_groups = decode_error_groups(require_argument());
    } else {
      telemetry.unhandled_messages.push_back(message);
    }
  }
  return telemetry;
}

std::vector<std::string> StreamDecoder::feed(std::string_view bytes) {
  buffer_.append(bytes.data(), bytes.size());
  std::vector<std::string> frames;

  while (true) {
    auto start = buffer_.find("VM");
    if (start == std::string::npos) {
      if (!buffer_.empty() && buffer_.back() == 'V') {
        discarded_noise_bytes_ += buffer_.size() - 1U;
        buffer_.erase(0U, buffer_.size() - 1U);
      } else {
        discarded_noise_bytes_ += buffer_.size();
        buffer_.clear();
      }
      break;
    }
    if (start > 0U) {
      discarded_noise_bytes_ += start;
      buffer_.erase(0U, start);
    }

    const auto end = buffer_.find("\r\n", 2U);
    const auto nested_start = buffer_.find("VM", 2U);
    if (nested_start != std::string::npos &&
        (end == std::string::npos || nested_start < end)) {
      discarded_truncated_frames_ += 1U;
      buffer_.erase(0U, nested_start);
      continue;
    }
    if (end == std::string::npos) {
      if (buffer_.size() > kMcp2MaximumFrameBytes) {
        discarded_oversize_frames_ += 1U;
        discarded_noise_bytes_ += buffer_.size();
        buffer_.clear();
      }
      break;
    }

    const auto frame_size = end + 2U;
    if (frame_size <= kMcp2MaximumFrameBytes) {
      frames.emplace_back(buffer_.substr(0U, frame_size));
    } else {
      discarded_oversize_frames_ += 1U;
    }
    buffer_.erase(0U, frame_size);
  }
  return frames;
}

void StreamDecoder::reset() noexcept { buffer_.clear(); }
std::size_t StreamDecoder::buffered_bytes() const noexcept { return buffer_.size(); }
std::size_t StreamDecoder::discarded_noise_bytes() const noexcept {
  return discarded_noise_bytes_;
}
std::size_t StreamDecoder::discarded_oversize_frames() const noexcept {
  return discarded_oversize_frames_;
}
std::size_t StreamDecoder::discarded_truncated_frames() const noexcept {
  return discarded_truncated_frames_;
}

}  // namespace ppbng_rsm400
