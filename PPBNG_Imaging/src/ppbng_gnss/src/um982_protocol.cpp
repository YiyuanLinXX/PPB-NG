#include "ppbng_gnss/um982_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ppbng_gnss {
namespace {

std::string_view trim_line_endings(std::string_view text) {
  while (!text.empty() && (text.back() == '\r' || text.back() == '\n')) {
    text.remove_suffix(1U);
  }
  return text;
}

std::vector<std::string_view> split(std::string_view text, char delimiter) {
  std::vector<std::string_view> fields;
  std::size_t begin = 0U;
  while (true) {
    const auto end = text.find(delimiter, begin);
    if (end == std::string_view::npos) {
      fields.push_back(text.substr(begin));
      return fields;
    }
    fields.push_back(text.substr(begin, end - begin));
    begin = end + 1U;
  }
}

int hex_digit(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'a' && value <= 'f') {
    return 10 + value - 'a';
  }
  if (value >= 'A' && value <= 'F') {
    return 10 + value - 'A';
  }
  throw std::invalid_argument("invalid hexadecimal checksum");
}

std::uint32_t parse_hex(std::string_view text, std::size_t digits) {
  if (text.size() < digits) {
    throw std::invalid_argument("truncated hexadecimal checksum");
  }
  std::uint32_t result = 0U;
  for (std::size_t index = 0U; index < digits; ++index) {
    result = (result << 4U) | static_cast<std::uint32_t>(hex_digit(text[index]));
  }
  return result;
}

double parse_double(std::string_view text, const char* field_name) {
  if (text.empty()) {
    throw std::invalid_argument(std::string("empty ") + field_name);
  }
  std::string owned(text);
  std::size_t parsed = 0U;
  double value = 0.0;
  try {
    value = std::stod(owned, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  if (parsed != owned.size() || !std::isfinite(value)) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  return value;
}

int parse_int(std::string_view text, const char* field_name) {
  if (text.empty()) {
    throw std::invalid_argument(std::string("empty ") + field_name);
  }
  std::string owned(text);
  std::size_t parsed = 0U;
  long value = 0L;
  try {
    value = std::stol(owned, &parsed, 10);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  if (parsed != owned.size() || value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  return static_cast<int>(value);
}

std::uint32_t parse_u32(std::string_view text, const char* field_name) {
  if (text.empty() || text.front() == '-') {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  std::string owned(text);
  std::size_t parsed = 0U;
  unsigned long long value = 0U;
  try {
    value = std::stoull(owned, &parsed, 10);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  if (parsed != owned.size() || value > (std::numeric_limits<std::uint32_t>::max)()) {
    throw std::invalid_argument(std::string("invalid ") + field_name);
  }
  return static_cast<std::uint32_t>(value);
}

std::optional<std::int64_t> receiver_utc_ns(const ReceiverTimeHeader& header) {
  if (header.time_status != "FINE") return std::nullopt;
  constexpr std::uint64_t kWeekMs = 604800000ULL;
  if (header.milliseconds_of_week >= kWeekMs) {
    throw std::invalid_argument("receiver milliseconds of week outside range");
  }
  std::int64_t epoch_seconds = 0;
  if (header.time_reference == "GPS") {
    epoch_seconds = 315964800LL;  // 1980-01-06T00:00:00Z
  } else if (header.time_reference == "BDS") {
    epoch_seconds = 1136073600LL;  // 2006-01-01T00:00:00Z
  } else {
    throw std::invalid_argument("unsupported receiver time reference");
  }
  const std::uint64_t reference_ms =
    static_cast<std::uint64_t>(header.week) * kWeekMs + header.milliseconds_of_week;
  constexpr auto kMax = (std::numeric_limits<std::int64_t>::max)();
  const auto epoch_ns = epoch_seconds * 1000000000LL;
  if (reference_ms > static_cast<std::uint64_t>((kMax - epoch_ns) / 1000000LL)) {
    throw std::invalid_argument("receiver time overflows int64 nanoseconds");
  }
  const auto reference_ns = static_cast<std::int64_t>(reference_ms * 1000000ULL);
  const auto leap_ns = static_cast<std::int64_t>(header.leap_seconds) * 1000000000LL;
  return epoch_ns + reference_ns - leap_ns;
}

double parse_coordinate(
    std::string_view value, std::string_view hemisphere,
    std::size_t degree_digits) {
  if (value.size() <= degree_digits) {
    throw std::invalid_argument("empty NMEA coordinate");
  }
  const double degrees = parse_double(value.substr(0U, degree_digits), "coordinate degrees");
  const double minutes = parse_double(value.substr(degree_digits), "coordinate minutes");
  if (minutes < 0.0 || minutes >= 60.0) {
    throw std::invalid_argument("NMEA coordinate minutes outside range");
  }
  double coordinate = degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    coordinate = -coordinate;
  } else if (hemisphere != "N" && hemisphere != "E") {
    throw std::invalid_argument("invalid NMEA hemisphere");
  }
  return coordinate;
}

std::uint32_t crc32_value(std::uint32_t value) {
  std::uint32_t crc = value;
  for (int bit = 0; bit < 8; ++bit) {
    crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
  }
  return crc;
}

}  // namespace

std::uint8_t nmea_checksum(std::string_view payload) {
  std::uint8_t checksum = 0U;
  for (const unsigned char byte : payload) {
    checksum ^= byte;
  }
  return checksum;
}

std::uint32_t unicore_crc32(std::string_view payload) {
  std::uint32_t crc = 0U;
  for (const unsigned char byte : payload) {
    crc = ((crc >> 8U) & 0x00FFFFFFU) ^ crc32_value((crc ^ byte) & 0xFFU);
  }
  return crc;
}

GgaFix parse_gga(std::string_view sentence) {
  sentence = trim_line_endings(sentence);
  if (sentence.empty() || sentence.front() != '$') {
    throw std::invalid_argument("GGA sentence must start with '$'");
  }
  const auto star = sentence.rfind('*');
  if (star == std::string_view::npos) {
    throw std::invalid_argument("incomplete GGA sentence");
  }
  if (sentence.size() - star - 1U != 2U) {
    throw std::invalid_argument("GGA checksum must contain exactly two hexadecimal digits");
  }
  const auto payload = sentence.substr(1U, star - 1U);
  const auto expected = parse_hex(sentence.substr(star + 1U), 2U);
  if (nmea_checksum(payload) != expected) {
    throw std::invalid_argument("NMEA checksum mismatch");
  }
  const auto fields = split(payload, ',');
  if (fields.size() < 15U || fields[0].size() != 5U ||
      fields[0].substr(2U) != "GGA") {
    throw std::invalid_argument("not a complete GGA sentence");
  }

  GgaFix fix;
  fix.utc_hhmmss = std::string(fields[1]);
  fix.latitude_deg = parse_coordinate(fields[2], fields[3], 2U);
  fix.longitude_deg = parse_coordinate(fields[4], fields[5], 3U);
  fix.quality = parse_int(fields[6], "GGA quality");
  fix.satellites_used = parse_int(fields[7], "GGA satellites used");
  fix.hdop = parse_double(fields[8], "GGA HDOP");
  fix.altitude_m = parse_double(fields[9], "GGA altitude");
  if (!fields[13].empty()) {
    fix.has_differential_age = true;
    fix.differential_age_sec = parse_double(fields[13], "GGA differential age");
  }
  fix.raw_sentence = std::string(sentence);
  return fix;
}

HeadingSolution parse_uniheadinga(std::string_view sentence) {
  sentence = trim_line_endings(sentence);
  constexpr std::string_view prefix{"#UNIHEADINGA,"};
  if (sentence.substr(0U, prefix.size()) != prefix) {
    throw std::invalid_argument("not a UNIHEADINGA sentence");
  }
  const auto star = sentence.rfind('*');
  const auto semicolon = sentence.find(';');
  if (star == std::string_view::npos || semicolon == std::string_view::npos ||
      semicolon >= star) {
    throw std::invalid_argument("incomplete UNIHEADINGA sentence");
  }
  if (sentence.size() - star - 1U != 8U) {
    throw std::invalid_argument("UNIHEADINGA CRC must contain exactly eight hexadecimal digits");
  }
  const auto crc_payload = sentence.substr(1U, star - 1U);
  const auto expected = parse_hex(sentence.substr(star + 1U), 8U);
  if (unicore_crc32(crc_payload) != expected) {
    throw std::invalid_argument("UNIHEADINGA CRC mismatch");
  }
  const auto fields = split(sentence.substr(semicolon + 1U, star - semicolon - 1U), ',');
  if (fields.size() < 14U) {
    throw std::invalid_argument("truncated UNIHEADINGA body");
  }

  HeadingSolution solution;
  const auto header = split(sentence.substr(prefix.size(), semicolon - prefix.size()), ',');
  if (header.size() != 9U) {
    throw std::invalid_argument("UNIHEADINGA header must contain nine fields");
  }
  solution.receiver_time.cpu_idle_percent = parse_int(header[0], "receiver CPU idle");
  if (solution.receiver_time.cpu_idle_percent < 0 ||
    solution.receiver_time.cpu_idle_percent > 100)
  {
    throw std::invalid_argument("receiver CPU idle outside range");
  }
  solution.receiver_time.time_reference = std::string(header[1]);
  solution.receiver_time.time_status = std::string(header[2]);
  const auto week = parse_u32(header[3], "receiver week");
  if (week > (std::numeric_limits<std::uint16_t>::max)()) {
    throw std::invalid_argument("receiver week outside uint16 range");
  }
  solution.receiver_time.week = static_cast<std::uint16_t>(week);
  solution.receiver_time.milliseconds_of_week =
    parse_u32(header[4], "receiver milliseconds of week");
  solution.receiver_time.format_version = parse_u32(header[5], "receiver format version");
  const auto reserved = parse_u32(header[6], "receiver reserved field");
  const auto leap = parse_u32(header[7], "receiver leap seconds");
  const auto delay = parse_u32(header[8], "receiver output delay");
  if (reserved > (std::numeric_limits<std::uint8_t>::max)() ||
    leap > (std::numeric_limits<std::uint8_t>::max)() ||
    delay > (std::numeric_limits<std::uint16_t>::max)())
  {
    throw std::invalid_argument("receiver header narrow field outside range");
  }
  solution.receiver_time.reserved = static_cast<std::uint8_t>(reserved);
  solution.receiver_time.leap_seconds = static_cast<std::uint8_t>(leap);
  solution.receiver_time.output_delay_ms = static_cast<std::uint16_t>(delay);
  solution.receiver_time.utc_unix_nanoseconds = receiver_utc_ns(solution.receiver_time);
  solution.solution_status = std::string(fields[0]);
  solution.position_type = std::string(fields[1]);
  solution.baseline_m = parse_double(fields[2], "heading baseline");
  solution.heading_deg = std::fmod(parse_double(fields[3], "heading angle"), 360.0);
  if (solution.heading_deg < 0.0) {
    solution.heading_deg += 360.0;
  }
  solution.pitch_deg = parse_double(fields[4], "heading pitch");
  solution.heading_stddev_deg = parse_double(fields[6], "heading standard deviation");
  solution.pitch_stddev_deg = parse_double(fields[7], "pitch standard deviation");
  solution.satellites_tracked = parse_int(fields[9], "heading satellites tracked");
  solution.satellites_used = parse_int(fields[10], "heading satellites used");
  solution.raw_sentence = std::string(sentence);
  return solution;
}

IncrementalLineDecoder::IncrementalLineDecoder(std::size_t max_buffer_bytes)
    : max_buffer_bytes_(max_buffer_bytes) {
  if (max_buffer_bytes_ == 0U) {
    throw std::invalid_argument("line decoder buffer limit must be positive");
  }
}

std::vector<std::string> IncrementalLineDecoder::feed(std::string_view bytes) {
  if (bytes.size() > max_buffer_bytes_ - std::min(buffer_.size(), max_buffer_bytes_)) {
    reset();
    throw std::length_error("UM982 input line exceeds decoder buffer limit");
  }
  buffer_.append(bytes.data(), bytes.size());

  std::vector<std::string> lines;
  std::size_t consumed = 0U;
  while (true) {
    const auto newline = buffer_.find('\n', consumed);
    if (newline == std::string::npos) {
      break;
    }
    auto length = newline - consumed;
    if (length > 0U && buffer_[consumed + length - 1U] == '\r') {
      --length;
    }
    lines.emplace_back(buffer_.substr(consumed, length));
    consumed = newline + 1U;
  }
  if (consumed > 0U) {
    buffer_.erase(0U, consumed);
  }
  return lines;
}

void IncrementalLineDecoder::reset() noexcept { buffer_.clear(); }

std::size_t IncrementalLineDecoder::buffered_bytes() const noexcept {
  return buffer_.size();
}

}  // namespace ppbng_gnss
