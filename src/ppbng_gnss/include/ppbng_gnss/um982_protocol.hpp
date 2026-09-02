#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ppbng_gnss {

struct ReceiverTimeHeader {
  int cpu_idle_percent{};
  std::string time_reference;
  std::string time_status;
  std::uint16_t week{};
  std::uint32_t milliseconds_of_week{};
  std::uint32_t format_version{};
  std::uint8_t reserved{};
  std::uint8_t leap_seconds{};
  std::uint16_t output_delay_ms{};
  // Present only when TimeStatus is FINE and every field passes strict range
  // and overflow validation. This is the receiver measurement epoch; output
  // delay is retained separately and is not silently applied.
  std::optional<std::int64_t> utc_unix_nanoseconds;
};

struct GgaFix {
  std::string utc_hhmmss;
  double latitude_deg{};
  double longitude_deg{};
  double altitude_m{};
  int quality{};
  int satellites_used{};
  double hdop{};
  bool has_differential_age{};
  double differential_age_sec{};
  std::string raw_sentence;
};

struct HeadingSolution {
  ReceiverTimeHeader receiver_time;
  std::string solution_status;
  std::string position_type;
  double baseline_m{};
  double heading_deg{};
  double pitch_deg{};
  double heading_stddev_deg{};
  double pitch_stddev_deg{};
  int satellites_tracked{};
  int satellites_used{};
  std::string raw_sentence;
};

// These functions validate the protocol checksum/CRC and throw
// std::invalid_argument for malformed or incomplete input.
GgaFix parse_gga(std::string_view sentence);
HeadingSolution parse_uniheadinga(std::string_view sentence);

std::uint8_t nmea_checksum(std::string_view payload);
std::uint32_t unicore_crc32(std::string_view payload);

// A transport-independent line framer. It performs no serial I/O and can be
// fed arbitrary chunks from a file, test fixture, or a future read-only node.
class IncrementalLineDecoder {
 public:
  explicit IncrementalLineDecoder(std::size_t max_buffer_bytes = 65536U);

  std::vector<std::string> feed(std::string_view bytes);
  void reset() noexcept;
  std::size_t buffered_bytes() const noexcept;

 private:
  std::size_t max_buffer_bytes_;
  std::string buffer_;
};

}  // namespace ppbng_gnss
