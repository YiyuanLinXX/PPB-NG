#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace ppbng_hsi
{

struct EnviPartVerification
{
  bool success{false};
  std::uint64_t source_segment{};
  std::uint64_t sdk_segment{};
  std::uint64_t part{};
  std::uint64_t line_count{};
  std::uint64_t payload_bytes{};
  std::uint64_t bytes_per_line{};
  std::uint64_t crc_lines_checked{};
  std::uint64_t first_source_line{};
  std::uint64_t last_source_line{};
  std::uint64_t first_camera_sequence{};
  std::uint64_t last_camera_sequence{};
  bool first_sequence_gap_before{false};
  std::uint64_t first_missing_trigger_count{};
  std::string message;
};

struct EnviVerificationOptions
{
  // Full mode recalculates every line CRC. Quick mode still validates every
  // index/timestamp/header/extent record and samples raw CRCs at a bounded
  // interval; it must never be reported as a complete payload verification.
  bool full_payload_crc{true};
  std::uint64_t quick_crc_interval_lines{128U};
  std::uint64_t progress_interval_bytes{512ULL * 1024ULL * 1024ULL};
  std::function<void(
    const std::string & part_stem, std::uint64_t processed_bytes,
    std::uint64_t total_bytes)> progress;
};

// Read-only verification of one writer-produced ENVI part. part_stem is the
// filename without .raw/.index.csv, and may contain only the writer's safe
// identifier characters.
EnviPartVerification verify_envi_part(
  const std::filesystem::path & session_directory, const std::string & part_stem,
  const EnviVerificationOptions & options = {});

struct EnviStreamVerification
{
  std::string stream_name;
  std::uint64_t segment_count{};
  std::uint64_t part_count{};
  std::uint64_t line_count{};
  std::uint64_t payload_bytes{};
  std::uint64_t crc_lines_checked{};
};

struct EnviDatasetVerification
{
  bool success{false};
  std::vector<EnviStreamVerification> streams;
  std::string message;
};

EnviDatasetVerification verify_envi_dataset(
  const std::filesystem::path & session_directory,
  const std::vector<std::string> & expected_streams = {"fx10e", "swir"},
  bool reject_unexpected_streams = true,
  const EnviVerificationOptions & options = {});

}  // namespace ppbng_hsi
