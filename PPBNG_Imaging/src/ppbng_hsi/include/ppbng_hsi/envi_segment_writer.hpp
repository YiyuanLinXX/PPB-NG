#pragma once

#include "ppbng_hsi/hsi_types.hpp"
#include "ppbng_hsi/writer_performance.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

namespace ppbng_hsi
{

struct EnviWriterOptions
{
  std::filesystem::path session_directory;
  std::string stream_stem;
  std::uint64_t maximum_segment_bytes{8ULL * 1024ULL * 1024ULL * 1024ULL};
  std::uint64_t flush_every_lines{120U};
  // Deterministic test-only failure injection. Production keeps max().
  std::uint64_t fail_after_payload_bytes{(std::numeric_limits<std::uint64_t>::max)()};
  // Optional application-level checksum; does not control transport checks.
  bool payload_crc_enabled{true};
};

class EnviSegmentWriter
{
public:
  static std::pair<OperationResult, std::unique_ptr<EnviSegmentWriter>> create(
    const EnviWriterOptions & options, const HsiConfig & config);
  ~EnviSegmentWriter();
  EnviSegmentWriter(const EnviSegmentWriter &) = delete;
  EnviSegmentWriter & operator=(const EnviSegmentWriter &) = delete;

  OperationResult append(const LineRecord & line);
  OperationResult flush();
  // Flush/checkpoint and close the current raw+sidecar part while keeping the
  // writer available for a later, strictly new source segment.
  OperationResult finalize_current_segment();
  OperationResult close();
  std::uint32_t current_source_segment() const noexcept {return source_segment_;}
  std::uint32_t current_part() const noexcept {return part_;}
  std::uint64_t committed_lines() const noexcept {return total_lines_;}
  const WriterPerformance & performance() const noexcept {return performance_;}

private:
  EnviSegmentWriter(EnviWriterOptions options, HsiConfig config);
  OperationResult open_part(std::uint32_t source_segment, std::uint32_t part);
  OperationResult finalize_part();
  OperationResult checkpoint();

  EnviWriterOptions options_;
  HsiConfig config_;
  std::ofstream raw_;
  std::ofstream timestamps_;
  std::ofstream index_;
  std::filesystem::path header_relative_;
  std::uint32_t source_segment_{0};
  std::uint32_t part_{0};
  std::uint64_t part_lines_{0};
  std::uint64_t part_payload_bytes_{0};
  std::uint64_t total_lines_{0};
  std::uint64_t total_payload_bytes_{0};
  bool part_open_{false};
  bool closed_{false};
  WriterPerformance performance_;
};

}  // namespace ppbng_hsi
