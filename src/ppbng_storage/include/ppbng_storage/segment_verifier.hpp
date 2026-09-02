#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ppbng_storage
{

struct SegmentVerificationResult
{
  bool success{false};
  std::uint64_t record_count{0};
  std::uint64_t payload_bytes{0};
  std::uint64_t first_sample_id{0};
  std::uint64_t last_sample_id{0};
  std::string message;
};

// Read-only verification for one ppbng_storage segment. The relative path must
// remain beneath session_root/segments. Every record header, payload and commit
// trailer is checksum-verified by SegmentReader. Sample IDs must be non-zero and
// strictly increasing within the file.
SegmentVerificationResult verify_segment_file(
  const std::filesystem::path & session_root,
  const std::filesystem::path & relative_segment) noexcept;

struct SegmentStreamVerification
{
  struct File
  {
    std::uint64_t segment_index{0};
    std::uint64_t record_count{0};
    std::uint64_t first_sample_id{0};
    std::uint64_t last_sample_id{0};
  };
  std::string stream_name;
  std::uint64_t segment_count{0};
  std::uint64_t record_count{0};
  std::uint64_t payload_bytes{0};
  std::uint64_t last_sample_id{0};
  std::vector<File> files;
};

struct SegmentSetVerificationResult
{
  bool success{false};
  std::vector<SegmentStreamVerification> streams;
  std::string message;
};

// Scans direct *.ppbseg children of session_root/segments without following
// directory entries that are not regular files. Filenames must use
// <stream>_<six-digit-index>.ppbseg. Segment and sample IDs must be contiguous;
// an empty segment is allowed only as the last segment of a stream.
SegmentSetVerificationResult verify_segment_set(
  const std::filesystem::path & session_root,
  const std::vector<std::string> & expected_streams = {"rgb", "thermal"}) noexcept;

}  // namespace ppbng_storage
