#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace ppbng_storage
{

struct SessionIdentity
{
  // UTC format: YYYYMMDDTHHMMSSZ. Unique ID format: lowercase UUID.
  std::string utc_timestamp;
  std::string unique_id;
};

SessionIdentity make_session_identity_now();
std::string sanitize_windows_component(std::string_view input);
bool is_valid_session_identity(const SessionIdentity & identity) noexcept;

struct DatasetSessionPaths
{
  std::filesystem::path output_root;
  std::filesystem::path session_directory;
  std::filesystem::path manifest_path;
  std::filesystem::path segments_directory;
  std::filesystem::path segment_index_path;
  std::filesystem::path checkpoint_path;
};

enum class DatasetSessionError
{
  none,
  empty_user_name,
  invalid_identity,
  output_root_not_directory,
  unsafe_path,
  already_exists,
  io_error,
};

struct DatasetSessionResult
{
  DatasetSessionError error{DatasetSessionError::none};
  DatasetSessionPaths paths{};
  std::string detail;

  bool ok() const noexcept {return error == DatasetSessionError::none;}
};

// The output root must already exist. The function creates exactly one new direct child and its
// empty segments directory. It never reuses or overwrites an existing session directory.
DatasetSessionResult create_dataset_session(
  const std::filesystem::path & output_root,
  std::string_view user_name,
  const SessionIdentity & identity) noexcept;

enum class AtomicWriteError
{
  none,
  invalid_root,
  unsafe_target,
  target_parent_missing,
  temporary_file_exists,
  io_error,
};

struct AtomicWriteResult
{
  AtomicWriteError error{AtomicWriteError::none};
  std::string detail;

  bool ok() const noexcept {return error == AtomicWriteError::none;}
};

// Writes to a uniquely named sibling temporary file and atomically replaces relative_target.
// relative_target must stay beneath session_root. The caller supplies an operation ID so tests and
// logs can identify the exact temporary file without using a broad cleanup pattern.
AtomicWriteResult atomic_write_text(
  const std::filesystem::path & session_root,
  const std::filesystem::path & relative_target,
  std::string_view contents,
  std::string_view operation_id) noexcept;

// Explicit throughput-first variant. Atomic replacement is retained, but false
// does not request Windows write-through or a FlushFileBuffers durability barrier.
AtomicWriteResult atomic_write_text_with_durability(
  const std::filesystem::path & session_root,
  const std::filesystem::path & relative_target,
  std::string_view contents, std::string_view operation_id, bool durable) noexcept;

struct SegmentRecord
{
  std::uint64_t segment_index{0};
  std::string relative_path;
  std::uint64_t first_event_id{0};
  std::uint64_t last_event_id{0};
  std::uint64_t frame_count{0};
};

enum class SegmentIndexError
{
  none,
  unexpected_segment_index,
  unsafe_relative_path,
  invalid_event_range,
  zero_frame_count,
};

class SegmentIndex
{
public:
  SegmentIndexError append(const SegmentRecord & record);
  const std::vector<SegmentRecord> & records() const noexcept {return records_;}
  std::string serialize_tsv() const;

private:
  std::vector<SegmentRecord> records_;
};

struct DatasetCheckpoint
{
  std::uint64_t next_segment_index{0};
  std::uint64_t committed_frame_count{0};
  std::uint64_t last_committed_event_id{0};
};

std::string serialize_checkpoint(const DatasetCheckpoint & checkpoint);
bool parse_checkpoint(std::string_view text, DatasetCheckpoint & checkpoint) noexcept;

}  // namespace ppbng_storage
