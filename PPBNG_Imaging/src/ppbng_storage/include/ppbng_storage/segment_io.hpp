#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ppbng_storage
{
enum class TimeQuality : std::uint8_t {invalid=0, holdover=1, locked=2};
struct FrameEnvelope
{
  std::uint64_t sample_id{0};
  std::uint64_t trigger_id{0};
  std::uint64_t pps_sequence{0};
  std::int64_t utc_nanoseconds{0};
  TimeQuality time_quality{TimeQuality::invalid};
};
enum class SegmentIoError {none,unsafe_path,already_exists,open_failed,write_failed,
    flush_failed,invalid_file_header,truncated_tail,checksum_error,format_error,end_of_file};
struct SegmentStatus {SegmentIoError error{SegmentIoError::none};std::string detail;std::uint64_t offset{0};bool ok()const noexcept{return error==SegmentIoError::none;}};
struct SegmentCheckpoint {std::uint64_t committed_records{0};std::uint64_t committed_bytes{0};std::uint64_t last_sample_id{0};};
struct SegmentWriterOptions
{
  // Test-only deterministic write failure. Production leaves this at max().
  std::uint64_t fail_after_total_bytes{std::numeric_limits<std::uint64_t>::max()};
};
class SegmentWriter
{
public:
  static std::pair<SegmentStatus,std::unique_ptr<SegmentWriter>> create(
    const std::filesystem::path & session_root,const std::filesystem::path & relative_segment,
    SegmentWriterOptions options={});
  ~SegmentWriter();
  SegmentWriter(const SegmentWriter&)=delete;SegmentWriter&operator=(const SegmentWriter&)=delete;
  SegmentStatus append(const FrameEnvelope&,const std::byte * payload,std::size_t payload_size);
  SegmentStatus flush();
  std::pair<SegmentStatus,SegmentCheckpoint> checkpoint();
  std::uint64_t committed_bytes()const noexcept{return committed_bytes_;}
private:
  SegmentWriter(std::filesystem::path path,SegmentWriterOptions options);
  SegmentStatus open_exclusive();
  SegmentStatus write_bytes(const std::byte*,std::size_t);
  std::filesystem::path path_;SegmentWriterOptions options_;std::ofstream stream_;
  std::uint64_t physical_bytes_{0},committed_bytes_{0},committed_records_{0},last_sample_id_{0};
};
struct SegmentFrame {FrameEnvelope envelope;std::vector<std::byte> payload;std::uint64_t record_offset{0};};
class SegmentReader
{
public:
  static std::pair<SegmentStatus,std::unique_ptr<SegmentReader>> open(
    const std::filesystem::path & session_root,const std::filesystem::path & relative_segment);
  SegmentStatus next(SegmentFrame&);
  SegmentStatus scan_to_end();
  std::uint64_t last_committed_offset()const noexcept{return last_committed_offset_;}
private:
  explicit SegmentReader(std::filesystem::path path):path_(std::move(path)){}
  SegmentStatus open_file();
  std::filesystem::path path_;std::ifstream stream_;std::uint64_t offset_{0},last_committed_offset_{0};
};
SegmentStatus recover_truncated_segment(const std::filesystem::path & session_root,
  const std::filesystem::path & relative_segment,std::uint64_t & recovered_size);
class SegmentRolloverPolicy
{
public: explicit SegmentRolloverPolicy(std::uint64_t maximum_bytes):maximum_bytes_(maximum_bytes){}
  bool should_rollover(std::uint64_t committed_bytes,std::uint64_t next_payload_bytes)const noexcept;
  static constexpr std::uint64_t record_overhead_bytes()noexcept{return 96;}
private:std::uint64_t maximum_bytes_;
};
std::uint32_t payload_crc32(const std::byte * data,std::size_t size)noexcept;
} // namespace ppbng_storage
