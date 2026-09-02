#include "ppbng_storage/association_verifier.hpp"

#include "ppbng_storage/json_verifier.hpp"
#include "ppbng_storage/segment_verifier.hpp"

#include <charconv>
#include <fstream>
#include <limits>
#include <map>

namespace ppbng_storage
{
namespace
{
struct SampleState
{
  std::uint64_t segment{};
  std::uint64_t sdk_frame{};
  std::uint64_t host_ns{};
  std::uint64_t camera_timestamp_ns{};
};

class CommittedSampleCursor
{
public:
  explicit CommittedSampleCursor(const SegmentStreamVerification & stream)
  : stream_(stream) {}

  bool validate(std::string & detail)
  {
    std::uint64_t records = 0U;
    std::uint64_t next_sample = 1U;
    std::uint64_t next_segment = 0U;
    for (std::size_t index = 0U; index < stream_.files.size(); ++index) {
      const auto & file = stream_.files[index];
      if (file.segment_index != next_segment++) {
        detail = "verified stream summary has non-contiguous segment indices";
        return false;
      }
      if (file.record_count == 0U) {
        if (index + 1U != stream_.files.size() || file.first_sample_id != 0U ||
          file.last_sample_id != 0U)
        {
          detail = "verified stream summary has an invalid empty segment";
          return false;
        }
        continue;
      }
      if (file.first_sample_id != next_sample || file.last_sample_id < file.first_sample_id ||
        file.last_sample_id - file.first_sample_id + 1U != file.record_count ||
        records > (std::numeric_limits<std::uint64_t>::max)() - file.record_count)
      {
        detail = "verified stream summary has sparse, overflowing, or non-contiguous samples";
        return false;
      }
      records += file.record_count;
      if (file.last_sample_id == (std::numeric_limits<std::uint64_t>::max)()) {
        if (records != stream_.record_count || index + 1U != stream_.files.size()) {
          detail = "verified stream summary sample range overflows";
          return false;
        }
      } else {
        next_sample = file.last_sample_id + 1U;
      }
    }
    if (records != stream_.record_count || stream_.segment_count != stream_.files.size() ||
      stream_.last_sample_id != (records == 0U ? 0U : records))
    {
      detail = "verified stream summary counts are inconsistent";
      return false;
    }
    return true;
  }

  bool consume(const std::uint64_t segment, const std::uint64_t sample)
  {
    while (file_index_ < stream_.files.size() &&
      stream_.files[file_index_].record_count == 0U)
    {
      ++file_index_;
    }
    if (file_index_ >= stream_.files.size()) {return false;}
    const auto & file = stream_.files[file_index_];
    if (sample_index_ == 0U) {sample_index_ = file.first_sample_id;}
    if (segment != file.segment_index || sample != sample_index_) {return false;}
    ++consumed_;
    if (sample_index_ == file.last_sample_id) {
      ++file_index_;
      sample_index_ = 0U;
    } else {
      ++sample_index_;
    }
    return true;
  }

  std::uint64_t consumed() const noexcept {return consumed_;}

private:
  const SegmentStreamVerification & stream_;
  std::size_t file_index_{0U};
  std::uint64_t sample_index_{0U};
  std::uint64_t consumed_{0U};
};

AssociationVerificationResult failure(
  const std::uint64_t expected, const std::uint64_t pending,
  const std::uint64_t terminal, const std::string & message)
{
  return {false, expected, pending, terminal, 0U, 0U, message};
}

bool safe_name(const std::string & value)
{
  return !value.empty() && value.find_first_not_of(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos;
}

const JsonScalar * member(
  const std::map<std::string, JsonScalar> & members, const std::string & key)
{
  const auto found = members.find(key);
  return found == members.end() ? nullptr : &found->second;
}

bool text_member(
  const std::map<std::string, JsonScalar> & members,
  const std::string & key, std::string & value)
{
  const auto * scalar = member(members, key);
  if (!scalar || scalar->type != JsonScalarType::string) {return false;}
  value = scalar->text;
  return true;
}

bool uint_member(
  const std::map<std::string, JsonScalar> & members,
  const std::string & key, std::uint64_t & value)
{
  const auto * scalar = member(members, key);
  if (!scalar || scalar->type != JsonScalarType::number || scalar->text.empty()) {return false;}
  const auto parsed = std::from_chars(
    scalar->text.data(), scalar->text.data() + scalar->text.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == scalar->text.data() + scalar->text.size();
}

bool int_member(
  const std::map<std::string, JsonScalar> & members,
  const std::string & key, std::int64_t & value)
{
  const auto * scalar = member(members, key);
  if (!scalar || scalar->type != JsonScalarType::number || scalar->text.empty()) {return false;}
  const auto parsed = std::from_chars(
    scalar->text.data(), scalar->text.data() + scalar->text.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == scalar->text.data() + scalar->text.size();
}

bool bool_member(
  const std::map<std::string, JsonScalar> & members,
  const std::string & key, bool & value)
{
  const auto * scalar = member(members, key);
  if (!scalar || scalar->type != JsonScalarType::boolean) {return false;}
  value = scalar->text == "true";
  return true;
}
}  // namespace

AssociationVerificationResult verify_camera_association(
  const std::filesystem::path & session_root,
  const std::string & stream_name, const std::size_t maximum_pending_records) noexcept
{
  try {
    if (!safe_name(stream_name)) {return failure(0U, 0U, 0U, "unsafe stream name");}
    const auto segments = verify_segment_set(session_root);
    if (!segments.success) {
      return failure(0U, 0U, 0U, "ppbseg set invalid: " + segments.message);
    }
    const SegmentStreamVerification * stream = nullptr;
    for (const auto & candidate : segments.streams) {
      if (candidate.stream_name == stream_name) {stream = &candidate; break;}
    }
    if (!stream) {return failure(0U, 0U, 0U, "stream absent from ppbseg set");}
    return verify_camera_association(session_root, *stream, maximum_pending_records);
  } catch (const std::exception & error) {return failure(0U, 0U, 0U, error.what());}
  catch (...) {return failure(0U, 0U, 0U, "unknown association verification failure");}
}

AssociationVerificationResult verify_camera_association(
  const std::filesystem::path & session_root,
  const SegmentStreamVerification & stream, const std::size_t maximum_pending_records) noexcept
{
  try {
    if (!safe_name(stream.stream_name)) {return failure(0U, 0U, 0U, "unsafe stream name");}
    if (maximum_pending_records == 0U ||
      maximum_pending_records > kAssociationPendingHardCap)
    {
      return failure(stream.record_count, 0U, 0U,
        "pending association limit must be positive and no greater than the hard cap");
    }
    CommittedSampleCursor committed(stream);
    std::string summary_detail;
    if (!committed.validate(summary_detail)) {
      return failure(stream.record_count, 0U, 0U, summary_detail);
    }
    std::map<std::uint64_t, SampleState> pending_samples;

    const auto path = session_root / "segments" / (stream.stream_name + "_association.ndjson");
    std::error_code error;
    const auto status = std::filesystem::symlink_status(path, error);
    if (error || !std::filesystem::is_regular_file(status)) {
      return failure(stream.record_count, 0U, 0U,
        "association sidecar is missing or not a regular file");
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) {return failure(stream.record_count, 0U, 0U, "cannot open association sidecar");}
    AssociationVerificationResult result;
    result.expected_samples = stream.record_count;
    std::string line;
    std::uint64_t line_number = 0U;
    while (std::getline(input, line)) {
      ++line_number;
      if (input.eof()) {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "association sidecar final record is not newline-terminated");
      }
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (line.empty() || line.size() > 1024U * 1024U) {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "empty or oversized association record at line " + std::to_string(line_number));
      }
      const auto parsed = parse_flat_json_object(line);
      if (!parsed.success) {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "invalid association JSON at line " + std::to_string(line_number) + ": " + parsed.message);
      }
      std::string state;
      std::uint64_t segment = 0U;
      if (!text_member(parsed.members, "state", state) ||
        !uint_member(parsed.members, "segment", segment))
      {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "association state/segment missing at line " + std::to_string(line_number));
      }
      if (state == "SEGMENT_RESET" || state == "DEGRADED") {
        std::string detail;
        bool known_segment = false;
        for (const auto & file : stream.files) {
          if (file.segment_index == segment) {known_segment = true; break;}
        }
        if (!known_segment || !text_member(parsed.members, "detail", detail) || detail.empty()) {
          return failure(result.expected_samples, result.pending_records, result.terminal_records,
            "invalid association event at line " + std::to_string(line_number));
        }
        if (state == "DEGRADED") {++result.degraded_records;}
        continue;
      }
      const bool terminal = state == "MATCHED" || state == "CONSISTENT_UNVERIFIED" ||
        state == "UNMATCHED";
      if (state != "PENDING" && !terminal) {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "unknown association state at line " + std::to_string(line_number));
      }
      std::uint64_t sample{}, sdk_frame{}, host_ns{}, camera_timestamp_ns{};
      bool trigger_valid = false;
      std::string detail;
      if (!uint_member(parsed.members, "sample", sample) || sample == 0U ||
        sample > result.expected_samples || !uint_member(parsed.members, "sdk_frame", sdk_frame) ||
        !uint_member(parsed.members, "camera_timestamp_ns", camera_timestamp_ns) ||
        !uint_member(parsed.members, "frame_host_monotonic_ns", host_ns) || host_ns == 0U ||
        !bool_member(parsed.members, "trigger_valid", trigger_valid) ||
        !text_member(parsed.members, "detail", detail) || detail.empty())
      {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "invalid frame association fields at line " + std::to_string(line_number));
      }
      if (state == "PENDING") {
        if (trigger_valid || !committed.consume(segment, sample)) {
          return failure(result.expected_samples, result.pending_records, result.terminal_records,
            "duplicate or out-of-order PENDING does not match the next committed ppbseg sample at sample " +
            std::to_string(sample));
        }
        if (pending_samples.size() >= maximum_pending_records) {
          return failure(result.expected_samples, result.pending_records, result.terminal_records,
            "pending association hard cap exceeded at sample " + std::to_string(sample));
        }
        const auto inserted = pending_samples.emplace(sample,
          SampleState{segment, sdk_frame, host_ns, camera_timestamp_ns});
        if (!inserted.second) {
          return failure(result.expected_samples, result.pending_records, result.terminal_records,
            "duplicate PENDING at sample " + std::to_string(sample));
        }
        ++result.pending_records;
        continue;
      }
      const auto pending = pending_samples.find(sample);
      if (pending == pending_samples.end() || pending->second.segment != segment ||
        pending->second.sdk_frame != sdk_frame || pending->second.host_ns != host_ns ||
        pending->second.camera_timestamp_ns != camera_timestamp_ns)
      {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "terminal association has no unique matching PENDING at sample " +
          std::to_string(sample));
      }
      if (trigger_valid) {
        std::uint64_t trigger{}, pps{}, tick{}, ticks_per_second{}, time_status{}, uncertainty{},
          trigger_host{};
        std::int64_t utc_ns{};
        if (!uint_member(parsed.members, "trigger_sequence", trigger) || trigger == 0U ||
          !uint_member(parsed.members, "pps_sequence", pps) ||
          !uint_member(parsed.members, "controller_tick", tick) ||
          !uint_member(parsed.members, "ticks_per_second", ticks_per_second) ||
          ticks_per_second == 0U ||
          !uint_member(parsed.members, "time_status", time_status) || time_status > 2U ||
          (time_status == 2U && tick >= ticks_per_second) ||
          !uint_member(parsed.members, "uncertainty_ns", uncertainty) ||
          !int_member(parsed.members, "utc_ns", utc_ns) ||
          !uint_member(parsed.members, "trigger_receipt_monotonic_ns", trigger_host) ||
          trigger_host == 0U)
        {
          return failure(result.expected_samples, result.pending_records, result.terminal_records,
            "invalid trigger evidence at sample " + std::to_string(sample));
        }
        (void)pps; (void)uncertainty; (void)utc_ns;
      } else {
        for (const std::string key : {"trigger_sequence", "pps_sequence", "controller_tick",
            "ticks_per_second", "utc_ns", "time_status", "uncertainty_ns",
            "trigger_receipt_monotonic_ns"})
        {
          if (member(parsed.members, key)) {
            return failure(result.expected_samples, result.pending_records, result.terminal_records,
              "trigger fields present while trigger_valid=false at sample " +
              std::to_string(sample));
          }
        }
      }
      if ((state == "UNMATCHED") == trigger_valid) {
        return failure(result.expected_samples, result.pending_records, result.terminal_records,
          "terminal state disagrees with trigger_valid at sample " + std::to_string(sample));
      }
      pending_samples.erase(pending);
      ++result.terminal_records;
      if (state == "MATCHED") {++result.matched_records;}
    }
    if (!input.eof()) {
      return failure(result.expected_samples, result.pending_records, result.terminal_records,
        "association sidecar read failed");
    }
    if (committed.consumed() != result.expected_samples || !pending_samples.empty() ||
      result.pending_records != result.expected_samples ||
      result.terminal_records != result.expected_samples)
    {
      return failure(result.expected_samples, result.pending_records, result.terminal_records,
        "missing association lifecycle for one or more committed samples");
    }
    result.success = true;
    result.message = "association sidecar matches every committed sample";
    return result;
  } catch (const std::exception & error) {return failure(0U, 0U, 0U, error.what());}
  catch (...) {return failure(0U, 0U, 0U, "unknown association verification failure");}
}
}  // namespace ppbng_storage
