#include "ppbng_storage/segment_verifier.hpp"

#include "ppbng_storage/segment_io.hpp"

#include <limits>
#include <map>
#include <set>

namespace ppbng_storage
{

SegmentVerificationResult verify_segment_file(
  const std::filesystem::path & session_root,
  const std::filesystem::path & relative_segment) noexcept
{
  try {
    auto opened = SegmentReader::open(session_root, relative_segment);
    if (!opened.first.ok()) {
      return {false, 0U, 0U, 0U, 0U, opened.first.detail};
    }

    SegmentVerificationResult result;
    SegmentFrame frame;
    for (;;) {
      const auto status = opened.second->next(frame);
      if (status.error == SegmentIoError::end_of_file) {
        result.success = true;
        result.message = "segment checksums and sample order verified";
        return result;
      }
      if (!status.ok()) {
        result.message = status.detail + " at offset " + std::to_string(status.offset);
        return result;
      }
      if (frame.envelope.sample_id == 0U) {
        result.message = "zero sample ID at offset " + std::to_string(frame.record_offset);
        return result;
      }
      if (result.record_count != 0U &&
        (result.last_sample_id == std::numeric_limits<std::uint64_t>::max() ||
        frame.envelope.sample_id != result.last_sample_id + 1U))
      {
        result.message = "sample IDs are not contiguous at offset " +
          std::to_string(frame.record_offset);
        return result;
      }
      if (frame.payload.size() >
        std::numeric_limits<std::uint64_t>::max() - result.payload_bytes)
      {
        result.message = "payload byte count overflow";
        return result;
      }
      if (result.record_count == 0U) {
        result.first_sample_id = frame.envelope.sample_id;
      }
      result.last_sample_id = frame.envelope.sample_id;
      result.payload_bytes += static_cast<std::uint64_t>(frame.payload.size());
      ++result.record_count;
    }
  } catch (const std::exception & error) {
    return {false, 0U, 0U, 0U, 0U, error.what()};
  } catch (...) {
    return {false, 0U, 0U, 0U, 0U, "unknown verification failure"};
  }
}

SegmentSetVerificationResult verify_segment_set(
  const std::filesystem::path & session_root,
  const std::vector<std::string> & expected_streams) noexcept
{
  try {
    const auto directory = session_root / "segments";
    std::error_code error;
    if (!std::filesystem::is_directory(directory, error) || error) {
      return {false, {}, "segments directory is missing or unreadable"};
    }
    std::map<std::string, std::map<std::uint64_t, std::filesystem::path>> grouped;
    const std::string suffix = ".ppbseg";
    for (std::filesystem::directory_iterator it(directory, error), end;
      it != end && !error; it.increment(error))
    {
      const auto name = it->path().filename().string();
      if (name.size() <= suffix.size() ||
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {continue;}
      const auto status = it->symlink_status(error);
      if (error) {break;}
      if (!std::filesystem::is_regular_file(status)) {
        return {false, {}, "ppbseg entry is not a regular file: " + name};
      }
      const auto stem = name.substr(0U, name.size() - suffix.size());
      const auto delimiter = stem.rfind('_');
      if (delimiter == std::string::npos || delimiter == 0U ||
        stem.size() - delimiter - 1U != 6U)
      {
        return {false, {}, "invalid ppbseg filename: " + name};
      }
      const auto digits = stem.substr(delimiter + 1U);
      if (digits.find_first_not_of("0123456789") != std::string::npos) {
        return {false, {}, "invalid ppbseg segment index: " + name};
      }
      const auto index = static_cast<std::uint64_t>(std::stoul(digits));
      const auto stream = stem.substr(0U, delimiter);
      const auto relative = std::filesystem::path("segments") / it->path().filename();
      if (!grouped[stream].emplace(index, relative).second) {
        return {false, {}, "duplicate ppbseg segment index for stream: " + stream};
      }
    }
    if (error) {return {false, {}, "cannot enumerate ppbseg files: " + error.message()};}

    std::set<std::string> expected;
    for (const auto & stream : expected_streams) {
      if (stream.empty() || !expected.insert(stream).second) {
        return {false, {}, "expected stream names must be non-empty and unique"};
      }
      if (grouped.count(stream) == 0U) {
        return {false, {}, "missing expected ppbseg stream: " + stream};
      }
    }
    for (const auto & entry : grouped) {
      if (!expected.empty() && expected.count(entry.first) == 0U) {
        return {false, {}, "unexpected ppbseg stream: " + entry.first};
      }
    }

    SegmentSetVerificationResult result;
    for (const auto & stream_entry : grouped) {
      SegmentStreamVerification summary;
      summary.stream_name = stream_entry.first;
      std::uint64_t expected_index = 0U;
      std::uint64_t expected_sample = 1U;
      std::size_t position = 0U;
      for (const auto & segment_entry : stream_entry.second) {
        if (segment_entry.first != expected_index) {
          return {false, {}, "non-contiguous segment index in stream: " + stream_entry.first};
        }
        const auto verified = verify_segment_file(session_root, segment_entry.second);
        if (!verified.success) {
          return {false, {}, segment_entry.second.generic_string() + ": " + verified.message};
        }
        const bool last = position + 1U == stream_entry.second.size();
        if (verified.record_count == 0U && !last) {
          return {false, {}, "empty non-final segment in stream: " + stream_entry.first};
        }
        if (verified.record_count != 0U) {
          if (verified.first_sample_id != expected_sample) {
            return {false, {}, "cross-segment sample discontinuity in stream: " +
              stream_entry.first};
          }
          if (verified.last_sample_id == std::numeric_limits<std::uint64_t>::max()) {
            return {false, {}, "sample sequence overflow in stream: " + stream_entry.first};
          }
          expected_sample = verified.last_sample_id + 1U;
          summary.last_sample_id = verified.last_sample_id;
        }
        if (verified.record_count >
          std::numeric_limits<std::uint64_t>::max() - summary.record_count ||
          verified.payload_bytes >
          std::numeric_limits<std::uint64_t>::max() - summary.payload_bytes)
        {
          return {false, {}, "aggregate counter overflow in stream: " + stream_entry.first};
        }
        summary.record_count += verified.record_count;
        summary.payload_bytes += verified.payload_bytes;
        summary.files.push_back({
          segment_entry.first, verified.record_count,
          verified.first_sample_id, verified.last_sample_id});
        ++summary.segment_count;
        ++expected_index;
        ++position;
      }
      result.streams.push_back(std::move(summary));
    }
    result.success = true;
    result.message = "ppbseg stream set verified";
    return result;
  } catch (const std::exception & error) {
    return {false, {}, error.what()};
  } catch (...) {
    return {false, {}, "unknown segment-set verification failure"};
  }
}

}  // namespace ppbng_storage
