#include "ppbng_hsi/envi_part_verifier.hpp"
#include "ppbng_storage/segment_io.hpp"
#include <charconv>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <string_view>
#include <vector>

namespace ppbng_hsi
{
namespace
{
constexpr std::string_view kIndexHeader =
  "segment,sdk_segment,part,part_line,source_segment_line,capture_kind,raw_offset,"
  "payload_bytes,payload_crc32,sequence_gap_before,missing_trigger_count,"
  "association_status,anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger";
constexpr std::string_view kTimestampHeader =
  "segment,sdk_segment,part,part_line,camera_sequence,host_receive_monotonic_ns,"
  "trigger_sequence,pps_sequence,controller_tick,controller_ticks_per_second,utc_ns,time_status,"
  "uncertainty_ns,association_status,"
  "anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger";

struct ParsedStem {std::string stream; std::uint64_t segment{}; std::uint64_t part{};};

bool safe_stem(const std::string & value)
{
  return !value.empty() && value != "." && value != ".." &&
    value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") ==
    std::string::npos;
}
bool parse_u64(const std::string_view text, std::uint64_t & value)
{
  if (text.empty()) {return false;}
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}
bool parse_i64(const std::string_view text, std::int64_t & value)
{
  if (text.empty()) {return false;}
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}
bool parse_stem(const std::string & stem, ParsedStem & parsed)
{
  const auto part_marker = stem.rfind("_part_");
  if (part_marker == std::string::npos) {return false;}
  const auto segment_marker = stem.rfind("_segment_", part_marker);
  if (segment_marker == std::string::npos || segment_marker == 0U) {return false;}
  parsed.stream = stem.substr(0U, segment_marker);
  return safe_stem(parsed.stream) &&
    parse_u64(std::string_view(stem).substr(
      segment_marker + 9U, part_marker - segment_marker - 9U), parsed.segment) &&
    parse_u64(std::string_view(stem).substr(part_marker + 6U), parsed.part);
}
std::vector<std::string_view> split_csv(const std::string & line)
{
  std::vector<std::string_view> fields;
  std::size_t begin = 0U;
  while (true) {
    const auto comma = line.find(',', begin);
    fields.emplace_back(line.data() + begin,
      (comma == std::string::npos ? line.size() : comma) - begin);
    if (comma == std::string::npos) {break;}
    begin = comma + 1U;
  }
  return fields;
}
bool valid_bool(const std::string_view value) {return value == "0" || value == "1";}
bool valid_association(const std::string_view value)
{
  return value == "MATCHED" || value == "UNMATCHED" ||
    value == "CONSISTENT_UNVERIFIED" || value == "UNVERIFIED";
}
bool valid_capture(const std::string_view value) {return value == "dark" || value == "sample";}
bool valid_time(const std::string_view value)
{
  return value == "locked" || value == "holdover" || value == "unsynced";
}
EnviPartVerification failure(const std::string & message)
{
  EnviPartVerification result;
  result.message = message;
  return result;
}
bool regular_file(const std::filesystem::path & path, std::error_code & error)
{
  const auto status = std::filesystem::symlink_status(path, error);
  return !error && std::filesystem::is_regular_file(status);
}
bool parse_header_value(
  const std::map<std::string, std::string> & fields, const std::string & key,
  std::uint64_t & value)
{
  const auto found = fields.find(key);
  return found != fields.end() && parse_u64(found->second, value);
}
bool verify_envi_header(
  const std::filesystem::path & path, const std::uint64_t lines,
  const std::uint64_t bytes_per_line, std::string & error)
{
  std::ifstream input(path, std::ios::binary);
  std::string line;
  if (!std::getline(input, line) || line != "ENVI") {error = "invalid ENVI header preamble"; return false;}
  std::map<std::string, std::string> fields;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    const auto equal = line.find('=');
    if (equal == std::string::npos) {error = "invalid ENVI header field"; return false;}
    auto key = line.substr(0U, equal);
    auto value = line.substr(equal + 1U);
    while (!key.empty() && key.back() == ' ') {key.pop_back();}
    while (!value.empty() && value.front() == ' ') {value.erase(value.begin());}
    if (!fields.emplace(std::move(key), std::move(value)).second) {
      error = "duplicate ENVI header field";
      return false;
    }
  }
  std::uint64_t samples{}, header_lines{}, bands{}, offset{}, data_type{}, byte_order{};
  if (!parse_header_value(fields, "samples", samples) ||
    !parse_header_value(fields, "lines", header_lines) ||
    !parse_header_value(fields, "bands", bands) ||
    !parse_header_value(fields, "header offset", offset) ||
    !parse_header_value(fields, "data type", data_type) ||
    !parse_header_value(fields, "byte order", byte_order) || samples == 0U || bands == 0U ||
    samples > std::numeric_limits<std::uint64_t>::max() / bands ||
    samples * bands > std::numeric_limits<std::uint64_t>::max() / 2U ||
    samples * bands * 2U != bytes_per_line || header_lines != lines || offset != 0U ||
    data_type != 12U || byte_order != 0U || fields["interleave"] != "bil" ||
    fields["file type"] != "ENVI Standard")
  {error = "ENVI header geometry or format disagrees with committed payload"; return false;}
  return true;
}
bool sequence_evidence(
  const std::uint64_t previous, const std::uint64_t current,
  const bool gap, const std::uint64_t missing)
{
  if (current <= previous) {return false;}
  const auto delta = current - previous;
  return delta == 1U ? (!gap && missing == 0U) : (gap && missing == delta - 1U);
}
}  // namespace

EnviPartVerification verify_envi_part(
  const std::filesystem::path & session_directory, const std::string & part_stem,
  const EnviVerificationOptions & options)
{
  if (!options.full_payload_crc && options.quick_crc_interval_lines == 0U) {
    return failure("quick verification CRC interval must be greater than zero");
  }
  ParsedStem expected;
  std::error_code error;
  if (!safe_stem(part_stem) || !parse_stem(part_stem, expected) ||
    !std::filesystem::is_directory(session_directory, error) || error)
  {return failure("invalid session directory or ENVI part stem");}
  const auto raw_path = session_directory / (part_stem + ".raw");
  const auto index_path = session_directory / (part_stem + ".index.csv");
  const auto timestamp_path = session_directory / (part_stem + ".timestamps.csv");
  const auto header_path = session_directory / (part_stem + ".hdr");
  for (const auto & path : {raw_path, index_path, timestamp_path, header_path}) {
    error.clear();
    if (!regular_file(path, error)) {return failure("part companion is missing or not a regular file");}
  }
  const auto raw_size = std::filesystem::file_size(raw_path, error);
  if (error) {return failure("raw file is unreadable");}
  std::ifstream raw(raw_path, std::ios::binary);
  std::ifstream index(index_path, std::ios::binary);
  std::ifstream timestamps(timestamp_path, std::ios::binary);
  if (!raw || !index || !timestamps) {return failure("part file cannot be opened");}
  std::string index_line, timestamp_line;
  if (!std::getline(index, index_line) || !std::getline(timestamps, timestamp_line)) {
    return failure("index or timestamp header is missing");
  }
  if (!index_line.empty() && index_line.back() == '\r') {index_line.pop_back();}
  if (!timestamp_line.empty() && timestamp_line.back() == '\r') {timestamp_line.pop_back();}
  if (index_line != kIndexHeader || timestamp_line != kTimestampHeader) {
    return failure("index or timestamp header does not match schema");
  }

  EnviPartVerification result;
  result.source_segment = expected.segment;
  result.part = expected.part;
  std::uint64_t expected_offset = 0U;
  std::uint64_t expected_source_line = 0U;
  std::uint64_t last_offset = 0U;
  std::uint64_t last_payload_size = 0U;
  std::uint32_t last_expected_crc = 0U;
  bool last_payload_checked = false;
  std::uint64_t next_progress = options.progress_interval_bytes;
  bool source_line_initialized = false;
  const auto verify_payload = [&](const std::uint64_t offset, const std::uint64_t payload_size,
      const std::uint32_t expected_crc) -> bool
    {
      std::vector<std::byte> payload(static_cast<std::size_t>(payload_size));
      raw.clear();
      raw.seekg(static_cast<std::streamoff>(offset));
      raw.read(reinterpret_cast<char *>(payload.data()), static_cast<std::streamsize>(payload_size));
      if (raw.gcount() != static_cast<std::streamsize>(payload_size) ||
        ppbng_storage::payload_crc32(payload.data(), payload.size()) != expected_crc)
      {return false;}
      ++result.crc_lines_checked;
      return true;
    };
  while (std::getline(index, index_line)) {
    if (index.eof()) {return failure("final index record is not newline-terminated");}
    if (!std::getline(timestamps, timestamp_line)) {
      return failure("timestamp file has fewer records than index");
    }
    if (timestamps.eof()) {return failure("final timestamp record is not newline-terminated");}
    if (!index_line.empty() && index_line.back() == '\r') {index_line.pop_back();}
    if (!timestamp_line.empty() && timestamp_line.back() == '\r') {timestamp_line.pop_back();}
    const auto index_fields = split_csv(index_line);
    const auto time_fields = split_csv(timestamp_line);
    if (index_fields.size() != 16U || time_fields.size() != 18U) {
      return failure("index or timestamp row has the wrong field count");
    }
    std::uint64_t segment{}, sdk_segment{}, part{}, part_line{}, source_line{}, offset{},
      payload_size{}, expected_crc{}, gap_value{}, missing{}, anchor{}, anchor_frame{},
      anchor_trigger{}, camera_sequence{}, host_ns{}, trigger_sequence{}, pps_sequence{},
      controller_tick{}, controller_frequency{}, uncertainty{};
    std::int64_t delta{}, utc{};
    if (!parse_u64(index_fields[0], segment) || !parse_u64(index_fields[1], sdk_segment) ||
      !parse_u64(index_fields[2], part) || !parse_u64(index_fields[3], part_line) ||
      !parse_u64(index_fields[4], source_line) || !parse_u64(index_fields[6], offset) ||
      !parse_u64(index_fields[7], payload_size) || !parse_u64(index_fields[8], expected_crc) ||
      !parse_u64(index_fields[9], gap_value) || !parse_u64(index_fields[10], missing) ||
      !parse_u64(index_fields[12], anchor) || !parse_i64(index_fields[13], delta) ||
      !parse_u64(index_fields[14], anchor_frame) || !parse_u64(index_fields[15], anchor_trigger) ||
      !parse_u64(time_fields[4], camera_sequence) || !parse_u64(time_fields[5], host_ns) ||
      !parse_u64(time_fields[6], trigger_sequence) || !parse_u64(time_fields[7], pps_sequence) ||
      !parse_u64(time_fields[8], controller_tick) ||
      !parse_u64(time_fields[9], controller_frequency) ||
      !parse_i64(time_fields[10], utc) || !parse_u64(time_fields[12], uncertainty) ||
      segment != expected.segment || part != expected.part || part_line != result.line_count ||
      offset != expected_offset || payload_size == 0U || expected_crc > UINT32_MAX ||
      gap_value > 1U || anchor > 1U || !valid_capture(index_fields[5]) ||
      !valid_association(index_fields[11]) || !valid_time(time_fields[11]) ||
      index_fields[0] != time_fields[0] || index_fields[1] != time_fields[1] ||
      index_fields[2] != time_fields[2] || index_fields[3] != time_fields[3] ||
      index_fields[11] != time_fields[13] || index_fields[12] != time_fields[14] ||
      index_fields[13] != time_fields[15] || index_fields[14] != time_fields[16] ||
      index_fields[15] != time_fields[17] || !valid_bool(index_fields[9]) ||
      !valid_bool(index_fields[12]) || host_ns == 0U ||
      (time_fields[11] == "locked" && controller_tick >= controller_frequency) ||
      payload_size > raw_size || offset > raw_size - payload_size ||
      payload_size > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
      payload_size > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max()))
    {return failure("invalid or inconsistent index/timestamp record");}
    const bool internal_untriggered = trigger_sequence == 0U && pps_sequence == 0U &&
      controller_tick == 0U && controller_frequency == 0U && utc == 0 && uncertainty == 0U &&
      time_fields[11] == "unsynced" && index_fields[11] == "UNVERIFIED" && anchor == 0U;
    if (!internal_untriggered && (trigger_sequence == 0U || controller_frequency == 0U)) {
      return failure("timestamp has neither complete trigger evidence nor valid internal timing evidence");
    }
    (void)pps_sequence; (void)utc; (void)uncertainty; (void)delta;
    (void)anchor_frame; (void)anchor_trigger;
    if (!source_line_initialized) {
      result.first_source_line = source_line;
      expected_source_line = source_line;
      result.first_camera_sequence = camera_sequence;
      result.first_sequence_gap_before = gap_value != 0U;
      result.first_missing_trigger_count = missing;
      result.sdk_segment = sdk_segment;
      result.bytes_per_line = payload_size;
      source_line_initialized = true;
    } else if (source_line != expected_source_line || payload_size != result.bytes_per_line ||
      sdk_segment != result.sdk_segment ||
      !sequence_evidence(result.last_camera_sequence, camera_sequence, gap_value != 0U, missing))
    {return failure("line sequence, gap evidence, or payload geometry is inconsistent");}
    if (source_line != expected_source_line) {return failure("source segment line is not contiguous");}
    const bool check_payload = options.full_payload_crc || result.line_count == 0U ||
      result.line_count % options.quick_crc_interval_lines == 0U;
    if (check_payload && !verify_payload(
        offset, payload_size, static_cast<std::uint32_t>(expected_crc)))
    {return failure("raw payload is truncated or CRC32 mismatched");}
    last_offset = offset;
    last_payload_size = payload_size;
    last_expected_crc = static_cast<std::uint32_t>(expected_crc);
    last_payload_checked = check_payload;
    result.last_source_line = source_line;
    result.last_camera_sequence = camera_sequence;
    expected_offset += payload_size;
    ++expected_source_line;
    ++result.line_count;
    if (options.progress && options.progress_interval_bytes > 0U &&
      expected_offset >= next_progress)
    {
      options.progress(part_stem, expected_offset, raw_size);
      while (next_progress <= expected_offset &&
        next_progress <= std::numeric_limits<std::uint64_t>::max() - options.progress_interval_bytes)
      {next_progress += options.progress_interval_bytes;}
    }
  }
  if (!index.eof()) {return failure("index read failed");}
  if (std::getline(timestamps, timestamp_line)) {return failure("timestamp file has extra records");}
  if (!timestamps.eof()) {return failure("timestamp read failed");}
  if (result.line_count == 0U || expected_offset != raw_size) {
    return failure("part has no committed lines or raw file has an unindexed tail");
  }
  if (!options.full_payload_crc && !last_payload_checked &&
    !verify_payload(last_offset, last_payload_size, last_expected_crc))
  {return failure("raw payload is truncated or CRC32 mismatched");}
  if (options.progress) {options.progress(part_stem, expected_offset, raw_size);}
  std::string header_error;
  if (!verify_envi_header(header_path, result.line_count, result.bytes_per_line, header_error)) {
    return failure(header_error);
  }
  result.success = true;
  result.payload_bytes = expected_offset;
  result.message = options.full_payload_crc ? "ENVI four-file part fully verified" :
    "ENVI four-file part structurally verified with sampled CRC32";
  return result;
}

EnviDatasetVerification verify_envi_dataset(
  const std::filesystem::path & session_directory,
  const std::vector<std::string> & expected_streams,
  const bool reject_unexpected_streams,
  const EnviVerificationOptions & options)
{
  try {
    std::error_code error;
    if (!std::filesystem::is_directory(session_directory, error) || error) {
      return {false, {}, "session directory is missing or unreadable"};
    }
    std::map<std::string, std::map<std::uint64_t, std::map<std::uint64_t, std::string>>> grouped;
    std::map<std::string, unsigned> companions;
    constexpr unsigned kRaw = 1U;
    constexpr unsigned kHeader = 2U;
    constexpr unsigned kIndex = 4U;
    constexpr unsigned kTimestamps = 8U;
    constexpr unsigned kComplete = kRaw | kHeader | kIndex | kTimestamps;
    for (std::filesystem::directory_iterator it(session_directory, error), end;
      it != end && !error; it.increment(error))
    {
      const auto name = it->path().filename().string();
      std::string suffix;
      unsigned kind = 0U;
      for (const auto & candidate : {
          std::pair<std::string_view, unsigned>{".timestamps.csv", kTimestamps},
          {".index.csv", kIndex}, {".raw", kRaw}, {".hdr", kHeader}})
      {
        if (name.size() > candidate.first.size() &&
          name.compare(name.size() - candidate.first.size(), candidate.first.size(), candidate.first) == 0)
        {
          suffix = candidate.first;
          kind = candidate.second;
          break;
        }
      }
      if (kind == 0U) {continue;}
      const auto status = it->symlink_status(error);
      if (error) {break;}
      if (!std::filesystem::is_regular_file(status)) {
        return {false, {}, "HSI companion entry is not a regular file"};
      }
      const auto stem = name.substr(0U, name.size() - suffix.size());
      ParsedStem parsed;
      if (!parse_stem(stem, parsed)) {
        return {false, {}, "invalid HSI companion filename: " + name};
      }
      companions[stem] |= kind;
    }
    if (error) {return {false, {}, "cannot enumerate HSI parts"};}
    for (const auto & companion : companions) {
      if (companion.second != kComplete) {
        return {false, {}, "orphan or incomplete HSI part companions: " + companion.first};
      }
      ParsedStem parsed;
      if (!parse_stem(companion.first, parsed) ||
        !grouped[parsed.stream][parsed.segment].emplace(parsed.part, companion.first).second)
      {
        return {false, {}, "duplicate HSI part identity"};
      }
    }
    std::set<std::string> expected;
    for (const auto & stream : expected_streams) {
      if (stream.empty() || !expected.insert(stream).second) {
        return {false, {}, "expected HSI stream names must be non-empty and unique"};
      }
      if (grouped.count(stream) == 0U) {return {false, {}, "missing HSI stream: " + stream};}
    }
    for (const auto & stream : grouped) {
      if (reject_unexpected_streams && !expected.empty() && expected.count(stream.first) == 0U) {
        return {false, {}, "unexpected HSI stream: " + stream.first};
      }
    }
    EnviDatasetVerification result;
    for (const auto & stream : grouped) {
      if (!expected.empty() && expected.count(stream.first) == 0U) {continue;}
      EnviStreamVerification summary;
      summary.stream_name = stream.first;
      std::uint64_t expected_segment = 0U;
      std::uint64_t stream_bytes_per_line = 0U;
      for (const auto & segment : stream.second) {
        if (segment.first != expected_segment) {
          return {false, {}, "non-contiguous HSI segment index in stream: " + stream.first};
        }
        std::uint64_t expected_part = 0U;
        std::uint64_t expected_source_line = 0U;
        std::uint64_t prior_camera_sequence = 0U;
        bool have_prior = false;
        std::uint64_t sdk_segment = 0U;
        for (const auto & part : segment.second) {
          if (part.first != expected_part) {
            return {false, {}, "non-contiguous HSI part index in stream: " + stream.first};
          }
          const auto verified = verify_envi_part(session_directory, part.second, options);
          if (!verified.success) {return {false, {}, part.second + ": " + verified.message};}
          if (verified.first_source_line != expected_source_line ||
            (have_prior && verified.sdk_segment != sdk_segment) ||
            (stream_bytes_per_line != 0U && verified.bytes_per_line != stream_bytes_per_line))
          {return {false, {}, "HSI cross-part geometry or line discontinuity: " + stream.first};}
          if (have_prior && !sequence_evidence(
              prior_camera_sequence, verified.first_camera_sequence,
              verified.first_sequence_gap_before, verified.first_missing_trigger_count))
          {return {false, {}, "HSI cross-part camera sequence evidence mismatch: " + stream.first};}
          expected_source_line = verified.last_source_line + 1U;
          prior_camera_sequence = verified.last_camera_sequence;
          sdk_segment = verified.sdk_segment;
          stream_bytes_per_line = verified.bytes_per_line;
          have_prior = true;
          summary.line_count += verified.line_count;
          summary.payload_bytes += verified.payload_bytes;
          summary.crc_lines_checked += verified.crc_lines_checked;
          ++summary.part_count;
          ++expected_part;
        }
        ++summary.segment_count;
        ++expected_segment;
      }
      result.streams.push_back(std::move(summary));
    }
    result.success = true;
    result.message = "dual HSI ENVI dataset verified";
    return result;
  } catch (const std::exception & error) {return {false, {}, error.what()};}
  catch (...) {return {false, {}, "unknown HSI dataset verification failure"};}
}
}  // namespace ppbng_hsi
