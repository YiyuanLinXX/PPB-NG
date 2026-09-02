#include "ppbng_hsi/dataset_context_verifier.hpp"
#include "ppbng_hsi/frame_evidence_verifier.hpp"

#include "ppbng_storage/json_verifier.hpp"

#include <charconv>
#include <fstream>
#include <limits>
#include <map>

namespace ppbng_hsi
{
namespace
{
DatasetContextVerification fail(
  const std::uint64_t expected, const std::uint64_t records, const std::string & message)
{
  return {false, expected, records, 0U, 0U, message};
}

bool u64(const std::string_view text, std::uint64_t & value)
{
  if (text.empty()) {return false;}
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

const ppbng_storage::JsonScalar * scalar(
  const std::map<std::string, ppbng_storage::JsonScalar> & values, const std::string & key)
{
  const auto found = values.find(key);
  return found == values.end() ? nullptr : &found->second;
}

bool text_value(const std::map<std::string, ppbng_storage::JsonScalar> & values,
  const std::string & key, std::string & value)
{
  const auto * item = scalar(values, key);
  if (!item || item->type != ppbng_storage::JsonScalarType::string) {return false;}
  value = item->text;
  return true;
}

bool uint_value(const std::map<std::string, ppbng_storage::JsonScalar> & values,
  const std::string & key, std::uint64_t & value)
{
  const auto * item = scalar(values, key);
  return item && item->type == ppbng_storage::JsonScalarType::number && u64(item->text, value);
}

bool int_value(const std::map<std::string, ppbng_storage::JsonScalar> & values,
  const std::string & key, std::int64_t & value)
{
  const auto * item = scalar(values, key);
  if (!item || item->type != ppbng_storage::JsonScalarType::number || item->text.empty()) {
    return false;
  }
  const auto result = std::from_chars(item->text.data(), item->text.data() + item->text.size(), value);
  return result.ec == std::errc{} && result.ptr == item->text.data() + item->text.size();
}

bool bool_value(const std::map<std::string, ppbng_storage::JsonScalar> & values,
  const std::string & key, bool & value)
{
  const auto * item = scalar(values, key);
  if (!item || item->type != ppbng_storage::JsonScalarType::boolean) {return false;}
  value = item->text == "true";
  return true;
}

bool availability(const std::string & value)
{
  return value == "AVAILABLE" || value == "DEGRADED" || value == "UNAVAILABLE";
}
}  // namespace

DatasetContextVerification verify_dataset_frame_context(
  const std::filesystem::path & session_root, const std::string & session_id,
  const ppbng_storage::SegmentSetVerificationResult & segment_set) noexcept
{
  try {
    if (session_id.empty() || !segment_set.success) {
      return fail(0U, 0U, "verified manifest session and ppbseg set are required");
    }
    std::uint64_t camera_records = 0U;
    bool rgb_seen = false;
    bool thermal_seen = false;
    for (const auto & stream : segment_set.streams) {
      if (stream.stream_name != "rgb" && stream.stream_name != "thermal") {
        return fail(camera_records, 0U, "unexpected ppbseg stream in context verification");
      }
      if (stream.stream_name == "rgb") {rgb_seen = true;} else {thermal_seen = true;}
      if (camera_records > (std::numeric_limits<std::uint64_t>::max)() - stream.record_count) {
        return fail(camera_records, 0U, "camera record count overflows");
      }
      camera_records += stream.record_count;
    }
    if (!rgb_seen || !thermal_seen || segment_set.streams.size() != 2U) {
      return fail(camera_records, 0U, "RGB and thermal ppbseg streams are both required exactly once");
    }

    std::error_code error;
    const auto path = session_root / "segments" / "frame_context.ndjson";
    const auto status = std::filesystem::symlink_status(path, error);
    if (error || !std::filesystem::is_regular_file(status)) {
      return fail(camera_records, 0U, "frame_context.ndjson is missing or not a regular file");
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) {return fail(camera_records, 0U, "cannot open frame_context.ndjson");}
    DatasetContextVerification result;
    std::string line;
    std::uint64_t line_number = 0U;
    while (std::getline(input, line)) {
      ++line_number;
      if (input.eof()) {
        return fail(camera_records, result.context_records, "frame context tail is not newline-terminated");
      }
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (line.empty() || line.size() > 1024U * 1024U) {
        return fail(camera_records, result.context_records, "empty or oversized frame context record");
      }
      const auto parsed = ppbng_storage::parse_flat_json_object(line);
      if (!parsed.success) {
        return fail(camera_records, result.context_records, "invalid frame context JSON at line " +
          std::to_string(line_number));
      }
      std::string record_session, stream, device, trigger, time_detail, gnss_status, gnss_detail,
        rsm_status, rsm_detail;
      std::uint64_t segment{}, sample{}, host{}, trigger_sequence{}, pps{}, tick{}, frequency{},
        time_status{}, uncertainty{}, camera_frame{}, camera_timestamp{};
      std::int64_t utc{};
      bool camera_frame_valid{}, camera_timestamp_valid{};
      if (!text_value(parsed.members, "session_id", record_session) || record_session != session_id ||
        !text_value(parsed.members, "device_id", device) || device.empty() ||
        !uint_value(parsed.members, "segment", segment) ||
        !uint_value(parsed.members, "sample", sample) ||
        !int_value(parsed.members, "frame_utc_ns", utc) ||
        !uint_value(parsed.members, "frame_host_ns", host) || host == 0U ||
        !text_value(parsed.members, "trigger_channel", trigger) || trigger.empty() ||
        !uint_value(parsed.members, "trigger_sequence", trigger_sequence) ||
        !uint_value(parsed.members, "pps_sequence", pps) ||
        !uint_value(parsed.members, "controller_tick", tick) ||
        !uint_value(parsed.members, "controller_ticks_per_second", frequency) ||
        !uint_value(parsed.members, "time_status", time_status) || time_status > 2U ||
        !uint_value(parsed.members, "time_uncertainty_ns", uncertainty) ||
        !text_value(parsed.members, "time_detail", time_detail) || time_detail.empty() ||
        !bool_value(parsed.members, "camera_frame_id_valid", camera_frame_valid) ||
        !uint_value(parsed.members, "camera_frame_id", camera_frame) ||
        !bool_value(parsed.members, "camera_timestamp_valid", camera_timestamp_valid) ||
        !uint_value(parsed.members, "camera_timestamp", camera_timestamp) ||
        !text_value(parsed.members, "gnss_status", gnss_status) || !availability(gnss_status) ||
        !text_value(parsed.members, "gnss_detail", gnss_detail) ||
        !text_value(parsed.members, "rsm_status", rsm_status) || !availability(rsm_status) ||
        !text_value(parsed.members, "rsm_detail", rsm_detail) || !camera_frame_valid ||
        (time_status != 0U && (utc <= 0 || trigger_sequence == 0U || frequency == 0U)) ||
        (time_status == 2U && tick >= frequency))
      {
        return fail(camera_records, result.context_records,
          "invalid frame context evidence at line " + std::to_string(line_number));
      }
      (void)pps; (void)uncertainty; (void)camera_frame; (void)camera_timestamp;
      (void)camera_timestamp_valid; (void)gnss_detail; (void)rsm_detail;
      stream = device;
      if (stream != "rgb" && stream != "thermal" && stream != "fx10e" && stream != "swir") {
        return fail(camera_records, result.context_records, "unexpected frame context device ID");
      }
      ++result.context_records;
      if (gnss_status == "UNAVAILABLE") {++result.unavailable_gnss_records;}
      if (rsm_status == "UNAVAILABLE") {++result.unavailable_rsm_records;}
    }
    if (!input.eof()) {return fail(camera_records, result.context_records, "frame context read failed");}
    const auto evidence = verify_dataset_frame_evidence(session_root, session_id);
    if (!evidence.success || evidence.compared_records != result.context_records) {
      return fail(camera_records, result.context_records,
        "frame evidence mismatch: " + evidence.message);
    }
    result.expected_records = evidence.compared_records;
    if (result.expected_records < camera_records) {
      return fail(camera_records, result.context_records,
        "exact source evidence contains fewer records than committed camera samples");
    }
    result.success = true;
    result.message = "every committed scene sample has exactly one exact frame context record";
    return result;
  } catch (const std::exception & error) {return fail(0U, 0U, error.what());}
  catch (...) {return fail(0U, 0U, "unknown frame context verification failure");}
}
}  // namespace ppbng_hsi
