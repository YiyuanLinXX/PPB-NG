#include "ppbng_hsi/frame_evidence_verifier.hpp"

#include "ppbng_storage/json_verifier.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
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
  "uncertainty_ns,association_status,anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger";

struct Evidence
{
  std::string stream;
  std::string time_detail;
  std::uint64_t segment{}, sample{}, host_ns{}, trigger_sequence{}, pps_sequence{},
    controller_tick{}, controller_frequency{}, time_status{}, uncertainty_ns{}, camera_frame{},
    camera_timestamp{};
  std::int64_t utc_ns{};
  bool camera_frame_valid{true};
  bool camera_timestamp_valid{false};
};

FrameEvidenceVerification fail(const std::uint64_t compared, const std::string & message)
{
  return {false, compared, message};
}

bool u64(const std::string_view text, std::uint64_t & value)
{
  if (text.empty()) {return false;}
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
}

bool i64(const std::string_view text, std::int64_t & value)
{
  if (text.empty()) {return false;}
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
}

std::vector<std::string_view> split(const std::string & line)
{
  std::vector<std::string_view> fields;
  std::size_t begin = 0U;
  while (true) {
    const auto comma = line.find(',', begin);
    fields.emplace_back(line.data() + begin,
      (comma == std::string::npos ? line.size() : comma) - begin);
    if (comma == std::string::npos) {return fields;}
    begin = comma + 1U;
  }
}

const ppbng_storage::JsonScalar * scalar(
  const std::map<std::string, ppbng_storage::JsonScalar> & members, const std::string & key)
{
  const auto found = members.find(key);
  return found == members.end() ? nullptr : &found->second;
}

bool text_member(const std::map<std::string, ppbng_storage::JsonScalar> & members,
  const std::string & key, std::string & value)
{
  const auto * item = scalar(members, key);
  if (!item || item->type != ppbng_storage::JsonScalarType::string) {return false;}
  value = item->text;
  return true;
}

bool uint_member(const std::map<std::string, ppbng_storage::JsonScalar> & members,
  const std::string & key, std::uint64_t & value)
{
  const auto * item = scalar(members, key);
  return item && item->type == ppbng_storage::JsonScalarType::number && u64(item->text, value);
}

bool int_member(const std::map<std::string, ppbng_storage::JsonScalar> & members,
  const std::string & key, std::int64_t & value)
{
  const auto * item = scalar(members, key);
  return item && item->type == ppbng_storage::JsonScalarType::number && i64(item->text, value);
}

bool bool_member(const std::map<std::string, ppbng_storage::JsonScalar> & members,
  const std::string & key, bool & value)
{
  const auto * item = scalar(members, key);
  if (!item || item->type != ppbng_storage::JsonScalarType::boolean) {return false;}
  value = item->text == "true";
  return true;
}

bool utc_representable(const std::int64_t utc_ns)
{
  return utc_ns > 0 && utc_ns / 1'000'000'000LL <=
    static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::max)());
}

class EvidenceCursor
{
public:
  virtual ~EvidenceCursor() = default;
  // true means a record was produced; false with empty error means EOF.
  virtual bool next(Evidence & evidence, std::string & error) = 0;
};

class CameraCursor final : public EvidenceCursor
{
public:
  CameraCursor(const std::filesystem::path & root, std::string stream)
  : stream_(std::move(stream)), input_(root / "segments" / (stream_ + "_association.ndjson"),
      std::ios::binary)
  {
    if (!input_) {initial_error_ = "cannot open " + stream_ + " association sidecar";}
  }

  bool next(Evidence & evidence, std::string & error) override
  {
    if (!initial_error_.empty()) {error = initial_error_; initial_error_.clear(); return false;}
    std::string line;
    while (std::getline(input_, line)) {
      if (input_.eof()) {error = stream_ + " association tail is not newline-terminated"; return false;}
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      const auto parsed = ppbng_storage::parse_flat_json_object(line);
      std::string state;
      if (!parsed.success || !text_member(parsed.members, "state", state)) {
        error = "invalid " + stream_ + " association record"; return false;
      }
      if (state == "PENDING" || state == "SEGMENT_RESET" || state == "DEGRADED") {continue;}
      const bool matched = state == "MATCHED";
      const bool consistent = state == "CONSISTENT_UNVERIFIED";
      const bool unmatched = state == "UNMATCHED";
      bool trigger_valid{};
      Evidence value;
      value.stream = stream_;
      value.camera_timestamp_valid = true;
      if ((!matched && !consistent && !unmatched) ||
        !uint_member(parsed.members, "segment", value.segment) ||
        !uint_member(parsed.members, "sample", value.sample) ||
        !uint_member(parsed.members, "sdk_frame", value.camera_frame) ||
        !uint_member(parsed.members, "camera_timestamp_ns", value.camera_timestamp) ||
        !uint_member(parsed.members, "frame_host_monotonic_ns", value.host_ns) ||
        !bool_member(parsed.members, "trigger_valid", trigger_valid) ||
        trigger_valid != (matched || consistent))
      {error = "invalid " + stream_ + " terminal association evidence"; return false;}
      if (trigger_valid) {
        std::uint64_t raw_status{};
        std::int64_t raw_utc{};
        if (!uint_member(parsed.members, "trigger_sequence", value.trigger_sequence) ||
          !uint_member(parsed.members, "pps_sequence", value.pps_sequence) ||
          !uint_member(parsed.members, "controller_tick", value.controller_tick) ||
          !uint_member(parsed.members, "ticks_per_second", value.controller_frequency) ||
          !uint_member(parsed.members, "uncertainty_ns", value.uncertainty_ns) ||
          !uint_member(parsed.members, "time_status", raw_status) || raw_status > 2U ||
          !int_member(parsed.members, "utc_ns", raw_utc))
        {error = "invalid " + stream_ + " trigger evidence"; return false;}
        value.time_status = matched ? raw_status : 0U;
        if (matched && raw_status != 0U) {
          if (!utc_representable(raw_utc)) {
            error = stream_ + " synchronized UTC is not ROS int32-sec representable"; return false;
          }
          value.utc_ns = raw_utc;
        }
      }
      value.time_detail = matched ?
        "MATCHED by segment-local SDK-frame/channel-sequence delta" :
        state + "; canonical UTC withheld; raw evidence is retained in association sidecar";
      evidence = std::move(value);
      return true;
    }
    if (!input_.eof()) {error = stream_ + " association read failed";}
    return false;
  }

private:
  std::string stream_;
  std::ifstream input_;
  std::string initial_error_;
};

class HsiCursor final : public EvidenceCursor
{
public:
  HsiCursor(std::filesystem::path root, std::string stream)
  : root_(std::move(root)), stream_(std::move(stream)) {}

  bool next(Evidence & evidence, std::string & error) override
  {
    while (true) {
      if (!open_) {
        if (!open_part(error)) {return false;}
      }
      std::string index_line, time_line;
      if (!std::getline(index_, index_line)) {
        if (!index_.eof()) {error = stream_ + " index read failed"; return false;}
        std::string extra;
        if (std::getline(timestamps_, extra) || !timestamps_.eof()) {
          error = stream_ + " timestamp/index length mismatch"; return false;
        }
        index_.close(); timestamps_.close(); open_ = false;
        if (!advance_part()) {finished_ = true; return false;}
        continue;
      }
      if (index_.eof() || !std::getline(timestamps_, time_line) || timestamps_.eof()) {
        error = stream_ + " index/timestamp tail is truncated"; return false;
      }
      if (!index_line.empty() && index_line.back() == '\r') {index_line.pop_back();}
      if (!time_line.empty() && time_line.back() == '\r') {time_line.pop_back();}
      const auto index_fields = split(index_line);
      const auto time_fields = split(time_line);
      if (index_fields.size() != 16U || time_fields.size() != 18U) {
        error = stream_ + " index/timestamp schema mismatch"; return false;
      }
      if (index_fields[5] == "dark") {continue;}
      if (index_fields[5] != "sample") {error = stream_ + " capture kind is invalid"; return false;}
      Evidence value;
      value.stream = stream_;
      std::uint64_t raw_status{};
      std::int64_t raw_utc{};
      const std::string association(index_fields[11]);
      if (!u64(index_fields[0], value.segment) || !u64(index_fields[4], value.sample) ||
        !u64(time_fields[4], value.camera_frame) || !u64(time_fields[5], value.host_ns) ||
        !u64(time_fields[6], value.trigger_sequence) || !u64(time_fields[7], value.pps_sequence) ||
        !u64(time_fields[8], value.controller_tick) ||
        !u64(time_fields[9], value.controller_frequency) ||
        !i64(time_fields[10], raw_utc) || !u64(time_fields[12], value.uncertainty_ns) ||
        time_fields[13] != index_fields[11] ||
        (time_fields[11] == "locked" ? (raw_status = 2U, false) :
          time_fields[11] == "holdover" ? (raw_status = 1U, false) :
          time_fields[11] == "unsynced" ? false : true))
      {error = "invalid " + stream_ + " scene timing evidence"; return false;}
      const bool matched = association == "MATCHED";
      if (!matched && association != "UNMATCHED" && association != "UNVERIFIED" &&
        association != "CONSISTENT_UNVERIFIED")
      {error = "unknown " + stream_ + " association state"; return false;}
      value.time_status = matched ? raw_status : 0U;
      const bool representable = utc_representable(raw_utc);
      if (matched && raw_status != 0U) {
        if (!representable) {
          error = stream_ + " synchronized UTC is not ROS int32-sec representable"; return false;
        }
        value.utc_ns = raw_utc;
      }
      const std::string raw_text = raw_status == 2U ? "LOCKED" :
        (raw_status == 1U ? "HOLDOVER" : "UNSYNCED");
      value.time_detail = "association=" + association + "; raw_trigger_time_status=" + raw_text +
        "; raw_trigger_utc=" + (representable ? "preserved" : "unavailable_or_out_of_range");
      if (!matched) {value.time_detail += "; canonical_status_forced_UNSYNCED";}
      evidence = std::move(value);
      return true;
    }
  }

private:
  std::filesystem::path stem(const std::uint64_t segment, const std::uint64_t part) const
  {
    return root_ / (stream_ + "_segment_" + std::to_string(segment) + "_part_" +
      std::to_string(part));
  }

  bool exists(const std::uint64_t segment, const std::uint64_t part) const
  {
    std::error_code error;
    return std::filesystem::is_regular_file(stem(segment, part).string() + ".index.csv", error) &&
      !error;
  }

  bool advance_part()
  {
    if (exists(segment_, part_ + 1U)) {++part_; return true;}
    if (exists(segment_ + 1U, 0U)) {++segment_; part_ = 0U; return true;}
    return false;
  }

  bool open_part(std::string & error)
  {
    if (finished_) {return false;}
    if (!exists(segment_, part_)) {
      error = "missing first " + stream_ + " ENVI part"; return false;
    }
    const auto base = stem(segment_, part_);
    index_.open(base.string() + ".index.csv", std::ios::binary);
    timestamps_.open(base.string() + ".timestamps.csv", std::ios::binary);
    std::string index_header, time_header;
    if (!index_ || !timestamps_ || !std::getline(index_, index_header) ||
      !std::getline(timestamps_, time_header))
    {error = "cannot open " + stream_ + " index/timestamp part"; return false;}
    if (!index_header.empty() && index_header.back() == '\r') {index_header.pop_back();}
    if (!time_header.empty() && time_header.back() == '\r') {time_header.pop_back();}
    if (index_header != kIndexHeader || time_header != kTimestampHeader) {
      error = stream_ + " index/timestamp header mismatch"; return false;
    }
    open_ = true;
    return true;
  }

  std::filesystem::path root_;
  std::string stream_;
  std::uint64_t segment_{0U}, part_{0U};
  bool open_{false}, finished_{false};
  std::ifstream index_, timestamps_;
};

bool compare(const Evidence & expected,
  const std::map<std::string, ppbng_storage::JsonScalar> & values, std::string & error)
{
  std::string session, trigger, device, detail;
  Evidence observed;
  bool frame_valid{}, timestamp_valid{};
  if (!text_member(values, "session_id", session) ||
    !text_member(values, "device_id", device) || device.empty() ||
    !text_member(values, "trigger_channel", trigger) || trigger.empty() ||
    !uint_member(values, "segment", observed.segment) ||
    !uint_member(values, "sample", observed.sample) ||
    !int_member(values, "frame_utc_ns", observed.utc_ns) ||
    !uint_member(values, "frame_host_ns", observed.host_ns) ||
    !uint_member(values, "trigger_sequence", observed.trigger_sequence) ||
    !uint_member(values, "pps_sequence", observed.pps_sequence) ||
    !uint_member(values, "controller_tick", observed.controller_tick) ||
    !uint_member(values, "controller_ticks_per_second", observed.controller_frequency) ||
    !uint_member(values, "time_status", observed.time_status) ||
    !uint_member(values, "time_uncertainty_ns", observed.uncertainty_ns) ||
    !text_member(values, "time_detail", detail) ||
    !bool_member(values, "camera_frame_id_valid", frame_valid) ||
    !uint_member(values, "camera_frame_id", observed.camera_frame) ||
    !bool_member(values, "camera_timestamp_valid", timestamp_valid) ||
    !uint_member(values, "camera_timestamp", observed.camera_timestamp))
  {error = "frame context lacks exact canonical evidence fields"; return false;}
  const bool expected_channel = trigger == expected.stream ||
    trigger == "internal:" + expected.stream;
  if (!expected_channel || observed.segment != expected.segment ||
    observed.sample != expected.sample)
  {error = "duplicate, out-of-order, or wrong frame context sample identity"; return false;}
  if (observed.host_ns != expected.host_ns ||
    observed.trigger_sequence != expected.trigger_sequence ||
    observed.pps_sequence != expected.pps_sequence ||
    observed.controller_tick != expected.controller_tick ||
    observed.controller_frequency != expected.controller_frequency ||
    observed.time_status != expected.time_status ||
    observed.uncertainty_ns != expected.uncertainty_ns ||
    observed.utc_ns != expected.utc_ns || detail != expected.time_detail ||
    frame_valid != expected.camera_frame_valid || observed.camera_frame != expected.camera_frame ||
    timestamp_valid != expected.camera_timestamp_valid ||
    observed.camera_timestamp != expected.camera_timestamp)
  {error = "frame context numeric/time evidence disagrees with source sidecar"; return false;}
  return true;
}
}  // namespace

FrameEvidenceVerification verify_dataset_frame_evidence(
  const std::filesystem::path & session_root, const std::string & session_id) noexcept
{
  try {
    if (session_id.empty()) {return fail(0U, "verified session ID is required");}
    std::array<std::string, 4U> names{"rgb", "thermal", "fx10e", "swir"};
    std::array<std::unique_ptr<EvidenceCursor>, 4U> cursors{
      std::make_unique<CameraCursor>(session_root, names[0]),
      std::make_unique<CameraCursor>(session_root, names[1]),
      std::make_unique<HsiCursor>(session_root / "segments", names[2]),
      std::make_unique<HsiCursor>(session_root / "segments", names[3])};
    const auto path = session_root / "segments" / "frame_context.ndjson";
    std::ifstream input(path, std::ios::binary);
    if (!input) {return fail(0U, "cannot open frame_context.ndjson for evidence comparison");}
    std::uint64_t compared = 0U;
    std::string line;
    while (std::getline(input, line)) {
      if (input.eof()) {return fail(compared, "frame context tail is not newline-terminated");}
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      const auto parsed = ppbng_storage::parse_flat_json_object(line);
      std::string stream, record_session, device, trigger;
      if (!parsed.success || !text_member(parsed.members, "session_id", record_session) ||
        record_session != session_id || !text_member(parsed.members, "device_id", device) ||
        device.empty() || !text_member(parsed.members, "trigger_channel", trigger))
      {return fail(compared, "invalid session or stream in frame context evidence");}
      stream = trigger.rfind("internal:", 0U) == 0U ? trigger.substr(9U) : trigger;
      const auto found = std::find(names.begin(), names.end(), stream);
      if (found == names.end()) {return fail(compared, "unexpected frame context stream");}
      const auto index = static_cast<std::size_t>(std::distance(names.begin(), found));
      Evidence expected;
      std::string error;
      if (!cursors[index]->next(expected, error)) {
        return fail(compared, error.empty() ?
          "duplicate or extra frame context has no corresponding source record" : error);
      }
      if (!compare(expected, parsed.members, error)) {return fail(compared, error);}
      ++compared;
    }
    if (!input.eof()) {return fail(compared, "frame context read failed");}
    for (std::size_t i = 0U; i < cursors.size(); ++i) {
      Evidence remaining;
      std::string error;
      if (cursors[i]->next(remaining, error)) {
        return fail(compared,
          "one or more source samples lack frame context: " + names[i]);
      }
      if (!error.empty()) {return fail(compared, error);}
    }
    return {true, compared,
      "every frame context exactly matches its ordered authoritative source evidence"};
  } catch (const std::exception & error) {return fail(0U, error.what());}
  catch (...) {return fail(0U, "unknown frame evidence verification failure");}
}
}  // namespace ppbng_hsi
