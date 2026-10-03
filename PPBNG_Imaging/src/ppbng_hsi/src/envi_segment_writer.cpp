#include "ppbng_hsi/envi_segment_writer.hpp"

#include "ppbng_hsi/hsi_format.hpp"
#include "ppbng_storage/dataset_session.hpp"
#include "ppbng_storage/segment_io.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ppbng_hsi
{
namespace
{
bool safe_stem(const std::string & value)
{
  if (value.empty() || value == "." || value == "..") {return false;}
  return value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") ==
         std::string::npos;
}

bool create_exclusive(const std::filesystem::path & path, std::string & error)
{
#ifdef _WIN32
  HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    error = "exclusive create failed (Win32 error " + std::to_string(GetLastError()) + ")";
    return false;
  }
  CloseHandle(handle);
  return true;
#else
  if (std::filesystem::exists(path)) {error = "target already exists"; return false;}
  std::ofstream stream(path, std::ios::binary | std::ios::out);
  if (!stream) {error = "exclusive create failed"; return false;}
  return true;
#endif
}

std::string part_stem(const EnviWriterOptions & options, std::uint32_t segment, std::uint32_t part)
{
  return options.stream_stem + "_segment_" + std::to_string(segment) + "_part_" +
         std::to_string(part);
}

std::string capture_text(CaptureKind kind) {return kind == CaptureKind::dark ? "dark" : "sample";}
std::string time_text(TimeStatus status)
{
  if (status == TimeStatus::locked) {return "locked";}
  if (status == TimeStatus::holdover) {return "holdover";}
  return "unsynced";
}
std::string association_text(AssociationStatus status)
{
  if (status == AssociationStatus::matched) {return "MATCHED";}
  if (status == AssociationStatus::unmatched) {return "UNMATCHED";}
  if (status == AssociationStatus::consistent_unverified) {return "CONSISTENT_UNVERIFIED";}
  return "UNVERIFIED";
}
}  // namespace

EnviSegmentWriter::EnviSegmentWriter(EnviWriterOptions options, HsiConfig config)
: options_(std::move(options)), config_(std::move(config)) {}

EnviSegmentWriter::~EnviSegmentWriter() {close();}

std::pair<OperationResult, std::unique_ptr<EnviSegmentWriter>> EnviSegmentWriter::create(
  const EnviWriterOptions & options, const HsiConfig & config)
{
  std::error_code ec;
  if (options.session_directory.empty() || !std::filesystem::is_directory(options.session_directory, ec)) {
    return std::make_pair(OperationResult{false,
      "session_directory must explicitly name an existing directory"},
      std::unique_ptr<EnviSegmentWriter>{});
  }
  if (!safe_stem(options.stream_stem) || options.maximum_segment_bytes == 0U ||
    !HsiFormat::validate_config(config).success)
  {
    return std::make_pair(OperationResult{false, "invalid ENVI writer configuration"},
      std::unique_ptr<EnviSegmentWriter>{});
  }
  std::string error;
  if (!create_exclusive(options.session_directory / (options.stream_stem + ".checkpoint"), error)) {
    return std::make_pair(OperationResult{false, "checkpoint: " + error},
      std::unique_ptr<EnviSegmentWriter>{});
  }
  return std::make_pair(OperationResult{true, "ENVI writer checkpoint exclusively reserved"},
    std::unique_ptr<EnviSegmentWriter>(new EnviSegmentWriter(options, config)));
}

OperationResult EnviSegmentWriter::open_part(std::uint32_t source_segment, std::uint32_t part)
{
  const auto stem = part_stem(options_, source_segment, part);
  const auto raw_path = options_.session_directory / (stem + ".raw");
  const auto timestamp_path = options_.session_directory / (stem + ".timestamps.csv");
  const auto index_path = options_.session_directory / (stem + ".index.csv");
  const auto header_path = options_.session_directory / (stem + ".hdr");
  std::string error;
  for (const auto & path : {raw_path, timestamp_path, index_path, header_path}) {
    if (!create_exclusive(path, error)) {return {false, path.filename().string() + ": " + error};}
  }
  raw_.open(raw_path, std::ios::binary | std::ios::app);
  timestamps_.open(timestamp_path, std::ios::app);
  index_.open(index_path, std::ios::app);
  if (!raw_ || !timestamps_ || !index_) {return {false, "failed to open exclusively reserved segment files"};}
  timestamps_ << "segment,sdk_segment,part,part_line,camera_sequence,host_receive_monotonic_ns,trigger_sequence,pps_sequence,controller_tick,controller_ticks_per_second,utc_ns,time_status,uncertainty_ns,association_status,anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger\n";
  index_ << "segment,sdk_segment,part,part_line,source_segment_line,capture_kind,raw_offset,payload_bytes,payload_crc32,sequence_gap_before,missing_trigger_count,association_status,anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger\n";
  header_relative_ = header_path.filename();
  source_segment_ = source_segment;
  part_ = part;
  part_lines_ = 0U;
  part_payload_bytes_ = 0U;
  part_open_ = true;
  return checkpoint();
}

OperationResult EnviSegmentWriter::checkpoint()
{
  const ScopedLatency timer(performance_.checkpoint);
  auto layout = HsiFormat::make_envi_layout(config_, source_segment_, part_lines_, options_.stream_stem);
  const auto operation = "hsi-" + std::to_string(source_segment_) + "-" + std::to_string(part_) +
    "-" + std::to_string(part_lines_);
  // flush_every_lines=0 is the throughput-first policy, including rollover.
  // Atomic headers remain, but do not force SSD cache flushes in the consumer.
  const bool durable = options_.flush_every_lines != 0U;
  const auto header = ppbng_storage::atomic_write_text_with_durability(
    options_.session_directory, header_relative_, HsiFormat::render_envi_header(layout) +
    "ppbng payload checksum = " + (options_.payload_crc_enabled ? "crc32\n" : "none\n"),
    operation + "-hdr", durable);
  if (!header.ok()) {return {false, "header checkpoint failed: " + header.detail};}
  std::ostringstream state;
  state << "source_segment=" << source_segment_ << '\n' << "part=" << part_ << '\n'
        << "part_lines=" << part_lines_ << '\n' << "total_lines=" << total_lines_ << '\n'
        << "committed_payload_bytes=" << total_payload_bytes_ << '\n';
  const auto cp = ppbng_storage::atomic_write_text_with_durability(
    options_.session_directory, options_.stream_stem + ".checkpoint", state.str(), operation + "-cp", durable);
  return cp.ok() ? OperationResult{true, "checkpoint committed"} :
    OperationResult{false, "writer checkpoint failed: " + cp.detail};
}

OperationResult EnviSegmentWriter::flush()
{
  if (!part_open_) {return {true, "no open ENVI part"};}
  {
    const ScopedLatency timer(performance_.stream_flush);
    raw_.flush(); timestamps_.flush(); index_.flush();
  }
  if (!raw_ || !timestamps_ || !index_) {return {false, "ENVI segment flush failed"};}
  return checkpoint();
}

OperationResult EnviSegmentWriter::finalize_part()
{
  if (!part_open_) {return {true, "no part to finalize"};}
  auto result = flush();
  raw_.close(); timestamps_.close(); index_.close();
  part_open_ = false;
  return result;
}

OperationResult EnviSegmentWriter::append(const LineRecord & line)
{
  const ScopedLatency append_timer(performance_.append);
  if (closed_) {return {false, "writer is closed"};}
  const auto expected = HsiFormat::payload_bytes_per_line(config_);
  const auto payload = static_cast<std::uint64_t>(line.pixels.size()) * sizeof(std::uint16_t);
  if (line.camera_kind != config_.kind || line.device_id != config_.device_id || payload != expected) {
    return {false, "line metadata or payload does not match frozen HSI configuration"};
  }
  if (!part_open_ || line.index.segment_id != source_segment_) {
    const ScopedLatency timer(performance_.rollover);
    auto result = finalize_part();
    if (!result.success) {return result;}
    result = open_part(line.index.segment_id, 0U);
    if (!result.success) {return result;}
  } else if (part_payload_bytes_ != 0U &&
    part_payload_bytes_ + payload > options_.maximum_segment_bytes)
  {
    const ScopedLatency timer(performance_.rollover);
    auto result = finalize_part();
    if (!result.success) {return result;}
    result = open_part(source_segment_, part_ + 1U);
    if (!result.success) {return result;}
  }
  if (total_payload_bytes_ > options_.fail_after_payload_bytes ||
    payload > options_.fail_after_payload_bytes - total_payload_bytes_)
  {
    return {false, "injected ENVI payload write failure"};
  }
  const auto raw_offset = part_payload_bytes_;
  std::uint32_t payload_crc{};
  if (options_.payload_crc_enabled) {
    const ScopedLatency timer(performance_.crc);
    payload_crc = ppbng_storage::payload_crc32(
      reinterpret_cast<const std::byte *>(line.pixels.data()), static_cast<std::size_t>(payload));
  }
  {
    const ScopedLatency timer(performance_.raw_write);
    raw_.write(reinterpret_cast<const char *>(line.pixels.data()), static_cast<std::streamsize>(payload));
  }
  {
    const ScopedLatency timer(performance_.sidecar_write);
  timestamps_ << source_segment_ << ',' << line.index.sdk_segment_id << ',' << part_ << ',' << part_lines_ << ','
              << line.index.camera_line_sequence << ',' << line.index.host_receive_monotonic_ns << ','
              << line.index.trigger_sequence << ','
              << line.index.pps_sequence << ',' << line.trigger.offset_ticks << ','
              << line.trigger.ticks_per_second << ',' << line.index.utc_time_ns << ','
              << time_text(line.index.time_status) << ',' << line.index.uncertainty_ns << ','
              << association_text(line.index.association_status) << ','
              << (line.index.association_anchor_valid ? 1 : 0) << ','
              << line.index.frame_trigger_delta << ',' << line.index.association_anchor_frame << ','
              << line.index.association_anchor_trigger << '\n';
  index_ << source_segment_ << ',' << line.index.sdk_segment_id << ',' << part_ << ',' << part_lines_ << ','
         << line.index.segment_line_index << ',' << capture_text(line.index.capture_kind) << ','
         << raw_offset << ',' << payload << ',';
  if (options_.payload_crc_enabled) {index_ << payload_crc;}
  index_ << ',' << (line.index.sequence_gap_before ? 1 : 0) << ',' << line.index.missing_trigger_count << ','
         << association_text(line.index.association_status) << ','
         << (line.index.association_anchor_valid ? 1 : 0) << ','
         << line.index.frame_trigger_delta << ',' << line.index.association_anchor_frame << ','
         << line.index.association_anchor_trigger << '\n';
  }
  if (!raw_ || !timestamps_ || !index_) {return {false, "ENVI append failed"};}
  ++part_lines_; ++total_lines_;
  part_payload_bytes_ += payload;
  total_payload_bytes_ += payload;
  if (options_.flush_every_lines != 0U &&
    part_lines_ % options_.flush_every_lines == 0U) {return flush();}
  return {true, "line persisted"};
}

OperationResult EnviSegmentWriter::close()
{
  if (closed_) {return {true, "writer already closed"};}
  const auto result = finalize_part();
  closed_ = true;
  return result;
}

OperationResult EnviSegmentWriter::finalize_current_segment()
{
  if (closed_) {return {false, "writer is closed"};}
  return finalize_part();
}

}  // namespace ppbng_hsi
