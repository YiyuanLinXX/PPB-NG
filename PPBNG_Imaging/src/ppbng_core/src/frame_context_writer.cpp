#include "ppbng_core/frame_context_writer.hpp"

#include <iomanip>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ppbng_core
{
namespace
{
const char * status(const ContextAvailability value)
{
  return value == ContextAvailability::available ? "AVAILABLE" :
         value == ContextAvailability::degraded ? "DEGRADED" : "UNAVAILABLE";
}

bool safe(const std::string & value)
{
  return !value.empty() && value.find_first_not_of(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  return output.str();
}
}  // namespace

std::pair<ContextWriterResult, std::unique_ptr<FrameContextWriter>>
FrameContextWriter::create(ContextWriterOptions options)
{
  if (options.session_root.empty() || !safe(options.stem) ||
    !std::filesystem::is_directory(options.session_root / "segments"))
  {
    return {ContextWriterResult{false, "invalid context writer path"}, nullptr};
  }
  auto writer = std::unique_ptr<FrameContextWriter>(
    new FrameContextWriter(std::move(options)));
  const auto opened = writer->open();
  if (!opened.ok) return {opened, nullptr};
  return {ContextWriterResult{true, "exclusive context sidecar created"}, std::move(writer)};
}

ContextWriterResult FrameContextWriter::open()
{
  const auto path = options_.session_root / "segments" / (options_.stem + ".ndjson");
#ifdef _WIN32
  const auto handle = CreateFileW(
    path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    return {false, "context sidecar already exists or cannot be exclusively created"};
  }
  if (!CloseHandle(handle)) return {false, "context sidecar reservation close failed"};
#else
  if (std::filesystem::exists(path)) return {false, "context sidecar already exists"};
#endif
  stream_.open(path, std::ios::out | std::ios::app | std::ios::binary);
  return stream_ ? ContextWriterResult{true, "opened"} :
         ContextWriterResult{false, "context sidecar open failed"};
}

ContextWriterResult FrameContextWriter::append(
  const ContextFrameIdentity & frame, const FrameContextResult & context)
{
  if (closed_ || !stream_) return {false, "context writer not open"};
  if (records_ >= options_.fail_after_records) {
    return {false, "injected context storage failure"};
  }

  stream_ << std::setprecision(17) <<
    "{\"session_id\":\"" << json_escape(frame.session_id) <<
    "\",\"device_id\":\"" << json_escape(frame.device_id) <<
    "\",\"segment\":" << frame.segment_id <<
    ",\"sample\":" << frame.sample_sequence <<
    ",\"frame_utc_ns\":" << frame.utc_ns <<
    ",\"frame_host_ns\":" << frame.host_ns <<
    ",\"trigger_channel\":\"" << json_escape(frame.trigger_channel) <<
    "\",\"trigger_sequence\":" << frame.trigger_sequence <<
    ",\"pps_sequence\":" << frame.pps_sequence <<
    ",\"controller_tick\":" << frame.controller_tick <<
    ",\"controller_ticks_per_second\":" << frame.controller_ticks_per_second <<
    ",\"time_status\":" << static_cast<unsigned>(frame.time_status) <<
    ",\"time_uncertainty_ns\":" << frame.time_uncertainty_ns <<
    ",\"time_detail\":\"" << json_escape(frame.time_detail) <<
    "\",\"camera_frame_id_valid\":" <<
    (frame.camera_frame_id_valid ? "true" : "false") <<
    ",\"camera_frame_id\":" << frame.camera_frame_id <<
    ",\"camera_timestamp_valid\":" <<
    (frame.camera_timestamp_valid ? "true" : "false") <<
    ",\"camera_timestamp\":" << frame.camera_timestamp <<
    ",\"thermal_nuc_state_valid\":" <<
    (frame.thermal_nuc_state_valid ? "true" : "false") <<
    ",\"thermal_nuc_active\":" <<
    (frame.thermal_nuc_active ? "true" : "false") <<
    ",\"thermal_nuc_state_sequence\":" << frame.thermal_nuc_state_sequence <<
    ",\"thermal_nuc_observed_host_monotonic_ns\":" <<
    frame.thermal_nuc_observed_host_monotonic_ns <<
    ",\"thermal_correction_auto_in_progress\":" <<
    (frame.thermal_correction_auto_in_progress ? "true" : "false") <<
    ",\"thermal_correction_status\":\"" <<
    json_escape(frame.thermal_correction_status) <<
    "\",\"thermal_flag_state\":\"" << json_escape(frame.thermal_flag_state) <<
    "\",\"thermal_nuc_detail\":\"" << json_escape(frame.thermal_nuc_detail) << "\"" <<
    ",\"gnss_status\":\"" << status(context.gnss_status) <<
    "\",\"gnss_detail\":\"" << json_escape(context.gnss_detail) << "\"";

  if (context.gnss) {
    stream_ <<
      ",\"gnss_before\":" << context.gnss->before_sequence <<
      ",\"gnss_after\":" << context.gnss->after_sequence <<
      ",\"gnss_before_age_ns\":" << context.gnss->before_age_nanoseconds <<
      ",\"gnss_after_age_ns\":" << context.gnss->after_age_nanoseconds <<
      ",\"gnss_before_utc_anchor_sequence\":" <<
      context.gnss->before_utc_anchor_sequence <<
      ",\"gnss_after_utc_anchor_sequence\":" << context.gnss->after_utc_anchor_sequence <<
      ",\"gnss_before_anchor_host_delta_ns\":" <<
      context.gnss->before_utc_anchor_host_delta_nanoseconds <<
      ",\"gnss_after_anchor_host_delta_ns\":" <<
      context.gnss->after_utc_anchor_host_delta_nanoseconds <<
      ",\"gnss_before_anchor_clock_residual_ns\":" <<
      context.gnss->before_utc_anchor_clock_residual_nanoseconds <<
      ",\"gnss_after_anchor_clock_residual_ns\":" <<
      context.gnss->after_utc_anchor_clock_residual_nanoseconds <<
      ",\"before_fix_quality\":" <<
      static_cast<unsigned>(context.gnss->before_fix_quality) <<
      ",\"after_fix_quality\":" <<
      static_cast<unsigned>(context.gnss->after_fix_quality) <<
      ",\"latitude_deg\":" << context.gnss->latitude_degrees <<
      ",\"longitude_deg\":" << context.gnss->longitude_degrees <<
      ",\"altitude_m\":" << context.gnss->altitude_meters;
  }
  if (context.heading_deg && context.pitch_deg) {
    stream_ <<
      ",\"heading_deg\":" << *context.heading_deg <<
      ",\"gnss_pitch_deg\":" << *context.pitch_deg <<
      ",\"heading_before\":" << context.heading_before_sequence <<
      ",\"heading_after\":" << context.heading_after_sequence;
  }

  stream_ <<
    ",\"rsm_status\":\"" << status(context.rsm_status) <<
    "\",\"rsm_detail\":\"" << json_escape(context.rsm_detail) <<
    "\",\"rsm_measurement_type\":\"gimbal_angles\"" <<
    ",\"rsm_reference_frame\":\"rsm400_base_plate\"" <<
    ",\"rsm_earth_referenced\":false";
  if (context.rsm) {
    stream_ <<
      ",\"rsm_before\":" << context.rsm->before_sample_id <<
      ",\"rsm_after\":" << context.rsm->after_sample_id <<
      ",\"rsm_before_age_ns\":" << context.rsm->before_age_nanoseconds <<
      ",\"rsm_after_age_ns\":" << context.rsm->after_age_nanoseconds <<
      ",\"rsm_before_raw_status\":" << context.rsm->before_raw_status <<
      ",\"rsm_after_raw_status\":" << context.rsm->after_raw_status <<
      ",\"roll_deg\":" << context.rsm->roll_deg <<
      ",\"pitch_deg\":" << context.rsm->pitch_deg;
    if (context.rsm->yaw_deg) {
      stream_ << ",\"yaw_valid\":true,\"yaw_deg\":" << *context.rsm->yaw_deg;
    } else {
      stream_ << ",\"yaw_valid\":false,\"yaw_deg\":null";
    }
  }
  stream_ << "}\n";
  if (!stream_) return {false, "context sidecar write failed"};
  ++records_;
  return flush();
}

ContextWriterResult FrameContextWriter::flush()
{
  stream_.flush();
  return stream_ ? ContextWriterResult{true, "flushed"} :
         ContextWriterResult{false, "context sidecar flush failed"};
}

ContextWriterResult FrameContextWriter::close()
{
  if (closed_) return {true, "already closed"};
  const auto result = flush();
  stream_.close();
  closed_ = true;
  if (!result.ok || stream_.fail()) return {false, "context sidecar flush/close failed"};
  return {true, "finalized"};
}

FrameContextWriter::~FrameContextWriter()
{
  if (!closed_) (void)close();
}
}  // namespace ppbng_core
