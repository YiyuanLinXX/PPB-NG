#include "ppbng_hsi/sample_stamp_mapper.hpp"

#include <cstdint>
#include <limits>

namespace ppbng_hsi
{
namespace
{
const char * association_text(const AssociationStatus value) noexcept
{
  switch (value) {
    case AssociationStatus::matched: return "MATCHED";
    case AssociationStatus::unmatched: return "UNMATCHED";
    case AssociationStatus::consistent_unverified: return "CONSISTENT_UNVERIFIED";
    default: return "UNVERIFIED";
  }
}

const char * time_text(const TimeStatus value) noexcept
{
  switch (value) {
    case TimeStatus::locked: return "LOCKED";
    case TimeStatus::holdover: return "HOLDOVER";
    default: return "UNSYNCED";
  }
}
}  // namespace

std::optional<ppbng_interfaces::msg::SampleStamp> make_sample_stamp(
  const LineRecord & line, const std::string & session_id)
{
  if (line.index.capture_kind != CaptureKind::sample) return std::nullopt;

  ppbng_interfaces::msg::SampleStamp stamp;
  stamp.session_id = session_id;
  stamp.device_id = line.device_id;
  stamp.segment_id = line.index.segment_id;
  stamp.sample_sequence = line.index.segment_line_index;
  stamp.trigger_channel = line.trigger.channel;
  stamp.trigger_sequence = line.index.trigger_sequence;
  stamp.pps_sequence = line.index.pps_sequence;
  stamp.controller_tick = line.trigger.offset_ticks;
  stamp.controller_ticks_per_second = line.trigger.ticks_per_second;
  stamp.time_quality.pps_sequence = line.index.pps_sequence;
  stamp.time_quality.hardware_tick = line.trigger.offset_ticks;
  stamp.time_quality.uncertainty_ns = line.index.uncertainty_ns;

  const auto raw_status = line.index.time_status;
  const bool matched = line.index.association_status == AssociationStatus::matched;
  stamp.time_quality.status = matched ? static_cast<std::uint8_t>(raw_status) :
    ppbng_interfaces::msg::TimeQuality::UNSYNCED;

  bool utc_representable = false;
  if (line.index.utc_time_ns > 0) {
    const auto seconds = line.index.utc_time_ns / 1'000'000'000LL;
    const auto nanoseconds = line.index.utc_time_ns % 1'000'000'000LL;
    if (seconds <= (std::numeric_limits<std::int32_t>::max)()) {
      stamp.time_quality.utc_time.sec = static_cast<std::int32_t>(seconds);
      stamp.time_quality.utc_time.nanosec = static_cast<std::uint32_t>(nanoseconds);
      utc_representable = true;
    }
  }
  stamp.time_quality.detail = std::string("association=") +
    association_text(line.index.association_status) + "; raw_trigger_time_status=" +
    time_text(raw_status) + "; raw_trigger_utc=" +
    (utc_representable ? "preserved" : "unavailable_or_out_of_range");
  if (!matched) stamp.time_quality.detail += "; canonical_status_forced_UNSYNCED";

  stamp.camera_frame_id_valid = true;
  stamp.camera_frame_id = line.index.camera_line_sequence;
  stamp.camera_timestamp_valid = false;
  stamp.camera_timestamp = 0U;
  stamp.host_receive_monotonic_ns = line.index.host_receive_monotonic_ns;
  return stamp;
}

}  // namespace ppbng_hsi
