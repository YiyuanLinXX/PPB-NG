#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ppbng_core
{

enum class FrameTimeQuality : std::uint8_t {unsynced = 0, holdover = 1, locked = 2};

struct TriggerObservation
{
  std::uint64_t event_id{0};
  std::uint64_t channel_sequence{0};
  std::uint64_t pps_sequence{0};
  std::int64_t utc_time_ns{0};
  FrameTimeQuality time_quality{FrameTimeQuality::unsynced};
  std::uint64_t uncertainty_ns{0};
  std::int64_t host_observed_monotonic_ns{0};
};

struct CameraFrameObservation
{
  std::uint64_t frame_id{0};
  std::optional<std::uint64_t> camera_timestamp_ns;
  std::int64_t host_receive_monotonic_ns{0};
  bool complete{true};
};

struct FrameTriggerMatch
{
  std::uint64_t segment_id{0};
  TriggerObservation trigger;
  CameraFrameObservation frame;
  std::uint64_t trigger_gap_before{0};
  std::uint64_t frame_gap_before{0};
  bool exact_sequence_mapping{true};
};

struct AssociationUpdate
{
  bool accepted{false};
  std::string detail;
  std::vector<FrameTriggerMatch> matches;
};

struct AssociationSummary
{
  std::uint64_t segment_id{0};
  std::size_t matched_count{0};
  std::size_t pending_trigger_count{0};
  std::size_t pending_frame_count{0};
  std::uint64_t observed_trigger_gaps{0};
  std::uint64_t observed_frame_gaps{0};
};

// Associates two independently delivered ordered streams without using host arrival order.
// The first camera frame in a new segment is explicitly mapped to
// expected_first_trigger_sequence. Camera frame-ID deltas must then equal trigger sequence
// deltas. A reconnect or camera counter reset requires a new segment and a new associator.
class FrameTriggerAssociator
{
public:
  explicit FrameTriggerAssociator(std::size_t maximum_pending_per_stream = 4096);

  void begin_segment(std::uint64_t segment_id, std::uint64_t expected_first_trigger_sequence);
  AssociationUpdate observe_trigger(const TriggerObservation & trigger);
  AssociationUpdate observe_frame(const CameraFrameObservation & frame);
  AssociationSummary summary() const noexcept;

private:
  AssociationUpdate drain_matches(std::string accepted_detail);
  std::size_t maximum_pending_per_stream_;
  bool segment_started_{false};
  std::uint64_t segment_id_{0};
  std::uint64_t anchor_trigger_sequence_{0};
  std::optional<std::uint64_t> anchor_frame_id_;
  std::optional<std::uint64_t> last_trigger_sequence_;
  std::optional<std::uint64_t> last_frame_id_;
  std::uint64_t observed_trigger_gaps_{0};
  std::uint64_t observed_frame_gaps_{0};
  std::size_t matched_count_{0};
  std::map<std::uint64_t, TriggerObservation> triggers_;
  std::map<std::uint64_t, CameraFrameObservation> frames_;
  std::map<std::uint64_t, std::uint64_t> trigger_gap_by_sequence_;
  std::map<std::uint64_t, std::uint64_t> frame_gap_by_id_;
};

}  // namespace ppbng_core
