#include "ppbng_core/frame_trigger_association.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace ppbng_core
{

FrameTriggerAssociator::FrameTriggerAssociator(const std::size_t maximum_pending_per_stream)
: maximum_pending_per_stream_(maximum_pending_per_stream)
{
  if (maximum_pending_per_stream_ == 0) {
    throw std::invalid_argument("maximum pending observations must be greater than zero");
  }
}

void FrameTriggerAssociator::begin_segment(
  const std::uint64_t segment_id, const std::uint64_t expected_first_trigger_sequence)
{
  segment_started_ = true;
  segment_id_ = segment_id;
  anchor_trigger_sequence_ = expected_first_trigger_sequence;
  anchor_frame_id_.reset();
  last_trigger_sequence_.reset();
  last_frame_id_.reset();
  observed_trigger_gaps_ = 0;
  observed_frame_gaps_ = 0;
  matched_count_ = 0;
  triggers_.clear();
  frames_.clear();
  trigger_gap_by_sequence_.clear();
  frame_gap_by_id_.clear();
}

AssociationUpdate FrameTriggerAssociator::observe_trigger(const TriggerObservation & trigger)
{
  if (!segment_started_) {
    return {false, "segment has not been started", {}};
  }
  if (trigger.channel_sequence < anchor_trigger_sequence_) {
    return {false, "trigger precedes the segment anchor", {}};
  }
  if (last_trigger_sequence_ && trigger.channel_sequence <= *last_trigger_sequence_) {
    return {false, "trigger sequence is duplicate or regressed", {}};
  }
  if (triggers_.size() >= maximum_pending_per_stream_) {
    return {false, "pending association limit reached", {}};
  }

  const std::uint64_t gap = last_trigger_sequence_ ?
    trigger.channel_sequence - *last_trigger_sequence_ - 1U :
    trigger.channel_sequence - anchor_trigger_sequence_;
  observed_trigger_gaps_ += gap;
  trigger_gap_by_sequence_.emplace(trigger.channel_sequence, gap);
  triggers_.emplace(trigger.channel_sequence, trigger);
  last_trigger_sequence_ = trigger.channel_sequence;
  return drain_matches("trigger accepted");
}

AssociationUpdate FrameTriggerAssociator::observe_frame(const CameraFrameObservation & frame)
{
  if (!segment_started_) {
    return {false, "segment has not been started", {}};
  }
  if (last_frame_id_ && frame.frame_id <= *last_frame_id_) {
    return {false, "camera frame ID is duplicate or regressed", {}};
  }
  if (frames_.size() >= maximum_pending_per_stream_) {
    return {false, "pending association limit reached", {}};
  }

  if (!anchor_frame_id_) {
    anchor_frame_id_ = frame.frame_id;
  }
  const std::uint64_t gap = last_frame_id_ ? frame.frame_id - *last_frame_id_ - 1U : 0U;
  observed_frame_gaps_ += gap;
  frame_gap_by_id_.emplace(frame.frame_id, gap);
  frames_.emplace(frame.frame_id, frame);
  last_frame_id_ = frame.frame_id;
  return drain_matches("frame accepted");
}

AssociationSummary FrameTriggerAssociator::summary() const noexcept
{
  return {segment_id_, matched_count_, triggers_.size(), frames_.size(),
    observed_trigger_gaps_, observed_frame_gaps_};
}

AssociationUpdate FrameTriggerAssociator::drain_matches(std::string accepted_detail)
{
  AssociationUpdate update{true, std::move(accepted_detail), {}};
  if (!anchor_frame_id_) {
    return update;
  }

  for (auto frame_it = frames_.begin(); frame_it != frames_.end();) {
    const std::uint64_t frame_delta = frame_it->first - *anchor_frame_id_;
    if (frame_delta > std::numeric_limits<std::uint64_t>::max() - anchor_trigger_sequence_) {
      update.accepted = false;
      update.detail = "frame-to-trigger mapping overflow";
      return update;
    }
    const std::uint64_t mapped_trigger = anchor_trigger_sequence_ + frame_delta;
    const auto trigger_it = triggers_.find(mapped_trigger);
    if (trigger_it == triggers_.end()) {
      ++frame_it;
      continue;
    }

    const auto trigger_gap = trigger_gap_by_sequence_.find(mapped_trigger);
    const auto frame_gap = frame_gap_by_id_.find(frame_it->first);
    update.matches.push_back(FrameTriggerMatch{
      segment_id_, trigger_it->second, frame_it->second,
      trigger_gap == trigger_gap_by_sequence_.end() ? 0U : trigger_gap->second,
      frame_gap == frame_gap_by_id_.end() ? 0U : frame_gap->second,
      true});
    trigger_gap_by_sequence_.erase(mapped_trigger);
    frame_gap_by_id_.erase(frame_it->first);
    triggers_.erase(trigger_it);
    frame_it = frames_.erase(frame_it);
    ++matched_count_;
  }
  return update;
}

}  // namespace ppbng_core
