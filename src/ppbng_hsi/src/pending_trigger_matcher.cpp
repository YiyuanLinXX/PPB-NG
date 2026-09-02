#include "ppbng_hsi/pending_trigger_matcher.hpp"

#include <limits>

namespace ppbng_hsi
{
PendingTriggerMatcher::PendingTriggerMatcher(const std::size_t capacity,
  const std::uint64_t maximum_age_ns, const bool association_evidence_confirmed)
: capacity_(capacity), maximum_age_ns_(maximum_age_ns),
  association_evidence_confirmed_(association_evidence_confirmed) {}

TriggerEnqueueResult PendingTriggerMatcher::enqueue(const TriggerEvent & trigger,
  const std::uint64_t arrival_monotonic_ns)
{
  if (capacity_ == 0U || maximum_age_ns_ == 0U) {return {false, false, "matcher is disabled"};}
  if (trigger.channel_sequence == 0U ||
    (highest_sequence_ != 0U && trigger.channel_sequence <= highest_sequence_))
  {
    return {false, false, "duplicate or out-of-order trigger sequence"};
  }
  const bool sequence_gap = highest_sequence_ != 0U &&
    trigger.channel_sequence != highest_sequence_ + 1U;
  highest_sequence_ = trigger.channel_sequence;
  if (queue_.size() >= capacity_) {
    ++overflow_count_;
    boundary_on_next_enqueue_ = true;
    return {false, true, "pending trigger queue overflow"};
  }
  queue_.push_back({trigger, arrival_monotonic_ns, sequence_gap || boundary_on_next_enqueue_});
  boundary_on_next_enqueue_ = false;
  return {true, false, "trigger queued for SDK frame"};
}

TriggerMatchResult PendingTriggerMatcher::poll(IHsiAdapter & adapter,
  const std::uint64_t now_monotonic_ns)
{
  if (queue_.empty()) {return {};}
  auto & pending = queue_.front();
  if (now_monotonic_ns >= pending.arrival_ns &&
    now_monotonic_ns - pending.arrival_ns > maximum_age_ns_)
  {
    queue_.pop_front();
    ++expired_count_;
    boundary_before_next_ = true;
    return {TriggerMatchStatus::expired, std::nullopt, "trigger expired before SDK frame arrived"};
  }
  bool source_boundary_applied = false;
  const auto reset_anchor = [this, &source_boundary_applied](const bool increment) {
      // Several pieces of evidence can describe the same boundary (for example an
      // explicit recovery, an SDK-segment change, and a first-frame counter gap).
      // Only advance after the current source segment has actually received a line;
      // otherwise reuse the already-open empty segment.
      if (increment && segment_started_ && source_segment_line_ != 0U) {++source_segment_;}
      segment_started_ = true;
      source_segment_line_ = 0U;
      source_boundary_applied = true;
      anchor_candidate_valid_ = false;
      anchor_confirmed_ = false;
      previous_frame_valid_ = false;
      previous_sdk_segment_valid_ = false;
    };
  if (boundary_before_next_ || pending.boundary_before) {
    reset_anchor(segment_started_);
    boundary_before_next_ = false;
    pending.boundary_before = false;
  }
  auto result = adapter.on_trigger(pending.trigger);
  if (result.status == LineStatus::not_ready) {
    return {TriggerMatchStatus::waiting_for_frame, std::nullopt, result.message};
  }
  queue_.pop_front();
  if (result.status == LineStatus::produced) {
    auto & line = *result.line;
    line.index.sdk_segment_id = line.index.segment_id;
    const bool sdk_segment_changed = previous_sdk_segment_valid_ &&
      line.index.sdk_segment_id != previous_sdk_segment_;
    if (sdk_segment_changed && !source_boundary_applied) {reset_anchor(true);}
    const auto frame = line.index.camera_line_sequence;
    const auto trigger = line.index.trigger_sequence;
    bool delta_representable = frame <= static_cast<std::uint64_t>((std::numeric_limits<std::int64_t>::max)()) &&
      trigger <= static_cast<std::uint64_t>((std::numeric_limits<std::int64_t>::max)());
    const auto delta = delta_representable ? static_cast<std::int64_t>(frame) -
      static_cast<std::int64_t>(trigger) : 0;
    const bool frame_regression = previous_frame_valid_ && frame <= previous_frame_;
    const bool frame_gap = line.index.sequence_gap_before ||
      (previous_frame_valid_ && frame > previous_frame_ && frame - previous_frame_ != 1U);
    const bool delta_mismatch = anchor_candidate_valid_ &&
      (!delta_representable || delta != anchor_delta_);
    if (frame_regression || frame_gap || delta_mismatch || !delta_representable) {
      if (!source_boundary_applied) {reset_anchor(true);}
      line.index.association_status = AssociationStatus::unmatched;
      line.index.association_anchor_valid = false;
      line.index.frame_trigger_delta = delta;
    } else if (!anchor_candidate_valid_) {
      anchor_candidate_valid_ = true;
      anchor_delta_ = delta;
      anchor_frame_ = frame;
      anchor_trigger_ = trigger;
      line.index.association_status = AssociationStatus::unverified;
      line.index.association_anchor_valid = false;
      line.index.frame_trigger_delta = delta;
      line.index.association_anchor_frame = anchor_frame_;
      line.index.association_anchor_trigger = anchor_trigger_;
    } else {
      anchor_confirmed_ = true;
      line.index.association_status = association_evidence_confirmed_ ?
        AssociationStatus::matched : AssociationStatus::consistent_unverified;
      line.index.association_anchor_valid = association_evidence_confirmed_;
      line.index.frame_trigger_delta = anchor_delta_;
      line.index.association_anchor_frame = anchor_frame_;
      line.index.association_anchor_trigger = anchor_trigger_;
    }
    line.index.segment_id = source_segment_;
    line.index.segment_line_index = source_segment_line_++;
    previous_frame_ = frame;
    previous_frame_valid_ = true;
    previous_sdk_segment_ = line.index.sdk_segment_id;
    previous_sdk_segment_valid_ = true;
    return {TriggerMatchStatus::produced, std::move(result.line), result.message};
  }
  if (result.status == LineStatus::disconnected) {
    return {TriggerMatchStatus::disconnected, std::nullopt, result.message};
  }
  if (result.status == LineStatus::fault) {
    return {TriggerMatchStatus::fault, std::nullopt, result.message};
  }
  return {TriggerMatchStatus::invalid, std::nullopt, result.message};
}

void PendingTriggerMatcher::clear() noexcept
{
  queue_.clear();
  highest_sequence_ = 0U;
  boundary_on_next_enqueue_ = false;
  boundary_before_next_ = false;
  if (segment_started_ && source_segment_line_ != 0U) {++source_segment_;}
  source_segment_line_ = 0U;
  anchor_candidate_valid_ = false;
  anchor_confirmed_ = false;
  previous_frame_valid_ = false;
  previous_sdk_segment_valid_ = false;
}

void PendingTriggerMatcher::start_new_segment() noexcept
{
  const bool had_segment = segment_started_;
  clear();
  if (!had_segment) {segment_started_ = true;}
}
}  // namespace ppbng_hsi
