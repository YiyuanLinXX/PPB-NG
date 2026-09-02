#pragma once

#include "ppbng_hsi/hsi_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace ppbng_hsi
{
enum class TriggerMatchStatus {idle, waiting_for_frame, produced, expired, invalid, disconnected, fault};
struct TriggerEnqueueResult {bool accepted{false}; bool overflow{false}; std::string detail;};
struct TriggerMatchResult
{
  TriggerMatchStatus status{TriggerMatchStatus::idle};
  std::optional<LineRecord> line;
  std::string detail;
};

class PendingTriggerMatcher
{
public:
  PendingTriggerMatcher(std::size_t capacity, std::uint64_t maximum_age_ns,
    bool association_evidence_confirmed = false);
  TriggerEnqueueResult enqueue(const TriggerEvent & trigger, std::uint64_t arrival_monotonic_ns);
  TriggerMatchResult poll(IHsiAdapter & adapter, std::uint64_t now_monotonic_ns);
  void clear() noexcept;
  // Recovery boundary: forget every timing/counter association and force the
  // next persisted line into a distinct logical source segment.
  void start_new_segment() noexcept;
  std::size_t pending() const noexcept {return queue_.size();}
  std::uint64_t expired_count() const noexcept {return expired_count_;}
  std::uint64_t overflow_count() const noexcept {return overflow_count_;}
  bool configured() const noexcept {return capacity_ > 0U && maximum_age_ns_ > 0U;}
  std::uint32_t source_segment() const noexcept {return source_segment_;}
private:
  struct Pending {TriggerEvent trigger; std::uint64_t arrival_ns; bool boundary_before{false};};
  std::size_t capacity_;
  std::uint64_t maximum_age_ns_;
  bool association_evidence_confirmed_;
  std::uint64_t highest_sequence_{0U};
  std::uint64_t expired_count_{0U};
  std::uint64_t overflow_count_{0U};
  std::uint32_t source_segment_{0U};
  std::uint64_t source_segment_line_{0U};
  bool segment_started_{false};
  bool boundary_before_next_{true};
  bool boundary_on_next_enqueue_{false};
  bool anchor_candidate_valid_{false};
  bool anchor_confirmed_{false};
  std::int64_t anchor_delta_{0};
  std::uint64_t anchor_frame_{0U};
  std::uint64_t anchor_trigger_{0U};
  std::uint64_t previous_frame_{0U};
  bool previous_frame_valid_{false};
  std::uint32_t previous_sdk_segment_{0U};
  bool previous_sdk_segment_valid_{false};
  std::deque<Pending> queue_;
};
}  // namespace ppbng_hsi
