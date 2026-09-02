#include "ppbng_core/time_authority_correlator.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ppbng_core {
namespace {
constexpr std::uint64_t kNsPerSecond = 1'000'000'000ULL;

std::optional<std::uint64_t> apply_offset(std::uint64_t value, std::int64_t offset) {
  if (offset >= 0) {
    const auto amount = static_cast<std::uint64_t>(offset);
    if (value > (std::numeric_limits<std::uint64_t>::max)() - amount) return std::nullopt;
    return value + amount;
  }
  const auto amount = static_cast<std::uint64_t>(-(offset + 1)) + 1U;
  if (value < amount) return std::nullopt;
  return value - amount;
}

std::uint64_t distance(std::uint64_t left, std::uint64_t right) {
  return left >= right ? left - right : right - left;
}
}  // namespace

TimeAuthorityCorrelator::TimeAuthorityCorrelator(TimeAuthorityConfiguration configuration)
    : configuration_(configuration) {
  if (configuration_.maximum_pps_queue == 0U || configuration_.maximum_gnss_queue == 0U ||
      configuration_.maximum_pending_triggers == 0U ||
      configuration_.trigger_wait_timeout_ns == 0U ||
      configuration_.confirmed_uncertainty_ns == 0U ||
      configuration_.unverified_uncertainty_ns < configuration_.maximum_host_skew_ns ||
      configuration_.maximum_host_skew_ns >
        (std::numeric_limits<std::uint64_t>::max)() - configuration_.trigger_wait_timeout_ns)
    throw std::invalid_argument("invalid time authority bounds/uncertainty");
}

std::optional<std::uint64_t> TimeAuthorityCorrelator::corrected_gnss_host(
    const AuthorityGnssEvidence& value) const {
  const auto delay = static_cast<std::uint64_t>(value.receiver_output_delay_ms) * 1'000'000ULL;
  if (value.host_receive_monotonic_ns < delay) return std::nullopt;
  return value.host_receive_monotonic_ns - delay;
}

std::vector<CorrelatedTrigger> TimeAuthorityCorrelator::add_pps(
    const AuthorityPpsAnchor& anchor) {
  std::vector<CorrelatedTrigger> output;
  if (anchor.boot_id == 0U || anchor.host_receive_monotonic_ns == 0U) {
    ++statistics_.invalid_inputs; return {};
  }
  if (current_boot_id_ && *current_boot_id_ != anchor.boot_id) {
    for (const auto& trigger : triggers_)
      output.push_back(unsynced(trigger, "timing controller reboot before PPS association"));
    triggers_.clear();
    pps_.clear(); gnss_.clear(); mapped_.clear(); last_pps_sequence_.reset();
    last_gnss_epoch_.reset(); last_gnss_utc_second_.reset();
    last_mapped_sequence_.reset(); last_mapped_utc_second_.reset();
  }
  current_boot_id_ = anchor.boot_id;
  if (last_pps_sequence_ && anchor.pps_sequence <= *last_pps_sequence_) {
    ++statistics_.reordered_inputs; return {};
  }
  last_pps_sequence_ = anchor.pps_sequence;
  if (pps_.size() == configuration_.maximum_pps_queue) {
    pps_.pop_front(); ++statistics_.queue_overflows; ++statistics_.anchors_missed;
  }
  pps_.push_back({anchor, false}); correlate();
  auto ready = release_ready();
  output.insert(output.end(), ready.begin(), ready.end());
  return output;
}

std::vector<CorrelatedTrigger> TimeAuthorityCorrelator::add_gnss(
    const AuthorityGnssEvidence& evidence) {
  if (!evidence.uniheadinga || !evidence.fine || evidence.utc_second == kUnknownUtcSecond ||
      evidence.host_receive_monotonic_ns == 0U || !corrected_gnss_host(evidence)) {
    ++statistics_.invalid_inputs; return {};
  }
  if (last_gnss_epoch_ && evidence.connection_epoch < *last_gnss_epoch_) {
    ++statistics_.reordered_inputs; return {};
  }
  std::vector<CorrelatedTrigger> output;
  if (last_gnss_epoch_ && evidence.connection_epoch > *last_gnss_epoch_) {
    for (const auto& trigger : triggers_)
      output.push_back(unsynced(trigger, "GNSS connection epoch changed before PPS association"));
    triggers_.clear();
    pps_.clear();
    gnss_.clear();
    mapped_.clear();
    last_gnss_utc_second_.reset();
    last_mapped_sequence_.reset();
    last_mapped_utc_second_.reset();
  }
  if (last_gnss_epoch_ && evidence.connection_epoch == *last_gnss_epoch_ &&
      last_gnss_utc_second_ && evidence.utc_second <= *last_gnss_utc_second_) {
    ++statistics_.reordered_inputs; return {};
  }
  last_gnss_epoch_ = evidence.connection_epoch;
  last_gnss_utc_second_ = evidence.utc_second;
  if (gnss_.size() == configuration_.maximum_gnss_queue) {
    gnss_.pop_front(); ++statistics_.queue_overflows;
  }
  gnss_.push_back({evidence}); correlate();
  auto ready = release_ready();
  output.insert(output.end(), ready.begin(), ready.end());
  return output;
}

std::vector<CorrelatedTrigger> TimeAuthorityCorrelator::add_trigger(
    const AuthorityTrigger& trigger) {
  if (trigger.channel.empty() || trigger.ticks_per_second == 0U ||
      trigger.arrival_monotonic_ns == 0U || trigger.controller_time_status > 2U) {
    ++statistics_.invalid_inputs; return {unsynced(trigger, "invalid trigger timing fields")};
  }
  if (!configuration_.pps_required) {
    return {unsynced(trigger, "PPS disabled for this task; immediate hardware-clock trigger")};
  }
  if (std::any_of(mapped_.begin(), mapped_.end(), [&](const auto& item) {
        return item.anchor.pps_sequence == trigger.pps_sequence;
      })) return {map(trigger)};
  std::vector<CorrelatedTrigger> output;
  if (triggers_.size() == configuration_.maximum_pending_triggers) {
    output.push_back(unsynced(triggers_.front(), "pending trigger queue overflow"));
    triggers_.pop_front(); ++statistics_.queue_overflows;
  }
  triggers_.push_back(trigger);
  return output;
}

void TimeAuthorityCorrelator::correlate() {
  for (auto pps_it = pps_.begin(); pps_it != pps_.end();) {
    const auto target = apply_offset(pps_it->value.host_receive_monotonic_ns,
                                     configuration_.expected_system_offset_ns);
    if (!target) { ++statistics_.invalid_inputs; pps_it = pps_.erase(pps_it); continue; }
    auto best = gnss_.end();
    auto best_skew = (std::numeric_limits<std::uint64_t>::max)();
    bool tied = false;
    for (auto candidate = gnss_.begin(); candidate != gnss_.end(); ++candidate) {
      const auto corrected = corrected_gnss_host(candidate->value);
      if (!corrected) continue;
      const auto skew = distance(*corrected, *target);
      if (skew > configuration_.maximum_host_skew_ns) continue;
      if (skew < best_skew) { best = candidate; best_skew = skew; tied = false; }
      else if (skew == best_skew) tied = true;
    }
    if (best == gnss_.end()) { ++pps_it; continue; }
    if (tied) {
      if (!pps_it->ambiguity_reported) {
        ++statistics_.ambiguous_candidates;
        pps_it->ambiguity_reported = true;
      }
      ++pps_it; continue;
    }
    const auto sequence_delta = last_mapped_sequence_ &&
        pps_it->value.pps_sequence > *last_mapped_sequence_ ?
      std::optional<std::uint64_t>(pps_it->value.pps_sequence - *last_mapped_sequence_) :
      std::nullopt;
    const auto utc_delta = last_mapped_utc_second_ &&
        best->value.utc_second > *last_mapped_utc_second_ ?
      std::optional<std::uint64_t>(static_cast<std::uint64_t>(
        best->value.utc_second - *last_mapped_utc_second_)) : std::nullopt;
    const bool mapping_monotonic = !last_mapped_sequence_ ||
      (sequence_delta && utc_delta && *sequence_delta == *utc_delta);
    if (!mapping_monotonic) {
      ++statistics_.reordered_inputs; gnss_.erase(best); pps_it = pps_.erase(pps_it); continue;
    }
    last_mapped_sequence_ = pps_it->value.pps_sequence;
    last_mapped_utc_second_ = best->value.utc_second;
    mapped_.push_back({pps_it->value, best->value.utc_second});
    while (mapped_.size() > configuration_.maximum_pps_queue) mapped_.pop_front();
    gnss_.erase(best); pps_it = pps_.erase(pps_it); ++statistics_.anchors_correlated;
  }
}

CorrelatedTrigger TimeAuthorityCorrelator::map(const AuthorityTrigger& trigger) const {
  const auto anchor = std::find_if(mapped_.begin(), mapped_.end(), [&](const auto& item) {
    return item.anchor.pps_sequence == trigger.pps_sequence;
  });
  if (anchor == mapped_.end()) return unsynced(trigger, "PPS anchor not correlated");
  if (trigger.offset_ticks > (std::numeric_limits<std::uint64_t>::max)() / kNsPerSecond)
    return unsynced(trigger, "tick conversion overflow");
  const auto product = trigger.offset_ticks * kNsPerSecond;
  const auto offset_ns = product / trigger.ticks_per_second +
    static_cast<std::uint64_t>((product % trigger.ticks_per_second) >=
      (trigger.ticks_per_second / 2U + trigger.ticks_per_second % 2U));
  if (anchor->utc_second > (std::numeric_limits<std::int64_t>::max)() /
        static_cast<std::int64_t>(kNsPerSecond) ||
      anchor->utc_second < (std::numeric_limits<std::int64_t>::min)() /
        static_cast<std::int64_t>(kNsPerSecond))
    return unsynced(trigger, "UTC conversion overflow");
  CorrelatedTrigger output;
  output.trigger = trigger;
  output.utc_nanoseconds = anchor->utc_second * static_cast<std::int64_t>(kNsPerSecond) +
    static_cast<std::int64_t>(offset_ns);
  if (trigger.controller_time_status == 0U) {
    return unsynced(trigger, "controller reports UNSYNCED despite mapped PPS anchor");
  }
  const bool controller_locked = trigger.controller_time_status == 2U;
  const bool verified_locked = configuration_.pps_gnss_evidence_confirmed && controller_locked;
  output.status = verified_locked ? TimeStatus::locked : TimeStatus::holdover;
  output.uncertainty_nanoseconds = verified_locked ? configuration_.confirmed_uncertainty_ns :
    configuration_.unverified_uncertainty_ns;
  output.detail = verified_locked ? "LOCKED: verified PPS/GNSS hardware evidence" :
    (controller_locked ? "UNVERIFIED PPS/GNSS association; UTC estimate retained as HOLDOVER" :
    "controller PPS holdover; UTC estimate retained as HOLDOVER");
  return output;
}

CorrelatedTrigger TimeAuthorityCorrelator::unsynced(
    const AuthorityTrigger& trigger, const std::string& reason) const {
  return {trigger, 0, (std::numeric_limits<std::uint64_t>::max)(),
          TimeStatus::unsynced, reason + "; sequence retained for offline repair"};
}

std::vector<CorrelatedTrigger> TimeAuthorityCorrelator::release_ready() {
  std::vector<CorrelatedTrigger> output;
  for (auto it = triggers_.begin(); it != triggers_.end();) {
    if (std::any_of(mapped_.begin(), mapped_.end(), [&](const auto& item) {
          return item.anchor.pps_sequence == it->pps_sequence;
        })) {
      output.push_back(map(*it)); it = triggers_.erase(it);
    } else ++it;
  }
  return output;
}

std::vector<CorrelatedTrigger> TimeAuthorityCorrelator::advance(std::uint64_t now) {
  correlate();
  auto output = release_ready();
  for (auto it = triggers_.begin(); it != triggers_.end();) {
    if (now >= it->arrival_monotonic_ns &&
        now - it->arrival_monotonic_ns >= configuration_.trigger_wait_timeout_ns) {
      output.push_back(unsynced(*it, "PPS anchor wait timeout"));
      it = triggers_.erase(it); ++statistics_.trigger_timeouts;
    } else ++it;
  }
  for (auto it = pps_.begin(); it != pps_.end();) {
    if (now >= it->value.host_receive_monotonic_ns &&
        now - it->value.host_receive_monotonic_ns >
          configuration_.maximum_host_skew_ns + configuration_.trigger_wait_timeout_ns) {
      it = pps_.erase(it); ++statistics_.anchors_missed;
    } else ++it;
  }
  return output;
}

}  // namespace ppbng_core
