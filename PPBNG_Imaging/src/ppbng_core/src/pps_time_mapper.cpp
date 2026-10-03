#include "ppbng_core/pps_time_mapper.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ppbng_core
{

namespace
{
constexpr std::uint64_t nanoseconds_per_second = 1'000'000'000ULL;

std::optional<std::uint64_t> ticks_to_nanoseconds(
  const std::uint64_t ticks,
  const std::uint64_t ticks_per_second)
{
  if (ticks_per_second == 0 || ticks >= ticks_per_second ||
    ticks > std::numeric_limits<std::uint64_t>::max() / nanoseconds_per_second)
  {
    return std::nullopt;
  }

  // Round half up. The range check keeps the multiplication exact and overflow-free.
  const auto product = ticks * nanoseconds_per_second;
  const auto quotient = product / ticks_per_second;
  const auto remainder = product % ticks_per_second;
  return quotient + static_cast<std::uint64_t>(
    remainder >= (ticks_per_second / 2 + ticks_per_second % 2));
}

std::optional<std::int64_t> checked_add(
  const std::int64_t base,
  const std::uint64_t positive_offset)
{
  if (positive_offset > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  const auto signed_offset = static_cast<std::int64_t>(positive_offset);
  if (base > std::numeric_limits<std::int64_t>::max() - signed_offset) {
    return std::nullopt;
  }
  return base + signed_offset;
}

std::optional<std::uint64_t> holdover_uncertainty(
  const std::uint64_t duration_nanoseconds,
  const std::uint64_t drift_ppb)
{
  const auto whole_seconds = duration_nanoseconds / nanoseconds_per_second;
  const auto remainder_nanoseconds = duration_nanoseconds % nanoseconds_per_second;
  if (drift_ppb != 0 &&
    whole_seconds > std::numeric_limits<std::uint64_t>::max() / drift_ppb)
  {
    return std::nullopt;
  }
  const auto whole = whole_seconds * drift_ppb;
  if (remainder_nanoseconds != 0 &&
    drift_ppb > std::numeric_limits<std::uint64_t>::max() / remainder_nanoseconds)
  {
    return std::nullopt;
  }
  const auto fractional_product = remainder_nanoseconds * drift_ppb;
  const auto fractional = fractional_product / nanoseconds_per_second +
    static_cast<std::uint64_t>(fractional_product % nanoseconds_per_second != 0);
  if (whole > std::numeric_limits<std::uint64_t>::max() - fractional) {
    return std::nullopt;
  }
  return whole + fractional;
}
}

PpsTimeMapper::PpsTimeMapper(const std::size_t maximum_anchors)
: maximum_anchors_(maximum_anchors)
{
  if (maximum_anchors_ == 0) {
    throw std::invalid_argument("maximum_anchors must be greater than zero");
  }
}

bool PpsTimeMapper::add_anchor(
  const std::uint64_t pps_sequence,
  const std::int64_t utc_nanoseconds)
{
  const auto existing = anchors_.find(pps_sequence);
  if (existing != anchors_.end()) {
    return existing->second == utc_nanoseconds;
  }

  anchors_.emplace(pps_sequence, utc_nanoseconds);
  while (anchors_.size() > maximum_anchors_) {
    const auto oldest = std::min_element(
      anchors_.begin(), anchors_.end(),
      [](const auto & left, const auto & right) {return left.first < right.first;});
    anchors_.erase(oldest);
  }
  return true;
}

std::optional<MappedTime> PpsTimeMapper::map_event(
  const std::uint64_t pps_sequence,
  const std::uint64_t offset_ticks,
  const std::uint64_t ticks_per_second,
  const bool controller_locked,
  const std::uint64_t controller_uncertainty_nanoseconds,
  const std::uint64_t maximum_holdover_sequences,
  const std::uint64_t oscillator_drift_ppb) const
{
  const auto subsecond_nanoseconds = ticks_to_nanoseconds(offset_ticks, ticks_per_second);
  if (!subsecond_nanoseconds.has_value()) {
    return std::nullopt;
  }

  auto anchor = anchors_.find(pps_sequence);
  std::uint64_t elapsed_sequences = 0;
  if (anchor == anchors_.end()) {
    if (controller_locked || maximum_holdover_sequences == 0) {
      return std::nullopt;
    }
    anchor = std::max_element(
      anchors_.begin(), anchors_.end(),
      [pps_sequence](const auto & left, const auto & right) {
        const bool left_usable = left.first < pps_sequence;
        const bool right_usable = right.first < pps_sequence;
        if (left_usable != right_usable) {
          return !left_usable;
        }
        return left.first < right.first;
      });
    if (anchor == anchors_.end() || anchor->first >= pps_sequence) {
      return std::nullopt;
    }
    elapsed_sequences = pps_sequence - anchor->first;
    if (elapsed_sequences > maximum_holdover_sequences) {
      return std::nullopt;
    }
  }

  if (elapsed_sequences >
    (std::numeric_limits<std::uint64_t>::max() - *subsecond_nanoseconds) /
    nanoseconds_per_second)
  {
    return std::nullopt;
  }
  const auto elapsed_nanoseconds =
    elapsed_sequences * nanoseconds_per_second + *subsecond_nanoseconds;
  const auto mapped = checked_add(anchor->second, elapsed_nanoseconds);
  if (!mapped.has_value()) {
    return std::nullopt;
  }

  auto uncertainty = controller_uncertainty_nanoseconds;
  if (!controller_locked) {
    const auto drift = holdover_uncertainty(elapsed_nanoseconds, oscillator_drift_ppb);
    if (!drift.has_value() || uncertainty > std::numeric_limits<std::uint64_t>::max() - *drift) {
      return std::nullopt;
    }
    uncertainty += *drift;
  }

  MappedTime result;
  result.utc_nanoseconds = *mapped;
  result.uncertainty_nanoseconds = uncertainty;
  result.status = controller_locked ? TimeStatus::locked : TimeStatus::holdover;
  result.detail = controller_locked ? "PPS locked" :
    "controller holdover from PPS sequence " + std::to_string(anchor->first);
  return result;
}

std::size_t PpsTimeMapper::anchor_count() const noexcept
{
  return anchors_.size();
}

}  // namespace ppbng_core
