#include "ppbng_core/rsm_attitude_association.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ppbng_core {
namespace {

std::uint64_t nonnegative_difference(
    std::int64_t later, std::int64_t earlier) {
  return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
}

double interpolate_linear(double before, double after, long double fraction) {
  return before + (after - before) * static_cast<double>(fraction);
}

double normalize_degrees(double angle) {
  double normalized = std::fmod(angle, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  // Avoid returning 360 due to floating-point rounding at the boundary.
  return normalized >= 360.0 ? 0.0 : normalized;
}

double interpolate_circular_degrees(
    double before, double after, long double fraction) {
  const double start = normalize_degrees(before);
  double delta = std::remainder(normalize_degrees(after) - start, 360.0);
  // Exactly opposite headings have two equally short paths. Choose the
  // positive direction deterministically instead of depending on sign-zero.
  if (delta == -180.0) {
    delta = 180.0;
  }
  return normalize_degrees(start + delta * static_cast<double>(fraction));
}

bool valid_numeric_fields(const RsmAttitudeSample& sample) {
  return (!sample.roll_valid || std::isfinite(sample.roll_deg)) &&
         (!sample.pitch_valid || std::isfinite(sample.pitch_deg)) &&
         (!sample.yaw_valid || std::isfinite(sample.yaw_deg));
}

}  // namespace

RsmAttitudeAssociator::RsmAttitudeAssociator(
    std::uint64_t maximum_bracket_age_nanoseconds,
    std::size_t maximum_samples)
    : maximum_bracket_age_nanoseconds_(maximum_bracket_age_nanoseconds),
      maximum_samples_(maximum_samples) {
  if (maximum_bracket_age_nanoseconds_ == 0U || maximum_samples_ < 2U) {
    throw std::invalid_argument(
        "RSM association requires positive age and at least two samples");
  }
}

bool RsmAttitudeAssociator::add_sample(const RsmAttitudeSample& sample) {
  if (!valid_numeric_fields(sample)) {
    return false;
  }
  if (!samples_.empty()) {
    const auto& previous = samples_.back();
    if (sample.utc_nanoseconds <= previous.utc_nanoseconds ||
        sample.sample_id <= previous.sample_id) {
      return false;
    }
  }
  samples_.push_back(sample);
  if (samples_.size() > maximum_samples_) {
    samples_.erase(samples_.begin());
  }
  return true;
}

std::optional<AssociatedRsmAttitude> RsmAttitudeAssociator::associate(
    std::int64_t target_utc_nanoseconds) const {
  if (samples_.size() < 2U) {
    return std::nullopt;
  }
  const auto after = std::upper_bound(
      samples_.begin(), samples_.end(), target_utc_nanoseconds,
      [](std::int64_t target, const RsmAttitudeSample& sample) {
        return target < sample.utc_nanoseconds;
      });
  if (after == samples_.begin() || after == samples_.end()) {
    return std::nullopt;
  }
  const auto before = std::prev(after);
  const auto before_age =
      nonnegative_difference(target_utc_nanoseconds, before->utc_nanoseconds);
  const auto after_age =
      nonnegative_difference(after->utc_nanoseconds, target_utc_nanoseconds);
  if (before_age > maximum_bracket_age_nanoseconds_ ||
      after_age > maximum_bracket_age_nanoseconds_) {
    return std::nullopt;
  }
  if (!before->sample_status_valid || !after->sample_status_valid ||
      !before->roll_valid || !after->roll_valid ||
      !before->pitch_valid || !after->pitch_valid) {
    return std::nullopt;
  }

  const auto span = nonnegative_difference(
      after->utc_nanoseconds, before->utc_nanoseconds);
  if (span == 0U) {
    return std::nullopt;
  }
  const long double fraction =
      static_cast<long double>(before_age) / static_cast<long double>(span);

  AssociatedRsmAttitude result;
  result.target_utc_nanoseconds = target_utc_nanoseconds;
  result.roll_deg = interpolate_linear(before->roll_deg, after->roll_deg, fraction);
  result.pitch_deg = interpolate_linear(before->pitch_deg, after->pitch_deg, fraction);
  if (before->yaw_valid && after->yaw_valid) {
    result.yaw_deg = interpolate_circular_degrees(
        before->yaw_deg, after->yaw_deg, fraction);
  }
  result.before_sample_id = before->sample_id;
  result.after_sample_id = after->sample_id;
  result.before_age_nanoseconds = before_age;
  result.after_age_nanoseconds = after_age;
  result.before_raw_status = before->raw_status;
  result.after_raw_status = after->raw_status;
  return result;
}

std::size_t RsmAttitudeAssociator::sample_count() const noexcept {
  return samples_.size();
}

}  // namespace ppbng_core
