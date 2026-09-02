#include "ppbng_core/frame_context_association.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ppbng_core
{
namespace
{
double heading_at(const GnssHeadingSample & before, const GnssHeadingSample & after,
  const double fraction)
{
  const double delta = std::remainder(after.heading_deg - before.heading_deg, 360.0);
  return std::fmod(before.heading_deg + fraction * delta + 360.0, 360.0);
}

bool host_fits_i64(const std::uint64_t value)
{
  return value != 0U && value <=
    static_cast<std::uint64_t>((std::numeric_limits<std::int64_t>::max)());
}
}  // namespace

FrameContextAssociator::FrameContextAssociator(
  const std::uint64_t span, const std::uint64_t age, const std::uint64_t rsm_age,
  const std::size_t capacity)
: gnss_max_span_ns_(span), gnss_max_age_ns_(age), rsm_max_age_ns_(rsm_age),
  capacity_(capacity), rsm_(rsm_age, capacity)
{
  if (!span || !age || !rsm_age || capacity < 2U) {
    throw std::invalid_argument("invalid frame context bounds");
  }
}

bool FrameContextAssociator::add_gnss_position(const GnssObservation & sample)
{
  if (!sample.position_valid || sample.utc_nanoseconds <= 0 ||
    !host_fits_i64(sample.host_monotonic_nanoseconds) ||
    (!positions_.empty() &&
    (sample.utc_nanoseconds <= positions_.back().utc_nanoseconds ||
    sample.host_monotonic_nanoseconds <= positions_.back().host_monotonic_nanoseconds ||
    sample.sequence <= positions_.back().sequence)))
  {
    return false;
  }
  positions_.push_back(sample);
  if (positions_.size() > capacity_) {positions_.pop_front();}
  return true;
}

bool FrameContextAssociator::add_gnss_heading(const GnssHeadingSample & sample)
{
  if (!sample.valid || sample.utc_nanoseconds <= 0 || !host_fits_i64(sample.host_monotonic_ns) ||
    !std::isfinite(sample.heading_deg) || !std::isfinite(sample.pitch_deg) ||
    (!headings_.empty() &&
    (sample.utc_nanoseconds <= headings_.back().utc_nanoseconds ||
    sample.host_monotonic_ns <= headings_.back().host_monotonic_ns ||
    sample.sequence <= headings_.back().sequence)))
  {
    return false;
  }
  headings_.push_back(sample);
  if (headings_.size() > capacity_) {headings_.pop_front();}
  return true;
}

bool FrameContextAssociator::add_rsm(const RsmAttitudeSample & sample)
{
  return rsm_.add_sample(sample);
}

FrameContextResult FrameContextAssociator::associate(const FrameContextQuery & query) const
{
  FrameContextResult output;
  const bool utc_mode = query.utc_nanoseconds > 0;
  const bool host_mode = !utc_mode && host_fits_i64(query.host_monotonic_ns);

  if (!utc_mode && !host_mode) {
    output.gnss_detail = "frame UTC and monotonic time unavailable";
  } else {
    auto position_after = utc_mode ?
      std::upper_bound(positions_.begin(), positions_.end(), query.utc_nanoseconds,
        [](const auto target, const auto & sample) {return target < sample.utc_nanoseconds;}) :
      std::upper_bound(positions_.begin(), positions_.end(), query.host_monotonic_ns,
        [](const auto target, const auto & sample) {
          return target < sample.host_monotonic_nanoseconds;
        });
    if (position_after != positions_.begin() && position_after != positions_.end()) {
      const auto before = std::prev(position_after);
      if (utc_mode) {
        output.gnss = associate_gnss(
          *before, *position_after, query.utc_nanoseconds,
          gnss_max_span_ns_, gnss_max_age_ns_);
      } else {
        auto before_axis = *before;
        auto after_axis = *position_after;
        before_axis.utc_nanoseconds = static_cast<std::int64_t>(before->host_monotonic_nanoseconds);
        after_axis.utc_nanoseconds =
          static_cast<std::int64_t>(position_after->host_monotonic_nanoseconds);
        output.gnss = associate_gnss(
          before_axis, after_axis, static_cast<std::int64_t>(query.host_monotonic_ns),
          gnss_max_span_ns_, gnss_max_age_ns_);
        if (output.gnss) {
          const long double utc = static_cast<long double>(before->utc_nanoseconds) +
            static_cast<long double>(position_after->utc_nanoseconds - before->utc_nanoseconds) *
            output.gnss->interpolation_fraction;
          if (utc <= 0.0L || utc >
            static_cast<long double>((std::numeric_limits<std::int64_t>::max)()))
          {
            output.gnss.reset();
          } else {
            output.gnss->utc_nanoseconds = static_cast<std::int64_t>(std::llround(utc));
          }
        }
      }
    }
    if (output.gnss) {
      output.gnss_status = utc_mode ? ContextAvailability::available :
        ContextAvailability::degraded;
      output.gnss_detail = utc_mode ? "strict UTC bracket interpolation" :
        "DEGRADED host-monotonic receive-time bracket; PPS unavailable";
    } else {
      output.gnss_detail = utc_mode ?
        "GNSS UTC bracket missing, stale, invalid, or reordered" :
        "GNSS host-time bracket missing, stale, invalid, or reordered";
    }

    auto heading_after = utc_mode ?
      std::upper_bound(headings_.begin(), headings_.end(), query.utc_nanoseconds,
        [](const auto target, const auto & sample) {return target < sample.utc_nanoseconds;}) :
      std::upper_bound(headings_.begin(), headings_.end(), query.host_monotonic_ns,
        [](const auto target, const auto & sample) {return target < sample.host_monotonic_ns;});
    if (heading_after != headings_.begin() && heading_after != headings_.end()) {
      const auto before = std::prev(heading_after);
      const auto target = utc_mode ? query.utc_nanoseconds :
        static_cast<std::int64_t>(query.host_monotonic_ns);
      const auto before_time = utc_mode ? before->utc_nanoseconds :
        static_cast<std::int64_t>(before->host_monotonic_ns);
      const auto after_time = utc_mode ? heading_after->utc_nanoseconds :
        static_cast<std::int64_t>(heading_after->host_monotonic_ns);
      if (before_time < target && target < after_time) {
        const auto before_age = static_cast<std::uint64_t>(target - before_time);
        const auto after_age = static_cast<std::uint64_t>(after_time - target);
        const auto span = before_age + after_age;
        if (span && span <= gnss_max_span_ns_ && before_age <= gnss_max_age_ns_ &&
          after_age <= gnss_max_age_ns_)
        {
          const double fraction = static_cast<double>(before_age) / static_cast<double>(span);
          output.heading_deg = heading_at(*before, *heading_after, fraction);
          output.pitch_deg = before->pitch_deg + fraction *
            (heading_after->pitch_deg - before->pitch_deg);
          output.heading_before_sequence = before->sequence;
          output.heading_after_sequence = heading_after->sequence;
        }
      }
    }
    if (output.gnss && !output.heading_deg) {
      output.gnss_status = ContextAvailability::degraded;
      output.gnss_detail += "; heading bracket unavailable";
    }
  }

  if (!host_fits_i64(query.host_monotonic_ns)) {
    output.rsm_detail = "frame monotonic time unavailable";
  } else {
    output.rsm = rsm_.associate(static_cast<std::int64_t>(query.host_monotonic_ns));
    if (output.rsm) {
      const bool endpoint_error = output.rsm->before_raw_status != 0U ||
        output.rsm->after_raw_status != 0U;
      output.rsm_status = endpoint_error ? ContextAvailability::degraded :
        ContextAvailability::available;
      output.rsm_detail = endpoint_error ?
        "strict host-monotonic bracket; endpoint RSM error level is nonzero" :
        "strict host-monotonic bracket interpolation";
    } else {
      output.rsm_detail = "RSM monotonic bracket missing, stale, invalid, or reordered";
    }
  }
  return output;
}

void FrameContextAssociator::reset()
{
  positions_.clear();
  headings_.clear();
  rsm_ = RsmAttitudeAssociator(rsm_max_age_ns_, capacity_);
}
}  // namespace ppbng_core
