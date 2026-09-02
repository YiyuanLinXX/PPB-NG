#include "ppbng_core/gnss_association.hpp"

#include <cmath>

namespace ppbng_core
{
namespace
{
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double degrees_to_radians = pi / 180.0;
constexpr double radians_to_degrees = 180.0 / pi;
constexpr double wgs84_a = 6378137.0;
constexpr double wgs84_f = 1.0 / 298.257223563;
constexpr double wgs84_e2 = wgs84_f * (2.0 - wgs84_f);

struct Ecef {double x; double y; double z;};

bool finite_observation(const GnssObservation & sample)
{
  return sample.position_valid && std::isfinite(sample.latitude_degrees) &&
    std::isfinite(sample.longitude_degrees) && std::isfinite(sample.altitude_meters) &&
    sample.latitude_degrees >= -90.0 && sample.latitude_degrees <= 90.0 &&
    sample.longitude_degrees >= -180.0 && sample.longitude_degrees <= 180.0;
}

Ecef to_ecef(const GnssObservation & sample)
{
  const double latitude = sample.latitude_degrees * degrees_to_radians;
  const double longitude = sample.longitude_degrees * degrees_to_radians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double prime_vertical = wgs84_a / std::sqrt(1.0 - wgs84_e2 * sin_latitude * sin_latitude);
  return {
    (prime_vertical + sample.altitude_meters) * cos_latitude * std::cos(longitude),
    (prime_vertical + sample.altitude_meters) * cos_latitude * std::sin(longitude),
    (prime_vertical * (1.0 - wgs84_e2) + sample.altitude_meters) * sin_latitude};
}

bool from_ecef(const Ecef & point, double & latitude_degrees, double & longitude_degrees,
  double & altitude_meters)
{
  const double horizontal = std::hypot(point.x, point.y);
  if (!std::isfinite(horizontal) || !std::isfinite(point.z) ||
    (horizontal == 0.0 && point.z == 0.0))
  {
    return false;
  }
  double latitude = std::atan2(point.z, horizontal * (1.0 - wgs84_e2));
  double altitude = 0.0;
  for (int iteration = 0; iteration < 10; ++iteration) {
    const double sine = std::sin(latitude);
    const double prime_vertical = wgs84_a / std::sqrt(1.0 - wgs84_e2 * sine * sine);
    const double cosine = std::cos(latitude);
    if (std::abs(cosine) < 1e-15) {
      altitude = std::abs(point.z) - wgs84_a * std::sqrt(1.0 - wgs84_e2);
      break;
    }
    altitude = horizontal / cosine - prime_vertical;
    const double updated = std::atan2(
      point.z, horizontal * (1.0 - wgs84_e2 * prime_vertical / (prime_vertical + altitude)));
    if (std::abs(updated - latitude) < 1e-14) {
      latitude = updated;
      break;
    }
    latitude = updated;
  }
  latitude_degrees = latitude * radians_to_degrees;
  longitude_degrees = std::atan2(point.y, point.x) * radians_to_degrees;
  altitude_meters = altitude;
  return std::isfinite(latitude_degrees) && std::isfinite(longitude_degrees) &&
    std::isfinite(altitude_meters);
}

double circular_interpolate_degrees(const double start, const double end, const double fraction)
{
  const double normalized_start = std::fmod(std::fmod(start, 360.0) + 360.0, 360.0);
  const double normalized_end = std::fmod(std::fmod(end, 360.0) + 360.0, 360.0);
  const double shortest_delta = std::fmod(normalized_end - normalized_start + 540.0, 360.0) - 180.0;
  return std::fmod(normalized_start + fraction * shortest_delta + 360.0, 360.0);
}
}  // namespace

std::optional<AssociatedGnss> associate_gnss(
  const GnssObservation & before, const GnssObservation & after,
  const std::int64_t target_utc_nanoseconds, const std::uint64_t max_bracket_span_nanoseconds,
  const std::uint64_t max_endpoint_age_nanoseconds)
{
  if (!finite_observation(before) || !finite_observation(after) ||
    max_bracket_span_nanoseconds == 0 || max_endpoint_age_nanoseconds == 0 ||
    before.utc_nanoseconds < 0 || after.utc_nanoseconds < 0 || target_utc_nanoseconds < 0 ||
    before.utc_nanoseconds >= after.utc_nanoseconds ||
    target_utc_nanoseconds < before.utc_nanoseconds || target_utc_nanoseconds > after.utc_nanoseconds)
  {
    return std::nullopt;
  }
  const auto span = static_cast<std::uint64_t>(after.utc_nanoseconds - before.utc_nanoseconds);
  const auto before_age = static_cast<std::uint64_t>(target_utc_nanoseconds - before.utc_nanoseconds);
  const auto after_age = static_cast<std::uint64_t>(after.utc_nanoseconds - target_utc_nanoseconds);
  if (span > max_bracket_span_nanoseconds || before_age > max_endpoint_age_nanoseconds ||
    after_age > max_endpoint_age_nanoseconds)
  {
    return std::nullopt;
  }
  const double fraction = static_cast<double>(before_age) / static_cast<double>(span);
  const auto first = to_ecef(before);
  const auto second = to_ecef(after);
  const Ecef point{
    first.x + fraction * (second.x - first.x), first.y + fraction * (second.y - first.y),
    first.z + fraction * (second.z - first.z)};

  AssociatedGnss result;
  result.utc_nanoseconds = target_utc_nanoseconds;
  result.before_sequence = before.sequence;
  result.after_sequence = after.sequence;
  result.before_age_nanoseconds = before_age;
  result.after_age_nanoseconds = after_age;
  result.interpolation_fraction = fraction;
  if (!from_ecef(point, result.latitude_degrees, result.longitude_degrees, result.altitude_meters)) {
    return std::nullopt;
  }
  result.before_fix_quality = before.fix_quality;
  result.after_fix_quality = after.fix_quality;
  result.before_utc_anchor_sequence = before.utc_anchor_sequence;
  result.after_utc_anchor_sequence = after.utc_anchor_sequence;
  result.before_utc_anchor_host_delta_nanoseconds = before.utc_anchor_host_delta_nanoseconds;
  result.after_utc_anchor_host_delta_nanoseconds = after.utc_anchor_host_delta_nanoseconds;
  result.before_utc_anchor_clock_residual_nanoseconds =
    before.utc_anchor_clock_residual_nanoseconds;
  result.after_utc_anchor_clock_residual_nanoseconds = after.utc_anchor_clock_residual_nanoseconds;
  result.heading_valid = before.heading_valid && after.heading_valid &&
    std::isfinite(before.heading_degrees) && std::isfinite(after.heading_degrees) &&
    std::isfinite(before.pitch_degrees) && std::isfinite(after.pitch_degrees);
  if (result.heading_valid) {
    result.heading_degrees = circular_interpolate_degrees(
      before.heading_degrees, after.heading_degrees, fraction);
    result.pitch_degrees = before.pitch_degrees + fraction * (after.pitch_degrees - before.pitch_degrees);
  }
  return result;
}

}  // namespace ppbng_core
