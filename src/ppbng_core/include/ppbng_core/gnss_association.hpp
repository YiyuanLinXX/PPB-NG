#pragma once

#include <cstdint>
#include <optional>

namespace ppbng_core
{

struct GnssObservation
{
  std::uint64_t sequence{};
  std::int64_t utc_nanoseconds{};
  std::uint64_t host_monotonic_nanoseconds{};
  double latitude_degrees{};
  double longitude_degrees{};
  double altitude_meters{};
  double heading_degrees{};
  double pitch_degrees{};
  std::uint8_t fix_quality{};
  bool position_valid{};
  bool heading_valid{};
  std::uint64_t utc_anchor_sequence{};
  std::uint64_t utc_anchor_host_delta_nanoseconds{};
  std::uint64_t utc_anchor_clock_residual_nanoseconds{};
};

struct AssociatedGnss
{
  std::int64_t utc_nanoseconds{};
  std::uint64_t before_sequence{};
  std::uint64_t after_sequence{};
  std::uint64_t before_age_nanoseconds{};
  std::uint64_t after_age_nanoseconds{};
  double interpolation_fraction{};
  double latitude_degrees{};
  double longitude_degrees{};
  double altitude_meters{};
  double heading_degrees{};
  double pitch_degrees{};
  std::uint8_t before_fix_quality{};
  std::uint8_t after_fix_quality{};
  bool heading_valid{};
  std::uint64_t before_utc_anchor_sequence{};
  std::uint64_t after_utc_anchor_sequence{};
  std::uint64_t before_utc_anchor_host_delta_nanoseconds{};
  std::uint64_t after_utc_anchor_host_delta_nanoseconds{};
  std::uint64_t before_utc_anchor_clock_residual_nanoseconds{};
  std::uint64_t after_utc_anchor_clock_residual_nanoseconds{};
};

[[nodiscard]] std::optional<AssociatedGnss> associate_gnss(
  const GnssObservation & before,
  const GnssObservation & after,
  std::int64_t target_utc_nanoseconds,
  std::uint64_t max_bracket_span_nanoseconds,
  std::uint64_t max_endpoint_age_nanoseconds);

}  // namespace ppbng_core
