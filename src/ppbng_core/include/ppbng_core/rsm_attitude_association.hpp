#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ppbng_core {

struct RsmAttitudeSample {
  std::uint64_t sample_id{};
  std::int64_t utc_nanoseconds{};
  double roll_deg{};
  double pitch_deg{};
  double yaw_deg{};
  bool sample_status_valid{};
  bool roll_valid{};
  bool pitch_valid{};
  bool yaw_valid{};
  std::uint32_t raw_status{};
};

struct AssociatedRsmAttitude {
  std::int64_t target_utc_nanoseconds{};
  double roll_deg{};
  double pitch_deg{};
  std::optional<double> yaw_deg;
  // RSM400 has no enabled yaw stabilization axis. Even when a protocol field
  // is present, it is not a substitute for GNSS platform heading.
  bool yaw_is_platform_heading{false};
  std::uint64_t before_sample_id{};
  std::uint64_t after_sample_id{};
  std::uint64_t before_age_nanoseconds{};
  std::uint64_t after_age_nanoseconds{};
  std::uint32_t before_raw_status{};
  std::uint32_t after_raw_status{};
};

class RsmAttitudeAssociator {
 public:
  explicit RsmAttitudeAssociator(
      std::uint64_t maximum_bracket_age_nanoseconds,
      std::size_t maximum_samples = 256U);

  // Samples must arrive with strictly increasing UTC time and sample ID.
  // Reordered or duplicate input is rejected without modifying the history.
  bool add_sample(const RsmAttitudeSample& sample);

  // Returns only a bracketed interpolation. It never extrapolates and never
  // skips across an invalid adjacent sample to create a wider bracket.
  std::optional<AssociatedRsmAttitude> associate(
      std::int64_t target_utc_nanoseconds) const;

  std::size_t sample_count() const noexcept;

 private:
  std::uint64_t maximum_bracket_age_nanoseconds_;
  std::size_t maximum_samples_;
  std::vector<RsmAttitudeSample> samples_;
};

}  // namespace ppbng_core
