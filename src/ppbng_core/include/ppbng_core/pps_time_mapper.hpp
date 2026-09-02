#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace ppbng_core
{

enum class TimeStatus : std::uint8_t
{
  unsynced = 0,
  holdover = 1,
  locked = 2,
};

struct MappedTime
{
  std::int64_t utc_nanoseconds{};
  std::uint64_t uncertainty_nanoseconds{};
  TimeStatus status{TimeStatus::unsynced};
  std::string detail;
};

class PpsTimeMapper
{
public:
  explicit PpsTimeMapper(std::size_t maximum_anchors = 8);

  bool add_anchor(std::uint64_t pps_sequence, std::int64_t utc_nanoseconds);

  [[nodiscard]] std::optional<MappedTime> map_event(
    std::uint64_t pps_sequence,
    std::uint64_t offset_ticks,
    std::uint64_t ticks_per_second,
    bool controller_locked,
    std::uint64_t controller_uncertainty_nanoseconds = 0,
    std::uint64_t maximum_holdover_sequences = 0,
    std::uint64_t oscillator_drift_ppb = 0) const;

  [[nodiscard]] std::size_t anchor_count() const noexcept;

private:
  std::size_t maximum_anchors_;
  std::unordered_map<std::uint64_t, std::int64_t> anchors_;
};

}  // namespace ppbng_core
