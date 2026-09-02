#pragma once

#include <cstdint>
#include <string>

namespace ppbng_runtime
{

struct StorageCapacityPolicy
{
  std::uint64_t estimated_bytes_per_second{};
  std::uint64_t planned_duration_seconds{};
  std::uint64_t minimum_reserve_bytes{};
  // 1500 means 15% beyond the calculated raw stream volume.
  std::uint32_t headroom_basis_points{1500U};
};

struct StorageCapacityDecision
{
  bool valid{false};
  bool sufficient{false};
  std::uint64_t required_available_bytes{};
  std::uint64_t remaining_stream_bytes{};
  std::string detail;
};

// Evaluates the space still needed to finish the planned task. All arithmetic is
// checked; an overflow is an invalid policy rather than a wrapped, permissive result.
StorageCapacityDecision evaluate_storage_capacity(
  const StorageCapacityPolicy & policy, std::uint64_t available_bytes,
  std::uint64_t elapsed_seconds) noexcept;

}  // namespace ppbng_runtime
