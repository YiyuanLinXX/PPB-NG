#include "ppbng_runtime/storage_capacity_guard.hpp"

#include <limits>

namespace ppbng_runtime
{
namespace
{
constexpr std::uint64_t kBasisPoints = 10'000U;

bool checked_add(const std::uint64_t first, const std::uint64_t second,
  std::uint64_t & result) noexcept
{
  if (first > (std::numeric_limits<std::uint64_t>::max)() - second) return false;
  result = first + second;
  return true;
}

bool checked_multiply(const std::uint64_t first, const std::uint64_t second,
  std::uint64_t & result) noexcept
{
  if (first != 0U && second > (std::numeric_limits<std::uint64_t>::max)() / first) return false;
  result = first * second;
  return true;
}
}  // namespace

StorageCapacityDecision evaluate_storage_capacity(
  const StorageCapacityPolicy & policy, const std::uint64_t available_bytes,
  const std::uint64_t elapsed_seconds) noexcept
{
  StorageCapacityDecision result;
  if (policy.estimated_bytes_per_second == 0U || policy.planned_duration_seconds == 0U) {
    result.detail = "estimated stream rate and planned duration must be positive";
    return result;
  }
  if (policy.headroom_basis_points > 10'000U) {
    result.detail = "storage headroom must be between 0 and 10000 basis points";
    return result;
  }

  const auto remaining_seconds = elapsed_seconds >= policy.planned_duration_seconds ? 0U :
    policy.planned_duration_seconds - elapsed_seconds;
  std::uint64_t raw_bytes{};
  if (!checked_multiply(policy.estimated_bytes_per_second, remaining_seconds, raw_bytes)) {
    result.detail = "raw storage budget overflows uint64";
    return result;
  }

  const std::uint64_t factor = kBasisPoints + policy.headroom_basis_points;
  std::uint64_t quotient_component{};
  if (!checked_multiply(raw_bytes / kBasisPoints, factor, quotient_component)) {
    result.detail = "headroom-adjusted storage budget overflows uint64";
    return result;
  }
  const auto remainder_product = (raw_bytes % kBasisPoints) * factor;
  const auto remainder_component = remainder_product / kBasisPoints +
    (remainder_product % kBasisPoints == 0U ? 0U : 1U);
  if (!checked_add(quotient_component, remainder_component, result.remaining_stream_bytes) ||
    !checked_add(result.remaining_stream_bytes, policy.minimum_reserve_bytes,
    result.required_available_bytes))
  {
    result.detail = "storage budget plus reserve overflows uint64";
    return result;
  }

  result.valid = true;
  result.sufficient = available_bytes >= result.required_available_bytes;
  result.detail = result.sufficient ? "storage capacity satisfies remaining task budget" :
    "insufficient free space for remaining task budget and reserve";
  return result;
}

}  // namespace ppbng_runtime
