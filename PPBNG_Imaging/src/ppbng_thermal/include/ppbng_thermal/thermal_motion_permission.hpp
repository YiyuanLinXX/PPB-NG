#pragma once

#include <cstdint>

namespace ppbng_thermal
{

// Pure, steady-clock-based safety policy used by the ROS relay and unit tests.
// Permission is never inferred: it requires a recent, valid A6701 NUC state.
class ThermalMotionPermission
{
public:
  void observe(bool valid, bool active, std::uint64_t receipt_monotonic_ns) noexcept
  {
    observed_ = true;
    valid_ = valid;
    active_ = active;
    receipt_monotonic_ns_ = receipt_monotonic_ns;
  }

  bool permitted(std::uint64_t now_monotonic_ns, std::uint64_t timeout_ns) const noexcept
  {
    if (!observed_ || !valid_ || active_ || now_monotonic_ns < receipt_monotonic_ns_) {
      return false;
    }
    return now_monotonic_ns - receipt_monotonic_ns_ <= timeout_ns;
  }

private:
  bool observed_{false};
  bool valid_{false};
  bool active_{false};
  std::uint64_t receipt_monotonic_ns_{0U};
};

}  // namespace ppbng_thermal
