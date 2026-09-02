#include "ppbng_thermal/spinnaker_a6701_backend.hpp"

namespace ppbng_thermal
{
namespace
{
Status unavailable()
{
  return {ErrorCode::invalid_state,
    "SDK_NOT_BUILT: ppbng_thermal was built with PPBNG_THERMAL_ENABLE_SPINNAKER=OFF"};
}
}  // namespace

struct SpinnakerA6701Backend::Impl {};

SpinnakerA6701Backend::SpinnakerA6701Backend(SpinnakerA6701Identity) :
  impl_(std::make_unique<Impl>()) {}
SpinnakerA6701Backend::~SpinnakerA6701Backend() = default;
Status SpinnakerA6701Backend::discover(
  std::chrono::milliseconds, const StopToken &, std::vector<std::string> & ids)
{ids.clear(); return unavailable();}
Status SpinnakerA6701Backend::open(
  const std::string &, std::chrono::milliseconds, const StopToken &) {return unavailable();}
Status SpinnakerA6701Backend::configure(
  const ThermalConfiguration &, std::chrono::milliseconds, const StopToken &) {return unavailable();}
Status SpinnakerA6701Backend::readback(
  std::chrono::milliseconds, const StopToken &, ThermalReadback &) {return unavailable();}
Status SpinnakerA6701Backend::arm(
  std::chrono::milliseconds, const StopToken &) {return unavailable();}
FrameResult SpinnakerA6701Backend::next_frame(
  std::chrono::milliseconds, const StopToken &) {return {unavailable(), {}};}
Status SpinnakerA6701Backend::recover(
  std::chrono::milliseconds, const StopToken &) {return unavailable();}
Status SpinnakerA6701Backend::stop(std::chrono::milliseconds) noexcept {return {};}
LifecycleState SpinnakerA6701Backend::state() const noexcept {return LifecycleState::idle;}
}  // namespace ppbng_thermal
