#include "ppbng_rgb/spinnaker_rgb_backend.hpp"

namespace ppbng_rgb
{
namespace
{
Status unavailable()
{
  return {ErrorCode::invalid_state,
    "SDK_NOT_BUILT: ppbng_rgb was built with PPBNG_RGB_ENABLE_SPINNAKER=OFF"};
}
}  // namespace

class SpinnakerRgbBackend::Impl {};

SpinnakerRgbBackend::SpinnakerRgbBackend() : impl_(std::make_unique<Impl>()) {}
SpinnakerRgbBackend::~SpinnakerRgbBackend() = default;
Status SpinnakerRgbBackend::discover(
  std::chrono::milliseconds, const StopToken &, std::vector<std::string> & ids)
{ids.clear(); return unavailable();}
Status SpinnakerRgbBackend::open(
  const std::string &, std::chrono::milliseconds, const StopToken &) {return unavailable();}
Status SpinnakerRgbBackend::configure(
  const RgbConfiguration &, std::chrono::milliseconds, const StopToken &) {return unavailable();}
Status SpinnakerRgbBackend::readback(
  std::chrono::milliseconds, const StopToken &, RgbReadback &) {return unavailable();}
Status SpinnakerRgbBackend::arm(
  std::chrono::milliseconds, const StopToken &) {return unavailable();}
FrameResult SpinnakerRgbBackend::next_frame(
  std::chrono::milliseconds, const StopToken &) {return {unavailable(), {}};}
Status SpinnakerRgbBackend::recover(
  std::chrono::milliseconds, const StopToken &) {return unavailable();}
Status SpinnakerRgbBackend::stop(std::chrono::milliseconds) noexcept {return {};}
LifecycleState SpinnakerRgbBackend::state() const noexcept {return LifecycleState::idle;}
}  // namespace ppbng_rgb
