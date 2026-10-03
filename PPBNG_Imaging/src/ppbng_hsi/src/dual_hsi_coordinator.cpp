#include "ppbng_hsi/dual_hsi_coordinator.hpp"

#include <stdexcept>

namespace ppbng_hsi
{

DualHsiCoordinator::DualHsiCoordinator(IHsiAdapter & fx10e, IHsiAdapter & swir)
: fx10e_(fx10e), swir_(swir)
{
  if (fx10e_.kind() != CameraKind::fx10e || swir_.kind() != CameraKind::swir) {
    throw std::invalid_argument("DualHsiCoordinator requires FX10e then SWIR adapters");
  }
}

DualOperationResult DualHsiCoordinator::connect_both()
{
  return {fx10e_.connect(), swir_.connect()};
}

DualOperationResult DualHsiCoordinator::configure_both(
  const HsiConfig & fx10e, const HsiConfig & swir)
{
  return {fx10e_.configure(fx10e), swir_.configure(swir)};
}

DualOperationResult DualHsiCoordinator::close_both_shutters()
{
  return {fx10e_.close_shutter(), swir_.close_shutter()};
}

DualOperationResult DualHsiCoordinator::begin_dark_both(
  const std::size_t fx10e_lines, const std::size_t swir_lines)
{
  return {fx10e_.begin_dark_capture(fx10e_lines), swir_.begin_dark_capture(swir_lines)};
}

DualOperationResult DualHsiCoordinator::open_both_shutters()
{
  return {fx10e_.open_shutter(), swir_.open_shutter()};
}

DualOperationResult DualHsiCoordinator::start_both()
{
  return {fx10e_.start_streaming(), swir_.start_streaming()};
}

DualOperationResult DualHsiCoordinator::stop_both()
{
  return {fx10e_.stop_streaming(), swir_.stop_streaming()};
}

LineResult DualHsiCoordinator::dispatch(const CameraKind camera, const TriggerEvent & trigger)
{
  return camera == CameraKind::fx10e ? fx10e_.on_trigger(trigger) : swir_.on_trigger(trigger);
}

}  // namespace ppbng_hsi

