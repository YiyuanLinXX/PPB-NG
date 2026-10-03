#pragma once

#include "ppbng_hsi/hsi_adapter.hpp"

#include <cstddef>

namespace ppbng_hsi
{

struct DualOperationResult
{
  OperationResult fx10e;
  OperationResult swir;
  [[nodiscard]] bool both_succeeded() const noexcept
  {
    return fx10e.success && swir.success;
  }
};

class DualHsiCoordinator
{
public:
  DualHsiCoordinator(IHsiAdapter & fx10e, IHsiAdapter & swir);

  DualOperationResult connect_both();
  DualOperationResult configure_both(const HsiConfig & fx10e, const HsiConfig & swir);
  DualOperationResult close_both_shutters();
  DualOperationResult begin_dark_both(std::size_t fx10e_lines, std::size_t swir_lines);
  DualOperationResult open_both_shutters();
  DualOperationResult start_both();
  DualOperationResult stop_both();
  LineResult dispatch(CameraKind camera, const TriggerEvent & trigger);

private:
  IHsiAdapter & fx10e_;
  IHsiAdapter & swir_;
};

}  // namespace ppbng_hsi

