#pragma once

#include "ppbng_hsi/hsi_types.hpp"

#include <cstddef>

namespace ppbng_hsi
{

class IHsiAdapter
{
public:
  virtual ~IHsiAdapter() = default;

  [[nodiscard]] virtual CameraKind kind() const noexcept = 0;
  [[nodiscard]] virtual HsiState state() const noexcept = 0;
  [[nodiscard]] virtual const HsiConfig & config() const noexcept = 0;
  [[nodiscard]] virtual std::uint32_t segment_id() const noexcept = 0;

  virtual OperationResult connect() = 0;
  virtual OperationResult configure(const HsiConfig & config) = 0;
  virtual OperationResult close_shutter() = 0;
  virtual OperationResult begin_dark_capture(std::size_t line_count) = 0;
  virtual OperationResult open_shutter() = 0;
  virtual OperationResult start_streaming() = 0;
  virtual OperationResult stop_streaming() = 0;
  virtual OperationResult recover() = 0;
  virtual LineResult on_trigger(const TriggerEvent & trigger) = 0;
  // Polls one SDK frame when Camera.Trigger.Mode is Internal. The default keeps
  // existing external-trigger adapters fail-closed.
  virtual LineResult poll_internal()
  {
    return {LineStatus::fault, std::nullopt, "internal acquisition is not supported"};
  }
};

}  // namespace ppbng_hsi
