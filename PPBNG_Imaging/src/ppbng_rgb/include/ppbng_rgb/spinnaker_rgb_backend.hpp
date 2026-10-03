#pragma once

#include "ppbng_rgb/rgb_adapter.hpp"

#include <memory>

namespace ppbng_rgb
{
// This backend performs no SDK or hardware work until discover() is explicitly called.
class SpinnakerRgbBackend final : public IRgbBackend
{
public:
  SpinnakerRgbBackend();
  ~SpinnakerRgbBackend() override;
  SpinnakerRgbBackend(const SpinnakerRgbBackend &) = delete;
  SpinnakerRgbBackend & operator=(const SpinnakerRgbBackend &) = delete;

  Status discover(std::chrono::milliseconds, const StopToken &, std::vector<std::string> &) override;
  Status open(const std::string &, std::chrono::milliseconds, const StopToken &) override;
  Status configure(const RgbConfiguration &, std::chrono::milliseconds, const StopToken &) override;
  Status readback(std::chrono::milliseconds, const StopToken &, RgbReadback &) override;
  Status arm(std::chrono::milliseconds, const StopToken &) override;
  FrameResult next_frame(std::chrono::milliseconds, const StopToken &) override;
  Status recover(std::chrono::milliseconds, const StopToken &) override;
  Status stop(std::chrono::milliseconds) noexcept override;
  LifecycleState state() const noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ppbng_rgb
