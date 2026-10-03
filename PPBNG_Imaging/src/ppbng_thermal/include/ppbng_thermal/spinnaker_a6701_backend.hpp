#pragma once
#include "ppbng_thermal/thermal_adapter.hpp"

namespace ppbng_thermal
{
struct SpinnakerA6701Identity
{
  std::string device_id;
  std::string expected_model{"A6701"};
};

// Optional production backend. Merely constructing this object does not enumerate or open a
// device. Hardware activity occurs only through explicit lifecycle calls.
class SpinnakerA6701Backend final : public IThermalBackend
{
public:
  explicit SpinnakerA6701Backend(SpinnakerA6701Identity identity);
  ~SpinnakerA6701Backend() override;
  SpinnakerA6701Backend(const SpinnakerA6701Backend &) = delete;
  SpinnakerA6701Backend & operator=(const SpinnakerA6701Backend &) = delete;
  Status discover(std::chrono::milliseconds,const StopToken&,std::vector<std::string>&) override;
  Status open(const std::string&,std::chrono::milliseconds,const StopToken&) override;
  Status configure(const ThermalConfiguration&,std::chrono::milliseconds,const StopToken&) override;
  Status readback(std::chrono::milliseconds,const StopToken&,ThermalReadback&) override;
  Status calibration_snapshot(std::chrono::milliseconds,const StopToken&,
    ThermalCalibrationSnapshot&) override;
  Status arm(std::chrono::milliseconds,const StopToken&) override;
  FrameResult next_frame(std::chrono::milliseconds,const StopToken&) override;
  Status recover(std::chrono::milliseconds,const StopToken&) override;
  Status stop(std::chrono::milliseconds) noexcept override;
  LifecycleState state() const noexcept override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace ppbng_thermal
