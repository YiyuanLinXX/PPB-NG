#pragma once
#include "ppbng_thermal/thermal_adapter.hpp"

namespace ppbng_thermal
{
struct FakeThermalNodeMap
{
  bool discoverable{true};
  bool ready{true};
  bool fpa_cold{true};
  AccessMode access{AccessMode::control};
  bool force_readback_mismatch{false};
  std::uint64_t incomplete_every_n_frames{0};
  bool nuc_status_valid{true};
  bool nuc_active{false};
  bool correction_auto_in_progress{false};
  std::string correction_status{"Ready"};
  std::string correction_status_text{"Ready"};
  std::string flag_state{"Stowed"};
  ThermalCalibrationSnapshot calibration{
    {
      {"CameraModel", "A6701", true},
      {"ActivePreset", "0", true},
      {"PS0CalibrationTag", "17mm_factory", true},
      {"CalibrationIsFactoryCalibrated", "true", true},
      {"CalibrationQueryIndex", "0", true},
      {"CalibrationQueryIndexMax", "0", true},
      {"CalibrationQueryLens", "17mm", true},
      {"CalibrationQueryLensFilter", "factory", true},
      {"CalibrationQueryName", "17mm_factory", true},
      {"CalibrationQueryTag", "17mm_factory", true},
      {"CalibrationQueryMinCounts", "1", true},
      {"CalibrationQueryMaxCounts", "65535", true},
      {"CalibrationQueryMinTemp", "-20", true},
      {"CalibrationQueryMaxTemp", "350", true},
      {"CalibrationQueryOrder", "2", true},
      {"CalibrationQueryCoeff0", "1", true},
      {"CalibrationQueryCoeff1", "1", true},
      {"CalibrationQueryCoeff2", "1", true},
      {"CalibrationQueryR", "1", true},
      {"CalibrationQueryB", "1", true},
      {"CalibrationQueryF", "1", true},
      {"ObjectEmissivity", "0.95", true},
      {"ReflectedTemperature", "293.15", true},
      {"AtmosphericTemperature", "293.15", true},
      {"ObjectDistance", "1", true},
      {"RelativeHumidity", "0.5", true},
      {"EstimatedTransmission", "1", true},
      {"ExtOpticsTemperature", "293.15", true},
      {"ExtOpticsTransmission", "1", true},
    }};
  ThermalConfiguration values{};
};

class MockThermalBackend final : public IThermalBackend
{
public:
  explicit MockThermalBackend(FakeThermalNodeMap nodes = {});
  Status discover(std::chrono::milliseconds, const StopToken &, std::vector<std::string> &) override;
  Status open(const std::string &, std::chrono::milliseconds, const StopToken &) override;
  Status configure(const ThermalConfiguration &, std::chrono::milliseconds, const StopToken &) override;
  Status readback(std::chrono::milliseconds, const StopToken &, ThermalReadback &) override;
  Status calibration_snapshot(std::chrono::milliseconds, const StopToken &,
    ThermalCalibrationSnapshot &) override;
  Status arm(std::chrono::milliseconds, const StopToken &) override;
  FrameResult next_frame(std::chrono::milliseconds, const StopToken &) override;
  Status recover(std::chrono::milliseconds, const StopToken &) override;
  Status stop(std::chrono::milliseconds) noexcept override;
  LifecycleState state() const noexcept override {return state_;}
  std::uint64_t release_count() const noexcept {return release_count_->load();}
  std::uint64_t segment_index() const noexcept {return segment_index_;}
  FakeThermalNodeMap & nodes() noexcept {return nodes_;}
private:
  Status preflight(std::chrono::milliseconds, const StopToken &) const;
  FakeThermalNodeMap nodes_;
  LifecycleState state_{LifecycleState::idle};
  ThermalConfiguration requested_{};
  std::uint64_t frame_id_{0};
  std::uint64_t segment_index_{0};
  std::shared_ptr<std::atomic_uint64_t> release_count_;
};
}  // namespace ppbng_thermal
