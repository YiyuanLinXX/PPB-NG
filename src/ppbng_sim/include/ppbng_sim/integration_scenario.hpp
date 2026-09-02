#pragma once

#include "ppbng_core/pps_time_mapper.hpp"
#include "ppbng_hsi/dual_hsi_coordinator.hpp"
#include "ppbng_hsi/mock_hsi_adapter.hpp"
#include "ppbng_orchestrator/acquisition_state_machine.hpp"
#include "ppbng_rgb/mock_rgb_backend.hpp"
#include "ppbng_storage/dataset_session.hpp"
#include "ppbng_thermal/mock_thermal_backend.hpp"
#include "ppbng_timing/controller_model.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ppbng_sim
{

enum class SimDevice : std::uint8_t {fx10e = 0, swir = 1, rgb = 2, thermal = 3};

struct TriggerObservation
{
  std::uint64_t event_id{0};
  SimDevice device{SimDevice::fx10e};
  std::uint64_t channel_sequence{0};
  std::uint64_t pps_sequence{0};
  std::int64_t utc_time_ns{0};
  std::uint32_t segment_id{0};
  std::uint64_t sample_index{0};
  bool produced{false};
  bool rtk_fixed{true};
};

class IntegrationScenario
{
public:
  IntegrationScenario();

  bool start_and_preflight();
  bool collect_dark_and_wait_for_pps();
  bool start_on_next_pps();
  void run_one_second();
  bool normal_stop_and_finalize();

  void set_thermal_disconnected(bool disconnected) noexcept;
  bool recover_thermal();
  void disconnect_hsi_on_next_line(ppbng_hsi::CameraKind camera);
  bool recover_hsi(ppbng_hsi::CameraKind camera);
  void set_rtk_fixed(bool fixed);

  void report_disk_fault(const std::string & detail);
  void report_trigger_fault(const std::string & detail);
  bool controlled_stop_after_fault();

  [[nodiscard]] const std::vector<TriggerObservation> & observations() const noexcept;
  [[nodiscard]] std::size_t produced_count(SimDevice device) const noexcept;
  [[nodiscard]] std::size_t trigger_count(SimDevice device) const noexcept;
  [[nodiscard]] const ppbng_storage::SegmentIndex & segment_index(SimDevice device) const;
  [[nodiscard]] const ppbng_orchestrator::AcquisitionStateMachine & task() const noexcept;
  [[nodiscard]] const ppbng_hsi::MockHsiAdapter & fx10e() const noexcept;
  [[nodiscard]] const ppbng_hsi::MockHsiAdapter & swir() const noexcept;
  [[nodiscard]] const ppbng_thermal::MockThermalBackend & thermal() const noexcept;
  [[nodiscard]] const ppbng_rgb::MockRgbBackend & rgb() const noexcept;
  [[nodiscard]] ppbng_timing::ControllerState timing_state() const;
  [[nodiscard]] bool indices_finalized() const noexcept;

private:
  struct SegmentAccumulator
  {
    bool used{false};
    std::uint64_t first_event_id{0};
    std::uint64_t last_event_id{0};
    std::uint64_t sample_count{0};
  };

  static constexpr std::uint32_t ticks_per_second = 1200;
  static constexpr std::int64_t initial_utc_ns = 1'700'000'000'000'000'000LL;

  bool configure_rgb();
  bool configure_thermal();
  ppbng_timing::TriggerEvent emit(ppbng_timing::Channel channel, std::uint64_t sequence);
  ppbng_hsi::TriggerEvent as_hsi_trigger(const ppbng_timing::TriggerEvent & event) const;
  TriggerObservation & add_observation(
    const ppbng_timing::TriggerEvent & event, SimDevice device);
  void emit_hsi(ppbng_hsi::CameraKind camera, std::uint64_t sequence);
  void emit_rgb(std::uint64_t sequence);
  void emit_thermal(std::uint64_t sequence);
  bool stop_devices_and_finalize();
  bool finalize_indices();
  void accumulate(const TriggerObservation & observation);
  static std::size_t device_index(SimDevice device) noexcept;
  static std::string device_stem(SimDevice device);

  ppbng_orchestrator::AcquisitionStateMachine task_;
  ppbng_timing::ControllerModel timing_;
  ppbng_core::PpsTimeMapper time_mapper_;
  ppbng_hsi::MockHsiAdapter fx10e_;
  ppbng_hsi::MockHsiAdapter swir_;
  ppbng_hsi::DualHsiCoordinator hsi_;
  ppbng_rgb::MockRgbBackend rgb_;
  ppbng_thermal::MockThermalBackend thermal_;
  ppbng_rgb::StopSource rgb_stop_;
  ppbng_thermal::StopSource thermal_stop_;

  std::vector<TriggerObservation> observations_;
  std::array<std::vector<SegmentAccumulator>, 4> accumulators_;
  std::array<ppbng_storage::SegmentIndex, 4> segment_indices_;
  bool indices_finalized_{false};
  bool thermal_disconnected_{false};
  bool rtk_fixed_{true};
  std::uint64_t current_pps_sequence_{101};
  std::uint64_t current_pps_tick_{ticks_per_second};
  std::int64_t current_utc_ns_{initial_utc_ns};
  std::uint64_t fx_sequence_{3};
  std::uint64_t swir_sequence_{3};
  std::uint64_t rgb_sequence_{1};
  std::uint64_t thermal_sequence_{1};
};

}  // namespace ppbng_sim

