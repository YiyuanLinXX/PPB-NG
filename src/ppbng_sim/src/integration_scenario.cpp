#include "ppbng_sim/integration_scenario.hpp"

#include "ppbng_storage/thermal_payload_layout.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace ppbng_sim
{

namespace
{
constexpr std::chrono::milliseconds mock_timeout{10};

ppbng_hsi::HsiConfig fx_config()
{
  return {ppbng_hsi::CameraKind::fx10e, "sim-fx10e", "fx10e_line", 8, 4, 120.0, 2500.0};
}

ppbng_hsi::HsiConfig swir_config()
{
  return {ppbng_hsi::CameraKind::swir, "sim-swir", "swir_line", 6, 3, 80.0, 4000.0};
}

std::string hsi_channel(const ppbng_hsi::CameraKind camera)
{
  return camera == ppbng_hsi::CameraKind::fx10e ? "fx10e_line" : "swir_line";
}

ppbng_hsi::TimeStatus hsi_time_status(const ppbng_timing::TimeLock lock)
{
  switch (lock) {
    case ppbng_timing::TimeLock::kLocked:
      return ppbng_hsi::TimeStatus::locked;
    case ppbng_timing::TimeLock::kHoldover:
      return ppbng_hsi::TimeStatus::holdover;
    case ppbng_timing::TimeLock::kUnsynced:
    default:
      return ppbng_hsi::TimeStatus::unsynced;
  }
}
}  // namespace

IntegrationScenario::IntegrationScenario()
: task_([]() {return "sim-session";}),
  timing_(1, ticks_per_second, 5),
  time_mapper_(8),
  fx10e_(ppbng_hsi::CameraKind::fx10e),
  swir_(ppbng_hsi::CameraKind::swir),
  hsi_(fx10e_, swir_)
{}

bool IntegrationScenario::start_and_preflight()
{
  if (!task_.start("sim-start", "sim-dataset", false).accepted) {
    return false;
  }

  ppbng_timing::ScheduleConfig schedule;
  schedule.schedule_id = 7;
  schedule.ticks_per_second = ticks_per_second;
  schedule.channels = {
    {ppbng_timing::Channel::kFx10e, true, 120, 1, 1, 0},
    {ppbng_timing::Channel::kSwir, true, 80, 1, 1, 0},
    {ppbng_timing::Channel::kRgb, true, 2, 1, 1, 0},
    {ppbng_timing::Channel::kThermal, true, 2, 1, 1, 0}};
  try {
    timing_.configure(schedule);
    timing_.freeze(schedule.schedule_id);
  } catch (...) {
    return false;
  }

  if (!hsi_.connect_both().both_succeeded() ||
    !hsi_.configure_both(fx_config(), swir_config()).both_succeeded() ||
    !configure_rgb() || !configure_thermal())
  {
    return false;
  }
  return task_.complete_preflight(true, false, "all simulated devices ready").accepted;
}

bool IntegrationScenario::collect_dark_and_wait_for_pps()
{
  if (!task_.confirm_dark_ready("sim-dark-ready").accepted ||
    !hsi_.close_both_shutters().both_succeeded() ||
    !hsi_.begin_dark_both(2, 2).both_succeeded())
  {
    return false;
  }

  for (std::uint64_t sequence = 1; sequence <= 2; ++sequence) {
    ppbng_hsi::TriggerEvent fx_dark{
      "fx10e_line", sequence, 100, 0, ticks_per_second,
      initial_utc_ns - 1'000'000'000LL, ppbng_hsi::TimeStatus::locked, 100};
    ppbng_hsi::TriggerEvent swir_dark = fx_dark;
    swir_dark.channel = "swir_line";
    if (hsi_.dispatch(ppbng_hsi::CameraKind::fx10e, fx_dark).status !=
      ppbng_hsi::LineStatus::produced ||
      hsi_.dispatch(ppbng_hsi::CameraKind::swir, swir_dark).status !=
      ppbng_hsi::LineStatus::produced)
    {
      return false;
    }
  }

  if (!task_.complete_dark_capture(true, "dark references collected").accepted ||
    !hsi_.open_both_shutters().both_succeeded())
  {
    return false;
  }
  return task_.confirm_sample_ready("sim-sample-ready").accepted;
}

bool IntegrationScenario::start_on_next_pps()
{
  try {
    timing_.arm_next_whole_second({7, 100});
    timing_.observe_pps(current_pps_sequence_, current_utc_ns_ / 1'000'000'000LL,
      current_pps_tick_);
  } catch (...) {
    return false;
  }
  if (!time_mapper_.add_anchor(current_pps_sequence_, current_utc_ns_) ||
    !hsi_.start_both().both_succeeded())
  {
    return false;
  }
  return task_.reach_start_pps().accepted;
}

void IntegrationScenario::run_one_second()
{
  if (task_.state() != ppbng_orchestrator::AcquisitionState::recording ||
    timing_.status().state != ppbng_timing::ControllerState::kRunning)
  {
    throw std::logic_error("scenario must be recording before triggers run");
  }

  for (std::uint64_t offset = 0; offset < ticks_per_second; ++offset) {
    timing_.advance_to(current_pps_tick_ + offset);
    if (offset % 10U == 0U) {
      emit_hsi(ppbng_hsi::CameraKind::fx10e, fx_sequence_++);
    }
    if (offset % 15U == 0U) {
      emit_hsi(ppbng_hsi::CameraKind::swir, swir_sequence_++);
    }
    if (offset % 600U == 0U) {
      emit_rgb(rgb_sequence_++);
      emit_thermal(thermal_sequence_++);
    }
  }

  ++current_pps_sequence_;
  current_pps_tick_ += ticks_per_second;
  current_utc_ns_ += 1'000'000'000LL;
  timing_.observe_pps(
    current_pps_sequence_, current_utc_ns_ / 1'000'000'000LL, current_pps_tick_);
  if (!time_mapper_.add_anchor(current_pps_sequence_, current_utc_ns_)) {
    throw std::logic_error("simulated PPS anchor conflict");
  }
}

bool IntegrationScenario::normal_stop_and_finalize()
{
  if (!task_.stop("sim-stop", "normal simulated stop").accepted) {
    return false;
  }
  return stop_devices_and_finalize() &&
    task_.complete_stop(true, "simulated files finalized").accepted;
}

void IntegrationScenario::set_thermal_disconnected(const bool disconnected) noexcept
{
  thermal_disconnected_ = disconnected;
}

bool IntegrationScenario::recover_thermal()
{
  if (!thermal_disconnected_) {
    return false;
  }
  auto status = thermal_.recover(mock_timeout, thermal_stop_.token());
  if (!status.ok()) {
    return false;
  }
  ppbng_thermal::ThermalConfiguration configuration;
  status = thermal_.configure(configuration, mock_timeout, thermal_stop_.token());
  if (!status.ok()) {
    return false;
  }
  status = thermal_.arm(mock_timeout, thermal_stop_.token());
  if (!status.ok()) {
    return false;
  }
  thermal_disconnected_ = false;
  return true;
}

void IntegrationScenario::disconnect_hsi_on_next_line(const ppbng_hsi::CameraKind camera)
{
  (camera == ppbng_hsi::CameraKind::fx10e ? fx10e_ : swir_).disconnect_on_next_line();
}

bool IntegrationScenario::recover_hsi(const ppbng_hsi::CameraKind camera)
{
  auto & adapter = camera == ppbng_hsi::CameraKind::fx10e ? fx10e_ : swir_;
  return adapter.recover().success && adapter.start_streaming().success;
}

void IntegrationScenario::set_rtk_fixed(const bool fixed)
{
  rtk_fixed_ = fixed;
  if (fixed) {
    task_.clear_degraded("RTK fixed recovered");
  } else {
    task_.mark_degraded("RTK fix degraded; acquisition continues");
  }
}

void IntegrationScenario::report_disk_fault(const std::string & detail)
{
  task_.report_global_fault(ppbng_orchestrator::GlobalFault::disk_write_failure, detail);
}

void IntegrationScenario::report_trigger_fault(const std::string & detail)
{
  task_.report_global_fault(
    ppbng_orchestrator::GlobalFault::trigger_controller_failure, detail);
}

bool IntegrationScenario::controlled_stop_after_fault()
{
  if (!task_.begin_fault_stop().accepted) {
    return false;
  }
  return stop_devices_and_finalize() &&
    task_.complete_stop(true, "best-effort simulated finalization").accepted;
}

const std::vector<TriggerObservation> & IntegrationScenario::observations() const noexcept
{
  return observations_;
}

std::size_t IntegrationScenario::produced_count(const SimDevice device) const noexcept
{
  return static_cast<std::size_t>(std::count_if(
    observations_.begin(), observations_.end(), [device](const auto & observation) {
      return observation.device == device && observation.produced;
    }));
}

std::size_t IntegrationScenario::trigger_count(const SimDevice device) const noexcept
{
  return static_cast<std::size_t>(std::count_if(
    observations_.begin(), observations_.end(), [device](const auto & observation) {
      return observation.device == device;
    }));
}

const ppbng_storage::SegmentIndex & IntegrationScenario::segment_index(
  const SimDevice device) const
{
  return segment_indices_.at(device_index(device));
}

const ppbng_orchestrator::AcquisitionStateMachine & IntegrationScenario::task() const noexcept
{
  return task_;
}

const ppbng_hsi::MockHsiAdapter & IntegrationScenario::fx10e() const noexcept {return fx10e_;}
const ppbng_hsi::MockHsiAdapter & IntegrationScenario::swir() const noexcept {return swir_;}
const ppbng_thermal::MockThermalBackend & IntegrationScenario::thermal() const noexcept
{
  return thermal_;
}
const ppbng_rgb::MockRgbBackend & IntegrationScenario::rgb() const noexcept {return rgb_;}
ppbng_timing::ControllerState IntegrationScenario::timing_state() const
{
  return timing_.status().state;
}
bool IntegrationScenario::indices_finalized() const noexcept {return indices_finalized_;}

bool IntegrationScenario::configure_rgb()
{
  std::vector<std::string> ids;
  auto status = rgb_.discover(mock_timeout, rgb_stop_.token(), ids);
  if (!status.ok() || ids.size() != 1) {
    return false;
  }
  status = rgb_.open(ids.front(), mock_timeout, rgb_stop_.token());
  if (!status.ok()) {
    return false;
  }
  ppbng_rgb::RgbConfiguration configuration;
  configuration.width = 8;
  configuration.height = 4;
  configuration.row_stride_bytes = 8;
  configuration.payload_bytes = 32;
  status = rgb_.configure(configuration, mock_timeout, rgb_stop_.token());
  return status.ok() && rgb_.arm(mock_timeout, rgb_stop_.token()).ok();
}

bool IntegrationScenario::configure_thermal()
{
  std::vector<std::string> ids;
  auto status = thermal_.discover(mock_timeout, thermal_stop_.token(), ids);
  if (!status.ok() || ids.size() != 1) {
    return false;
  }
  status = thermal_.open(ids.front(), mock_timeout, thermal_stop_.token());
  if (!status.ok()) {
    return false;
  }
  ppbng_thermal::ThermalConfiguration configuration;
  status = thermal_.configure(configuration, mock_timeout, thermal_stop_.token());
  return status.ok() && thermal_.arm(mock_timeout, thermal_stop_.token()).ok();
}

ppbng_timing::TriggerEvent IntegrationScenario::emit(
  const ppbng_timing::Channel channel, const std::uint64_t sequence)
{
  return timing_.emit_trigger(channel, sequence);
}

ppbng_hsi::TriggerEvent IntegrationScenario::as_hsi_trigger(
  const ppbng_timing::TriggerEvent & event) const
{
  const auto mapped = time_mapper_.map_event(
    event.pps_sequence, event.offset_ticks, event.ticks_per_second,
    event.lock == ppbng_timing::TimeLock::kLocked, 100, 5, 50);
  if (!mapped.has_value()) {
    throw std::logic_error("trigger has no simulated UTC mapping");
  }
  return {
    event.channel == ppbng_timing::Channel::kFx10e ? "fx10e_line" : "swir_line",
    event.channel_sequence, event.pps_sequence, event.offset_ticks, event.ticks_per_second,
    mapped->utc_nanoseconds, hsi_time_status(event.lock), mapped->uncertainty_nanoseconds};
}

TriggerObservation & IntegrationScenario::add_observation(
  const ppbng_timing::TriggerEvent & event, const SimDevice device)
{
  const auto mapped = time_mapper_.map_event(
    event.pps_sequence, event.offset_ticks, event.ticks_per_second,
    event.lock == ppbng_timing::TimeLock::kLocked, 100, 5, 50);
  observations_.push_back({
    event.event_sequence, device, event.channel_sequence, event.pps_sequence,
    mapped.has_value() ? mapped->utc_nanoseconds : 0, 0, 0, false, rtk_fixed_});
  return observations_.back();
}

void IntegrationScenario::emit_hsi(
  const ppbng_hsi::CameraKind camera, const std::uint64_t sequence)
{
  const auto channel = camera == ppbng_hsi::CameraKind::fx10e ?
    ppbng_timing::Channel::kFx10e : ppbng_timing::Channel::kSwir;
  const auto event = emit(channel, sequence);
  auto & observation = add_observation(
    event, camera == ppbng_hsi::CameraKind::fx10e ? SimDevice::fx10e : SimDevice::swir);
  const auto result = hsi_.dispatch(camera, as_hsi_trigger(event));
  if (result.status == ppbng_hsi::LineStatus::produced && result.line.has_value()) {
    observation.produced = true;
    observation.segment_id = result.line->index.segment_id;
    observation.sample_index = result.line->index.segment_line_index;
    accumulate(observation);
  }
}

void IntegrationScenario::emit_rgb(const std::uint64_t sequence)
{
  const auto event = emit(ppbng_timing::Channel::kRgb, sequence);
  auto & observation = add_observation(event, SimDevice::rgb);
  auto result = rgb_.next_frame(mock_timeout, rgb_stop_.token());
  if (result.status.ok() && result.frame) {
    observation.produced = true;
    observation.segment_id = static_cast<std::uint32_t>(result.frame.info().segment_index);
    observation.sample_index = result.frame.info().frame_id;
    accumulate(observation);
  }
}

void IntegrationScenario::emit_thermal(const std::uint64_t sequence)
{
  const auto event = emit(ppbng_timing::Channel::kThermal, sequence);
  auto & observation = add_observation(event, SimDevice::thermal);
  if (thermal_disconnected_) {
    return;
  }
  auto result = thermal_.next_frame(mock_timeout, thermal_stop_.token());
  const auto layout = ppbng_storage::validate_and_split_thermal_payload(
    ppbng_storage::ThermalPayloadLayout::a6701_mono16(), result.frame.size());
  if (result.status.ok() && result.frame && layout.ok()) {
    observation.produced = true;
    observation.segment_id = static_cast<std::uint32_t>(result.frame.info().segment_index);
    observation.sample_index = result.frame.info().frame_id;
    accumulate(observation);
  }
}

bool IntegrationScenario::stop_devices_and_finalize()
{
  const auto hsi_stop = hsi_.stop_both();
  const auto rgb_stop = rgb_.stop(mock_timeout);
  const auto thermal_stop = thermal_.stop(mock_timeout);
  timing_.disarm();
  return hsi_stop.fx10e.success && hsi_stop.swir.success && rgb_stop.ok() &&
    thermal_stop.ok() && finalize_indices();
}

bool IntegrationScenario::finalize_indices()
{
  if (indices_finalized_) {
    return true;
  }
  for (std::size_t device = 0; device < accumulators_.size(); ++device) {
    const auto sim_device = static_cast<SimDevice>(device);
    for (std::size_t segment = 0; segment < accumulators_[device].size(); ++segment) {
      const auto & accumulator = accumulators_[device][segment];
      if (!accumulator.used) {
        return false;
      }
      const ppbng_storage::SegmentRecord record{
        segment,
        "segments/" + device_stem(sim_device) + "_" + std::to_string(segment) + ".raw",
        accumulator.first_event_id,
        accumulator.last_event_id,
        accumulator.sample_count};
      if (segment_indices_[device].append(record) != ppbng_storage::SegmentIndexError::none) {
        return false;
      }
    }
  }
  indices_finalized_ = true;
  return true;
}

void IntegrationScenario::accumulate(const TriggerObservation & observation)
{
  auto & segments = accumulators_[device_index(observation.device)];
  if (segments.size() <= observation.segment_id) {
    segments.resize(static_cast<std::size_t>(observation.segment_id) + 1U);
  }
  auto & accumulator = segments[observation.segment_id];
  if (!accumulator.used) {
    accumulator.used = true;
    accumulator.first_event_id = observation.event_id;
  }
  accumulator.last_event_id = observation.event_id;
  ++accumulator.sample_count;
}

std::size_t IntegrationScenario::device_index(const SimDevice device) noexcept
{
  return static_cast<std::size_t>(device);
}

std::string IntegrationScenario::device_stem(const SimDevice device)
{
  switch (device) {
    case SimDevice::fx10e: return "fx10e";
    case SimDevice::swir: return "swir";
    case SimDevice::rgb: return "rgb";
    case SimDevice::thermal: return "thermal";
    default: return "unknown";
  }
}

}  // namespace ppbng_sim

