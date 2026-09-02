#include "ppbng_hsi/mock_hsi_adapter.hpp"

#include "ppbng_hsi/hsi_format.hpp"

#include <chrono>
#include <limits>
#include <utility>

namespace ppbng_hsi
{

MockHsiAdapter::MockHsiAdapter(const CameraKind kind)
: kind_(kind)
{
  config_.kind = kind;
}

CameraKind MockHsiAdapter::kind() const noexcept {return kind_;}
HsiState MockHsiAdapter::state() const noexcept {return state_;}
const HsiConfig & MockHsiAdapter::config() const noexcept {return config_;}
std::uint32_t MockHsiAdapter::segment_id() const noexcept {return segment_id_;}

OperationResult MockHsiAdapter::connect()
{
  if (consume_failure(FailurePoint::connect)) {
    return fail(FailurePoint::connect, "injected connect failure");
  }
  if (state_ != HsiState::disconnected) {
    return {false, "connect requires DISCONNECTED"};
  }
  state_ = HsiState::connected;
  return {true, "connected"};
}

OperationResult MockHsiAdapter::configure(const HsiConfig & config)
{
  if (consume_failure(FailurePoint::configure)) {
    return fail(FailurePoint::configure, "injected configure failure");
  }
  if (state_ != HsiState::connected && state_ != HsiState::configured) {
    return {false, "configure requires CONNECTED or CONFIGURED"};
  }
  if (config.kind != kind_) {
    return {false, "camera kind does not match adapter"};
  }
  const auto validation = HsiFormat::validate_config(config);
  if (!validation.success) {
    return validation;
  }
  config_ = config;
  configured_ = true;
  state_ = HsiState::configured;
  return {true, "configured"};
}

OperationResult MockHsiAdapter::close_shutter()
{
  if (consume_failure(FailurePoint::close_shutter)) {
    return fail(FailurePoint::close_shutter, "injected shutter-close failure");
  }
  if (state_ != HsiState::configured && state_ != HsiState::ready) {
    return {false, "close_shutter requires CONFIGURED or READY"};
  }
  state_ = HsiState::shutter_closed;
  return {true, "shutter closed"};
}

OperationResult MockHsiAdapter::begin_dark_capture(const std::size_t line_count)
{
  if (consume_failure(FailurePoint::begin_dark)) {
    return fail(FailurePoint::begin_dark, "injected dark-capture start failure");
  }
  if (state_ != HsiState::shutter_closed) {
    return {false, "begin_dark_capture requires SHUTTER_CLOSED"};
  }
  if (line_count == 0) {
    return {false, "dark line count must be non-zero"};
  }
  dark_lines_remaining_ = line_count;
  segment_dark_line_index_ = 0;
  state_ = HsiState::dark_collecting;
  return {true, "dark capture started"};
}

OperationResult MockHsiAdapter::open_shutter()
{
  if (consume_failure(FailurePoint::open_shutter)) {
    return fail(FailurePoint::open_shutter, "injected shutter-open failure");
  }
  if (state_ != HsiState::shutter_closed) {
    return {false, "open_shutter requires SHUTTER_CLOSED"};
  }
  state_ = HsiState::ready;
  return {true, "shutter open"};
}

OperationResult MockHsiAdapter::start_streaming()
{
  if (consume_failure(FailurePoint::start_streaming)) {
    return fail(FailurePoint::start_streaming, "injected stream-start failure");
  }
  if (state_ != HsiState::ready) {
    return {false, "start_streaming requires READY"};
  }
  if (has_started_segment_) {
    if (segment_id_ == std::numeric_limits<std::uint32_t>::max()) {
      state_ = HsiState::fault;
      return {false, "segment ID exhausted"};
    }
    ++segment_id_;
  } else {
    has_started_segment_ = true;
  }
  segment_sample_line_index_ = 0;
  state_ = HsiState::streaming;
  return {true, "streaming"};
}

OperationResult MockHsiAdapter::stop_streaming()
{
  if (state_ != HsiState::streaming) {
    return {false, "stop_streaming requires STREAMING"};
  }
  state_ = HsiState::ready;
  return {true, "stream stopped"};
}

OperationResult MockHsiAdapter::recover()
{
  if (consume_failure(FailurePoint::recover)) {
    return fail(FailurePoint::recover, "injected recovery failure");
  }
  if (state_ == HsiState::disconnected) {
    return {false, "recover requires a previously connected adapter"};
  }
  shutter_open_before_recovery_ = state_ == HsiState::streaming ||
    state_ == HsiState::ready || shutter_open_before_recovery_;
  state_ = HsiState::recovering;
  if (!configured_) {
    state_ = HsiState::connected;
    return {false, "no validated configuration is available"};
  }
  state_ = shutter_open_before_recovery_ ? HsiState::ready : HsiState::configured;
  return {true, "rediscovered, reconfigured, and critical state read back"};
}

LineResult MockHsiAdapter::on_trigger(const TriggerEvent & trigger)
{
  if (disconnect_on_next_line_) {
    disconnect_on_next_line_ = false;
    shutter_open_before_recovery_ = state_ == HsiState::streaming || state_ == HsiState::ready;
    state_ = HsiState::recovering;
    return {LineStatus::disconnected, std::nullopt, "injected disconnect"};
  }
  if (consume_failure(FailurePoint::acquire_line)) {
    state_ = HsiState::fault;
    return {LineStatus::fault, std::nullopt, "injected line acquisition failure"};
  }
  if (state_ != HsiState::streaming && state_ != HsiState::dark_collecting) {
    return {LineStatus::not_ready, std::nullopt, "adapter is not accepting line triggers"};
  }
  if (config_.trigger_mode != "External") {
    return {LineStatus::invalid_trigger, std::nullopt,
      "external trigger is invalid in Internal mode"};
  }
  if (trigger.channel != config_.trigger_channel || trigger.ticks_per_second == 0 ||
    (trigger.time_status == TimeStatus::locked &&
    trigger.offset_ticks >= trigger.ticks_per_second))
  {
    return {LineStatus::invalid_trigger, std::nullopt, "invalid trigger channel or tick fields"};
  }
  if (last_trigger_sequence_.has_value() && trigger.channel_sequence <= *last_trigger_sequence_) {
    return {LineStatus::invalid_trigger, std::nullopt, "trigger sequence is duplicate or out of order"};
  }

  const CaptureKind capture_kind =
    state_ == HsiState::dark_collecting ? CaptureKind::dark : CaptureKind::sample;
  auto result = make_line(trigger, capture_kind);
  last_trigger_sequence_ = trigger.channel_sequence;

  if (capture_kind == CaptureKind::dark) {
    --dark_lines_remaining_;
    if (dark_lines_remaining_ == 0) {
      state_ = HsiState::shutter_closed;
    }
  }
  return result;
}

LineResult MockHsiAdapter::poll_internal()
{
  if (state_ != HsiState::streaming && state_ != HsiState::dark_collecting) {
    return {LineStatus::not_ready, std::nullopt, "adapter is not acquiring"};
  }
  if (config_.trigger_mode != "Internal") {
    return {LineStatus::invalid_trigger, std::nullopt,
      "internal polling is invalid in External mode"};
  }
  TriggerEvent evidence;
  evidence.channel = "internal:" + config_.device_id;
  evidence.channel_sequence = ++internal_sequence_;
  evidence.time_status = TimeStatus::unsynced;
  const CaptureKind capture_kind =
    state_ == HsiState::dark_collecting ? CaptureKind::dark : CaptureKind::sample;
  auto result = make_line(evidence, capture_kind);
  result.line->index.trigger_sequence = 0U;
  result.line->index.association_status = AssociationStatus::unverified;
  if (capture_kind == CaptureKind::dark && --dark_lines_remaining_ == 0U) {
    state_ = HsiState::shutter_closed;
  }
  return result;
}

void MockHsiAdapter::fail_next(const FailurePoint point) noexcept {next_failure_ = point;}
void MockHsiAdapter::disconnect_on_next_line() noexcept {disconnect_on_next_line_ = true;}

bool MockHsiAdapter::consume_failure(const FailurePoint point) noexcept
{
  if (next_failure_.has_value() && *next_failure_ == point) {
    next_failure_.reset();
    return true;
  }
  return false;
}

OperationResult MockHsiAdapter::fail(const FailurePoint, const char * message)
{
  state_ = HsiState::fault;
  return {false, message};
}

LineResult MockHsiAdapter::make_line(
  const TriggerEvent & trigger, const CaptureKind capture_kind)
{
  const std::uint64_t element_count =
    static_cast<std::uint64_t>(config_.spatial_samples) * config_.spectral_bands;
  std::vector<std::uint16_t> pixels(
    static_cast<std::size_t>(element_count),
    capture_kind == CaptureKind::dark ? std::uint16_t{64} :
    static_cast<std::uint16_t>((camera_line_sequence_ % 4095U) + 1U));

  const bool gap = last_trigger_sequence_.has_value() &&
    trigger.channel_sequence > *last_trigger_sequence_ + 1U;
  const std::uint64_t missing = gap ? trigger.channel_sequence - *last_trigger_sequence_ - 1U : 0U;
  const std::uint64_t line_index = capture_kind == CaptureKind::dark ?
    segment_dark_line_index_++ : segment_sample_line_index_++;
  const std::uint64_t payload_bytes = HsiFormat::payload_bytes_per_line(config_);

  LineIndexEntry index;
  index.segment_id = segment_id_;
  index.capture_kind = capture_kind;
  index.segment_line_index = line_index;
  index.camera_line_sequence = camera_line_sequence_++;
  index.trigger_sequence = trigger.channel_sequence;
  index.pps_sequence = trigger.pps_sequence;
  index.utc_time_ns = trigger.utc_time_ns;
  index.time_status = trigger.time_status;
  index.uncertainty_ns = trigger.uncertainty_ns;
  index.host_receive_monotonic_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  index.raw_file_offset_bytes = line_index * payload_bytes;
  index.payload_size_bytes = payload_bytes;
  index.sequence_gap_before = gap;
  index.missing_trigger_count = missing;

  LineRecord record;
  record.device_id = config_.device_id;
  record.camera_kind = kind_;
  record.trigger = trigger;
  record.index = index;
  record.pixels = std::move(pixels);
  return {LineStatus::produced, std::move(record), "line produced"};
}

}  // namespace ppbng_hsi
