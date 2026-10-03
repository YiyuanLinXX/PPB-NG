#include "ppbng_timing/host_client.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ppbng_timing {
namespace {

HostClientStatus from_transport(const TransportResult& result) {
  switch (result.code) {
    case TransportCode::ok: return {};
    case TransportCode::timeout: return {HostClientCode::timeout, result.detail};
    case TransportCode::cancelled: return {HostClientCode::cancelled, result.detail};
    case TransportCode::disconnected: return {HostClientCode::disconnected, result.detail};
    case TransportCode::io_error: return {HostClientCode::io_error, result.detail};
  }
  return {HostClientCode::io_error, "unknown transport result"};
}

std::chrono::milliseconds remaining_until(
    std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) return std::chrono::milliseconds(0);
  return std::max(std::chrono::milliseconds(1),
                  std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now));
}

bool valid_identity(const ControllerPortIdentity& identity) {
  return !identity.com_path.empty() && identity.baud_rate != 0U &&
         identity.usb_vid != 0U && identity.usb_pid != 0U &&
         !identity.usb_serial.empty();
}

std::uint64_t host_monotonic_ns() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

bool HostStopToken::stop_requested() const noexcept { return flag_ && flag_->load(); }
HostStopSource::HostStopSource() : flag_(std::make_shared<std::atomic_bool>(false)) {}
void HostStopSource::request_stop() noexcept { flag_->store(true); }

TimingControllerClient::TimingControllerClient(
    std::unique_ptr<IControllerTransport> transport)
    : transport_(std::move(transport)) {
  if (!transport_) throw std::invalid_argument("controller transport is required");
}

TimingControllerClient::~TimingControllerClient() { close(); }

HostClientStatus TimingControllerClient::fatal(std::string detail) {
  state_ = HostClientState::fault;
  return {HostClientCode::fatal, std::move(detail)};
}

TimingControllerClient::Reply TimingControllerClient::transact(
    MessageType request_type, const std::vector<std::uint8_t>& payload,
    MessageType response_type, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  if (timeout.count() <= 0) {
    return {{HostClientCode::timeout, "finite positive timeout required"}, {}, 0U};
  }
  if (stop.stop_requested()) {
    return {{HostClientCode::cancelled, "stop requested"}, {}, 0U};
  }
  if (!transport_->is_open()) {
    return {{HostClientCode::disconnected, "controller transport is closed"}, {}, 0U};
  }
  const std::uint32_t sequence = next_command_sequence_++;
  Packet request;
  request.type = request_type;
  request.sequence = sequence;
  request.payload = payload;
  std::vector<std::uint8_t> bytes;
  try {
    bytes = serialize_packet(request);
  } catch (const std::exception& error) {
    return {{HostClientCode::invalid_configuration, error.what()}, {}, sequence};
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (int attempt = 0; attempt < 2; ++attempt) {
    auto remaining = remaining_until(deadline);
    if (remaining.count() <= 0) {
      return {{HostClientCode::timeout, "controller transaction timed out"}, {}, sequence};
    }
    auto written = transport_->write(bytes, remaining, stop);
    if (!written.ok()) {
      const auto status = from_transport(written);
      if (written.code == TransportCode::timeout && attempt == 0) continue;
      return {status, {}, sequence};
    }

    while ((remaining = remaining_until(deadline)).count() > 0) {
      auto received = transport_->read(4096U, remaining, stop);
      if (!received.ok()) {
        if (received.code == TransportCode::timeout && attempt == 0) break;
        return {from_transport(received), {}, sequence};
      }
      std::vector<Packet> packets;
      try {
        packets = decoder_.feed(received.bytes);
      } catch (const std::exception& error) {
        return {fatal(std::string("controller stream decoder failed: ") + error.what()),
                {}, sequence};
      }
      if (decoder_.crc_or_format_errors() != decoder_errors_seen_) {
        decoder_errors_seen_ = decoder_.crc_or_format_errors();
        return {fatal("controller stream CRC or framing error"), {}, sequence};
      }
      for (const auto& packet : packets) {
        bool matches = packet.type == response_type;
        if (matches && response_type != MessageType::kAck && packet.sequence != sequence) {
          matches = false;
        }
        if (matches) return {{}, packet, sequence};
        auto status = validate_async_packet(packet, queued_events_);
        if (!status.ok()) return {status, {}, sequence};
      }
    }
  }
  return {{HostClientCode::timeout, "controller transaction timed out after idempotent retry"},
          {}, sequence};
}

HostClientStatus TimingControllerClient::command_with_ack(
    MessageType type, const std::vector<std::uint8_t>& payload,
    ControllerState expected_state, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  auto reply = transact(type, payload, MessageType::kAck, timeout, stop);
  if (!reply.status.ok()) return reply.status;
  try {
    const auto ack = decode_ack(reply.packet.payload);
    if (ack.acknowledged_sequence != reply.command_sequence || ack.code != 0U ||
        ack.state != expected_state) {
      return fatal("controller ACK sequence/code/state mismatch");
    }
  } catch (const std::exception& error) {
    return fatal(std::string("invalid controller ACK: ") + error.what());
  }
  return {};
}

HostClientStatus TimingControllerClient::query_status(
    ControllerState expected_state, std::chrono::milliseconds timeout,
    const HostStopToken& stop, bool allow_running) {
  auto reply = transact(MessageType::kStatus, {}, MessageType::kStatus, timeout, stop);
  if (!reply.status.ok()) return reply.status;
  try {
    const auto status = decode_status_report(reply.packet.payload);
    if (status.boot_id != boot_id_) return fatal("controller boot ID changed");
    if (status.error_flags != 0U || status.state == ControllerState::kFault) {
      return fatal("controller status reports fatal error flags");
    }
    if (status.state != expected_state &&
        !(allow_running && status.state == ControllerState::kRunning)) {
      return fatal("controller state readback mismatch");
    }
    if (schedule_ && status.active_schedule_id != schedule_->schedule_id) {
      return fatal("controller active schedule ID readback mismatch");
    }
    if (!schedule_ && status.active_schedule_id != 0U) {
      return fatal("controller reports an unexpected active schedule");
    }
    last_status_ = status;
  } catch (const std::exception& error) {
    return fatal(std::string("invalid controller status readback: ") + error.what());
  }
  return {};
}

HostClientStatus TimingControllerClient::connect(
    const ControllerPortIdentity& identity, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  if (state_ != HostClientState::disconnected) {
    return {HostClientCode::invalid_configuration, "client must be disconnected before connect"};
  }
  if (!valid_identity(identity)) {
    return {HostClientCode::invalid_configuration,
            "explicit COM path, baud, USB VID/PID and serial are required"};
  }
  if (!transport_->can_verify_usb_identity() &&
      !identity.allow_unverified_os_usb_identity) {
    return {HostClientCode::identity_unverified,
            "COM API cannot verify VID/PID/serial; trusted external identity attestation required"};
  }
  auto opened = transport_->open(identity, timeout, stop);
  if (!opened.ok()) return from_transport(opened);
  auto reply = transact(MessageType::kGetVersion, {}, MessageType::kVersionReport,
                        timeout, stop);
  if (!reply.status.ok()) {
    transport_->close();
    return reply.status;
  }
  try {
    const auto version = decode_version_info(reply.packet.payload);
    if (version.boot_id == 0U || version.ticks_per_second == 0U) {
      transport_->close();
      return {HostClientCode::readback_mismatch, "invalid controller version identity"};
    }
    if (identity.expected_boot_id && *identity.expected_boot_id != version.boot_id) {
      transport_->close();
      return {HostClientCode::readback_mismatch, "controller boot ID mismatch"};
    }
    boot_id_ = version.boot_id;
    capabilities_ = version.capabilities;
  } catch (const std::exception& error) {
    transport_->close();
    return {HostClientCode::protocol_error,
            std::string("invalid controller version response: ") + error.what()};
  }
  state_ = HostClientState::connected;
  return {};
}

HostClientStatus TimingControllerClient::configure(
    const ScheduleConfig& schedule, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  if (state_ != HostClientState::connected && state_ != HostClientState::configured) {
    return {HostClientCode::invalid_configuration, "connect before configure"};
  }
  try { validate_schedule(schedule); }
  catch (const std::exception& error) {
    return {HostClientCode::invalid_configuration, error.what()};
  }
  auto result = command_with_ack(MessageType::kConfigureSchedule, encode(schedule),
                                 ControllerState::kConfigured, timeout, stop);
  if (!result.ok()) return result;
  const auto previous = schedule_;
  schedule_ = schedule;
  result = query_status(ControllerState::kConfigured, timeout, stop);
  if (!result.ok()) { schedule_ = previous; return result; }
  state_ = HostClientState::configured;
  return {};
}

HostClientStatus TimingControllerClient::freeze(
    std::chrono::milliseconds timeout, const HostStopToken& stop) {
  if (state_ != HostClientState::configured || !schedule_) {
    return {HostClientCode::invalid_configuration, "configure before freeze"};
  }
  auto result = command_with_ack(MessageType::kFreezeConfiguration, {},
                                 ControllerState::kFrozen, timeout, stop);
  if (!result.ok()) return result;
  result = query_status(ControllerState::kFrozen, timeout, stop);
  if (!result.ok()) return result;
  state_ = HostClientState::frozen;
  return {};
}

HostClientStatus TimingControllerClient::arm(
    const ArmRequest& request, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  if (state_ != HostClientState::frozen || !schedule_ ||
      request.schedule_id != schedule_->schedule_id) {
    return {HostClientCode::invalid_configuration, "freeze matching schedule before arm"};
  }
  auto result = command_with_ack(MessageType::kArmNextWholeSecond, encode(request),
                                 ControllerState::kWaitingForPps, timeout, stop);
  if (!result.ok()) return result;
  result = query_status(ControllerState::kWaitingForPps, timeout, stop, true);
  if (!result.ok()) return result;
  state_ = last_status_->state == ControllerState::kRunning
               ? HostClientState::running : HostClientState::waiting_for_pps;
  return {};
}

HostClientStatus TimingControllerClient::start_at_current_tick(
    const StartAtCurrentTickRequest& request, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  if (state_ != HostClientState::frozen || !schedule_ ||
      request.schedule_id != schedule_->schedule_id) {
    return {HostClientCode::invalid_configuration,
            "freeze matching schedule before start_at_current_tick"};
  }
  if ((capabilities_ & kCapabilityStartAtCurrentTick) == 0U) {
    return {HostClientCode::invalid_configuration,
            "controller does not advertise start-at-current-tick capability"};
  }
  auto result = command_with_ack(MessageType::kStartAtCurrentTick, encode(request),
                                 ControllerState::kRunning, timeout, stop);
  if (!result.ok()) return result;
  result = query_status(ControllerState::kRunning, timeout, stop);
  if (!result.ok()) return result;
  state_ = HostClientState::running;
  return {};
}

HostClientStatus TimingControllerClient::disarm(
    std::chrono::milliseconds timeout, const HostStopToken& stop) {
  // A latched host fault must not prevent a best-effort output shutdown. The
  // controller may still be reachable, and Disarm is idempotent in protocol v1.
  if (state_ == HostClientState::disconnected || !transport_->is_open()) {
    return {HostClientCode::invalid_configuration, "active healthy connection required"};
  }
  auto result = command_with_ack(MessageType::kDisarm, {}, ControllerState::kIdle,
                                 timeout, stop);
  if (!result.ok()) return result;
  schedule_.reset();
  result = query_status(ControllerState::kIdle, timeout, stop);
  if (!result.ok()) return result;
  state_ = HostClientState::connected;
  return {};
}

HostClientStatus TimingControllerClient::disarm_keep_configuration(
    std::chrono::milliseconds timeout, const HostStopToken& stop) {
  if ((state_ != HostClientState::waiting_for_pps &&
       state_ != HostClientState::running && state_ != HostClientState::fault) ||
      !schedule_ || !transport_->is_open()) {
    return {HostClientCode::invalid_configuration,
            "armed/running connection with frozen configuration required"};
  }
  auto result = command_with_ack(MessageType::kDisarmKeepConfiguration, {},
                                 ControllerState::kFrozen, timeout, stop);
  if (!result.ok()) return result;
  result = query_status(ControllerState::kFrozen, timeout, stop);
  if (!result.ok()) return result;
  state_ = HostClientState::frozen;
  return {};
}

HostClientStatus TimingControllerClient::validate_async_packet(
    const Packet& packet, std::vector<ControllerEvent>& events) {
  try {
    if (packet.type == MessageType::kError) {
      const auto error = decode_error_report(packet.payload);
      return fatal("controller error " + std::to_string(error.code) + ": " + error.detail);
    }
    if (packet.type == MessageType::kStatus) {
      const auto status = decode_status_report(packet.payload);
      if (status.boot_id != boot_id_) return fatal("controller reboot detected");
      if (status.error_flags != 0U || status.state == ControllerState::kFault) {
        return fatal("controller overflow/fault status detected");
      }
      last_status_ = status;
      if (status.state == ControllerState::kRunning) state_ = HostClientState::running;
      events.emplace_back(status);
      return {};
    }
    if (packet.type == MessageType::kPpsAnchor) {
      const auto anchor = decode_pps_anchor(packet.payload);
      if (anchor.boot_id != boot_id_) return fatal("controller reboot detected in PPS anchor");
      if (last_pps_sequence_ && anchor.pps_sequence <= *last_pps_sequence_) {
        return fatal("PPS sequence duplicate or regression");
      }
      last_pps_sequence_ = anchor.pps_sequence;
      events.emplace_back(ReceivedPpsAnchor{anchor, host_monotonic_ns()});
      return {};
    }
    if (packet.type == MessageType::kTriggerEvent) {
      const auto event = decode_trigger_event(packet.payload);
      if (event.boot_id != boot_id_) return fatal("controller reboot detected in trigger event");
      const auto observation = global_events_.observe(event.boot_id, event.event_sequence);
      if (observation != SequenceObservation::kFirst &&
          observation != SequenceObservation::kInOrder) {
        return fatal("trigger global sequence gap/duplicate/regression/reboot");
      }
      const auto index = static_cast<std::size_t>(event.channel);
      if (index == 0U || index >= channel_sequences_.size()) {
        return fatal("invalid trigger channel");
      }
      if (channel_sequences_[index] &&
          (*channel_sequences_[index] ==
               (std::numeric_limits<std::uint64_t>::max)() ||
           event.channel_sequence != *channel_sequences_[index] + 1U)) {
        return fatal("trigger channel sequence gap/duplicate/regression");
      }
      channel_sequences_[index] = event.channel_sequence;
      if (event.channel == Channel::kRgb) {
        if (pending_snapshot_rgb_) return fatal("snapshot RGB event missing thermal pair");
        pending_snapshot_rgb_ = event;
      } else if (event.channel == Channel::kThermal) {
        if (!pending_snapshot_rgb_ ||
            pending_snapshot_rgb_->channel_sequence != event.channel_sequence ||
            pending_snapshot_rgb_->pps_sequence != event.pps_sequence ||
            pending_snapshot_rgb_->offset_ticks != event.offset_ticks ||
            pending_snapshot_rgb_->lock != event.lock) {
          return fatal("snapshot RGB/thermal pair mismatch");
        }
        pending_snapshot_rgb_.reset();
      }
      events.emplace_back(ReceivedTriggerEvent{event, host_monotonic_ns()});
      return {};
    }
    return fatal("unexpected controller packet type");
  } catch (const std::exception& error) {
    return fatal(std::string("invalid asynchronous controller packet: ") + error.what());
  }
}

EventPollResult TimingControllerClient::poll_events(
    std::chrono::milliseconds timeout, const HostStopToken& stop) {
  EventPollResult result;
  result.events = std::move(queued_events_);
  queued_events_.clear();
  if (state_ == HostClientState::disconnected || state_ == HostClientState::fault) {
    result.status = {state_ == HostClientState::fault ? HostClientCode::fatal
                                                      : HostClientCode::disconnected,
                     "controller client is not healthy and connected"};
    return result;
  }
  auto received = transport_->read(4096U, timeout, stop);
  if (!received.ok()) {
    result.status = from_transport(received);
    return result;
  }
  try {
    for (const auto& packet : decoder_.feed(received.bytes)) {
      result.status = validate_async_packet(packet, result.events);
      if (!result.status.ok()) return result;
    }
  } catch (const std::exception& error) {
    result.status = fatal(std::string("controller stream decoder failed: ") + error.what());
    return result;
  }
  if (decoder_.crc_or_format_errors() != decoder_errors_seen_) {
    decoder_errors_seen_ = decoder_.crc_or_format_errors();
    result.status = fatal("controller stream CRC or framing error");
  }
  return result;
}

void TimingControllerClient::close() noexcept {
  transport_->close();
  decoder_.reset();
  state_ = HostClientState::disconnected;
  boot_id_ = 0U;
  capabilities_ = 0U;
  schedule_.reset();
  last_status_.reset();
  global_events_.reset();
  channel_sequences_ = {};
  last_pps_sequence_.reset();
  pending_snapshot_rgb_.reset();
  queued_events_.clear();
  decoder_errors_seen_ = 0U;
}

}  // namespace ppbng_timing
