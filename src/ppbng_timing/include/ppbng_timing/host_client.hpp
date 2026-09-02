#pragma once

#include "ppbng_timing/controller_model.hpp"
#include "ppbng_timing/protocol.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace ppbng_timing {

class HostStopToken {
 public:
  bool stop_requested() const noexcept;
 private:
  friend class HostStopSource;
  explicit HostStopToken(std::shared_ptr<std::atomic_bool> flag) : flag_(std::move(flag)) {}
  std::shared_ptr<std::atomic_bool> flag_;
};

class HostStopSource {
 public:
  HostStopSource();
  HostStopToken token() const { return HostStopToken(flag_); }
  void request_stop() noexcept;
 private:
  std::shared_ptr<std::atomic_bool> flag_;
};

enum class TransportCode { ok, timeout, cancelled, disconnected, io_error };
struct TransportResult {
  TransportCode code{TransportCode::ok};
  std::vector<std::uint8_t> bytes;
  std::string detail;
  bool ok() const noexcept { return code == TransportCode::ok; }
};

struct ControllerPortIdentity {
  std::string com_path;
  std::uint32_t baud_rate{};
  std::uint16_t usb_vid{};
  std::uint16_t usb_pid{};
  std::string usb_serial;
  std::optional<std::uint32_t> expected_boot_id;
  bool allow_unverified_os_usb_identity{false};
};

class IControllerTransport {
 public:
  virtual ~IControllerTransport() = default;
  virtual bool can_verify_usb_identity() const noexcept = 0;
  virtual TransportResult open(const ControllerPortIdentity&, std::chrono::milliseconds,
                               const HostStopToken&) = 0;
  virtual TransportResult write(const std::vector<std::uint8_t>&,
                                std::chrono::milliseconds, const HostStopToken&) = 0;
  virtual TransportResult read(std::size_t, std::chrono::milliseconds,
                               const HostStopToken&) = 0;
  virtual void close() noexcept = 0;
  virtual bool is_open() const noexcept = 0;
};

// Construction has no OS side effects. This backend opens only the exact COM path
// supplied to open(); it never enumerates devices. Win32 COM handles cannot directly
// prove VID/PID/USB serial, so can_verify_usb_identity() is false.
class WindowsControllerSerialTransport final : public IControllerTransport {
 public:
  WindowsControllerSerialTransport();
  ~WindowsControllerSerialTransport() override;
  WindowsControllerSerialTransport(const WindowsControllerSerialTransport&) = delete;
  WindowsControllerSerialTransport& operator=(const WindowsControllerSerialTransport&) = delete;
  bool can_verify_usb_identity() const noexcept override { return false; }
  TransportResult open(const ControllerPortIdentity&, std::chrono::milliseconds,
                       const HostStopToken&) override;
  TransportResult write(const std::vector<std::uint8_t>&, std::chrono::milliseconds,
                        const HostStopToken&) override;
  TransportResult read(std::size_t, std::chrono::milliseconds,
                       const HostStopToken&) override;
  void close() noexcept override;
  bool is_open() const noexcept override;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

enum class HostClientCode {
  ok,
  invalid_configuration,
  identity_unverified,
  timeout,
  cancelled,
  disconnected,
  protocol_error,
  readback_mismatch,
  fatal,
  io_error,
};
struct HostClientStatus {
  HostClientCode code{HostClientCode::ok};
  std::string detail;
  bool ok() const noexcept { return code == HostClientCode::ok; }
};

enum class HostClientState {
  disconnected,
  connected,
  configured,
  frozen,
  waiting_for_pps,
  running,
  fault,
};

struct ReceivedPpsAnchor {
  PpsAnchor anchor;
  std::uint64_t host_receive_monotonic_ns{};
};
struct ReceivedTriggerEvent {
  TriggerEvent trigger;
  std::uint64_t host_receive_monotonic_ns{};
};
using ControllerEvent = std::variant<ReceivedPpsAnchor, ReceivedTriggerEvent, StatusReport>;
struct EventPollResult {
  HostClientStatus status;
  std::vector<ControllerEvent> events;
};

class TimingControllerClient {
 public:
  explicit TimingControllerClient(std::unique_ptr<IControllerTransport> transport);
  ~TimingControllerClient();
  TimingControllerClient(const TimingControllerClient&) = delete;
  TimingControllerClient& operator=(const TimingControllerClient&) = delete;

  HostClientStatus connect(const ControllerPortIdentity&, std::chrono::milliseconds,
                           const HostStopToken&);
  HostClientStatus configure(const ScheduleConfig&, std::chrono::milliseconds,
                             const HostStopToken&);
  HostClientStatus freeze(std::chrono::milliseconds, const HostStopToken&);
  HostClientStatus arm(const ArmRequest&, std::chrono::milliseconds,
                       const HostStopToken&);
  HostClientStatus start_at_current_tick(const StartAtCurrentTickRequest&,
                                         std::chrono::milliseconds,
                                         const HostStopToken&);
  HostClientStatus disarm(std::chrono::milliseconds, const HostStopToken&);
  HostClientStatus disarm_keep_configuration(std::chrono::milliseconds,
                                             const HostStopToken&);
  EventPollResult poll_events(std::chrono::milliseconds, const HostStopToken&);
  void close() noexcept;

  HostClientState state() const noexcept { return state_; }
  std::uint32_t boot_id() const noexcept { return boot_id_; }
  std::uint32_t capabilities() const noexcept { return capabilities_; }
  bool is_transport_open() const noexcept { return transport_->is_open(); }
  const std::optional<StatusReport>& last_status() const noexcept { return last_status_; }

 private:
  struct Reply { HostClientStatus status; Packet packet; std::uint32_t command_sequence{}; };
  Reply transact(MessageType, const std::vector<std::uint8_t>&, MessageType,
                 std::chrono::milliseconds, const HostStopToken&);
  HostClientStatus command_with_ack(MessageType, const std::vector<std::uint8_t>&,
                                    ControllerState, std::chrono::milliseconds,
                                    const HostStopToken&);
  HostClientStatus query_status(ControllerState, std::chrono::milliseconds,
                                const HostStopToken&, bool allow_running = false);
  HostClientStatus fatal(std::string);
  HostClientStatus validate_async_packet(const Packet&, std::vector<ControllerEvent>&);

  std::unique_ptr<IControllerTransport> transport_;
  StreamDecoder decoder_;
  HostClientState state_{HostClientState::disconnected};
  std::uint32_t next_command_sequence_{1U};
  std::uint32_t boot_id_{};
  std::uint32_t capabilities_{};
  std::optional<ScheduleConfig> schedule_;
  std::optional<StatusReport> last_status_;
  EventSequenceTracker global_events_;
  std::array<std::optional<std::uint64_t>, 5> channel_sequences_{};
  std::optional<std::uint64_t> last_pps_sequence_;
  std::optional<TriggerEvent> pending_snapshot_rgb_;
  std::vector<ControllerEvent> queued_events_;
  std::size_t decoder_errors_seen_{};
};

}  // namespace ppbng_timing
