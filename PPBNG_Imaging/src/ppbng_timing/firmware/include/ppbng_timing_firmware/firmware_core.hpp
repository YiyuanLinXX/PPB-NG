#pragma once

#include "ppbng_timing/protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ppbng_timing_firmware {

enum class PhysicalOutput : std::uint8_t { fx10e, swir, snapshot };
enum class CaptureEdge : std::uint8_t { rising, falling };

// Board/HAL implementation contract. Every method may be called from an ISR and
// therefore must be bounded, nonblocking, and allocation-free.
class ITimingHardware {
 public:
  using CriticalToken = std::uintptr_t;
  virtual ~ITimingHardware() = default;
  // Masks every interrupt/task which can call this core and returns an opaque
  // restore token. Calls must be nesting-safe and bounded, including from ISR.
  virtual CriticalToken enter_timing_critical() noexcept = 0;
  virtual void exit_timing_critical(CriticalToken token) noexcept = 0;
  virtual void configure_pps_input_capture(CaptureEdge edge) noexcept = 0;
  // Initial compares are preloaded before PPS. Board hardware must use a timer
  // slave-reset/trigger path so phase=0 is a real hardware edge, not an ISR write.
  virtual bool prepare_pps_synchronous_output(PhysicalOutput output,
      std::uint32_t phase_ticks, std::uint32_t pulse_width_ticks) noexcept = 0;
  virtual bool arm_output_gate_on_next_pps() noexcept = 0;
  virtual std::uint32_t minimum_compare_lead_ticks() const noexcept = 0;
  virtual bool configure_output_compare(PhysicalOutput output,
      std::uint64_t absolute_tick, std::uint32_t pulse_width_ticks) noexcept = 0;
  virtual void cancel_output_compare(PhysicalOutput output) noexcept = 0;
  virtual void set_output_gate(bool enabled) noexcept = 0;
};

// Optional board adapter boundary. USB CDC work is explicitly not performed in
// capture/compare ISRs. Implementations must feed complete v1 packets to the core
// from a non-ISR task and transmit returned packets from that task.
class IUsbCdc {
 public:
  virtual ~IUsbCdc() = default;
  virtual bool try_read(std::uint8_t* destination, std::size_t capacity,
                        std::size_t& bytes_read) noexcept = 0;
  virtual bool try_write(const std::uint8_t* source, std::size_t size) noexcept = 0;
};

struct SafetyPolicy {
  // Zero is deliberately fail-closed: Arm is rejected until both limits are set.
  std::uint64_t maximum_host_silence_ticks{};
  std::uint64_t maximum_pps_silence_ticks{};
};

enum ErrorFlag : std::uint32_t {
  kErrorNone = 0U,
  kErrorInvalidSchedule = 1U << 0U,
  kErrorEventOverflow = 1U << 1U,
  kErrorCounterOrTimeRegression = 1U << 2U,
  kErrorHardwareSchedule = 1U << 3U,
  kErrorHostWatchdog = 1U << 4U,
  kErrorPpsWatchdog = 1U << 5U,
  kErrorProtocol = 1U << 6U,
};

enum class EventKind : std::uint8_t { pps, trigger };
struct FirmwareEvent {
  EventKind kind{EventKind::pps};
  ppbng_timing::PpsAnchor pps;
  ppbng_timing::TriggerEvent trigger;
};

template <std::size_t Capacity>
class FixedEventRing {
 public:
  static_assert(Capacity > 1U, "event ring capacity must exceed one");
  bool push_isr(const FirmwareEvent& event) noexcept {
    const auto next = (head_ + 1U) % Capacity;
    if (next == tail_) return false;
    events_[head_] = event;
    head_ = next;
    return true;
  }
  bool pop(FirmwareEvent& event) noexcept {
    if (tail_ == head_) return false;
    event = events_[tail_];
    tail_ = (tail_ + 1U) % Capacity;
    return true;
  }
  std::size_t size() const noexcept {
    return head_ >= tail_ ? head_ - tail_ : Capacity - tail_ + head_;
  }
  void clear() noexcept { head_ = tail_ = 0U; }
 private:
  std::array<FirmwareEvent, Capacity> events_{};
  std::size_t head_{};
  std::size_t tail_{};
};

class FirmwareCore {
 public:
  static constexpr std::size_t kEventRingSlots = 257U;  // usable capacity: 256

  FirmwareCore(ITimingHardware& hardware, std::uint32_t boot_id,
               std::uint32_t hardware_ticks_per_second,
               SafetyPolicy policy = {}) noexcept;
  void reset(std::uint32_t new_boot_id) noexcept;

  // Non-ISR protocol entry point. The returned Packet is v1 wire-compatible.
  ppbng_timing::Packet handle_command(const ppbng_timing::Packet& request,
                                      std::uint64_t current_tick);

  // ISR entry points: fixed storage only, no allocation and no blocking.
  void on_pps_capture_isr(std::uint64_t captured_tick) noexcept;
  void on_output_compare_isr(PhysicalOutput output, std::uint64_t captured_tick) noexcept;
  void service_watchdogs_isr(std::uint64_t current_tick) noexcept;

  bool pop_event(FirmwareEvent& event) noexcept;
  ppbng_timing::StatusReport status(std::uint64_t current_tick) const noexcept;
  ppbng_timing::ControllerState state() const noexcept;
  bool output_gate_enabled() const noexcept;
  std::uint32_t error_flags() const noexcept;
  std::uint32_t boot_id() const noexcept;

 private:
  struct Oscillator {
    bool enabled{};
    PhysicalOutput output{};
    ppbng_timing::Channel logical_channel{};
    std::uint32_t numerator{};
    std::uint32_t denominator{1U};
    std::uint32_t width{};
    std::uint32_t phase{};
    std::uint64_t base_interval{};
    std::uint64_t interval_remainder{};
    std::uint64_t remainder_accumulator{};
    std::uint64_t next_tick{};
    std::uint64_t channel_sequence{};
  };

  void force_fault_isr(std::uint32_t flag) noexcept;
  void disable_outputs_isr() noexcept;
  bool prepare_synchronous_start_locked() noexcept;
  bool start_at_current_tick_locked(std::uint64_t current_tick) noexcept;
  bool schedule_next_isr(Oscillator& oscillator, std::uint64_t from_tick) noexcept;
  void emit_trigger_isr(Oscillator& oscillator, std::uint64_t tick) noexcept;
  bool push_event_isr(const FirmwareEvent& event) noexcept;
  ppbng_timing::Packet make_ack(const ppbng_timing::Packet& request,
                                ppbng_timing::ControllerState state,
                                const char* detail) const;
  ppbng_timing::Packet make_error(const ppbng_timing::Packet& request,
                                  std::uint16_t code, const char* detail) const;
  bool same_as_cached(const ppbng_timing::Packet& request) const noexcept;
  void cache(const ppbng_timing::Packet& request, const ppbng_timing::Packet& response);
  void configure_oscillators(const ppbng_timing::ScheduleConfig& schedule) noexcept;
  ppbng_timing::TimeLock time_quality(std::uint64_t tick) const noexcept;
  ppbng_timing::StatusReport status_locked(std::uint64_t current_tick) const noexcept;

  ITimingHardware& hardware_;
  SafetyPolicy policy_;
  std::uint32_t boot_id_{};
  std::uint32_t hardware_ticks_per_second_{};
  ppbng_timing::ControllerState state_{ppbng_timing::ControllerState::kIdle};
  std::uint32_t error_flags_{};
  bool output_gate_enabled_{};
  std::optional<ppbng_timing::ScheduleConfig> schedule_;
  std::array<Oscillator, 3U> oscillators_{};
  std::uint64_t pps_sequence_{};
  std::uint64_t last_pps_tick_{};
  bool have_pps_{};
  std::uint64_t global_event_sequence_{};
  std::uint64_t last_host_activity_tick_{};
  std::uint64_t last_observed_tick_{};
  ppbng_timing::ArmRequest arm_request_{};
  bool synchronous_start_armed_{};
  bool pps_watchdog_required_{};
  FixedEventRing<kEventRingSlots> events_;
  std::optional<ppbng_timing::Packet> cached_request_;
  std::optional<ppbng_timing::Packet> cached_response_;
};

}  // namespace ppbng_timing_firmware
