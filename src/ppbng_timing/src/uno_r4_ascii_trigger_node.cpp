#include "ppbng_timing/timing_event_writer.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/msg/trigger_event.hpp>
#include <ppbng_interfaces/srv/prepare_device.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace ppbng_timing {
namespace {

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

class SerialPort {
 public:
  ~SerialPort() { close(); }
  bool open(const std::string& name, std::uint32_t baud, std::string& error) {
#ifdef _WIN32
    close();
    const std::string path = name.rfind("\\\\.\\", 0) == 0 ? name : "\\\\.\\" + name;
    handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
        OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
      error = "CreateFile failed for " + name + ": " + std::to_string(GetLastError());
      return false;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) {
      error = "GetCommState failed: " + std::to_string(GetLastError()); close(); return false;
    }
    dcb.BaudRate = baud; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE; dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(handle_, &dcb)) {
      error = "SetCommState failed: " + std::to_string(GetLastError()); close(); return false;
    }
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 500;
    if (!SetCommTimeouts(handle_, &timeouts)) {
      error = "SetCommTimeouts failed: " + std::to_string(GetLastError()); close(); return false;
    }
    PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
#else
    (void)name; (void)baud; error = "UNO R4 serial transport is Windows-only"; return false;
#endif
  }
  void close() {
#ifdef _WIN32
    if (handle_ != INVALID_HANDLE_VALUE) { CloseHandle(handle_); handle_ = INVALID_HANDLE_VALUE; }
#endif
  }
  bool is_open() const {
#ifdef _WIN32
    return handle_ != INVALID_HANDLE_VALUE;
#else
    return false;
#endif
  }
  bool write_line(const std::string& line, std::string& error) {
#ifdef _WIN32
    const std::string wire = line + "\n";
    DWORD written = 0;
    if (!WriteFile(handle_, wire.data(), static_cast<DWORD>(wire.size()), &written, nullptr) ||
        written != wire.size()) {
      error = "serial write failed: " + std::to_string(GetLastError()); return false;
    }
    return true;
#else
    (void)line; error = "unsupported"; return false;
#endif
  }
  bool read_available(std::string& output, std::string& error) {
#ifdef _WIN32
    DWORD queued = 0; COMSTAT stat{}; DWORD flags = 0;
    if (!ClearCommError(handle_, &flags, &stat)) {
      error = "ClearCommError failed: " + std::to_string(GetLastError()); return false;
    }
    if (flags != 0) {
      error = "serial line error flags: " + std::to_string(flags); return false;
    }
    queued = stat.cbInQue;
    while (queued > 0) {
      char buffer[512]; const DWORD wanted = (std::min)(queued, static_cast<DWORD>(sizeof(buffer)));
      DWORD received = 0;
      if (!ReadFile(handle_, buffer, wanted, &received, nullptr)) {
        error = "serial read failed: " + std::to_string(GetLastError()); return false;
      }
      output.append(buffer, buffer + received);
      queued -= received;
      if (received == 0) break;
    }
    return true;
#else
    error = "unsupported"; return false;
#endif
  }
 private:
#ifdef _WIN32
  HANDLE handle_{INVALID_HANDLE_VALUE};
#endif
};

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> result; std::stringstream stream(text); std::string part;
  while (std::getline(stream, part, delimiter)) result.push_back(part);
  return result;
}

}  // namespace

class UnoR4AsciiTriggerNode final : public rclcpp::Node {
 public:
  UnoR4AsciiTriggerNode() : Node("uno_r4_ascii_trigger") {
    hardware_enabled_ = declare_parameter<bool>("hardware_enabled", false);
    com_path_ = declare_parameter<std::string>("com_path", "");
    baud_rate_ = static_cast<std::uint32_t>(declare_parameter<std::int64_t>("baud_rate", 115200));
    ticks_per_second_ = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>("ticks_per_second", 1000000));
    rate_numerator_ = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>("rate_numerator_hz", 2));
    rate_denominator_ = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>("rate_denominator", 1));
    pulse_width_ticks_ = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>("pulse_width_ticks", 1000));
    device_id_ = declare_parameter<std::string>("device_id", "uno_r4_trigger");
    flush_every_events_ = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>("flush_every_events", 32));
    binding_ = std::make_unique<TimingSessionBinding>(
        declare_parameter<std::string>("allowed_output_root", ""));

    trigger_pub_ = create_publisher<ppbng_interfaces::msg::TriggerEvent>("trigger_event", 256);
    status_pub_ = create_publisher<ppbng_interfaces::msg::DeviceStatus>("device_status", 10);
    fault_pub_ = create_publisher<ppbng_interfaces::msg::FaultEvent>("fault_event", 10);
    prepare_srv_ = create_service<ppbng_interfaces::srv::PrepareDevice>("prepare",
      [this](const std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Request> request,
             std::shared_ptr<ppbng_interfaces::srv::PrepareDevice::Response> response) {
        const auto r = binding_->prepare(request->request_id, request->session_id,
            request->session_directory, state_ != State::inert);
        response->accepted = r.accepted; response->duplicate_request = r.duplicate;
        response->message = r.detail; publish_status(r.detail);
      });
    arm_srv_ = service("arm", [this] { return arm(); });
    start_srv_ = service("start", [this] { return start(); });
    arm_immediate_srv_ = service("arm_immediate", [this] { return arm_immediate(); });
    disarm_srv_ = service("disarm_keep_config", [this] { return disarm(); });
    status_srv_ = service("status", [this] {
      return Result{state_ == State::running,
          state_ == State::running ? "RUNNING pulses=" + std::to_string(pulses_) :
          "not running; state=" + std::to_string(static_cast<int>(state_))};
    });
    stop_srv_ = service("stop", [this] { return stop(); });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { poll(); });
    publish_status("inert; serial port has not been opened");
  }

  ~UnoR4AsciiTriggerNode() override {
    if (serial_.is_open()) { std::string ignored; serial_.write_line("STOP", ignored); }
    serial_.close();
  }

 private:
  enum class State { inert, armed, started, running, fault };
  struct Result { bool success; std::string detail; };
  using TriggerSrv = std_srvs::srv::Trigger;

  rclcpp::Service<TriggerSrv>::SharedPtr service(
      const std::string& name, std::function<Result()> operation) {
    return create_service<TriggerSrv>(name, [this, operation = std::move(operation)](
        const std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> response) {
      const auto result = operation(); response->success = result.success;
      response->message = result.detail; publish_status(result.detail);
    });
  }

  Result arm() {
    if (state_ == State::armed) return {true, "already armed; outputs remain LOW"};
    if (state_ != State::inert)
      return {false, "arm is allowed only from inert state"};
    if (!binding_->bound()) return {false, "PrepareDevice is required before arm"};
    if (!hardware_enabled_) return {false, "hardware_enabled is false"};
    if (com_path_.empty() || com_path_.find("TO_BE_CONFIRMED") != std::string::npos)
      return {false, "timing.port must be set to the connected UNO R4 COM port"};
    if (baud_rate_ != 115200 || ticks_per_second_ != 1000000 || rate_numerator_ != 2 ||
        rate_denominator_ != 1 || pulse_width_ticks_ != 1000 || flush_every_events_ == 0)
      return {false, "configuration does not match validated UNO firmware (115200 baud, 2 Hz, 1 ms)"};
    state_ = State::armed;
    return {true, "armed without opening the COM port; outputs remain LOW"};
  }

  Result start() {
    if (state_ != State::armed) return {false, "start requires a successful arm"};
    auto created = TimingEventWriter::create(binding_->directory(), "segments/uno_r4_timing_events.bin");
    if (!created.first.ok()) return fault("timing log create failed: " + created.first.detail);
    writer_ = std::move(created.second);
    std::string error;
    if (!serial_.open(com_path_, baud_rate_, error)) { writer_.reset(); return fault(error); }
    std::this_thread::sleep_for(std::chrono::milliseconds(2200));
    input_.clear();
    if (!serial_.read_available(input_, error)) return fault_and_close(error);
    startup_drain_ = true;
    consume_lines();
    startup_drain_ = false;
    if (!command_and_wait("STOP", "STOPPED", 1500, error)) return fault_and_close(error);
    if (!command_and_wait("STATUS", "STOPPED,rgb_pin=11,thermal_pin=12,rate_hz=2,pulse_width_us=1000",
                          1500, error)) return fault_and_close("UNO firmware identity mismatch: " + error);
    state_ = State::started; last_keepalive_ns_ = monotonic_ns();
    return {true, "UNO R4 verified and held STOPPED; D11/D12 remain LOW"};
  }

  Result arm_immediate() {
    if (state_ != State::started) return {false, "arm_immediate requires started STOPPED controller"};
    std::string error;
    if (!command_and_wait("START", "ARMED,first_pulse_in_us=1000000,rate_hz=2,pulse_width_us=1000",
                          1500, error)) return fault_and_close(error);
    state_ = State::running; last_keepalive_ns_ = monotonic_ns();
    return {true, "shared D11/D12 trigger armed; first pulse follows after 1 second"};
  }

  Result disarm() {
    if (!serial_.is_open()) return {true, "controller already closed"};
    std::string error;
    stop_command_pending_ = true;
    const bool stopped = command_and_wait("STOP", "STOPPED", 1500, error);
    stop_command_pending_ = false;
    if (!stopped) return fault_and_close(error);
    state_ = State::started;
    if (writer_) { const auto f = writer_->flush(); if (!f.ok()) return fault_and_close(f.detail); }
    return {true, "trigger outputs confirmed LOW; serial configuration retained"};
  }

  Result stop() {
    if (serial_.is_open()) {
      const auto result = disarm();
      if (!result.success) return result;
    }
    if (writer_) {
      const auto flushed = writer_->flush();
      if (!flushed.ok()) return fault_and_close("timing log final flush failed: " + flushed.detail);
    }
    writer_.reset(); serial_.close(); state_ = State::inert;
    return {true, "UNO stopped, outputs LOW, COM port released"};
  }

  Result fault(const std::string& detail) {
    state_ = State::fault; publish_fault(detail); return {false, detail};
  }
  Result fault_and_close(const std::string& detail) {
    if (serial_.is_open()) { std::string ignored; serial_.write_line("STOP", ignored); }
    serial_.close(); writer_.reset(); return fault(detail);
  }

  bool command_and_wait(const std::string& command, const std::string& expected,
                        int timeout_ms, std::string& error) {
    if (!serial_.write_line(command, error)) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      std::string bytes;
      if (!serial_.read_available(bytes, error)) return false;
      input_ += bytes;
      auto lines = consume_lines();
      for (const auto& line : lines) if (line.rfind(expected, 0) == 0) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    error = "timeout waiting for '" + expected + "' after " + command; return false;
  }

  std::vector<std::string> consume_lines() {
    std::vector<std::string> complete;
    for (;;) {
      const auto pos = input_.find_first_of("\r\n");
      if (pos == std::string::npos) break;
      std::string line = input_.substr(0, pos); input_.erase(0, pos + 1);
      while (!input_.empty() && (input_[0] == '\r' || input_[0] == '\n')) input_.erase(0, 1);
      if (!line.empty()) { complete.push_back(line); process_line(line); }
    }
    return complete;
  }

  void poll() {
    if (!serial_.is_open()) return;
    const auto now_ns = monotonic_ns();
    if (state_ == State::running && now_ns - last_keepalive_ns_ >= 800000000ULL) {
      std::string error;
      if (!serial_.write_line("KEEPALIVE", error)) { fault_and_close(error); return; }
      last_keepalive_ns_ = now_ns;
    }
    std::string bytes, error;
    if (!serial_.read_available(bytes, error)) { fault_and_close(error); return; }
    input_ += bytes;
    if (input_.size() > 4096) { fault_and_close("serial input exceeded bounded line buffer"); return; }
    consume_lines();
  }

  void process_line(const std::string& line) {
    if (line.rfind("PULSE,", 0) == 0) {
      if (state_ != State::running) {
        if (!startup_drain_)
          fault_and_close("unexpected PULSE while trigger controller is not running");
        return;
      }
      const auto fields = split(line, ',');
      if (fields.size() != 3) { fault_and_close("malformed PULSE line: " + line); return; }
      try {
        std::size_t sequence_end = 0, tick_end = 0;
        const auto sequence = std::stoull(fields[1], &sequence_end);
        const auto tick_value = std::stoull(fields[2], &tick_end);
        if (sequence_end != fields[1].size() || tick_end != fields[2].size() ||
            tick_value > 0xffffffffULL)
          throw std::invalid_argument("non-canonical numeric field");
        const auto raw = static_cast<std::uint32_t>(tick_value);
        if (have_pulse_ && sequence != last_pulse_sequence_ + 1)
          throw std::runtime_error("PULSE sequence is not contiguous");
        if (have_tick_ && raw < last_raw_tick_ && last_raw_tick_ - raw > 0x80000000U)
          tick_epoch_ += (1ULL << 32);
        const auto extended_tick = tick_epoch_ + raw;
        if (have_tick_ && extended_tick <= last_extended_tick_)
          throw std::runtime_error("PULSE tick did not advance");
        have_tick_ = true; have_pulse_ = true; last_raw_tick_ = raw;
        last_extended_tick_ = extended_tick; last_pulse_sequence_ = sequence;
        publish_pulse(sequence, extended_tick);
      } catch (const std::exception& error) {
        fault_and_close("invalid PULSE evidence: " + line + "; " + error.what());
      }
    } else if (state_ == State::running && !stop_command_pending_ &&
               line.rfind("STOPPED,", 0) == 0) {
      fault_and_close("UNO stopped unexpectedly while armed: " + line);
    }
  }

  void publish_pulse(std::uint64_t sequence, std::uint64_t tick) {
    for (const auto& entry : {std::pair{Channel::kRgb, std::string("rgb")},
                              std::pair{Channel::kThermal, std::string("thermal")}}) {
      TriggerEvent stored{}; stored.event_sequence = ++event_sequence_;
      stored.channel = entry.first; stored.channel_sequence = sequence;
      stored.offset_ticks = tick; stored.ticks_per_second = ticks_per_second_;
      stored.lock = TimeLock::kUnsynced;
      const auto status = writer_->append_trigger(stored, monotonic_ns());
      if (!status.ok()) { fault_and_close("timing log write failed: " + status.detail); return; }
      ppbng_interfaces::msg::TriggerEvent message;
      message.header.stamp = now(); message.channel = entry.second;
      message.channel_sequence = sequence; message.offset_ticks = tick;
      message.ticks_per_second = ticks_per_second_;
      message.time_quality.hardware_tick = tick;
      message.time_quality.uncertainty_ns = 250000000ULL;
      message.time_quality.status = ppbng_interfaces::msg::TimeQuality::UNSYNCED;
      message.time_quality.detail = "shared atomic UNO R4 pulse; UTC/PPS not connected";
      trigger_pub_->publish(message);
    }
    pulses_ = sequence;
    if (++events_since_flush_ >= flush_every_events_) {
      const auto status = writer_->flush(); events_since_flush_ = 0;
      if (!status.ok()) fault_and_close("timing log flush failed: " + status.detail);
    }
  }

  void publish_status(const std::string& detail) {
    ppbng_interfaces::msg::DeviceStatus message; message.status_time = now();
    message.status_host_monotonic_ns = monotonic_ns();
    message.session_id = binding_->bound() ? binding_->session_id() : "";
    message.device_id = device_id_; message.required = true;
    message.lifecycle_state = state_ == State::running ? 5U :
        (state_ == State::started ? 3U : (state_ == State::armed ? 4U : 0U));
    message.health = state_ == State::fault ? 3U : (state_ == State::inert ? 0U : 1U);
    message.last_sample_valid = pulses_ > 0; message.last_sample_sequence = pulses_;
    message.samples_received = pulses_; message.detail = detail; status_pub_->publish(message);
  }
  void publish_fault(const std::string& detail) {
    ppbng_interfaces::msg::FaultEvent message;
    message.event_id = device_id_ + "-" + std::to_string(++fault_count_);
    message.session_id = binding_->bound() ? binding_->session_id() : "";
    message.source_id = device_id_; message.fault_code = "UNO_R4_TRIGGER_FATAL";
    message.severity = ppbng_interfaces::msg::FaultEvent::SEVERITY_FATAL;
    message.first_host_monotonic_ns = monotonic_ns();
    message.last_host_monotonic_ns = message.first_host_monotonic_ns;
    message.occurrence_count = 1; message.latched = true; message.causes_global_stop = true;
    message.detail = detail; fault_pub_->publish(message);
    RCLCPP_FATAL(get_logger(), "%s", detail.c_str());
  }

  bool hardware_enabled_{}; std::string com_path_, device_id_, input_;
  std::uint32_t baud_rate_{}, ticks_per_second_{}, rate_numerator_{}, rate_denominator_{};
  std::uint32_t pulse_width_ticks_{}, flush_every_events_{}, events_since_flush_{};
  State state_{State::inert}; SerialPort serial_;
  std::unique_ptr<TimingSessionBinding> binding_; std::unique_ptr<TimingEventWriter> writer_;
  std::uint64_t last_keepalive_ns_{}, event_sequence_{}, pulses_{}, tick_epoch_{}, fault_count_{};
  std::uint64_t last_extended_tick_{}, last_pulse_sequence_{};
  std::uint32_t last_raw_tick_{};
  bool have_tick_{}, have_pulse_{}, stop_command_pending_{}, startup_drain_{};
  rclcpp::Publisher<ppbng_interfaces::msg::TriggerEvent>::SharedPtr trigger_pub_;
  rclcpp::Publisher<ppbng_interfaces::msg::DeviceStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Service<ppbng_interfaces::srv::PrepareDevice>::SharedPtr prepare_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr arm_srv_, start_srv_, arm_immediate_srv_, disarm_srv_,
      status_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ppbng_timing

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppbng_timing::UnoR4AsciiTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
