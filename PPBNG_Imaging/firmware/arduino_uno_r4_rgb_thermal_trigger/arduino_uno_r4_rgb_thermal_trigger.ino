// PPB-NG synchronized RGB + A6701 trigger source.
// Target: Arduino UNO R4 WiFi.
//   D11 -> FLIR Blackfly S Line0 / OPTOIN
//   D12 -> FLIR A6701 SYNC IN
// Both outputs are active high, 2 Hz, 1 ms, and are changed by one masked
// Port 4 write so the MCU does not introduce digitalWrite-to-digitalWrite skew.
//
// SAFETY: Both outputs remain LOW after reset and only start after the exact
// serial command START. Missing host KEEPALIVE commands stop both outputs.

#include <Arduino.h>

namespace
{
constexpr uint8_t kRgbTriggerPin = 11;      // P411
constexpr uint8_t kThermalSyncPin = 12;     // P410
constexpr uint16_t kRgbPortMask = 1U << 11;
constexpr uint16_t kThermalPortMask = 1U << 10;
constexpr uint16_t kTriggerPortMask = kRgbPortMask | kThermalPortMask;
constexpr uint32_t kPeriodUs = 500000U;     // 2 Hz
constexpr uint32_t kPulseWidthUs = 1000U;   // 1 ms high
constexpr uint32_t kStartDelayUs = 1000000U;
constexpr uint32_t kHostWatchdogTimeoutMs = 3000U;

bool running = false;
bool pulse_high = false;
uint32_t next_rise_us = 0;
uint32_t rise_us = 0;
uint64_t pulse_count = 0;
uint32_t last_keepalive_ms = 0;
String command;

bool reached(uint32_t now, uint32_t deadline)
{
  return static_cast<int32_t>(now - deadline) >= 0;
}

bool write_trigger_pair(bool high)
{
  const ioport_size_t value = high ? kTriggerPortMask : 0U;
  return R_IOPORT_PortWrite(
    &g_ioport_ctrl, BSP_IO_PORT_04, value, kTriggerPortMask) == FSP_SUCCESS;
}

void stop_output(const __FlashStringHelper * reason)
{
  write_trigger_pair(false);
  pulse_high = false;
  running = false;
  Serial.print(F("STOPPED,"));
  Serial.println(reason);
}

void process_command(String input)
{
  input.trim();
  input.toUpperCase();
  if (input == "START") {
    if (running) {
      Serial.println(F("ALREADY_RUNNING"));
      return;
    }
    if (!write_trigger_pair(false)) {
      stop_output(F("port_write_failed"));
      return;
    }
    pulse_high = false;
    pulse_count = 0;
    next_rise_us = micros() + kStartDelayUs;
    last_keepalive_ms = millis();
    running = true;
    Serial.println(F("ARMED,first_pulse_in_us=1000000,rate_hz=2,pulse_width_us=1000,rgb_pin=11,thermal_pin=12,edge_write=atomic_port4,watchdog_ms=3000"));
  } else if (input == "KEEPALIVE") {
    if (running) last_keepalive_ms = millis();
  } else if (input == "STOP") {
    stop_output(F("operator_command"));
  } else if (input == "STATUS") {
    Serial.print(running ? F("RUNNING") : F("STOPPED"));
    Serial.print(F(",rgb_pin=11,thermal_pin=12,rate_hz=2,pulse_width_us=1000,pulse_count="));
    Serial.print(pulse_count);
    Serial.print(F(",watchdog_ms="));
    Serial.println(kHostWatchdogTimeoutMs);
  } else if (input.length() != 0) {
    Serial.println(F("ERROR,commands=START|STOP|STATUS|KEEPALIVE"));
  }
}
}  // namespace

void setup()
{
  pinMode(kRgbTriggerPin, OUTPUT);
  pinMode(kThermalSyncPin, OUTPUT);
  write_trigger_pair(false);
  Serial.begin(115200);
  const uint32_t serial_wait_started = millis();
  while (!Serial && millis() - serial_wait_started < 3000U) {}
  Serial.println(F("PPBNG_RGB_THERMAL_TRIGGER_READY,outputs=LOW,rgb_pin=11,thermal_pin=12,edge_write=atomic_port4"));
  Serial.println(F("Commands: START, STOP, STATUS, KEEPALIVE"));
}

void loop()
{
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (command.length() != 0) {
        process_command(command);
        command = "";
      }
    } else if (command.length() < 32U) {
      command += c;
    } else {
      command = "";
      stop_output(F("serial_command_too_long"));
    }
  }

  if (!running) return;
  if (millis() - last_keepalive_ms > kHostWatchdogTimeoutMs) {
    stop_output(F("host_watchdog_timeout"));
    return;
  }
  const uint32_t now = micros();
  if (!pulse_high && reached(now, next_rise_us)) {
    rise_us = now;
    if (!write_trigger_pair(true)) {
      stop_output(F("port_write_failed"));
      return;
    }
    pulse_high = true;
    next_rise_us += kPeriodUs;
  } else if (pulse_high && reached(now, rise_us + kPulseWidthUs)) {
    if (!write_trigger_pair(false)) {
      stop_output(F("port_write_failed"));
      return;
    }
    pulse_high = false;
    ++pulse_count;
    Serial.print(F("PULSE,"));
    Serial.print(pulse_count);
    Serial.print(',');
    Serial.println(rise_us);
  }
}
