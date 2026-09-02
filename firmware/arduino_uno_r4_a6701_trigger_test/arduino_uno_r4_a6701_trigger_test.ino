// PPB-NG temporary FLIR A6701 external-sync bench source.
// Target: Arduino UNO R4 WiFi. Output: D12, active high, 2 Hz, 1 ms.
//
// SAFETY: The output remains LOW after reset. It starts only after the exact
// serial command START. Before START, verify the electrical driver/termination,
// connect signal reference correctly, and use the camera SYNC IN (not TRIGGER IN).

#include <Arduino.h>

namespace
{
constexpr uint8_t kThermalSyncPin = 12;
constexpr uint32_t kPeriodUs = 500000U;   // 2 Hz
constexpr uint32_t kPulseWidthUs = 1000U; // 1 ms high
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

void stop_output(const __FlashStringHelper * reason)
{
  digitalWrite(kThermalSyncPin, LOW);
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
    digitalWrite(kThermalSyncPin, LOW);
    pulse_high = false;
    pulse_count = 0;
    next_rise_us = micros() + kStartDelayUs;
    last_keepalive_ms = millis();
    running = true;
    Serial.println(F("ARMED,first_pulse_in_us=1000000,rate_hz=2,pulse_width_us=1000,pin=12,watchdog_ms=3000"));
  } else if (input == "KEEPALIVE") {
    if (running) last_keepalive_ms = millis();
  } else if (input == "STOP") {
    stop_output(F("operator_command"));
  } else if (input == "STATUS") {
    Serial.print(running ? F("RUNNING") : F("STOPPED"));
    Serial.print(F(",pin=12,rate_hz=2,pulse_width_us=1000,pulse_count="));
    Serial.print(pulse_count);
    Serial.print(F(",watchdog_ms="));
    Serial.println(kHostWatchdogTimeoutMs);
  } else if (input == "LOADTEST") {
    if (running) {
      Serial.println(F("ERROR,STOP_before_LOADTEST"));
      return;
    }
    // A weak internal pull-up probes for a low-impedance/terminated input
    // without strongly driving the unknown load. The camera must remain in
    // Internal frame-sync mode during this test.
    digitalWrite(kThermalSyncPin, LOW);
    pinMode(kThermalSyncPin, INPUT_PULLUP);
    delay(100);
    uint8_t high_samples = 0;
    for (uint8_t i = 0; i < 32; ++i) {
      if (digitalRead(kThermalSyncPin) == HIGH) ++high_samples;
      delay(2);
    }
    pinMode(kThermalSyncPin, OUTPUT);
    digitalWrite(kThermalSyncPin, LOW);
    Serial.print(F("LOADTEST,high_samples="));
    Serial.print(high_samples);
    Serial.println(F("/32,output_restored_LOW"));
  } else if (input.length() != 0) {
    Serial.println(F("ERROR,commands=LOADTEST|START|STOP|STATUS|KEEPALIVE"));
  }
}
}  // namespace

void setup()
{
  pinMode(kThermalSyncPin, OUTPUT);
  digitalWrite(kThermalSyncPin, LOW);
  Serial.begin(115200);
  const uint32_t serial_wait_started = millis();
  while (!Serial && millis() - serial_wait_started < 3000U) {}
  Serial.println(F("PPBNG_A6701_TRIGGER_TEST_READY,output=LOW"));
  Serial.println(F("Commands: LOADTEST, START, STOP, STATUS, KEEPALIVE"));
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
    digitalWrite(kThermalSyncPin, HIGH);
    pulse_high = true;
    next_rise_us += kPeriodUs;
  } else if (pulse_high && reached(now, rise_us + kPulseWidthUs)) {
    digitalWrite(kThermalSyncPin, LOW);
    pulse_high = false;
    ++pulse_count;
    Serial.print(F("PULSE,"));
    Serial.print(pulse_count);
    Serial.print(',');
    Serial.println(rise_us);
  }
}
