# UNO R4 WiFi — A6701 temporary external-trigger test

This sketch is a temporary low-rate bench source, not the final PPB-NG timing
controller. It drives D12 active-high at 2 Hz with a 1 ms pulse only after receiving
`START` over USB serial at 115200 baud. Reset, disconnect, `STOP`, and malformed
overlength commands leave D12 low.

## Electrical hold point

Do not issue `START` until the actual interface has been identified. The available
A6701 documentation confirms the `SYNC IN` voltage thresholds and maximum voltage but
does not settle input impedance/termination or isolation for this exact setup. A direct
UNO GPIO must not drive a 50-ohm terminated input. The preferred production interface
is a buffered or isolated 5 V output with measured loaded voltage/current and clamping.

Use the A6701 rear-panel `SYNC IN`, never the nearby `TRIGGER IN`. Signal and return
must both be connected according to the verified interface: BNC center is signal and
BNC shield is return.

## Upload

1. In Arduino IDE select **Arduino UNO R4 WiFi** and the board's USB serial port.
2. Open `arduino_uno_r4_a6701_trigger_test.ino` and upload it.
3. Open Serial Monitor at 115200 baud and select a newline-ending mode.
4. Confirm `PPBNG_A6701_TRIGGER_TEST_READY,output=LOW` appears.
5. With the camera still configured for internal frame sync, send `LOADTEST`.
6. Do not send `START` until `LOADTEST` has reported `high_samples=32/32` and
   the software-side camera preparation is complete.

Commands are `LOADTEST`, `START`, `STOP`, `STATUS`, and `KEEPALIVE`. `LOADTEST` temporarily uses
the UNO's weak internal pull-up and samples the connected line; it never strongly
drives the unknown load and always restores D12 to output-low. A result below 32/32
is a conservative electrical-test failure and external pulses must not be started.

The first pulse occurs one second after
`START`, allowing an already configured camera receiver time to settle.

While output is running, the host must send `KEEPALIVE` at least once every
three seconds. If heartbeats stop, the firmware automatically restores D12 low
and reports `STOPPED,host_watchdog_timeout`. This protects the camera if the
focus-test terminal or host process exits unexpectedly.
