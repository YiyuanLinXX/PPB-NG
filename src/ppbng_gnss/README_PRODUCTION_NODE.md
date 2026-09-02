# UM982 receive-only ROS 2 node

`um982_production_node` is inert by default. Construction creates publishers, services, and a
timer only; it does not create a serial receiver or open a COM port.

The activation sequence is deliberately two-step:

1. Set `hardware_enabled:=true` and provide an explicit `COMn` path, baud rate, and bounded buffer
   sizes. Calling `arm` validates these values without touching hardware.
2. Call `start`. Only this service constructs the receive-only source and opens the port.

`stop` cancels any pending read, closes the port, destroys the receiver, and returns the gate to
inert state. A failed open latches the activation gate in fault until `stop` is called.

The Windows transport requests exactly `GENERIC_READ` in `CreateFileW`. There is no write method in
`IReceiveOnlyByteSource`, `WindowsReceiveOnlySerial`, `Um982Receiver`, or the ROS node. The package
contains no UM982 configuration command, NTRIP client, correction-data sender, serial `WriteFile`,
or serial `GENERIC_WRITE` request. Host-side DCB settings only configure how Windows receives the existing
TTL-to-USB byte stream; they do not transmit bytes to UM982.

`allowed_output_root` must name the configured dataset root; construction remains session-less.
The `prepare` service binds a request/session ID only to an already-created strict child containing
its existing `segments` directory. It creates neither root nor session. Request history is retained
for full-lifetime idempotency, conflicting reuse is rejected, and prepare after start is refused.
`start` exclusively creates append-only raw, GGA, and
UNIHEADINGA JSONL files. Every framed line, including unknown or invalid recognized messages, is
retained with host system time, host monotonic time, connection epoch, and parse status. Valid GGA
and UNIHEADINGA fields are additionally written to structured logs. `flush_every_sentences`
controls periodic flush and atomic checkpoint updates. Existing targets or storage failures are
fatal to this node and close the receive-only port. No credentials are accepted or stored.

The node publishes checksum/CRC-validated GGA and UNIHEADINGA sentences as strings plus structured
device health counters. Downstream association remains responsible for converting those validated
protocol records into the project's canonical GNSS/time data model.

Hardware validation still requires user approval to confirm the deployed COM number, baud rate,
USB adapter reconnect identity, actual 10 Hz message mix, latency under load, and unplug/replug
behavior. None of those items was tested while implementing this node.

Structured faults use the relative `fault_event` topic. GNSS log create/write/flush/close failures
set `causes_global_stop=true`; serial disconnect and receive transport faults set it to `false`,
allowing unrelated sensors to continue while the operator is warned. Disconnect/I/O errors and a
configurable run of finite read timeouts use bounded, stop-interruptible exponential backoff. Every
attempt reopens only the frozen configured COM path and baud settings, never requests write access,
and never transmits. Before an attempt the current JSONL/checkpoint set is finalized; success resets
the decoder/time epoch and exclusively creates a suffixed segment. Exhaustion latches a persistent
prominent fault. A COM path alone cannot cryptographically bind the USB adapter: VID/PID/USB serial
identity remains an explicit deployment validation gap. Hardware enable remains off by default.
