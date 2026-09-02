# ppbng_rsm400

This package implements the receive-side MCP 2.0 parser plus a conservative command transaction layer. The source of truth is SOMAG ICD `323403-901-08/02` (47 pages).

## Safety behavior

- `CommandClientOptions::allow_control` defaults to `false`. In observe-only mode every control request fails before the transport receives a write.
- Construction of `Win32SerialTransport` is inert. Only the explicit `open_port("COM<number>")` call opens a handle. No package test calls it.
- Startup code must not call a control transaction automatically. In particular, `RC 1` is not in the allowlist because it changes control source, selects external attitude/setpoint sources, and puts the Mount in MAN mode until another mode command is received.
- Only `ST 1`, `OFR` + `OFP`, `FL`, and `RER` can be produced by the public control model. Unknown enum values fail closed.
- OF002 must be explicitly confirmed before `OFR`/`OFP`; OF005 must be explicitly confirmed before `RER`. Unknown and unavailable are distinct failures.

## Confirmed protocol facts

- Serial transport: 115200 baud, 8 data bits, no parity, one stop bit, no hardware/software flow control (ICD 4.3).
- `ST 1`: activates STAB mode for horizon axes and leaves the drift axis unchanged (ICD 5.4.1).
- `FL`: argument-free fast-level trigger during STAB mode (ICD 5.4.1).
- `OFR` and `OFP`: OF002 leveling offsets, `-3000..3000`, unit 0.01 degree (ICD 5.4.10). Zero deactivates the corresponding offset.
- `RER`: resets errors except built-in-test errors and requires OF005 (ICD 5.4.3).
- An outgoing connection byte `H` requests an acknowledgement. Incoming ACKN/CMD ERROR/CHKS ERROR/RETR bits follow ICD 5.3.3. Retransmission reuses the exact frame, at most twice.

MCP 2.0 contains no transaction sequence or echoed command identifier. `CommandClient` therefore serializes all commands, permits one in-flight transaction, and assigns a local monotonic sequence for logs. A data-bearing ACK is rejected as uncorrelatable; non-ACK telemetry received before the empty ACK is returned separately.

## Deliberately unresolved

- The required operational procedure for entering remote control remains unresolved. The ICD states that `ST`, `FL`, and related controls require `RC 1`; however `RC 1` also sets `SCA 1`/`SCD 1` and initially causes MAN mode. It is intentionally not sent or exposed until an operator-approved startup transaction is designed and hardware behavior is validated.
- The ICD does not define an on-wire sequence number or ACK command echo. Correlation beyond one serialized in-flight command cannot be implemented without guessing.
- `RER` cannot reset built-in-test errors. No broader reset command is exposed.
- OF002/OF005 discovery (`FAO`) is not automatically transmitted by this layer. Feature availability must come from a separately approved/read-only preflight and is unknown by default.
- The USB converter's electrical standard (RS232 versus RS422), exact COM identity, RSM firmware/protocol ID, unlocked feature mask, and physical sign convention of positive roll/pitch require hardware inspection/validation before enabling control.

All unit tests use an in-memory fake transport and cover fragmented I/O, timeout, command error, ACK mismatch, retransmission, feature gates, unsolicited telemetry, and zero writes in observe-only mode.

## ROS production node

`rsm400_production_node` is inert when constructed. `hardware_enabled` defaults to false;
`~/arm` validates only and keeps the COM port closed, and only a later explicit `~/start`
opens the configured port. `allow_control` separately defaults to false, so observation cannot
silently become control. No command is sent at startup. Telemetry is published as validated
`ppbng_interfaces/RsmTelemetry` with a host monotonic receipt timestamp and the original MCP2
frame. The operational `RC 1` prerequisite remains unresolved and is never emitted.

Production readiness is also fail-closed. `require_ready_telemetry` and
`stab_motion_status_confirmed` must both be explicitly true, and
`expected_stab_motion_status` must contain the Stage 5-confirmed digit for the installed firmware.
After opening, startup only listens: it never sends `RC 1`, `ST`, or another command. Within the
bounded `ready_timeout_ms`, checksum-valid telemetry must provide finite roll and pitch plus an
`MS` general status whose error level is no greater than `maximum_ready_error_level` and whose
motion status exactly equals the confirmed STAB digit. Otherwise the port and writer are closed and
the start service fails. The example machine configuration intentionally leaves this evidence
unconfirmed, so production launch refuses it.
