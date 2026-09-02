# Hardware validation plan

This plan is intentionally staged. Full filesystem access is not authorization to enumerate,
open, configure, trigger, move, or update any device. Each stage begins only after explicit
operator approval, and every state-changing stage requires the operator to confirm that the
robot and mount are physically safe.

Within every stage, devices advance independently. A pass for one device does not authorize or
qualify another device, and first-open tests never start more than one device. The detailed
device-specific commands, evidence, time bounds, and combination order are defined in
`device_by_device_validation_zh.md`.

## Stage 0 — software baseline (complete)

- Build all packages with hardware backends disabled.
- Run all unit/integration tests and the 7200-second virtual soak.
- Verify that production launch refuses its safe defaults.
- Compile/link the installed Spinnaker and SpecSensor SDK backends without running them, then
  restore all backend CMake switches to `OFF`.

## Stage 1 — read-only identity inventory

Requires one explicit approval covering read-only enumeration only. Do not open acquisition,
write GenICam nodes, transmit serial bytes, change network settings, or move the RSM400.

Record locally for operator review:

- RGB serial/model/MAC/IP/interface;
- A6701 serial/model/MAC/IP/interface and discoverable node-map names;
- FX10e and SWIR SpecSensor identity and SDK index;
- configured COM path plus USB VID/PID/serial for UM982, RSM400, and timing controller;
- SDK/runtime versions and architecture.

Stage 1 produces a proposed machine YAML. It does not enable `safety.configured` or
`safety.hardware_enabled`.

## Stage 2 — receive-only checks

Requires separate approval. Open one device at a time, with all trigger outputs hard-gated:

- read UM982 GGA and UNIHEADINGA bytes without ever writing to its COM handle;
- read RSM400 MCP2 telemetry without transmitting a control command;
- confirm thermal and RGB streaming only if the operator explicitly approves camera opening;
- confirm that closing an HSI session leaves its shutter closed.

Any identity mismatch stops the stage. Logs go to a disposable validation dataset, never an
operator production dataset.

## Stage 3 — electrical and timing bench validation

Requires the completed controller board, protected PPS input, four electrically independent
camera output drivers, and an oscilloscope or logic analyzer. No robot motion is needed.

- verify UM982 PPS voltage, polarity, pulse width, and selected time system;
- measure PPS-to-controller capture latency/jitter and output edge timing;
- verify each channel independently at a low rate before 120 Hz;
- test missing PPS, host loss, controller reset, queue overflow, and emergency gate-off;
- correlate UNIHEADINGA integer epochs to PPS using the reported output delay;
- keep `pps_gnss_evidence_confirmed=false` until the measurements establish the configured
  offset/skew bound.

## Stage 4 — per-camera trigger proof

Requires separate approval for trigger emission and acquisition.

- RGB: prove Line0 polarity/electrical compatibility, frame-counter behavior, raw Bayer geometry,
  and the behavior of a deliberately missed trigger;
- A6701: prove that SYNC IN provides the required exposure-start semantics (FSSI versus FSSR),
  and preserve the complete 640x513 Mono16 transport payload. Row 0 is 1280 bytes of opaque FLIR
  metadata/header; rows 1–512 are the 640x512 radiometric image;
- both HSI cameras: prove callback frame-number semantics, missed-trigger behavior, independent
  rates, shutter commands/readback, and simultaneous acquisition.

Only measured evidence may enable `association_evidence_confirmed=true` for a stream.

## Stage 5 — RSM400 control proof

Requires an operator-created exclusion zone and separate motion approval.

- confirm RS-232 versus RS-422 converter/interface and the exact MCP2 firmware/feature set;
- establish the approved `RC 1` entry/exit transaction before attempting STAB control;
- verify `ST 1`, fast level, signs, limits, telemetry rate, watchdog behavior, and stop behavior;
- verify OF002 before target roll/pitch offsets and OF005 before reset commands.

The production code deliberately does not send `RC 1` automatically while this procedure is
unresolved.

## Stage 6 — integrated low-rate trial

This stage begins only after every participating device has passed its independent D0-D5 gates.
Start with 1 Hz or lower camera triggers, short duration, stationary robot, covered/controlled
scene, and ample disk space. Validate manifests, raw payload checksums, dark workflow, warning
visibility, disarm-first stop, device dropout continuation, and storage-fault global stop before
raising HSI rates.

## Stage 7 — full-rate and two-hour acceptance

Raise one stream at a time, then run all streams. Acceptance requires bounded queues, no silent
sample loss, correct segment recovery, stable disk throughput/temperature, auditable time-quality
states, closed HSI shutters after normal stop, and a finalized recoverable dataset.
