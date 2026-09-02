# PPBNG acquisition architecture

## 1. Scope

The Windows industrial PC owns sensor acquisition, stabilization-mount monitoring and
control, time synchronization, and data recording. A separate Raspberry Pi controls the
robot base and has independent GNSS. The two computers are not required to communicate.

The system is command-line only. One ROS 2 launch command starts the software; ROS
services start and stop a dataset. Device power remains manual.

Image correction, FFC, georeferencing to each optical center, spectral normalization,
and use of the in-view Spectralon bar are offline responsibilities.

## 2. Hardware ownership

| Device | Connection | Acquisition | Mounting |
| --- | --- | --- | --- |
| Specim FX10e | GigE / LUMO SDK | Continuous line scan, external trigger | RSM400 |
| Specim SWIR | Camera Link / NI / LUMO SDK | Continuous line scan, external trigger | RSM400 |
| BFS-U3-122S6C-C | USB3 / Spinnaker 4.4 C++ | 2 Hz snapshot, external trigger | Robot frame |
| FLIR A6701 MWIR | Dedicated GigE / Spinnaker 4.4 C++ | 2 Hz snapshot, external trigger | Robot frame |
| RSM400 | USB serial bridge, MCP 2.0 | Continuous telemetry and commands | Robot frame |
| UM982 | TXD+GND to TTL-USB, read-only | GGA + UNIHEADINGA at 10 Hz | Robot frame |
| Timing controller | USB + electrical I/O | PPS capture and trigger generation | Robot frame |

FX10e and A6701 must use independent Ethernet adapters. The ordinary network connection
must remain available independently.

## 3. Package boundaries

- `ppbng_interfaces`: messages and services shared by all packages.
- `ppbng_core`: hardware-independent time, interpolation, session, and file-integrity logic.
- `ppbng_hsi`: reusable LUMO-based dual-HSI acquisition and ENVI writer.
- `ppbng_rgb`: BFS-U3 Spinnaker adapter and raw Bayer writer.
- `ppbng_thermal`: independently supervised A6701 Spinnaker/GenICam adapter and
  radiometric-count writer.
- `ppbng_gnss`: read-only UM982 parser, validation, and interpolation source.
- `ppbng_rsm400`: MCP 2.0 framing, telemetry, status, offsets, and fault reset.
- `ppbng_timing`: trigger-controller protocol and PPS-lock monitoring.
- `ppbng_orchestrator`: task state machine and cross-device failure policy.
- `ppbng_bringup`: launch files and non-secret configuration.

Vendor SDK objects remain behind adapter interfaces. ROS nodes stay thin and do not own
file-format or device-protocol algorithms.

### 3.1 Thermal clean-room policy

The A6701 implementation is written from the camera, GenICam, GigE Vision, and Spinnaker
documentation. Legacy thermal code may be inspected to identify historical symptoms or
known device identity only; its connection, configuration, acquisition, and recovery logic
is not copied into the production driver.

Thermal runs in a separate ROS process from RGB so a blocked SDK call, access conflict, or
reconnect does not take down the RGB path. Its adapter has an explicit lifecycle:

`DISCONNECTED -> DISCOVERED -> OPEN -> CONFIGURED -> ARMED -> STREAMING`

Failures transition through `RECOVERING` and fully release stale stream and camera handles
before rediscovery. Recovery is bounded, visible in diagnostics, and starts a new data
segment; it never silently continues an old segment.

Every configurable GenICam node is checked for existence, availability, readability, and
writability. Critical writes are immediately read back and compared with the requested
value. Configuration is applied only while acquisition is stopped and is treated as a
transaction: any failed or mismatched setting prevents `ARMED` state. The driver does not
blindly load a saved camera user set or assume that a node name is shared with the RGB
camera.

The preflight and runtime checks include:

- selection by expected model and configured identity, with ambiguous discovery rejected;
- control-access status, firmware/SDK versions, camera readiness, and detector/cooler state;
- negotiated NIC, link speed, packet size, payload size, pixel format, width, and height;
- hardware-trigger source, polarity, acquisition mode, and actual settings readback;
- image completeness, frame identity, camera timestamp, host receive time, trigger-event
  association, and raw byte count before accepting a frame;
- stream received/incomplete/lost/dropped frame counts plus packet resend statistics;
- bounded frame waits so shutdown and fault recovery cannot hang indefinitely.

The writer preserves the complete received 16-bit radiometric payload. For the installed
A6701 contract this is 640x513: the first 1,280-byte row is the opaque FLIR frame header and
the following 640x512 samples are the image counts. Preview excludes the header row; disk
storage retains it bit-for-bit. Geometry and component offsets are recorded in every segment
and must agree with SDK readback. See `thermal_a6701_contract.md`. Conversion to temperature
and display AGC remain offline operations.

Jumbo frames or packet-size changes are not assumed to be beneficial. Initial operation
uses a conservative documented configuration; network tuning is introduced one change at
a time and retained only after packet-loss and reconnect soak tests demonstrate improvement.

## 4. Time model

### 4.1 Sources

When connected and enabled, UM982 PPS is the phase reference for each UTC second. GGA supplies the UTC label and
position; UNIHEADINGA supplies dual-antenna heading and pitch. The Windows serial receive
time is retained for diagnostics but is not treated as the measurement time.

PPS enters a protected 3.3 V Schmitt input and hardware capture timer. Camera-facing
voltages are produced by separate per-channel interface drivers; PPS is never connected
directly to a camera. The electrical contract and all unresolved installed-hardware checks
are defined in `timing_hardware.md`.

### 4.2 Trigger generation

The timing controller produces independent,
configurable trigger trains:

- FX10e line trigger;
- SWIR line trigger;
- shared RGB and thermal trigger, default 2 Hz;
- reserved RSM400 event input.

Every output event has a channel-local sequence, PPS sequence, hardware tick offset, and
lock state. USB delivery latency does not alter its timestamp.

PPS is selected per task with `timing.pps_required`. In PPS-required mode, the controller captures
the next accepted edge and phase-aligns the frozen schedule. In optional mode, it starts the same
schedule immediately from its hardware timer. Before any valid PPS relationship exists, events use
PPS sequence zero and `UNSYNCED`; sequence and hardware tick remain valid ordering evidence.

Snapshot camera frames are not paired to trigger events by Windows arrival order. At each new
segment, the first accepted camera frame ID is anchored to the expected first channel trigger
sequence. Subsequent frame-ID and trigger-sequence deltas must agree; gaps, regressions, and
unmatched records remain explicit. A reconnect or camera counter reset always starts a new
segment and a new anchor.

The A6701 per-frame input is `SYNC IN`, not `TRIGGER IN`. Because vendor sources conflict on
whether this model supports external FSSI or only FSSR, exposure-start semantics remain
unverified until a separately approved low-rate pulse test confirms actual behavior.

Acquisition starts at the first whole PPS second when PPS is required, or immediately from the
controller hardware clock when it is optional. Rates,
exposure, gain, pixel format, trigger mode, and output paths are frozen until stop.

### 4.3 Quality states

- `LOCKED`: event is referenced to a valid PPS anchor.
- `HOLDOVER`: PPS is temporarily absent; hardware timing continues from the last model.
- `UNSYNCED`: no defensible UTC mapping exists.

No missing or low-quality time is silently replaced. Each record carries a quality state,
uncertainty, and explanatory detail.

### 4.4 GPS association

Raw GNSS messages are always preserved. HSI lines and snapshot frames receive interpolated
GNSS metadata when two suitable 10 Hz solutions bracket the event. Position interpolation
is performed in a Cartesian Earth/local frame, not directly across longitude wrap. Heading
uses circular interpolation. Extrapolation beyond a configured age is forbidden.

Without PPS-derived frame UTC, the system instead brackets by the Windows monotonic receive times
of the frame and GNSS packets. The result is retained as useful approximate position and is always
marked `DEGRADED`; serial/USB delivery uncertainty means it is not equivalent to UTC/PPS timing.

RTK degradation never stops acquisition. The associated record contains fix quality,
differential age, RTK state, interpolation age, and validity.

## 5. RSM400 behavior

RSM400 stabilizes roll and pitch only; it has no usable yaw axis. GNSS supplies heading.

On launch, software connects and observes without changing the mode. The mount normally
enters STAB after its power-on initialization. Start is refused unless initialization is
complete and status is acceptable, unless the operator explicitly forces a degraded start.

Supported operations are:

- verify STAB and current gimbal angles;
- read timestamped roll/pitch and reported yaw field;
- request roll/pitch leveling offsets when optional feature OF002 is present;
- monitor general/extended status, temperature, humidity, motor state, and errors;
- reset resettable errors;
- trigger fast leveling;
- explicitly enable stabilization when needed.

The installed firmware version and optional-feature bitmask must be queried before enabling
feature-dependent commands. Severe RSM faults are prominent but do not control the robot
base.

## 6. Dataset state machine

`IDLE -> PREFLIGHT -> WAITING_FOR_DARK -> CAPTURING_DARK -> WAITING_FOR_SAMPLE ->`
`(WAITING_FOR_PPS when required) -> RECORDING -> STOPPING -> FINALIZED`

Device and session health (`OK`, `DEGRADED`, or `FAULT`) is orthogonal to the task state;
for example, a task remains `RECORDING` while a non-critical device reconnects. A global
disk or trigger-clock failure transitions the task to `FAULT` and performs a controlled stop.

The user supplies one dataset name. The system sanitizes it and appends a UTC timestamp and
unique ID. A task-level manifest records requested and actual device settings, SDK/firmware
versions, calibration-pack identities, serial numbers, and all warnings.

## 7. Data layout

```text
<dataset>_<UTC>_<id>/
  session.json
  events.jsonl
  gnss/raw.nmea
  gnss/samples.csv
  rsm400/raw.mcp
  rsm400/attitude.csv
  timing/triggers.bin
  timing/summary.json
  hsi/fx10e/segment_000.{raw,hdr}
  hsi/fx10e/segment_000.timestamps.bin
  hsi/swir/segment_000.{raw,hdr}
  hsi/swir/segment_000.timestamps.bin
  rgb/segment_000/
  thermal/segment_000/
```

HSI remains ENVI-compatible. RGB stores raw Bayer samples. Thermal stores original
radiometric counts. Previews are derived and never replace raw data.

Long streams are segmented at a configurable time/size boundary. Files are flushed and
checkpoints updated throughout a two-hour task so an interrupted finalization cannot destroy
the entire dataset.

## 8. Failure policy

- A single device disconnect does not stop other devices.
- The terminal emits repeated, rate-limited, visually prominent warnings.
- Reconnection uses bounded backoff. Recovered acquisition begins a new segment.
- Missing sequence ranges and all reconnect attempts are recorded.
- GPS, RTK, PPS holdover, or RSM telemetry degradation is recorded without stopping images.
- Disk write failure, insufficient free space, or loss of the trigger controller causes a
  controlled global stop.
- Normal stop disables triggers, drains queues, checkpoints indexes, closes files, and closes
  both HSI shutters.

## 9. Configuration and secrets

Version-controlled YAML contains safe defaults. Machine-specific COM ports, NIC bindings,
output roots, serial numbers, and calibration-pack paths use a local ignored overlay.
Credentials never appear in source control or dataset manifests. Windows receives UM982 data
only; it does not configure UM982 or own the NTRIP connection.

## 10. Verification ladder

1. Pure protocol, time, interpolation, and file-recovery unit tests.
2. Build and launch with simulated devices.
3. Read-only device enumeration and version queries.
4. One device at a time, with explicit operator approval.
5. Trigger loopback and sequence validation.
6. Thermal-only repeated open/configure/arm/stop/close cycles, followed by cable-loss and
   access-conflict recovery tests.
7. Short synchronized recording, then deliberate disconnects.
8. Storage and thermal reconnect soak test longer than two hours.
9. Field test with prominent acknowledgment that electrical timing precision remains
   specification-based until measured with suitable instrumentation.
