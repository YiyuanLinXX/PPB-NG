# PPBNG ROS 2 Workspace

Windows 10 / ROS 2 Jazzy acquisition software under development for the PPBNG field robot.

The target production system controls and records:

- Specim FX10e and Specim SWIR line-scan hyperspectral cameras;
- FLIR Blackfly-S BFS-U3-122S6C-C RGB camera;
- FLIR A6701 MWIR 640x512 thermal camera;
- SOMAG RSM400 two-axis stabilization mount;
- read-only UM982 dual-antenna GNSS data;
- a multi-channel hardware trigger controller with optional PPS discipline.

The legacy projects beside this workspace are read-only references. New implementation
must stay inside this directory.

Development and hardware work must follow [`docs/safety.md`](docs/safety.md). Full machine
access is never treated as permission to connect to hardware or change system settings.

## Intended operating model

1. The operator powers all hardware and waits for thermal stabilization.
2. One production launch command connects to devices and performs non-moving checks.
3. The operator supplies a dataset name and starts the guided dark-reference workflow.
4. With `timing.pps_required: true`, acquisition begins on the next PPS boundary. With it set
   to `false`, the frozen schedule starts immediately from the controller hardware clock.
5. Raw image data is written directly to disk. ROS carries control, status, preview,
   timing metadata, and diagnostics only.
6. A normal stop flushes all files and closes both HSI shutters.

No hardware is started merely by building or testing this workspace.

## Current validation status

The orchestration, storage contracts, conservative PPS/GNSS time authority, device mocks,
production ROS adapters, optional vendor backends, and fail-closed production gate are
implemented and hardware-free tested. `production.launch.py` now wires the real production
processes, but it must not be used on hardware until the remaining identity, electrical,
trigger-semantics, RSM400 ICD, and timing-latency bench checks are completed with explicit
operator approval.

The A6701 transport contract deliberately retains every 640x513 Mono16 payload byte: the first
1280-byte row is preserved as opaque FLIR metadata/header data, and the remaining 640x512 samples
are the radiometric image. No software path silently crops the transport payload.

Each persisted RGB, thermal, FX10e, and SWIR scene sample now emits one common `SampleStamp` and
one append-only `FrameContext` record. GNSS position is strictly UTC-bracketed; GGA dates are
resolved only from nearby full-UTC UNIHEADINGA records in the same serial connection epoch,
including either arrival order and midnight rollover. RSM attitude is independently bracketed on
the host monotonic clock. Missing, stale, ambiguous, or holdover evidence is recorded as
`UNAVAILABLE`/`DEGRADED` rather than guessed. Each `FrameContext` is self-contained: its sidecar
retains the complete canonical stamp evidence (trigger channel and sequence, PPS sequence,
controller tick and frequency, uncertainty, status/detail, and camera counter/timestamp), source
sequences, endpoint ages, fix quality, GNSS pitch, RSM roll/pitch/yaw, RSM endpoint error levels,
and GGA UTC-anchor residuals for later audit. RGB/thermal frames still enter this audit path when
their sequence relationship is not bench-confirmed, but their canonical time is forced to
`UNSYNCED`.

PPS is optional per task. Without PPS, raw acquisition, trigger sequence/tick evidence, GNSS,
RSM400 telemetry, and all camera timestamps continue normally. Frames use an explicitly
`DEGRADED` host-monotonic receive-time bracket for best-effort GNSS association, while canonical
UTC remains `UNSYNCED`; this lower-quality path is never presented as PPS synchronization.

RGB and thermal segment records carry committed payload CRCs, and every HSI ENVI index row carries
the CRC32, byte offset, and byte length of its corresponding raw scan line. This permits later
bit-integrity checking without converting the original Bayer, radiometric, or hyperspectral data.

`manifest.json` schema 3 also carries a per-device runtime summary. The production manager merges
only status records bound to the known current session and configured device-role mapping, retains
monotonic maxima for sample/incomplete/lost/dropped/reconnect counters, rejects stale status from
overwriting the latest lifecycle detail, and rewrites a terminal manifest when a late final status
arrives. RGB, thermal, and HSI nodes publish their validated readback settings so the manifest can
distinguish requested configuration from what the backend actually accepted.

RSM400 startup is observe-only and fails closed until the installed firmware's exact STAB status
digit and acceptable error rule have been confirmed. It does not send `RC1`, `ST`, reset, or target
commands during readiness. Thermal identity uses the exact GenICam `DeviceID`; MAC and IP remain
read-only inventory evidence until their installed node-map keys are verified. A6701 frame-sync
semantics have a separate fail-closed flag that may only be enabled after Stage 4 validation.

The hardware-free baseline can be reproduced with:

```bat
tools\build.cmd
tools\test.cmd
tools\run_soak.cmd 7200
```

The current hardware-free baseline is 13 packages built, 427 tests passed, and a successful
7,200-second virtual soak producing 1,468,800 trigger events with clean finalization. The soak
advances a virtual clock, so it completes quickly. It exercises independent
HSI rates, shared 2 Hz RGB/thermal triggering, temporary thermal and HSI disconnects, RTK
degradation, segment recovery, and clean finalization without opening any device.

## Simulation launch

After building, run the hardware-free orchestration node with:

```bat
tools\run_simulation.cmd
```

This script loads only the simulation runtime. It does not enumerate cameras, open serial
ports, drive the RSM400, or emit trigger signals. With the default empty `output_root`, a
Start request is refused and no dataset directory is created.

See [docs/architecture.md](docs/architecture.md) for the authoritative system design and
[docs/thermal_a6701_contract.md](docs/thermal_a6701_contract.md) for the A6701 payload and
synchronization contract. Configuration and hardware opt-in rules are in
[docs/configuration.md](docs/configuration.md). The proposed PPS/trigger wiring and its mandatory
bench checks are in [docs/timing_hardware.md](docs/timing_hardware.md).
The approval-gated path from read-only identity inventory to a two-hour full-rate acceptance run
is in [docs/hardware_validation_plan.md](docs/hardware_validation_plan.md).
The separately approval-gated, bounded Windows SSD durable-write check is documented in
[docs/storage_qualification.md](docs/storage_qualification.md); production launch never runs it
automatically.
The exact default two-hour rate and free-space calculation is in
[docs/storage_budget.md](docs/storage_budget.md).
Single-device software and approval-gated hardware testing is organized in
[docs/device_by_device_validation_zh.md](docs/device_by_device_validation_zh.md). The guided
RGB + thermal + dual-HSI workflow, safe sequencing, and GitHub upload procedure are in
[docs/four_camera_ros2_test_zh.md](docs/four_camera_ros2_test_zh.md). The local vendor
document inventory and source-retention rules are in
[docs/references/source_register.tsv](docs/references/source_register.tsv) and
[docs/references/README.md](docs/references/README.md).

After a task, HSI, RGB, and thermal payload files can be checked without loading any SDK or
touching hardware:

```bat
tools\verify_dataset.cmd D:\PPBNG_DATA\<session-directory>
```

The command is read-only. It validates every HSI `.raw`/`.index.csv`/`.timestamps.csv`/`.hdr`
four-file part for contiguous offsets, exact indexed extent, timestamp/index row agreement, ENVI
geometry, and per-line CRC32. Both HSI streams must have contiguous segment, part, source-line, and
camera sequence-gap evidence. It also checks every RGB/thermal `.ppbseg` record header,
payload CRC, commit trailer, and in-file sample ordering. `manifest.json` must be a bounded valid
schema-3 terminal manifest with one consistent run mode, final storage evidence, and exactly one
entry for every required PPBNG device role. GNSS, RSM, frame-context, and camera-association
JSONL/NDJSON files are checked incrementally with bounded memory, one flat object per
newline-terminated record, with duplicate keys rejected. Each committed RGB/thermal sample must
have exactly one matching `PENDING` record followed by exactly one terminal association record;
segment/sample identity, SDK frame, camera timestamp, host timestamp, trigger evidence, UTC
integer value, and terminal-state semantics are cross-checked against the already verified
`.ppbseg` summary. Association lifecycle verification retains only unsettled records, has an
unraiseable 4,096-record hard cap, and releases each slot as soon as its terminal record is
validated. A
validation failure returns a non-zero exit code. The old `verify_hsi_dataset.cmd` name remains as a
compatibility alias.

The same command also proves scene-context coverage: after excluding HSI dark-reference rows,
every committed RGB, thermal, FX10e, and SWIR scene identity must occur exactly once in
`segments/frame_context.ndjson` under the manifest's exact session ID. Missing, duplicate,
cross-session, unknown-stream, and no-corresponding-payload records fail validation. Explicit
source-evidence cursors then compare every canonical host/UTC time, PPS and trigger sequence,
controller tick/frequency, uncertainty/status/detail, SDK frame, and camera timestamp exactly
against the RGB/thermal association sidecars or HSI timestamp sidecars. This comparison is
streaming and uses bounded memory. Explicit
`UNAVAILABLE` GNSS or RSM status remains valid evidence—it records that synchronization context
was unavailable instead of silently inventing a value—and its count is printed in the report.

For RGB and thermal, filenames must be `<stream>_<six-digit-segment>.ppbseg`; both streams must
exist, segment indices must start at zero without gaps, and sample IDs must start at one without
gaps within or across segments. Only the final segment of a stream may be empty.
