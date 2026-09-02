# FLIR A6701 acquisition contract

This document separates confirmed transport facts from settings that require controlled
hardware verification. It is authoritative for the first implementation of
`ppbng_thermal`.

## Confirmed payload layout

For the installed A6701 configuration, the transport image is `640 x 513 Mono16`:

| Component | Byte offset | Byte length | Meaning |
| --- | ---: | ---: | --- |
| FLIR frame header | 0 | 1,280 | First transport row; opaque metadata bytes |
| Radiometric image | 1,280 | 655,360 | Following `640 x 512` 16-bit count samples |
| Complete payload | 0 | 656,640 | Header plus image, preserved bit-for-bit |

The first row, not the last row, is the FLIR frame header. ROS previews and ordinary image
views expose only the following 512 image rows. The writer preserves the complete payload
and records component offsets in the segment manifest.

Until the matching FLIR BHP header definition is obtained, the 1,280-byte header is opaque.
The program must not reverse-engineer fields, assign timestamp semantics, byte-swap it, or
silently discard it. Payload geometry must still be validated from actual SDK readback at
each configuration and reconnect. A mismatch prevents `ARMED` state rather than being
cropped or coerced.

`IRFormat=Radiometric` means count output; it does not prove that digital NUC, bad-pixel
replacement, digital gain, or digital offset are disabled. Their actual states are recorded
without automatically changing the established camera calibration chain.

## External synchronization

Per-frame timing uses the camera `SYNC IN` input and the GenICam frame-sync controls. The
`TRIGGER IN` input starts a sequence and is not treated as an equivalent per-frame clock.

The intended configuration is:

- `FrameSyncSource=External`;
- active edge/polarity explicitly configured and read back;
- 1 ms input pulse, subject to final electrical-interface verification;
- `TriggerMode` not used as the 2 Hz per-frame timing mechanism.

FLIR sources conflict on whether the A670x supports external frame-sync-start-integration
(FSSI) or only frame-sync-start-readout (FSSR). Node presence is not proof of physical
behavior. The production manifest therefore records the actual `FrameSyncMode` readback and
an evidence state: `unverified`, `verified_fssi`, or `verified_fssr`. No exposure-start time
claim is made while it is `unverified`.

## Required preflight readback

The thermal driver may arm only when all applicable checks pass:

- exactly one configured identity matches and read/write control access is held;
- `CameraModel=A6701` (plus configured MAC/IP/user ID and manufacturer identity);
- `Ready=true` and `FPACold=true` are independently satisfied;
- `Width=640`, `Height=513`, `PixelFormat=Mono16`, and `PayloadSize=656640`;
- `IRFormat=Radiometric`;
- frame-sync source, mode, and polarity match the task configuration;
- stream buffer and packet-resend settings are readable and actual values are recorded;
- correction/calibration states and relevant firmware/SDK versions are recorded.

`CoolDownTime` is diagnostic only and must not substitute for `FPACold`. Read-only or busy
access is a hard preflight failure, not a degraded acquisition mode.

## Frame acceptance and ownership

Every acquired SDK image is owned by an RAII guard and is released exactly once on success,
incomplete-frame, timeout, conversion, queue, writer, and shutdown paths.

A frame enters the scientific sequence only if:

- the SDK marks it complete;
- its payload length and geometry match the active segment contract;
- its frame identity is not duplicate or out of the accepted ordering policy;
- it can be associated with a trigger event without inventing a timestamp.

Incomplete or rejected frames still create a fault/gap record with available camera frame
ID, image status, transport counters, and host monotonic receive time. They do not masquerade
as valid frames.

Acquisition, ROS control/status, preview conversion, and disk writing use separate execution
paths. The acquisition-to-writer queue is bounded; saturation is visible and cannot silently
discard or overwrite scientific frames.

## Prohibited automatic operations

The driver must not automatically:

- reset the device or load factory/user defaults;
- alter persistent IP, NIC MTU/jumbo frames, firewall, driver, or power settings;
- update firmware;
- write calibration/NUC files or disable the established correction chain;
- disable GVCP heartbeat in production;
- select `camera[0]` without identity validation.

## Verification gates

Pure software tests precede hardware access:

1. exact `640 x 513` split and bit-for-bit round trip;
2. invalid payload, stride, padding, byte-order, and geometry fixtures;
3. fake NodeMap missing/read-only/out-of-range/readback-mismatch cases;
4. state-machine exception injection and exactly-once resource release;
5. incomplete-frame storms and bounded-queue backpressure;
6. segment rollover and truncated-write recovery.

After separate operator approval, hardware validation proceeds from read-only identity and
node inspection to low-rate external sync, 1,000-pulse accounting, disconnect recovery, and
finally long-duration soak testing. NIC changes require their own approval.

## Sources

- Local A6701 user manual: `../../docs/Thermal_Cam_FLIR_A6701/FLIR_A6701_Thermal_Camera_User_Mannual.pdf`
- Local FLIR cooled-camera GenICam processes document: `../../docs/Thermal_Cam_FLIR_A6701/FLIR US Cooled Camera Genicam Processes Document_April 2020.pdf`
- Local A6701 datasheet: `../../docs/Thermal_Cam_FLIR_A6701/FLIR-A6701-MWIR_29440-201.pdf`
- Historical read-only NodeMap snapshot: `../../docs/Thermal_Cam_FLIR_A6701/NodeMapInfo.md`
- FLIR FAQ, additional header row: https://flir.custhelp.com/app/answers/detail/a_id/1637/
- FLIR FAQ, trigger versus sync: https://flir.custhelp.com/app/answers/detail/a_id/3385/
- FLIR external frame-sync settings: https://flir.custhelp.com/ci/fattach/get/301213/0/filename/Research%2BStudio%2B-%2BExternal%2BFrame%2BSynch%2Bsettings.pdf
- Spinnaker SDK documentation: https://softwareservices.flir.com/Spinnaker/latest/
