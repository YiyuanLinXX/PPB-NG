# PPBNG development pause checkpoint — 2026-08-26

Pause time: 2026-08-26 19:30 EDT. The operator requested a battery-safe pause no later than
21:27:44 EDT. A usable software-stage version was completed early, so work stopped without using
the remaining battery window.

## Verified baseline

- Workspace: `C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws`
- Build: 13 ROS 2 packages completed successfully on Windows / ROS 2 Jazzy.
- Tests: 406 passed; 0 errors, 0 failures, 0 skipped.
- Virtual soak: 7,200 seconds and 1,468,800 trigger events completed previously.
- No hardware was enumerated, opened, configured, armed, triggered, moved, or acquired from.
- No camera, serial, USB, NIC, SSD qualification, firmware, service, registry, or power setting
  was changed during this stage.

## Added immediately before the pause

- `tools\test_device.cmd` runs one fixed package's hardware-free tests. Supported names are
  `core`, `storage`, `orchestrator`, `timing`, `gnss`, `rsm400`, `rgb`, `thermal`, `hsi`,
  `fx10e`, `swir`, `integration`, and `bringup`.
- `docs\device_by_device_validation_zh.md` defines independent D0-D5 qualification for every
  device and forbids first-time all-device testing.
- `docs\references\source_register.tsv` registers 14 used vendor sources. Eleven local PDF
  entries include immutable absolute paths, sizes, and SHA-256 values; three official online
  sources are explicitly marked `ONLINE_ONLY`.
- RGB and thermal association records now retain `camera_timestamp_ns` in both PENDING and
  terminal records. Offline validation rejects lifecycle timestamp disagreement.
- One new negative test covers camera-timestamp tampering, raising the baseline from 405 to 406.

## Safe resume point

Do not run a production launch when resuming. First confirm the computer is on stable external
power. Then choose one of these paths:

1. Continue pure-software work: cross-check every `FrameContext` numeric timestamp field against
   RGB/thermal association records and HSI timestamp CSV records with bounded memory.
2. Begin Stage 1 only after explicit user authorization: read-only PnP/COM/USB/NIC/SDK-visible
   identity inventory, with no initialization, acquisition, serial transmit, or configuration.
3. After Stage 1, create a local ignored machine YAML and implement the approval-gated
   single-device validation launch. Do not use the all-device production launch as a shortcut.

The active long-term goal is intentionally not marked complete: hardware identities, electrical
timing, per-device physical validation, integrated low-rate testing, SSD qualification, and the
two-hour real acquisition acceptance test remain outstanding.
