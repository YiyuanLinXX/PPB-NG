# PPBNG software checkpoint — 2026-08-27

This checkpoint records the hardware-free continuation after AC power became available. No camera,
serial port, timing output, RSM400 command channel, or other hardware interface was opened.

## Verified baseline

- ROS 2 Jazzy workspace: 13 packages build successfully on Windows.
- Full test suite: 427 tests, 0 errors, 0 failures, 0 skipped.
- Virtual two-hour soak: 7,200 seconds, 1,468,800 trigger events, correct counts and segments,
  clean finalization.

## Changes completed

- RGB/thermal association verification now uses memory proportional only to unsettled records,
  with a compile-time hard cap of 4,096. Sparse, huge and overflowing sample identities fail
  closed without sample-indexed allocation.
- HSI timestamp CSV records controller tick and controller tick frequency in addition to PPS,
  UTC, uncertainty and association evidence.
- Offline dataset validation streams authoritative RGB/thermal association evidence and HSI
  timestamp/index evidence, comparing every canonical `FrameContext` time and identity field
  exactly with bounded memory.
- `frame_context_node` emits records in receipt order. A not-yet-ready queue head bounds later
  records until the configured wait expires, keeping online publication order identical to the
  persisted evidence order. UNSYNCED records publish canonical UTC zero both online and offline.
- HSI logical source-segment line numbering resets at every real boundary. Coincident recovery,
  SDK-segment change and first-frame gap evidence create one new segment rather than double
  incrementing or creating an empty segment.
- HSI dataset verification requires each `.raw`, `.hdr`, `.index.csv`, and `.timestamps.csv`
  companion set to be complete; orphan companions are rejected.
- Negative tests cover association hard-cap behavior, sparse/huge identities, orphan HSI
  companions, combined recovery boundaries, and exact camera/HSI timestamp-evidence tampering.
- PPS is now an explicit per-task option. PPS-required tasks retain next-edge start; optional-PPS
  tasks use the controller's capability-gated `StartAtCurrentTick` transition and immediately emit
  `UNSYNCED` trigger evidence without waiting for an association timeout.
- The no-PPS trigger path retains channel sequence, absolute controller tick and tick frequency.
  Firmware starts compares beyond a board-supplied minimum lead time before enabling the hardware
  output gate. Host watchdog, disarm, fault latching and event-overflow behavior are unchanged.
- Frames without defensible UTC can receive a best-effort GNSS position by strict Windows
  host-monotonic bracketing. This is always persisted as `DEGRADED`; canonical frame UTC remains
  zero/`UNSYNCED`.
- Controller tick validation now permits multi-second HOLDOVER and no-PPS absolute ticks while
  preserving the within-one-second rule for `LOCKED` records.

## Next approval boundary

The next physical step remains Stage 1 read-only hardware inventory. It requires explicit user
authorization immediately before execution. Later device tests must follow
`docs/device_by_device_validation_zh.md`, one device at a time. Trigger outputs and RSM400 control
remain disabled until their separate approval stages and bench prerequisites are satisfied.
