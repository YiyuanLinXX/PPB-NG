# ppbng_rgb

The default build contains only the vendor-neutral RGB contract and deterministic mock. It neither loads Spinnaker nor touches camera hardware.

## Optional Spinnaker 4.4 backend

The real backend is deliberately opt-in:

```powershell
colcon build --packages-select ppbng_rgb --cmake-args `
  -DPPBNG_RGB_ENABLE_SPINNAKER=ON `
  "-DPPBNG_SPINNAKER_ROOT=C:/Program Files/Teledyne/Spinnaker"
```

It was compile-validated against the installed Windows x64 SDK 4.4.0.246. Enabling the build only compiles and links the adapter; constructing it also performs no SDK or hardware operation. A caller must explicitly invoke `discover()` before any camera enumeration occurs. Hardware calls still require the project's separate operator approval and safety procedure.

The caller must provide an exact, non-empty camera serial to `open()`. Missing or duplicate identities fail closed. Configuration is a transaction: the backend turns trigger mode off, writes and reads back continuous acquisition, raw Bayer format, width/height, FrameStart selector, external trigger source/activation, configurable ExposureAuto/GainAuto/BalanceWhiteAuto policy, white-balance profile, and required Chunk Data, then enables trigger mode. Any error or mismatch rolls the changed nodes back. `arm()` repeats readback before starting acquisition. Defaults use continuous exposure/gain and one-shot Outdoor white balance; they remain ROS parameters rather than hidden constants.

`next_frame()` uses the caller's finite timeout with `GetNextImage(timeout)`. A local guard explicitly calls `Image::Release()` exactly once on success, early return, or exception; bytes are copied without color conversion into owned storage before release. Incomplete frames are rejected. Frame metadata records Image API frame/timestamp evidence, Chunk FrameID/Timestamp/ExposureTime/Gain/BlackLevel, immediate post-frame red/blue BalanceRatio readbacks, and the host `steady_clock` receive time. Image and Chunk counters are preserved separately rather than assumed equal. Recovery tears down the old SDK objects, rediscovers the exact same serial, returns to the open state, and increments the segment index; the caller must configure/read back/arm again. A missing camera fails closed so the orchestrator can retry without accidentally switching devices.

SDK discovery/open and camera initialization are synchronous vendor calls for which Spinnaker exposes no cancellation API. The timeout is therefore a bounded image-wait contract, not a hard deadline around SDK enumeration or initialization. These lifecycle calls must run on the driver's worker thread so they cannot block ROS executor progress.

Unit tests never instantiate the Spinnaker backend and never enumerate or open hardware. They cover device identity selection, trigger/raw-Bayer contract validation, lifecycle behavior, frame lease release, queue backpressure, and mock recovery.

When the SDK option is ON, `ppbng_rgb_camera_node` is also built. It remains inert until `allow_hardware_access:=true`, `~/prepare`, and the no-I/O `~/arm` preflight succeed; only an explicit `~/start` may then access the SDK. Start requires an exact `device_id`, the prepared dataset root containing `segments/`, and a safe `stream_name`. It writes exclusive `ppbng_storage` `.ppbseg` files rather than an unsafe long raw file. Records have explicit boundaries and payload CRCs; `maximum_segment_bytes` controls rollover, and each rollover/final close is checkpoint-flushed. Existing segments are never overwritten and truncated tails are recoverable with `recover_truncated_segment`.

Every discover/open/configure/readback/segment-create/arm/thread-create failure is covered by a start transaction guard which invokes backend stop/release and drops the writer exactly once. Worker acquisition, validation, and storage exceptions become a persistent fault visible through `~/status`; `~/stop` always joins the worker and reports the fault. Complete raw Bayer payloads are stored without conversion. The node is intentionally absent from SDK-OFF builds. Dataset-level `segments.tsv` and manifest atomic finalization remain the responsibility of the orchestrator/storage owner; this node provides recoverable per-segment finalization only.

`~/arm` is a no-I/O preflight: it creates no files and does not obtain a Spinnaker System, enumerate, or open a camera. `~/start` requires a successful arm for the current prepared session. Trigger observations are filtered by `trigger_channel`; the append-only `segments/<stream>_association.ndjson` records PENDING and final MATCHED, CONSISTENT_UNVERIFIED, or UNMATCHED states plus the SDK frame counter, camera timestamp in nanoseconds, frame-callback time, and trigger-receipt monotonic time. Offline verification rejects disagreement between the PENDING and terminal frame evidence. Because constant counter delta cannot exclude a missed-trigger/FIFO shift, `association_evidence_confirmed` defaults false and no trigger/PPS/UTC FrameMetadata is published in that state. MATCHED publication requires an explicit true setting after hardware semantics have been validated.

Runtime recovery is bounded and fail-closed. `consecutive_timeout_threshold` selects when timeouts start recovery; `recovery_maximum_attempts`, `recovery_initial_backoff_ms`, and `recovery_maximum_backoff_ms` control exponential backoff. Each attempt stops the backend, re-discovers the exact identity, reapplies configuration, validates readback, and arms. Success exclusively creates the next segment and resets association anchors. The old segment and sidecar are settled, checkpointed, flushed, and closed first. Storage faults cause global stop and are never retried; acquisition/transport recovery remains device-local. `~/stop` interrupts backoff.
