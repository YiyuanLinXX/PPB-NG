# ppbng_thermal

The default SDK-OFF build provides the vendor-neutral A6701 contract, deterministic mock, queue, and pure runtime metadata mapping. It loads no Spinnaker library and performs no camera operation.

With `PPBNG_THERMAL_ENABLE_SPINNAKER=ON`, the real backend and `ppbng_thermal_camera_node` are compiled. The node remains inert until `allow_hardware_access:=true` and an explicit `~/start` call. It requires exact non-empty identity, `session_id`, an existing dataset `session_root` containing `segments/`, and a safe `stream_name`. Construction and parameter loading never enumerate or open a camera.

Each accepted frame is written exactly as one 656640-byte payload inside a checksummed `ppbng_storage` record. The first 1280 bytes remain an opaque FLIR metadata row; bytes 1280..656639 remain the 640x512 Mono16 radiometric image region. No conversion, byte swapping, header parsing, or row removal occurs. `maximum_segment_bytes` controls rollover; segments are exclusive-created, checkpoint-flushed, and recoverable after a truncated tail. Published `~/frame_metadata` carries the SDK frame counter, camera timestamp in SDK-documented nanoseconds, host monotonic receive timestamp, and both transport and image/auxiliary geometry.

Every start-stage failure invokes backend stop/release and closes the segment writer through a transaction guard. Worker failures persist as `FAULT` through `~/status`, and `~/stop` joins the worker and reports the cause. Dataset-level segment index/manifest atomic finalization remains owned by the orchestrator; the node provides recoverable per-segment records and final checkpoints.

The current production node deliberately refuses arm while `FrameSyncEvidence` is `unverified`. The installed A6701 documentation does not conclusively establish whether this camera/firmware provides FSSI or only FSSR semantics, and the backend currently cannot assert verified exposure-start synchronization merely from `FrameSyncSource=External`. A later operator-approved write/readback plus low-rate pulse validation must establish that evidence. No override is provided here because silently accepting unverified synchronization would corrupt the timing contract.

Trigger observations are now filtered by `trigger_channel`; the append-only `segments/<stream>_association.ndjson` records PENDING and final MATCHED, CONSISTENT_UNVERIFIED, or UNMATCHED states plus the SDK frame counter, camera timestamp in nanoseconds, frame-callback time, and trigger-receipt monotonic time. Offline verification rejects disagreement between the PENDING and terminal frame evidence. Because constant counter delta cannot exclude a missed-trigger/FIFO shift, `association_evidence_confirmed` defaults false and no trigger/PPS/UTC FrameMetadata is published in that state. MATCHED publication requires an explicit true setting after hardware semantics have been validated. Dataset-level manifest finalization and approved-hardware validation of exact A6701 node names and Ready/FPACold behavior remain outstanding. No unit test constructs the production node or calls the SDK.

`~/arm` is a no-I/O preflight: it creates no files and does not obtain a Spinnaker System, enumerate, or open the A6701. `~/start` requires a successful arm for the current prepared session.

Runtime recovery is bounded and fail-closed. `consecutive_timeout_threshold` selects when timeouts start recovery; `recovery_maximum_attempts`, `recovery_initial_backoff_ms`, and `recovery_maximum_backoff_ms` control exponential backoff. Each attempt stops the backend, re-discovers exactly the configured A6701 identity, reapplies frame-sync configuration, validates readback/evidence, and arms. Success exclusively creates the next segment and resets association anchors while preserving the full 640x513 payload contract. Storage faults cause global stop and are never retried; acquisition/transport recovery remains device-local. `~/stop` interrupts backoff.

The 2026-08-27 connected-device result is recorded in
`docs/validation/thermal_a6701_stage1_2026-08-27.md`. It confirms exact A6701 identity,
the 640x513 radiometric payload, internal free-run operation near 60 Hz, and 20/20
complete volatile frames. It also confirms that the present production node is not yet
the correct entry point when no external trigger is connected; internal/free-run mode
must be implemented as a separate explicit configuration rather than bypassing the
external-sync evidence check.

For isolated A6701 + UNO R4 validation, use the dedicated
`a6701_uno_r4_test.launch.py`. It is a real ROS 2 node, records raw and timing evidence,
keeps UTC explicitly UNSYNCED when GPS/PPS is absent, and owns both the Arduino and
camera shutdown sequence. Detailed operator instructions are in
`docs/thermal_ros2_uno_r4_test.md`.
