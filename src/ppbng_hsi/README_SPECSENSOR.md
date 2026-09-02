# SpecSensor backend safety and verification status

The SpecSensor/LUMO hardware backend is compiled only when
`PPBNG_HSI_ENABLE_SPECSENSOR=ON`. The default is `OFF`; default builds do not include
`SI_sensor.h`, link `SpecSensor.lib`, load a vendor DLL, enumerate profiles, or access hardware.

The implementation uses only APIs and features present in the installed SpecSensor 2020_519
headers, official SDK PDF, and installed FX10e/Pleora and SWIR/NI profiles:

- one process-wide `SI_Load`/`SI_Unload` lifetime with all `SI_*` calls serialized;
- separate handles, configurations, line rates and callback queues for FX10e/Pleora and SWIR/NI;
- `Camera.Trigger.Mode=Internal` for the robot deployment, with immediate enum
  readback. ROS issues one `Acquisition.Start`/`Acquisition.Stop` pair and the
  cameras free-run continuously on their internal line clocks; RGB/thermal
  external-trigger behavior is independent of this HSI setting;
- frame rate, exposure, geometry, byte depth and payload-size readback before arming;
- shutter close/open commands with `Camera.Shutter.IsOpen` readback;
- callback work limited to bounded validation, preallocated-buffer copy and queue insertion;
- production profiles use 128 preallocated line slots (about 1.07 seconds at 120 Hz), with the
  allocation derived from the exact configured line geometry and capped at 512 slots / 1 GiB;
- callback queue overflow or an invalid callback payload is counted in `DeviceStatus`, closes the
  current source segment, and starts bounded recovery for only the affected HSI camera; other
  sensors continue and the fault remains auditable;
- SDK frame numbers retained as camera line sequence numbers and checked for gaps;
- recovery starts a new segment and requires an explicit subsequent start.

## Hardware verification still required

Do not enable acquisition until a user-approved test confirms all items below:

1. The stable SpecSensor device indices/profile names and both physical serial numbers. `SI_Load`
   itself enumerates profiles, so this cannot be verified in a hardware-free run.
2. The deployed FX10e Pleora profile and SWIR NI profile/channel strings, NI camera file, license,
   calibration packs, and every dependent DLL match the known-working installation.
3. Internal-line-rate stability, SDK callback latency/jitter, and the relationship between the
   configured exposure feature and the actual integration interval for each camera.
4. The strongest timestamp evidence available from each transport. The current integration has
   SDK frame numbers and host callback-arrival monotonic time, but no confirmed per-line camera
   exposure timestamp; this limitation must be characterized before claiming GPS-level line timing.
5. Measured readback geometry, byte depth, BIL orientation, maximum sustainable rate, ring-buffer
   lag, dropped-frame reporting, and required queue capacity for a two-hour run.
6. Shutter readback latency, dark-line settling/discard count, SWIR NUC behavior, and whether either
   profile resets its SDK frame number between dark and sample acquisitions.
7. Disconnect/reconnect behavior for Pleora and NI. Production isolates the two SDK lifetimes in
   separate processes so one worker can be stopped/restarted without unloading the peer's DLLs.

The SDK-enabled build is a compile-time compatibility check only. It is not evidence of hardware
support or synchronization correctness.

## ROS 2 production node

`hsi_production_node` is inert at construction. It creates no adapter until the `arm` service has
validated a complete configuration and a separate `start` service is called. `start` connects,
configures, verifies readback, and closes the shutter, but does not begin acquisition. Dark capture
and sample acquisition require separate `begin_dark` and `start_sample` service calls. Production
bringup launches `/fx10e/camera` and `/swir/camera` in two `hsi_production_node` processes. Real
hardware testing showed that placing Pleora and NI handles in one process can crash inside
`PtConvertersLib64.dll` or an unloaded `PvPersistence64.dll`; that executable is therefore not built
or installed. A Windows named mutex serializes each complete SDK initialize/stop/recovery lifecycle
transaction across the two workers, while their acquisition callbacks and writers run concurrently
in isolated address spaces. Each node owns its services, configuration, callback queue, writer,
watchdog, recovery state, and segment sequence. All identities and transport settings come from the
unified configuration. With the default SDK-OFF build, `arm` always fails closed.

Production storage is also fail-closed. `allowed_output_root` must name the configured dataset root.
Construction has no session. The `prepare` service binds a request/session ID to an already-created
strict child containing its existing `segments` directory; it never creates a session directory.
All request IDs are retained for lifetime idempotency, conflicting payloads are rejected, and
prepare is forbidden after start. Each camera process uses its fixed
`camera_kind` (`fx10e` or `swir`) as an independent filename stem. Every produced line is written as
unframed little-endian uint16 BIL data to an ENVI `.raw`, with an atomically refreshed `.hdr` and
append-only timestamp/index CSV sidecars. The index explicitly marks `dark` versus `sample` and
records source segment and rollover part. Recovery starts a new segment; `maximum_segment_bytes`
starts a new part. `flush_every_lines` controls periodic flush/checkpoint. Existing targets are
never reused. Any storage failure publishes a fault and stops only that HSI stream.

Trigger callbacks only enqueue bounded `TriggerEvent` records. A 1 ms ROS timer retries the oldest
channel sequence against the bounded SDK-frame queue, so a transient `not_ready` does not immediately
discard the trigger and no callback busy-waits. This alone is not evidence of synchronization.
For each logical source segment, the first observed SDK `frame_number - channel_sequence` delta is
`UNVERIFIED`; subsequent consecutive frames with the same delta are only
`CONSISTENT_UNVERIFIED` by default. A constant delta cannot detect the ambiguous case where a
trigger produces no frame and the following frame is paired with the older queued trigger.
`MATCHED` is therefore allowed only when `association_evidence_confirmed=true`, after an explicit
hardware validation has established the vendor frame-counter, missed-trigger, reset, and rollover
semantics for that exact camera/transport configuration. A trigger gap,
frame-number gap/regression, delta mismatch, queue overflow, or timeout breaks the anchor, records
affected output as `UNMATCHED`/`UNVERIFIED`, and opens a new logical source segment for re-anchoring.
Each ENVI index row also stores a CRC32 of that exact raw scan-line payload, alongside its byte
offset and length, so an offline verifier can detect localized raw-data corruption without
interpreting calibrated radiance.
The raw frame is retained, but downstream processing must never treat non-`MATCHED` lines as
synchronized. Timestamp and index sidecars carry association status, SDK and logical segment IDs,
delta, anchor frame/trigger IDs, anchor validity, PPS sequence, controller tick, and controller
tick frequency. `pending_trigger_capacity` and
`trigger_match_timeout_ms` bound memory and latency; overflow and expiry are exposed as dropped/lost
counters. Persistence happens before the trigger is consumed. Completion of the requested dark
lines publishes the exact status detail `dark_complete`.

After, and only after, a scene (`sample`) line is successfully appended, the node publishes a
`SampleStamp` on relative topic `sample_stamp`; dark-reference lines never publish scene stamps.
The stamp carries session/device/source-segment and stored line identity, trigger/PPS/controller
tick evidence, SDK frame number, and the monotonic arrival captured inside the SDK callback before
the bounded queue. The same arrival value is written to the timestamp CSV. SpecSensor provides no
confirmed per-line camera timestamp here, so `camera_timestamp_valid` remains false. A non-MATCHED
association is still emitted for offline audit with raw trigger UTC and uncertainty preserved, but
its canonical `TimeQuality.status` is forced to `UNSYNCED` and the original status plus association
classification are recorded in `TimeQuality.detail`.

Structured faults use the relative `fault_event` topic. ENVI create/write/checkpoint/flush/close
failures set `causes_global_stop=true`, because continued collection could leave an incomplete
dataset. Integrity and timing faults are likewise fatal and are never retried. Camera disconnects
and repeated bounded frame timeouts enter configurable, stop-interruptible exponential backoff.
Before reconnecting, the current ENVI part and both sidecars are finalized. Recovery reselects the
frozen device index, verifies the configured sensor serial, reapplies configuration/readback and
arm state, resets the trigger/frame association epoch, and begins a new source segment. Dark
recovery conservatively restarts the complete requested dark-line count. Exhausting the configured
attempt limit latches a persistent prominent fault. The SpecSensor SDK does not expose a proven
cancellable reconnect call in this integration, so interruption is guaranteed between attempts,
not while a vendor call itself is executing. Hardware behavior remains unverified.
