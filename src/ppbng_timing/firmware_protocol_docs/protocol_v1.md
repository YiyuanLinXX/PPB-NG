# PPBNG timing-controller protocol v1

This document defines a hardware- and transport-independent binary protocol. It does not prescribe or implement serial-port access.

All integers are little-endian. A packet is:

| Field | Size |
| --- | ---: |
| Magic `PT` | 2 bytes |
| Protocol version | 1 byte |
| Message type | 1 byte |
| Payload length | uint16 |
| Packet sequence | uint32 |
| Payload | 0..1024 bytes |
| CRC-32/IEEE | uint32 |

CRC covers protocol version through the final payload byte. It excludes magic and the CRC field. Version 1 uses the reflected IEEE polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, and final XOR `0xFFFFFFFF`.

The four logical trigger channels are FX10e, SWIR, RGB, and Thermal. Their rates are rational `numerator/denominator` Hz. RGB and Thermal identify two consumers of one physical snapshot edge. Protocol v1 therefore requires their enable state and, when enabled, rate, phase, and pulse width to be identical. Firmware produces one physical timer transition through two electrically independent output drivers and reports paired logical RGB and Thermal events with the same PPS sequence and tick offset. A passive cable T is not the specified fan-out.

Configuration is legal only before freeze. `FreezeConfiguration` makes rates, phases, pulse widths, and tick rate immutable for the task. `ArmNextWholeSecond` is accepted only after freeze and starts on the first PPS whose sequence is greater than the supplied `after_pps_sequence`. Disarm releases the frozen task configuration.

PPS is optional. A host that intentionally has no PPS uses
`StartAtCurrentTick` (`0x15`, payload: frozen `schedule_id` as `uint32`) instead of
`ArmNextWholeSecond`. Support is advertised by
`kCapabilityStartAtCurrentTick`. Firmware reads the current hardware timer, adds at
least the board's `minimum_compare_lead_ticks`, preloads every enabled compare while
the output gate is inactive, and only then enables the gate. The schedule remains
frozen; USB/host arrival time is not presented as an edge timestamp.

`DisarmKeepConfiguration` (`0x14`) is the dark-to-sample transition command. It
hard-gates and cancels all outputs, retains the immutable schedule, and returns an
ACK plus status state `FROZEN`. A later `ArmNextWholeSecond` may reuse that exact
schedule. Ordinary `Disarm` (`0x13`) releases a healthy schedule and returns `IDLE`.
If a fault is latched, either command still hard-gates outputs but reports `FAULT`;
only controller reset clears the fault latch.

Every trigger event carries controller boot ID, global event sequence, logical channel, channel sequence, PPS sequence, hardware tick information, tick rate, and `LOCKED`, `HOLDOVER`, or `UNSYNCED` quality. Before the first PPS in `StartAtCurrentTick` mode, `pps_sequence` is exactly zero, `lock` is `UNSYNCED`, and `offset_ticks` contains the absolute monotonic controller tick of the edge (there is no PPS from which an offset could honestly be computed). If a real PPS later arrives, it is emitted as sequence 1 with `utc_second=INT64_MIN`; subsequent triggers use that PPS sequence and an offset from its captured tick. The running trigger phase is not silently shifted onto the late PPS. PPS alone never fabricates UTC: host-side GNSS association is still required. A changed boot ID explicitly marks a controller restart even if counters return to zero.

Firmware requirements:

- Outputs remain inactive from reset through configuration and freeze. Only an
  accepted `ArmNextWholeSecond` or `StartAtCurrentTick` transition may enable timer
  outputs.
- `StartAtCurrentTick` is the other explicit output-enabling transition. It requires
  the same frozen schedule and nonzero host-liveness watchdog as PPS arming. The PPS
  silence watchdog remains mandatory and unchanged for `ArmNextWholeSecond`; it is
  not applicable to explicitly PPS-independent operation. Host watchdog, overflow,
  fault latching, and both disarm commands are identical in both modes.
- PPS and output edges use hardware input-capture/output-compare resources. USB packet timing never schedules an edge.
- Initial compare registers and the output gate are armed before PPS using a hardware
  timer slave-reset/trigger path. The PPS ISR must not program `captured_tick + phase`;
  that cannot safely implement zero or small phase values. Board HAL integration must
  also publish and enforce its minimum lead time for subsequent compare updates.
- Fractional rates use an integer phase accumulator; firmware must not repeatedly round a period and accumulate drift.
- Each command is idempotent by packet sequence and payload. A duplicate returns the cached ACK or error and does not repeat a state transition.
- A trigger event is committed to the event ring in the edge ISR before deferred USB delivery. Ring overflow latches `FAULT`, disables all outputs, and sets a persistent error flag.
- `Disarm`, invalid configuration, watchdog reset, or controller reset immediately places all physical outputs in their inactive state.
- The controller does not receive NMEA and cannot independently know UTC. For controller-originated `PpsAnchor`, `utc_second` is `INT64_MIN`; the host resolves the UTC label from GGA and preserves both the unresolved hardware anchor and the association result.
- Protocol v1 assumes the external PPS conditioner presents the selected active edge as a rising controller input. PPS polarity is an electrical preflight item, not something firmware may guess.
- An empty `Status` request also acts as the host liveness poll. The production watchdog interval and whether loss of host liveness stops outputs are `UNVERIFIED` design decisions that must be fixed before firmware release; they are not silently assigned here.
- Portable-core policy defaults are fail-closed: PPS Arm is rejected until explicit,
  nonzero host-liveness and PPS-silence limits are configured. `StartAtCurrentTick`
  requires an explicit nonzero host-liveness limit and does not require a PPS limit.
- All core access shared by USB task and capture/compare/watchdog ISRs is serialized
  by a board-supplied, bounded critical-section primitive. Exactly one USB task owns
  the command/idempotency cache; ISR paths use only fixed storage.

Message type values and payload layouts are defined by `include/ppbng_timing/protocol.hpp`; the header is the canonical machine-readable definition for firmware and host implementations.

## Windows host identity and readback limits

The host must be configured with an exact COM path, baud rate, controller USB VID,
PID, and USB serial number. Opening a bare Win32 COM handle does not prove the
USB VID/PID/serial that produced that path. Production startup therefore requires
an external, trusted COM-to-device identity resolution step, or an explicit operator
override acknowledging that the OS-level USB identity remains unverified. The
runtime transport never scans COM ports and opens only the configured path.

Protocol v1 `StatusReport` confirms controller state, boot ID, error flags, and the
active schedule ID. It does **not** echo every channel's enable/rate/phase/pulse-width
fields and does not return a configuration hash. Consequently v1 can strictly check
ACK sequence/state and active schedule identity, but cannot prove a bit-for-bit full
schedule readback. A production protocol revision should add a complete schedule
report or canonical configuration hash before claiming full parameter readback.

Win32 overlapped cancellation is requested with `CancelIoEx`. Windows does not give
a hard upper bound for completion of a cancelled operation; the host must keep the
`OVERLAPPED` storage alive until cancellation completes. Therefore the requested I/O
deadline is bounded under normal driver behavior, but a defective kernel driver can
delay cancellation cleanup beyond that deadline. This is a platform limitation, not
a controller protocol guarantee.
