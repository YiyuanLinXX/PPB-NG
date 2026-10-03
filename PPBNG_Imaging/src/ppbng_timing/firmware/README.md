# Portable timing-controller firmware core

This directory is a host-buildable state-machine and scheduler core. It is not a board project and contains no STM32 pin, clock, DMA, USB, linker, or startup setup. It must not be flashed as-is.

## Required HAL guarantees

`ITimingHardware` is a safety contract, not a convenience wrapper:

- `set_output_gate(false)` must asynchronously force every physical trigger output to its inactive electrical level, independent of timer compare state.
- `enter_timing_critical` / `exit_timing_critical` must mask or otherwise serialize the USB task, PPS capture ISR, every output-compare ISR, watchdog ISR, event-ring consumer, and status reader. It must be nesting-safe and bounded. The core keeps dynamic packet parsing/encoding outside this critical region; only bounded state copies, state transitions, fixed-array operations, and bounded HAL register calls occur inside. Exactly one USB task may call `handle_command`; the command cache is single-owner. `reset` is a boot/reset operation and must not race that task.
- `prepare_pps_synchronous_output` must preload timer compare registers before PPS. `arm_output_gate_on_next_pps` must use a hardware timer slave-reset/trigger/gate path so even `phase_ticks == 0` is generated on the PPS hardware edge. Implementing this by programming `compare = captured_tick + phase` from the PPS ISR is forbidden.
- When a phase-zero compare and PPS capture share one timer tick, the board interrupt dispatcher must call `on_pps_capture_isr` before `on_output_compare_isr` for that tick. The physical edge is hardware-generated either way, but this ordering is required to label and enqueue the trigger against the new PPS anchor.
- Subsequent DDS compare updates are accepted only when the minimum interval is at least `minimum_compare_lead_ticks`. A board whose timer/register path cannot meet this limit must reject Arm.
- Capture/compare entry points and their HAL calls must not block or allocate. ISR events use a fixed ring with 256 usable slots. Overflow immediately hard-gates all outputs and latches FAULT.
- USB CDC parsing/transmission runs outside timing ISRs and feeds complete protocol v1 packets to `handle_command`.

The default `SafetyPolicy` is fail-closed. PPS arming requires nonzero host- and PPS-silence limits. The explicit PPS-independent `StartAtCurrentTick` mode requires a nonzero host-silence limit; its PPS-silence limit is intentionally inapplicable. It safely preloads compares at least `minimum_compare_lead_ticks` beyond the sampled hardware tick before opening the output gate. Board integration must call `service_watchdogs_isr` from a monotonic timer in either mode.

Before a PPS is ever observed, PPS-independent trigger events use `pps_sequence=0`, `UNSYNCED`, and place the absolute controller edge tick in `offset_ticks`. A later real PPS starts sequence 1 and is reported with unresolved UTC (`INT64_MIN`); the free-running trigger phase is not altered. Only host GNSS association may assign UTC.

`DisarmKeepConfiguration` gates/cancels outputs but retains the frozen schedule and returns `FROZEN`. This supports dark-reference triggering followed by a second `ArmNextWholeSecond` without reconnecting or changing task configuration. Ordinary `Disarm` releases a healthy schedule and returns `IDLE`. A latched fault is cleared only by reset; both disarm variants still hard-gate outputs while returning `FAULT`.
