# PPBNG PPS and camera-trigger hardware contract

Status: design baseline, not hardware-verified.  
Scope: UM982 PPS capture; independent FX10e and SWIR line triggers; one logical
snapshot trigger fanned out through separate RGB and A6701 electrical drivers.

This document is intentionally conservative. `CONFIRMED` means that the local
manufacturer material states the value. `UNVERIFIED` means that the installed unit,
cable, breakout, or electrical behavior must be measured before connection. A proposed
project target is not a vendor specification.

## 1. Safety boundary

- Do not connect a controller output directly to any camera until the open-circuit
  voltage, loaded voltage, inactive level, polarity, and pulse width have passed the
  tests in section 9.
- The Windows PC never drives PPS and does not configure UM982. The timing controller
  only receives PPS. UM982/NTRIP configuration remains on the Raspberry Pi.
- All trigger outputs must power up and reset inactive. A floating, boot-glitching, or
  tri-stated camera wire is a failed design.
- Do not passively T one logic output into RGB and thermal. They share a timer event but
  require two independent protected drivers because their grounds and thresholds differ.
- Connector shells, BNC shields, signal returns, protective earth, and DC zero volts are
  not assumed to be interchangeable. Check continuity with all equipment unpowered.

## 2. Confirmed interface matrix

| Endpoint | Signal and connector | Confirmed electrical behavior | Required project behavior | Status / source |
| --- | --- | --- | --- | --- |
| UM982 module | PPS, module pin 30, single-ended relative to GND | Output; pulse width and polarity configurable. At VCC=3.3 V and 2 mA load, low is at most 0.45 V and high is at least VCC-0.45 V. | Capture the conditioned active edge with a hardware timer. Never drive this pin. | `CONFIRMED` for the bare UM982: `WTRTK-982_User Manual_EN_R1.3.pdf`, PDF pp. 13 and 15, tables 2-1 and 2-4. |
| MJRTK breakout | Header marked PPS | The reseller sheet shows PPS on pin 1 of one 6-pin group and pin 7 of another; it does not establish which physical connector the installed lead uses. | Identify the installed header by label, continuity and oscilloscope. | `UNVERIFIED`: `MIRTK-UM982 Product Specifications.pdf`, PDF p. 10. Do not infer breakout routing from the bare-module pin number. |
| UM982 PPS configuration | PPS output | Positive means active high/rising edge; negative means active low/falling edge. Width and period are configurable. Default `ENABLE` starts after fix and PPS convergence and may continue about 30 s after positioning loss. | Record measured polarity and width in the task manifest. Protocol v1 expects the conditioner to present the selected edge as rising. | `CONFIRMED`: `Unicore Reference Commands Manual For N4 High Precision Products_V2_EN_R1.2.pdf`, PDF pp. 35-36, section 4.3. Current installed configuration is `UNVERIFIED`. |
| Specim FX10e | Fischer DBPLU1031Z012\|130G power connector; pin 8 `ISO_TRIGGER`, pin 6 `ISO_GND` | Opto-isolated input, 5-15 VDC, 15 V maximum, internal constant-current diode, minimum pulse 0.2 us; maximum pulse is half the frame period. Active high or low is configurable. Input isolator delay is typically 5 us and at most 18 us. | Use the Specim external-trigger cable. Default project pulse: positive 1.0 ms, provided it remains shorter than half the configured period. | `CONFIRMED`: `fx10-user-manual.pdf`, PDF pp. 9, 27-32 and 39-41, tables 1, 5 and 7. |
| Specim SWIR PCU | Female BNC `Trig IN`; BNC center signal and shield return are assumed by connector convention | Rising edge starts integration; 5 V TTL; required pulse width 0.5-2 ms. | Dedicated output channel, positive 1.0 ms default. Use the PCU `Trig IN`, not `Chain In/Out`, until chain semantics are separately validated. | Voltage, edge and width `CONFIRMED`: `spectral-camera-swir-user-manual-2.5.pdf`, PDF p. 15, section 4. BNC center/shield assignment, input impedance, termination and isolation are `UNVERIFIED`. |
| Specim SWIR chain ports | Deutsch AS012-98PD/SD, pins E/F `TRIGGER_START`/return and G/H `PPS+`/`PPS-` | Names and connector pin assignments only. | Leave disconnected in the first implementation. | `CONFIRMED` names only: SWIR manual PDF p. 6, tables 2-3. Levels, direction behavior and isolation are `UNVERIFIED`. |
| FLIR BFS-U3-122S6C-C | Hirose HR10A-7R-6PB; pin 2 black `OPTOIN`/Line 0, pin 5 blue `Opto GND` | Opto-isolated input: low 0-1.4 V, high 2.6-30 V, input current 3.5-7 mA; propagation delay at most 18 us low-to-high and 9 us high-to-low. | Use the vendor GPIO cable and an isolated 5 V branch. Confirm the installed cable and loaded waveform before connecting. | `CONFIRMED`: Teledyne FLIR model-specific online camera reference, `Input/Output Control BFS-U3-122S6`, retrieved 2026-08-25. The local two-page sheet confirms the pin colors but omits these limits. Minimum accepted trigger pulse and isolation rating remain `UNVERIFIED`. |
| FLIR A6701 | Rear-panel BNC `SYNC IN`; center signal and shield return | Rising-edge TTL/LVCMOS; nominal 0-5.5 V, absolute -0.5 to 6.5 V; VIH=2.0 V, VIL=0.8 V; minimum width 160 ns; selectable polarity is also listed. | Use `SYNC IN`, never `TRIGGER IN`, positive 1.0 ms default. Configure external sync and read it back. | `CONFIRMED`: `FLIR_A6701_Thermal_Camera_User_Mannual.pdf`, PDF pp. 10, 14, 32-38 and 76-80, especially section 6.5.5. Input impedance, BNC termination and galvanic isolation are `UNVERIFIED`. |
| A6701 frame semantics | External frame sync | The family manual describes FSSI and FSSR, but other supplied FLIR material and the installed-node evidence do not settle which behavior is valid on this exact A6701/firmware combination. | Store `frame_sync_evidence=unverified`; validate exposure/readout phase with a scope before assigning FSSI/FSSR semantics. | `UNVERIFIED` due to vendor-source/model ambiguity. See `thermal_a6701_contract.md`. |

The model-specific Teledyne FLIR camera reference resolves the Line 0 voltage, current
and propagation-delay limits that are absent from the local model sheet. The opto-isolated
Line 0 remains the only proposed RGB trigger input; the installed cable and loaded branch
still require bench verification.

## 3. Recommended controller

### 3.1 Controller category

Use a microcontroller with all of the following:

- a free-running hardware counter of at least 32 bits, extended to 64 bits in firmware;
- hardware input capture for PPS;
- at least three independent output-compare resources: FX10e, SWIR, and snapshot;
- DMA or interrupt-driven USB CDC transport that cannot delay timer edges;
- an independent watchdog, brownout reset, unique device identity, and nonvolatile
  firmware version;
- 3.3 V logic separated from camera-facing voltage/ground domains by an interface PCB.

Recommended new-purchase prototype: **ST NUCLEO-H753ZI**, followed by a custom
STM32H743/753-based controller/interface PCB for deployment. ST now marks the
NUCLEO-H743ZI/ZI2 product page obsolete and names NUCLEO-H753ZI as its replacement, while
the STM32H743ZIT6 MCU itself remains active. Do not buy the obsolete board for a new build.
The Nucleo board is a development platform, not a field-ready industrial controller. Exact
timer instances, pin alternate functions, USB mode, oscillator tolerance, temperature range
and Windows driver behavior remain design-review items against UM2407, the board schematic,
the selected MCU datasheet and reference manual.

Acceptable proof-of-concept alternative: **Teensy 4.1 plus the same external interface
PCB**. It must not be connected bare to PPS or a camera. Its pin tolerance, timer routing,
boot behavior and long-term availability are `UNVERIFIED` here.

Do not use Arduino `delay()`, Windows USB arrival time, a Raspberry Pi userspace GPIO
loop, or four unrelated USB trigger dongles as the timing source. Those approaches do not
provide a single captured PPS timebase and correlated per-edge sequence record.

### 3.2 Logical versus physical channels

The controller has three timer schedules and four camera-facing drivers:

```text
UM982 PPS -> protected Schmitt input -> hardware capture timer

timer OC1 -> FX10e 5 V branch ----------------------> FX10e ISO_TRIGGER
timer OC2 -> SWIR 5 V branch -----------------------> SWIR PCU Trig IN
timer OC3 -> snapshot edge -> RGB protected branch -> BFS Line 0 OPTOIN
                            -> A6701 5 V branch -----> A6701 SYNC IN
```

FX10e and SWIR remain independent even when both are set to 120 Hz. RGB and thermal
receive the same output-compare transition but never share a driver, cable, connector or
ground return. Firmware reports paired RGB and thermal logical events with the same PPS
sequence and tick offset.

## 4. Interface-board electrical strategy

### 4.1 PPS input

Preferred first build, when UM982 and controller can share one deliberate star-point
signal ground:

1. PPS and its adjacent signal return enter on a keyed two-pin terminal.
2. Fit a small series resistor (candidate 100-330 ohm), a low-capacitance 3.3 V ESD
   suppressor and a 3.3 V Schmitt buffer close to the connector.
3. Feed the buffer output only to a timer input-capture pin. Do not fan PPS directly to
   cameras.
4. Keep the PPS trace away from USB, DC/DC switch nodes and camera-output edges.

Candidate Schmitt part: `SN74LVC1G17` at 3.3 V. Candidate ESD device:
`PESD3V3U1UL`. These are placeholders, not approved BOM lines: package, threshold,
clamp current, capacitance, propagation delay and temperature grade require current
manufacturer-datasheet review before schematic release.

An optocoupler is not the default PPS conditioner because its delay and temperature
variation directly degrade the time reference. If galvanic isolation is required after the
ground survey, use a low-skew digital isolator with an isolated supply on the UM982 side,
measure channel propagation across temperature, and store the measured correction and
uncertainty. `ISO7721` is a candidate family only; it is not approved without datasheet and
bench review.

### 4.2 Camera outputs

Each physical output gets its own buffer, enable/pull-down, series damping resistor, ESD
device and return terminal. Hardware pull-downs must dominate during MCU reset. Driver
enable is gated by both a firmware `RUNNING` signal and a hardware reset/watchdog signal.

- **FX10e:** 5.0 V push-pull branch into pin 8 with pin 6 as the isolated return. The
  camera already provides optical isolation; no additional isolator is required unless
  the completed cable/ground survey says otherwise. Never use CAMERA_GND as ISO_GND.
- **SWIR:** isolated 5.0 V BNC driver is preferred because PCU input isolation and
  termination are undocumented. Give this branch its own isolated 5 V DC/DC return tied
  only to the output BNC shield.
- **RGB:** isolated 5.0 V branch, initially fitted with an open enable jumper. Its loaded
  high must remain inside the documented 2.6-30 V range and supply the documented
  3.5-7 mA input current. Use pin 2 and pin 5 only; do not use VAUX, VOUT, GPIO3 or
  camera power ground.
- **A6701:** isolated 5.0 V BNC branch, hard-clamped below the 5.5 V nominal maximum.
  A 3.3 V driver is also logically valid against the documented 2.0 V VIH, but the choice
  depends on the measured input termination and cable drop.

Candidate driver families for prototype evaluation are `SN74AHCT125` for high-impedance
loads and `TC4427A` where a measured 50-ohm load requires more current. Neither may be
selected until loaded amplitude, overshoot and current have been measured. A source-series
resistor footprint should accept 0, 22, 33, 49.9 and 100 ohm values. Do not install a
49.9-ohm source resistor merely because the connector is BNC: if the camera is internally
50-ohm terminated, the resulting divider changes the high level.

### 4.3 Grounding and shielding

- Establish one controller-side chassis/star point. Do not daisy-chain camera returns.
- Coaxial BNC shield is the signal return and stays paired with its center conductor.
- FX10e `ISO_TRIGGER` and `ISO_GND` travel as a twisted pair in the vendor-approved
  external-trigger cable. Terminate an overall shield to chassis as specified by that cable,
  not to an arbitrary signal return.
- Prefer isolated DC/DC per non-isolated BNC branch. Check DC resistance between BNC
  shield, camera chassis, power negative and protective earth before deciding where an
  isolation barrier belongs.
- Keep trigger cables separate from motor, cooler, power-entry and Ethernet bundles. If
  crossing is unavoidable, cross at approximately 90 degrees.

## 5. Initial pulse plan

These are project starting values, not a substitute for readback and scope validation.

| Schedule | Starting rate | Active edge / width | Constraint |
| --- | ---: | --- | --- |
| PPS capture | 1 Hz | measured/configured edge normalized to rising | Record actual UM982 polarity and width. |
| FX10e | configurable, expected 120 Hz | rising, 1.0 ms | Width must remain below half the configured frame period and above 0.2 us. |
| SWIR | configurable, expected 120 Hz | rising, 1.0 ms | Vendor requires 0.5-2 ms. |
| Shared snapshot | configurable, default 2 Hz | rising, 1.0 ms | A6701 accepts this; BFS voltage/current are confirmed and its 1 ms trigger response remains a bench acceptance item. |

At 120 Hz the 8.333 ms period leaves 7.333 ms low time with a 1 ms pulse. Rates,
phase and width freeze for the task. Changing them requires disarm and a new configuration.

## 6. Cable and connector plan

| Run | Recommended assembly | Notes |
| --- | --- | --- |
| UM982 breakout to controller | Short twisted PPS/return pair, keyed pluggable terminal at controller | Breakout uses an `XH2.54x6Pin`-described connector, but exact mate, pin group and installed harness are `UNVERIFIED`. Buy only after physical keying and continuity check. |
| Controller to FX10e | Specim power/external-trigger cable compatible with Fischer DBPLU1031Z012\|130G | The ordinary power cable explicitly does not support external triggering. Avoid a hand-built camera-end Fischer harness for the first system. |
| Controller to SWIR | 50-ohm RG-58-class coax, BNC male to PCU `Trig IN` female | Keep `Trig OUT` visibly labeled and unused during acceptance. Input termination remains `UNVERIFIED`. |
| Controller to RGB | Teledyne FLIR ACC-01-3009 (1 m) or ACC-01-3010 (4.5 m) six-pin Hirose GPIO cable to a labeled interface | Black pin 2 is OPTOIN; blue pin 5 is Opto GND. ACC-01-3011 is the compatible loose connector, not a finished cable. Verify the received cable colors and choose length after routing survey. |
| Controller to A6701 | 50-ohm RG-58-class coax, BNC male to `SYNC IN` | Physically key/label against nearby video BNCs. Never connect to Trigger In. |

Use strain relief at both the controller enclosure and moving robot harness. Label both
ends with channel name, voltage domain and direction. Record finished cable length and
continuity in the build sheet.

## 7. Prototype BOM

Quantities are for one robot and include no spares. Candidate semiconductors require
manufacturer-datasheet approval before purchase.

| Qty | Item | Candidate / requirement | Approval state |
| ---: | --- | --- | --- |
| 1 | Timing MCU board | NUCLEO-H753ZI | Current replacement for the obsolete H743 Nucleo board; timer pin map and oscillator still require design review. |
| 1 | Custom interface PCB | PPS conditioner, four protected output branches, watchdog gate, test points | Must be reviewed before fabrication. |
| 1 | PPS Schmitt buffer | SN74LVC1G17 candidate | `UNVERIFIED` component selection. |
| 1 | PPS ESD suppressor | PESD3V3U1UL candidate plus 100-330 ohm series footprint | `UNVERIFIED` component selection. |
| 4 | Camera line-driver channels | Socket/footprint strategy for SN74AHCT125 or TC4427A-class evaluation | Final device depends on measured load. |
| 2 | Isolated 5 V DC/DC domains | One each for SWIR and A6701; reinforced/basic rating selected from ground survey | Voltage, power and isolation rating `UNVERIFIED`. |
| 4 | Output ESD and damping networks | Low-capacitance TVS plus selectable 0/22/33/49.9/100-ohm source resistor | Select from measured waveforms. |
| 1 | Hardware output gate/watchdog | Forces all drivers inactive on reset/fault | Exact implementation `UNVERIFIED`. |
| 1 | Enclosure and terminal set | Shielded metal enclosure, keyed pluggable terminals, chassis stud, strain relief | Fit to industrial PC mounting location. |
| 2 | BNC bulkhead outputs | SWIR and A6701, clearly keyed/labeled | Do not provide a passive T. |
| 2 | BNC coax assemblies | RG-58-class, male-male, length after routing survey | Verify shield continuity and no shorts. |
| 1 | Specim FX external-trigger cable | Vendor-approved cable for FX power connector | Exact sales part `UNVERIFIED`; confirm with Specim. |
| 1 | FLIR six-pin GPIO cable | ACC-01-3009 (1 m) or ACC-01-3010 (4.5 m) | Official compatible parts; select length after routing survey. |
| 1 | UM982 PPS harness | Correct mating plug plus PPS/return twisted pair | Pinout must be physically verified. |
| 1 set | Test points and dummy loads | High impedance, 1 kohm, 100 ohm and 50 ohm non-inductive loads | Bench-only acceptance aids. |

Add spare fuses, terminal labels and one spare cable only after the installed connector
part numbers are confirmed. Do not order a guessed Fischer, Deutsch or GPIO mate.

## 8. Host-controller and firmware behavior

The binary wire format remains `ppbng_timing` protocol v1. USB is transport only; all
edge generation and timestamping occur in timer hardware.

### 8.1 Host sequence

1. Open only the configured USB VID/PID/serial identity; reject ambiguous devices.
2. Send `GetVersion`; require protocol version, firmware version, boot ID, capabilities
   and nonzero tick rate.
3. Send one `ConfigureSchedule`. RGB and Thermal must have identical enable, rational
   rate, phase and pulse width because they share one physical timer edge.
4. Require ACK, then `FreezeConfiguration`; no task-critical timing value can change.
5. If `timing.pps_required` is true, send `ArmNextWholeSecond(after_pps_sequence)`;
   firmware waits for a strictly newer captured PPS. Otherwise require the advertised
   `StartAtCurrentTick` capability and send that command for the frozen schedule. Firmware
   preloads compares beyond its minimum safe lead time before opening the output gate.
6. Consume `PpsAnchor`, paired snapshot events, per-channel trigger events and periodic
   `Status`. Associate UTC on the host from GGA; do not pretend USB arrival time is PPS.
7. On normal stop send `Disarm`, require outputs inactive, drain all event records and
   preserve the final status.

Only one state-changing request is outstanding. Retries reuse the same packet sequence and
identical payload. Firmware caches and returns the original ACK/error without repeating the
transition. Reuse of one packet sequence with different payload is a protocol fault.

### 8.2 Firmware invariants

- GPIO outputs are held inactive by hardware from reset until the first armed compare.
- PPS uses hardware input capture. Output events use hardware output compare; a USB ISR
  cannot directly toggle a camera pin.
- Rational rates use an integer phase accumulator to avoid cumulative period rounding.
- The edge ISR records boot ID, global and channel sequences, PPS sequence, absolute tick,
  offset ticks and lock state before deferred USB transmission.
- One snapshot compare creates exactly one physical source transition, two independently
  buffered electrical outputs, and two logical records with equal PPS/tick fields.
- Loss of PPS transitions `LOCKED -> HOLDOVER -> UNSYNCED` without inventing a UTC label.
  Triggers may continue according to the frozen policy, and every event carries quality.
- In explicit no-PPS mode, pre-PPS events use PPS sequence zero, `UNSYNCED`, and retain the
  absolute controller tick. A late PPS is recorded but never silently shifts the running phase.
- A controller reset changes boot ID. Counters restarting without a changed boot ID is a
  firmware fault.
- Event-ring overflow, timer inconsistency, watchdog reset or driver-gate fault latches
  `FAULT` and forces all outputs inactive. It is never reported as a harmless packet drop.
- Firmware does not know UTC because it does not parse GGA. Controller-originated
  `PpsAnchor.utc_second` is `INT64_MIN`; the host records its later GNSS association.
- The final production host-liveness timeout is `UNVERIFIED`. Protocol v1 can treat empty
  `Status` requests as liveness polls, but the timeout and stop/continue policy must be an
  explicit pre-firmware-release decision and tested for Windows USB stalls.

### 8.3 Pure-model guarantees

The current pure C++ model tests cover packet CRC/framing, immutable frozen schedules,
whole-PPS arming, PPS holdover, counter regression/restart and rejection of unequal
RGB/Thermal schedules. Hardware timer, GPIO, USB and watchdog behavior still require
firmware-in-the-loop tests.

## 9. Oscilloscope acceptance procedure

Use a recently calibrated oscilloscope with at least four channels, 10x high-impedance
probes, short ground springs, a differential probe where grounds differ, and switchable
50-ohm termination. Never attach a grounded probe clip across an unverified isolated
domain.

### Phase A - unpowered inspection

1. Photograph and record every model, serial, connector label and cable part number.
2. With all devices unpowered, measure continuity among each BNC shield, camera chassis,
   camera power negative, PC chassis, controller ground and protective earth.
3. Verify the MJRTK header PPS and return by schematic/label/continuity. Resolve whether
   the installed breakout uses the sheet's first or second six-pin group.
4. Confirm all controller outputs have physical pull-downs and no continuity to an
   unintended power rail.

### Phase B - controller only

1. Power the controller without cameras. During power-on, reset, firmware update, USB
   attach/detach and watchdog reset, verify every output remains inactive.
2. Drive PPS input from an isolated laboratory pulse generator first. Sweep documented
   UM982 low/high levels and verify one capture per edge with no double triggering.
3. Exercise each output into high impedance, 1 kohm, 100 ohm and 50 ohm dummy loads.
   Record open/loaded high, low, rise/fall time, overshoot, undershoot and pulse width.
4. Verify FX and SWIR schedules are independent. Verify the two snapshot branches have
   one timer edge and measure branch-to-branch skew. Initial project target: at most 1 us;
   this is an engineering target, not a vendor guarantee.

### Phase C - live PPS, no cameras

1. Observe raw breakout PPS with a high-impedance probe. Record amplitude, inactive level,
   active edge, width, 1 Hz period and behavior during GNSS fix loss.
2. Simultaneously observe raw PPS and conditioned timer input. Measure propagation delay
   and jitter for at least 1,000 pulses. Record temperature and probe setup.
3. Verify the host records monotonically increasing PPS sequence and captured ticks while
   artificial USB load changes only delivery latency, not tick offsets.

### Phase D - one camera at a time

1. Power down before connecting each cable. Start with an intentionally low trigger rate
   such as 1 Hz; for snapshots, 0.2-1 Hz is acceptable.
2. Confirm at the camera connector that voltage, polarity and width remain within its
   confirmed limits. For RGB, determine and approve its limits before enabling the branch.
3. Confirm exactly one accepted frame per pulse using camera frame IDs and missed-trigger
   counters. A scope waveform alone does not prove frame acceptance.
4. Where a camera exposes strobe/integrate-active/sync-out, observe trigger and response
   together. Measure latency distribution and polarity; do not infer exposure start solely
   from host frame arrival.
5. For A6701, separately test candidate Integration/FSSI and Readout/FSSR settings at a low
   rate. Record which nodes exist, their readback, the first-frame behavior after a pause,
   and trigger-to-integrate/sync-out timing. Until then the semantic result is
   `UNVERIFIED`.

### Phase E - integrated acceptance

1. Run FX10e and SWIR at the intended rates and snapshots at 2 Hz. Observe PPS, FX,
   SWIR and snapshot on four channels for phase and drift.
2. Verify paired RGB/Thermal logical records have equal PPS/tick fields and measured
   physical branch skew stays inside the accepted project target.
3. Run at least 30 minutes with zero unexplained missed triggers, duplicate sequence,
   counter regression or event-ring overflow before a short dataset test.
4. Disconnect and reconnect one camera cable at a time. Other channels must continue;
   the affected stream starts a new segment. Disconnecting or resetting the timing
   controller must cause the global controlled-stop policy.
5. Repeat power-cycle and two-hour soak tests after the short tests pass. Store scope
   captures, firmware hash, interface-board revision, resistor options, cable lengths and
   measured delay/uncertainty in the acceptance record.

## 10. Open decisions and mandatory verification

The following are blockers for a production wiring release:

1. Exact MJRTK breakout connector and PPS/return pins on the installed unit.
2. Actual UM982 PPS enable mode, polarity, width, voltage and loss-of-fix behavior.
3. BFS Line 0 selected cable length, loaded 5 V waveform, polarity/configuration and accepted
   1 ms trigger response (compatible cable part numbers and voltage/current limits are now documented).
4. SWIR `Trig IN` input impedance, termination, center/shield definition and isolation.
5. A6701 `SYNC IN` input impedance/termination and measured FSSI versus FSSR behavior.
6. Finished cable lengths, shield/chassis continuity and ground-loop survey.
7. Controller timer/pin assignment, oscillator accuracy, output-driver choice and loaded
   waveforms.
8. Host-liveness watchdog interval and whether holdover continues indefinitely or stops at
   a configured bound.
9. Quantitative end-to-end timing uncertainty budget and acceptance limit.

No item above may be filled with a guessed value in firmware or configuration.

## 11. Local source register

- `PPBv2_20260825/dual_GPS/MJRTK-UM982/WTRTK-982_User Manual_EN_R1.3.pdf`
- `PPBv2_20260825/dual_GPS/MJRTK-UM982/Unicore Reference Commands Manual For N4 High Precision Products_V2_EN_R1.2.pdf`
- `PPBv2_20260825/dual_GPS/MJRTK-UM982/MIRTK-UM982 Product Specifications.pdf`
- `docs/Hyper_Spec_Cam_Specim_FX10e/Manuals/fx10-user-manual.pdf`
- `docs/Hyper_Spec_Cam_Specim_SWIR/spectral-camera-swir-user-manual-2.5.pdf`
- `docs/RGB_Cam_FLIR_BFS_U3/BFS_U3_Getting_Started.pdf`
- `docs/RGB_Cam_FLIR_BFS_U3/FLIR_Blackfly_S_BFS-U3-122S6.pdf`
- `docs/Thermal_Cam_FLIR_A6701/FLIR_A6701_Thermal_Camera_User_Mannual.pdf`
- `ppbng_ros2_ws/docs/thermal_a6701_contract.md`
- `ppbng_ros2_ws/docs/architecture.md`

Online manufacturer source added after the local-document review:

- [Teledyne FLIR: Input/Output Control, BFS-U3-122S6](https://softwareservices.flir.com/BFS-U3-122S6/latest/40-Installation/InputOutputControl.htm), retrieved 2026-08-25.
- [Teledyne FLIR: GPIO cables with 6-pin Hirose HR10 connector](https://www.teledynevisionsolutions.com/en-ca/products/hirose-hr10-6-pin-circular-connector/), retrieved 2026-08-25.
- [ST: NUCLEO-H743ZI product page and replacement notice](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html), retrieved 2026-08-25.
- [ST UM2407: STM32H7 Nucleo-144 boards](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-board-stmicroelectronics.pdf), Rev. 6, April 2026.
- [ST: STM32H743ZI active MCU product page](https://www.st.com/en/microcontrollers-microprocessors/stm32h743zi.html), retrieved 2026-08-25.
