# FLIR A6701 external-trigger validation — 2026-08-27

## Result

The exact A6701 accepted the Arduino UNO R4 WiFi D12 signal on the rear-panel `SYNC IN`
and produced ten complete, consecutive frames from a 2 Hz, 1 ms, active-high pulse
source. Full raw transport payloads and previews were preserved under:

`hardware_test_data/thermal_a6701_external_20260827_171424/`

This is a successful low-rate functional test. It is not yet the 1,000-pulse timing
qualification or oscilloscope proof required to label FSSI exposure-start semantics as
confirmed production timing evidence.

## Electrical checks

The installed FLIR manual specifies `SYNC IN` as a rising-edge TTL/LVCMOS input,
nominal 0–5.5 V, absolute -0.5–6.5 V, `Vih=2.0 V`, `Vil=0.8 V`, and minimum pulse width
160 ns. It does not specify input impedance or termination.

The test firmware therefore defaulted D12 low and implemented `LOADTEST` with the UNO's
weak internal pull-up. Results were:

- D12 unconnected: 32/32 high samples.
- D12 plus unterminated BNC cable: 32/32 high samples after the RGB branch was removed.
- D12 plus A6701 `SYNC IN`: 32/32 high samples.

This excludes a low-impedance termination detectable by the weak pull-up for this
connected A6701 input. The RGB camera must not be passively paralleled onto the same
GPIO; the earlier parallel connection pulled the test line low. Production fan-out
still requires independent buffered/isolated branches.

## Misconnection incident and checks

During the first attempt, the BNC was accidentally connected to the A6701 `VIDEO OUT`
instead of `SYNC IN`. The Arduino emitted 11 pulses before software stop. This was an
output-to-output connection and is not electrically approved. After disconnection:

- the Arduino continued to respond to serial commands and held D12 low;
- the A6701 initialized normally over GigE;
- a fresh internal/free-run test captured 20/20 complete frames at approximately 60 Hz;
- all frame geometry and payload checks passed;
- the subsequent external-sync test on the correct port succeeded.

These checks show no observed damage to the Arduino GPIO or the A6701 GigE acquisition
path. They do not prove that the independent HD-SDI `VIDEO OUT` driver remains healthy.
If that port will be used, validate it later with a proper 75-ohm HD-SDI monitor or
capture device, never with a GPIO.

## Failed attempt retained

The failed wrong-port attempt is retained at
`hardware_test_data/thermal_a6701_external_20260827_171028/`. It contains only a
249-byte configuration record and no raw frame. The camera timed out waiting for image
buffers and the program attempted transactional restoration.

## Successful capture evidence

- DeviceID: `00111C0408CD`
- CameraModel: `A6701`
- Configuration during capture: External / Integration / ActiveHigh
- Arduino: D12, 2 Hz, 1 ms high pulse
- Complete raw frames: 10/10
- Frame IDs: 1 through 10, no discontinuity
- Geometry: 640 x 513, stride 1280 bytes
- Payload: 656,640 bytes per frame
- Unique full-payload hashes: 10/10
- Mean camera timestamp interval: 500.302962 ms
- Minimum interval: 500.284530 ms
- Maximum interval: 500.326920 ms
- Population standard deviation: 0.013322 ms

The frame interval closely follows the Arduino schedule, but the UNO serial pulse log
and camera timestamps do not share a qualified clock. Do not interpret the 13 us
interval standard deviation as physical trigger jitter without an oscilloscope or
logic-analyzer measurement.

## Post-test state

Arduino serial readback after stop:

- `STOPPED`
- D12 output restored low

Independent A6701 readback after capture:

- `FrameSyncSource=Internal`
- `FrameSyncMode=Integration`
- `FrameSyncPolarity=ActiveHigh`
- `TriggerMode=FreeRun`
- `TriggerSource=Internal`
- `Ready=1`
- `FPACold=1`

## Clean-restart recovery regression

After a clean Windows and A6701 restart, the corrected allow-list-only health
check passed three consecutive Init/DeInit cycles with no residual process.
An internal acquisition test then passed 20/20 frames, followed by five more
open/acquire/stop/close cycles totaling 100/100 complete frames. Core A6701
nodes remained readable after every completed operation.

A fresh external-trigger regression was then recorded in
`hardware_test_data/thermal_a6701_external_recovery_20260827_175238`:

- Arduino was confirmed STOPPED before arming the camera.
- START was sent only after the camera reported `ARMED_WAITING_FOR_EXTERNAL_PULSES`.
- Arduino emitted exactly 10 pulses; A6701 returned 10/10 complete frames.
- Frame IDs were consecutive 1 through 10; all raw files were 656,640 bytes.
- All 10 full-payload hashes were unique.
- Camera timestamp interval mean was 500.296038 ms, with 500.265195 ms minimum
  and 500.324205 ms maximum.
- The capture restored Internal / Integration / ActiveHigh and FreeRun state.
- Post-capture health check returned `CameraModel=A6701`, `Ready=1`, and
  `FPACold=1`.
- Arduino ended STOPPED with pulse_count=10; no related process remained.
