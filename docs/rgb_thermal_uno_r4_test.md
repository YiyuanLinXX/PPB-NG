# RGB + thermal synchronized UNO R4 test

This bench launch captures the installed FLIR Blackfly S BFS-U3-123S6C and
FLIR A6701 from the same UNO R4 trigger event.

## Wiring

- UNO D11 -> Blackfly S black wire, pin 2 `OPTOIN` / Line 0.
- UNO GND -> Blackfly S blue wire, pin 5 `Opto GND`.
- UNO D12 -> A6701 `SYNC IN` BNC center.
- UNO GND -> A6701 `SYNC IN` BNC shield.

Upload the firmware in
`firmware/arduino_uno_r4_rgb_thermal_trigger/arduino_uno_r4_rgb_thermal_trigger.ino`.
Close Arduino Serial Monitor before starting ROS 2.

## Run

From a normal PowerShell terminal:

```powershell
cd C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws
.\tools\ros2.cmd launch ppbng_thermal rgb_thermal_uno_r4_test.launch.py `
  dataset_name:=rgb_thermal_test_01 `
  duration_sec:=60
```

`duration_sec:=0` runs until Ctrl+C. Press Ctrl+C once and wait for the final
`configuration_restored=true` message before closing the terminal or removing
power. The node sends Arduino `STOP` before ending either camera acquisition.

Override `serial_port`, `thermal_device_id`, or `rgb_device_id` on the command
line only if the verified hardware identity has changed.

The vineyard-oriented RGB defaults are `rgb_exposure_auto:=Continuous`,
`rgb_gain_auto:=Continuous`, `rgb_balance_white_auto:=Once`, and
`rgb_balance_white_auto_profile:=Outdoor`. All four are launch arguments and
can be overridden explicitly. There is no hidden software pre-roll: the
operator may wait for auto controls to settle, and those early frames remain
preserved with their actual settings.

For the verified PPBNG travel speed of 0.2 m/s, the defaults also set
`rgb_auto_exposure_control_priority:=Gain`,
`rgb_auto_exposure_time_upper_limit_us:=5000`, and
`rgb_auto_exposure_gain_upper_limit_db:=12`. Five milliseconds corresponds to
about 1 mm of platform travel. Hardware validation showed that the auto loop
approaches these limits gradually: old settings can remain above the new limit
for the first few frames, so keep the robot stationary until the recorded
values settle. In the 2026-08-27 bench test, one-shot white balance changed to
Off after about 10 frames / 5 seconds.

## Output

One new directory is created under `hardware_test_data`:

```text
rgb_thermal_ros2_<dataset>_<timestamp>/
  arduino_events.ndjson
  synchronized_pairs.ndjson
  capture_complete.json
  thermal/
    calibration_metadata.json
    frames.ndjson
    frame_..._640x513_mono16.raw
  rgb/
    camera_metadata.json
    frames.ndjson
    frame_..._4096x3000_bayerrg8.raw
```

Each pair record carries the common Arduino pulse sequence and rise tick. The
two camera timestamp counters have independent epochs and must not be directly
subtracted. The thermal converter can be pointed at the `thermal` subdirectory.

RGB `frames.ndjson` stores hardware Chunk Data for every frame:
`chunk_frame_id`, `chunk_timestamp`, `exposure_time_us`, `gain_db`, and
`black_level`. `white_balance_red` and `white_balance_blue` are immediate
post-frame node-map readbacks because BFS-U3-123S6C does not expose those ratios
as image chunks. `settings_source` preserves this distinction. Image API frame
IDs and Chunk frame IDs are retained separately; hardware validation showed a
constant counter offset rather than equality, while their timestamp values
were equal.
