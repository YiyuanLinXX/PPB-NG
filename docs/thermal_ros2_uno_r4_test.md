# A6701 + Arduino UNO R4 standalone ROS 2 test

This launch runs only the FLIR A6701 (`DeviceID=00111C0408CD`) and the UNO R4 on
`COM9`. It does not open the USB RGB camera or start any other PPB-NG sensor.

## Before starting

- A6701 is powered, cold, and reports READY.
- Ethernet is connected and the camera is reachable.
- UNO R4 contains `firmware/arduino_uno_r4_a6701_trigger_test/`.
- D12 goes only to A6701 `SYNC IN`; Arduino GND goes to the BNC shield.
- Close SpinView, Research Studio, Arduino Serial Monitor, and any previous test.
- Use a new dataset name containing only letters, digits, `_`, or `-`.

## Timed stability run (recommended)

From a normal PowerShell or Command Prompt (administrator is not required):

```powershell
cd C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws
.\tools\ros2.cmd launch ppbng_thermal a6701_uno_r4_test.launch.py dataset_name:=thermal_soak_01 duration_sec:=3600
```

This example runs for one hour. Use `duration_sec:=7200` for two hours. A value of
zero runs until Ctrl+C:

```powershell
.\tools\ros2.cmd launch ppbng_thermal a6701_uno_r4_test.launch.py dataset_name:=thermal_manual_01 duration_sec:=0
```

Do not close the terminal window. For the unlimited run, press Ctrl+C once and wait
for `configuration_restored=true`. The node sends UNO `STOP` before ending camera
acquisition. UNO independently forces D12 LOW if host keepalives disappear for 3 s.

## Data and timing meaning

The new directory is printed under `hardware_test_data/thermal_ros2_<name>_<time>/`.
It contains full 640x513 raw payloads, `frames.ndjson`, `arduino_events.ndjson`,
`calibration_metadata.json`, `timing_summary.json`, and `capture_complete.json`.

At 2 Hz, raw data is about 4.73 GB/hour (4.40 GiB/hour) before any PNG products. The node refuses
to continue below 10 GiB free space.

Without GPS/PPS, the test can verify:

- one Arduino pulse per camera frame;
- missing/duplicate sequence numbers;
- Arduino tick period and jitter;
- camera timestamp period and jitter;
- host receive monotonic timing and clean shutdown.

It cannot verify absolute UTC accuracy or camera-to-GPS offset. Therefore ROS
`FrameMetadata.time_quality` is intentionally `UNSYNCED`; no fabricated UTC is used.
When GNSS/PPS is later connected, the same records can be anchored to UTC and the
end-to-end offset/uncertainty can be measured.

## Preview conversion

After stopping, substitute the printed dataset path. This streaming converter is safe
for multi-hour datasets and creates both fixed-scale count PNGs and, when the same
session contains calibration metadata, apparent-temperature PNGs:

```powershell
.\tools\convert_thermal_raw_to_png.cmd "<DATASET_DIRECTORY>"
```

For a quick whole-session preview, convert every 60th frame:

```powershell
.\tools\convert_thermal_raw_to_png.cmd "<DATASET_DIRECTORY>" --every 60
```

The conversion excludes the first transport row, which is the FLIR GigE header, and
uses the following 512 rows as the image. It performs a numerical quality scan of all
raw frames and records zero/uniform/out-of-calibration frames in
`quality_flags.ndjson`. Raw files remain authoritative.

## Temperature metadata and calibration workflow

Every run now records the active factory calibration coefficients and bounds plus
emissivity, reflected temperature, atmospheric temperature, distance, humidity,
estimated transmission, and external-optics settings before acquisition. These
values are read only and the raw radiometric counts are retained.

For defensible object temperatures:

1. Use the factory calibration matching the installed 17 mm lens, empty filter
   position, and expected temperature range.
2. Before a field task, set and lock the environmental/object parameters. For mixed
   outdoor materials, emissivity is object-dependent; one global value cannot make
   every pixel a true surface temperature.
3. Record a calibrated blackbody at two or more temperatures spanning the expected
   range, after warm-up and at the working geometry. Spectralon is a reflectance
   standard for the HSI cameras; it is not a thermal blackbody reference.
4. Preserve raw counts plus the session calibration snapshot. Post-processing must
   convert counts to radiance with the saved factory coefficients, compensate for
   emissivity/reflection/atmosphere/external optics, then convert corrected radiance
   to temperature. The current PNG preview is deliberately not labeled as full
   object temperature.

The FLIR Science Camera SDK is the preferred vendor-supported post-processing route.
Until its conversion is implemented and checked against a calibrated blackbody, the
system may report apparent factory-calibrated temperature but must not claim
traceable true object surface temperature.

## Verified recovery behavior

The controlled recovery suite passed on 2026-08-27:

- a commanded four-second loss of external sync produced no frames, then resumed
  without reopening the A6701;
- withholding keepalive caused UNO to report `STOPPED,host_watchdog_timeout`, after
  which triggering and capture resumed without reopening the A6701;
- final cleanup restored `FrameSyncSource=Internal`, left `Ready=1` and `FPACold=1`,
  and left no PPBNG camera process running.

Physical cable removal, Ethernet removal, and camera power loss are deliberately not
part of this safe automatic test. Those require an operator present and will be
tested separately before production deployment.
