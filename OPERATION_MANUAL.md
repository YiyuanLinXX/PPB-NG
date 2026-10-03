# PPB-NG Operation Manual

For system overview and module documentation, see the [README](README.md). The Windows acquisition workspace is `PPBNG_Imaging/`; Raspberry Pi navigation code belongs in `PPBNG_Navigation/`. From the repository root, enter the acquisition workspace in Windows PowerShell before running the commands below:

```powershell
cd .\PPBNG_Imaging
```

All command paths and configuration paths below are relative to `PPBNG_Imaging/`.

## Configure and build

Install ROS 2 Jazzy, Visual Studio 2019 x64, Spinnaker, SpecSensor and the required Pleora/NI frame-grabber drivers. Vendor software, licenses and camera calibration packs are not included. Review local SDK paths in `tools/build*.cmd`, `tools/ros2.cmd` and package CMake files on a different computer.

Edit `src/ppbng_bringup/config/ppbng_config.yaml`: output directory, device IDs, COM ports, calibration paths, frame rates and exposures. The supplied configuration retains the original robot's settings, including absolute paths; it is not a portable plug-and-play configuration. Configure a new output directory and qualify its disk before using this workspace on another installation. Do not overwrite or delete the working workspace's data.

```powershell
.\tools\build_production.cmd
.\tools\test.cmd
.\tools\plan_storage_qualification.cmd
```

The planner is read-only. Review and explicitly approve its printed disk-write test, then set `session.throughput_evidence_path` to the resulting local evidence. Do not copy another disk's qualification result or bypass free-space checks. Do not copy `build/` or `install/` between workspaces: rebuild here.

## Collect

Warm up the thermal camera; close HyperFusion/eBUS/SpinView and serial monitors. Start the Raspberry Pi base/navigation safety stack, but keep the robot stopped. From this workspace in PowerShell:

```powershell
.\tools\run_production_acquisition.cmd field_01 60
```

`field_01` is the dataset name; `60` is the maximum scene duration in minutes. Cover both HSI lenses at the first prompt, press Enter once, then remove the covers and confirm at the second prompt. Move only after recording begins and the navigation safety checks permit it. Stop robot motion first; press Enter or Ctrl+C once in the acquisition terminal and wait for ordered shutdown.

Never launch two acquisition instances. A red alert requires operator attention; do not bypass the motion interlock or repeatedly restart against open hardware. Configuration edits apply on the next launch, with no rebuild.

## Configuration essentials

- `hsi.fx10e.exposure_us` / `hsi.swir.exposure_us`: microseconds, equal to the HyperFusion value in milliseconds multiplied by 1000. Change while stopped; acquire new dark references after exposure/binning changes.
- `line_rate_hz` and exposure must satisfy camera timing limits. Keep `timing_policy: clamp_exposure`; inspect saved actual settings after startup.
- HSI runs continuously with internal timing; RGB/thermal use UNO external triggers. Do not assume the line cameras share a hardware trigger.
- Default operation is without PPS. Host-time association is not precise exposure-time UTC synchronization. Do not treat unsynced timestamps as locked.
- UM982 is receive-only on this PC. RSM400 attitude describes its platform, not the robot chassis or the independently mounted RGB/thermal cameras.

## Network and robot safety

The supplied configuration uses domain 47 and Cyclone DDS, Windows robot LAN `192.168.5.150` and Raspberry Pi `192.168.5.200`. Adjust `src/ppbng_bringup/config/cyclonedds_robot.xml` and the wrapper/launch peer settings together if addresses change. Camera NICs are separate networks; do not alter their bindings or IPs while acquiring. Network configuration scripts are optional administrator setup tools, not daily startup commands.

The Pi subscribes to `/ppbng/safety/thermal_motion_permitted` (`std_msgs/msg/Bool`, reliable, transient-local, depth 1). False or stale messages must inhibit motion. The Pi must implement an independent timeout watchdog and retain its RTK and emergency-stop logic. This topic now also denies motion for acquisition/device health failures, despite its thermal-specific name. NUC normally tags data and inhibits motion without pausing other sensors. Verify the cross-host stop behavior before allowing autonomous movement.

## View and export

```powershell
.\tools\setup_export_python.cmd
.\tools\export_snapshot_images.cmd "C:\path\to\session" --output-directory "C:\exports\session_images" --rgb-color-preview-every 1 --thermal-false-color-preview-every 1
.\tools\export_hsi_rgb_full.cmd "C:\path\to\session" --calpack "C:\calibration\matching_fx10e.scp"
```

Use finalized sessions and new export directories. See [Data export](PPBNG_Imaging/docs/DATA_EXPORT.md).

## Verify a session

After ordered shutdown, run the read-only dataset verifier:

```powershell
.\tools\verify_dataset.cmd "C:\path\to\session"
```

Keep the manifest, raw payloads, configuration snapshot, calibration metadata and sidecars together. The current configuration sets `session.hsi_payload_crc_enabled: false`, so HSI payload corruption cannot be detected by a stored per-line CRC. With `session.hsi_flush_every_lines: 0`, headers and checkpoints update at part boundaries and orderly close; an abrupt stop may leave the active part header stale. Review verification output alongside runtime warnings and recorded time-quality flags.

## Diagnostics

`tools/run_four_camera_test.cmd` is an optional camera-only diagnostic. Never run it alongside production acquisition. The portable timing-controller core and protocol v1 documentation describe a separate implementation; the supplied robot configuration uses the UNO R4 ASCII backend.
