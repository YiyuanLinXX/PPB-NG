# PPB-NG Imaging

This directory is the Windows ROS 2 Jazzy acquisition workspace for PPB-NG. It contains the hyperspectral, RGB, thermal, GNSS, stabilization-platform telemetry, and trigger packages. Raspberry Pi navigation code belongs in [PPBNG_Navigation](../PPBNG_Navigation/README.md).

For system information, see the [project README](../README.md). Follow the [operation manual](../OPERATION_MANUAL.md) for SDK installation, machine configuration, storage qualification, acquisition, and safe shutdown. Image conversion is covered by the [Data Export Guide](docs/DATA_EXPORT.md).

## Build and Run

From the repository root in Windows PowerShell, enter this workspace before building or running acquisition:

```powershell
cd .\PPBNG_Imaging
.\tools\build_production.cmd
.\tools\test.cmd
.\tools\run_production_acquisition.cmd field_01 60
```

Complete the setup in the operation manual before starting hardware. Configure device identities, ports, output paths, and calibration paths in [ppbng_config.yaml](src/ppbng_bringup/config/ppbng_config.yaml). Build outputs belong in this workspace's `build/`, `install/`, and `log/` directories. Rebuild after relocating a workspace; do not reuse build or install directories from the previous location.

## Workspace Layout

- `src/`: 11 production ROS 2 packages and regression tests.
- `tools/`: build, acquisition, configuration, verification, and export commands.
- `firmware/`: UNO R4 RGB/thermal trigger firmware.
- `docs/`: data export documentation.

Paths such as `tools/` and `src/` in package documentation are relative to this workspace unless explicitly stated otherwise. The command wrappers resolve the workspace from their own location.

## Known Bug
The command may fail with an error the first time it is executed. The cause of this issue is currently unknown. If this happens, simply run the same command again. Subsequent executions should work normally.
```powershell
PS C:\Users\cairlab> & "C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\tools\run_production_acquisition.cmd" concord_20261022 60
Frozen run config: C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\logs\production_20261022_135715\machine_config.effective.yaml (storage plan: 60 minute(s))
Starting all ROS 2 system nodes. Do not press keys until the HSI dark prompt appears.
Live launch log: C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\logs\production_20261022_135715\launch.stdout.log
Calling production control: status ...
Still waiting for production control status (5 s elapsed)...
Still waiting for production control status (10 s elapsed)...
Still waiting for production control status (15 s elapsed)...
success=true
message=state=0;session_id=;prepare_services_ready=false;missing_prepare_services=/fx10e/prepare,/swir/prepare,/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare,/context/prepare;operator_warning_count=0;operator_warnings_hex=;detail=inert
Waiting for required device services: /fx10e/prepare,/swir/prepare,/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare,/context/prepare
Calling production control: status ...
success=true
message=state=0;session_id=;prepare_services_ready=false;missing_prepare_services=/fx10e/prepare,/swir/prepare,/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare,/context/prepare;operator_warning_count=0;operator_warnings_hex=;detail=inert
Calling production control: status ...
success=true
message=state=0;session_id=;prepare_services_ready=false;missing_prepare_services=/fx10e/prepare,/swir/prepare,/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare,/context/prepare;operator_warning_count=0;operator_warnings_hex=;detail=inert
Calling production control: status ...
success=true
message=state=0;session_id=;prepare_services_ready=false;missing_prepare_services=/fx10e/prepare,/swir/prepare,/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare,/context/prepare;operator_warning_count=0;operator_warnings_hex=;detail=inert
Calling production control: status ...
success=true
message=state=0;session_id=;prepare_services_ready=false;missing_prepare_services=/fx10e/prepare,/swir/prepare,/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare,/context/prepare;operator_warning_count=0;operator_warnings_hex=;detail=inert
Calling production control: status ...
success=true
message=state=0;session_id=;prepare_services_ready=false;missing_prepare_services=/rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare;operator_warning_count=0;operator_warnings_hex=;detail=inert
Waiting for required device services: /rgb/prepare,/thermal/prepare,/gnss/prepare,/rsm400/prepare,/timing/prepare
Calling production control: status ...
success=true
message=state=0;session_id=;prepare_services_ready=true;missing_prepare_services=;operator_warning_count=0;operator_warnings_hex=;detail=inert
All required device preparation services are discovered and ready.
ROS 2 launch is alive; contacting the acquisition manager...
Calling production control: start ...
accepted=true
duplicate_request=false
session_id=7ac115b0-b439-407c-b183-2ab7a0ee481b
message=bind inert storage devices to the existing session
session_directory=C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\data\concord_20261022__20261022T175747Z__7ac115b0-b439-407c-b183-2ab7a0ee481b

Initializing devices in safe order. Dual-HSI SDK startup may take 30-60 seconds.

!!!!!!!!!!!!!!!! DATA ACQUISITION ALERT 13:57:53 !!!!!!!!!!!!!!!!
timing:UNO_R4_TRIGGER_FATAL: timeout waiting for 'STOPPED' after STOP
timing: controller already closed
motion_interlock:MOTION_INTERLOCK_LATCHED: timing: UNO_R4_TRIGGER_FATAL
Data may be incomplete. Other streams may still be recording. Inspect the affected device.
Press ENTER or Ctrl+C once for an ordered stop. This alert does not command robot motion.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
PRODUCTION ACQUISITION FAILED: Acquisition manager entered FAULT while waiting for WAITING_FOR_DARK.

!!!!!!!!!!!!!!!! DATA ACQUISITION ALERT 13:57:53 !!!!!!!!!!!!!!!!
timing:UNO_R4_TRIGGER_FATAL: timeout waiting for 'STOPPED' after STOP
timing: controller already closed
motion_interlock:MOTION_INTERLOCK_LATCHED: timing: UNO_R4_TRIGGER_FATAL
Data may be incomplete. Other streams may still be recording. Inspect the affected device.
Press ENTER or Ctrl+C once for an ordered stop. This alert does not command robot motion.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
WARNING: The manager already completed its fault-stop sequence; all opened devices were closed.
```