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
