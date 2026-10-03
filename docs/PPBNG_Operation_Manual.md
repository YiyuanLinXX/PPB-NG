![Logo_CAIR_Text_Horizontal](../assets/Logo_CAIR_Text_Horizontal.png)



# PPBNG Operation Manual

Internal operator guide · Latest update by [Yiyuan Lin](mailto:yl3663@cornell.edu) on Oct 03, 2026

For assistance, contact Yiyuan Lin at yl3663@cornell.edu or (607) 339-6758



> [!NOTE]
>
> ***Remote Desktop Connection*** is a remote desktop software on the <u>Windows laptop</u> for remote control.
>
> ***HyperFusion*** is a software on the <u>PPBNG PC</u> for hyper spectral cameras control and visualization.



## 0. Remote in the PPBNG PC

Connect to the robot network `RUT_xxxx`. Then open ***Remote Desktop Connection***. Connect to 192.168.5.150` (PPBNG PC).



## 1. Set exposure

Open ***HyperFusion***. Adjust FX10e exposure, then close ***HyperFusion***.

Open this file (pinned to quick access in the File Explorer):

```powershell
C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\src\ppbng_bringup\config\ppbng_config.yaml
```

Edit line 165, `hsi → fx10e → exposure_us`, and save. YAML value = ***HyperFusion*** exposure in ms × 1000.

Example: **4.5 ms → 4500.0**. Line number checked September 26, 2026; locate the key if lines change.



## 2. Open three terminals (PowerShell)

Right click the Windows Start icon → Windows PowerShell. Open three windows.

> [!WARNING]
>
> Do **NOT** open the Windows PowerShell (Admin)



## 3. PowerShell 1: Raspberry Pi basic bringup

```powershell
ssh cairlab@192.168.5.200
```

After login, run:

```bash
ros2 launch amiga_navigation basic_bringup.launch.py
```

Leave it running. Confirm RTK Fixed and the thermal safety monitor is running.



## 4. PowerShell 2: Windows acquisition

Run this single command:

```powershell
"C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\tools\run_production_acquisition.cmd" trams_20261001_row1 60
```

> [!TIP]
>
> Change `trams_20261001_row1` to your dataset name. 60 is the maximum recording duration in minutes.

> [!IMPORTANT]
>
> 1. At the cover prompt: cover both HSI lenses, then press `Enter` **==once==**.
> 2. After dark capture: remove both covers, then press `Enter` **==once==**.
> 3. Wait for `Full acquisition started`.



## 5. PowerShell 3: start waypoint follower

```powershell
ssh cairlab@192.168.5.200
```

After login, wait for `Full acquisition started` in PowerShell 2, then run:

```bash
ros2 run amiga_navigation waypoint_follower --waypoints /home/cairlab/navigation_waypoints/traminette_row1.csv
```

> [!TIP]
>
> Change `traminette_row1.csv` to your waypoint file name.

## 6. Stop

Press `Enter` (recommended) or `CTRL+C` **once** in PowerShell 2 to stop data acquisition. Robot will detect the sensor closure and stop navigation. Then press `CTRL+C` in PowerShell 1 and PowerShell 3. Put the covers back to the camera lenses.