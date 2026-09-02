# A6701 manual focus test

This workflow continuously records externally triggered A6701 frames while an
operator adjusts the 17 mm lens. It saves every raw frame, a lossless 16-bit
counts PNG, a false-color focus PNG, frame metadata, and a simple focus metric.

## Safety contract

- Exact camera DeviceID: `00111C0408CD`; the USB RGB camera is never opened.
- Camera is armed before Arduino output starts.
- Arduino D12: 2 Hz, 1 ms active-high pulse.
- Arduino stops D12 if `KEEPALIVE` is absent for 3 seconds.
- Camera recorder stops if the host heartbeat is absent for 5 seconds.
- Press Enter or Ctrl+C in the controlling terminal for an orderly stop.
- Arduino is stopped before the camera receives the stop request.
- The camera is restored to its prior frame-sync configuration before DeInit.
- Recording refuses to continue if available storage falls below 10 GiB.

## One-time firmware update

In Arduino IDE, select **Arduino UNO R4 WiFi** and upload:

`firmware/arduino_uno_r4_a6701_trigger_test/arduino_uno_r4_a6701_trigger_test.ino`

The updated `STATUS` response must include `watchdog_ms=3000`. The focus runner
checks this and refuses to arm the camera when old firmware is present.

## Run

Open a normal PowerShell or Command Prompt; administrator privileges are not
required.

```powershell
cd C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws
.\tools\run_thermal_focus_test.cmd focus_trial_01
```

Dataset names may contain letters, numbers, underscores, and hyphens. A browser
opens after the first preview is ready and refreshes `latest_preview.png` every
500 ms. The display uses a per-frame 1st–99th percentile auto scale and prints a
focus metric. Compare the metric only when the target position and framing are
similar.

Adjust the lens in small increments and pause for at least two or three frames
after every adjustment. Press Enter in the terminal when finished; do not close
the terminal window. Wait for the final Arduino STOPPED message and A6701 health
check.

## Output

The terminal prints the exact output directory under `hardware_test_data/`.
Each session contains:

- `frame_*_640x513_mono16.raw`: authoritative transport payloads;
- `frames.ndjson`: frame ID, camera timestamp, host monotonic timestamp, bytes,
  hash, and filename;
- `gray16_png/`: lossless 640x512 radiometric-count images;
- `focus_png/`: annotated false-color focus previews;
- `latest_preview.png` and `viewer.html`: live view;
- `focus_metrics.ndjson`: display ranges and focus scores;
- `capture_complete.json`: final frame count and stop reason;
- camera/watcher stdout and stderr logs.

At 2 Hz, raw payload alone is about 79 MB/minute. PNG products add variable
overhead, so the focus test should be stopped after the lens position is chosen.
