# Data export

Run the commands below from the `PPBNG_Imaging/` workspace in Windows PowerShell. From the repository root, enter it with `cd .\PPBNG_Imaging`. See the [operation manual](../../OPERATION_MANUAL.md) for acquisition and shutdown.

Keep original sessions read-only. Export only after ordered shutdown, to a new directory outside the session. No camera or ROS connection is needed for Python image conversion; NumPy, Pillow and PyYAML are listed in `tools/requirements_export.txt`.

## RGB and thermal

```powershell
.\tools\setup_export_python.cmd
.\tools\export_snapshot_images.cmd "C:\data\session" --output-directory "C:\exports\session_images" --rgb-color-preview-every 1 --thermal-false-color-preview-every 1
```

RGB and thermal are stored in separate `.ppbseg` files. This exports every saved record unless selection flags are added. Filenames use acquisition sample IDs; SDK frame IDs may reset after recovery. Metadata links outputs to source records, association and frame context. Equal IDs across cameras do not prove simultaneity.

- RGB: lossless BayerRG8 PGM plus demosaiced color PNG.
- Thermal: 16-bit count PNG, complete transport PNG, and false-color display PNG. Current A6701 transport has 513 rows: row 0 is auxiliary; rows 1–512 are image counts. Keep the complete transport image as well as the crop.
- Thermal counts/false color are **not Celsius**. Temperature conversion requires valid matching calibration and object/environment parameters.
- Missing metadata, NUC tags, time-quality flags and source warnings remain relevant; successful conversion does not prove a gap-free acquisition.

## Complete FX10e view

```powershell
.\tools\export_hsi_rgb_full.cmd "C:\data\session" --calpack "C:\calibration\matching_fx10e.scp" --tile-lines 4096
```

Every saved scan line is rendered into ordered PNG tiles; dark and sample tiles are separate. Check `full_export_metadata.json` for matching source/exported line counts and `line_step=1`. Only three visible bands are displayed, not the complete spectrum. Output is stretched 8-bit display RGB, not reflectance or georectified imagery. The quicklook/preview scripts may subsample and are not full exports.

HSI raw files are little-endian uint16 ENVI BIL. Read each header: array order is `(lines, bands, samples)`. A part is a file rollover; a segment is a capture interval. Keep indices/timestamps and do not assume boundaries imply continuity. SWIR is not supported by this FX10e command; SWIR visualization requires a separate explicitly labeled false-color band selection and the same full-line coverage.

Retain the session manifest, configuration snapshot, calibration metadata and all sidecars with raw data. Another machine needs a relocated matching calibration path, not the acquisition PC's original absolute path.
