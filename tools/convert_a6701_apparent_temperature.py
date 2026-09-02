#!/usr/bin/env python3
"""Create PS0 factory-calibrated apparent-temperature products for one A6701 dataset.

This is an apparent blackbody temperature conversion, not a complete object-
temperature correction.  It refuses to use hard-coded or stale coefficients and
requires the calibration_metadata.json captured in the same ROS 2 session.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from convert_a6701_raw import IMAGE_HEIGHT, load_frame, thermal_colormap


def load_session_calibration(source: Path) -> dict:
    path = source / "calibration_metadata.json"
    if not path.is_file():
        raise RuntimeError(
            "calibration_metadata.json is required; refusing stale/hard-coded coefficients"
        )
    values = json.loads(path.read_text(encoding="utf-8")).get("values", {})
    required = [
        "CalibrationQueryMinCounts", "CalibrationQueryMaxCounts",
        "CalibrationQueryCoeff0", "CalibrationQueryCoeff1", "CalibrationQueryCoeff2",
        "CalibrationQueryR", "CalibrationQueryB", "CalibrationQueryF",
    ]
    missing = [key for key in required if values.get(key) in (None, "", "UNAVAILABLE")]
    if missing:
        raise RuntimeError(f"session calibration snapshot is incomplete: {missing}")
    return {
        "name": values.get("CalibrationQueryName") or values.get("CalibrationQueryLens"),
        "preset": int(values.get("ActivePreset", 0)),
        "count_min": int(float(values["CalibrationQueryMinCounts"])),
        "count_max": int(float(values["CalibrationQueryMaxCounts"])),
        "count_to_radiance_coefficients": [
            float(values["CalibrationQueryCoeff0"]),
            float(values["CalibrationQueryCoeff1"]),
            float(values["CalibrationQueryCoeff2"]),
        ],
        "planck_r": float(values["CalibrationQueryR"]),
        "planck_b_kelvin": float(values["CalibrationQueryB"]),
        "planck_f": float(values["CalibrationQueryF"]),
        "source": str(path),
    }


def counts_to_temperature_c(
    counts: np.ndarray, calibration: dict
) -> tuple[np.ndarray, np.ndarray]:
    c = counts.astype(np.float64)
    a0, a1, a2 = calibration["count_to_radiance_coefficients"]
    radiance = a0 + a1 * c + a2 * c * c
    valid = (
        (counts >= calibration["count_min"])
        & (counts <= calibration["count_max"])
        & (radiance > 0.0)
    )
    temperature = np.full(counts.shape, np.nan, dtype=np.float32)
    r = calibration["planck_r"]
    b = calibration["planck_b_kelvin"]
    f = calibration["planck_f"]
    temperature[valid] = (b / np.log(r / radiance[valid] + f) - 273.15).astype(np.float32)
    return temperature, valid


def annotate(
    rgb: np.ndarray,
    index: int,
    low: float,
    high: float,
    valid_fraction: float,
    calibration_name: str,
) -> Image.Image:
    canvas = Image.new("RGB", (850, 615), "white")
    canvas.paste(Image.fromarray(rgb), (20, 55))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text((20, 18), f"A6701 frame {index:04d} - factory-calibrated apparent temperature", fill="black", font=font)
    gradient_values = np.linspace(1.0, 0.0, IMAGE_HEIGHT, dtype=np.float32)[:, None]
    gradient = np.repeat(thermal_colormap(gradient_values), 28, axis=1)
    canvas.paste(Image.fromarray(gradient), (680, 55))
    draw.rectangle((679, 54, 708, 567), outline="black")
    draw.text((716, 51), f"{high:.2f} C", fill="black", font=font)
    draw.text((716, 304), "apparent", fill="black", font=font)
    draw.text((716, 318), "temperature", fill="black", font=font)
    draw.text((716, 555), f"{low:.2f} C", fill="black", font=font)
    draw.text(
        (20, 580),
        f"Session calibration {calibration_name}; no object/environment correction; valid pixels {valid_fraction:.3%}.",
        fill="black",
        font=font,
    )
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_directory", type=Path)
    parser.add_argument("--output-directory", type=Path)
    args = parser.parse_args()

    source = args.input_directory.resolve()
    output = (args.output_directory or (source / "apparent_temperature")).resolve()
    raw_paths = sorted(source.glob("*.raw"))
    if not raw_paths:
        raise RuntimeError(f"no .raw frames found in {source}")
    calibration = load_session_calibration(source)

    temperatures = []
    valid_masks = []
    for path in raw_paths:
        temperature, valid = counts_to_temperature_c(
            load_frame(path)[1 : IMAGE_HEIGHT + 1], calibration
        )
        temperatures.append(temperature)
        valid_masks.append(valid)
    stack = np.stack(temperatures)
    masks = np.stack(valid_masks)
    valid_values = stack[masks]
    display_min, display_max = np.percentile(valid_values, [1.0, 99.0]).astype(float)

    preview_dir = output / "false_color_png"
    annotated_dir = output / "annotated_png"
    numeric_dir = output / "float32_celsius_tiff"
    for directory in (preview_dir, annotated_dir, numeric_dir):
        directory.mkdir(parents=True, exist_ok=True)

    frame_metadata = []
    preview_images = []
    for index, (path, temperature, valid) in enumerate(
        zip(raw_paths, temperatures, valid_masks), start=1
    ):
        normalized = np.clip((temperature - display_min) / (display_max - display_min), 0.0, 1.0)
        normalized = np.nan_to_num(normalized, nan=0.0)
        rgb = thermal_colormap(normalized)
        rgb[~valid] = np.array([0, 255, 255], dtype=np.uint8)
        preview = Image.fromarray(rgb)
        stem = path.stem
        preview.save(preview_dir / f"{stem}_apparent_temperature.png")
        annotate(
            rgb,
            index,
            display_min,
            display_max,
            float(valid.mean()),
            calibration["name"],
        ).save(
            annotated_dir / f"{stem}_apparent_temperature_annotated.png"
        )
        Image.fromarray(temperature.astype(np.float32)).save(
            numeric_dir / f"{stem}_apparent_temperature_celsius.tiff"
        )
        preview_images.append(preview)
        frame_metadata.append(
            {
                "source": path.name,
                "valid_fraction": float(valid.mean()),
                "valid_min_c": float(np.nanmin(temperature)),
                "valid_max_c": float(np.nanmax(temperature)),
                "valid_mean_c": float(np.nanmean(temperature)),
                "valid_median_c": float(np.nanmedian(temperature)),
            }
        )

    montage = Image.new("RGB", (1800, 650), "white")
    draw = ImageDraw.Draw(montage)
    draw.text((20, 15), "FLIR A6701 - factory-calibrated apparent temperature (not corrected object temperature)", fill="black")
    for index, preview in enumerate(preview_images, start=1):
        thumb = preview.resize((320, 256))
        column = (index - 1) % 5
        row = (index - 1) // 5
        x = 10 + column * 330
        y = 55 + row * 290
        montage.paste(thumb, (x, y))
        draw.text((x, y + 260), f"Frame {index:02d}", fill="black")
    gradient_values = np.linspace(1.0, 0.0, 512, dtype=np.float32)[:, None]
    gradient = np.repeat(thermal_colormap(gradient_values), 24, axis=1)
    montage.paste(Image.fromarray(gradient), (1680, 55))
    draw.text((1645, 35), f"{display_max:.2f} C", fill="black")
    draw.text((1645, 570), f"{display_min:.2f} C", fill="black")
    draw.text((1570, 615), "fixed batch apparent-temperature scale", fill="black")
    montage.save(output / "montage_10_frames_apparent_temperature.png")

    metadata = {
        "source_directory": str(source),
        "product": "factory-calibrated apparent blackbody temperature",
        "complete_object_temperature_correction_applied": False,
        "warning": (
            "Do not interpret as true object surface temperature. This conversion uses "
            "only factory count-to-radiance/Planck calibration; it does not yet apply the "
            "captured emissivity, reflected-temperature, atmosphere, distance, humidity, "
            "or external-optics corrections."
        ),
        "calibration": calibration,
        "display_min_c": display_min,
        "display_max_c": display_max,
        "invalid_pixel_color": "cyan",
        "frames": frame_metadata,
    }
    (output / "temperature_conversion_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    (output / "README.md").write_text(
        "# A6701 apparent-temperature products\n\n"
        "These images use the calibration snapshot captured from the exact camera at "
        "the beginning of this acquisition to convert counts to radiance and then apparent "
        "blackbody temperature. They are **not fully corrected object temperatures**.\n\n"
        "- `false_color_png/`: fixed-scale temperature previews.\n"
        "- `annotated_png/`: previews with Celsius colorbar and warning.\n"
        "- `float32_celsius_tiff/`: numerical float32 apparent-temperature arrays; "
        "out-of-calibration pixels are NaN.\n"
        "- `temperature_conversion_metadata.json`: formula inputs and per-frame statistics.\n\n"
        "Cyan pixels are outside the saved calibration count range. Preserve the original "
        "raw frames as the authoritative record.\n",
        encoding="utf-8",
    )
    print(json.dumps({"event": "temperature_conversion_complete", **metadata}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
