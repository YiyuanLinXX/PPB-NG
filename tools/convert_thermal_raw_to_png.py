#!/usr/bin/env python3
"""Stream FLIR A6701 PPB-NG raw datasets into reviewable PNG products.

The 640x513 transport payload contains one FLIR GigE header row followed by a
640x512 Mono16 radiometric-count image.  This program never loads a complete
long-duration dataset into memory.  It uses a streaming histogram pass to find
one stable display range, followed by a streaming PNG conversion pass.

If calibration_metadata.json from the same acquisition is present, the program
also creates factory-calibrated apparent-blackbody-temperature PNGs.  These are
not fully corrected object-surface temperatures.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH = 640
TRANSPORT_HEIGHT = 513
IMAGE_HEIGHT = 512
EXPECTED_BYTES = WIDTH * TRANSPORT_HEIGHT * 2


def load_image(path: Path) -> np.ndarray:
    payload = path.read_bytes()
    if len(payload) != EXPECTED_BYTES:
        raise ValueError(
            f"{path.name}: expected {EXPECTED_BYTES} bytes, got {len(payload)}"
        )
    transport = np.frombuffer(payload, dtype="<u2").reshape(TRANSPORT_HEIGHT, WIDTH)
    return transport[1 : IMAGE_HEIGHT + 1]


def thermal_colormap(normalized: np.ndarray) -> np.ndarray:
    positions = np.array([0.0, 0.18, 0.38, 0.58, 0.78, 1.0], dtype=np.float32)
    colors = np.array(
        [
            [0, 0, 0],
            [35, 10, 92],
            [135, 20, 115],
            [225, 55, 45],
            [252, 170, 20],
            [255, 255, 230],
        ],
        dtype=np.float32,
    )
    channels = [
        np.interp(normalized, positions, colors[:, channel]) for channel in range(3)
    ]
    return np.stack(channels, axis=-1).round().clip(0, 255).astype(np.uint8)


def histogram_percentile(histogram: np.ndarray, percentile: float) -> float:
    total = int(histogram.sum())
    if total <= 0:
        raise RuntimeError("empty histogram")
    target = percentile / 100.0 * (total - 1)
    index = int(np.searchsorted(np.cumsum(histogram, dtype=np.uint64), target, side="right"))
    return float(min(index, len(histogram) - 1))


def load_calibration(source: Path) -> dict | None:
    path = source / "calibration_metadata.json"
    if not path.is_file():
        return None
    values = json.loads(path.read_text(encoding="utf-8")).get("values", {})
    required = [
        "CalibrationQueryMinCounts",
        "CalibrationQueryMaxCounts",
        "CalibrationQueryCoeff0",
        "CalibrationQueryCoeff1",
        "CalibrationQueryCoeff2",
        "CalibrationQueryR",
        "CalibrationQueryB",
        "CalibrationQueryF",
    ]
    missing = [key for key in required if values.get(key) in (None, "", "UNAVAILABLE")]
    if missing:
        raise RuntimeError(f"incomplete session calibration metadata: {missing}")
    order = int(float(values.get("CalibrationQueryOrder", 2)))
    if order < 0 or order > 6:
        raise RuntimeError(f"unsupported calibration polynomial order: {order}")
    coefficients = []
    for index in range(order + 1):
        value = values.get(f"CalibrationQueryCoeff{index}")
        if value in (None, "", "UNAVAILABLE"):
            if index <= 2:
                raise RuntimeError(f"missing CalibrationQueryCoeff{index}")
            value = "0"
        coefficients.append(float(value))
    return {
        "name": values.get("CalibrationQueryName")
        or values.get("CalibrationQueryTag")
        or values.get("CalibrationQueryLens")
        or "unnamed session calibration",
        "count_min": int(float(values["CalibrationQueryMinCounts"])),
        "count_max": int(float(values["CalibrationQueryMaxCounts"])),
        "coefficients": coefficients,
        "planck_r": float(values["CalibrationQueryR"]),
        "planck_b_kelvin": float(values["CalibrationQueryB"]),
        "planck_f": float(values["CalibrationQueryF"]),
        "object_parameters_recorded": {
            key: values.get(key)
            for key in (
                "ObjectEmissivity",
                "ReflectedTemperature",
                "AtmosphericTemperature",
                "ObjectDistance",
                "RelativeHumidity",
                "EstimatedTransmission",
                "ExtOpticsTemperature",
                "ExtOpticsTransmission",
            )
        },
        "source": str(path),
    }


def counts_to_apparent_temperature(
    counts: np.ndarray, calibration: dict
) -> tuple[np.ndarray, np.ndarray]:
    values = counts.astype(np.float64)
    radiance = np.zeros(values.shape, dtype=np.float64)
    power = np.ones(values.shape, dtype=np.float64)
    for coefficient in calibration["coefficients"]:
        radiance += coefficient * power
        power *= values
    valid = (
        (counts >= calibration["count_min"])
        & (counts <= calibration["count_max"])
        & (radiance > 0.0)
    )
    temperature = np.full(counts.shape, np.nan, dtype=np.float32)
    temperature[valid] = (
        calibration["planck_b_kelvin"]
        / np.log(
            calibration["planck_r"] / radiance[valid]
            + calibration["planck_f"]
        )
        - 273.15
    ).astype(np.float32)
    return temperature, valid


def temperature_display_range(
    count_histogram: np.ndarray,
    calibration: dict,
    low_percentile: float,
    high_percentile: float,
) -> tuple[float, float]:
    counts = np.arange(65536, dtype=np.uint16)
    temperatures, valid = counts_to_apparent_temperature(counts, calibration)
    weights = count_histogram[valid]
    values = temperatures[valid].astype(np.float64)
    positive = weights > 0
    weights = weights[positive]
    values = values[positive]
    if not len(values):
        raise RuntimeError("no pixels fall within the captured calibration range")
    order = np.argsort(values)
    values = values[order]
    cumulative = np.cumsum(weights[order], dtype=np.uint64)
    total = int(cumulative[-1])

    def weighted(percentile: float) -> float:
        target = percentile / 100.0 * (total - 1)
        index = int(np.searchsorted(cumulative, target, side="right"))
        return float(values[min(index, len(values) - 1)])

    return weighted(low_percentile), weighted(high_percentile)


def atomic_save_png(rgb: np.ndarray, destination: Path) -> None:
    temporary = destination.with_name(destination.stem + ".partial.png")
    Image.fromarray(rgb, mode="RGB").save(temporary, format="PNG", optimize=False)
    os.replace(temporary, destination)


def make_legend(path: Path, low: float, high: float, units: str, title: str) -> None:
    canvas = Image.new("RGB", (220, 600), "white")
    gradient = np.linspace(1.0, 0.0, 512, dtype=np.float32)[:, None]
    gradient = np.repeat(thermal_colormap(gradient), 36, axis=1)
    canvas.paste(Image.fromarray(gradient), (24, 52))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text((12, 16), title, fill="black", font=font)
    draw.rectangle((23, 51, 60, 564), outline="black")
    draw.text((72, 48), f"{high:.3f} {units}", fill="black", font=font)
    draw.text((72, 550), f"{low:.3f} {units}", fill="black", font=font)
    canvas.save(path)


def make_montage(directory: Path, destination: Path, title: str) -> None:
    paths = sorted(directory.glob("*.png"))
    if not paths:
        return
    count = min(12, len(paths))
    indices = np.linspace(0, len(paths) - 1, count, dtype=int)
    canvas = Image.new("RGB", (1320, 1120), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text((20, 15), title, fill="black", font=font)
    for position, source_index in enumerate(indices):
        path = paths[int(source_index)]
        with Image.open(path) as source:
            thumbnail = source.convert("RGB").resize((320, 256))
        column = position % 4
        row = position // 4
        x = 10 + column * 328
        y = 50 + row * 350
        canvas.paste(thumbnail, (x, y))
        draw.text((x, y + 262), path.stem[:48], fill="black", font=font)
    canvas.save(destination)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Stream A6701 raw frames to fixed-scale false-color PNGs"
    )
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument(
        "--every",
        type=int,
        default=1,
        help="convert every Nth frame (1 converts all frames)",
    )
    parser.add_argument("--low-percentile", type=float, default=1.0)
    parser.add_argument("--high-percentile", type=float, default=99.0)
    parser.add_argument(
        "--no-temperature",
        action="store_true",
        help="skip apparent-temperature products even when calibration metadata exists",
    )
    parser.add_argument(
        "--save-temperature-tiff",
        action="store_true",
        help="also save float32 Celsius TIFF arrays (uses substantial disk space)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace an existing output directory instead of refusing",
    )
    args = parser.parse_args()

    source = args.dataset.resolve()
    if not source.is_dir():
        raise RuntimeError(f"dataset directory not found: {source}")
    if args.every <= 0:
        raise ValueError("--every must be positive")
    if not 0.0 <= args.low_percentile < args.high_percentile <= 100.0:
        raise ValueError("percentiles must satisfy 0 <= low < high <= 100")
    all_paths = sorted(source.glob("frame_*_640x513_mono16.raw"))
    if not all_paths:
        all_paths = sorted(source.glob("*.raw"))
    selected = all_paths[:: args.every]
    if not selected:
        raise RuntimeError("no raw frames selected")

    suffix = "all" if args.every == 1 else f"every_{args.every}"
    output = (
        args.output_directory or source / f"converted_png_{suffix}"
    ).resolve()
    if output.exists() and not args.overwrite:
        raise RuntimeError(
            f"output already exists: {output}; choose another directory or pass --overwrite"
        )
    output.mkdir(parents=True, exist_ok=True)
    counts_dir = output / "false_color_counts_png"
    counts_dir.mkdir(exist_ok=True)

    calibration = None if args.no_temperature else load_calibration(source)
    temperature_dir = output / "apparent_temperature_png"
    numeric_dir = output / "apparent_temperature_float32_celsius_tiff"
    if calibration:
        temperature_dir.mkdir(exist_ok=True)
        if args.save_temperature_tiff:
            numeric_dir.mkdir(exist_ok=True)

    started = time.monotonic()
    histogram = np.zeros(65536, dtype=np.uint64)
    selected_set = set(selected)
    quality_records = []
    for index, path in enumerate(all_paths, start=1):
        image = load_image(path)
        if path in selected_set:
            histogram += np.bincount(image.ravel(), minlength=65536).astype(np.uint64)
        zero_fraction = float(np.count_nonzero(image == 0) / image.size)
        image_min = int(image.min())
        image_max = int(image.max())
        flags = []
        if zero_fraction > 0.001:
            flags.append("EXCESS_ZERO_PIXELS")
        if image_max - image_min <= 1:
            flags.append("NEAR_UNIFORM_FRAME")
        outside_calibration_fraction = None
        if calibration:
            within = (
                (image >= calibration["count_min"])
                & (image <= calibration["count_max"])
            )
            outside_calibration_fraction = float(1.0 - within.mean())
            if outside_calibration_fraction > 0.01:
                flags.append("OUTSIDE_CALIBRATION_RANGE")
        if flags:
            quality_records.append(
                {
                    "sample": int(path.name.split("_")[1]),
                    "source": path.name,
                    "flags": flags,
                    "zero_fraction": zero_fraction,
                    "outside_calibration_fraction": outside_calibration_fraction,
                    "min_count": image_min,
                    "max_count": image_max,
                    "mean_count": float(image.mean()),
                }
            )
        if index == 1 or index % 1000 == 0 or index == len(all_paths):
            print(f"SCAN {index}/{len(all_paths)}", flush=True)

    with (output / "quality_flags.ndjson").open("w", encoding="utf-8") as stream:
        for record in quality_records:
            stream.write(json.dumps(record) + "\n")

    quality_ranges = []
    for record in quality_records:
        sample = record["sample"]
        if not quality_ranges or sample > quality_ranges[-1][1] + 2:
            quality_ranges.append([sample, sample])
        else:
            quality_ranges[-1][1] = sample

    count_low = histogram_percentile(histogram, args.low_percentile)
    count_high = histogram_percentile(histogram, args.high_percentile)
    if count_high <= count_low:
        raise RuntimeError("invalid global count display range")
    temp_range = None
    if calibration:
        temp_range = temperature_display_range(
            histogram, calibration, args.low_percentile, args.high_percentile
        )
        if temp_range[1] <= temp_range[0]:
            raise RuntimeError("invalid global temperature display range")

    manifest_path = output / "conversion_frames.ndjson"
    converted = 0
    invalid_temperature_pixels = 0
    total_temperature_pixels = 0
    with manifest_path.open("w", encoding="utf-8") as manifest:
        for index, path in enumerate(selected, start=1):
            image = load_image(path)
            normalized = np.clip(
                (image.astype(np.float32) - count_low) / (count_high - count_low),
                0.0,
                1.0,
            )
            count_name = f"{path.stem}_counts_false_color.png"
            atomic_save_png(thermal_colormap(normalized), counts_dir / count_name)
            record = {
                "source": path.name,
                "counts_png": str(Path(counts_dir.name) / count_name),
                "min_count": int(image.min()),
                "max_count": int(image.max()),
            }
            if calibration and temp_range:
                temperature, valid = counts_to_apparent_temperature(image, calibration)
                temp_normalized = np.clip(
                    (temperature - temp_range[0]) / (temp_range[1] - temp_range[0]),
                    0.0,
                    1.0,
                )
                temp_rgb = thermal_colormap(np.nan_to_num(temp_normalized, nan=0.0))
                temp_rgb[~valid] = np.array([0, 255, 255], dtype=np.uint8)
                temp_name = f"{path.stem}_apparent_temperature.png"
                atomic_save_png(temp_rgb, temperature_dir / temp_name)
                record["apparent_temperature_png"] = str(
                    Path(temperature_dir.name) / temp_name
                )
                record["valid_temperature_fraction"] = float(valid.mean())
                if valid.any():
                    record["valid_temperature_min_c"] = float(np.nanmin(temperature))
                    record["valid_temperature_max_c"] = float(np.nanmax(temperature))
                invalid_temperature_pixels += int((~valid).sum())
                total_temperature_pixels += int(valid.size)
                if args.save_temperature_tiff:
                    tiff_name = f"{path.stem}_apparent_temperature_celsius.tiff"
                    Image.fromarray(temperature.astype(np.float32)).save(
                        numeric_dir / tiff_name
                    )
                    record["apparent_temperature_tiff"] = str(
                        Path(numeric_dir.name) / tiff_name
                    )
            manifest.write(json.dumps(record) + "\n")
            converted += 1
            if index == 1 or index % 100 == 0 or index == len(selected):
                print(f"PNG {index}/{len(selected)}", flush=True)

    make_legend(
        output / "legend_radiometric_counts.png",
        count_low,
        count_high,
        "counts",
        "A6701 radiometric counts",
    )
    if temp_range:
        make_legend(
            output / "legend_apparent_temperature.png",
            temp_range[0],
            temp_range[1],
            "C",
            "Apparent temperature",
        )
    make_montage(
        counts_dir,
        output / "montage_radiometric_counts.png",
        "A6701 radiometric counts - fixed scale across selected frames",
    )
    if calibration:
        make_montage(
            temperature_dir,
            output / "montage_apparent_temperature.png",
            "A6701 apparent temperature - not fully corrected object temperature",
        )

    metadata = {
        "source_directory": str(source),
        "output_directory": str(output),
        "source_raw_frame_count": len(all_paths),
        "converted_frame_count": converted,
        "every_nth_frame": args.every,
        "transport_geometry": [WIDTH, TRANSPORT_HEIGHT],
        "image_geometry": [WIDTH, IMAGE_HEIGHT],
        "excluded_transport_row": 1,
        "excluded_transport_row_type": "FLIR GigE image header",
        "fixed_scale_percentiles": [args.low_percentile, args.high_percentile],
        "fixed_count_display_range": [count_low, count_high],
        "temperature_product": (
            "factory-calibrated apparent blackbody temperature"
            if calibration
            else "not generated"
        ),
        "complete_object_temperature_correction_applied": False,
        "temperature_warning": (
            "Apparent temperature only. Captured object/environment parameters are "
            "recorded but emissivity, reflection, atmosphere, distance, humidity and "
            "external-optics corrections are not yet applied."
            if calibration
            else "No usable same-session calibration metadata was found."
        ),
        "fixed_apparent_temperature_display_range_c": temp_range,
        "invalid_temperature_pixel_fraction": (
            invalid_temperature_pixels / total_temperature_pixels
            if total_temperature_pixels
            else None
        ),
        "quality_flagged_frame_count": len(quality_records),
        "quality_flagged_sample_ranges_with_gap_tolerance_2": quality_ranges,
        "quality_flag_file": "quality_flags.ndjson",
        "quality_note": (
            "Flags identify numerical payloads unsuitable as ordinary scene frames; "
            "on this camera, periodic blocks can be produced by automatic NUC/internal-flag updates."
        ),
        "calibration": calibration,
        "elapsed_seconds": time.monotonic() - started,
    }
    (output / "conversion_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    (output / "README.txt").write_text(
        "false_color_counts_png: fixed-scale radiometric-count previews\n"
        "apparent_temperature_png: same-session factory-calibrated apparent temperature\n"
        "Cyan temperature pixels: outside the captured calibration range\n"
        "The first 640x1 transport row is a FLIR GigE header, not image data.\n"
        "Apparent temperature is not fully corrected object-surface temperature.\n"
        "Original raw files remain authoritative.\n"
        "quality_flags.ndjson lists zero/uniform/out-of-calibration frames.\n",
        encoding="utf-8",
    )
    print(json.dumps({"event": "conversion_complete", **metadata}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
