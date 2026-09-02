#!/usr/bin/env python3
"""Convert PPB-NG A6701 Mono16 raw frames into reviewable PNG products.

The A6701 transport frame is 640 x 513 uint16 pixels.  The first row is the
FLIR GigE header and the following 512 rows are the radiometric image.  The
header is retained in the .raw source but excluded from display.
This utility does not claim to convert counts to temperature without a camera
calibration model and acquisition/environment parameters.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH = 640
TRANSPORT_HEIGHT = 513
IMAGE_HEIGHT = 512
EXPECTED_BYTES = WIDTH * TRANSPORT_HEIGHT * 2


def load_frame(path: Path) -> np.ndarray:
    payload = path.read_bytes()
    if len(payload) != EXPECTED_BYTES:
        raise ValueError(f"{path.name}: expected {EXPECTED_BYTES} bytes, got {len(payload)}")
    return np.frombuffer(payload, dtype="<u2").reshape(TRANSPORT_HEIGHT, WIDTH)


def thermal_colormap(values: np.ndarray) -> np.ndarray:
    """Apply a deterministic thermal-style palette to normalized values."""
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
    channels = [np.interp(values, positions, colors[:, channel]) for channel in range(3)]
    return np.stack(channels, axis=-1).round().clip(0, 255).astype(np.uint8)


def annotated_image(rgb: np.ndarray, frame_index: int, low: float, high: float) -> Image.Image:
    canvas = Image.new("RGB", (820, 600), "white")
    canvas.paste(Image.fromarray(rgb, mode="RGB"), (20, 52))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text((20, 18), f"A6701 frame {frame_index:04d} - radiometric counts (not temperature)", fill="black", font=font)
    gradient_values = np.linspace(1.0, 0.0, IMAGE_HEIGHT, dtype=np.float32)[:, None]
    gradient = np.repeat(thermal_colormap(gradient_values), 28, axis=1)
    canvas.paste(Image.fromarray(gradient, mode="RGB"), (680, 52))
    draw.rectangle((679, 51, 708, 564), outline="black")
    draw.text((716, 48), f"{high:.0f}", fill="black", font=font)
    draw.text((716, 298), "counts", fill="black", font=font)
    draw.text((716, 552), f"{low:.0f}", fill="black", font=font)
    draw.text((20, 574), "Display uses one fixed 1st-99th percentile scale for the full batch.", fill="black", font=font)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_directory", type=Path)
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--low-percentile", type=float, default=1.0)
    parser.add_argument("--high-percentile", type=float, default=99.0)
    args = parser.parse_args()

    source = args.input_directory.resolve()
    output = (args.output_directory or (source / "png_preview")).resolve()
    raw_paths = sorted(source.glob("*.raw"))
    if not raw_paths:
        raise RuntimeError(f"no .raw frames found in {source}")
    if not 0.0 <= args.low_percentile < args.high_percentile <= 100.0:
        raise ValueError("percentiles must satisfy 0 <= low < high <= 100")

    frames = [load_frame(path) for path in raw_paths]
    image_stack = np.stack([frame[1 : IMAGE_HEIGHT + 1] for frame in frames])
    display_min, display_max = np.percentile(
        image_stack, [args.low_percentile, args.high_percentile]
    ).astype(float)
    if display_max <= display_min:
        raise RuntimeError("invalid display range")

    gray_dir = output / "gray16"
    false_dir = output / "false_color"
    annotated_dir = output / "annotated"
    for directory in (gray_dir, false_dir, annotated_dir):
        directory.mkdir(parents=True, exist_ok=True)

    normalized = np.clip(
        (image_stack.astype(np.float32) - display_min) / (display_max - display_min),
        0.0,
        1.0,
    )

    per_frame = []
    for index, (raw_path, transport, norm) in enumerate(
        zip(raw_paths, frames, normalized), start=1
    ):
        image = transport[1 : IMAGE_HEIGHT + 1]
        stem = raw_path.stem
        Image.fromarray(image).save(gray_dir / f"{stem}_counts16.png")
        rgb = thermal_colormap(norm)
        Image.fromarray(rgb, mode="RGB").save(false_dir / f"{stem}_thermal.png")
        annotated_image(rgb, index, display_min, display_max).save(
            annotated_dir / f"{stem}_annotated.png"
        )

        per_frame.append(
            {
                "source": raw_path.name,
                "min_count": int(image.min()),
                "max_count": int(image.max()),
                "mean_count": float(image.mean()),
                "median_count": float(np.median(image)),
                "metadata_row_min": int(transport[0].min()),
                "metadata_row_max": int(transport[0].max()),
            }
        )

    montage = Image.new("RGB", (1800, 650), "white")
    montage_draw = ImageDraw.Draw(montage)
    montage_draw.text((20, 15), "FLIR A6701 - radiometric counts (not temperature)", fill="black")
    for index, norm in enumerate(normalized, start=1):
        thumb = Image.fromarray(thermal_colormap(norm), mode="RGB").resize((320, 256))
        column = (index - 1) % 5
        row = (index - 1) // 5
        x = 10 + column * 330
        y = 55 + row * 290
        montage.paste(thumb, (x, y))
        montage_draw.text((x, y + 260), f"Frame {index:02d}", fill="black")
    gradient_values = np.linspace(1.0, 0.0, 512, dtype=np.float32)[:, None]
    gradient = np.repeat(thermal_colormap(gradient_values), 24, axis=1)
    montage.paste(Image.fromarray(gradient, mode="RGB"), (1680, 55))
    montage_draw.text((1660, 35), f"{display_max:.0f}", fill="black")
    montage_draw.text((1660, 570), f"{display_min:.0f}", fill="black")
    montage_draw.text((1610, 615), "fixed batch scale (counts)", fill="black")
    montage.save(output / "montage_10_frames_thermal.png")

    metadata = {
        "source_directory": str(source),
        "frame_count": len(raw_paths),
        "raw_encoding": "little-endian uint16",
        "transport_geometry": [WIDTH, TRANSPORT_HEIGHT],
        "display_geometry": [WIDTH, IMAGE_HEIGHT],
        "excluded_transport_row": 1,
        "excluded_transport_row_type": "FLIR GigE image header",
        "display_colormap": "thermal-style black-purple-red-orange-yellow-white",
        "display_scale_scope": "fixed across entire batch",
        "display_low_percentile": args.low_percentile,
        "display_high_percentile": args.high_percentile,
        "display_min_count": display_min,
        "display_max_count": display_max,
        "temperature_conversion_applied": False,
        "temperature_conversion_reason": (
            "This preview utility preserves/display counts only. Use the separate "
            "apparent-temperature converter when a session calibration_metadata.json "
            "snapshot is present. Full object temperature additionally requires "
            "validated environmental and material parameters."
        ),
        "frames": per_frame,
    }
    (output / "conversion_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    (output / "README.md").write_text(
        "# A6701 PNG previews\n\n"
        "- `gray16/`: lossless 16-bit PNG containing the original radiometric counts.\n"
        "- `false_color/`: 8-bit thermal-style previews using one fixed batch scale.\n"
        "- `annotated/`: previews with axes and a radiometric-count colorbar.\n"
        "- `montage_10_frames_thermal.png`: batch overview.\n\n"
        f"Display range: {display_min:.3f} to {display_max:.3f} counts "
        f"({args.low_percentile:g}th–{args.high_percentile:g}th percentile across the batch).\n\n"
        "These products are **not temperature maps**. Converting radiometric counts to "
        "temperature requires traceable camera calibration and acquisition/environment "
        "parameters that are not stored in this dataset. The source `.raw` files remain "
        "the authoritative data.\n",
        encoding="utf-8",
    )
    print(json.dumps({"event": "conversion_complete", **metadata}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
