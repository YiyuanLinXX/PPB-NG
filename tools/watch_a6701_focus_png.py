#!/usr/bin/env python3
"""Watch a focus-capture directory and create PNG previews as frames arrive."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from convert_a6701_raw import IMAGE_HEIGHT, load_frame, thermal_colormap


def focus_metric(image: np.ndarray) -> float:
    data = image.astype(np.float32)
    low, high = np.percentile(data, [1.0, 99.0])
    if high <= low:
        return 0.0
    data = np.clip((data - low) / (high - low), 0.0, 1.0)
    gx = np.diff(data, axis=1)
    gy = np.diff(data, axis=0)
    return float(np.mean(gx * gx) + np.mean(gy * gy))


def make_preview(image: np.ndarray, frame_number: int, metric: float) -> tuple[Image.Image, float, float]:
    low, high = np.percentile(image, [1.0, 99.0]).astype(float)
    normalized = np.clip((image.astype(np.float32) - low) / max(high - low, 1.0), 0.0, 1.0)
    rgb = thermal_colormap(normalized)
    canvas = Image.new("RGB", (850, 615), "white")
    canvas.paste(Image.fromarray(rgb), (20, 55))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text((20, 18), f"A6701 live focus - frame {frame_number:08d}", fill="black", font=font)
    gradient_values = np.linspace(1.0, 0.0, IMAGE_HEIGHT, dtype=np.float32)[:, None]
    gradient = np.repeat(thermal_colormap(gradient_values), 28, axis=1)
    canvas.paste(Image.fromarray(gradient), (680, 55))
    draw.rectangle((679, 54, 708, 567), outline="black")
    draw.text((716, 51), f"{high:.0f}", fill="black", font=font)
    draw.text((716, 304), "counts", fill="black", font=font)
    draw.text((716, 555), f"{low:.0f}", fill="black", font=font)
    draw.text(
        (20, 580),
        f"AUTO SCALE 1st-99th percentile | focus metric {metric:.8f} | compare only similar target positions",
        fill="black",
        font=font,
    )
    return canvas, low, high


def atomic_save(image: Image.Image, destination: Path) -> None:
    temporary = destination.with_name(destination.stem + ".partial.png")
    image.save(temporary, format="PNG")
    os.replace(temporary, destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_directory", type=Path)
    args = parser.parse_args()
    capture = args.capture_directory.resolve()
    while not capture.exists():
        time.sleep(0.1)

    png_dir = capture / "focus_png"
    gray_dir = capture / "gray16_png"
    png_dir.mkdir(exist_ok=True)
    gray_dir.mkdir(exist_ok=True)
    (capture / "viewer.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>A6701 live focus</title>"
        "<style>body{margin:0;background:#222;color:white;font:16px sans-serif;text-align:center}"
        "img{max-width:96vw;max-height:92vh;margin-top:1vh}</style>"
        "<h3>A6701 live focus — press Enter in the terminal to stop safely</h3>"
        "<img id='p' src='latest_preview.png'>"
        "<script>setInterval(()=>{document.getElementById('p').src='latest_preview.png?t='+Date.now()},500)</script>",
        encoding="utf-8",
    )
    metrics_path = capture / "focus_metrics.ndjson"
    processed: set[str] = set()
    frame_number = 0
    while True:
        pending = [path for path in sorted(capture.glob("*.raw")) if path.name not in processed]
        for path in pending:
            frame_number += 1
            transport = load_frame(path)
            image = transport[1 : IMAGE_HEIGHT + 1]
            metric = focus_metric(image)
            preview, low, high = make_preview(image, frame_number, metric)
            Image.fromarray(image).save(gray_dir / f"{path.stem}_counts16.png")
            preview_path = png_dir / f"{path.stem}_focus.png"
            preview.save(preview_path)
            atomic_save(preview, capture / "latest_preview.png")
            record = {
                "frame": frame_number,
                "raw_file": path.name,
                "preview_file": str(preview_path.relative_to(capture)),
                "display_min_count": low,
                "display_max_count": high,
                "focus_metric": metric,
            }
            with metrics_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record) + "\n")
            processed.add(path.name)
            print(json.dumps({"event": "preview_ready", **record}), flush=True)

        if (capture / "capture_complete.json").exists() and not pending:
            break
        time.sleep(0.1)
    print(json.dumps({"event": "preview_watcher_complete", "frames": frame_number}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
