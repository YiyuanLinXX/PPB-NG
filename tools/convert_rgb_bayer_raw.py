#!/usr/bin/env python3
"""Convert PPBNG 4096x3000 BayerRG8 RAW frames to exact PGM and viewable RGB PNG."""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH = 4096
HEIGHT = 3000
EXPECTED_BYTES = WIDTH * HEIGHT
NAME_PATTERN = re.compile(r"frame_(\d+)_4096x3000_bayerrg8\.raw$", re.IGNORECASE)


def demosaic_bayer_rg8(bayer: np.ndarray) -> np.ndarray:
    """Bilinear demosaic for an RG/GB Bayer mosaic whose (0, 0) pixel is red."""
    p = np.pad(bayer.astype(np.uint16), 1, mode="edge")
    c = p[1:-1, 1:-1]
    u, d = p[:-2, 1:-1], p[2:, 1:-1]
    l, r = p[1:-1, :-2], p[1:-1, 2:]
    ul, ur = p[:-2, :-2], p[:-2, 2:]
    dl, dr = p[2:, :-2], p[2:, 2:]

    rgb = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    red, green, blue = rgb[..., 0], rgb[..., 1], rgb[..., 2]

    red[0::2, 0::2] = c[0::2, 0::2]
    red[0::2, 1::2] = ((l[0::2, 1::2] + r[0::2, 1::2] + 1) // 2).astype(np.uint8)
    red[1::2, 0::2] = ((u[1::2, 0::2] + d[1::2, 0::2] + 1) // 2).astype(np.uint8)
    red[1::2, 1::2] = ((ul[1::2, 1::2] + ur[1::2, 1::2] + dl[1::2, 1::2] + dr[1::2, 1::2] + 2) // 4).astype(np.uint8)

    blue[1::2, 1::2] = c[1::2, 1::2]
    blue[1::2, 0::2] = ((l[1::2, 0::2] + r[1::2, 0::2] + 1) // 2).astype(np.uint8)
    blue[0::2, 1::2] = ((u[0::2, 1::2] + d[0::2, 1::2] + 1) // 2).astype(np.uint8)
    blue[0::2, 0::2] = ((ul[0::2, 0::2] + ur[0::2, 0::2] + dl[0::2, 0::2] + dr[0::2, 0::2] + 2) // 4).astype(np.uint8)

    green[0::2, 1::2] = c[0::2, 1::2]
    green[1::2, 0::2] = c[1::2, 0::2]
    green[0::2, 0::2] = ((u[0::2, 0::2] + d[0::2, 0::2] + l[0::2, 0::2] + r[0::2, 0::2] + 2) // 4).astype(np.uint8)
    green[1::2, 1::2] = ((u[1::2, 1::2] + d[1::2, 1::2] + l[1::2, 1::2] + r[1::2, 1::2] + 2) // 4).astype(np.uint8)
    return rgb


def atomic_pgm(path: Path, bayer: np.ndarray) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    with temporary.open("wb") as output:
        output.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        output.write(bayer.tobytes(order="C"))
    temporary.replace(path)


def atomic_png(path: Path, rgb: np.ndarray) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    Image.fromarray(rgb, mode="RGB").save(temporary, format="PNG", optimize=False)
    temporary.replace(path)


def montage(paths: list[Path], output: Path) -> None:
    if not paths:
        return
    count = min(12, len(paths))
    indices = np.linspace(0, len(paths) - 1, count, dtype=int)
    selected = [paths[index] for index in indices]
    thumb_width, thumb_height = 384, 281
    canvas = Image.new("RGB", (thumb_width * 3, (thumb_height + 28) * 4), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    for slot, path in enumerate(selected):
        with Image.open(path) as image:
            thumbnail = image.convert("RGB")
            thumbnail.thumbnail((thumb_width, thumb_height))
            x = (slot % 3) * thumb_width
            y = (slot // 3) * (thumb_height + 28)
            canvas.paste(thumbnail, (x, y))
            draw.text((x + 4, y + thumb_height + 6), path.stem, fill="black", font=font)
    temporary = output.with_suffix(output.suffix + ".partial")
    canvas.save(temporary, format="PNG", optimize=False)
    temporary.replace(output)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path, help="RGB directory containing BayerRG8 RAW files")
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--every", type=int, default=1, help="convert every Nth frame")
    parser.add_argument("--no-pgm", action="store_true", help="skip exact Bayer PGM output")
    parser.add_argument("--no-color", action="store_true", help="skip demosaiced color PNG output")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.every < 1:
        parser.error("--every must be at least 1")
    source = args.dataset.resolve()
    if not source.is_dir():
        parser.error(f"dataset directory does not exist: {source}")
    frames = []
    for path in source.glob("*.raw"):
        match = NAME_PATTERN.match(path.name)
        if match:
            frames.append((int(match.group(1)), path))
    frames.sort()
    selected = frames[:: args.every]
    if not selected:
        parser.error("no matching 4096x3000 BayerRG8 RAW frames found")
    output = (args.output_directory or source / "converted_rgb").resolve()
    if output.exists() and not args.overwrite:
        parser.error(f"output already exists; choose another directory or use --overwrite: {output}")
    output.mkdir(parents=True, exist_ok=True)
    pgm_dir, png_dir = output / "bayer_pgm", output / "color_png"
    if not args.no_pgm:
        pgm_dir.mkdir(exist_ok=True)
    if not args.no_color:
        png_dir.mkdir(exist_ok=True)

    started = time.perf_counter()
    png_paths: list[Path] = []
    records = output / "conversion_frames.ndjson"
    with records.open("w", encoding="utf-8", newline="\n") as log:
        for position, (sample, path) in enumerate(selected, 1):
            if path.stat().st_size != EXPECTED_BYTES:
                raise RuntimeError(f"unexpected file size for {path.name}: {path.stat().st_size}")
            bayer = np.fromfile(path, dtype=np.uint8).reshape(HEIGHT, WIDTH)
            pgm_name = f"frame_{sample:08d}_bayer_rg8.pgm"
            png_name = f"frame_{sample:08d}_rgb_bilinear.png"
            if not args.no_pgm:
                atomic_pgm(pgm_dir / pgm_name, bayer)
            if not args.no_color:
                rgb = demosaic_bayer_rg8(bayer)
                png_path = png_dir / png_name
                atomic_png(png_path, rgb)
                png_paths.append(png_path)
            log.write(json.dumps({
                "sample": sample,
                "source": path.name,
                "pgm": None if args.no_pgm else f"bayer_pgm/{pgm_name}",
                "color_png": None if args.no_color else f"color_png/{png_name}",
                "bayer_pattern": "RGGB",
                "demosaic": None if args.no_color else "bilinear",
            }, separators=(",", ":")) + "\n")
            log.flush()
            print(f"[{position}/{len(selected)}] {path.name}", flush=True)

    if png_paths:
        montage(png_paths, output / "montage_rgb.png")
    report = {
        "complete": True,
        "source": str(source),
        "source_frames": len(frames),
        "converted_frames": len(selected),
        "every": args.every,
        "width": WIDTH,
        "height": HEIGHT,
        "source_pixel_format": "BayerRG8",
        "bayer_pattern": "RGGB",
        "pgm_pixel_values_identical_to_raw": not args.no_pgm,
        "color_png_demosaic": None if args.no_color else "bilinear",
        "elapsed_seconds": time.perf_counter() - started,
    }
    (output / "conversion_metadata.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
