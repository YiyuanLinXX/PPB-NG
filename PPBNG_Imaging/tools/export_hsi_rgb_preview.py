"""Read-only, bounded-size RGB quicklooks from finalized PPB-NG ENVI sessions."""
import argparse
import csv
import hashlib
import json
import re
import sys
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def choose_dataset(value, root):
    if value != "latest":
        result = Path(value).resolve()
    else:
        candidates = []
        for path in root.glob("*/manifest.json"):
            manifest = read_json(path)
            if manifest.get("state") == "finalized" and any(
                (path.parent / "segments").glob("fx10e_segment_*_part_*.raw")
            ):
                candidates.append((manifest.get("created_utc", ""), path.parent))
        if not candidates:
            raise ValueError("No finalized FX10e dataset found; provide a dataset directory.")
        result = max(candidates)[1]
    if read_json(result / "manifest.json").get("state") != "finalized":
        raise ValueError("Refusing an active or unfinalized dataset.")
    return result


def read_part(raw):
    fields = {}
    for line in raw.with_suffix(".hdr").read_text(encoding="utf-8-sig").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            fields[key.strip().lower()] = value.strip().lower()
    if (fields.get("interleave"), fields.get("data type"), fields.get("byte order")) != ("bil", "12", "0"):
        raise ValueError(f"Expected little-endian uint16 BIL: {raw.name}")
    shape = tuple(int(fields[key]) for key in ("lines", "bands", "samples"))
    offset = int(fields.get("header offset", "0"))
    if min(shape) <= 0 or offset < 0 or raw.stat().st_size != offset + int(np.prod(shape)) * 2:
        raise ValueError(f"Header/file-size mismatch: {raw.name}")
    frame_bytes = shape[1] * shape[2] * 2
    kinds = set()
    with raw.with_suffix(".index.csv").open(newline="", encoding="utf-8-sig") as stream:
        count = 0
        for count, row in enumerate(csv.DictReader(stream), 1):
            kinds.add(row["capture_kind"])
            if int(row["part_line"]) != count - 1 or int(row["raw_offset"]) != (count - 1) * frame_bytes or int(row["payload_bytes"]) != frame_bytes:
                raise ValueError(f"Noncontiguous or inconsistent index: {raw.name}")
    if count != shape[0] or len(kinds) != 1:
        raise ValueError(f"Index row count or capture-kind mismatch: {raw.name}")
    return {"path": raw, "shape": shape, "offset": offset, "kind": kinds.pop()}


def choose_bands(dataset, bands_count, calpack, explicit):
    if explicit is not None:
        if min(explicit) < 0 or max(explicit) >= bands_count:
            raise ValueError("Band indices are zero-based and must fit the saved cube.")
        return explicit, {"mode": "explicit_band_indices", "bands_zero_based_rgb": explicit}
    manifest = read_json(dataset / "manifest.json")
    device = next(d for d in manifest["devices"] if d["role"] == "fx10e")
    binning = int(device.get("actual_settings", {}).get("spectral_binning", 0))
    if binning not in (1, 2, 4, 8):
        raise ValueError("Actual spectral binning missing; use --bands after verifying band mapping.")
    if calpack is None:
        config = yaml.safe_load((dataset / "machine_config.snapshot.yaml").read_text(encoding="utf-8-sig"))
        calpack = config["hsi"]["fx10e"]["calibration_pack_path"]
    calpack = Path(calpack).resolve()
    entry = f"spectral/wlcal{binning}b.wls"
    with zipfile.ZipFile(calpack) as archive:
        calibration = ElementTree.fromstring(archive.read("manifest.xml"))
        serial = calibration.findtext("serialnumbers/sensor", "").strip()
        if not serial or serial != str(device["identity"]):
            raise ValueError("Calibration sensor identity does not match the dataset.")
        table = archive.read(entry).decode("utf-8-sig")
    wavelengths = np.array([float(line.split()[0]) for line in table.splitlines() if line.strip()])
    if len(wavelengths) != bands_count or not np.all(np.isfinite(wavelengths)) or not np.all(np.diff(wavelengths) > 0):
        raise ValueError("Wavelength table does not match saved spectral geometry; no rescaling is allowed.")
    targets = [650.0, 550.0, 450.0]
    if min(targets) < wavelengths.min() or max(targets) > wavelengths.max():
        raise ValueError("Visible RGB wavelengths are outside this calibration table.")
    indices = [int(np.argmin(abs(wavelengths - target))) for target in targets]
    return indices, {"mode": "nearest_visible_wavelength", "target_nm_rgb": targets,
                     "actual_nm_rgb": wavelengths[indices].tolist(), "bands_zero_based_rgb": indices,
                     "calpack": str(calpack), "calpack_sha256": hashlib.sha256(calpack.read_bytes()).hexdigest(),
                     "wavelength_table": entry, "spectral_binning": binning}


def extract(parts, bands, max_lines):
    total = sum(p["shape"][0] for p in parts)
    step = max(1, (total + max_lines - 1) // max_lines)
    selected = np.arange(0, total, step, dtype=np.int64)
    pixels = np.empty((len(selected), parts[0]["shape"][2], 3), dtype=np.uint16)
    begin = 0
    for part in parts:
        end = begin + part["shape"][0]
        positions = np.flatnonzero((selected >= begin) & (selected < end))
        rows = selected[positions] - begin
        cube = np.memmap(part["path"], mode="r", dtype="<u2", offset=part["offset"], shape=part["shape"])
        for channel, band in enumerate(bands):
            pixels[positions, :, channel] = cube[rows, band, :]
        del cube
        begin = end
    return pixels, step, total


def stretch(pixels, gamma):
    bounds = np.percentile(pixels, [2, 98], axis=(0, 1))
    denominator = np.maximum(bounds[1] - bounds[0], 1)
    scaled = np.clip((pixels.astype(np.float32) - bounds[0]) / denominator, 0, 1)
    return np.rint(scaled ** (1 / gamma) * 255).astype(np.uint8), bounds.tolist()


def run(args):
    dataset = choose_dataset(args.dataset, args.data_root)
    print(f"Source (read-only): {dataset}", flush=True)
    source_warnings = read_json(dataset / "manifest.json").get("warnings", [])
    for warning in source_warnings:
        if "fx10e" in str(warning).lower():
            print(f"SOURCE WARNING (not a preview failure): {warning}", flush=True)
    groups = {}
    skipped_dark = 0
    pattern = re.compile(r"fx10e_segment_(\d+)_part_(\d+)\.raw$")
    for raw in (dataset / "segments").glob("fx10e_segment_*_part_*.raw"):
        match = pattern.fullmatch(raw.name)
        if not match:
            continue
        part = read_part(raw)
        if part["kind"] == "dark":
            skipped_dark += part["shape"][0]
            continue
        if part["kind"] != "sample":
            raise ValueError(f"Unsupported capture kind: {part['kind']}")
        segment, number = map(int, match.groups())
        groups.setdefault(segment, []).append((number, part))
    if not groups:
        raise ValueError("No FX10e sample data found.")
    planned = []
    for segment, numbered in sorted(groups.items()):
        numbered.sort(key=lambda item: item[0])
        if [n for n, _ in numbered] != list(range(len(numbered))):
            raise ValueError(f"Missing part in segment {segment}.")
        parts = [p for _, p in numbered]
        if len({p["shape"][1:] for p in parts}) != 1:
            raise ValueError(f"Inconsistent geometry in segment {segment}.")
        bands, mapping = choose_bands(dataset, parts[0]["shape"][1], args.calpack, args.bands)
        planned.append((segment, parts, bands, mapping))
    output = args.output_directory or ROOT / "data_exports" / dataset.name / (
        "hsi_rgb_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:6])
    output = output.resolve()
    if output == dataset or dataset in output.parents:
        raise ValueError("Output must be outside the source dataset.")
    output.mkdir(parents=True, exist_ok=False)
    report = {"source_dataset": str(dataset), "dark_lines_excluded": skipped_dark,
              "source_manifest_warnings": source_warnings,
              "processing": "SDK counts; no dark subtraction, FFC, reflectance or colorimetric correction; independent 2-98 percentile channel stretch",
              "gamma": args.gamma, "axes": "columns=cross-track spatial pixels; rows=increasing acquisition order (not distance or georectified)",
              "integrity_scope": "Header sizes and index layout checked; payload CRC and timing gaps NOT verified. Downsampled quicklook can miss brief defects.",
              "segments": []}
    for segment, parts, bands, mapping in planned:
        pixels, step, total = extract(parts, bands, args.max_lines)
        rgb, bounds = stretch(pixels, args.gamma)
        name = f"fx10e_segment_{segment}_rgb.png"
        Image.fromarray(rgb).save(output / name)
        report["segments"].append({"png": name, "segment": segment, "source_lines": total,
                                   "preview_lines": len(rgb), "line_step": step, "first_source_line_zero_based": 0,
                                   "source_parts": [str(p["path"]) for p in parts],
                                   "spectral_mapping": mapping, "stretch_bounds_rgb": bounds,
                                   "width": rgb.shape[1]})
        print(f"Saved {name}: {rgb.shape[1]} x {len(rgb)}; {total} source lines, step {step}", flush=True)
    (output / "preview_metadata.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Output: {output}", flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", nargs="?", default="latest", help="Dataset directory or latest (default).")
    parser.add_argument("--data-root", type=Path, default=ROOT / "data")
    parser.add_argument("--output-directory", type=Path, help="New directory outside source; existing paths refused.")
    parser.add_argument("--calpack", type=Path, help="Override relocated SCP path; otherwise use session configuration snapshot.")
    parser.add_argument("--bands", nargs=3, type=int, metavar=("R", "G", "B"), help="Explicit zero-based bands; bypass wavelength calibration, potentially false color.")
    parser.add_argument("--max-lines", type=int, default=4096, help="Maximum preview rows per segment (default 4096).")
    parser.add_argument("--gamma", type=float, default=2.2, help="Display gamma; 1 for linear (default 2.2).")
    args = parser.parse_args()
    if args.max_lines < 1 or not np.isfinite(args.gamma) or args.gamma <= 0:
        parser.error("max-lines and gamma must be positive and finite")
    try:
        run(args)
    except (OSError, ValueError, KeyError, StopIteration, zipfile.BadZipFile) as error:
        print(f"HSI PREVIEW FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
