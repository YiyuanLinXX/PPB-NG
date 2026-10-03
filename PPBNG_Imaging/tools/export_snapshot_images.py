#!/usr/bin/env python3
"""Losslessly export PPB-NG RGB and thermal PPBSEG records as image files.

The source dataset is opened read-only.  PPBSEG headers, record headers,
payloads, and commit trailers are checked before an image is published.
"""

from __future__ import annotations

import argparse
import json
import random
import struct
import sys
import time
import zlib
from pathlib import Path
from typing import BinaryIO, Iterator

import numpy as np
from PIL import Image


FILE_MAGIC = b"PPBNGSG1"
FILE_HEADER_SIZE = 16
RECORD_HEADER_SIZE = 72
TRAILER_SIZE = 24
TIME_QUALITY = {0: "UNSYNCED", 1: "HOLDOVER", 2: "LOCKED"}


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def exact_read(stream: BinaryIO, size: int, description: str) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise RuntimeError(f"truncated {description}: expected {size} bytes, got {len(data)}")
    return data


def read_segment(path: Path, *, metadata_only: bool = False,
                 selected_samples: set[int] | None = None) -> Iterator[tuple[dict, bytes | None]]:
    """Default: verify every payload. Quicklook modes seek past unselected pixels.

    Every visited header/trailer and extent is checked in either mode. Skipped
    payload CRCs are NOT checked; callers must not claim full verification.
    """
    with path.open("rb") as stream:
        file_size = path.stat().st_size
        header = exact_read(stream, FILE_HEADER_SIZE, f"file header in {path.name}")
        if (
            header[:8] != FILE_MAGIC
            or struct.unpack_from("<H", header, 8)[0] != 1
            or struct.unpack_from("<I", header, 12)[0] != FILE_HEADER_SIZE
        ):
            raise RuntimeError(f"invalid PPBSEG file header: {path}")

        offset = FILE_HEADER_SIZE
        while True:
            record_header = stream.read(RECORD_HEADER_SIZE)
            if not record_header:
                return
            if len(record_header) != RECORD_HEADER_SIZE:
                raise RuntimeError(f"partial record header at {path.name}:{offset}")
            if (
                record_header[:4] != b"FRM1"
                or struct.unpack_from("<H", record_header, 4)[0] != 1
                or struct.unpack_from("<H", record_header, 6)[0] != RECORD_HEADER_SIZE
            ):
                raise RuntimeError(f"invalid record header at {path.name}:{offset}")
            if crc32(record_header[:68]) != struct.unpack_from("<I", record_header, 68)[0]:
                raise RuntimeError(f"record-header CRC mismatch at {path.name}:{offset}")

            sample_id = struct.unpack_from("<Q", record_header, 8)[0]
            trigger_id = struct.unpack_from("<Q", record_header, 16)[0]
            pps_sequence = struct.unpack_from("<Q", record_header, 24)[0]
            utc_nanoseconds = struct.unpack_from("<q", record_header, 32)[0]
            quality = record_header[40]
            payload_size = struct.unpack_from("<Q", record_header, 48)[0]
            payload_crc = struct.unpack_from("<I", record_header, 56)[0]
            total_size = struct.unpack_from("<Q", record_header, 60)[0]
            expected_total = RECORD_HEADER_SIZE + payload_size + TRAILER_SIZE
            if quality not in TIME_QUALITY or total_size != expected_total:
                raise RuntimeError(f"invalid record fields at {path.name}:{offset}")

            if total_size > file_size - offset:
                raise RuntimeError(f"truncated record at {path.name}:{offset}")
            selected = selected_samples is None or sample_id in selected_samples
            if metadata_only or not selected:
                stream.seek(payload_size, 1)
                payload = None
            else:
                payload = exact_read(stream, payload_size, f"payload at {path.name}:{offset}")
            trailer = exact_read(stream, TRAILER_SIZE, f"trailer at {path.name}:{offset}")
            if (
                trailer[:4] != b"CMIT"
                or struct.unpack_from("<H", trailer, 4)[0] != 1
                or struct.unpack_from("<H", trailer, 6)[0] != TRAILER_SIZE
                or struct.unpack_from("<Q", trailer, 8)[0] != total_size
                or struct.unpack_from("<I", trailer, 16)[0] != payload_crc
                or crc32(trailer[:20]) != struct.unpack_from("<I", trailer, 20)[0]
                or (payload is not None and crc32(payload) != payload_crc)
            ):
                raise RuntimeError(f"payload or commit-trailer CRC mismatch at {path.name}:{offset}")

            envelope = {
                "sample_id": sample_id,
                "trigger_id": trigger_id,
                "pps_sequence": pps_sequence,
                "utc_nanoseconds": utc_nanoseconds,
                "time_quality": TIME_QUALITY[quality],
                "payload_bytes": payload_size,
                "payload_crc32": f"{payload_crc:08x}",
                "source_segment": path.name,
                "source_record_offset": offset,
            }
            if metadata_only or selected:
                yield envelope, payload
            offset += total_size


def load_last_records(path: Path, aliases: dict[str, str] | None = None) -> dict[tuple[str, int], dict]:
    records: dict[tuple[str, int], dict] = {}
    if not path.is_file():
        return records
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            source_device = str(value.get("device_id", ""))
            if aliases is not None and source_device not in aliases:
                continue
            device = aliases.get(source_device, source_device) if aliases is not None else source_device
            sample = value.get("sample")
            if isinstance(sample, int):
                records[(device, sample)] = value
    return records


def load_associations(path: Path, role: str) -> dict[tuple[str, int], dict]:
    records: dict[tuple[str, int], dict] = {}
    if not path.is_file():
        return records
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            sample = value.get("sample")
            if isinstance(sample, int):
                records[(role, sample)] = value
    return records


def atomic_write(path: Path, writer) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    writer(temporary)
    temporary.replace(path)


def write_rgb_pgm(path: Path, payload: bytes, width: int, height: int) -> None:
    expected = width * height
    if len(payload) != expected:
        raise RuntimeError(f"RGB sample has {len(payload)} bytes; expected {expected}")

    def writer(temporary: Path) -> None:
        with temporary.open("wb") as stream:
            stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            stream.write(payload)

    atomic_write(path, writer)


def demosaic_bayer_rg8(payload: bytes, width: int, height: int) -> np.ndarray:
    bayer = np.frombuffer(payload, dtype=np.uint8).reshape(height, width)
    p = np.pad(bayer.astype(np.uint16), 1, mode="edge")
    c = p[1:-1, 1:-1]
    u, d = p[:-2, 1:-1], p[2:, 1:-1]
    l, r = p[1:-1, :-2], p[1:-1, 2:]
    ul, ur = p[:-2, :-2], p[:-2, 2:]
    dl, dr = p[2:, :-2], p[2:, 2:]
    rgb = np.empty((height, width, 3), dtype=np.uint8)
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


def save_png(path: Path, array: np.ndarray, mode: str | None = None) -> None:
    def writer(temporary: Path) -> None:
        image = Image.fromarray(array, mode=mode) if mode else Image.fromarray(array)
        image.save(temporary, format="PNG", optimize=False)

    atomic_write(path, writer)


def thermal_false_color(image: np.ndarray) -> np.ndarray:
    low, high = np.percentile(image, [1.0, 99.0]).astype(float)
    if high <= low:
        return np.zeros((*image.shape, 3), dtype=np.uint8)
    values = np.clip((image.astype(np.float32) - low) / (high - low), 0.0, 1.0)
    positions = np.array([0.0, 0.18, 0.38, 0.58, 0.78, 1.0], dtype=np.float32)
    colors = np.array([[0, 0, 0], [35, 10, 92], [135, 20, 115], [225, 55, 45], [252, 170, 20], [255, 255, 230]], dtype=np.float32)
    channels = [np.interp(values, positions, colors[:, channel]) for channel in range(3)]
    return np.stack(channels, axis=-1).round().clip(0, 255).astype(np.uint8)


def settings(manifest: dict, role: str) -> dict:
    for device in manifest.get("devices", []):
        if device.get("role") == role:
            return device.get("actual_settings", {})
    raise RuntimeError(f"manifest has no device entry for {role}")


def export_role(
    source: Path,
    output: Path,
    role: str,
    geometry: tuple[int, int],
    associations: dict[tuple[str, int], dict],
    contexts: dict[tuple[str, int], dict],
    every: int,
    limit: int,
    rgb_color_every: int,
    thermal_false_every: int,
    selected_samples: set[int] | None,
) -> dict:
    role_output = output / role
    role_output.mkdir(parents=True, exist_ok=False)
    index_path = role_output / "frame_metadata.ndjson"
    if role == "rgb":
        primary = role_output / "bayer_pgm"
        preview = role_output / "color_preview_png"
    else:
        primary = role_output / "radiometric_counts_png"
        transport = role_output / "transport_640x513_png"
        preview = role_output / "false_color_preview_png"
        transport.mkdir()
    primary.mkdir()
    if (role == "rgb" and rgb_color_every) or (role == "thermal" and thermal_false_every):
        preview.mkdir()

    segment_paths = sorted((source / "segments").glob(f"{role}_*.ppbseg"))
    if not segment_paths:
        raise RuntimeError(f"no {role}_*.ppbseg files found")

    seen = exported = total_payload = 0
    started = time.perf_counter()
    with index_path.open("x", encoding="utf-8", newline="\n") as index:
        stop = False
        for segment_path in segment_paths:
            for envelope, payload in read_segment(segment_path):
                ordinal = seen
                seen += 1
                if selected_samples is not None and envelope["sample_id"] not in selected_samples:
                    continue
                if ordinal % every:
                    continue
                if limit and exported >= limit:
                    stop = True
                    break
                sample = envelope["sample_id"]
                name = f"frame_{sample:08d}"
                files: dict[str, str | None] = {}
                if role == "rgb":
                    width, height = geometry
                    image_path = primary / f"{name}_bayer_rg8.pgm"
                    write_rgb_pgm(image_path, payload, width, height)
                    files["bayer_pgm"] = image_path.relative_to(output).as_posix()
                    files["color_preview_png"] = None
                    if rgb_color_every and exported % rgb_color_every == 0:
                        color_path = preview / f"{name}_rgb_bilinear.png"
                        save_png(color_path, demosaic_bayer_rg8(payload, width, height), "RGB")
                        files["color_preview_png"] = color_path.relative_to(output).as_posix()
                else:
                    width, height = geometry
                    expected = width * height * 2
                    if len(payload) != expected:
                        raise RuntimeError(f"thermal sample has {len(payload)} bytes; expected {expected}")
                    array = np.frombuffer(payload, dtype="<u2").reshape(height, width)
                    full_path = transport / f"{name}_transport_mono16.png"
                    counts_path = primary / f"{name}_counts16.png"
                    save_png(full_path, array)
                    save_png(counts_path, array[1:])
                    files["transport_png"] = full_path.relative_to(output).as_posix()
                    files["radiometric_counts_png"] = counts_path.relative_to(output).as_posix()
                    files["false_color_preview_png"] = None
                    if thermal_false_every and exported % thermal_false_every == 0:
                        color_path = preview / f"{name}_false_color.png"
                        save_png(color_path, thermal_false_color(array[1:]), "RGB")
                        files["false_color_preview_png"] = color_path.relative_to(output).as_posix()

                record = {
                    "device_id": role,
                    **envelope,
                    "files": files,
                    "association": associations.get((role, sample)),
                    "frame_context": contexts.get((role, sample)),
                }
                index.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
                index.flush()
                exported += 1
                total_payload += len(payload)
                if exported == 1 or exported % 25 == 0:
                    print(f"{role}: exported {exported} frame(s); verified through {segment_path.name}", flush=True)
            if stop:
                break

    return {
        "source_records_seen": seen,
        "exported_frames": exported,
        "exported_payload_bytes": total_payload,
        "segment_files": [path.name for path in segment_paths],
        "elapsed_seconds": time.perf_counter() - started,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="finalized PPB-NG dataset directory")
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--devices", choices=("both", "rgb", "thermal"), default="both")
    parser.add_argument("--every", type=int, default=1, help="export every Nth frame")
    parser.add_argument("--limit", type=int, default=0, help="maximum frames per device; 0 means all")
    parser.add_argument("--random-count", type=int, default=0, help="randomly export this many frames per device; 0 disables")
    parser.add_argument("--random-seed", type=int, default=20260919, help="reproducible seed used by --random-count")
    parser.add_argument("--rgb-color-preview-every", type=int, default=0, help="also demosaic every Nth exported RGB frame; 0 disables")
    parser.add_argument("--thermal-false-color-preview-every", type=int, default=0, help="also create false color every Nth exported thermal frame; 0 disables")
    args = parser.parse_args()
    for name in ("every", "limit", "random_count", "rgb_color_preview_every", "thermal_false_color_preview_every"):
        value = getattr(args, name)
        if value < (1 if name == "every" else 0):
            parser.error(f"--{name.replace('_', '-')} has an invalid value")

    source = args.dataset.resolve()
    if not (source / "manifest.json").is_file() or not (source / "segments").is_dir():
        parser.error(f"not a PPB-NG dataset: {source}")
    manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("state") != "finalized":
        parser.error("the source dataset is not finalized; refusing to export a live dataset")
    if args.output_directory:
        output = args.output_directory.resolve()
    elif source.parent.name.lower() == "data":
        output = source.parent.parent / "data_exports" / source.name
    else:
        output = source.parent / f"{source.name}_images"
    if output.exists():
        parser.error(f"output already exists; choose a new --output-directory: {output}")
    output.mkdir(parents=True)

    roles = {"rgb", "thermal"} if args.devices == "both" else {args.devices}
    context_aliases = {role: role for role in roles}
    for device in manifest.get("devices", []):
        role = str(device.get("role", ""))
        identity = str(device.get("identity", ""))
        if role in roles and identity:
            context_aliases[identity] = role
    contexts = load_last_records(source / "segments" / "frame_context.ndjson", context_aliases)
    associations = {}
    for role in roles:
        associations.update(load_associations(source / "segments" / f"{role}_association.ndjson", role))

    results = {}
    try:
        if "rgb" in roles:
            actual = settings(manifest, "rgb")
            if actual.get("pixel_format") != "BayerRG8":
                raise RuntimeError(f"unsupported RGB pixel format: {actual.get('pixel_format')}")
            results["rgb"] = export_role(
                source, output, "rgb", (int(actual["width"]), int(actual["height"])),
                associations, contexts, args.every, args.limit,
                args.rgb_color_preview_every, args.thermal_false_color_preview_every,
                set(random.Random(args.random_seed).sample(
                    sorted(sample for role, sample in associations if role == "rgb"),
                    min(args.random_count, len({sample for role, sample in associations if role == "rgb"}))))
                if args.random_count else None)
        if "thermal" in roles:
            actual = settings(manifest, "thermal")
            if actual.get("pixel_format") != "Mono16":
                raise RuntimeError(f"unsupported thermal pixel format: {actual.get('pixel_format')}")
            results["thermal"] = export_role(
                source, output, "thermal", (int(actual["width"]), int(actual["transport_height"])),
                associations, contexts, args.every, args.limit,
                args.rgb_color_preview_every, args.thermal_false_color_preview_every,
                set(random.Random(args.random_seed + 1).sample(
                    sorted(sample for role, sample in associations if role == "thermal"),
                    min(args.random_count, len({sample for role, sample in associations if role == "thermal"}))))
                if args.random_count else None)
    except Exception:
        (output / "EXPORT_INCOMPLETE.txt").write_text(
            "Export did not complete. Source dataset was not modified.\n", encoding="utf-8")
        raise

    report = {
        "complete": True,
        "source_dataset": str(source),
        "source_dataset_read_only": True,
        "source_session_id": manifest.get("session_id"),
        "every": args.every,
        "limit_per_device": args.limit,
        "random_count_per_device": args.random_count,
        "random_seed": args.random_seed if args.random_count else None,
        "rgb_primary_format": "lossless BayerRG8 PGM; no demosaic or color correction",
        "thermal_primary_format": "lossless 16-bit PNG of 512 radiometric-count rows",
        "thermal_transport_format": "lossless 16-bit PNG of all 513 rows including FLIR transport metadata row",
        "temperature_conversion_applied": False,
        "results": results,
    }
    (output / "export_manifest.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output / "README.txt").write_text(
        "PPB-NG snapshot image export\n\n"
        "The source .ppbseg files remain the authoritative, byte-preserving records.\n"
        "RGB bayer_pgm files preserve BayerRG8 values exactly. Color PNGs are previews.\n"
        "Thermal counts16 PNGs preserve radiometric counts, but are not temperatures.\n"
        "Thermal transport PNGs additionally retain the first FLIR metadata/header row.\n"
        "frame_metadata.ndjson links every export to its source record and saved metadata.\n",
        encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print(f"Export complete: {output}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"EXPORT FAILED: {error}", file=sys.stderr, flush=True)
        raise SystemExit(1)
