"""Offline FX10e/RGB/thermal gallery. Source datasets are strictly read-only."""
import argparse
import html
import json
from pathlib import Path
import re
import sys
from datetime import datetime, timezone
import uuid

import numpy as np
from PIL import Image

import export_hsi_rgb_preview as hsi
import export_snapshot_images as snapshots


def spaced_samples(records, count):
    """Include both ends when count >= 2; IDs need not be contiguous."""
    if not records:
        raise ValueError("No snapshot frames were recorded")
    if count <= 0:
        raise ValueError("count must be positive")
    n = min(count, len(records))
    indices = [len(records) // 2] if n == 1 else [i * (len(records) - 1) // (n - 1) for i in range(n)]
    return {records[i]["sample_id"] for i in indices}


def export_samples(dataset, output, role, manifest, count):
    actual = snapshots.settings(manifest, role)
    expected_format = "BayerRG8" if role == "rgb" else "Mono16"
    if actual.get("pixel_format") != expected_format:
        raise ValueError(f"Unsupported {role} pixel format")
    width = int(actual["width"])
    height = int(actual["height" if role == "rgb" else "transport_height"])
    if width <= 0 or height <= 1 or (role == "thermal" and (width, height) != (640, 513)):
        raise ValueError(f"Unsupported {role} geometry")
    expected_bytes = width * height * (1 if role == "rgb" else 2)
    paths = list((dataset / "segments").glob(f"{role}_*.ppbseg"))
    paths.sort(key=lambda p: int(re.fullmatch(rf"{role}_(\d+)\.ppbseg", p.name).group(1)))
    records = []
    previous = 0
    print(f"{role}: scanning record headers, not all image payloads...", flush=True)
    for path in paths:
        for envelope, _ in snapshots.read_segment(path, metadata_only=True):
            if envelope["sample_id"] <= previous or envelope["payload_bytes"] != expected_bytes:
                raise ValueError(f"Duplicate/regressing sample ID or geometry mismatch: {path.name}")
            previous = envelope["sample_id"]
            records.append(envelope)
    selected = spaced_samples(records, count)
    associations = {}
    sidecar = dataset / "segments" / f"{role}_association.ndjson"
    if sidecar.exists():
        with sidecar.open(encoding="utf-8-sig") as stream:
            for line in stream:
                item = json.loads(line)
                if item.get("sample") in selected:
                    associations[item["sample"]] = item
    folder = output / role
    folder.mkdir()
    result = []
    for path in paths:
        if not any(r["source_segment"] == path.name and r["sample_id"] in selected for r in records):
            continue
        for envelope, payload in snapshots.read_segment(path, selected_samples=selected):
            sample = envelope["sample_id"]
            if payload is None or len(payload) != expected_bytes:
                raise ValueError("Selected payload unavailable or inconsistent")
            association = associations.get(sample)
            if role == "rgb":
                pixels = snapshots.demosaic_bayer_rg8(payload, width, height)
                description = "Bilinear BayerRG8 preview; no additional white balance or color correction"
            else:
                transport = np.frombuffer(payload, dtype="<u2").reshape(height, width)
                pixels = snapshots.thermal_false_color(transport[1:])
                description = "Counts-only false color; independent 1-99 percentile scale; NOT temperature"
            target = folder / f"frame_{sample:08d}.png"
            snapshots.save_png(target, pixels)
            result.append({**envelope, "png": target.relative_to(output).as_posix(),
                           "association": association, "processing": description,
                           "payload_crc_verified": True})
            print(f"{role}: preview sample {sample}", flush=True)
    if len(result) != len(selected):
        raise ValueError(f"{role}: selected frame coverage changed during export")
    return {"record_count": len(records), "selected_frames": result,
            "selection": "evenly spaced by recorded frame ordinal, including endpoints",
            "integrity_scope": "All record headers/trailers/extents; selected payload CRC only"}


def write_gallery(output, report):
    parts = ['<!doctype html><html lang="en"><meta charset="utf-8"><title>PPB-NG quicklook</title>',
             '<style>body{font:16px system-ui;margin:24px;background:#182027;color:#eee}a{color:#8bd5ff}.grid{display:flex;flex-wrap:wrap;gap:16px}figure{margin:0;width:300px}img{width:300px;height:240px;object-fit:contain;background:#080c10}figcaption{overflow-wrap:anywhere}.warning{color:#ffb56b}</style>',
             '<h1>PPB-NG visual quicklook</h1>',
             '<p>' + html.escape(report['source_dataset']) + '</p>',
             '<p>Sampled visual inspection only. Not full integrity, continuity, temperature or synchronization validation. Images across devices are independently sampled, not time-matched.</p>']
    for warning in report['source_manifest_warnings']:
        parts.append('<p class="warning">Source warning: ' + html.escape(str(warning)) + '</p>')
    for role, items in [('fx10e', report['fx10e']['segments'])] + [
            (r, report[r]['selected_frames']) for r in ('rgb', 'thermal')]:
        parts.append('<h2>' + role.upper() + '</h2><div class="grid">')
        for item in items:
            relative = ('fx10e/' if role == 'fx10e' else '') + item['png']
            with Image.open(output / relative) as image:
                image.thumbnail((600, 480))
                thumb = relative.replace('.png', '_thumb.png')
                image.save(output / thumb)
            if role == 'fx10e':
                label = f"Segment {item['segment']}: {item['source_lines']} source lines; display stride {item['line_step']}. Stretched counts, no FFC; axes not equal-distance."
            else:
                association = item.get('association') or {}
                label = f"Sample {item['sample_id']}; SDK frame {association.get('sdk_frame', 'unavailable')}; trigger {association.get('trigger_sequence', 'unavailable')}. {item['processing']}"
            parts.append(f'<figure><a href="{html.escape(relative, quote=True)}"><img src="{html.escape(thumb, quote=True)}" loading="lazy"></a><figcaption>{html.escape(label)}</figcaption></figure>')
        parts.append('</div>')
    parts.append('<p>Click an image to open its larger preview. Metadata: quicklook_metadata.json.</p></html>')
    (output / 'index.html').write_text('\n'.join(parts), encoding='utf-8')


def run(args):
    dataset = hsi.choose_dataset(args.dataset, args.data_root)
    manifest = hsi.read_json(dataset / 'manifest.json')
    output = (args.output_directory or hsi.ROOT / 'data_exports' / dataset.name / (
        'quicklook_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:6])).resolve()
    if output == dataset or dataset in output.parents:
        raise ValueError('Output must be outside source dataset')
    output.mkdir(parents=True, exist_ok=False)
    report = {'source_dataset': str(dataset), 'source_manifest_warnings': manifest.get('warnings', []),
              'complete': False, 'temperature_conversion_applied': False}
    try:
        for role in ('rgb', 'thermal'):
            report[role] = export_samples(dataset, output, role, manifest, args.count)
        hsi.run(argparse.Namespace(dataset=str(dataset), data_root=args.data_root,
                output_directory=output / 'fx10e', calpack=args.calpack, bands=args.bands,
                max_lines=args.max_lines, gamma=2.2))
        report['fx10e'] = hsi.read_json(output / 'fx10e' / 'preview_metadata.json')
        write_gallery(output, report)
        report['complete'] = True
    except Exception as error:
        report['error'] = str(error)
        (output / 'QUICKLOOK_INCOMPLETE.txt').write_text(str(error) + '\n', encoding='utf-8')
        raise
    finally:
        (output / 'quicklook_metadata.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f"Open gallery: {output / 'index.html'}", flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', help='Explicit finalized dataset path; latest is also supported')
    parser.add_argument('--data-root', type=Path, default=hsi.ROOT / 'data')
    parser.add_argument('--output-directory', type=Path)
    parser.add_argument('--count', type=int, default=10, help='RGB and thermal samples per camera')
    parser.add_argument('--max-lines', type=int, default=2048, help='Maximum FX10e overview rows per scene segment')
    parser.add_argument('--calpack', type=Path)
    parser.add_argument('--bands', type=int, nargs=3, metavar=('R', 'G', 'B'))
    args = parser.parse_args()
    if not 1 <= args.count <= 100 or not 1 <= args.max_lines <= 16384:
        parser.error('count must be 1..100 and max-lines 1..16384')
    try:
        run(args)
    except Exception as error:
        print(f'QUICKLOOK FAILED: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
