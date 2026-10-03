"""Export every saved FX10e scan line as bounded-size RGB PNG tiles (read-only)."""
import argparse
import json
from pathlib import Path
import re
import sys
import uuid
from datetime import datetime, timezone
import zipfile

import numpy as np
from PIL import Image

from export_hsi_rgb_preview import ROOT, choose_dataset, choose_bands, read_json, read_part


def chunks(part, bands, tile_lines):
    """Copy bounded blocks; never skip a row, including the last partial tile."""
    cube = np.memmap(part['path'], mode='r', dtype='<u2', offset=part['offset'], shape=part['shape'])
    try:
        for start in range(0, part['shape'][0], tile_lines):
            end = min(start + tile_lines, part['shape'][0])
            pixels = np.empty((end - start, part['shape'][2], 3), dtype=np.uint16)
            for channel, band in enumerate(bands):
                pixels[:, :, channel] = cube[start:end, band, :]
            yield start, pixels
    finally:
        del cube


def histogram_bounds(histogram):
    """Exact linear 2/98 percentiles from uint16 histograms of ALL selected-band pixels."""
    bounds = np.empty((2, 3), dtype=np.float64)
    for channel in range(3):
        cumulative = histogram[channel].cumsum()
        count = int(cumulative[-1])
        if count == 0:
            raise ValueError('Empty pixel histogram')
        for row, quantile in enumerate((0.02, 0.98)):
            position = (count - 1) * quantile
            lower, upper = int(np.floor(position)), int(np.ceil(position))
            lo = np.searchsorted(cumulative, lower, side='right')
            hi = np.searchsorted(cumulative, upper, side='right')
            bounds[row, channel] = lo + (hi - lo) * (position - lower)
    return bounds


def render(pixels, bounds, gamma):
    scaled = pixels.astype(np.float32)
    scaled -= bounds[0].astype(np.float32)
    scaled /= np.maximum(bounds[1] - bounds[0], 1).astype(np.float32)
    np.clip(scaled, 0, 1, out=scaled)
    np.power(scaled, 1 / gamma, out=scaled)
    scaled *= 255
    return np.rint(scaled).astype(np.uint8)


def run(args):
    dataset = choose_dataset(args.dataset, args.data_root)
    print(f'Source (READ ONLY): {dataset}', flush=True)
    warnings = read_json(dataset / 'manifest.json').get('warnings', [])
    for warning in warnings:
        print(f'SOURCE WARNING: {warning}', flush=True)
    groups = {}
    for raw in (dataset / 'segments').glob('fx10e_segment_*_part_*.raw'):
        match = re.fullmatch(r'fx10e_segment_(\d+)_part_(\d+)\.raw', raw.name)
        if not match:
            raise ValueError(f'Unexpected FX10e filename: {raw.name}')
        segment, number = map(int, match.groups())
        part = read_part(raw)
        if part['kind'] not in ('sample', 'dark'):
            raise ValueError(f'Unsupported capture kind: {part["kind"]}')
        groups.setdefault(segment, []).append((number, part))
    if not groups:
        raise ValueError('No FX10e data found.')
    planned = []
    for segment, numbered in sorted(groups.items()):
        numbered.sort(key=lambda item: item[0])
        if [n for n, _ in numbered] != list(range(len(numbered))):
            raise ValueError(f'Missing part in segment {segment}')
        parts = [p for _, p in numbered]
        if len({(p['kind'], p['shape'][1:]) for p in parts}) != 1:
            raise ValueError(f'Inconsistent geometry/capture kind in segment {segment}')
        bands, mapping = choose_bands(dataset, parts[0]['shape'][1], args.calpack, args.bands)
        planned.append((segment, numbered, bands, mapping))
    output = args.output_directory or ROOT / 'data_exports' / dataset.name / (
        'hsi_rgb_full_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:6])
    output = output.resolve()
    if output == dataset or dataset in output.parents:
        raise ValueError('Output must be outside the source dataset.')
    output.mkdir(parents=True, exist_ok=False)
    report = {'source_dataset': str(dataset), 'source_manifest_warnings': warnings,
              'complete': False, 'line_step': 1, 'tile_lines_max': args.tile_lines,
              'processing': 'Every saved row; three selected bands only, NOT all spectral bands. '
                            '8-bit display RGB, no dark subtraction, FFC, reflectance or georectification. '
                            'Exact all-pixel 2/98 percentile stretch fixed within each segment.',
              'gamma': args.gamma, 'axes': 'columns=spatial pixels; rows=acquisition order, not distance',
              'integrity_scope': 'Header extents/index layout checked; CRC and timing continuity NOT verified. '
                                 'Coverage describes saved rows, not missing acquisition time.',
              'segments': []}
    metadata = output / 'full_export_metadata.json'
    metadata.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    for segment, numbered, bands, mapping in planned:
        kind = numbered[0][1]['kind']
        histogram = np.zeros((3, 65536), dtype=np.uint64)
        for number, part in numbered:
            print(f'Pass 1/2: {kind} segment {segment}, part {number}: all-line histogram', flush=True)
            for _, pixels in chunks(part, bands, args.tile_lines):
                for channel in range(3):
                    histogram[channel] += np.bincount(pixels[:, :, channel].ravel(), minlength=65536).astype(np.uint64)
        bounds = histogram_bounds(histogram)
        entry = {'segment': segment, 'capture_kind': kind, 'spectral_mapping': mapping,
                 'stretch_bounds_rgb': bounds.tolist(), 'source_lines': sum(p['shape'][0] for _, p in numbered),
                 'exported_lines': 0, 'source_parts': [str(p['path']) for _, p in numbered], 'tiles': []}
        folder = output / kind
        folder.mkdir(exist_ok=True)
        global_start = 0
        for number, part in numbered:
            for tile_index, (start, pixels) in enumerate(chunks(part, bands, args.tile_lines)):
                name = f'fx10e_segment_{segment:04d}_part_{number:04d}_tile_{tile_index:04d}_rgb.png'
                Image.fromarray(render(pixels, bounds, args.gamma)).save(folder / name)
                entry['tiles'].append({'png': f'{kind}/{name}', 'source_part': str(part['path']),
                                       'part_start_line_zero_based': start, 'lines': len(pixels),
                                       'segment_start_line_zero_based': global_start + start,
                                       'width': pixels.shape[1]})
                entry['exported_lines'] += len(pixels)
                print(f'Pass 2/2: saved {kind}/{name} ({len(pixels)} lines)', flush=True)
            global_start += part['shape'][0]
        if entry['exported_lines'] != entry['source_lines']:
            raise ValueError('Internal coverage mismatch; export is incomplete')
        report['segments'].append(entry)
        metadata.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    report['complete'] = True
    metadata.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f'COMPLETE: every saved FX10e sample AND dark row exported. Output: {output}', flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', nargs='?', default='latest')
    parser.add_argument('--data-root', type=Path, default=ROOT / 'data')
    parser.add_argument('--output-directory', type=Path, help='New directory outside dataset; existing paths refused.')
    parser.add_argument('--calpack', type=Path)
    parser.add_argument('--bands', nargs=3, type=int, metavar=('R', 'G', 'B'))
    parser.add_argument('--tile-lines', type=int, default=4096, help='Maximum rows per PNG, no subsampling (1-8192).')
    parser.add_argument('--gamma', type=float, default=2.2)
    args = parser.parse_args()
    if not 1 <= args.tile_lines <= 8192 or not np.isfinite(args.gamma) or args.gamma <= 0:
        parser.error('tile-lines must be 1-8192 and gamma positive and finite')
    try:
        run(args)
    except (OSError, ValueError, KeyError, StopIteration, zipfile.BadZipFile) as error:
        print(f'FULL EXPORT FAILED (partial output must not be treated as complete): {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
