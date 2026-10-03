"""Read-only storage qualification planner for the frozen PPB-NG configuration."""

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

import yaml


MIB = 1024 ** 2
GIB = 1024 ** 3


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _positive_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def estimated_payload_rate(document):
    """Mirror production.launch.py's all-stream payload estimate exactly."""
    hsi = document["hsi"]
    timing = document["timing"]["channels"]
    fx = hsi["fx10e"]
    swir = hsi["swir"]
    rgb = document["rgb"]["acquisition"]

    def channel_rate(name):
        channel = timing[name]
        if channel.get("enabled") is not True:
            raise ValueError(f"timing.channels.{name} must be enabled")
        return (_positive_integer(channel.get("rate_numerator_hz"),
                                  f"timing.channels.{name}.rate_numerator_hz") /
                _positive_integer(channel.get("rate_denominator"),
                                  f"timing.channels.{name}.rate_denominator"))

    rgb_rate = channel_rate("rgb")
    thermal_rate = channel_rate("thermal")
    if rgb_rate != thermal_rate:
        raise ValueError("RGB and thermal logical channels must share one rate")
    fx_rate = _positive_number(fx["line_rate_hz"], "hsi.fx10e.line_rate_hz")
    swir_rate = _positive_number(swir["line_rate_hz"], "hsi.swir.line_rate_hz")
    total = (
        _positive_integer(fx["spatial_samples"], "hsi.fx10e.spatial_samples") *
        _positive_integer(fx["spectral_bands"], "hsi.fx10e.spectral_bands") * 2 * fx_rate +
        _positive_integer(swir["spatial_samples"], "hsi.swir.spatial_samples") *
        _positive_integer(swir["spectral_bands"], "hsi.swir.spectral_bands") * 2 * swir_rate +
        _positive_integer(rgb["payload_bytes"], "rgb.acquisition.payload_bytes") * rgb_rate +
        640 * 513 * 2 * thermal_rate
    )
    return math.ceil(total)


def make_plan(document, available_bytes, duration_seconds=20, block_mib=16,
              maximum_test_mib=8192):
    if not 1 <= duration_seconds <= 60:
        raise ValueError("duration_seconds must be within 1..60")
    if not 1 <= block_mib <= 64:
        raise ValueError("block_mib must be within 1..64")
    if not 1 <= maximum_test_mib <= 16384 or maximum_test_mib < block_mib:
        raise ValueError("maximum_test_mib must be within 1..16384 and hold one block")
    session = document["session"]
    rate = estimated_payload_rate(document)
    headroom = session["capacity_headroom_basis_points"]
    if (isinstance(headroom, bool) or not isinstance(headroom, int) or
            not 0 <= headroom <= 10_000):
        raise ValueError("capacity headroom must not exceed 10000 basis points")
    planned = _positive_integer(session["planned_duration_seconds"],
                                "session.planned_duration_seconds")
    reserve = _positive_integer(session["minimum_reserve_bytes"],
                                "session.minimum_reserve_bytes")
    factor = 10_000 + headroom
    required_rate = (rate * factor + 9_999) // 10_000
    required_capacity = (rate * planned * factor + 9_999) // 10_000 + reserve
    qualification_capacity = maximum_test_mib * MIB + 100 * GIB
    return {
        "estimated_payload_bytes_per_second": rate,
        "required_durable_bytes_per_second": required_rate,
        "planned_task_required_available_bytes": required_capacity,
        "qualification_required_available_bytes": qualification_capacity,
        "available_bytes": available_bytes,
        "planned_task_capacity_ready": available_bytes >= required_capacity,
        "qualification_capacity_ready": available_bytes >= qualification_capacity,
        "duration_seconds": duration_seconds,
        "block_mib": block_mib,
        "maximum_test_mib": maximum_test_mib,
    }


def main(argv=None):
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Read-only plan; this command never creates, writes, or deletes a file.")
    parser.add_argument("--config", type=Path,
                        default=workspace / "src/ppbng_bringup/config/ppbng_config.yaml")
    parser.add_argument("--duration-seconds", type=int, default=20)
    parser.add_argument("--block-mib", type=int, default=16)
    parser.add_argument("--maximum-test-mib", type=int, default=8192)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        config_path = args.config.resolve(strict=True)
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        output_root = Path(document["session"]["output_root"]).resolve(strict=True)
        if not output_root.is_dir():
            raise ValueError("session.output_root must be an existing directory")
        plan = make_plan(document, shutil.disk_usage(output_root).free,
                         args.duration_seconds, args.block_mib, args.maximum_test_mib)
        executable = (workspace / "install/ppbng_storage/lib/ppbng_storage/"
                      "ppbng_storage_durable_write_qualification.exe").resolve()
        command = [str(executable), "--confirm-durable-write-qualification",
                   "--output-root", str(output_root), "--duration-seconds",
                   str(args.duration_seconds), "--block-mib", str(args.block_mib),
                   "--maximum-test-mib", str(args.maximum_test_mib)]
        result = {"mode": "READ_ONLY_NO_WRITE", "config": str(config_path),
                  "output_root": str(output_root), **plan,
                  "qualification_command": command}
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"PREFLIGHT REFUSED: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print("READ-ONLY STORAGE QUALIFICATION PLAN (no file was written)")
        for key, value in result.items():
            if key != "qualification_command":
                print(f"{key}: {value}")
        print("qualification_command:")
        print("  " + " ".join(f'\"{item}\"' if " " in item else item for item in command))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
