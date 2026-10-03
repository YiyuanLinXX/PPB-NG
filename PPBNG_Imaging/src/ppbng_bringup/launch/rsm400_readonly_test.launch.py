from pathlib import Path
import math

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config_path():
    share = Path(get_package_share_directory("ppbng_bringup"))
    source = share.parents[3] / "src" / "ppbng_bringup" / "config" / "ppbng_config.yaml"
    return str(source if source.is_file() else share / "config" / "ppbng_config.yaml")


def _enabled(text, name):
    if text.strip().lower() in ("true", "1", "yes", "on"):
        return True
    raise RuntimeError(f"{name} must be explicitly true")


def _load(context):
    _enabled(LaunchConfiguration("hardware_enabled").perform(context), "hardware_enabled")
    path = Path(LaunchConfiguration("config_file").perform(context)).resolve(strict=True)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise RuntimeError("invalid PPB-NG configuration schema")
    values = document.get("rsm400", {})
    if values.get("enabled") is not True:
        raise RuntimeError("rsm400.enabled must be true")
    if values.get("allow_control") is not False:
        raise RuntimeError("read-only test requires rsm400.allow_control=false")
    if values.get("allow_observation_commands") is not True:
        raise RuntimeError("rsm400.allow_observation_commands must be true")
    rate = float(values.get("telemetry_hz", 0.0))
    if not math.isfinite(rate) or not 0.1 <= rate <= 100.0:
        raise RuntimeError("rsm400.telemetry_hz must be within 0.1..100 Hz")
    period = round(100.0 / rate)
    if abs(100.0 / period - rate) > 1e-9:
        raise RuntimeError("telemetry_hz must map exactly to a 10 ms MCP period")
    session = document.get("session", {})
    return [Node(
        package="ppbng_rsm400",
        executable="rsm400_production_node",
        name="rsm400",
        output="screen",
        parameters=[{
            "hardware_enabled": True,
            "com_port": str(values["port"]),
            "device_id": "rsm400",
            "allow_control": False,
            "allow_observation_commands": True,
            "telemetry_period_10ms": period,
            "allowed_output_root": str(session["output_root"]),
            "storage_stem": values.get("storage_stem", "rsm400"),
            "flush_every_records": int(values.get("flush_every_records", 10)),
            "features_confirmed": bool(values.get("features_confirmed", False)),
            "of002_available": bool(values.get("of002_available", False)),
            "of005_available": bool(values.get("of005_available", False)),
            "require_ready_telemetry": bool(values.get("require_ready_telemetry", False)),
            "stab_major_status_confirmed": bool(
                values.get("stab_major_status_confirmed", False)),
            "expected_stab_major_status": int(
                values.get("expected_stab_major_status", -1)),
            "maximum_ready_error_level": int(
                values.get("maximum_ready_error_level", 0)),
            "ready_timeout_ms": int(values.get("ready_timeout_ms", 2000)),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=_default_config_path()),
        OpaqueFunction(function=_load),
    ])
