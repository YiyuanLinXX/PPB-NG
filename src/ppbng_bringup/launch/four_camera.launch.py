"""Inert four-camera hardware nodes for the interactive acquisition workflow.

The launch file opens no device by itself.  The companion PowerShell workflow
performs Prepare/Arm/Start calls and arms the UNO trigger only after every
camera is ready.
"""

from importlib.util import module_from_spec, spec_from_file_location
import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _helpers():
    path = Path(__file__).with_name("production.launch.py")
    spec = spec_from_file_location("ppbng_four_camera_helpers", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _default_config_path():
    share = Path(get_package_share_directory("ppbng_bringup"))
    source = share.parents[3] / "src" / "ppbng_bringup" / "config" / "ppbng_config.yaml"
    return str(source if source.is_file() else share / "config" / "ppbng_config.yaml")


def _require_hardware_flag(text):
    if text.strip().lower() not in ("true", "1", "yes", "on"):
        raise RuntimeError("pass hardware_enabled:=true explicitly for the four-camera workflow")


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise RuntimeError(f"{name} must be a positive integer")
    return value


def _load(context):
    _require_hardware_flag(LaunchConfiguration("hardware_enabled").perform(context))
    path = Path(LaunchConfiguration("config_file").perform(context)).resolve(strict=True)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise RuntimeError("invalid PPB-NG configuration schema")
    for name in ("rgb", "thermal", "hsi", "timing"):
        if document.get(name, {}).get("enabled") is not True:
            raise RuntimeError(f"{name}.enabled must be true")

    session = document.get("session", {})
    output_root = str(session.get("output_root", "")).strip()
    if not output_root or not Path(output_root).is_dir():
        raise RuntimeError("session.output_root must be an existing directory")
    maximum_segment_bytes = _positive_integer(
        session.get("maximum_segment_bytes"), "session.maximum_segment_bytes")
    checkpoint_interval_frames = _positive_integer(
        session.get("checkpoint_interval_frames"), "session.checkpoint_interval_frames")
    dark_seconds = float(session.get("hsi_dark_duration_seconds", 0.0))
    if not math.isfinite(dark_seconds) or dark_seconds <= 0:
        raise RuntimeError("session.hsi_dark_duration_seconds must be positive")

    helpers = _helpers()
    helpers._validate_hsi_queue_bounds(document)
    helpers._validate_hsi_continuous_mode(document)
    hsi = document["hsi"]
    timing = document["timing"]
    if timing.get("pps_required") is not False:
        raise RuntimeError("camera-only workflow requires timing.pps_required: false")
    port = str(timing.get("port", "")).strip()
    if not port or "TO_BE_CONFIRMED" in port:
        raise RuntimeError("set timing.port to the connected UNO R4 COM port in ppbng_config.yaml")
    rgb_timing = timing.get("channels", {}).get("rgb", {})
    thermal_timing = timing.get("channels", {}).get("thermal", {})
    for name, channel in (("rgb", rgb_timing), ("thermal", thermal_timing)):
        if channel.get("enabled") is not True:
            raise RuntimeError(f"timing.channels.{name}.enabled must be true")
        if (channel.get("rate_numerator_hz"), channel.get("rate_denominator"),
                channel.get("pulse_width_ticks"), channel.get("phase_ticks")) != (2, 1, 1000, 0):
            raise RuntimeError(
                f"timing.channels.{name} must match validated UNO firmware: 2 Hz, 1000 ticks, phase 0")

    hsi_remappings = [
        ("trigger", "/acquisition/trigger"),
        ("sample_stamp", "/acquisition/sample_stamp"),
        ("status", "/acquisition/device_status"),
        ("fault_event", "/acquisition/fault_event"),
    ]
    fx_params = helpers._hsi_parameters(hsi, session, "fx10e", output_root, dark_seconds)
    swir_params = helpers._hsi_parameters(hsi, session, "swir", output_root, dark_seconds)
    sdk_bin = str(Path(hsi["specsensor_sdk_root"]) / "bin" / "x64")
    hsi_environment = {"PATH": sdk_bin + os.pathsep + os.environ.get("PATH", "")}
    spinnaker_environment = {
        "PATH": "C:/Program Files/Teledyne/Spinnaker/bin64/vs2015;" + os.environ.get("PATH", "")
    }

    rgb_params = {
        **document["rgb"].get("acquisition", {}),
        "allow_hardware_access": True,
        "allowed_output_root": output_root,
        "device_id": document["rgb"]["expected_serial"],
        "stream_name": "rgb",
        "trigger_channel": "rgb",
        "trigger_topic": "/acquisition/trigger",
        "maximum_segment_bytes": maximum_segment_bytes,
        "checkpoint_interval_frames": checkpoint_interval_frames,
    }
    thermal_params = {
        **document["thermal"].get("acquisition", {}),
        "allow_hardware_access": True,
        "allowed_output_root": output_root,
        "device_id": document["thermal"]["expected_device_id"],
        "expected_model": document["thermal"].get("expected_model", "A6701"),
        "stream_name": "thermal",
        "trigger_channel": "thermal",
        "trigger_topic": "/acquisition/trigger",
        "maximum_segment_bytes": maximum_segment_bytes,
        "checkpoint_interval_frames": checkpoint_interval_frames,
    }
    timing_params = {
        "hardware_enabled": True,
        "allowed_output_root": output_root,
        "com_path": port,
        "baud_rate": timing.get("baud_rate", 115200),
        "ticks_per_second": timing.get("ticks_per_second", 1000000),
        "rate_numerator_hz": rgb_timing["rate_numerator_hz"],
        "rate_denominator": rgb_timing["rate_denominator"],
        "pulse_width_ticks": rgb_timing["pulse_width_ticks"],
        "device_id": "uno_r4_trigger",
        "flush_every_events": 32,
    }

    return [
        # Pleora and NI runtimes remain in separate processes for fault isolation.
        Node(package="ppbng_hsi", executable="hsi_production_node", namespace="fx10e",
             name="camera", output="screen", parameters=[fx_params],
             remappings=hsi_remappings, additional_env=hsi_environment),
        Node(package="ppbng_hsi", executable="hsi_production_node", namespace="swir",
             name="camera", output="screen", parameters=[swir_params],
             remappings=hsi_remappings, additional_env=hsi_environment),
        Node(package="ppbng_rgb", executable="ppbng_rgb_camera_node", name="rgb",
             output="screen", parameters=[rgb_params], additional_env=spinnaker_environment,
             remappings=[("/rgb/device_status", "/acquisition/device_status"),
                         ("/rgb/fault_event", "/acquisition/fault_event")]),
        Node(package="ppbng_thermal", executable="ppbng_thermal_camera_node", name="thermal",
             output="screen", parameters=[thermal_params], additional_env=spinnaker_environment,
             remappings=[("/thermal/device_status", "/acquisition/device_status"),
                         ("/thermal/fault_event", "/acquisition/fault_event")]),
        Node(package="ppbng_timing", executable="uno_r4_ascii_trigger_node",
             namespace="timing", name="controller", output="screen", parameters=[timing_params],
             remappings=[("trigger_event", "/acquisition/trigger"),
                         ("device_status", "/acquisition/device_status"),
                         ("fault_event", "/acquisition/fault_event")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=_default_config_path()),
        OpaqueFunction(function=_load),
    ])
