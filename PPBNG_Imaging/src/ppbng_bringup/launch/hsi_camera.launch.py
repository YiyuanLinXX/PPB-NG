from pathlib import Path
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(text, name):
    value = text.strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _default_config_path():
    """Prefer the editable source YAML in this workspace, then installed fallback."""
    share = Path(get_package_share_directory("ppbng_bringup"))
    workspace_source = share.parents[3] / "src" / "ppbng_bringup" / "config" / \
        "ppbng_config.yaml"
    return str(workspace_source if workspace_source.is_file() else
               share / "config" / "ppbng_config.yaml")


def _load(context):
    camera = LaunchConfiguration("camera").perform(context).strip().lower()
    if camera not in ("fx10e", "swir"):
        raise RuntimeError("camera must be exactly fx10e or swir")
    enabled = _bool(LaunchConfiguration("hardware_enabled").perform(context),
                    "hardware_enabled")
    if not enabled:
        raise RuntimeError("pass hardware_enabled:=true explicitly to open HSI hardware")
    path = Path(LaunchConfiguration("config_file").perform(context)).resolve(strict=True)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise RuntimeError("invalid PPB-NG configuration schema")
    hsi = document.get("hsi", {})
    values = hsi.get(camera, {})
    if values.get("enabled") is not True:
        raise RuntimeError(f"hsi.{camera}.enabled must be true")
    session = document.get("session", {})
    if not isinstance(session.get("hsi_payload_crc_enabled", True), bool):
        raise RuntimeError("session.hsi_payload_crc_enabled must be a boolean")
    dark_seconds = float(session.get("hsi_dark_duration_seconds", 0.0))
    rate = float(values.get("line_rate_hz", 0.0))
    if not math.isfinite(dark_seconds) or dark_seconds <= 0.0 or not math.isfinite(rate) or rate <= 0.0:
        raise RuntimeError("HSI dark duration and line rate must be positive")
    dark_lines = int(math.ceil(dark_seconds * rate))
    spatial = int(values["spatial_samples"])
    bands = int(values["spectral_bands"])
    sdk_root = hsi.get("specsensor_sdk_root", "")
    sdk_bin = str(Path(sdk_root) / "bin" / "x64")
    parameters = {
        "hardware_enabled": True,
        "camera_kind": camera,
        "device_index": int(values["device_index"]),
        "license_path": hsi.get("license_path", ""),
        "expected_profile_name": values["expected_profile_name"],
        "expected_sensor_serial": str(values["expected_identity"]),
        "calibration_pack_path": values["calibration_pack_path"],
        "grabber_channel": values.get("grabber_channel", ""),
        "ni_grabber_channel": values.get("ni_grabber_channel", ""),
        "ni_camera_file_path": values.get("ni_camera_file_path", ""),
        "ni_camera_serial_port": values.get("ni_camera_serial_port", ""),
        "pleora_packet_size": int(values.get("pleora_packet_size", 0)),
        "initialization_timeout_ms": int(hsi.get("initialization_timeout_ms", 5000)),
        "callback_queue_capacity": int(values.get("callback_queue_capacity", 128)),
        "maximum_frame_bytes": spatial * bands * 2,
        "device_id": f"{camera}_{values['expected_identity']}",
        "trigger_channel": f"hsi_{camera}",
        "trigger_mode": values.get("trigger_mode", "Internal"),
        "spatial_samples": spatial,
        "spectral_bands": bands,
        "line_rate_hz": rate,
        "exposure_us": float(values["exposure_us"]),
        "timing_policy": values.get("timing_policy", "strict"),
        "spatial_binning": values.get("spatial_binning", 0),
        "spectral_binning": values.get("spectral_binning", 0),
        "dark_line_count": dark_lines,
        "allowed_output_root": session.get("output_root", ""),
        "maximum_segment_bytes": int(session.get("maximum_segment_bytes", 8589934592)),
        "flush_every_lines": int(session.get("hsi_flush_every_lines", 120)),
        "payload_crc_enabled": session.get("hsi_payload_crc_enabled", True),
        "pending_trigger_capacity": int(values.get("pending_trigger_capacity", 128)),
        "trigger_match_timeout_ms": int(values.get("trigger_match_timeout_ms", 500)),
        "association_evidence_confirmed": bool(
            values.get("association_evidence_confirmed", False)),
        "fail_fast_on_sample_fault": bool(
            values.get("fail_fast_on_sample_fault", True)),
        **values.get("recovery", {}),
    }
    return [Node(
        package="ppbng_hsi",
        executable="hsi_production_node",
        namespace=f"hsi_{camera}",
        name=camera,
        output="screen",
        parameters=[parameters],
        additional_env={"PATH": sdk_bin + os.pathsep + os.environ.get("PATH", "")},
    )]


def generate_launch_description():
    default_config = _default_config_path()
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="fx10e"),
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        OpaqueFunction(function=_load),
    ])
