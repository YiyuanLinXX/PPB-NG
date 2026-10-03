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


def _production_helpers():
    path = Path(__file__).with_name("production.launch.py")
    spec = spec_from_file_location("ppbng_production_launch_hsi_helpers", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _default_config_path():
    share = Path(get_package_share_directory("ppbng_bringup"))
    workspace_source = share.parents[3] / "src" / "ppbng_bringup" / "config" / \
        "ppbng_config.yaml"
    return str(workspace_source if workspace_source.is_file() else
               share / "config" / "ppbng_config.yaml")


def _enabled(text):
    if text.strip().lower() not in ("true", "1", "yes", "on"):
        raise RuntimeError(
            "pass hardware_enabled:=true explicitly to open both HSI cameras")


def _load(context):
    _enabled(LaunchConfiguration("hardware_enabled").perform(context))
    path = Path(LaunchConfiguration("config_file").perform(context)).resolve(strict=True)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise RuntimeError("invalid PPB-NG configuration schema")
    hsi = document.get("hsi", {})
    if any(hsi.get(camera, {}).get("enabled") is not True
           for camera in ("fx10e", "swir")):
        raise RuntimeError("hsi.fx10e.enabled and hsi.swir.enabled must both be true")
    session = document.get("session", {})
    dark_seconds = float(session.get("hsi_dark_duration_seconds", 0.0))
    if not math.isfinite(dark_seconds) or dark_seconds <= 0.0:
        raise RuntimeError("session.hsi_dark_duration_seconds must be positive")

    helpers = _production_helpers()
    helpers._validate_hsi_queue_bounds(document)
    helpers._validate_hsi_continuous_mode(document)
    output_root = session.get("output_root", "")
    remappings = [
        ("trigger", "/acquisition/trigger"),
        ("sample_stamp", "/acquisition/sample_stamp"),
        ("status", "/acquisition/device_status"),
        ("fault_event", "/acquisition/fault_event"),
    ]
    fx10e_parameters = helpers._hsi_parameters(
        hsi, session, "fx10e", output_root, dark_seconds)
    swir_parameters = helpers._hsi_parameters(
        hsi, session, "swir", output_root, dark_seconds)
    sdk_bin = str(Path(hsi["specsensor_sdk_root"]) / "bin" / "x64")
    environment = {"PATH": sdk_bin + os.pathsep + os.environ.get("PATH", "")}
    # The Pleora runtime used by FX10e is not safe when the NI-backed SWIR
    # adapter is active in the same process.  Separate processes isolate their
    # callback/runtime lifetimes; hsi_production_node serializes the complete
    # SpecSensor initialization transaction with a Windows named mutex.
    return [
        Node(
            package="ppbng_hsi", executable="hsi_production_node",
            namespace="fx10e", name="camera", output="screen",
            parameters=[fx10e_parameters], remappings=remappings,
            additional_env=environment,
        ),
        Node(
            package="ppbng_hsi", executable="hsi_production_node",
            namespace="swir", name="camera", output="screen",
            parameters=[swir_parameters], remappings=remappings,
            additional_env=environment,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=_default_config_path()),
        OpaqueFunction(function=_load),
    ])
