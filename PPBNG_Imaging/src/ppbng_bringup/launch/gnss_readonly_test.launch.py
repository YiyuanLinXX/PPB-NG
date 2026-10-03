from pathlib import Path

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


def _load(context):
    if LaunchConfiguration("hardware_enabled").perform(context).strip().lower() != "true":
        raise RuntimeError("hardware_enabled must be explicitly true")
    path = Path(LaunchConfiguration("config_file").perform(context)).resolve(strict=True)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise RuntimeError("invalid PPB-NG configuration schema")
    values = document.get("gnss", {})
    if values.get("enabled") is not True:
        raise RuntimeError("gnss.enabled must be true")
    if values.get("receive_only") is not True:
        raise RuntimeError("GNSS test refuses any configuration that is not receive-only")
    port = str(values.get("port", ""))
    if not port or "TO_BE_CONFIRMED" in port or "REQUIRED" in port:
        raise RuntimeError("set gnss.port to the UM982 TTL-to-USB COM port first")
    session = document.get("session", {})
    return [Node(
        package="ppbng_gnss",
        executable="um982_production_node",
        namespace="gnss",
        name="receiver",
        output="screen",
        parameters=[{
            "hardware_enabled": True,
            "allowed_output_root": str(session["output_root"]),
            "com_path": port,
            "baud_rate": int(values.get("baud_rate", 115200)),
            "device_id": "gnss",
            "storage_stem": values.get("storage_stem", "um982"),
            "flush_every_sentences": int(values.get("flush_every_sentences", 10)),
            **values.get("recovery", {}),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=_default_config_path()),
        OpaqueFunction(function=_load),
    ])
