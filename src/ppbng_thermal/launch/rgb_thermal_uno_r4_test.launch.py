import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("dataset_name"),
        DeclareLaunchArgument("thermal_device_id", default_value="00111C0408CD"),
        DeclareLaunchArgument("rgb_device_id", default_value="22209867"),
        DeclareLaunchArgument("serial_port", default_value="COM9"),
        DeclareLaunchArgument(
            "output_root",
            default_value=(
                "C:/Users/cairlab/Desktop/Projects/PPB_NG_2026/"
                "ppbng_ros2_ws/hardware_test_data"
            ),
        ),
        DeclareLaunchArgument("duration_sec", default_value="0.0"),
        DeclareLaunchArgument("rgb_exposure_auto", default_value="Continuous"),
        DeclareLaunchArgument("rgb_gain_auto", default_value="Continuous"),
        DeclareLaunchArgument("rgb_balance_white_auto", default_value="Once"),
        DeclareLaunchArgument("rgb_balance_white_auto_profile", default_value="Outdoor"),
        DeclareLaunchArgument("rgb_auto_exposure_control_priority", default_value="Gain"),
        DeclareLaunchArgument(
            "rgb_auto_exposure_time_upper_limit_us", default_value="5000.0"
        ),
        DeclareLaunchArgument(
            "rgb_auto_exposure_gain_upper_limit_db", default_value="12.0"
        ),
    ]
    node = Node(
        package="ppbng_thermal",
        executable="ppbng_a6701_uno_r4_test_node",
        name="ppbng_rgb_thermal_uno_r4_test",
        output="screen",
        emulate_tty=True,
        additional_env={
            "PATH": (
                "C:/Program Files/Teledyne/Spinnaker/bin64/vs2015;"
                + os.environ.get("PATH", "")
            )
        },
        parameters=[
            {
                "dataset_name": ParameterValue(
                    LaunchConfiguration("dataset_name"), value_type=str
                ),
                "device_id": ParameterValue(
                    LaunchConfiguration("thermal_device_id"), value_type=str
                ),
                "rgb_enabled": True,
                "rgb_device_id": ParameterValue(
                    LaunchConfiguration("rgb_device_id"), value_type=str
                ),
                "serial_port": ParameterValue(
                    LaunchConfiguration("serial_port"), value_type=str
                ),
                "output_root": ParameterValue(
                    LaunchConfiguration("output_root"), value_type=str
                ),
                "duration_sec": ParameterValue(
                    LaunchConfiguration("duration_sec"), value_type=float
                ),
                "rgb_exposure_auto": ParameterValue(
                    LaunchConfiguration("rgb_exposure_auto"), value_type=str
                ),
                "rgb_gain_auto": ParameterValue(
                    LaunchConfiguration("rgb_gain_auto"), value_type=str
                ),
                "rgb_balance_white_auto": ParameterValue(
                    LaunchConfiguration("rgb_balance_white_auto"), value_type=str
                ),
                "rgb_balance_white_auto_profile": ParameterValue(
                    LaunchConfiguration("rgb_balance_white_auto_profile"), value_type=str
                ),
                "rgb_auto_exposure_control_priority": ParameterValue(
                    LaunchConfiguration("rgb_auto_exposure_control_priority"),
                    value_type=str,
                ),
                "rgb_auto_exposure_time_upper_limit_us": ParameterValue(
                    LaunchConfiguration("rgb_auto_exposure_time_upper_limit_us"),
                    value_type=float,
                ),
                "rgb_auto_exposure_gain_upper_limit_db": ParameterValue(
                    LaunchConfiguration("rgb_auto_exposure_gain_upper_limit_db"),
                    value_type=float,
                ),
                "fault_recovery_test": False,
            }
        ],
    )
    return LaunchDescription(arguments + [node])
