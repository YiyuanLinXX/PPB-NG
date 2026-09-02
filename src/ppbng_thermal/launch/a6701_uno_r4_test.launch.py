import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("dataset_name"),
        DeclareLaunchArgument("device_id", default_value="00111C0408CD"),
        DeclareLaunchArgument("serial_port", default_value="COM9"),
        DeclareLaunchArgument(
            "output_root",
            default_value=(
                "C:/Users/cairlab/Desktop/Projects/PPB_NG_2026/"
                "ppbng_ros2_ws/hardware_test_data"
            ),
        ),
        DeclareLaunchArgument("duration_sec", default_value="0.0"),
        DeclareLaunchArgument("fault_recovery_test", default_value="false"),
    ]
    node = Node(
        package="ppbng_thermal",
        executable="ppbng_a6701_uno_r4_test_node",
        name="ppbng_a6701_uno_r4_test",
        output="screen",
        emulate_tty=True,
        # The ROS pixi environment also ships a DLL named libiomp5md.dll, but
        # it is ABI-incompatible with Spinnaker 4.4.  Give this camera process
        # the vendor runtime first priority without changing the global PATH.
        additional_env={
            "PATH": (
                "C:/Program Files/Teledyne/Spinnaker/bin64/vs2015;"
                + os.environ.get("PATH", "")
            )
        },
        parameters=[
            {
                "dataset_name": LaunchConfiguration("dataset_name"),
                "device_id": LaunchConfiguration("device_id"),
                "serial_port": LaunchConfiguration("serial_port"),
                "output_root": LaunchConfiguration("output_root"),
                "duration_sec": ParameterValue(
                    LaunchConfiguration("duration_sec"), value_type=float
                ),
                "fault_recovery_test": ParameterValue(
                    LaunchConfiguration("fault_recovery_test"), value_type=bool
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
