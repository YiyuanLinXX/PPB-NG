from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_root = LaunchConfiguration("output_root")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_root",
                default_value="",
                description="Existing directory for simulated datasets; empty refuses Start",
            ),
            Node(
                package="ppbng_runtime",
                executable="acquisition_manager_node",
                name="acquisition_manager",
                output="screen",
                parameters=[{"simulation_mode": True, "output_root": output_root}],
            ),
        ]
    )

