#!/usr/bin/env python3
"""Launch the twist_mux node to arbitrate multiple cmd_vel sources."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amiga_navigation')
    config_file = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[config_file],
        remappings=[
            # ('cmd_vel_out', 'cmd_vel_out'),
        ]
    )

    return LaunchDescription([twist_mux_node])
