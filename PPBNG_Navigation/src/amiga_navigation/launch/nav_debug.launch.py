#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the base navigation stack plus one combined debug CSV logger.

This debug launch intentionally does not start ``waypoint_follower``.
Run the follower separately in another terminal so the waypoint file and
controller can still be chosen interactively.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amiga_navigation')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'basic_bringup.launch.py')
        )
    )

    nav_topic_debug_logger = Node(
        package='amiga_navigation',
        executable='nav_topic_debug_logger',
        name='nav_topic_debug_logger',
        output='screen'
    )

    return LaunchDescription([
        base_bringup,
        nav_topic_debug_logger,
    ])
