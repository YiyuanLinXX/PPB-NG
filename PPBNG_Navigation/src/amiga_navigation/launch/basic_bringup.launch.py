#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the UM982 navigation stack without starting waypoint following."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amiga_navigation')
    bringup_config_argument = DeclareLaunchArgument(
        'bringup_config',
        default_value=os.path.join(pkg, 'config', 'bringup.yaml'),
        description='Absolute path to the runtime bringup YAML configuration.',
    )
    ntrip_profile_argument = DeclareLaunchArgument(
        'ntrip_profile',
        default_value=os.path.join(pkg, 'config', 'ntrip.yaml'),
        description='Absolute path to the NTRIP profile YAML configuration.',
    )

    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'twist_mux_launch.launch.py')
        )
    )

    datum_publisher = Node(
        package='amiga_navigation',
        executable='datum_publisher',
        name='datum_publisher',
        output='screen'
    )

    um982_driver = Node(
        package='amiga_navigation',
        executable='um982_driver',
        name='um982_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('bringup_config'),
            LaunchConfiguration('ntrip_profile'),
        ],
    )

    rtk_monitor = Node(
        package='amiga_navigation',
        executable='rtk_monitor',
        name='rtk_monitor',
        output='screen'
    )

    thermal_nuc_safety_monitor = Node(
        package='amiga_navigation',
        executable='thermal_nuc_safety_monitor',
        name='thermal_nuc_safety_monitor',
        output='screen',
        # Restart into the fail-safe "waiting" state if the monitor crashes.
        # The delay remains below the existing 1.0 s cmd_vel_stop mux timeout.
        respawn=True,
        respawn_delay=0.2,
    )

    amiga_serial_bridge = Node(
        package='amiga_navigation',
        executable='amiga_serial_bridge',
        name='amiga_serial_bridge',
        output='screen',
        parameters=[LaunchConfiguration('bringup_config')],
    )

    return LaunchDescription([
        UnsetEnvironmentVariable('ROS_LOCALHOST_ONLY'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '47'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_STATIC_PEERS', '192.168.5.150'),
        bringup_config_argument,
        ntrip_profile_argument,
        twist_mux,
        datum_publisher,
        um982_driver,
        rtk_monitor,
        thermal_nuc_safety_monitor,
        amiga_serial_bridge,
    ])
