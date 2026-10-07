#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Log `/robot/odom` only to a timestamped CSV file."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
import tf_transformations
import datetime
from pathlib import Path

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw in degrees."""
    (_, _, yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
    return math.degrees(yaw)

class OdometryLoggerNode(Node):
    def __init__(self):
        super().__init__('robot_odom_logger')
        self.get_logger().info("Starting robot_odom logger")
        self.declare_parameter(
            'log_directory', str(Path.home() / 'robot_nav_debug_log')
        )
        self.log_directory = Path(
            str(self.get_parameter('log_directory').value)
        ).expanduser()
        self.log_directory.mkdir(parents=True, exist_ok=True)

        # Generate a timestamped filename
        now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_directory / f'robot_odom_log_{now_str}.csv'
        self.get_logger().info(f"Logging to file: {self.log_path}")

        # Open CSV file for writing
        self.csv_file = self.log_path.open(mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z', 'yaw_deg'])

        # Subscribe to /robot/odom
        self.create_subscription(Odometry, '/robot/odom', self.robot_odom_callback, 10)

    def log_odometry(self, x, y, z, yaw_deg):
        # Get current time in float seconds
        now = self.get_clock().now()
        timestamp = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        # Write a row to the CSV file
        self.csv_writer.writerow([timestamp, x, y, z, yaw_deg])
        self.csv_file.flush()  # Ensure data is saved immediately

    def robot_odom_callback(self, msg):
        """Callback for /robot/odom messages"""
        pose = msg.pose.pose
        yaw_deg = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.log_odometry(pose.position.x, pose.position.y, pose.position.z, yaw_deg)

    def destroy_node(self):
        self.get_logger().info("Shutting down robot_odom logger")
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
