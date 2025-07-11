#!/usr/bin/env python3
# coding=utf-8
# ROS2 node: subscribe to /gps/utc, /gps/fix, and /rtk_status, log into CSV

import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import NavSatFix

class GpsLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')

        # Declare log file path
        self.declare_parameter('log_file', '/tmp/gps_log.csv')
        log_file = self.get_parameter('log_file').get_parameter_value().string_value

        # Create log directory if needed
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        # Open CSV and write header
        self.csv_file = open(log_file, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'Satellite UTC',
            'ROS Time Stamp',
            'Latitude',
            'Longitude',
            'Altitude',
            'Fix Quality'
        ])
        self.csv_file.flush()

        # Latest values
        self.latest_utc = ''
        self.latest_quality = None

        # QoS profile to match GPS publisher (transient local)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscriptions
        self.create_subscription(String, '/gps/utc', self.utc_callback, qos)
        self.create_subscription(Int32, '/rtk_status', self.rtk_callback, qos)
        self.create_subscription(NavSatFix, '/gps/fix', self.fix_callback, qos)

    def utc_callback(self, msg: String):
        self.latest_utc = msg.data

    def rtk_callback(self, msg: Int32):
        self.latest_quality = msg.data

    def fix_callback(self, msg: NavSatFix):
        # Format ROS time
        t = msg.header.stamp
        ros_ts = f"{t.sec}.{t.nanosec:09d}"

        # Use quality from /rtk_status if available
        quality = self.latest_quality if self.latest_quality is not None else msg.status.status

        # Write row
        self.writer.writerow([
            self.latest_utc,
            ros_ts,
            f"{msg.latitude:.9f}",
            f"{msg.longitude:.9f}",
            f"{msg.altitude:.2f}",
            quality
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception as e:
            self.get_logger().warn(f"Failed to close CSV file: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
