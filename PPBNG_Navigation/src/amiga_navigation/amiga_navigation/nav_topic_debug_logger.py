#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Write a single timestamp-aligned CSV snapshot of navigation debug topics."""

import csv
import datetime
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64MultiArray
import tf_transformations


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_deg_from_quaternion(q) -> float:
    _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return math.degrees(yaw)


class NavTopicDebugLogger(Node):
    def __init__(self):
        super().__init__('nav_topic_debug_logger')
        self.declare_parameter(
            'log_directory', str(Path.home() / 'robot_nav_debug_log')
        )
        self.declare_parameter('log_frequency', 10.0)
        self.declare_parameter('flush_every_row', True)

        self.log_directory = Path(
            str(self.get_parameter('log_directory').value)
        ).expanduser()
        self.log_frequency = float(self.get_parameter('log_frequency').value)
        self.flush_every_row = bool(self.get_parameter('flush_every_row').value)
        self.log_directory.mkdir(parents=True, exist_ok=True)

        now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = self.log_directory / f'nav_topic_debug_log_{now_str}.csv'
        self.csv_file = self.log_path.open(mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self._header())

        self.latest = {}

        datum_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/robot/odom', self.robot_odom_callback, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_fix_callback, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/gps/datum', self.gps_datum_callback, datum_qos)
        self.create_subscription(Bool, '/gps/rtk_status_flag', self.rtk_status_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_out_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_stop', self.cmd_vel_stop_callback, 10)

        self.create_timer(1.0 / max(self.log_frequency, 1.0), self.write_snapshot)
        self.get_logger().info(f'Navigation topic debug logging to: {self.log_path}')

    def _header(self):
        return [
            'timestamp',
            'robot_odom_age_sec',
            'robot_odom_msg_stamp',
            'robot_odom_x',
            'robot_odom_y',
            'robot_odom_z',
            'robot_odom_yaw_deg',
            'gps_fix_age_sec',
            'gps_fix_msg_stamp',
            'gps_lat',
            'gps_lon',
            'gps_alt',
            'gps_status',
            'gps_datum_age_sec',
            'datum_lat',
            'datum_lon',
            'datum_alt',
            'rtk_status_age_sec',
            'rtk_status_not_fix',
            'cmd_vel_nav_age_sec',
            'cmd_vel_nav_linear_x',
            'cmd_vel_nav_angular_z',
            'cmd_vel_out_age_sec',
            'cmd_vel_out_linear_x',
            'cmd_vel_out_angular_z',
            'cmd_vel_stop_age_sec',
            'cmd_vel_stop_linear_x',
            'cmd_vel_stop_angular_z',
        ]

    def _now_sec(self) -> float:
        now = self.get_clock().now().seconds_nanoseconds()
        return float(now[0]) + float(now[1]) * 1e-9

    def _age(self, topic: str, now_sec: float):
        if topic not in self.latest:
            return ''
        return now_sec - self.latest[topic]['received_sec']

    def _value(self, topic: str, key: str):
        return self.latest.get(topic, {}).get(key, '')

    def robot_odom_callback(self, msg):
        pose = msg.pose.pose
        self.latest['robot_odom'] = {
            'received_sec': self._now_sec(),
            'msg_stamp': stamp_to_sec(msg.header.stamp),
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'yaw_deg': yaw_deg_from_quaternion(pose.orientation),
        }

    def gps_fix_callback(self, msg):
        self.latest['gps_fix'] = {
            'received_sec': self._now_sec(),
            'msg_stamp': stamp_to_sec(msg.header.stamp),
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status,
        }

    def gps_datum_callback(self, msg):
        data = list(msg.data)
        self.latest['gps_datum'] = {
            'received_sec': self._now_sec(),
            'lat': data[0] if len(data) > 0 else '',
            'lon': data[1] if len(data) > 1 else '',
            'alt': data[2] if len(data) > 2 else '',
        }

    def rtk_status_callback(self, msg):
        self.latest['rtk_status'] = {
            'received_sec': self._now_sec(),
            'not_fix': msg.data,
        }

    def _record_twist(self, topic: str, msg: Twist):
        self.latest[topic] = {
            'received_sec': self._now_sec(),
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
        }

    def cmd_vel_nav_callback(self, msg):
        self._record_twist('cmd_vel_nav', msg)

    def cmd_vel_out_callback(self, msg):
        self._record_twist('cmd_vel_out', msg)

    def cmd_vel_stop_callback(self, msg):
        self._record_twist('cmd_vel_stop', msg)

    def write_snapshot(self):
        if not self.latest:
            return

        now_sec = self._now_sec()
        row = [
            now_sec,
            self._age('robot_odom', now_sec),
            self._value('robot_odom', 'msg_stamp'),
            self._value('robot_odom', 'x'),
            self._value('robot_odom', 'y'),
            self._value('robot_odom', 'z'),
            self._value('robot_odom', 'yaw_deg'),
            self._age('gps_fix', now_sec),
            self._value('gps_fix', 'msg_stamp'),
            self._value('gps_fix', 'lat'),
            self._value('gps_fix', 'lon'),
            self._value('gps_fix', 'alt'),
            self._value('gps_fix', 'status'),
            self._age('gps_datum', now_sec),
            self._value('gps_datum', 'lat'),
            self._value('gps_datum', 'lon'),
            self._value('gps_datum', 'alt'),
            self._age('rtk_status', now_sec),
            self._value('rtk_status', 'not_fix'),
            self._age('cmd_vel_nav', now_sec),
            self._value('cmd_vel_nav', 'linear_x'),
            self._value('cmd_vel_nav', 'angular_z'),
            self._age('cmd_vel_out', now_sec),
            self._value('cmd_vel_out', 'linear_x'),
            self._value('cmd_vel_out', 'angular_z'),
            self._age('cmd_vel_stop', now_sec),
            self._value('cmd_vel_stop', 'linear_x'),
            self._value('cmd_vel_stop', 'angular_z'),
        ]
        self.csv_writer.writerow(row)
        if self.flush_every_row:
            self.csv_file.flush()

    def destroy_node(self):
        self.get_logger().info('Shutting down navigation topic debug logger')
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavTopicDebugLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
