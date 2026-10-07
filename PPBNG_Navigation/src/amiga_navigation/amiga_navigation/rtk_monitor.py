#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Monitor RTK status and publish `/cmd_vel_stop` whenever RTK is unavailable."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class RTKStatusMonitor(Node):
    def __init__(self):
        super().__init__('rtk_monitor')
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('status_timeout_sec', 1.0)

        self.sub = self.create_subscription(
            Bool, '/gps/rtk_status_flag', self.status_callback, 10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.publish_period_sec = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        self.status_timeout_sec = self.get_parameter('status_timeout_sec').get_parameter_value().double_value
        self.create_timer(self.publish_period_sec, self.timer_callback)

        self.active = False
        self.last_status_ok = False
        self.last_status_time = None
        self.active_reason = None
        self.get_logger().info("RTK Monitor Node started. Waiting for RTK status updates...")

    def status_callback(self, msg: Bool):
        self.last_status_ok = not msg.data
        self.last_status_time = self.get_clock().now()

    def timer_callback(self):
        reason = self.get_stop_reason()
        if reason is None:
            if self.active:
                self.get_logger().info("RTK FIX restored — stop publishing /cmd_vel_stop")
            self.active = False
            self.active_reason = None
            return

        self.publish_stop()
        if not self.active or reason != self.active_reason:
            if reason == 'waiting':
                self.get_logger().warn("No RTK status received yet — publishing /cmd_vel_stop")
            elif reason == 'stale':
                self.get_logger().warn("RTK status timed out — publishing /cmd_vel_stop")
            else:
                self.get_logger().warn("RTK not FIX — publishing /cmd_vel_stop")
        self.active = True
        self.active_reason = reason

    def get_stop_reason(self):
        if self.last_status_time is None:
            return 'waiting'

        age_sec = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
        if age_sec > self.status_timeout_sec:
            return 'stale'

        if not self.last_status_ok:
            return 'not_fix'

        return None

    def publish_stop(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RTKStatusMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
