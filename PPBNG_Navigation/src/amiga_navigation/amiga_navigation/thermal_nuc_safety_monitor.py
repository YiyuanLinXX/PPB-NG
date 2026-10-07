#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-safe motion interlock driven by the Windows thermal NUC status."""

import time
from typing import Callable, Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class ThermalMotionPermissionWatchdog:
    """Track permission freshness using only this machine's monotonic clock."""

    def __init__(
        self,
        timeout_sec: float,
        monotonic_now: Callable[[], float] = time.monotonic,
    ) -> None:
        if timeout_sec <= 0.0:
            raise ValueError('permission_timeout_sec must be greater than zero')
        self.timeout_sec = timeout_sec
        self._monotonic_now = monotonic_now
        self._last_permission: Optional[bool] = None
        self._last_receipt_time: Optional[float] = None

    def update(self, permitted: bool) -> None:
        self._last_permission = bool(permitted)
        self._last_receipt_time = self._monotonic_now()

    def stop_reason(self) -> Optional[str]:
        if self._last_receipt_time is None:
            return 'waiting'

        age_sec = self._monotonic_now() - self._last_receipt_time
        if age_sec > self.timeout_sec:
            return 'stale'

        if not self._last_permission:
            return 'denied'

        return None


def make_zero_twist() -> Twist:
    """Construct the monitor's only permitted output command."""
    return Twist()


class ThermalNucSafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__('thermal_nuc_safety_monitor')

        self.declare_parameter(
            'permission_topic', '/ppbng/safety/thermal_motion_permitted'
        )
        self.declare_parameter('stop_topic', '/cmd_vel_stop')
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('permission_timeout_sec', 1.0)

        permission_topic = self.get_parameter('permission_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        permission_timeout_sec = float(
            self.get_parameter('permission_timeout_sec').value
        )

        if publish_period_sec <= 0.0:
            raise ValueError('publish_period_sec must be greater than zero')

        permission_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._watchdog = ThermalMotionPermissionWatchdog(permission_timeout_sec)
        self._subscription = self.create_subscription(
            Bool,
            permission_topic,
            self._permission_callback,
            permission_qos,
        )
        self._stop_publisher = self.create_publisher(Twist, stop_topic, 10)
        self._timer = self.create_timer(publish_period_sec, self._timer_callback)
        self._last_reported_reason: object = object()

        self.get_logger().info(
            'Thermal NUC safety monitor started; motion is blocked until a fresh '
            'permission is received.'
        )

    def _permission_callback(self, msg: Bool) -> None:
        self._watchdog.update(msg.data)

    def _timer_callback(self) -> None:
        reason = self._watchdog.stop_reason()
        if reason is not None:
            self._stop_publisher.publish(make_zero_twist())

        if reason == self._last_reported_reason:
            return

        if reason is None:
            self.get_logger().info(
                'Fresh thermal NUC permission received; releasing /cmd_vel_stop.'
            )
        elif reason == 'waiting':
            self.get_logger().warning(
                'No thermal NUC permission received; publishing /cmd_vel_stop.'
            )
        elif reason == 'stale':
            self.get_logger().warning(
                'Thermal NUC permission timed out; publishing /cmd_vel_stop.'
            )
        else:
            self.get_logger().warning(
                'Thermal NUC motion permission denied; publishing /cmd_vel_stop.'
            )
        self._last_reported_reason = reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalNucSafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
