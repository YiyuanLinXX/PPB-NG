#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Serial bridge between ROS 2 `/cmd_vel_out` and the Amiga MCU."""

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

from amiga_navigation.utils.navigation_safety import validate_velocity_command


class AmigaCom(Node):
    def __init__(self, port: str | None = None, baud: int | None = None):
        super().__init__('amiga_serial_bridge')
        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_F6FF0DE648364C53202020542C1B0DFF-if00',
        )
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout_sec', 0.1)
        self.declare_parameter('serial_write_timeout_sec', 0.2)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('watchdog_timeout_sec', 0.6)
        self.declare_parameter('watchdog_period_sec', 0.1)

        port = port or str(self.get_parameter('serial_port').value)
        baud = int(baud if baud is not None else self.get_parameter('baudrate').value)
        timeout = float(self.get_parameter('serial_timeout_sec').value)
        write_timeout = float(self.get_parameter('serial_write_timeout_sec').value)
        watchdog_period = float(self.get_parameter('watchdog_period_sec').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.get_logger().info(
            f'Starting AmigaCom node on port {port} @ {baud}bps '
            f'(watchdog={self.watchdog_timeout:.2f}s)'
        )

        # Single serial instance for read & write
        self.ser = serial.Serial(
            port,
            baud,
            timeout=timeout,
            write_timeout=write_timeout,
        )

        # ROS interfaces
        self.cmd_sub  = self.create_subscription(
            Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)

        self.last_cmd_time = time.monotonic()
        self.watchdog_triggered = False
        self.last_serial_warning_time = 0.0
        # Timer at 10 Hz to enforce watchdog and drain MCU feedback
        self.create_timer(max(watchdog_period, 0.01), self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        """
        Receive Twist on /cmd_vel_out and forward to MCU.
        Format: "v,omega\n"
        """
        linear_velocity = float(msg.linear.x)
        angular_velocity = float(msg.angular.z)
        rejection = validate_velocity_command(
            linear_velocity,
            angular_velocity,
            self.max_linear_speed,
            self.max_angular_speed,
        )
        if rejection is not None:
            self.get_logger().error(f'Rejected unsafe cmd_vel: {rejection}; sending STOP.')
            self._send_stop()
            self.watchdog_triggered = True
            return

        if self._send_command(linear_velocity, angular_velocity):
            self.last_cmd_time = time.monotonic()
            self.watchdog_triggered = False

    def timer_callback(self):
        """Enforce watchdog and discard any pending MCU feedback."""

        dt_cmd = time.monotonic() - self.last_cmd_time
        if dt_cmd > self.watchdog_timeout:
            if not self.watchdog_triggered:
                self.get_logger().warn(
                    f"Watchdog triggered: {dt_cmd:.2f}s since last cmd_vel; "
                    "continuously sending STOP."
                )
            self.watchdog_triggered = True
            # Repeat STOP instead of trusting a single serial packet.
            self._send_stop()

        # Drain any feedback lines so the serial buffer does not grow unbounded.
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.ser.read(waiting)
        except Exception as e:
            self._warn_serial_limited(f"Serial read failed: {e}")

    def _send_command(self, linear_velocity, angular_velocity):
        try:
            command = f'{linear_velocity:.6f},{angular_velocity:.6f}\n'.encode('ascii')
            self.ser.write(command)
            self.get_logger().debug(f'Sent cmd_vel to MCU: {command.decode().strip()}')
            return True
        except Exception as exc:
            self._warn_serial_limited(f'Failed to send cmd_vel: {exc}')
            return False

    def _send_stop(self):
        return self._send_command(0.0, 0.0)

    def _warn_serial_limited(self, message):
        now = time.monotonic()
        if now - self.last_serial_warning_time >= 1.0:
            self.get_logger().warning(message)
            self.last_serial_warning_time = now
                
    def destroy_node(self):
        try:
            self.get_logger().info("Shutting down AmigaCom: sending final stop command")
            self._send_stop()
        except Exception as e:
            self.get_logger().warning(f"Failed to send final stop command: {e}")
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',      type=str,
                        default=None,
                        help='Serial port for MCU communication')
    parser.add_argument('--baudrate',  type=int, default=None,
                        help='Baudrate for the serial port')
    parsed, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = AmigaCom(port=parsed.port, baud=parsed.baudrate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
