#!/usr/bin/env python3
# coding=utf-8
# arduino_com_node.py
# ROS2 node: waits for /thermal_camera_ready and /rgb_camera_ready,
# then sends 's' x10 to Arduino to start trigger; sends 'e' x10 to stop on shutdown

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import serial
import time
import signal
import sys

class ArduinoComNode(Node):
    def __init__(self):
        super().__init__('arduino_com_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Initialize flags
        self.rgb_ready = False
        self.thermal_ready = False
        self.trigger_sent = False

        # Open serial connection to Arduino
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open Arduino serial port: {e}')
            rclpy.shutdown()
            return

        # Subscriptions with transient_local durability
        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_transient.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(Bool, '/thermal_camera_ready', self.thermal_callback, qos_transient)
        self.create_subscription(Bool, '/rgb_camera_ready', self.rgb_callback, qos_transient)

        self.get_logger().info('Waiting for both cameras to be ready...')

    def thermal_callback(self, msg: Bool):
        if msg.data and not self.thermal_ready:
            self.thermal_ready = True
            self.get_logger().info('[OK] Thermal camera ready')
            self.check_and_trigger()

    def rgb_callback(self, msg: Bool):
        if msg.data and not self.rgb_ready:
            self.rgb_ready = True
            self.get_logger().info('[OK] RGB camera ready')
            self.check_and_trigger()

    def check_and_trigger(self):
        if self.rgb_ready and self.thermal_ready and not self.trigger_sent:
            self.get_logger().info('Both cameras ready. Sending trigger start signal to Arduino...')
            try:
                for _ in range(10):
                    self.ser.write(b's')
                    time.sleep(0.05)
                self.trigger_sent = True
                self.get_logger().info('[OK] Trigger signal sent to Arduino.')
            except Exception as e:
                self.get_logger().error(f'Failed to send trigger to Arduino: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down: sending stop signal to Arduino...')
        try:
            for _ in range(10):
                self.ser.write(b'e')
                self.get_logger().info('sending [e]')
                time.sleep(0.05)
            self.ser.close()
            self.get_logger().info('[OK] Stop signal sent to Arduino.')
        except Exception as e:
            self.get_logger().error(f'Failed to send stop signal: {e}')
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoComNode()

    def shutdown_handler(signum, frame):
        node.get_logger().info('Received shutdown signal, sending stop signal to Arduino...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
