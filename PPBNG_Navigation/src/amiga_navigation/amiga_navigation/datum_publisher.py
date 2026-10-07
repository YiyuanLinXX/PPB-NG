#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish a fixed GPS datum derived from the first /gps/fix message."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class DatumPublisherNode(Node):
    def __init__(self):
        super().__init__('datum_publisher')

        # Define QoS profile with TRANSIENT_LOCAL so late subscribers can receive the latest datum
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher to /gps/datum
        self.datum_publisher = self.create_publisher(
            Float64MultiArray,
            '/gps/datum',
            qos_profile
        )

        # Subscriber to /gps/fix
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10  # queue size
        )

        # Timer to publish datum at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 sec = 10 Hz

        # Internal variables
        self.datum_set = False
        self.datum_msg = Float64MultiArray()

        self.get_logger().info("DatumPublisherNode started. Waiting for first /gps/fix...")

    def gps_callback(self, msg):
        """Callback function for /gps/fix subscription"""
        if not self.datum_set:
            # Set datum using first received GPS fix
            self.datum_msg.data = [msg.latitude, msg.longitude, msg.altitude]
            self.datum_set = True
            self.get_logger().info(
                f"Datum set to: lat={msg.latitude:.8f}, lon={msg.longitude:.8f}, alt={msg.altitude:.2f}"
            )
        # If datum already set, do nothing (we do not update it again)

    def timer_callback(self):
        """Timer callback to publish datum at 10 Hz"""
        if self.datum_set:
            self.datum_publisher.publish(self.datum_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DatumPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
