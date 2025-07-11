#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# gps_publisher_node.py
# ROS2 node: read NMEA GPGGA/GNGGA from serial and publish:
#   - sensor_msgs/NavSatFix on /gps/fix (including raw fix quality)
#   - std_msgs/String on /gps/utc for raw satellite UTC time hh:mm:ss.ss
#   - std_msgs/Int32 on /rtk_status for fix quality value
# =============================================================================

import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String, Int32

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Try to open serial port
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'[OK] Opened GPS serial on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to open GPS serial: {e}')
            rclpy.shutdown()
            return

        # QoS with TRANSIENT_LOCAL for late-joiner compatibility
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', qos)
        self.pub_utc = self.create_publisher(String, '/gps/utc', qos)
        self.pub_rtk = self.create_publisher(Int32, '/rtk_status', qos)

        # Start background reading thread
        thread = threading.Thread(target=self.read_loop, daemon=True)
        thread.start()

    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception as e:
                self.get_logger().warn(f'[WARN] Serial read failed: {e}')
                continue

            if not (line.startswith('$GPGGA') or line.startswith('$GNGGA')):
                continue

            parts = line.split(',')
            if len(parts) < 7:
                continue

            # UTC time string
            raw_utc = parts[1]
            utc_str = ''
            if len(raw_utc) >= 6:
                hh = raw_utc[0:2]
                mm = raw_utc[2:4]
                ss = raw_utc[4:]
                utc_str = f"{hh}:{mm}:{ss}"
                msg_utc = String()
                msg_utc.data = utc_str
                self.pub_utc.publish(msg_utc)

            # Latitude and longitude
            try:
                raw_lat = parts[2]
                lat_deg = float(raw_lat[:2])
                lat_min = float(raw_lat[2:])
                lat = lat_deg + lat_min / 60.0
                if parts[3] == 'S':
                    lat = -lat

                raw_lon = parts[4]
                lon_deg = float(raw_lon[:3])
                lon_min = float(raw_lon[3:])
                lon = lon_deg + lon_min / 60.0
                if parts[5] == 'W':
                    lon = -lon
            except Exception:
                continue

            # Fix quality
            try:
                quality = int(parts[6])
            except ValueError:
                quality = 0

            # Altitude
            alt = 0.0
            if len(parts) > 9 and parts[9]:
                try:
                    alt = float(parts[9])
                except ValueError:
                    pass

            # Publish NavSatFix
            msg_fix = NavSatFix()
            msg_fix.header.stamp = self.get_clock().now().to_msg()
            msg_fix.header.frame_id = 'gps'
            msg_fix.latitude = lat
            msg_fix.longitude = lon
            msg_fix.altitude = alt

            status = NavSatStatus()
            status.service = NavSatStatus.SERVICE_GPS
            status.status = quality
            msg_fix.status = status
            self.pub_fix.publish(msg_fix)

            # Publish RTK quality (e.g. 4 = RTK Fixed)
            msg_rtk = Int32()
            msg_rtk.data = quality
            self.pub_rtk.publish(msg_rtk)

    def destroy_node(self):
        try:
            self.get_logger().info('[INFO] Shutting down GPS serial...')
            self.ser.close()
        except Exception as e:
            self.get_logger().warn(f'[WARN] Failed to close serial: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
