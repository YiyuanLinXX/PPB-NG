#!/usr/bin/env python3
# coding=utf-8
# thermal_acquisition_node.py
# ROS2 node: use external GPIO trigger to acquire thermal images from a specific camera,
# save raw mono16 TIFF images including InfoLine, and synchronize with GPS coordinates.

import os
import time
import datetime
import PySpin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from collections import deque
import numpy as np
from PIL import Image

class ThermalAcquisitionNode(Node):
    def __init__(self):
        super().__init__('thermal_acquisition_node')

        # === Parameters ===
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('thermal_device_id', '00111C0408CD')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('gps_qos_depth', 50)
        self.declare_parameter('calib_tag', '17mm, Empty, 10C - 90C')

        p = self.get_parameter
        self.output_dir = p('output_dir').get_parameter_value().string_value
        self.thermal_device_id = p('thermal_device_id').get_parameter_value().string_value
        self.gps_topic = p('gps_topic').get_parameter_value().string_value
        self.calib_tag = p('calib_tag').get_parameter_value().string_value
        qdepth = p('gps_qos_depth').get_parameter_value().integer_value

        os.makedirs(self.output_dir, exist_ok=True)

        # === Setup GPS subscriber ===
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=qdepth)
        self.gps_history = deque(maxlen=qdepth)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_callback, qos)

        # === Camera Init ===
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        if self.cam_list.GetSize() == 0:
            self.get_logger().error('No FLIR thermal cameras detected.')
            rclpy.shutdown()
            return

        self.cam = self._select_camera_by_thermal_device_id(self.thermal_device_id)
        if self.cam is None:
            self.get_logger().error(f"Camera with Device ID '{self.thermal_device_id}' not found.")
            rclpy.shutdown()
            return

        self.cam.Init()
        self.nodemap = self.cam.GetNodeMap()
        self.nodemap_tldevice = self.cam.GetTLDeviceNodeMap()

        self.save_dir = os.path.join(self.output_dir, 'thermal')
        os.makedirs(self.save_dir, exist_ok=True)
        self.csv_path = os.path.join(self.save_dir, 'Timestamp_GPS.csv')
        self.csv_file = open(self.csv_path, 'w')
        self.csv_file.write('Frame ID,Computer Time,ROS Time,Latitude,Longitude,FixQuality\n')
        self.csv_file.flush()

        self._configure_camera()
        self.counter = 1

        # === Notify camera ready ===
        # QoS setting: Transient Local to retain message for late subscribers
        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_transient.reliability = QoSReliabilityPolicy.RELIABLE

        self.ready_pub = self.create_publisher(Bool, '/thermal_camera_ready', qos_transient)
        self.ready_pub.publish(Bool(data=True))

        self.timer = self.create_timer(0.01, self._acquire_frame)

    def _gps_callback(self, msg: NavSatFix):
        t = msg.header.stamp
        ts = t.sec + t.nanosec * 1e-9
        self.gps_history.append((ts, msg.latitude, msg.longitude, msg.status.status))

    def _select_camera_by_thermal_device_id(self, desired_thermal_device_id):
        selected = None
        for cam in self.cam_list:
            try:
                tl = cam.GetTLDeviceNodeMap()
                id_node = PySpin.CStringPtr(tl.GetNode("DeviceID"))
                thermal_device_id = id_node.GetValue() if PySpin.IsReadable(id_node) else ""
                if thermal_device_id == desired_thermal_device_id:
                    cam.Init()
                    selected = cam
                    break  # stop once found
            except Exception as e:
                continue
        return selected

    def _configure_camera(self):
        try:
            # StreamMode
            nodemap_tlstream = self.cam.GetTLStreamNodeMap()
            stream_mode = PySpin.CEnumerationPtr(nodemap_tlstream.GetNode('StreamMode'))
            stream_mode.SetIntValue(stream_mode.GetEntryByName('Socket').GetValue())

            # FrameSyncSource
            fsync = PySpin.CEnumerationPtr(self.nodemap.GetNode('FrameSyncSource'))
            fsync.SetIntValue(fsync.GetEntryByName('External').GetValue())

            # GigE Packet
            packet_size = PySpin.CIntegerPtr(self.nodemap.GetNode('GevSCPSPacketSize'))
            packet_size.SetValue(8192)
            packet_delay = PySpin.CIntegerPtr(self.nodemap.GetNode('GevSCPD'))
            packet_delay.SetValue(1000)

            # Calibration
            tag_node = PySpin.CStringPtr(self.nodemap.GetNode('PS0CalibrationLoadTag'))
            tag_node.SetValue(self.calib_tag)
            load_cmd = PySpin.CCommandPtr(self.nodemap.GetNode('PS0CalibrationLoad'))
            load_cmd.Execute()

            # Save calibration params
            self._save_calibration()

            # Acquisition Mode
            acq_mode = PySpin.CEnumerationPtr(self.nodemap.GetNode('AcquisitionMode'))
            acq_mode.SetIntValue(acq_mode.GetEntryByName('Continuous').GetValue())

            self.cam.BeginAcquisition()
            self.get_logger().info("[OK] Thermal camera configured and ready.")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Failed to configure camera: {e}")
            rclpy.shutdown()

    def _save_calibration(self):
        try:
            coeffs = [PySpin.CFloatPtr(self.nodemap.GetNode(f'CalibrationQueryCoeff{i}')).GetValue() for i in range(3)]
            R = PySpin.CFloatPtr(self.nodemap.GetNode('CalibrationQueryR')).GetValue()
            B = PySpin.CFloatPtr(self.nodemap.GetNode('CalibrationQueryB')).GetValue()
            F = PySpin.CFloatPtr(self.nodemap.GetNode('CalibrationQueryF')).GetValue()
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(os.path.join(self.save_dir, 'calibration_params.txt'), 'w') as f:
                f.write(f"# Timestamp: {ts}\n\n")
                f.write(f"Coeff0 = {coeffs[0]}\nCoeff1 = {coeffs[1]}\nCoeff2 = {coeffs[2]}\n")
                f.write(f"R = {R}\nB = {B}\nF = {F}\n")
        except Exception as e:
            self.get_logger().warn(f"[WARN] Failed to save calibration parameters: {e}")

    def _acquire_frame(self):
        try:
            image_result = self.cam.GetNextImage(5000)
            if image_result.IsIncomplete():
                self.get_logger().warn("[WARN] Incomplete image received. Waiting for next trigger...")
                return

            image_data = image_result.GetNDArray()
            t_ros = self.get_clock().now().to_msg()
            ros_ts = f"{t_ros.sec}.{t_ros.nanosec:09d}"
            ros_float = t_ros.sec + t_ros.nanosec * 1e-9
            t_str = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]
            filename = os.path.join(self.save_dir, f"Frame_{self.counter:06d}.tiff")
            Image.fromarray(image_data.astype(np.uint16)).save(filename)

            lat = lon = fix = ''
            if self.gps_history:
                best = min(self.gps_history, key=lambda x: abs(x[0] - ros_float))
                _, lat, lon, fix = best

            self.csv_file.write(f"{self.counter},{t_str},{ros_ts},{lat},{lon},{fix}\n")
            self.csv_file.flush()
            self.get_logger().info(f"[SAVED] Frame {self.counter:06d} at {t_str}")

            self.counter += 1
            image_result.Release()

        except PySpin.SpinnakerException as ex:
            msg = str(ex)
            if "timeout" in msg.lower() or "-1011" in msg or "eventdata" in msg.lower():
                self.get_logger().debug("[WAIT] No trigger signal received yet.")
            else:
                self.get_logger().warn(f"[ERROR] Unexpected acquisition error: {ex}")

    def destroy_node(self):
        try:
            self.cam.EndAcquisition()
            self.cam.DeInit()
            self.csv_file.close()
            self.cam_list.Clear()
            self.system.ReleaseInstance()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ThermalAcquisitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
