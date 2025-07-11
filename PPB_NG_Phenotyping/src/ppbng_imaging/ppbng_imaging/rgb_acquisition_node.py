#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# acquisition_node.py
# ROS2 node: capture images from a single RGB camera using hardware trigger
# continuously save raw BayerRG8 images as PGM until interrupted,
# record camera chunk timestamps, ROS timestamps, and synchronized GPS coords into per-camera CSVs.
# Configure exposure, white balance, and gain at initialization.
# Only acquire from camera with specific serial number (e.g., 22209867)
# =============================================================================

import os
import time
import datetime
import PySpin
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from collections import deque

class RGBImageAcquisitionNode(Node):
    def __init__(self):
        super().__init__('rgb_acquisition_node')

        # === Parameters ===
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('exposure_time', 400.0)
        self.declare_parameter('gain', 5.0)
        self.declare_parameter('wb_red', 1.34)
        self.declare_parameter('wb_blue', 2.98)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('gps_qos_depth', 50)
        self.declare_parameter('rgb_serial', '22209867')

        p = self.get_parameter
        self.output_dir = p('output_dir').get_parameter_value().string_value
        self.exposure_time = p('exposure_time').get_parameter_value().double_value
        self.gain_value = p('gain').get_parameter_value().double_value
        self.wb_red = p('wb_red').get_parameter_value().double_value
        self.wb_blue = p('wb_blue').get_parameter_value().double_value
        self.gps_topic = p('gps_topic').get_parameter_value().string_value
        self.rgb_serial = p('rgb_serial').get_parameter_value().string_value
        qdepth = p('gps_qos_depth').get_parameter_value().integer_value

        # === Setup GPS ===
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=qdepth)
        self.gps_history = deque(maxlen=qdepth)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_callback, qos)

        os.makedirs(self.output_dir, exist_ok=True)

        # === Camera init (only selected serial) ===
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        self.cam = self._select_camera_by_serial(self.rgb_serial)
        if not self.cam:
            self.get_logger().error(f"Camera with serial {self.rgb_serial} not found.")
            self.cleanup()
            rclpy.shutdown()
            return

        self.get_logger().info(f"Using RGB camera with serial {self.rgb_serial}")
        nodemap = self.cam.GetNodeMap()
        self._configure_camera(nodemap)
        self.cam.BeginAcquisition()

        # === Output folders ===
        cam_dir = os.path.join(self.output_dir, self.rgb_serial)
        os.makedirs(cam_dir, exist_ok=True)
        self.image_dir = cam_dir
        self.csv_path = os.path.join(cam_dir, 'Timestamp_GPS.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_file.write('FrameID,ComputerTime,ROSTime,ChunkFrameID,ChunkTimestamp,Latitude,Longitude,FixQuality\n')
        self.csv_file.flush()
        self.counter = 1

        # === Save metadata ===
        self._save_metadata(os.path.join(cam_dir, 'metadata.txt'))

        # === Publisher: /rgb_camera_ready ===
        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_transient.reliability = ReliabilityPolicy.RELIABLE

        self.ready_pub = self.create_publisher(Bool, '/rgb_camera_ready', qos_profile=qos_transient)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info("Published /rgb_camera_ready = True")

    def _select_camera_by_serial(self, target_serial):
        for cam in self.cam_list:
            try:
                tl = cam.GetTLDeviceNodeMap()
                serial_node = PySpin.CStringPtr(tl.GetNode('DeviceSerialNumber'))
                serial = serial_node.GetValue() if PySpin.IsReadable(serial_node) else ""
                if serial == target_serial:
                    cam.Init()
                    return cam
            except Exception:
                continue
        return None

    def _save_metadata(self, path):
        try:
            nodemap = self.cam.GetNodeMap()
            metadata = {}

            metadata['SerialNumber'] = self.rgb_serial

            # Exposure
            exposure_time_node = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime'))
            metadata['ExposureTime'] = exposure_time_node.GetValue() if PySpin.IsReadable(exposure_time_node) else 'N/A'

            # Gain
            gain_node = PySpin.CFloatPtr(nodemap.GetNode('Gain'))
            metadata['Gain'] = gain_node.GetValue() if PySpin.IsReadable(gain_node) else 'N/A'

            # White Balance
            wb_selector = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceRatioSelector'))
            wb_value = PySpin.CFloatPtr(nodemap.GetNode('BalanceRatio'))

            if PySpin.IsWritable(wb_selector) and PySpin.IsReadable(wb_value):
                wb_selector.SetIntValue(wb_selector.GetEntryByName('Red').GetValue())
                metadata['WhiteBalanceRed'] = wb_value.GetValue()
                wb_selector.SetIntValue(wb_selector.GetEntryByName('Blue').GetValue())
                metadata['WhiteBalanceBlue'] = wb_value.GetValue()
            else:
                metadata['WhiteBalanceRed'] = 'N/A'
                metadata['WhiteBalanceBlue'] = 'N/A'

            # Pixel Format
            pix_fmt = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            metadata['PixelFormat'] = pix_fmt.ToString() if PySpin.IsReadable(pix_fmt) else 'N/A'

            # Trigger settings
            trig_source = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
            metadata['TriggerSource'] = trig_source.ToString() if PySpin.IsReadable(trig_source) else 'N/A'

            trig_activation = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerActivation'))
            metadata['TriggerActivation'] = trig_activation.ToString() if PySpin.IsReadable(trig_activation) else 'N/A'

            with open(path, 'w') as f:
                f.write("# Camera Metadata (read from actual configuration)\n")
                for k, v in metadata.items():
                    f.write(f"{k} = {v}\n")

        except Exception as e:
            self.get_logger().warn(f"[WARN] Failed to save metadata: {e}")


    def _configure_camera(self, nodemap):
        self.get_logger().info("Configuring camera settings...")

        PySpin.CEnumerationPtr(nodemap.GetNode('ExposureAuto')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('ExposureAuto')).GetEntryByName('Off').GetValue())
        PySpin.CEnumerationPtr(nodemap.GetNode('ExposureMode')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('ExposureMode')).GetEntryByName('Timed').GetValue())
        PySpin.CFloatPtr(nodemap.GetNode('ExposureTime')).SetValue(self.exposure_time)

        PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto')).GetEntryByName('Off').GetValue())
        PySpin.CFloatPtr(nodemap.GetNode('Gain')).SetValue(self.gain_value)

        wb_selector = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceRatioSelector'))
        wb_value = PySpin.CFloatPtr(nodemap.GetNode('BalanceRatio'))
        PySpin.CEnumerationPtr(nodemap.GetNode('BalanceWhiteAuto')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('BalanceWhiteAuto')).GetEntryByName('Off').GetValue())
        wb_selector.SetIntValue(wb_selector.GetEntryByName('Red').GetValue())
        wb_value.SetValue(self.wb_red)
        wb_selector.SetIntValue(wb_selector.GetEntryByName('Blue').GetValue())
        wb_value.SetValue(self.wb_blue)

        trig_mode = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
        trig_mode.SetIntValue(trig_mode.GetEntryByName('Off').GetValue())

        PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource')).GetEntryByName('Line0').GetValue())
        PySpin.CEnumerationPtr(nodemap.GetNode('TriggerActivation')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('TriggerActivation')).GetEntryByName('RisingEdge').GetValue())
        trig_mode.SetIntValue(trig_mode.GetEntryByName('On').GetValue())

        PySpin.CBooleanPtr(nodemap.GetNode('ChunkModeActive')).SetValue(True)
        chunk_selector = PySpin.CEnumerationPtr(nodemap.GetNode('ChunkSelector'))
        for name in ['FrameID', 'Timestamp']:
            chunk_selector.SetIntValue(chunk_selector.GetEntryByName(name).GetValue())
            PySpin.CBooleanPtr(nodemap.GetNode('ChunkEnable')).SetValue(True)

        PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat')).SetIntValue(
            PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat')).GetEntryByName('BayerRG8').GetValue())

        self.get_logger().info("Camera configuration completed.")

    def _gps_callback(self, msg: NavSatFix):
        t = msg.header.stamp
        ts = t.sec + t.nanosec * 1e-9
        self.gps_history.append((ts, msg.latitude, msg.longitude, msg.status.status))

    def run(self):
        self.get_logger().info('[READY] RGB camera configured. Waiting for external trigger...')
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

                try:
                    img = self.cam.GetNextImage(5000)
                    if img.IsIncomplete():
                        self.get_logger().warn("[WAIT] Incomplete image received.")
                        img.Release()
                        continue
                except PySpin.SpinnakerException as ex:
                    if "-1011" in str(ex) or "timeout" in str(ex).lower():
                        continue
                    self.get_logger().warn(f"[ERROR] {ex}")
                    continue

                now = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]
                t = self.get_clock().now().to_msg()
                ros_ts = f"{t.sec}.{t.nanosec:09d}"
                ros_float = t.sec + t.nanosec * 1e-9

                cd = img.GetChunkData()
                cfid = cd.GetFrameID()
                cts = cd.GetTimestamp()
                lat = lon = fq = ''
                if self.gps_history:
                    best = min(self.gps_history, key=lambda x: abs(x[0] - ros_float))
                    _, lat, lon, fq = best

                w, h = img.GetWidth(), img.GetHeight()
                data = img.GetData()
                fn = os.path.join(self.image_dir, f"{self.counter:06d}.pgm")
                with open(fn, 'wb') as f:
                    f.write(b'P5\n')
                    f.write(f'{w} {h}\n'.encode())
                    f.write(b'255\n')
                    f.write(data)

                self.csv_file.write(f"{self.counter},{now},{ros_ts},{cfid},{cts:.5E},{lat},{lon},{fq}\n")
                self.csv_file.flush()

                self.get_logger().info(f"Saved frame {self.counter:06d} with FrameID {cfid}")
                self.counter += 1
                img.Release()

        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt received. Stopping acquisition...")
        finally:
            self.cleanup()

    def cleanup(self):
        try:
            if self.cam:
                self.cam.EndAcquisition()
                trig_mode = PySpin.CEnumerationPtr(self.cam.GetNodeMap().GetNode('TriggerMode'))
                trig_mode.SetIntValue(trig_mode.GetEntryByName('Off').GetValue())
                self.cam.DeInit()
        except:
            pass
        try:
            if self.csv_file:
                self.csv_file.close()
        except:
            pass
        try:
            self.cam_list.Clear()
            self.system.ReleaseInstance()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RGBImageAcquisitionNode()
    if rclpy.ok():
        node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
