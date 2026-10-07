#!/usr/bin/env python3
"""Unified UM982 serial, NTRIP, midpoint-position and odometry driver."""

import base64
from collections import deque
import math
import socket
import ssl
import threading
import time

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
from std_msgs.msg import Bool, Float64, Float64MultiArray
from pyproj import Transformer

from .um982_protocol import (
    heading_to_enu_yaw,
    midpoint_from_master,
    parse_gga,
    parse_uniheadinga,
    robot_heading_from_baseline,
)


class NtripProtocolRejected(RuntimeError):
    """The caster replied, but did not accept this HTTP request form."""


class NtripWorker:
    """Receive one NTRIP stream without ever owning the serial port."""

    def __init__(self, node, output_queue, stop_event):
        self.node = node
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.last_rx_monotonic = None
        self.connected = False
        self.error = 'not started'
        self.http_version = '1.1'
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        retry_sec = 1.0
        while not self.stop_event.is_set():
            try:
                self._receive_stream()
                retry_sec = 1.0
            except NtripProtocolRejected as exc:
                self.connected = False
                self.error = str(exc)
                self.http_version = (
                    '1.0' if self.http_version == '1.1' else '1.1'
                )
                self.node.get_logger().warning(
                    f'{exc}; trying NTRIP HTTP/{self.http_version} request'
                )
                self.stop_event.wait(0.5)
            except Exception as exc:
                self.connected = False
                self.error = str(exc)
                self.http_version = (
                    '1.0' if self.http_version == '1.1' else '1.1'
                )
                self.node.get_logger().warning(
                    f'NTRIP disconnected: {exc}; trying HTTP/'
                    f'{self.http_version} in {retry_sec:.1f}s'
                )
                self.stop_event.wait(retry_sec)
                retry_sec = min(retry_sec * 2.0, 30.0)

    def _receive_stream(self):
        host = self.node.string_parameter('ntrip_host')
        port = self.node.integer_parameter('ntrip_port')
        mountpoint = self.node.string_parameter('ntrip_mountpoint').lstrip('/')
        username = self.node.string_parameter('ntrip_username')
        password = self.node.ntrip_password()
        timeout = self.node.double_parameter('ntrip_socket_timeout_sec')
        use_tls = self.node.bool_parameter('ntrip_use_tls')

        if not host or not mountpoint:
            raise RuntimeError(
                'ntrip_host and ntrip_mountpoint must be configured'
            )

        sock = socket.create_connection((host, port), timeout=timeout)
        if use_tls:
            context = ssl.create_default_context()
            sock = context.wrap_socket(sock, server_hostname=host)
        sock.settimeout(timeout)

        auth = base64.b64encode(
            f'{username}:{password}'.encode('utf-8')
        ).decode('ascii')
        # Legacy NTRIP v1 casters (including Reach Local NTRIP) may behave
        # differently when an Ntrip/2.0 header is sent with an HTTP/1.0
        # request.  Keep the v1 request genuinely v1-compatible.
        version_header = (
            'Ntrip-Version: Ntrip/2.0\r\n'
            if self.http_version == '1.1'
            else ''
        )
        request = (
            f'GET /{mountpoint} HTTP/{self.http_version}\r\n'
            f'Host: {host}:{port}\r\n'
            f'{version_header}'
            'User-Agent: NTRIP PPBNG-UM982/1.0\r\n'
            'Accept: */*\r\n'
            f'Authorization: Basic {auth}\r\n'
            'Connection: close\r\n\r\n'
        )
        sock.sendall(request.encode('ascii'))
        header, initial_body = self._read_response_header(sock)
        status_line = header.splitlines()[0]
        if not (
            status_line.startswith('ICY 200')
            or status_line.startswith('HTTP/1.0 200')
            or status_line.startswith('HTTP/1.1 200')
        ):
            sock.close()
            if ' 401 ' in status_line or ' 403 ' in status_line:
                raise RuntimeError(
                    f'NTRIP authentication rejected: {status_line}'
                )
            raise NtripProtocolRejected(
                f'NTRIP request rejected: {status_line}'
            )

        self.connected = True
        self.error = ''
        self.node.get_logger().info(
            f'NTRIP connected to {host}:{port}/{mountpoint} '
            f'({"TLS" if use_tls else "TCP"}; response {status_line})'
        )
        if initial_body:
            self._enqueue(initial_body)

        try:
            while not self.stop_event.is_set():
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError('caster closed the stream')
                self._enqueue(chunk)
        finally:
            self.connected = False
            sock.close()

    @staticmethod
    def _read_response_header(sock):
        data = bytearray()
        while True:
            # Some legacy casters start the authenticated correction stream
            # directly with RTCM3, sometimes after a few non-HTTP prefix
            # bytes. RTCM3 frames start with the 0xD3 preamble.
            rtcm_start = data.find(b'\xd3')
            if rtcm_start != -1:
                return (
                    'ICY 200 OK (headerless RTCM3 stream)',
                    bytes(data[rtcm_start:]),
                )
            normalized = bytes(data).lstrip(b'\r\n')
            if normalized.startswith(b'ICY ') and b'\r\n' in normalized:
                header, body = normalized.split(b'\r\n', 1)
                return header.decode(
                    'iso-8859-1', errors='replace'
                ), body
            if b'\r\n\r\n' in data:
                header, body = bytes(data).split(b'\r\n\r\n', 1)
                return header.decode(
                    'iso-8859-1', errors='replace'
                ), body
            chunk = sock.recv(1024)
            if not chunk:
                raise ConnectionError(
                    'caster closed connection during handshake'
                )
            data.extend(chunk)
            if len(data) > 16384:
                raise RuntimeError('NTRIP response header is too large')

    def _enqueue(self, chunk):
        self.last_rx_monotonic = time.monotonic()
        self.output_queue.append(chunk)


class Um982Driver(Node):
    def __init__(self):
        super().__init__('um982_driver')
        self._declare_parameters()
        self.serial_port = serial.Serial(
            self.string_parameter('serial_port'),
            self.integer_parameter('baudrate'),
            timeout=0,
            write_timeout=self.double_parameter('serial_write_timeout_sec'),
        )
        if self.bool_parameter('configure_receiver_on_start'):
            self._configure_receiver()

        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        self.status_pub = self.create_publisher(
            Bool, '/gps/rtk_status_flag', 10
        )
        self.heading_pub = self.create_publisher(
            Float64, '/um982/heading_deg', 10
        )
        self.pitch_pub = self.create_publisher(
            Float64, '/um982/pitch_deg', 10
        )

        datum_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            Float64MultiArray, '/gps/datum', self.datum_callback, datum_qos
        )
        self.transformer = None
        self.datum_altitude = 0.0
        self.latest_heading = None
        self.latest_heading_rx = None
        self.latest_gga_rx = None
        self.last_valid_solution_rx = None
        self.latest_fix_quality = None
        self.latest_differential_age = None
        self.solution_valid = False
        self.solution_reason = 'waiting for GGA'
        self.last_logged_solution_valid = None
        self.last_status_log_time = None
        self.rx_buffer = bytearray()
        self.last_warning = {}
        self.stop_event = threading.Event()
        self.rtcm_queue = deque(
            maxlen=self.integer_parameter('rtcm_queue_max_chunks')
        )
        self.ntrip = None
        if self.bool_parameter('ntrip_enabled'):
            self.ntrip = NtripWorker(self, self.rtcm_queue, self.stop_event)
            self.ntrip.start()

        self.create_timer(
            self.double_parameter('poll_period_sec'), self.poll_serial
        )
        self.create_timer(0.1, self.publish_safety_status)
        self.get_logger().info(
            f'UM982 driver opened {self.string_parameter("serial_port")} '
            f'@ {self.integer_parameter("baudrate")} bps; '
            f'baseline={self.double_parameter("baseline_m"):.3f} m; '
            'ANT1-to-ANT2 angle relative to robot forward='
            f'{self.double_parameter("antenna_baseline_angle_deg"):.1f} deg'
        )

    def _declare_parameters(self):
        defaults = {
            'serial_port': '/dev/serial/by-id/REPLACE_WITH_UM982',
            'baudrate': 115200,
            'poll_period_sec': 0.01,
            'serial_write_timeout_sec': 0.5,
            'baseline_m': 1.0,
            'baseline_tolerance_m': 0.05,
            'antenna_baseline_angle_deg': 0.0,
            'heading_offset_deg': 0.0,
            'pitch_multiplier': 1.0,
            'max_heading_stddev_deg': 1.0,
            'max_heading_age_sec': 0.3,
            'max_position_heading_skew_sec': 0.2,
            'max_differential_age_sec': 3.0,
            'minimum_heading_satellites': 6,
            'configure_receiver_on_start': True,
            'receiver_output_period_sec': 0.1,
            'receiver_output_ports': ['COM1', 'COM2', 'COM3'],
            'receiver_port_baudrate': 115200,
            'receiver_enable_gpths': True,
            'ntrip_enabled': True,
            'ntrip_host': '',
            'ntrip_port': 2101,
            'ntrip_mountpoint': '',
            'ntrip_username': '',
            'ntrip_password': '',
            'ntrip_use_tls': False,
            'ntrip_socket_timeout_sec': 10.0,
            'ntrip_data_timeout_sec': 5.0,
            'rtcm_queue_max_chunks': 256,
            'gps_status_log_enabled': True,
            'gps_status_log_period_sec': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _configure_receiver(self):
        baseline_cm = round(self.double_parameter('baseline_m') * 100.0)
        tolerance_cm = max(
            1, round(self.double_parameter('baseline_tolerance_m') * 100.0)
        )
        period = self.double_parameter('receiver_output_period_sec')
        output_ports = self._receiver_output_ports()
        port_baudrate = self.integer_parameter('receiver_port_baudrate')
        commands = [
            'UNLOG',
            'MODE ROVER',
            'CONFIG HEADING FIXLENGTH',
            f'CONFIG HEADING LENGTH {baseline_cm} {tolerance_cm}',
        ]
        for port in output_ports:
            commands.extend([
                f'CONFIG {port} {port_baudrate} 8 N 1',
                f'GPGGA {port} {period:g}',
                f'UNIHEADINGA {port} {period:g}',
            ])
            if self.bool_parameter('receiver_enable_gpths'):
                commands.append(f'GPTHS {port} {period:g}')
        commands.append('SAVECONFIG')
        for command in commands:
            self.serial_port.write(f'{command}\r\n'.encode('ascii'))
            time.sleep(0.03)
        self.serial_port.flush()
        self.get_logger().info(
            'Configured UM982 rover, fixed heading baseline, and GNSS/'
            f'heading output on {", ".join(output_ports)} at '
            f'{port_baudrate} bps.'
        )

    def _receiver_output_ports(self):
        supported_ports = {'COM1', 'COM2', 'COM3'}
        configured_ports = self.get_parameter(
            'receiver_output_ports'
        ).value
        ports = []
        for configured_port in configured_ports:
            port = str(configured_port).upper()
            if port not in supported_ports:
                raise ValueError(
                    'receiver_output_ports contains unsupported port '
                    f'{configured_port!r}; use COM1, COM2, and/or COM3'
                )
            if port not in ports:
                ports.append(port)
        if not ports:
            raise ValueError('receiver_output_ports must not be empty')
        return ports

    def string_parameter(self, name):
        return self.get_parameter(name).value

    def integer_parameter(self, name):
        return int(self.get_parameter(name).value)

    def double_parameter(self, name):
        return float(self.get_parameter(name).value)

    def bool_parameter(self, name):
        return bool(self.get_parameter(name).value)

    def ntrip_password(self):
        return self.string_parameter('ntrip_password')

    def datum_callback(self, msg):
        if len(msg.data) < 3:
            return
        lat, lon, altitude = msg.data[:3]
        self.transformer = Transformer.from_crs(
            'epsg:4326',
            (
                f'+proj=tmerc +lat_0={lat} +lon_0={lon} +k=1 '
                '+x_0=0 +y_0=0 +datum=WGS84'
            ),
            always_xy=True,
        )
        self.datum_altitude = altitude

    def poll_serial(self):
        self._write_rtcm()
        try:
            available = self.serial_port.in_waiting
            if available:
                self.rx_buffer.extend(self.serial_port.read(available))
        except serial.SerialException as exc:
            self.warn_limited(
                'serial_read', f'UM982 serial read failed: {exc}'
            )
            return

        while b'\n' in self.rx_buffer:
            raw_line, _, remainder = self.rx_buffer.partition(b'\n')
            self.rx_buffer = bytearray(remainder)
            line = raw_line.decode('ascii', errors='ignore').strip()
            self._process_line(line)

    def _write_rtcm(self):
        try:
            for _ in range(16):
                if not self.rtcm_queue:
                    break
                self.serial_port.write(self.rtcm_queue.popleft())
        except serial.SerialException as exc:
            self.warn_limited(
                'serial_write', f'RTCM serial write failed: {exc}'
            )

    def _process_line(self, line):
        now = time.monotonic()
        try:
            if line.startswith('$') and 'GGA,' in line:
                fix = parse_gga(line)
                self.latest_gga_rx = now
                self._process_fix(fix, now)
            elif line.startswith('#UNIHEADINGA,'):
                self.latest_heading = parse_uniheadinga(line)
                self.latest_heading_rx = now
        except Exception as exc:
            self.warn_limited(
                'parse', f'Skipping malformed UM982 message: {exc}'
            )

    def _valid_heading(self, now):
        heading = self.latest_heading
        if heading is None or self.latest_heading_rx is None:
            return False, 'no heading solution'
        if now - self.latest_heading_rx > self.double_parameter(
            'max_heading_age_sec'
        ):
            return False, 'heading is stale'
        if heading.solution_status != 'SOL_COMPUTED':
            return False, f'heading status={heading.solution_status}'
        if heading.position_type != 'NARROW_INT':
            return False, f'heading type={heading.position_type}'
        if abs(
            heading.baseline_m - self.double_parameter('baseline_m')
        ) > self.double_parameter('baseline_tolerance_m'):
            return False, 'reported baseline is outside tolerance'
        if heading.heading_stddev_deg > self.double_parameter(
            'max_heading_stddev_deg'
        ):
            return False, 'heading standard deviation is too high'
        if heading.satellites_used < self.integer_parameter(
            'minimum_heading_satellites'
        ):
            return False, 'too few satellites in heading solution'
        return True, ''

    def _process_fix(self, fix, now):
        self.latest_fix_quality = fix.quality
        self.latest_differential_age = fix.differential_age_sec
        heading_ok, reason = self._valid_heading(now)
        position_ok = fix.quality == 4
        age_ok = (
            fix.differential_age_sec is not None
            and fix.differential_age_sec
            <= self.double_parameter('max_differential_age_sec')
        )
        skew_ok = (
            self.latest_heading_rx is not None
            and abs(now - self.latest_heading_rx)
            <= self.double_parameter('max_position_heading_skew_sec')
        )
        if not (position_ok and age_ok and heading_ok and skew_ok):
            self.solution_valid = False
            self.solution_reason = self._build_rejection_reason(
                position_ok, age_ok, heading_ok, skew_ok, reason
            )
            return

        heading = self.latest_heading
        robot_heading = robot_heading_from_baseline(
            heading.heading_deg,
            self.double_parameter('antenna_baseline_angle_deg'),
            self.double_parameter('heading_offset_deg'),
        )
        pitch = (
            heading.pitch_deg * self.double_parameter('pitch_multiplier')
        )
        # GGA reports ANT1. Use the raw ANT1-to-ANT2 direction to calculate
        # the antenna midpoint; robot_heading can point elsewhere.
        lat, lon, altitude = midpoint_from_master(
            fix.latitude,
            fix.longitude,
            fix.altitude_m,
            heading.heading_deg,
            pitch,
            self.double_parameter('baseline_m'),
        )
        stamp = self.get_clock().now().to_msg()
        self._publish_fix(stamp, lat, lon, altitude)
        self._publish_odometry(
            stamp, lat, lon, altitude, robot_heading,
            heading.heading_stddev_deg,
        )
        self.heading_pub.publish(Float64(data=robot_heading))
        self.pitch_pub.publish(Float64(data=pitch))
        self.last_valid_solution_rx = now
        self.solution_valid = True
        self.solution_reason = 'ok'

    def _build_rejection_reason(
        self, position_ok, age_ok, heading_ok, skew_ok, heading_reason
    ):
        reasons = []
        if not position_ok:
            reasons.append(f'GGA quality={self.latest_fix_quality}')
        if not age_ok:
            diff_age = self._format_optional_float(
                self.latest_differential_age
            )
            reasons.append(f'diff_age={diff_age}s')
        if not heading_ok:
            reasons.append(f'heading={heading_reason or "invalid"}')
        if not skew_ok:
            reasons.append('GGA/heading skew too large')
        return ', '.join(reasons) if reasons else 'invalid solution'

    def _publish_fix(self, stamp, latitude, longitude, altitude):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [
            0.02 ** 2, 0.0, 0.0,
            0.0, 0.02 ** 2, 0.0,
            0.0, 0.0, 0.05 ** 2,
        ]
        self.fix_pub.publish(msg)

    def _publish_odometry(
        self, stamp, latitude, longitude, altitude, heading_deg, heading_stddev
    ):
        if self.transformer is None:
            return
        east, north = self.transformer.transform(longitude, latitude)
        yaw = heading_to_enu_yaw(heading_deg)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = east
        msg.pose.pose.position.y = north
        msg.pose.pose.position.z = altitude - self.datum_altitude
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        )
        yaw_variance = math.radians(heading_stddev) ** 2
        msg.pose.covariance = [
            0.0004, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0004, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0025, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_variance,
        ]
        self.odom_pub.publish(msg)

    def publish_safety_status(self):
        now = time.monotonic()
        valid = self.last_valid_solution_rx is not None
        if valid:
            valid = now - self.last_valid_solution_rx <= 0.5
        if self.ntrip is not None:
            valid = valid and self.ntrip.connected
            if self.ntrip.last_rx_monotonic is None:
                valid = False
            else:
                valid = valid and (
                    now - self.ntrip.last_rx_monotonic
                    <= self.double_parameter('ntrip_data_timeout_sec')
                )
        self.status_pub.publish(Bool(data=not valid))
        self._maybe_log_gps_status(valid, now)

    def _maybe_log_gps_status(self, valid, now):
        if not self.bool_parameter('gps_status_log_enabled'):
            return

        state_changed = (
            self.last_logged_solution_valid is None
            or valid != self.last_logged_solution_valid
        )
        period = max(self.double_parameter('gps_status_log_period_sec'), 1.0)
        periodic_due = (
            self.last_status_log_time is None
            or now - self.last_status_log_time >= period
        )
        if not state_changed and not periodic_due:
            return

        message = self._format_gps_status(valid, now)
        if valid:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)
        self.last_logged_solution_valid = valid
        self.last_status_log_time = now

    def _format_gps_status(self, valid, now):
        status = 'OK' if valid else 'NOT FIX'
        reason = 'ok' if valid else self._runtime_status_reason(now)
        gga_age = self._age_text(now, self.latest_gga_rx)
        heading_age = self._age_text(now, self.latest_heading_rx)
        valid_age = self._age_text(now, self.last_valid_solution_rx)
        diff_age = self._format_optional_float(self.latest_differential_age)
        heading_text = self._heading_status_text()
        ntrip_text = self._ntrip_status_text(now)
        return (
            f'GPS status: {status} | reason={reason} | '
            f'GGA q={self.latest_fix_quality}, age={gga_age}, '
            f'diff_age={diff_age}s | '
            f'heading={heading_text}, age={heading_age} | '
            f'last_valid={valid_age} | NTRIP={ntrip_text}'
        )

    def _runtime_status_reason(self, now):
        reasons = []
        if not self.solution_valid and self.solution_reason:
            reasons.append(self.solution_reason)
        if self.last_valid_solution_rx is None:
            reasons.append('no valid navigation solution yet')
        elif now - self.last_valid_solution_rx > 0.5:
            reasons.append('valid solution is stale')
        if self.ntrip is not None:
            if not self.ntrip.connected:
                reasons.append('NTRIP disconnected')
            elif self.ntrip.last_rx_monotonic is None:
                reasons.append('no RTCM received')
            elif (
                now - self.ntrip.last_rx_monotonic
                > self.double_parameter('ntrip_data_timeout_sec')
            ):
                reasons.append('RTCM data timed out')
        return '; '.join(reasons) if reasons else self.solution_reason

    def _heading_status_text(self):
        heading = self.latest_heading
        if heading is None:
            return 'none'
        return (
            f'{heading.solution_status}/{heading.position_type}, '
            f'std={heading.heading_stddev_deg:.2f}deg, '
            f'sats={heading.satellites_used}'
        )

    def _ntrip_status_text(self, now):
        if self.ntrip is None:
            return 'disabled'
        if self.ntrip.last_rx_monotonic is None:
            age = 'none'
        else:
            age = f'{now - self.ntrip.last_rx_monotonic:.1f}s'
        if self.ntrip.connected:
            state = 'connected'
        else:
            state = f'disconnected:{self.ntrip.error or "unknown"}'
        return f'{state}, rx_age={age}'

    @staticmethod
    def _format_optional_float(value):
        if value is None:
            return 'none'
        return f'{value:.2f}'

    @staticmethod
    def _age_text(now, timestamp):
        if timestamp is None:
            return 'none'
        return f'{now - timestamp:.1f}s'

    def warn_limited(self, key, message, period_sec=2.0):
        now = time.monotonic()
        if now - self.last_warning.get(key, 0.0) >= period_sec:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.serial_port.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Um982Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
