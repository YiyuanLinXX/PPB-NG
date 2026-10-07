#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Record GPS waypoints from the keyboard into `latest_waypoints.csv` and a snapshot CSV.

Press SPACE to log the current GPS fix and `q` to quit.
"""

import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import re
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GnssWaypointKeyboardLogger(Node):
    def __init__(self, output_dir: str, snapshot_name: str | None = None):
        super().__init__('gnss_waypoint_keyboard_logger')
        self.output_dir = str(Path(output_dir).expanduser())
        os.makedirs(self.output_dir, exist_ok=True)

        self.latest_path = os.path.join(self.output_dir, 'latest_waypoints.csv')
        self.hist_path = os.path.join(self.output_dir, self._build_snapshot_filename(snapshot_name))

        self.last_gps = None  # type: NavSatFix
        self.logged_waypoints = []

        # Subscriptions
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)

        self._log_terminal_message('info', f"Output directory: {self.output_dir}")
        self._log_terminal_message('info', f"Historical snapshot file: {os.path.basename(self.hist_path)}")
        self._log_terminal_message('info', "Press SPACE to log waypoint, 'q' to quit.")

    @staticmethod
    def _prepare_terminal_line():
        """Clear any partial raw-terminal content before printing a normal log line."""
        if not sys.stdout.isatty():
            return
        sys.stdout.write('\r\033[2K')
        sys.stdout.flush()

    def _log_terminal_message(self, level: str, message: str):
        self._prepare_terminal_line()
        log_fn = getattr(self.get_logger(), level)
        log_fn(message)

    @staticmethod
    def _build_snapshot_filename(snapshot_name: str | None):
        if snapshot_name:
            return f'{snapshot_name}.csv'

        ts = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        return f"waypoints_{ts}.csv"

    def gps_cb(self, msg: NavSatFix):
        """Store the latest GPS fix."""
        self.last_gps = msg

    def log_waypoint(self):
        """Append the current (lat, lon) to memory and write to CSV files."""
        if self.last_gps is None:
            self._log_terminal_message('warn', "No GPS data received yet.")
            return

        lat = self.last_gps.latitude
        lon = self.last_gps.longitude
        self.logged_waypoints.append((lat, lon))

        # Save latest CSV (overwrite)
        try:
            with open(self.latest_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['latitude', 'longitude'])
                writer.writerows(self.logged_waypoints)
        except Exception as e:
            self._log_terminal_message('error', f"Failed to write latest CSV: {e}")
            return

        # Save historical snapshot CSV (overwrite each time for safety)
        try:
            with open(self.hist_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['latitude', 'longitude'])
                writer.writerows(self.logged_waypoints)
        except Exception as e:
            self._log_terminal_message('error', f"Failed to write history CSV: {e}")
            return

        idx = len(self.logged_waypoints)
        self._log_terminal_message(
            'info',
            f"Logged waypoint #{idx}: lat={lat:.6f}, lon={lon:.6f} → saved to CSV"
        )


def configure_terminal():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    return old_settings


def restore_terminal(old_settings):
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def sanitize_snapshot_name(name: str):
    cleaned = Path(name.strip()).name
    cleaned = re.sub(r'\.csv$', '', cleaned, flags=re.IGNORECASE)
    cleaned = re.sub(r'[^A-Za-z0-9._-]+', '_', cleaned).strip('._-')
    return cleaned or None


def choose_snapshot_name():
    if not sys.stdin.isatty():
        return None

    answer = input(
        'Do you want to manually name the waypoint snapshot CSV file? [y/N]: '
    ).strip().lower()
    if answer not in ('y', 'yes'):
        return None

    while True:
        raw_name = input(
            'Enter snapshot file name (without .csv, latest_waypoints.csv will still be updated): '
        )
        snapshot_name = sanitize_snapshot_name(raw_name)
        if snapshot_name:
            return snapshot_name
        print('Invalid file name. Use letters, numbers, ".", "_" or "-".')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output_dir',
        dest='output_dir',
        type=str,
        default=str(Path.home() / 'navigation_waypoints'),
        help='Directory to save latest and timestamped waypoint files'
    )
    parsed, unknown = parser.parse_known_args()
    snapshot_name = choose_snapshot_name()

    rclpy.init(args=unknown)
    node = GnssWaypointKeyboardLogger(parsed.output_dir, snapshot_name=snapshot_name)

    old_settings = configure_terminal()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == ' ':
                    node.log_waypoint()
                elif ch.lower() == 'q' or ch == '\x03':
                    break
    except KeyboardInterrupt:
        pass
    finally:
        restore_terminal(old_settings)
        node.destroy_node()
        rclpy.shutdown()

        if sys.stdout.isatty():
            sys.stdout.write('\r\033[2K')
            sys.stdout.flush()
        print("Exited GPS Keyboard Logger.")
        print(f"Waypoints saved to:")
        print(f"  Latest → {node.latest_path}")
        print(f"  Snapshot → {node.hist_path}")


if __name__ == '__main__':
    main()
