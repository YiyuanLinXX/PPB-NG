#!/usr/bin/env python3
"""Waypoint follower node with parameterized controller tuning and debug output."""

from argparse import ArgumentParser
import csv
import json
import math
from pathlib import Path
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from pyproj import Transformer

from amiga_navigation.utils.pid_line_controller import (
    LineTrackingConfig,
)
from amiga_navigation.utils.tracking_geometry import (
    TrackingCommand,
    compute_segment_metrics,
    is_goal_reached,
)
from amiga_navigation.utils.tracking_controller_factory import (
    FormalMPCConfig,
    MPCRolloutConfig,
    PurePursuitConfig,
    RowHybridConfig,
    build_tracking_controller,
)
from amiga_navigation.utils.alignment_turn_controller import TurnConfig, TurnCommand, compute_turn_command
from amiga_navigation.utils.navigation_safety import (
    NavigationProgressWatchdog,
    NavigationWatchdogConfig,
    validate_velocity_command,
)


DEFAULT_WAYPOINT_DIRECTORY = Path.home() / 'navigation_waypoints'
DEFAULT_WAYPOINTS_PATH = str(DEFAULT_WAYPOINT_DIRECTORY / 'latest_waypoints.csv')
CONTROLLER_CHOICES = ['pid_line', 'pure_pursuit', 'mpc_rollout', 'mpc_formal', 'row_hybrid']
SOURCE_PARAMS_PATH = (
    Path('src') / 'amiga_navigation' / 'config' /
    'waypoint_follower_params.yaml'
)
DEFAULT_PARAMS_FILE_CANDIDATES = [
    Path.home() / 'PPBNG_Navigation' / SOURCE_PARAMS_PATH,
    Path.home() / 'PPBNG' / 'PPBNG_Navigation' / SOURCE_PARAMS_PATH,
    Path(__file__).resolve().parents[1] /
    'config' / 'waypoint_follower_params.yaml',
    Path(__file__).resolve().parents[4] /
    'share' / 'amiga_navigation' / 'config' /
    'waypoint_follower_params.yaml',
]


class WaypointFollower(Node):
    def __init__(self, csv_path, resume_mode='ask', controller_override=None):
        super().__init__('waypoint_follower')
        self._declare_parameters()

        self.control_frequency = self._get_float('control_frequency')
        self.max_odom_age_sec = self._get_float('max_odom_age_sec')
        self.enable_csv_logging = self._get_bool('enable_csv_logging')
        self.publish_debug = self._get_bool('publish_debug')
        self.debug_publish_period_sec = self._get_float('debug_publish_period_sec')
        self.terminal_status_enabled = self._get_bool('terminal_status_enabled')
        self.terminal_status_level = self._get_str('terminal_status_level').lower()
        self.terminal_status_period_sec = self._get_float('terminal_status_period_sec')
        self.terminal_show_alignment_details = self._get_bool('terminal_show_alignment_details')
        self.terminal_show_tracking_details = self._get_bool('terminal_show_tracking_details')
        self.progress_watchdog_enabled = self._get_bool('progress_watchdog_enabled')
        self.min_segment_length_m = self._get_float('min_segment_length_m')
        self.command_max_linear_speed = self._get_float('command_max_linear_speed')
        self.command_max_angular_speed = self._get_float('command_max_angular_speed')

        self.log_directory = Path(self._get_str('log_directory')).expanduser()
        self.log_directory.mkdir(parents=True, exist_ok=True)
        self.status_path = Path(self._get_str('status_path')).expanduser()
        self.last_wp_path = Path(self._get_str('last_waypoints_path')).expanduser()
        self.requested_csv_path = Path(csv_path).expanduser()
        self.resume_mode = resume_mode
        self.csv_path, self.current_index = self._select_navigation_file()
        self.selected_controller_type = controller_override or self._get_str('controller_type')

        self.tracking_config = LineTrackingConfig(
            target_speed=self._get_float('target_speed'),
            max_lateral_speed=self._get_float('max_lateral_speed'),
            epsilon=self._get_float('epsilon'),
            pid_kp=self._get_float('pid_kp'),
            pid_ki=self._get_float('pid_ki'),
            pid_kd=self._get_float('pid_kd'),
            heading_gain=self._get_float('heading_gain'),
            max_angular_speed=self._get_float('max_angular_speed'),
            min_forward_ratio=self._get_float('min_forward_ratio'),
            max_heading_for_full_speed=self._get_float('max_heading_for_full_speed'),
            max_cross_track_error=self._get_float('max_cross_track_error'),
            goal_threshold=self._get_float('goal_threshold'),
            alignment_threshold=self._get_float('alignment_threshold'),
            dist_start_threshold=self._get_float('dist_start_threshold'),
            dist_stop_threshold=self._get_float('dist_stop_threshold'),
            initial_speed_ratio=self._get_float('initial_speed_ratio'),
            stop_speed_ratio=self._get_float('stop_speed_ratio'),
            enable_start_slowdown=self._get_bool('enable_start_slowdown'),
            enable_goal_slowdown=self._get_bool('enable_goal_slowdown'),
            regulate_target_speed=self._get_bool('regulate_target_speed'),
        )
        self.turn_config = TurnConfig(
            alignment_threshold=self._get_float('alignment_threshold'),
            gain=self._get_float('turn_gain'),
            min_turn_speed=self._get_float('turn_min_speed'),
            max_turn_speed=self._get_float('turn_max_speed'),
            enable_slowdown_near_target=self._get_bool('turn_enable_slowdown_near_target'),
            slowdown_angle=self._get_float('turn_slowdown_angle'),
            near_target_min_speed_ratio=self._get_float('turn_near_target_min_speed_ratio'),
        )
        self.pure_pursuit_config = PurePursuitConfig(
            target_speed=self._get_float('target_speed'),
            min_lookahead=self._get_float('pure_pursuit_min_lookahead'),
            max_lookahead=self._get_float('pure_pursuit_max_lookahead'),
            lookahead_gain=self._get_float('pure_pursuit_lookahead_gain'),
            slowdown_distance=self._get_float('pure_pursuit_slowdown_distance'),
            max_angular_speed=self._get_float('max_angular_speed'),
            min_forward_ratio=self._get_float('min_forward_ratio'),
            start_slowdown_distance=self._get_float('dist_start_threshold'),
            initial_speed_ratio=self._get_float('initial_speed_ratio'),
            enable_start_slowdown=self._get_bool('enable_start_slowdown'),
            enable_goal_slowdown=self._get_bool('enable_goal_slowdown'),
            max_cross_track_error=self._get_float('max_cross_track_error'),
            goal_threshold=self._get_float('goal_threshold'),
        )
        self.mpc_rollout_config = MPCRolloutConfig(
            target_speed=self._get_float('target_speed'),
            horizon_steps=self._get_int('mpc_horizon_steps'),
            step_time=self._get_float('mpc_step_time'),
            candidate_count=self._get_int('mpc_candidate_count'),
            max_angular_speed=self._get_float('max_angular_speed'),
            min_forward_ratio=self._get_float('min_forward_ratio'),
            slowdown_distance=self._get_float('mpc_slowdown_distance'),
            heading_weight=self._get_float('mpc_heading_weight'),
            cross_track_weight=self._get_float('mpc_cross_track_weight'),
            goal_distance_weight=self._get_float('mpc_goal_distance_weight'),
            effort_weight=self._get_float('mpc_effort_weight'),
            progress_weight=self._get_float('mpc_progress_weight'),
            start_slowdown_distance=self._get_float('dist_start_threshold'),
            initial_speed_ratio=self._get_float('initial_speed_ratio'),
            enable_start_slowdown=self._get_bool('enable_start_slowdown'),
            enable_goal_slowdown=self._get_bool('enable_goal_slowdown'),
            max_cross_track_error=self._get_float('max_cross_track_error'),
            goal_threshold=self._get_float('goal_threshold'),
        )
        self.formal_mpc_config = FormalMPCConfig(
            target_speed=self._get_float('target_speed'),
            horizon_steps=self._get_int('formal_mpc_horizon_steps'),
            step_time=self._get_float('formal_mpc_step_time'),
            min_forward_speed=self._get_float('formal_mpc_min_forward_speed'),
            max_angular_speed=self._get_float('max_angular_speed'),
            slowdown_distance=self._get_float('formal_mpc_slowdown_distance'),
            min_forward_ratio=self._get_float('min_forward_ratio'),
            cross_track_weight=self._get_float('formal_mpc_cross_track_weight'),
            heading_weight=self._get_float('formal_mpc_heading_weight'),
            goal_distance_weight=self._get_float('formal_mpc_goal_distance_weight'),
            terminal_cross_track_weight=self._get_float('formal_mpc_terminal_cross_track_weight'),
            terminal_heading_weight=self._get_float('formal_mpc_terminal_heading_weight'),
            terminal_goal_distance_weight=self._get_float('formal_mpc_terminal_goal_distance_weight'),
            linear_effort_weight=self._get_float('formal_mpc_linear_effort_weight'),
            angular_effort_weight=self._get_float('formal_mpc_angular_effort_weight'),
            linear_smooth_weight=self._get_float('formal_mpc_linear_smooth_weight'),
            angular_smooth_weight=self._get_float('formal_mpc_angular_smooth_weight'),
            progress_weight=self._get_float('formal_mpc_progress_weight'),
            solver_maxiter=self._get_int('formal_mpc_solver_maxiter'),
            solver_ftol=self._get_float('formal_mpc_solver_ftol'),
            start_slowdown_distance=self._get_float('dist_start_threshold'),
            initial_speed_ratio=self._get_float('initial_speed_ratio'),
            enable_start_slowdown=self._get_bool('enable_start_slowdown'),
            enable_goal_slowdown=self._get_bool('enable_goal_slowdown'),
            max_cross_track_error=self._get_float('max_cross_track_error'),
            goal_threshold=self._get_float('goal_threshold'),
        )
        self.row_hybrid_config = RowHybridConfig(
            connector_length_threshold=self._get_float('row_connector_length_threshold'),
            row_length_threshold=self._get_float('row_length_threshold'),
        )
        self.progress_watchdog = NavigationProgressWatchdog(
            NavigationWatchdogConfig(
                alignment_max_duration_sec=self._get_float('alignment_max_duration_sec'),
                alignment_progress_timeout_sec=self._get_float('alignment_progress_timeout_sec'),
                alignment_min_progress_rad=self._get_float('alignment_min_progress_rad'),
                tracking_progress_timeout_sec=self._get_float('tracking_progress_timeout_sec'),
                tracking_min_progress_m=self._get_float('tracking_min_progress_m'),
                motion_response_timeout_sec=self._get_float('motion_response_timeout_sec'),
                motion_min_translation_m=self._get_float('motion_min_translation_m'),
                motion_min_rotation_rad=self._get_float('motion_min_rotation_rad'),
                command_min_linear_mps=self._get_float('watchdog_command_min_linear_mps'),
                command_min_angular_rps=self._get_float('watchdog_command_min_angular_rps'),
            )
        )

        self.controller = build_tracking_controller(
            controller_type=self.selected_controller_type,
            line_config=self.tracking_config,
            pure_pursuit_config=self.pure_pursuit_config,
            mpc_rollout_config=self.mpc_rollout_config,
            formal_mpc_config=self.formal_mpc_config,
            row_hybrid_config=self.row_hybrid_config,
        )
        self._log_controller_configuration()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.safety_stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.debug_pub = self.create_publisher(String, '/nav/controller_debug', 10)
        self.create_subscription(Odometry, '/robot/odom', self.odom_callback, qos_profile_sensor_data)
        datum_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64MultiArray, '/gps/datum', self.datum_callback, datum_qos)

        self.create_timer(1.0 / max(self.control_frequency, 1.0), self.control_loop)
        self.create_timer(1.0, self.flush_log_file)

        self.pose = None
        self.last_odom_time = None
        self.last_odom_warn_time = None
        self.last_invalid_odom_warn_time = None
        self.last_debug_publish_time = None
        self.last_terminal_status_time = None
        self.transformer = None
        self.reached_final = False
        self.phase = 'waiting_for_pose'
        self.last_phase = None
        self.init_pose_inserted = False
        self.segment_aligned = False
        self.current_route = None
        self.last_segment_key = None
        self.active_controller_name = self.selected_controller_type
        self.safety_fault_reason = None

        self.waypoints_gps = self.load_csv_waypoints(self.csv_path)
        self.waypoints_enu = []
        self._prepare_waypoint_state()
        self._setup_log_file()
        self._log_startup_summary()

        self.get_logger().info('WaypointFollower initialized')

    def _declare_parameters(self):
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_odom_age_sec', 0.25)
        self.declare_parameter('enable_csv_logging', True)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_publish_period_sec', 0.2)
        self.declare_parameter('terminal_status_enabled', True)
        self.declare_parameter('terminal_status_level', 'normal')
        self.declare_parameter('terminal_status_period_sec', 1.5)
        self.declare_parameter('terminal_show_alignment_details', False)
        self.declare_parameter('terminal_show_tracking_details', False)
        self.declare_parameter('log_directory', str(DEFAULT_WAYPOINT_DIRECTORY))
        self.declare_parameter('status_path', str(DEFAULT_WAYPOINT_DIRECTORY / 'status.txt'))
        self.declare_parameter('last_waypoints_path', str(DEFAULT_WAYPOINT_DIRECTORY / 'last_waypoints.csv'))
        self.declare_parameter('controller_type', 'pid_line')
        self.declare_parameter('progress_watchdog_enabled', True)
        self.declare_parameter('min_segment_length_m', 0.05)
        self.declare_parameter('command_max_linear_speed', 2.0)
        self.declare_parameter('command_max_angular_speed', 1.5)
        self.declare_parameter('alignment_max_duration_sec', 15.0)
        self.declare_parameter('alignment_progress_timeout_sec', 15.0)
        self.declare_parameter('alignment_min_progress_rad', 0.05)
        self.declare_parameter('tracking_progress_timeout_sec', 10.0)
        self.declare_parameter('tracking_min_progress_m', 0.15)
        self.declare_parameter('motion_response_timeout_sec', 15.0)
        self.declare_parameter('motion_min_translation_m', 0.05)
        self.declare_parameter('motion_min_rotation_rad', 0.05)
        self.declare_parameter('watchdog_command_min_linear_mps', 0.05)
        self.declare_parameter('watchdog_command_min_angular_rps', 0.08)

        self.declare_parameter('target_speed', 0.85)
        self.declare_parameter('max_lateral_speed', 0.4)
        self.declare_parameter('epsilon', 0.5)
        self.declare_parameter('pid_kp', 0.38)
        self.declare_parameter('pid_ki', 0.05)
        self.declare_parameter('pid_kd', 0.28)
        self.declare_parameter('heading_gain', 1.0)
        self.declare_parameter('max_angular_speed', 0.65)
        self.declare_parameter('min_forward_ratio', 0.2)
        self.declare_parameter('max_heading_for_full_speed', 0.26)
        self.declare_parameter('max_cross_track_error', 1.0)
        self.declare_parameter('goal_threshold', 0.30)
        self.declare_parameter('alignment_threshold', 0.18)
        self.declare_parameter('dist_start_threshold', 4.5)
        self.declare_parameter('dist_stop_threshold', 1.0)
        self.declare_parameter('initial_speed_ratio', 0.22)
        self.declare_parameter('stop_speed_ratio', 0.0)
        self.declare_parameter('enable_start_slowdown', True)
        self.declare_parameter('enable_goal_slowdown', True)
        self.declare_parameter('regulate_target_speed', True)
        self.declare_parameter('turn_gain', 0.7)
        self.declare_parameter('turn_min_speed', 0.3)
        self.declare_parameter('turn_max_speed', 0.4)
        self.declare_parameter('turn_enable_slowdown_near_target', False)
        self.declare_parameter('turn_slowdown_angle', 0.35)
        self.declare_parameter('turn_near_target_min_speed_ratio', 0.35)
        self.declare_parameter('pure_pursuit_min_lookahead', 1.5)
        self.declare_parameter('pure_pursuit_max_lookahead', 6.0)
        self.declare_parameter('pure_pursuit_lookahead_gain', 2.5)
        self.declare_parameter('pure_pursuit_slowdown_distance', 3.0)
        self.declare_parameter('mpc_horizon_steps', 10)
        self.declare_parameter('mpc_step_time', 0.2)
        self.declare_parameter('mpc_candidate_count', 15)
        self.declare_parameter('mpc_slowdown_distance', 3.0)
        self.declare_parameter('mpc_heading_weight', 1.5)
        self.declare_parameter('mpc_cross_track_weight', 2.0)
        self.declare_parameter('mpc_goal_distance_weight', 4.0)
        self.declare_parameter('mpc_effort_weight', 0.3)
        self.declare_parameter('mpc_progress_weight', 1.0)
        self.declare_parameter('formal_mpc_horizon_steps', 8)
        self.declare_parameter('formal_mpc_step_time', 0.25)
        self.declare_parameter('formal_mpc_min_forward_speed', 0.0)
        self.declare_parameter('formal_mpc_slowdown_distance', 3.0)
        self.declare_parameter('formal_mpc_cross_track_weight', 8.0)
        self.declare_parameter('formal_mpc_heading_weight', 4.0)
        self.declare_parameter('formal_mpc_goal_distance_weight', 2.0)
        self.declare_parameter('formal_mpc_terminal_cross_track_weight', 12.0)
        self.declare_parameter('formal_mpc_terminal_heading_weight', 8.0)
        self.declare_parameter('formal_mpc_terminal_goal_distance_weight', 6.0)
        self.declare_parameter('formal_mpc_linear_effort_weight', 0.8)
        self.declare_parameter('formal_mpc_angular_effort_weight', 0.4)
        self.declare_parameter('formal_mpc_linear_smooth_weight', 0.5)
        self.declare_parameter('formal_mpc_angular_smooth_weight', 1.2)
        self.declare_parameter('formal_mpc_progress_weight', 1.0)
        self.declare_parameter('formal_mpc_solver_maxiter', 40)
        self.declare_parameter('formal_mpc_solver_ftol', 0.001)
        self.declare_parameter('row_connector_length_threshold', 3.0)
        self.declare_parameter('row_length_threshold', 15.0)

    def _get_float(self, name):
        return float(self.get_parameter(name).value)

    def _get_bool(self, name):
        return bool(self.get_parameter(name).value)

    def _get_int(self, name):
        return int(self.get_parameter(name).value)

    def _get_str(self, name):
        return str(self.get_parameter(name).value)

    def _terminal_level_value(self, level: str) -> int:
        levels = {
            'silent': 0,
            'normal': 1,
            'verbose': 2,
            'debug': 3,
        }
        return levels.get(level, levels['normal'])

    def _terminal_enabled_for(self, level: str) -> bool:
        if not self.terminal_status_enabled:
            return False
        return self._terminal_level_value(self.terminal_status_level) >= self._terminal_level_value(level)

    def _terminal_info(self, message: str, level: str = 'normal') -> None:
        if self._terminal_enabled_for(level):
            self.get_logger().info(message)

    def _set_phase(self, phase: str, controller_name: str | None = None, detail: str | None = None) -> None:
        if controller_name is not None:
            self.active_controller_name = controller_name

        if phase != self.phase:
            self.last_phase = self.phase
            self.phase = phase
            message = f'Phase change: {self.last_phase} -> {self.phase}'
            if detail:
                message += f' | {detail}'
            self._terminal_info(message, level='normal')
        else:
            self.phase = phase

    def _format_route_summary(self, route) -> str:
        start_x, start_y = route[0]
        goal_x, goal_y = route[1]
        metrics = compute_segment_metrics(route, [start_x, start_y, 0.0])
        return (
            f'segment {self.current_index}->{self.current_index + 1} | '
            f'start=({start_x:.2f}, {start_y:.2f}) | '
            f'goal=({goal_x:.2f}, {goal_y:.2f}) | '
            f'length={metrics.segment_length:.2f} m | '
            f'heading={math.degrees(metrics.path_angle):.2f} deg'
        )

    def _maybe_log_segment_start(self) -> None:
        if self.current_route is None:
            return

        segment_key = (
            self.current_index,
            round(self.current_route[0][0], 3),
            round(self.current_route[0][1], 3),
            round(self.current_route[1][0], 3),
            round(self.current_route[1][1], 3),
        )
        if segment_key == self.last_segment_key:
            return

        self.last_segment_key = segment_key
        self._terminal_info(f'Entering {self._format_route_summary(self.current_route)}', level='normal')

    def _maybe_log_runtime_status(
        self,
        tracking_command: TrackingCommand | None = None,
        turn_command: TurnCommand | None = None,
    ) -> None:
        now = self.get_clock().now()
        if self.last_terminal_status_time is not None:
            age = (now - self.last_terminal_status_time).nanoseconds / 1e9
            if age < self.terminal_status_period_sec:
                return

        if turn_command is not None and self.terminal_show_alignment_details:
            self.last_terminal_status_time = now
            self._terminal_info(
                'Aligning | '
                f'wp={self.current_index}->{self.current_index + 1} | '
                f'heading_error={math.degrees(turn_command.heading_error):.2f} deg | '
                f'cmd_w={turn_command.angular_velocity:.3f} rad/s | '
                f'threshold={math.degrees(self.turn_config.alignment_threshold):.2f} deg',
                level='verbose',
            )
            return

        if tracking_command is not None and self.terminal_show_tracking_details:
            self.last_terminal_status_time = now
            self._terminal_info(
                'Tracking | '
                f'wp={self.current_index}->{self.current_index + 1} | '
                f'heading_error={math.degrees(tracking_command.heading_error):.2f} deg | '
                f'cross_track={tracking_command.cross_track_error:.3f} m | '
                f'dist_to_goal={tracking_command.dist_to_goal:.2f} m | '
                f'cmd_v={tracking_command.linear_velocity:.3f} m/s | '
                f'cmd_w={tracking_command.angular_velocity:.3f} rad/s',
                level='verbose',
            )

    def _log_startup_summary(self) -> None:
        self._terminal_info(
            'Startup summary | '
            f'waypoints={self.csv_path} | '
            f'requested={self.requested_csv_path} | '
            f'resume={self.resume_mode} | '
            f'controller={self.selected_controller_type} | '
            f'gps_points={len(self.waypoints_gps)} | '
            f'terminal_level={self.terminal_status_level}',
            level='normal',
        )
        self._terminal_info(
            'Startup details | '
            f'csv_logging={self.enable_csv_logging} | '
            f'debug_topic={self.publish_debug} | '
            f'log_dir={self.log_directory} | '
            f'max_odom_age={self.max_odom_age_sec:.2f}s',
            level='normal',
        )

    def _log_controller_configuration(self):
        self.get_logger().info(
            "Controller selection: "
            f"requested={self.selected_controller_type}, "
            f"target_speed={self.tracking_config.target_speed:.2f}, "
            f"goal_threshold={self.tracking_config.goal_threshold:.2f}, "
            f"alignment_threshold={self.tracking_config.alignment_threshold:.2f}"
        )
        self.get_logger().info(
            "Controller families: "
            f"pid=({self.tracking_config.pid_kp:.3f}, {self.tracking_config.pid_ki:.3f}, {self.tracking_config.pid_kd:.3f}), "
            f"pure_pursuit_lookahead=[{self.pure_pursuit_config.min_lookahead:.2f}, {self.pure_pursuit_config.max_lookahead:.2f}], "
            f"mpc=(steps={self.mpc_rollout_config.horizon_steps}, dt={self.mpc_rollout_config.step_time:.2f}, candidates={self.mpc_rollout_config.candidate_count}), "
            f"formal_mpc=(steps={self.formal_mpc_config.horizon_steps}, dt={self.formal_mpc_config.step_time:.2f}, maxiter={self.formal_mpc_config.solver_maxiter}), "
            f"row_hybrid=(connector<={self.row_hybrid_config.connector_length_threshold:.2f}, row>={self.row_hybrid_config.row_length_threshold:.2f})"
        )

    def _select_navigation_file(self):
        resume_info = self._get_unfinished_navigation_info()
        if resume_info is None:
            return self.requested_csv_path, 0

        resume_index, remaining_points = resume_info
        if self.resume_mode == 'yes':
            self.get_logger().info(
                f'Automatically resuming unfinished navigation from {self.last_wp_path} at waypoint index {resume_index}.'
            )
            return self.last_wp_path, resume_index

        if self.resume_mode == 'no':
            self.get_logger().info('Ignoring unfinished navigation and starting from the requested waypoint file.')
            return self.requested_csv_path, 0

        if sys.stdin.isatty():
            prompt = (
                f"Detected unfinished navigation in {self.last_wp_path} "
                f"(resume from waypoint index {resume_index}, {remaining_points} points remaining). "
                "Continue it? [y/N]: "
            )
            answer = input(prompt).strip().lower()
            if answer in ('y', 'yes'):
                self.get_logger().info(
                    f'Resuming unfinished navigation from {self.last_wp_path} at waypoint index {resume_index}.'
                )
                return self.last_wp_path, resume_index

        self.get_logger().info('Starting from the requested waypoint file instead of the unfinished navigation snapshot.')
        return self.requested_csv_path, 0

    def _get_unfinished_navigation_info(self):
        if not self.last_wp_path.exists() or not self.status_path.exists():
            return None

        try:
            saved_index = int(self.status_path.read_text().splitlines()[0].strip())
            last_waypoints = self.load_csv_waypoints(self.last_wp_path)
        except Exception as exc:
            self.get_logger().warn(f'Could not inspect unfinished navigation state: {exc}')
            return None

        if len(last_waypoints) < 2:
            return None

        if 0 <= saved_index < len(last_waypoints) - 1:
            remaining_points = len(last_waypoints) - saved_index - 1
            return saved_index, remaining_points

        return None

    def _prepare_waypoint_state(self):
        if self.csv_path != self.last_wp_path:
            shutil.copy(self.csv_path, self.last_wp_path)

        if self.current_index > 0:
            self.get_logger().info(f'Resuming from waypoint index {self.current_index}')
        else:
            self.get_logger().info(f'Starting navigation using waypoint file: {self.csv_path}')

        self.update_status_file()

    def _setup_log_file(self):
        self.csv_log_file = None
        self.csv_log_writer = None
        if not self.enable_csv_logging:
            return

        timestamp = int(time.time())
        csv_log_path = self.log_directory / f'waypoint_control_log_{timestamp}.csv'
        self.csv_log_file = csv_log_path.open('w', newline='')
        self.csv_log_writer = csv.writer(self.csv_log_file)
        self.csv_log_writer.writerow([
            'timestamp',
            'phase',
            'controller',
            'waypoint_index',
            'segment_start_x',
            'segment_start_y',
            'x',
            'y',
            'yaw_deg',
            'path_heading_deg',
            'segment_length_m',
            'target_x',
            'target_y',
            'heading_error_deg',
            'cross_track_error_m',
            'dist_to_goal_m',
            'dist_from_start_m',
            'target_speed_mps',
            'lateral_speed_mps',
            'cmd_v',
            'cmd_w',
        ])
        self.get_logger().info(f'Control CSV logging enabled: {csv_log_path}')

    def load_csv_waypoints(self, path: Path):
        waypoints = []
        with path.open('r', newline='') as waypoint_file:
            reader = csv.reader(waypoint_file)
            next(reader)
            for row in reader:
                if len(row) >= 2:
                    waypoints.append((float(row[0]), float(row[1])))
        return waypoints

    def datum_callback(self, msg):
        if self.transformer is not None:
            return

        lat0, lon0, _ = msg.data
        self.transformer = Transformer.from_crs(
            'epsg:4326',
            f'+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +datum=WGS84',
            always_xy=True
        )
        self.waypoints_enu = []
        for lat, lon in self.waypoints_gps:
            x, y = self.transformer.transform(lon, lat)
            self.waypoints_enu.append([x, y])
        self.get_logger().info(f'Converted {len(self.waypoints_enu)} waypoints to ENU coordinates.')

        if self.pose is not None and not self.init_pose_inserted:
            self._insert_initial_pose()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        pose_values = (pos.x, pos.y, ori.x, ori.y, ori.z, ori.w)
        quaternion_norm = math.sqrt(
            ori.x * ori.x + ori.y * ori.y + ori.z * ori.z + ori.w * ori.w
        )
        if (
            not all(math.isfinite(value) for value in pose_values)
            or quaternion_norm < 1e-6
        ):
            now = self.get_clock().now()
            if (
                self.last_invalid_odom_warn_time is None
                or (now - self.last_invalid_odom_warn_time).nanoseconds > int(2e9)
            ):
                self.get_logger().error(
                    'Rejected /robot/odom containing non-finite values or an invalid quaternion.'
                )
                self.last_invalid_odom_warn_time = now
            return
        yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.pose = [pos.x, pos.y, yaw]
        self.last_odom_time = self.get_clock().now()

        if self.transformer is not None and not self.init_pose_inserted:
            self._insert_initial_pose()

    def _insert_initial_pose(self):
        self.waypoints_enu.insert(0, [self.pose[0], self.pose[1]])
        self.init_pose_inserted = True
        self.get_logger().info(f'Inserted current position as waypoint 0: {[self.pose[0], self.pose[1]]}')
        self._terminal_info(
            f'Navigation start pose inserted as waypoint 0 at ({self.pose[0]:.2f}, {self.pose[1]:.2f}).',
            level='normal',
        )

    def control_loop(self):
        if self.reached_final:
            self._publish_stop()
            return

        if self.safety_fault_reason is not None:
            self._set_phase('safety_stop', controller_name='safety_watchdog')
            self._publish_stop()
            self._publish_high_priority_stop()
            self._publish_debug({
                'reason': 'safety_stop',
                'message': self.safety_fault_reason,
                **self._build_debug_snapshot(error_message=self.safety_fault_reason),
            })
            return

        if self.pose is None or self.transformer is None or not self.init_pose_inserted:
            self._set_phase('waiting_for_pose', controller_name='waiting_for_pose')
            self._publish_stop()
            return

        if not self._odom_is_fresh():
            self._set_phase('stale_pose', controller_name='stale_pose', detail='Waiting for fresh /robot/odom.')
            self._publish_stop()
            self._publish_debug({'reason': 'stale_pose', **self._build_debug_snapshot()})
            return

        if self.current_index >= len(self.waypoints_enu) - 1:
            self._set_phase('complete', controller_name='complete')
            self.reached_final = True
            self._publish_stop()
            self.update_status_file()
            self.get_logger().info('Navigation complete.')
            self._terminal_info(
                f'Navigation complete | reached final waypoint index {self.current_index}',
                level='normal',
            )
            self._publish_debug({'reason': 'complete', **self._build_debug_snapshot()})
            return

        self.current_route = [
            self.waypoints_enu[self.current_index],
            self.waypoints_enu[self.current_index + 1],
        ]
        self._maybe_log_segment_start()
        segment_metrics = compute_segment_metrics(self.current_route, self.pose)
        if segment_metrics.segment_length < self.min_segment_length_m:
            self.get_logger().warning(
                f'Skipping degenerate waypoint segment {self.current_index} -> '
                f'{self.current_index + 1} (length={segment_metrics.segment_length:.3f} m).'
            )
            self._advance_waypoint('degenerate_segment')
            self._publish_stop()
            return

        # Do not rotate toward a waypoint that has already been reached or
        # passed. This also avoids an arbitrary zero turn on duplicate points.
        if is_goal_reached(self.current_route, self.pose, self.tracking_config.goal_threshold):
            self._advance_waypoint('already_reached')
            self._publish_stop()
            return

        if not self.segment_aligned:
            turn_command = compute_turn_command(self.current_route, self.pose, self.turn_config)
            if not turn_command.aligned:
                self._set_phase(
                    'aligning',
                    controller_name='alignment_turn',
                    detail=f'Preparing segment {self.current_index}->{self.current_index + 1}',
                )
                if self.progress_watchdog_enabled:
                    fault = self.progress_watchdog.check_alignment(
                        self.current_index,
                        time.monotonic(),
                        self.pose,
                        turn_command.heading_error,
                        turn_command.angular_velocity,
                    )
                    if fault is not None:
                        self._latch_safety_fault(fault)
                        self._publish_stop()
                        return
                self._publish_twist(0.0, turn_command.angular_velocity)
                self._publish_debug(self._build_debug_snapshot(turn_command=turn_command))
                self._log_control_sample(turn_command=turn_command)
                self._maybe_log_runtime_status(turn_command=turn_command)
                return

            self.segment_aligned = True
            self.controller.reset()
            self._set_phase('tracking', controller_name=self.selected_controller_type)
            self.get_logger().info(f'Alignment complete for waypoint segment {self.current_index} -> {self.current_index + 1}')
            self._terminal_info(
                'Alignment complete | '
                f'wp={self.current_index}->{self.current_index + 1} | '
                f'heading_error={math.degrees(turn_command.heading_error):.2f} deg',
                level='normal',
            )

        try:
            tracking_command = self.controller.compute_command(self.current_route, self.pose)
            self.active_controller_name = getattr(self.controller, 'active_controller_name', self.selected_controller_type)
        except Exception as exc:
            self._latch_safety_fault(f'controller error: {exc}')
            self._publish_stop()
            self.get_logger().error(str(exc))
            self.get_logger().warn(
                'Control error triggered a stop | '
                f'wp={self.current_index}->{self.current_index + 1} | '
                f'pose=({self.pose[0]:.2f}, {self.pose[1]:.2f}, {math.degrees(self.pose[2]):.2f} deg)'
            )
            self._publish_debug({
                'reason': 'control_error',
                'message': str(exc),
                **self._build_debug_snapshot(error_message=str(exc)),
            })
            self._log_control_sample(error_message=str(exc))
            return

        if self.progress_watchdog_enabled:
            fault = self.progress_watchdog.check_tracking(
                self.current_index,
                time.monotonic(),
                self.pose,
                tracking_command.dist_to_goal,
                tracking_command.linear_velocity,
                tracking_command.angular_velocity,
            )
            if fault is not None:
                self._latch_safety_fault(fault)
                self._publish_stop()
                return

        self._publish_twist(tracking_command.linear_velocity, tracking_command.angular_velocity)
        self._publish_debug(self._build_debug_snapshot(tracking_command=tracking_command))
        self._log_control_sample(tracking_command=tracking_command)
        self._maybe_log_runtime_status(tracking_command=tracking_command)

    def _odom_is_fresh(self):
        if self.last_odom_time is None:
            return False
        odom_age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if odom_age <= self.max_odom_age_sec:
            return True

        if self.last_odom_warn_time is None or (self.get_clock().now() - self.last_odom_warn_time).nanoseconds > int(2e9):
            self.get_logger().warn(f'/robot/odom is stale ({odom_age:.2f}s); publishing stop until pose updates resume.')
            self._terminal_info(
                f'Safety stop | stale /robot/odom for {odom_age:.2f}s (limit {self.max_odom_age_sec:.2f}s)',
                level='normal',
            )
            self.last_odom_warn_time = self.get_clock().now()
        return False

    def _advance_waypoint(self, reason):
        reached_index = self.current_index + 1
        self.get_logger().info(f'Reached waypoint {reached_index} ({reason})')
        self._terminal_info(
            'Reached waypoint | '
            f'wp={reached_index} | '
            f'pose=({self.pose[0]:.2f}, {self.pose[1]:.2f}) | '
            f'reason={reason}',
            level='normal',
        )
        self.current_index = reached_index
        self.segment_aligned = False
        self._set_phase(
            'aligning', controller_name='alignment_turn', detail='Switching to next segment.'
        )
        self.controller.reset()
        self.update_status_file()

    def _latch_safety_fault(self, reason):
        if self.safety_fault_reason is not None:
            return
        self.safety_fault_reason = str(reason)
        self._set_phase('safety_stop', controller_name='safety_watchdog')
        self._publish_high_priority_stop()
        self.get_logger().error(
            'NAVIGATION SAFETY STOP (latched): '
            f'{self.safety_fault_reason}. Inspect localization/command topics and restart '
            'waypoint_follower only after the cause is understood.'
        )
        self._log_control_sample(error_message=self.safety_fault_reason)
        self.flush_log_file()

    def _publish_twist(self, linear_x, angular_z):
        reason = validate_velocity_command(
            float(linear_x),
            float(angular_z),
            self.command_max_linear_speed,
            self.command_max_angular_speed,
        )
        if reason is not None:
            self._latch_safety_fault(reason)
            linear_x = 0.0
            angular_z = 0.0
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

    def _publish_stop(self):
        self._publish_twist(0.0, 0.0)

    def _publish_high_priority_stop(self):
        """Override every autonomous twist_mux input after a latched fault."""
        self.safety_stop_pub.publish(Twist())

    def _publish_debug(self, extra_fields):
        if not self.publish_debug:
            return

        now = self.get_clock().now()
        if self.last_debug_publish_time is not None:
            age = (now - self.last_debug_publish_time).nanoseconds / 1e9
            if age < self.debug_publish_period_sec:
                return
        self.last_debug_publish_time = now

        payload = {
            'phase': self.phase,
            'controller': self.active_controller_name,
            'waypoint_index': self.current_index,
            'pose': None if self.pose is None else {
                'x': round(self.pose[0], 3),
                'y': round(self.pose[1], 3),
                'yaw_deg': round(math.degrees(self.pose[2]), 2),
            },
            'route': None if self.current_route is None else {
                'start': [round(value, 3) for value in self.current_route[0]],
                'goal': [round(value, 3) for value in self.current_route[1]],
            },
        }
        for key, value in extra_fields.items():
            if isinstance(value, float):
                payload[key] = round(value, 4)
            else:
                payload[key] = value

        self.debug_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _build_debug_snapshot(
        self,
        tracking_command: TrackingCommand | None = None,
        turn_command: TurnCommand | None = None,
        error_message: str | None = None,
    ) -> dict:
        payload = {
            'phase': self.phase,
            'controller': self.active_controller_name if error_message is None else f'{self.active_controller_name}:{error_message}',
            'waypoint_index': self.current_index,
        }

        if self.pose is not None:
            payload.update({
                'x': round(self.pose[0], 3),
                'y': round(self.pose[1], 3),
                'yaw_deg': round(math.degrees(self.pose[2]), 2),
            })

        if self.current_route is not None:
            metrics = compute_segment_metrics(self.current_route, self.pose if self.pose is not None else [0.0, 0.0, 0.0])
            start_x, start_y = self.current_route[0]
            target_x, target_y = self.current_route[1]
            payload.update({
                'segment_start_x': round(start_x, 3),
                'segment_start_y': round(start_y, 3),
                'path_heading_deg': round(math.degrees(metrics.path_angle), 2),
                'segment_length_m': round(metrics.segment_length, 3),
                'target_x': round(target_x, 3),
                'target_y': round(target_y, 3),
            })

        heading_error = None
        cross_track_error = None
        dist_to_goal = None
        dist_from_start = None
        target_speed = 0.0
        lateral_speed = 0.0
        cmd_v = 0.0
        cmd_w = 0.0

        if self.current_route is not None and self.pose is not None:
            metrics = compute_segment_metrics(self.current_route, self.pose)
            heading_error = metrics.heading_error
            cross_track_error = metrics.cross_track_error
            dist_to_goal = metrics.dist_to_goal
            dist_from_start = metrics.dist_from_start

        if tracking_command is not None:
            heading_error = tracking_command.heading_error
            cross_track_error = tracking_command.cross_track_error
            dist_to_goal = tracking_command.dist_to_goal
            dist_from_start = tracking_command.dist_from_start
            target_speed = tracking_command.target_speed
            lateral_speed = tracking_command.lateral_speed
            cmd_v = tracking_command.linear_velocity
            cmd_w = tracking_command.angular_velocity

        if turn_command is not None:
            heading_error = turn_command.heading_error
            cmd_w = turn_command.angular_velocity

        if heading_error is not None:
            payload['heading_error_deg'] = round(math.degrees(heading_error), 2)
        if cross_track_error is not None:
            payload['cross_track_error_m'] = round(cross_track_error, 3)
        if dist_to_goal is not None:
            payload['dist_to_goal_m'] = round(dist_to_goal, 3)
        if dist_from_start is not None:
            payload['dist_from_start_m'] = round(dist_from_start, 3)

        payload.update({
            'target_speed_mps': round(target_speed, 3),
            'lateral_speed_mps': round(lateral_speed, 3),
            'cmd_v': round(cmd_v, 3),
            'cmd_w': round(cmd_w, 3),
        })

        return payload

    def _log_control_sample(
        self,
        tracking_command: TrackingCommand | None = None,
        turn_command: TurnCommand | None = None,
        error_message: str | None = None,
    ):
        if self.csv_log_writer is None or self.current_route is None or self.pose is None:
            return
        snapshot = self._build_debug_snapshot(
            tracking_command=tracking_command,
            turn_command=turn_command,
            error_message=error_message,
        )

        self.csv_log_writer.writerow([
            round(self.get_clock().now().nanoseconds / 1e9, 3),
            snapshot['phase'],
            snapshot['controller'],
            snapshot['waypoint_index'],
            snapshot.get('segment_start_x', 0.0),
            snapshot.get('segment_start_y', 0.0),
            snapshot.get('x', 0.0),
            snapshot.get('y', 0.0),
            snapshot.get('yaw_deg', 0.0),
            snapshot.get('path_heading_deg', 0.0),
            snapshot.get('segment_length_m', 0.0),
            snapshot.get('target_x', 0.0),
            snapshot.get('target_y', 0.0),
            snapshot.get('heading_error_deg', 0.0),
            snapshot.get('cross_track_error_m', 0.0),
            snapshot.get('dist_to_goal_m', 0.0),
            snapshot.get('dist_from_start_m', 0.0),
            snapshot.get('target_speed_mps', 0.0),
            snapshot.get('lateral_speed_mps', 0.0),
            snapshot.get('cmd_v', 0.0),
            snapshot.get('cmd_w', 0.0),
        ])

    def flush_log_file(self):
        if self.csv_log_file is not None:
            self.csv_log_file.flush()

    def update_status_file(self):
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        self.status_path.write_text(f'{self.current_index}\n')

    def destroy_node(self):
        try:
            self._publish_stop()
        except Exception:
            pass
        if self.csv_log_file is not None:
            self.csv_log_file.flush()
            self.csv_log_file.close()
        super().destroy_node()

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)


def main(args=None):
    raw_args = list(args) if args is not None else list(sys.argv)
    parser = ArgumentParser()
    parser.add_argument(
        '--waypoints',
        default=DEFAULT_WAYPOINTS_PATH,
        help='Path to CSV file containing latitude,longitude waypoints'
    )
    parser.add_argument(
        '--resume',
        choices=['ask', 'yes', 'no'],
        default='ask',
        help='Whether to resume an unfinished navigation snapshot from last_waypoints.csv'
    )
    parser.add_argument(
        '--controller',
        choices=CONTROLLER_CHOICES,
        help='Force the controller type for this run. This overrides the parameter file.'
    )
    parsed_args, ros_args = parser.parse_known_args(raw_args[1:])

    if '--params-file' not in ros_args:
        for candidate in DEFAULT_PARAMS_FILE_CANDIDATES:
            if candidate.exists():
                ros_args = ['--ros-args', '--params-file', str(candidate), *ros_args]
                break

    rclpy.init(args=ros_args)

    node = WaypointFollower(
        parsed_args.waypoints,
        resume_mode=parsed_args.resume,
        controller_override=parsed_args.controller,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
