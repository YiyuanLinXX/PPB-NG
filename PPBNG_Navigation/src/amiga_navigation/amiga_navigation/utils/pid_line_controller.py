#!/usr/bin/env python3
"""PID-based line tracking controller."""

from dataclasses import dataclass

import numpy as np
from simple_pid import PID

from amiga_navigation.utils.tracking_geometry import (
    Point3D,
    Route,
    TrackingCommand,
    compute_path_angle,
    get_cross_track_error,
    get_projection_point,
    normalize_angle,
)


@dataclass(frozen=True)
class LineTrackingConfig:
    target_speed: float = 0.8
    max_lateral_speed: float = 0.5
    epsilon: float = 0.5
    pid_kp: float = 0.4
    pid_ki: float = 0.05
    pid_kd: float = 0.4
    heading_gain: float = 1.2
    max_angular_speed: float = 0.8
    min_forward_ratio: float = 0.2
    max_heading_for_full_speed: float = 0.7
    max_cross_track_error: float = 0.5
    goal_threshold: float = 0.30
    alignment_threshold: float = 0.10
    dist_start_threshold: float = 1.0
    dist_stop_threshold: float = 1.0
    initial_speed_ratio: float = 0.3
    stop_speed_ratio: float = 0.0
    enable_start_slowdown: bool = True
    enable_goal_slowdown: bool = True
    regulate_target_speed: bool = False


def differential_drive(v_x: float, v_y: float, theta: float, epsilon: float) -> tuple[float, float]:
    """Convert world-frame velocity to differential-drive v/w commands."""
    v = v_x * np.cos(theta) + v_y * np.sin(theta)
    w = 1.0 / epsilon * (-v_x * np.sin(theta) + v_y * np.cos(theta))
    return float(v), float(w)


class LineTrackingController:
    """Line tracking controller with centralized, tunable configuration."""

    controller_name = 'pid_line'

    def __init__(self, config: LineTrackingConfig | None = None):
        self.config = config or LineTrackingConfig()
        self.active_controller_name = self.controller_name
        self.pid = PID(
            self.config.pid_kp,
            self.config.pid_ki,
            self.config.pid_kd,
            setpoint=0.0,
        )
        self.pid.sample_time = None
        self.pid.output_limits = (-self.config.max_lateral_speed, self.config.max_lateral_speed)
        self.start_speed = self.config.target_speed * self.config.initial_speed_ratio
        self.stop_speed = self.config.target_speed * self.config.stop_speed_ratio

    def reset(self) -> None:
        self.pid.reset()
        self.active_controller_name = self.controller_name

    def compute_command(self, route: Route, pose: Point3D, adjust_target_speed: float | None = None) -> TrackingCommand:
        self.active_controller_name = self.controller_name
        pt1 = np.array(route[0], dtype=float)
        pt2 = np.array(route[1], dtype=float)
        theta = float(pose[2])

        pt4 = np.array(get_projection_point(route[0], route[1], pose[0:2]), dtype=float)
        path_vector = pt2 - pt1
        path_angle = compute_path_angle(route)
        heading_error = normalize_angle(path_angle - theta)
        cross_track_error = get_cross_track_error(route[0], route[1], pose[0:2], pt4.tolist())
        dist_to_goal = float(np.linalg.norm(pt2 - pt4))
        dist_from_start = float(np.linalg.norm(pt4 - pt1))

        if abs(cross_track_error) > self.config.max_cross_track_error:
            raise ValueError(
                f"Cross-track error {cross_track_error:.2f} exceeds limit {self.config.max_cross_track_error:.2f}"
            )

        lateral_speed = float(self.pid(cross_track_error))
        target_speed = self.config.target_speed if adjust_target_speed is None else adjust_target_speed

        if self.config.regulate_target_speed:
            target_speed = max(abs(lateral_speed) + 1e-3, target_speed)
            target_speed = float(np.sqrt(max(target_speed**2 - lateral_speed**2, 1e-6)))

        if (
            self.config.enable_start_slowdown
            and self.config.dist_start_threshold > 1e-6
            and dist_from_start < self.config.dist_start_threshold
        ):
            start_ratio = dist_from_start / self.config.dist_start_threshold
            target_speed = min(
                target_speed,
                self.start_speed + (self.config.target_speed - self.start_speed) * start_ratio
            )

        if (
            self.config.enable_goal_slowdown
            and self.config.dist_stop_threshold > 1e-6
            and dist_to_goal < self.config.dist_stop_threshold
        ):
            stop_ratio = (self.config.dist_stop_threshold - dist_to_goal) / self.config.dist_stop_threshold
            target_speed = min(
                target_speed,
                self.config.target_speed - (self.config.target_speed - self.stop_speed) * stop_ratio
            )

        heading_ratio = 1.0 - min(abs(heading_error) / self.config.max_heading_for_full_speed, 1.0)
        error_ratio = 1.0 - min(abs(cross_track_error) / self.config.max_cross_track_error, 1.0)
        target_speed *= max(self.config.min_forward_ratio, min(heading_ratio, error_ratio))

        rotation = np.array([
            [np.cos(path_angle), -np.sin(path_angle)],
            [np.sin(path_angle), np.cos(path_angle)],
        ])
        v_x, v_y = np.dot(rotation, [target_speed, lateral_speed])
        linear_velocity, angular_velocity = differential_drive(v_x, v_y, theta, self.config.epsilon)
        angular_velocity += self.config.heading_gain * heading_error
        angular_velocity = float(np.clip(angular_velocity, -self.config.max_angular_speed, self.config.max_angular_speed))

        return TrackingCommand(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            cross_track_error=cross_track_error,
            heading_error=heading_error,
            dist_to_goal=dist_to_goal,
            dist_from_start=dist_from_start,
            target_speed=target_speed,
            lateral_speed=lateral_speed,
        )
