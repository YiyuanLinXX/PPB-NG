#!/usr/bin/env python3
"""Pure pursuit controller implementation."""

from dataclasses import dataclass

import numpy as np

from amiga_navigation.utils.tracking_geometry import (
    Point3D,
    Route,
    TrackingCommand,
    apply_speed_schedule,
    compute_segment_metrics,
    normalize_angle,
)


@dataclass(frozen=True)
class PurePursuitConfig:
    target_speed: float = 0.8
    min_lookahead: float = 1.5
    max_lookahead: float = 6.0
    lookahead_gain: float = 2.5
    slowdown_distance: float = 4.0
    max_angular_speed: float = 0.8
    min_forward_ratio: float = 0.2
    start_slowdown_distance: float = 0.0
    initial_speed_ratio: float = 0.3
    enable_start_slowdown: bool = False
    enable_goal_slowdown: bool = True
    max_cross_track_error: float = 3.0
    goal_threshold: float = 0.30


class PurePursuitController:
    controller_name = 'pure_pursuit'

    def __init__(self, config: PurePursuitConfig):
        self.config = config
        self.active_controller_name = self.controller_name

    def reset(self) -> None:
        self.active_controller_name = self.controller_name

    def compute_command(self, route: Route, pose: Point3D) -> TrackingCommand:
        self.active_controller_name = self.controller_name
        metrics = compute_segment_metrics(route, pose)
        if abs(metrics.cross_track_error) > self.config.max_cross_track_error:
            raise ValueError(
                f'Cross-track error {metrics.cross_track_error:.2f} exceeds limit {self.config.max_cross_track_error:.2f}'
            )

        pt1 = np.array(route[0], dtype=float)
        pt2 = np.array(route[1], dtype=float)
        position = np.array(pose[0:2], dtype=float)
        theta = float(pose[2])

        if metrics.segment_length <= 1e-9:
            return TrackingCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
                cross_track_error=metrics.cross_track_error,
                heading_error=metrics.heading_error,
                dist_to_goal=metrics.dist_to_goal,
                dist_from_start=metrics.dist_from_start,
                target_speed=0.0,
                lateral_speed=0.0,
            )

        segment_direction = (pt2 - pt1) / metrics.segment_length
        lookahead_distance = np.clip(
            self.config.target_speed * self.config.lookahead_gain,
            self.config.min_lookahead,
            self.config.max_lookahead,
        )
        lookahead_progress = min(metrics.segment_length, metrics.dist_from_start + lookahead_distance)
        lookahead_point = pt1 + segment_direction * lookahead_progress
        lookahead_vector = lookahead_point - position
        actual_lookahead = float(max(np.linalg.norm(lookahead_vector), 1e-6))
        angle_to_target = float(np.arctan2(lookahead_vector[1], lookahead_vector[0]))
        alpha = normalize_angle(angle_to_target - theta)

        linear_velocity = apply_speed_schedule(
            self.config.target_speed,
            metrics.dist_to_goal,
            self.config.slowdown_distance,
            self.config.min_forward_ratio,
            alpha,
            enable_goal_slowdown=self.config.enable_goal_slowdown,
            dist_from_start=metrics.dist_from_start,
            start_slowdown_distance=self.config.start_slowdown_distance,
            initial_speed_ratio=self.config.initial_speed_ratio,
            enable_start_slowdown=self.config.enable_start_slowdown,
        )
        curvature = 2.0 * np.sin(alpha) / actual_lookahead
        angular_velocity = float(np.clip(linear_velocity * curvature, -self.config.max_angular_speed, self.config.max_angular_speed))

        return TrackingCommand(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            cross_track_error=metrics.cross_track_error,
            heading_error=alpha,
            dist_to_goal=metrics.dist_to_goal,
            dist_from_start=metrics.dist_from_start,
            target_speed=linear_velocity,
            lateral_speed=curvature,
        )
