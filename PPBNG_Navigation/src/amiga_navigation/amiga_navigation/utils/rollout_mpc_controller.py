#!/usr/bin/env python3
"""Sampling-based rollout MPC controller."""

from dataclasses import dataclass

import numpy as np

from amiga_navigation.utils.tracking_geometry import (
    Point3D,
    Route,
    SegmentMetrics,
    TrackingCommand,
    apply_speed_schedule,
    compute_segment_metrics,
    normalize_angle,
)


@dataclass(frozen=True)
class MPCRolloutConfig:
    target_speed: float = 0.8
    horizon_steps: int = 10
    step_time: float = 0.2
    candidate_count: int = 15
    max_angular_speed: float = 0.8
    min_forward_ratio: float = 0.2
    slowdown_distance: float = 3.0
    heading_weight: float = 1.5
    cross_track_weight: float = 2.0
    goal_distance_weight: float = 4.0
    effort_weight: float = 0.3
    progress_weight: float = 1.0
    start_slowdown_distance: float = 0.0
    initial_speed_ratio: float = 0.3
    enable_start_slowdown: bool = False
    enable_goal_slowdown: bool = True
    max_cross_track_error: float = 3.0
    goal_threshold: float = 0.30


class MPCRolloutController:
    controller_name = 'mpc_rollout'

    def __init__(self, config: MPCRolloutConfig):
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

        base_speed = apply_speed_schedule(
            self.config.target_speed,
            metrics.dist_to_goal,
            self.config.slowdown_distance,
            self.config.min_forward_ratio,
            metrics.heading_error,
            enable_goal_slowdown=self.config.enable_goal_slowdown,
            dist_from_start=metrics.dist_from_start,
            start_slowdown_distance=self.config.start_slowdown_distance,
            initial_speed_ratio=self.config.initial_speed_ratio,
            enable_start_slowdown=self.config.enable_start_slowdown,
        )
        speed_candidates = [base_speed * scale for scale in (0.35, 0.6, 1.0)]
        angular_candidates = np.linspace(
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
            max(self.config.candidate_count, 3),
        )

        best_cost = None
        best_command = (0.0, 0.0)
        best_terminal = metrics

        for linear_velocity in speed_candidates:
            for angular_velocity in angular_candidates:
                cost, terminal_metrics = self._evaluate_candidate(route, pose, linear_velocity, float(angular_velocity))
                if best_cost is None or cost < best_cost:
                    best_cost = cost
                    best_command = (float(linear_velocity), float(angular_velocity))
                    best_terminal = terminal_metrics

        return TrackingCommand(
            linear_velocity=best_command[0],
            angular_velocity=best_command[1],
            cross_track_error=metrics.cross_track_error,
            heading_error=metrics.heading_error,
            dist_to_goal=metrics.dist_to_goal,
            dist_from_start=metrics.dist_from_start,
            target_speed=best_command[0],
            lateral_speed=best_terminal.cross_track_error,
        )

    def _evaluate_candidate(
        self,
        route: Route,
        pose: Point3D,
        linear_velocity: float,
        angular_velocity: float,
    ) -> tuple[float, SegmentMetrics]:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        cumulative_cost = 0.0
        terminal_metrics = compute_segment_metrics(route, pose)

        for _ in range(max(self.config.horizon_steps, 1)):
            x += linear_velocity * np.cos(yaw) * self.config.step_time
            y += linear_velocity * np.sin(yaw) * self.config.step_time
            yaw = normalize_angle(yaw + angular_velocity * self.config.step_time)
            terminal_metrics = compute_segment_metrics(route, [x, y, yaw])

            cumulative_cost += self.config.cross_track_weight * abs(terminal_metrics.cross_track_error)
            cumulative_cost += self.config.heading_weight * abs(terminal_metrics.heading_error)
            cumulative_cost += self.config.effort_weight * abs(angular_velocity)

        cumulative_cost += self.config.goal_distance_weight * terminal_metrics.dist_to_goal
        cumulative_cost -= self.config.progress_weight * terminal_metrics.progress
        return cumulative_cost, terminal_metrics
