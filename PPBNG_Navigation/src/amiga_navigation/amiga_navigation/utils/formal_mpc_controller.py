#!/usr/bin/env python3
"""Formal receding-horizon MPC controller."""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from amiga_navigation.utils.tracking_geometry import (
    Point3D,
    Route,
    TrackingCommand,
    apply_speed_schedule,
    compute_segment_metrics,
    normalize_angle,
)


@dataclass(frozen=True)
class FormalMPCConfig:
    target_speed: float = 0.8
    horizon_steps: int = 8
    step_time: float = 0.25
    min_forward_speed: float = 0.0
    max_angular_speed: float = 0.8
    slowdown_distance: float = 3.0
    min_forward_ratio: float = 0.2
    cross_track_weight: float = 8.0
    heading_weight: float = 4.0
    goal_distance_weight: float = 2.0
    terminal_cross_track_weight: float = 12.0
    terminal_heading_weight: float = 8.0
    terminal_goal_distance_weight: float = 6.0
    linear_effort_weight: float = 0.8
    angular_effort_weight: float = 0.4
    linear_smooth_weight: float = 0.5
    angular_smooth_weight: float = 1.2
    progress_weight: float = 1.0
    solver_maxiter: int = 40
    solver_ftol: float = 1e-3
    start_slowdown_distance: float = 0.0
    initial_speed_ratio: float = 0.3
    enable_start_slowdown: bool = False
    enable_goal_slowdown: bool = True
    max_cross_track_error: float = 3.0
    goal_threshold: float = 0.30


class FormalMPCController:
    controller_name = 'mpc_formal'

    def __init__(self, config: FormalMPCConfig):
        self.config = config
        self.active_controller_name = self.controller_name
        self._warm_start = None

    def reset(self) -> None:
        self.active_controller_name = self.controller_name
        self._warm_start = None

    def compute_command(self, route: Route, pose: Point3D) -> TrackingCommand:
        self.active_controller_name = self.controller_name
        metrics = compute_segment_metrics(route, pose)
        if abs(metrics.cross_track_error) > self.config.max_cross_track_error:
            raise ValueError(
                f'Cross-track error {metrics.cross_track_error:.2f} exceeds limit {self.config.max_cross_track_error:.2f}'
            )

        desired_speed = apply_speed_schedule(
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
        control_sequence = self._solve_optimal_sequence(route, pose, desired_speed)
        linear_velocity = float(control_sequence[0])
        angular_velocity = float(control_sequence[1])

        next_pose = self._step_dynamics(pose, linear_velocity, angular_velocity)
        predicted_metrics = compute_segment_metrics(route, next_pose)

        return TrackingCommand(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            cross_track_error=metrics.cross_track_error,
            heading_error=metrics.heading_error,
            dist_to_goal=metrics.dist_to_goal,
            dist_from_start=metrics.dist_from_start,
            target_speed=desired_speed,
            lateral_speed=predicted_metrics.cross_track_error,
        )

    def _solve_optimal_sequence(self, route: Route, pose: Point3D, desired_speed: float) -> np.ndarray:
        horizon = max(self.config.horizon_steps, 1)
        if self._warm_start is None or len(self._warm_start) != 2 * horizon:
            initial_v = np.clip(desired_speed, self.config.min_forward_speed, self.config.target_speed)
            self._warm_start = np.tile(np.array([initial_v, 0.0], dtype=float), horizon)

        bounds = []
        for _ in range(horizon):
            bounds.append((self.config.min_forward_speed, self.config.target_speed))
            bounds.append((-self.config.max_angular_speed, self.config.max_angular_speed))

        result = minimize(
            fun=self._objective,
            x0=self._warm_start,
            args=(route, pose, desired_speed),
            method='SLSQP',
            bounds=bounds,
            options={
                'maxiter': max(self.config.solver_maxiter, 1),
                'ftol': self.config.solver_ftol,
                'disp': False,
            },
        )

        if result.success:
            optimum = np.asarray(result.x, dtype=float)
        else:
            optimum = np.asarray(self._warm_start, dtype=float)

        shifted = np.copy(optimum)
        if len(shifted) >= 4:
            shifted[:-2] = shifted[2:]
            shifted[-2:] = shifted[-4:-2]
        self._warm_start = shifted
        return optimum

    def _objective(self, u_flat: np.ndarray, route: Route, pose: Point3D, desired_speed: float) -> float:
        state = np.array([float(pose[0]), float(pose[1]), float(pose[2])], dtype=float)
        total_cost = 0.0
        prev_v = None
        prev_w = None

        for step in range(max(self.config.horizon_steps, 1)):
            linear_velocity = float(u_flat[2 * step])
            angular_velocity = float(u_flat[2 * step + 1])
            state = np.array(self._step_dynamics(state.tolist(), linear_velocity, angular_velocity), dtype=float)
            metrics = compute_segment_metrics(route, state.tolist())

            normalized_goal = metrics.dist_to_goal / max(metrics.segment_length, 1.0)
            total_cost += self.config.cross_track_weight * metrics.cross_track_error ** 2
            total_cost += self.config.heading_weight * metrics.heading_error ** 2
            total_cost += self.config.goal_distance_weight * normalized_goal ** 2
            total_cost += self.config.linear_effort_weight * (linear_velocity - desired_speed) ** 2
            total_cost += self.config.angular_effort_weight * angular_velocity ** 2
            total_cost -= self.config.progress_weight * metrics.progress

            if prev_v is not None and prev_w is not None:
                total_cost += self.config.linear_smooth_weight * (linear_velocity - prev_v) ** 2
                total_cost += self.config.angular_smooth_weight * (angular_velocity - prev_w) ** 2

            prev_v = linear_velocity
            prev_w = angular_velocity

        terminal_metrics = compute_segment_metrics(route, state.tolist())
        terminal_goal = terminal_metrics.dist_to_goal / max(terminal_metrics.segment_length, 1.0)
        total_cost += self.config.terminal_cross_track_weight * terminal_metrics.cross_track_error ** 2
        total_cost += self.config.terminal_heading_weight * terminal_metrics.heading_error ** 2
        total_cost += self.config.terminal_goal_distance_weight * terminal_goal ** 2
        return float(total_cost)

    def _step_dynamics(self, pose: Point3D, linear_velocity: float, angular_velocity: float) -> Point3D:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        dt = self.config.step_time
        x += linear_velocity * np.cos(yaw) * dt
        y += linear_velocity * np.sin(yaw) * dt
        yaw = normalize_angle(yaw + angular_velocity * dt)
        return [float(x), float(y), float(yaw)]
