#!/usr/bin/env python3
"""Pure-Python progress watchdogs for safety-critical navigation commands."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class NavigationWatchdogConfig:
    alignment_max_duration_sec: float = 15.0
    alignment_progress_timeout_sec: float = 15.0
    alignment_min_progress_rad: float = 0.05
    tracking_progress_timeout_sec: float = 10.0
    tracking_min_progress_m: float = 0.15
    motion_response_timeout_sec: float = 15.0
    motion_min_translation_m: float = 0.05
    motion_min_rotation_rad: float = 0.05
    command_min_linear_mps: float = 0.05
    command_min_angular_rps: float = 0.08


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi] without depending on NumPy."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def validate_velocity_command(
    linear_velocity: float,
    angular_velocity: float,
    max_linear_speed: float,
    max_angular_speed: float,
) -> str | None:
    """Return a human-readable rejection reason, or ``None`` when safe."""
    if not math.isfinite(linear_velocity) or not math.isfinite(angular_velocity):
        return 'velocity command contains NaN or infinity'
    if abs(linear_velocity) > max_linear_speed:
        return (
            f'linear velocity {linear_velocity:.3f} m/s exceeds '
            f'{max_linear_speed:.3f} m/s'
        )
    if abs(angular_velocity) > max_angular_speed:
        return (
            f'angular velocity {angular_velocity:.3f} rad/s exceeds '
            f'{max_angular_speed:.3f} rad/s'
        )
    return None


class NavigationProgressWatchdog:
    """Latch a fault when commanded motion produces no useful pose progress.

    Fresh ROS messages alone are insufficient for motion safety: a receiver can
    repeatedly publish a numerically frozen pose. This watchdog therefore checks
    both physical pose response and task-level progress for every route segment.
    """

    def __init__(self, config: NavigationWatchdogConfig | None = None):
        self.config = config or NavigationWatchdogConfig()
        self.fault_reason: str | None = None
        self.segment_id = None
        self.phase = None
        self.phase_started = 0.0
        self.last_progress_time = 0.0
        self.best_metric = math.inf
        self.motion_reference_pose = None
        self.motion_reference_time = 0.0

    def reset(self) -> None:
        self.fault_reason = None
        self.segment_id = None
        self.phase = None
        self.best_metric = math.inf
        self.motion_reference_pose = None

    def check_alignment(
        self,
        segment_id,
        now: float,
        pose,
        heading_error: float,
        angular_velocity: float,
    ) -> str | None:
        if self.fault_reason is not None:
            return self.fault_reason
        invalid = self._validate_inputs(pose, heading_error, angular_velocity)
        if invalid:
            return self._fault(invalid)

        self._enter_phase(segment_id, 'alignment', now, pose, abs(heading_error))
        error = abs(heading_error)
        if self.best_metric - error >= self.config.alignment_min_progress_rad:
            self.best_metric = error
            self.last_progress_time = now

        if now - self.phase_started > self.config.alignment_max_duration_sec:
            return self._fault(
                'alignment exceeded maximum duration '
                f'({now - self.phase_started:.1f}s)'
            )
        if now - self.last_progress_time > self.config.alignment_progress_timeout_sec:
            return self._fault(
                'alignment heading error made no progress for '
                f'{now - self.last_progress_time:.1f}s'
            )

        return self._check_motion_response(now, pose, 0.0, angular_velocity)

    def check_tracking(
        self,
        segment_id,
        now: float,
        pose,
        distance_to_goal: float,
        linear_velocity: float,
        angular_velocity: float,
    ) -> str | None:
        if self.fault_reason is not None:
            return self.fault_reason
        invalid = self._validate_inputs(
            pose, distance_to_goal, linear_velocity, angular_velocity
        )
        if invalid:
            return self._fault(invalid)

        self._enter_phase(segment_id, 'tracking', now, pose, distance_to_goal)
        if self.best_metric - distance_to_goal >= self.config.tracking_min_progress_m:
            self.best_metric = distance_to_goal
            self.last_progress_time = now

        if now - self.last_progress_time > self.config.tracking_progress_timeout_sec:
            return self._fault(
                'distance to waypoint made no progress for '
                f'{now - self.last_progress_time:.1f}s'
            )

        return self._check_motion_response(
            now, pose, linear_velocity, angular_velocity
        )

    def _enter_phase(self, segment_id, phase, now, pose, metric) -> None:
        if self.segment_id == segment_id and self.phase == phase:
            return
        self.segment_id = segment_id
        self.phase = phase
        self.phase_started = now
        self.last_progress_time = now
        self.best_metric = metric
        self.motion_reference_pose = tuple(pose)
        self.motion_reference_time = now

    def _check_motion_response(
        self, now, pose, linear_velocity, angular_velocity
    ) -> str | None:
        active = (
            abs(linear_velocity) >= self.config.command_min_linear_mps
            or abs(angular_velocity) >= self.config.command_min_angular_rps
        )
        if not active:
            self.motion_reference_pose = tuple(pose)
            self.motion_reference_time = now
            return None

        reference = self.motion_reference_pose
        distance = math.hypot(pose[0] - reference[0], pose[1] - reference[1])
        rotation = abs(normalize_angle(pose[2] - reference[2]))
        if (
            distance >= self.config.motion_min_translation_m
            or rotation >= self.config.motion_min_rotation_rad
        ):
            self.motion_reference_pose = tuple(pose)
            self.motion_reference_time = now
            return None

        if now - self.motion_reference_time > self.config.motion_response_timeout_sec:
            return self._fault(
                'commanded motion produced no pose response for '
                f'{now - self.motion_reference_time:.1f}s'
            )
        return None

    def _fault(self, reason: str) -> str:
        if self.fault_reason is None:
            self.fault_reason = reason
        return self.fault_reason

    @staticmethod
    def _validate_inputs(pose, *values) -> str | None:
        try:
            all_values = [pose[0], pose[1], pose[2], *values]
        except (IndexError, TypeError):
            return 'pose or controller output has an invalid shape'
        if not all(math.isfinite(float(value)) for value in all_values):
            return 'pose or controller output contains NaN or infinity'
        return None
