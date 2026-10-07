#!/usr/bin/env python3
"""In-place alignment turn controller for waypoint segment entry."""

from dataclasses import dataclass

import numpy as np

from amiga_navigation.utils.tracking_geometry import compute_path_angle, normalize_angle


@dataclass(frozen=True)
class TurnConfig:
    alignment_threshold: float = 0.10
    gain: float = 1.2
    min_turn_speed: float = 0.3
    max_turn_speed: float = 0.6
    enable_slowdown_near_target: bool = False
    slowdown_angle: float = 0.35
    near_target_min_speed_ratio: float = 0.35


@dataclass
class TurnCommand:
    angular_velocity: float
    heading_error: float
    aligned: bool


def compute_turn_command(route, pose, config: TurnConfig | None = None) -> TurnCommand:
    config = config or TurnConfig()
    angle_diff = normalize_angle(compute_path_angle(route) - pose[2])
    aligned = abs(angle_diff) < config.alignment_threshold
    if aligned:
        return TurnCommand(angular_velocity=0.0, heading_error=angle_diff, aligned=True)

    angular_velocity = float(np.clip(config.gain * angle_diff, -config.max_turn_speed, config.max_turn_speed))
    effective_min_turn_speed = config.min_turn_speed
    slowdown_angle = max(config.slowdown_angle, config.alignment_threshold)
    if config.enable_slowdown_near_target and slowdown_angle > config.alignment_threshold:
        # Taper the minimum turn speed near alignment to reduce overshoot and dithering.
        ratio = (abs(angle_diff) - config.alignment_threshold) / (slowdown_angle - config.alignment_threshold)
        ratio = float(np.clip(ratio, 0.0, 1.0))
        min_ratio = float(np.clip(config.near_target_min_speed_ratio, 0.0, 1.0))
        effective_min_turn_speed *= min_ratio + (1.0 - min_ratio) * ratio

    if abs(angular_velocity) < effective_min_turn_speed:
        angular_velocity = effective_min_turn_speed * float(np.sign(angle_diff))
    return TurnCommand(angular_velocity=angular_velocity, heading_error=angle_diff, aligned=False)
