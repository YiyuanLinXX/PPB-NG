#!/usr/bin/env python3
"""Common geometry helpers and shared tracking data structures."""

from dataclasses import dataclass
from typing import List

import numpy as np


Point2D = List[float]
Point3D = List[float]
Route = List[Point2D]


@dataclass
class TrackingCommand:
    linear_velocity: float
    angular_velocity: float
    cross_track_error: float
    heading_error: float
    dist_to_goal: float
    dist_from_start: float
    target_speed: float
    lateral_speed: float


@dataclass
class SegmentMetrics:
    path_angle: float
    heading_error: float
    cross_track_error: float
    dist_to_goal: float
    dist_from_start: float
    segment_length: float
    progress: float
    projection: Point2D


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def compute_path_angle(route: Route) -> float:
    pt1, pt2 = np.array(route[0], dtype=float), np.array(route[1], dtype=float)
    path_vector = pt2 - pt1
    return float(np.arctan2(path_vector[1], path_vector[0]))


def get_projection_point(pt1: Point2D, pt2: Point2D, pt3: Point2D) -> tuple[float, float]:
    """Project a point onto the infinite line defined by pt1 -> pt2."""
    line = np.array(pt2, dtype=float) - np.array(pt1, dtype=float)
    denom = float(np.dot(line, line))
    if denom <= 1e-9:
        return float(pt1[0]), float(pt1[1])

    rel = np.array(pt3, dtype=float) - np.array(pt1, dtype=float)
    proj = np.array(pt1, dtype=float) + np.dot(rel, line) / denom * line
    return float(proj[0]), float(proj[1])


def get_cross_track_error(pt1: Point2D, pt2: Point2D, pt3: Point2D, pt4: Point2D) -> float:
    """Signed lateral distance from the robot position to the current path."""
    path_vector = np.array(pt2, dtype=float) - np.array(pt1, dtype=float)
    error_vector = np.array(pt3, dtype=float) - np.array(pt4, dtype=float)
    cross_product = path_vector[0] * error_vector[1] - path_vector[1] * error_vector[0]
    return float(np.linalg.norm(error_vector) * np.sign(cross_product))


def compute_segment_metrics(route: Route, pose: Point3D) -> SegmentMetrics:
    pt1 = np.array(route[0], dtype=float)
    pt2 = np.array(route[1], dtype=float)
    pt3 = np.array(pose[0:2], dtype=float)
    projection = np.array(get_projection_point(route[0], route[1], pose[0:2]), dtype=float)

    path_vector = pt2 - pt1
    segment_length = float(np.linalg.norm(path_vector))
    if segment_length <= 1e-9:
        path_angle = float(pose[2])
        progress = 1.0
        dist_from_start = 0.0
        dist_to_goal = float(np.linalg.norm(pt2 - pt3))
    else:
        path_angle = compute_path_angle(route)
        progress = float(np.clip(np.dot(pt3 - pt1, path_vector) / np.dot(path_vector, path_vector), 0.0, 1.0))
        dist_from_start = float(progress * segment_length)
        dist_to_goal = float(np.linalg.norm(pt2 - pt3))

    heading_error = normalize_angle(path_angle - float(pose[2]))
    cross_track_error = get_cross_track_error(route[0], route[1], pose[0:2], projection.tolist())

    return SegmentMetrics(
        path_angle=path_angle,
        heading_error=heading_error,
        cross_track_error=cross_track_error,
        dist_to_goal=dist_to_goal,
        dist_from_start=dist_from_start,
        segment_length=segment_length,
        progress=progress,
        projection=projection.tolist(),
    )


def apply_speed_schedule(
    target_speed: float,
    dist_to_goal: float,
    slowdown_distance: float,
    min_forward_ratio: float,
    heading_error: float,
    enable_goal_slowdown: bool = True,
    dist_from_start: float | None = None,
    start_slowdown_distance: float = 0.0,
    initial_speed_ratio: float | None = None,
    enable_start_slowdown: bool = False,
) -> float:
    speed = target_speed

    if (
        enable_start_slowdown
        and dist_from_start is not None
        and initial_speed_ratio is not None
        and start_slowdown_distance > 1e-6
        and dist_from_start < start_slowdown_distance
    ):
        ratio = dist_from_start / start_slowdown_distance
        start_ratio = initial_speed_ratio + (1.0 - initial_speed_ratio) * ratio
        speed *= max(min_forward_ratio, start_ratio)

    if enable_goal_slowdown and slowdown_distance > 1e-6 and dist_to_goal < slowdown_distance:
        ratio = dist_to_goal / slowdown_distance
        speed *= max(min_forward_ratio, ratio)

    heading_ratio = 1.0 - min(abs(heading_error) / np.pi, 1.0)
    speed *= max(min_forward_ratio, heading_ratio)
    return float(speed)


def is_goal_reached(route: Route, pose: Point3D, goal_threshold: float) -> bool:
    """Check whether the robot is close enough to or has passed the segment end."""
    pt1 = np.array(route[0], dtype=float)
    pt2 = np.array(route[1], dtype=float)
    pt3 = np.array(pose[0:2], dtype=float)
    path_vector = pt2 - pt1
    denom = float(np.dot(path_vector, path_vector))

    if denom <= 1e-9:
        return bool(np.linalg.norm(pt2 - pt3) < goal_threshold)

    progress = float(np.dot(pt3 - pt1, path_vector) / denom)
    dist_to_goal = float(np.linalg.norm(pt2 - pt3))
    return dist_to_goal < goal_threshold or progress >= 1.0
