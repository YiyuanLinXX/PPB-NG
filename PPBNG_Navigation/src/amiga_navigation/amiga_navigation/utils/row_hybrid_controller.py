#!/usr/bin/env python3
"""Hybrid controller that switches by segment length for vineyard row layouts."""

from dataclasses import dataclass

from amiga_navigation.utils.formal_mpc_controller import FormalMPCController
from amiga_navigation.utils.pid_line_controller import LineTrackingController
from amiga_navigation.utils.pure_pursuit_controller import PurePursuitController
from amiga_navigation.utils.tracking_geometry import Point3D, Route, TrackingCommand, compute_segment_metrics


@dataclass(frozen=True)
class RowHybridConfig:
    connector_length_threshold: float = 3.0
    row_length_threshold: float = 15.0


class RowHybridController:
    controller_name = 'row_hybrid'

    def __init__(
        self,
        row_config: RowHybridConfig,
        pid_controller: LineTrackingController,
        pure_pursuit_controller: PurePursuitController,
        formal_mpc_controller: FormalMPCController,
    ):
        self.row_config = row_config
        self.pid_controller = pid_controller
        self.pure_pursuit_controller = pure_pursuit_controller
        self.formal_mpc_controller = formal_mpc_controller
        self.active_controller_name = self.controller_name

    def reset(self) -> None:
        self.pid_controller.reset()
        self.pure_pursuit_controller.reset()
        self.formal_mpc_controller.reset()
        self.active_controller_name = self.controller_name

    def compute_command(self, route: Route, pose: Point3D) -> TrackingCommand:
        metrics = compute_segment_metrics(route, pose)
        if metrics.segment_length <= self.row_config.connector_length_threshold:
            controller = self.formal_mpc_controller
        elif metrics.segment_length >= self.row_config.row_length_threshold:
            controller = self.pid_controller
        else:
            controller = self.pure_pursuit_controller

        command = controller.compute_command(route, pose)
        self.active_controller_name = controller.active_controller_name
        return command
