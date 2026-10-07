#!/usr/bin/env python3
"""Factory for constructing waypoint tracking controllers."""

from typing import Protocol

from amiga_navigation.utils.formal_mpc_controller import FormalMPCConfig, FormalMPCController
from amiga_navigation.utils.pid_line_controller import LineTrackingConfig, LineTrackingController
from amiga_navigation.utils.pure_pursuit_controller import PurePursuitConfig, PurePursuitController
from amiga_navigation.utils.rollout_mpc_controller import MPCRolloutConfig, MPCRolloutController
from amiga_navigation.utils.row_hybrid_controller import RowHybridConfig, RowHybridController
from amiga_navigation.utils.tracking_geometry import Point3D, Route, TrackingCommand


class TrackingController(Protocol):
    controller_name: str
    active_controller_name: str

    def reset(self) -> None:
        ...

    def compute_command(self, route: Route, pose: Point3D) -> TrackingCommand:
        ...


def build_tracking_controller(
    controller_type: str,
    line_config: LineTrackingConfig,
    pure_pursuit_config: PurePursuitConfig,
    mpc_rollout_config: MPCRolloutConfig,
    formal_mpc_config: FormalMPCConfig,
    row_hybrid_config: RowHybridConfig,
) -> TrackingController:
    pid_controller = LineTrackingController(line_config)
    pure_pursuit_controller = PurePursuitController(pure_pursuit_config)
    rollout_mpc_controller = MPCRolloutController(mpc_rollout_config)
    formal_mpc_controller = FormalMPCController(formal_mpc_config)

    if controller_type == 'pid_line':
        return pid_controller
    if controller_type == 'pure_pursuit':
        return pure_pursuit_controller
    if controller_type == 'mpc_rollout':
        return rollout_mpc_controller
    if controller_type == 'mpc_formal':
        return formal_mpc_controller
    if controller_type == 'row_hybrid':
        return RowHybridController(
            row_config=row_hybrid_config,
            pid_controller=pid_controller,
            pure_pursuit_controller=pure_pursuit_controller,
            formal_mpc_controller=formal_mpc_controller,
        )

    raise ValueError(f'Unsupported controller type: {controller_type}')
