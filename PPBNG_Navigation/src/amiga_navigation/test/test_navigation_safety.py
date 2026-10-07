import math

from amiga_navigation.utils.navigation_safety import (
    NavigationProgressWatchdog,
    NavigationWatchdogConfig,
    validate_velocity_command,
)


def test_rejects_nonfinite_and_out_of_range_commands():
    assert validate_velocity_command(math.nan, 0.0, 2.0, 1.5)
    assert validate_velocity_command(0.0, math.inf, 2.0, 1.5)
    assert validate_velocity_command(2.1, 0.0, 2.0, 1.5)
    assert validate_velocity_command(0.0, -1.6, 2.0, 1.5)
    assert validate_velocity_command(1.0, 0.5, 2.0, 1.5) is None


def test_alignment_stops_when_fresh_pose_values_are_frozen():
    watchdog = NavigationProgressWatchdog(
        NavigationWatchdogConfig(
            alignment_progress_timeout_sec=5.0,
            motion_response_timeout_sec=2.0,
        )
    )
    pose = [0.0, 0.0, 0.0]

    assert watchdog.check_alignment(0, 0.0, pose, 1.0, 0.4) is None
    fault = watchdog.check_alignment(0, 2.1, pose, 1.0, 0.4)

    assert 'no pose response' in fault
    assert watchdog.check_alignment(0, 2.2, [1.0, 1.0, 1.0], 0.1, 0.1) == fault


def test_alignment_stops_when_turning_the_wrong_way():
    watchdog = NavigationProgressWatchdog(
        NavigationWatchdogConfig(
            alignment_progress_timeout_sec=2.0,
            motion_response_timeout_sec=5.0,
            motion_min_rotation_rad=0.02,
        )
    )

    assert watchdog.check_alignment(0, 0.0, [0.0, 0.0, 0.0], 1.0, 0.4) is None
    assert watchdog.check_alignment(0, 1.0, [0.0, 0.0, -0.1], 1.1, 0.4) is None
    fault = watchdog.check_alignment(0, 2.1, [0.0, 0.0, -0.2], 1.2, 0.4)

    assert 'heading error made no progress' in fault


def test_alignment_progress_prevents_false_stop():
    watchdog = NavigationProgressWatchdog()

    assert watchdog.check_alignment(3, 0.0, [0.0, 0.0, 0.0], 1.0, 0.4) is None
    assert watchdog.check_alignment(3, 2.0, [0.0, 0.0, 0.4], 0.6, 0.4) is None
    assert watchdog.check_alignment(3, 4.0, [0.0, 0.0, 0.8], 0.2, 0.2) is None


def test_tracking_stops_a_moving_circle_that_never_approaches_goal():
    watchdog = NavigationProgressWatchdog(
        NavigationWatchdogConfig(
            tracking_progress_timeout_sec=3.0,
            motion_response_timeout_sec=2.0,
            motion_min_translation_m=0.01,
        )
    )

    assert watchdog.check_tracking(1, 0.0, [0.0, 0.0, 0.0], 10.0, 0.5, 0.5) is None
    assert watchdog.check_tracking(1, 1.0, [0.2, 0.1, 0.5], 10.0, 0.5, 0.5) is None
    assert watchdog.check_tracking(1, 2.0, [0.3, 0.3, 1.0], 10.0, 0.5, 0.5) is None
    fault = watchdog.check_tracking(1, 3.1, [0.1, 0.4, 1.5], 10.0, 0.5, 0.5)

    assert 'distance to waypoint made no progress' in fault


def test_tracking_goal_progress_and_new_segment_reset_timers():
    watchdog = NavigationProgressWatchdog(
        NavigationWatchdogConfig(tracking_progress_timeout_sec=2.0)
    )

    assert watchdog.check_tracking(1, 0.0, [0.0, 0.0, 0.0], 4.0, 0.5, 0.0) is None
    assert watchdog.check_tracking(1, 1.5, [0.5, 0.0, 0.0], 3.7, 0.5, 0.0) is None
    assert watchdog.check_tracking(1, 3.0, [1.0, 0.0, 0.0], 3.4, 0.5, 0.0) is None
    assert watchdog.check_tracking(2, 10.0, [1.0, 0.0, 0.0], 5.0, 0.5, 0.0) is None
