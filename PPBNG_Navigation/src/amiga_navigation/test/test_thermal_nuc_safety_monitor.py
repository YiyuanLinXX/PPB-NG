from pathlib import Path

from amiga_navigation.thermal_nuc_safety_monitor import (
    make_zero_twist,
    ThermalMotionPermissionWatchdog,
    ThermalNucSafetyMonitor,
)
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


class FakeMonotonicClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture
def clock():
    return FakeMonotonicClock()


@pytest.fixture
def watchdog(clock):
    return ThermalMotionPermissionWatchdog(1.0, clock)


def test_startup_without_permission_requires_stop(watchdog):
    assert watchdog.stop_reason() == 'waiting'


def test_true_releases_stop_and_false_immediately_restores_it(watchdog):
    watchdog.update(True)
    assert watchdog.stop_reason() is None

    watchdog.update(False)
    assert watchdog.stop_reason() == 'denied'


def test_single_true_message_becomes_stale_after_timeout(clock, watchdog):
    watchdog.update(True)
    clock.advance(1.0)
    assert watchdog.stop_reason() is None

    clock.advance(0.001)
    assert watchdog.stop_reason() == 'stale'


def test_true_false_true_transitions_do_not_stick(watchdog):
    expected_reasons = [None, 'denied', None]
    actual_reasons = []

    for permitted in (True, False, True):
        watchdog.update(permitted)
        actual_reasons.append(watchdog.stop_reason())

    assert actual_reasons == expected_reasons


def test_stop_command_can_never_request_motion():
    command = make_zero_twist()

    assert command.linear.x == 0.0
    assert command.linear.y == 0.0
    assert command.linear.z == 0.0
    assert command.angular.x == 0.0
    assert command.angular.y == 0.0
    assert command.angular.z == 0.0


def test_node_qos_and_timer_enforce_fail_safe_transitions(clock):
    class CapturingPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    rclpy.init()
    node = ThermalNucSafetyMonitor()
    try:
        assert node._subscription.qos_profile.depth == 1
        assert (
            node._subscription.qos_profile.reliability
            == ReliabilityPolicy.RELIABLE
        )
        assert (
            node._subscription.qos_profile.durability
            == DurabilityPolicy.TRANSIENT_LOCAL
        )

        publisher = CapturingPublisher()
        node._stop_publisher = publisher
        node._watchdog = ThermalMotionPermissionWatchdog(1.0, clock)

        node._timer_callback()
        assert len(publisher.messages) == 1

        node._permission_callback(Bool(data=True))
        node._timer_callback()
        assert len(publisher.messages) == 1

        node._permission_callback(Bool(data=False))
        node._timer_callback()
        assert len(publisher.messages) == 2

        node._permission_callback(Bool(data=True))
        clock.advance(1.001)
        node._timer_callback()
        assert len(publisher.messages) == 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_either_monitor_stop_overrides_navigation_via_shared_mux_input():
    package_root = Path(__file__).parents[1]
    mux_config = (package_root / 'config' / 'twist_mux_topics.yaml').read_text()
    thermal_monitor = (
        package_root / 'amiga_navigation' / 'thermal_nuc_safety_monitor.py'
    ).read_text()
    rtk_monitor = (package_root / 'amiga_navigation' / 'rtk_monitor.py').read_text()

    assert 'topic   : cmd_vel_stop' in mux_config
    assert 'priority: 200' in mux_config
    assert 'topic   : cmd_vel_nav' in mux_config
    assert 'priority: 10' in mux_config
    assert "'/cmd_vel_stop'" in thermal_monitor
    assert "'/cmd_vel_stop'" in rtk_monitor


def test_launch_respawns_monitor_before_mux_stop_input_expires():
    package_root = Path(__file__).parents[1]
    launch_source = (
        package_root / 'launch' / 'basic_bringup.launch.py'
    ).read_text()
    mux_config = (package_root / 'config' / 'twist_mux_topics.yaml').read_text()

    assert 'respawn=True' in launch_source
    assert 'respawn_delay=0.2' in launch_source
    assert 'timeout : 1.0' in mux_config
