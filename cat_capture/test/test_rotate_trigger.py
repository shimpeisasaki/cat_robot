"""Check joystick gating without starting ROS nodes or moving the robot."""
import math
import time
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.action import GoalResponse
from sensor_msgs.msg import Joy

from cat_capture.rotate_node import PanoramaRotate


def test_rb_starts_only_in_manual_mode():
    node = SimpleNamespace(
        buttons=[], state='idle', action_goal=None, goal_reserved=False, late_start=None,
        safety_at=time.monotonic(), safety_mode=2, safety_source='navigation',
        begin=Mock(), abort=Mock(), fresh=lambda stamp: time.monotonic() - stamp < .7,
        rb_button=5)
    joy = Joy()
    joy.buttons = [0, 0, 0, 0, 0, 1]
    PanoramaRotate.on_joy(node, joy)
    node.begin.assert_not_called()

    joy.buttons[5] = 0
    PanoramaRotate.on_joy(node, joy)
    node.safety_source = 'manual'
    joy.buttons[5] = 1
    PanoramaRotate.on_joy(node, joy)
    node.begin.assert_called_once_with()


def test_only_one_action_goal_can_be_reserved():
    node = SimpleNamespace(state='idle', action_goal=None, goal_reserved=False, late_start=None)
    request = SimpleNamespace(frames=0, angle_deg=0.0)
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.ACCEPT
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.REJECT


def test_late_start_cleanup_can_retry_after_transport_failure():
    start, cleanup = Future(), Future()
    start.set_result(SimpleNamespace(success=True))
    client = Mock()
    client.call_async.side_effect = [RuntimeError('temporary failure'), cleanup]
    node = SimpleNamespace(late_start=start, cleanup_future=None,
                           service_clients={'cancel': client}, get_logger=Mock())
    PanoramaRotate.poll_late_start(node)
    assert node.late_start is start and node.cleanup_future is None
    PanoramaRotate.poll_late_start(node)
    assert node.cleanup_future is cleanup
    cleanup.set_result(SimpleNamespace(success=True))
    PanoramaRotate.poll_late_start(node)
    assert node.late_start is None


def test_centered_arc_prepares_scans_and_returns():
    node = SimpleNamespace(
        state='prepare', heading=0.0, half_angle=math.radians(60),
        step=math.radians(12), speed=.26, phase_at=time.monotonic(),
        action_goal=None, pending=None, frames=10, frame_count=0,
        centered=True, target=math.radians(120), stationary_since=None,
        motor_at=time.monotonic(), motor_ok=True, stationary=False,
        odom_at=time.monotonic(),
        capture_status={'state': 'recording'}, safety_at=time.monotonic(),
        safety_mode=2, safety_reason='PANORAMA: active',
        cmd=SimpleNamespace(publish=Mock()), zero=Mock(),
        poll_service=Mock(), poll_late_start=Mock(), abort=Mock(), call=Mock(return_value=True),
        fresh=lambda stamp: time.monotonic() - stamp < .7,
        panorama_enabled=lambda: True)
    PanoramaRotate.tick(node)
    assert node.cmd.publish.call_args.args[0].angular.z < 0
    node.heading = math.radians(-55)
    PanoramaRotate.tick(node)
    assert 0 < abs(node.cmd.publish.call_args.args[0].angular.z) < node.speed

    node.heading = -node.half_angle
    PanoramaRotate.tick(node)
    assert node.state == 'prepare_settle'
    PanoramaRotate.tick(node)
    assert node.state == 'prepare_settle'
    node.stationary = True
    node.stationary_since = time.monotonic() - .6
    PanoramaRotate.tick(node)
    assert node.state == 'scan'
    node.heading = math.radians(-53.9)
    PanoramaRotate.tick(node)
    node.call.assert_called_once_with('capture')

    node.frame_count = 10
    node.heading = node.half_angle
    PanoramaRotate.tick(node)
    assert node.state == 'scan_settle'
    node.stationary = False
    PanoramaRotate.tick(node)
    assert node.state == 'scan_settle'
    node.stationary = True
    node.stationary_since = time.monotonic() - .6
    PanoramaRotate.tick(node)
    assert node.state == 'return'
    PanoramaRotate.tick(node)
    assert node.cmd.publish.call_args.args[0].angular.z < 0
    node.heading = 0.0
    PanoramaRotate.tick(node)
    assert node.state == 'finish_settle'


def test_timed_out_start_is_cancelled_when_its_reply_arrives():
    start = Future()
    cleanup = Future()
    cancel_client = Mock()
    cancel_client.call_async.return_value = cleanup
    node = SimpleNamespace(
        state='start_request', pending=('start', start, time.monotonic()-6),
        late_start=None, cleanup_future=None, session_active=False,
        action_goal=None, goal_reserved=False, service_clients={'cancel': cancel_client},
        zero=Mock(), get_logger=Mock())
    node.abort = lambda reason: PanoramaRotate.abort(node, reason)
    PanoramaRotate.poll_service(node)
    assert node.state == 'brake_request' and node.late_start is start
    cancel_client.call_async.assert_not_called()
    node.state = 'idle'
    request = SimpleNamespace(frames=0, angle_deg=0.)
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.REJECT
    start.set_result(SimpleNamespace(success=True))
    PanoramaRotate.poll_late_start(node)
    cancel_client.call_async.assert_called_once()
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.REJECT
    cleanup.set_result(SimpleNamespace(success=True))
    PanoramaRotate.poll_late_start(node)
    assert node.late_start is None
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.ACCEPT
