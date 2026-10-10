"""Check joystick gating without starting ROS nodes or moving the robot."""
import time
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.action import GoalResponse
from sensor_msgs.msg import Joy

from cat_capture.rotate_node import PanoramaRotate


def test_rb_starts_only_in_manual_mode():
    node = SimpleNamespace(
        buttons=[], state='idle', action_goal=None, goal_reserved=False,
        safety_at=time.monotonic(), safety_mode=2, safety_reason='NAVIGATION: enabled',
        begin=Mock(), abort=Mock(), fresh=lambda stamp: time.monotonic() - stamp < .7,
        rb_button=5)
    joy = Joy()
    joy.buttons = [0, 0, 0, 0, 0, 1]
    PanoramaRotate.on_joy(node, joy)
    node.begin.assert_not_called()

    joy.buttons[5] = 0
    PanoramaRotate.on_joy(node, joy)
    node.safety_reason = 'MANUAL: X selected, right stick enabled'
    joy.buttons[5] = 1
    PanoramaRotate.on_joy(node, joy)
    node.begin.assert_called_once_with()


def test_only_one_action_goal_can_be_reserved():
    node = SimpleNamespace(state='idle', action_goal=None, goal_reserved=False)
    request = SimpleNamespace(frames=0)
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.ACCEPT
    assert PanoramaRotate.accept_goal(node, request) == GoalResponse.REJECT
