"""Action-driven panorama frame capture through the base_safety command gate."""

import json
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from cat_panorama_interfaces.action import CapturePanorama
from cat_panorama_interfaces.srv import StartCapture
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger


class PanoramaRotate(Node):
    def __init__(self):
        super().__init__('panorama_rotate')
        self.frames = int(self.declare_parameter('frames', 24).value)
        self.default_frames = self.frames
        requested_speed = float(self.declare_parameter('angular_speed', 0.26).value)
        self.speed = min(requested_speed, 0.26)
        self.rb_button = int(self.declare_parameter('panorama_button', 5).value)
        if not 3 <= self.frames <= 255 or self.speed <= 0:
            raise ValueError('frames must be 3..255 and angular_speed > 0')
        self.step = 2 * math.pi / self.frames
        self.cmd = self.create_publisher(Twist, '/cmd_vel_panorama', 1)
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/base/state', self.on_motor, 1)
        self.create_subscription(String, '/base_safety/status', self.on_safety, 1)
        self.create_subscription(String, '/panorama_capture/status', self.on_capture, 1)
        self.service_clients = {name: self.create_client(Trigger, service) for name, service in {
            'panorama': '/base_safety/panorama', 'brake': '/base_safety/brake',
            'capture': '/panorama_capture/capture',
            'finish': '/panorama_capture/finish', 'cancel': '/panorama_capture/cancel',
        }.items()}
        self.service_clients['start'] = self.create_client(StartCapture, '/panorama_capture/start')
        self.state = 'idle'
        self.action_server = ActionServer(
            self, CapturePanorama, '/panorama/capture', self.execute_goal,
            goal_callback=self.accept_goal, cancel_callback=self.cancel_goal,
            callback_group=ReentrantCallbackGroup())
        self.action_future = None
        self.action_goal = None
        self.goal_reserved = False
        self.outcome = None
        self.outcome_message = ''
        self.result_path = ''
        self.drive_requested = False
        self.started_at = 0.0
        self.pending = None
        self.buttons = []
        self.odom_yaw = None
        self.odom_at = 0.0
        self.heading = 0.0
        self.last_heading_yaw = None
        self.motor_ok = False
        self.stationary = False
        self.motor_at = 0.0
        self.safety_mode = None
        self.safety_reason = ''
        self.safety_at = 0.0
        self.capture_status = {}
        self.capture_at = 0.0
        self.frame_count = 0
        self.target = 2 * math.pi
        self.angle_deg = 360.0
        self.centered = False
        self.half_angle = 0.0
        self.phase_at = 0.0
        self.stationary_since = None
        self.session_active = False
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Panorama frame capture ready; {self.frames} frames')

    def on_joy(self, msg):
        previous = self.buttons
        self.buttons = list(msg.buttons)
        def pressed(index):
            return len(self.buttons) > index and self.buttons[index] and (
                len(previous) <= index or not previous[index])
        if pressed(1) or pressed(2) or pressed(3):
            if self.state != 'idle':
                self.abort('Joystick took control', operator=True)
        elif (pressed(self.rb_button) and self.state == 'idle' and
              not self.goal_reserved and self.action_goal is None and
              self.fresh(self.safety_at) and self.safety_mode == 2 and
              self.safety_reason.startswith('MANUAL')):
            self.begin()

    def accept_goal(self, request):
        if self.state != 'idle' or self.goal_reserved or self.action_goal is not None:
            return GoalResponse.REJECT
        if (request.frames != 0 and request.frames < 3) or not math.isfinite(request.angle_deg) or (
                request.angle_deg < 0 or request.angle_deg > 360):
            return GoalResponse.REJECT
        self.goal_reserved = True
        return GoalResponse.ACCEPT

    def cancel_goal(self, goal_handle):
        return CancelResponse.ACCEPT if goal_handle is self.action_goal else CancelResponse.REJECT

    async def execute_goal(self, goal_handle):
        self.action_goal = goal_handle
        self.action_future = Future()
        self.begin(goal_handle.request.frames or self.default_frames,
                   goal_handle.request.angle_deg)
        outcome, path, message = await self.action_future
        result = CapturePanorama.Result()
        result.success, result.path, result.message = outcome == 'success', path, message
        if outcome == 'success':
            goal_handle.succeed()
        elif outcome == 'cancelled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.action_goal = None
        self.action_future = None
        self.goal_reserved = False
        return result

    def complete_goal(self, outcome, message):
        if self.action_future is not None and not self.action_future.done():
            self.action_future.set_result((outcome, self.result_path, message))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        if not all(math.isfinite(x) for x in (q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if self.last_heading_yaw is not None:
            delta = math.atan2(math.sin(yaw - self.last_heading_yaw),
                               math.cos(yaw - self.last_heading_yaw))
            if abs(delta) > 0.35 and self.state not in ('idle', 'waiting'):
                self.abort('Odometry yaw jumped', operator=False)
            else:
                self.heading += delta
        self.last_heading_yaw = yaw
        self.odom_yaw = yaw
        self.odom_at = time.monotonic()

    def on_motor(self, msg):
        self.motor_ok = bool(msg.data & 1)
        self.stationary = self.motor_ok and bool(msg.data & 2)
        self.motor_at = time.monotonic()

    def on_safety(self, msg):
        try:
            mode, self.safety_reason = msg.data.split(': ', 1)
            self.safety_mode = int(mode)
            self.safety_at = time.monotonic()
        except ValueError:
            self.safety_mode = None

    def on_capture(self, msg):
        try:
            self.capture_status = json.loads(msg.data)
            self.capture_at = time.monotonic()
        except (ValueError, TypeError):
            self.capture_status = {}

    def begin(self, frames=None, angle_deg=0.0):
        self.frames = frames or self.default_frames
        self.centered = angle_deg > 0
        self.angle_deg = angle_deg if self.centered else 360.0
        self.target = math.radians(self.angle_deg)
        self.half_angle = self.target / 2 if self.centered else 0.0
        self.step = self.target / self.frames
        self.state = 'waiting'
        self.started_at = time.monotonic()
        self.last_heading_yaw = None
        self.heading = 0.0
        self.frame_count = 0
        self.session_active = False
        self.result_path = ''
        self.outcome = None
        self.outcome_message = ''
        self.drive_requested = False
        self.pending = None
        if self.call('panorama'):
            self.drive_requested = True
        self.get_logger().info('Panorama frames requested; waiting for motor, odometry and RGB')

    def fresh(self, stamp, age=0.7):
        return time.monotonic() - stamp < age

    def panorama_enabled(self):
        return (self.fresh(self.safety_at) and self.safety_mode == 2 and
                self.safety_reason.startswith('PANORAMA'))

    def zero(self):
        self.cmd.publish(Twist())

    def call(self, name):
        client = self.service_clients[name]
        if not client.service_is_ready():
            return False
        request = StartCapture.Request() if name == 'start' else Trigger.Request()
        if name == 'start':
            request.angle_deg = self.angle_deg
            request.frames = self.frames
        self.pending = (name, client.call_async(request), time.monotonic())
        return True

    def poll_service(self):
        if self.pending is None:
            return
        name, future, sent = self.pending
        if not future.done():
            if time.monotonic() - sent > 5:
                self.pending = None
                if name == 'brake':
                    self.get_logger().error('BRAKE request timed out; check base_safety/status')
                    self.complete_goal('error', 'BRAKE request timed out')
                    self.state = 'idle'
                else:
                    self.abort(f'{name} service timed out')
            return
        self.pending = None
        try:
            reply = future.result()
        except Exception as exc:
            if name == 'brake':
                self.get_logger().error(f'BRAKE request failed: {exc}')
                self.complete_goal('error', f'BRAKE request failed: {exc}')
                self.state = 'idle'
            else:
                self.abort(f'{name} service failed: {exc}')
            return
        if not reply.success:
            if name == 'brake':
                self.get_logger().error(f'BRAKE request rejected: {reply.message}')
                self.complete_goal('error', f'BRAKE request rejected: {reply.message}')
                self.state = 'idle'
            else:
                self.abort(f'{name} rejected: {reply.message}')
            return
        now = time.monotonic()
        if name == 'start':
            self.session_active = True
            self.result_path = reply.path
            self.state, self.phase_at = ('prepare' if self.centered else 'scan'), now
            self.last_heading_yaw = self.odom_yaw
            self.heading = 0.0
            self.capture_status = {'state': 'recording', 'frames': 0}
        elif name == 'capture':
            self.frame_count += 1
        elif name == 'finish':
            self.state, self.phase_at = 'finish_wait', now
        elif name == 'brake':
            self.state, self.phase_at = 'brake_wait', now

    def abort(self, reason, operator=False):
        if self.state == 'idle':
            return
        self.zero()
        if self.state in ('brake_request', 'brake_wait'):
            return
        self.pending = None
        if self.session_active and self.service_clients['cancel'].service_is_ready():
            self.service_clients['cancel'].call_async(Trigger.Request())
        self.session_active = False
        self.get_logger().warning(f'Panorama stopped: {reason}')
        self.outcome = 'cancelled' if self.action_goal and self.action_goal.is_cancel_requested else 'error'
        self.outcome_message = reason
        if operator:
            self.state = 'idle'  # base_safety handles B, X or Y.
            self.complete_goal(self.outcome, reason)
        else:
            self.state, self.phase_at = 'brake_request', time.monotonic()

    def finish_and_brake(self):
        self.session_active = False
        self.outcome = 'success'
        self.outcome_message = 'Panorama frames saved'
        self.state, self.phase_at = 'brake_request', time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.action_goal and self.action_goal.is_cancel_requested and self.state not in ('idle', 'brake_request', 'brake_wait'):
            self.abort('Action cancelled')
        self.poll_service()
        if self.state == 'idle':
            return
        if self.state not in ('prepare', 'scan', 'return'):
            self.zero()
        if self.state == 'brake_request':
            if self.pending is None and not self.call('brake') and now - self.phase_at > 5:
                self.get_logger().error('Could not contact /base_safety/brake')
                self.complete_goal('error', 'Could not contact /base_safety/brake')
                self.state = 'idle'
            return
        if self.state == 'brake_wait':
            if self.fresh(self.safety_at) and self.safety_mode == 0:
                self.get_logger().info('Base is in BRAKE mode')
                self.complete_goal(self.outcome or 'error', self.outcome_message or 'Panorama stopped')
                self.state = 'idle'
            elif now - self.phase_at > 5:
                self.get_logger().error('BRAKE mode was not confirmed; check base_safety/status')
                self.complete_goal('error', 'BRAKE mode was not confirmed')
                self.state = 'idle'
            return
        if self.state == 'waiting':
            if now - self.started_at > 60:
                self.abort('Motor, VIO or RGB/depth did not become ready within 60 s')
                return
            if not self.drive_requested and self.pending is None and self.call('panorama'):
                self.drive_requested = True
            ready = (self.fresh(self.odom_at) and self.fresh(self.motor_at) and self.motor_ok and
                     self.fresh(self.capture_at) and self.capture_status.get('ready') and
                     self.fresh(self.safety_at))
            if not ready or self.pending is not None:
                return
            if self.panorama_enabled():
                self.last_heading_yaw = self.odom_yaw
                self.heading = 0.0
                if self.call('start'):
                    self.state = 'start_request'
            return
        if self.state in ('start_request', 'finish_request'):
            return
        if self.state == 'finish_wait':
            result = self.capture_status.get('state')
            if result == 'complete':
                self.get_logger().info(f"Saved {self.capture_status.get('path', '')}")
                self.finish_and_brake()
            elif result in ('error', 'cancelled') or now - self.phase_at > 60:
                self.abort(self.capture_status.get('error') or 'Frame saving failed')
            return
        if not self.panorama_enabled() or not self.fresh(self.motor_at) or not self.motor_ok:
            self.abort('Base drive permission or motor feedback was lost')
            return
        if not self.fresh(self.odom_at):
            self.abort('Odometry is stale')
            return
        if self.capture_status.get('state') in ('error', 'cancelled'):
            self.abort(self.capture_status.get('error') or 'RGB/depth capture failed')
            return
        tolerance = min(math.radians(2), self.step / 4)
        if self.state == 'prepare':
            remaining = self.heading + self.half_angle
            if remaining <= tolerance:
                self.zero()
                self.state, self.phase_at = 'prepare_settle', now
                self.stationary_since = None
            elif now - self.phase_at > max(20, 2 * self.half_angle / self.speed + 10):
                self.abort('Timed out turning to capture start')
            else:
                command = Twist()
                command.angular.z = -min(self.speed, max(0.06, remaining * 0.8))
                self.cmd.publish(command)
            return
        if self.state in ('prepare_settle', 'scan_settle'):
            self.stationary_since = (self.stationary_since or now) if self.stationary else None
            if now - self.phase_at > 10:
                self.abort('Timed out waiting for wheels to stop before reversing')
            elif self.stationary_since is not None and now - self.stationary_since >= 0.5:
                self.state, self.phase_at = (
                    'scan' if self.state == 'prepare_settle' else 'return'), now
                self.stationary_since = None
            return
        if self.state == 'scan':
            if self.action_goal:
                feedback = CapturePanorama.Feedback()
                feedback.frames_captured = self.frame_count
                feedback.degrees_turned = math.degrees(self.heading + self.half_angle)
                self.action_goal.publish_feedback(feedback)
            scan_end = self.half_angle if self.centered else self.target
            remaining = scan_end - self.heading
            if remaining <= tolerance:
                self.zero()
                self.state, self.phase_at = (
                    'scan_settle' if self.centered else 'finish_settle'), now
                self.stationary_since = None
            elif (self.frame_count < self.frames and
                  self.heading >= -self.half_angle + (self.frame_count + 1.5) * self.step):
                self.abort('Missed a capture angle')
            elif now - self.phase_at > max(40, 2 * self.target / self.speed + 10):
                self.abort(f'Turn timed out: {math.degrees(remaining):.1f} deg remaining')
            else:
                command = Twist()
                command.angular.z = min(self.speed, max(0.06, remaining * 0.8))
                self.cmd.publish(command)
                if (self.pending is None and self.frame_count < self.frames and
                        self.heading >= -self.half_angle + (self.frame_count + 0.5) * self.step):
                    self.call('capture')
            return
        if self.state == 'return':
            if self.frame_count < self.frames and self.pending is None:
                self.abort('Scan ended before all capture requests completed')
            elif self.heading <= tolerance:
                self.zero()
                self.state, self.phase_at = 'finish_settle', now
                self.stationary_since = None
            elif now - self.phase_at > max(20, 2 * self.half_angle / self.speed + 10):
                self.abort('Timed out returning to starting heading')
            else:
                command = Twist()
                command.angular.z = -min(self.speed, max(0.06, self.heading * 0.8))
                self.cmd.publish(command)
            return
        if self.state == 'finish_settle':
            self.stationary_since = (self.stationary_since or now) if self.stationary else None
            if self.pending is None and self.frame_count < self.frames:
                self.abort('Scan ended before all capture requests completed')
            elif now - self.phase_at > 180:
                self.abort('Timed out waiting for saved frames or stationary base')
            elif (self.frame_count == self.frames and self.pending is None and
                  self.capture_status.get('frames', 0) == self.frames and
                  self.stationary_since is not None and now - self.stationary_since >= 0.5):
                if self.call('finish'):
                    self.state = 'finish_request'

    def stop_for_shutdown(self):
        if self.state == 'idle':
            return
        self.zero()
        if self.session_active and self.service_clients['cancel'].service_is_ready():
            self.service_clients['cancel'].call_async(Trigger.Request())
        if self.service_clients['brake'].service_is_ready():
            self.service_clients['brake'].call_async(Trigger.Request())
            deadline = time.monotonic() + 3
            while rclpy.ok() and time.monotonic() < deadline and self.safety_mode != 0:
                rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PanoramaRotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_for_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
