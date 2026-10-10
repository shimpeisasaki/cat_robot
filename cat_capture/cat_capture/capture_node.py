"""Capture RGB images at requested angles and save them as a session."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .session import CaptureSession


class PanoramaCapture(Node):
    def __init__(self, **kwargs):
        super().__init__('panorama_capture', **kwargs)
        param = lambda name, default: self.declare_parameter(name, default).value
        self.output_dir = param('output_dir', str(Path.home() / 'ros2_ws/data/panorama'))
        self.input_timeout = float(param('input_timeout', .75))
        self.capture_timeout = float(param('capture_timeout', 10.))
        rgb_topic = param('rgb_topic', '/zed/zed_node/rgb/color/rect/image')
        self.bridge = CvBridge()
        self.latest = None
        self.latest_at = 0.0
        self.last_input_ns = 0
        self.odometry = deque(maxlen=120)
        self.odom_at = 0.0
        self.session = None
        self.frames = 0
        self.state = 'idle'
        self.error = ''
        self.pending_capture = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='panorama')
        self.jobs = deque()
        self.cancel_reason = None
        self.create_subscription(Image, rgb_topic, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.receive_odom, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '~/status', qos)
        for name in ('start', 'capture', 'finish', 'cancel'):
            self.create_service(Trigger, '~/' + name, getattr(self, name))
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'RGB panorama capture ready; output: {self.output_dir}')

    def receive_image(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if stamp <= self.last_input_ns or not -.1 <= age <= self.input_timeout:
            return
        self.last_input_ns = stamp
        self.latest = (msg, stamp)
        self.latest_at = time.monotonic()

    def receive_odom(self, msg):
        q = msg.pose.pose.orientation
        if not all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.odometry.append((Time.from_msg(msg.header.stamp).nanoseconds, yaw))
        self.odom_at = time.monotonic()

    def angle_for(self, image_stamp):
        if not self.odometry:
            return None
        stamp, yaw = min(self.odometry, key=lambda item: abs(item[0] - image_stamp))
        if abs(stamp - image_stamp) > 150_000_000:
            return None
        degrees = round(math.degrees(yaw))
        return (degrees + 180) % 360 - 180

    def ready(self):
        if self.latest is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.latest[1]) * 1e-9
        return (-.1 <= age <= self.input_timeout and
                time.monotonic() - self.latest_at <= self.input_timeout and
                time.monotonic() - self.odom_at <= self.input_timeout)

    def start(self, _, response):
        if self.state in ('recording', 'finishing', 'cancelling') or self.jobs:
            response.message = 'A panorama session is already active'
        elif not self.ready():
            response.message = 'RGB image or /odom is stale'
        else:
            try:
                self.session = CaptureSession(self.output_dir)
                self.frames, self.error, self.cancel_reason = 0, '', None
                self.state = 'recording'
                response.success, response.message = True, str(self.session.path)
            except Exception as exc:
                response.message = str(exc)
        return response

    def capture(self, _, response):
        if self.state != 'recording' or self.pending_capture or len(self.jobs) >= 32:
            response.message = 'Session is not ready for another frame'
        else:
            self.pending_capture = (self.get_clock().now().nanoseconds, time.monotonic(), None)
            response.success, response.message = True, 'Waiting for RGB frame and image-time odometry'
        return response

    def save_frame(self, frame, angle):
        image = self.bridge.imgmsg_to_cv2(frame[0], desired_encoding='bgr8').copy()
        return self.session.capture(image, angle)

    def finish(self, _, response):
        if self.state != 'recording' or self.pending_capture:
            response.message = 'Session is not ready to finish'
        else:
            self.state = 'finishing'
            self.jobs.append(('finish', self.pool.submit(self.session.finish)))
            response.success = True
            response.message = 'Saving RGB frames'
        return response

    def cancel(self, _, response):
        if self.session and self.state in ('recording', 'finishing', 'cancelling'):
            self.pending_capture = None
            self.cancel_reason = 'Capture cancelled by operator or rotation controller'
            self.state = 'cancelling'
            self.jobs = deque((kind, future) for kind, future in self.jobs
                              if not future.cancel())
        response.success, response.message = True, 'Cancellation requested'
        return response

    def tick(self):
        if self.jobs and self.jobs[0][1].done():
            kind, future = self.jobs.popleft()
            try:
                result = future.result()
                if not self.cancel_reason:
                    if kind == 'finish':
                        self.state = 'complete'
                        self.get_logger().info(f'Saved panorama session: {result}')
                    else:
                        self.frames = result
            except Exception as exc:
                self.error, self.state = str(exc), 'error'
                self.cancel_reason = self.cancel_reason or self.error
                self.pending_capture = None
                self.jobs = deque((queued_kind, queued_future)
                                  for queued_kind, queued_future in self.jobs
                                  if not queued_future.cancel())
                self.get_logger().error(self.error)
        if self.cancel_reason and not self.jobs:
            self.state = 'error' if self.error else 'cancelled'
            self.cancel_reason = None
        if self.pending_capture:
            min_stamp, requested_at, candidate = self.pending_capture
            if time.monotonic() - requested_at > self.capture_timeout:
                self.pending_capture = None
                self.error = 'Timed out waiting for RGB frame and image-time /odom'
                self.state = 'error'
                self.cancel_reason = self.error
            else:
                if candidate is None and self.ready() and self.latest[1] >= min_stamp:
                    candidate = self.latest
                    self.pending_capture = (min_stamp, requested_at, candidate)
                if candidate is not None:
                    angle = self.angle_for(candidate[1])
                    if angle is not None:
                        self.jobs.append(('capture', self.pool.submit(self.save_frame, candidate, angle)))
                        self.pending_capture = None
        ready = self.ready()
        self.status.publish(String(data=json.dumps({
            'state': self.state, 'ready': ready, 'frames': self.frames,
            'path': str(self.session.path) if self.session else '',
            'error': self.error, 'waiting_for': 'RGB image or /odom is stale' if not ready else '',
            'input_stamp_ns': self.latest[1] if self.latest else 0,
        })))

    def close(self):
        self.pool.shutdown(wait=True)


def main():
    rclpy.init()
    node = PanoramaCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
