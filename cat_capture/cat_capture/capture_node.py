"""Capture matching RGB/depth frames and save them as a session."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from cat_panorama_interfaces.srv import StartCapture
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .session import CaptureSession


class PanoramaCapture(Node):
    def __init__(self, **kwargs):
        super().__init__('panorama_capture', **kwargs)
        param = lambda name, default: self.declare_parameter(name, default).value
        self.output_dir = param('output_dir', str(Path.home() / 'ros2_ws/data/d1_vision'))
        self.input_timeout = float(param('input_timeout', .75))
        self.capture_timeout = float(param('capture_timeout', 10.))
        rgb_topic = param('rgb_topic', '/zed/zed_node/rgb/color/rect/image')
        depth_topic = param('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.bridge = CvBridge()
        self.rgb_frames = {}
        self.depth_frames = {}
        self.rgb_at = 0.0
        self.depth_at = 0.0
        self.session = None
        self.frames = 0
        self.state = 'idle'
        self.error = ''
        self.pending_capture = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='panorama')
        self.jobs = deque()
        self.cancel_reason = None
        self.create_subscription(Image, rgb_topic, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.receive_depth, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '~/status', qos)
        self.create_service(StartCapture, '~/start', self.start)
        for name in ('capture', 'finish', 'cancel'):
            self.create_service(Trigger, '~/' + name, getattr(self, name))
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'RGB/depth capture ready; output: {self.output_dir}')

    @staticmethod
    def remember(frames, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp <= 0:
            return False
        frames[stamp] = msg
        while len(frames) > 30:
            frames.pop(next(iter(frames)))
        return True

    def receive_image(self, msg):
        if self.remember(self.rgb_frames, msg):
            self.rgb_at = time.monotonic()

    def receive_depth(self, msg):
        if self.remember(self.depth_frames, msg):
            self.depth_at = time.monotonic()

    def ready(self):
        return (bool(self.rgb_frames.keys() & self.depth_frames.keys()) and
                time.monotonic() - self.rgb_at <= self.input_timeout and
                time.monotonic() - self.depth_at <= self.input_timeout)

    def start(self, request, response):
        if self.state in ('recording', 'finishing', 'cancelling') or self.jobs:
            response.message = 'A panorama session is already active'
        elif not self.ready():
            response.message = 'RGB or depth image is stale'
        else:
            try:
                self.session = CaptureSession(self.output_dir, request.angle_deg, request.frames)
                self.frames, self.error, self.cancel_reason = 0, '', None
                self.state = 'recording'
                response.success, response.path = True, str(self.session.path)
                response.message = 'Session started'
            except Exception as exc:
                response.message = str(exc)
        return response

    def capture(self, _, response):
        if self.state != 'recording' or self.pending_capture or len(self.jobs) >= 32:
            response.message = 'Session is not ready for another frame'
        else:
            self.pending_capture = (self.get_clock().now().nanoseconds, time.monotonic(), None)
            response.success, response.message = True, 'Waiting for matching RGB and depth frames'
        return response

    def save_frame(self, rgb, depth):
        if depth.encoding.upper() != '32FC1':
            raise ValueError(f'Expected depth in meters as 32FC1, got {depth.encoding}')
        image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8').copy()
        distances = np.asarray(self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'),
                               dtype=np.float32).copy()
        if distances.shape != image.shape[:2]:
            raise ValueError('RGB and depth dimensions differ')
        return self.session.capture(image, distances)

    def finish(self, _, response):
        if self.state != 'recording' or self.pending_capture:
            response.message = 'Session is not ready to finish'
        else:
            self.state = 'finishing'
            self.jobs.append(('finish', self.pool.submit(self.session.finish)))
            response.success = True
            response.message = 'Saving RGB/depth frames'
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
                self.error = 'Timed out waiting for matching RGB and depth frames'
                self.state = 'error'
                self.cancel_reason = self.error
            else:
                if candidate is None and self.ready():
                    stamps = self.rgb_frames.keys() & self.depth_frames.keys()
                    stamp = min((stamp for stamp in stamps if stamp >= min_stamp), default=None)
                    if stamp is not None:
                        candidate = (self.rgb_frames[stamp], self.depth_frames[stamp])
                if candidate is not None:
                    self.jobs.append(('capture', self.pool.submit(self.save_frame, *candidate)))
                    self.pending_capture = None
        ready = self.ready()
        self.status.publish(String(data=json.dumps({
            'state': self.state, 'ready': ready, 'frames': self.frames,
            'path': str(self.session.path) if self.session else '',
            'error': self.error, 'waiting_for': 'RGB or depth image is stale' if not ready else '',
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
