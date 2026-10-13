"""ROS access on an executor thread; GUI receives snapshots through Qt signals."""

from collections import deque
import copy
from queue import Empty, Queue
from threading import Event, Thread
import time

from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from cat_control_interfaces.msg import ControlState
from nav_msgs.msg import OccupancyGrid, Odometry
from rosbag2_interfaces.srv import Pause, Resume, SetRate, IsPaused
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, UInt8, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .plans import IMAGE_TOPIC


class RosBridge(QObject):
    snapshot = pyqtSignal(dict)
    message = pyqtSignal(str)
    service_done = pyqtSignal(str, bool, str)

    def __init__(self, domain_id=None, offline=False):
        super().__init__()
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('cat_operator_monitor', context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.offline = offline
        self.commands = Queue()
        self.exit_event = Event()
        self.clients = {}
        self.pending = []
        self.states = {}
        self.samples = {}
        self.paused = None
        self.last_graph = 0.0
        volatile = qos_profile_sensor_data
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [('/scan', LaserScan, volatile), ('/odom', Odometry, volatile),
                         ('/tf', TFMessage, volatile), ('/tf_static', TFMessage, retained),
                         ('/map', OccupancyGrid, retained)]
        if not offline:
            subscriptions += [(IMAGE_TOPIC, Image, volatile), ('/base/state', UInt8, volatile),
                              ('/navigation/ready', Bool, volatile),
                              ('/base_safety/control_state', ControlState, retained),
                              ('/waypoint_route/status', String, retained)]
        for topic, kind, qos in subscriptions:
            self.node.create_subscription(
                kind, topic, lambda msg, t=topic: self.receive(t, msg), qos)
        self.node.create_timer(.1, self.tick)
        self.thread = Thread(target=self.spin, name=f'cat-gui-ros-{domain_id}', daemon=False)
        self.thread.start()

    def receive(self, topic, msg):
        now = time.monotonic()
        history = self.samples.setdefault(topic, deque())
        history.append(now)
        while history and history[0] < now - 3.0:
            history.popleft()
        state = {'received': now}
        if isinstance(msg, ControlState):
            state.update(mode=msg.mode, source=msg.source, reason=msg.reason)
        elif isinstance(msg, (Bool, UInt8, String)):
            state['value'] = msg.data
        elif isinstance(msg, OccupancyGrid):
            state['size'] = f'{msg.info.width}×{msg.info.height}'
        self.states[topic] = state

    def request(self, service, kind='trigger', value=None, tag=''):
        self.commands.put((service, kind, value, tag or service))

    def issue(self, service, kind, value, tag):
        types = {'trigger': Trigger, 'pause': Pause, 'resume': Resume,
                 'rate': SetRate, 'is_paused': IsPaused}
        service_type = types[kind]
        key = (service, kind)
        client = self.clients.get(key)
        if client is None:
            client = self.node.create_client(service_type, service)
            self.clients[key] = client
        if not client.service_is_ready():
            if kind == 'is_paused':
                self.paused = None
            else:
                self.service_done.emit(tag, False, f'サービスがありません: {service}')
            return
        request = service_type.Request()
        if kind == 'rate':
            request.rate = float(value)
        future = client.call_async(request)
        self.pending.append((future, time.monotonic() + 5.0, tag, kind))

    def tick(self):
        while True:
            try:
                self.issue(*self.commands.get_nowait())
            except Empty:
                break
        now = time.monotonic()
        for item in list(self.pending):
            future, deadline, tag, kind = item
            if not future.done() and now < deadline:
                continue
            self.pending.remove(item)
            if not future.done():
                future.cancel()
                if kind != 'is_paused':
                    self.service_done.emit(tag, False, 'サービス応答がタイムアウトしました')
                continue
            try:
                response = future.result()
                if kind == 'is_paused':
                    self.paused = response.paused
                else:
                    success = getattr(response, 'success', True)
                    text = getattr(response, 'message', '完了')
                    self.service_done.emit(tag, success, text)
            except Exception as exc:
                if kind != 'is_paused':
                    self.service_done.emit(tag, False, str(exc))
        if now - self.last_graph < 1.0:
            return
        self.last_graph = now
        topics = {name: {'types': types, 'publishers': self.node.count_publishers(name)}
                  for name, types in self.node.get_topic_names_and_types()}
        states = copy.deepcopy(self.states)
        for topic, state in states.items():
            state['age'] = now - state.pop('received')
            history = self.samples.get(topic, ())
            count = sum(item >= now - 3.0 for item in history)
            state['hz'] = round(count / 3.0, 1)
        self.snapshot.emit({'topics': topics, 'nodes': self.node.get_node_names(),
                            'states': states, 'paused': self.paused})
        if self.offline and not any(item[3] == 'is_paused' for item in self.pending):
            client = self.clients.get(('/rosbag2_player/is_paused', 'is_paused'))
            if client is not None or '/rosbag2_player/is_paused' in dict(
                    self.node.get_service_names_and_types()):
                self.issue('/rosbag2_player/is_paused', 'is_paused', None, 'is_paused')

    def spin(self):
        try:
            while not self.exit_event.is_set() and self.context.ok():
                self.executor.spin_once(timeout_sec=.1)
        except Exception as exc:
            if not self.exit_event.is_set():
                self.message.emit(f'ROS監視エラー: {exc}')

    def close(self):
        self.exit_event.set()
        self.executor.wake()
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            self.context.try_shutdown()
            self.thread.join(timeout=3.0)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.context.try_shutdown()
