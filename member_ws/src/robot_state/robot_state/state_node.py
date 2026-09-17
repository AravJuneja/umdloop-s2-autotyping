import copy
from functools import partial
import threading

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robot_state_interfaces.msg import RobotState
from robot_state_interfaces.srv import GetRobotState
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .observation import INPUTS, KNOWN_JOINTS, Observation

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
TF_DYNAMIC = QoSProfile(depth=100)
TF_STATIC = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TOPIC_SPECS: dict[str, tuple[type, QoSProfile | int]] = {
    'joints': (JointState, 10),
    'image': (Image, SENSOR),
    'calibration': (CameraInfo, LATCHED),
    'launch_key': (String, LATCHED),
    'tf': (TFMessage, TF_DYNAMIC),
    'tf_static': (TFMessage, TF_STATIC),
}


class StateNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_state', **kwargs)
        self._lock = threading.RLock()
        self._observations = {name: Observation(name) for name in INPUTS}
        self._callbacks = []
        self._last_health = {}
        self._update_publishers = {
            name: self.create_publisher(spec[0], f'~/updates/{name}', LATCHED)
            for name, spec in INPUTS.items()
        }
        self.tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self.tf_buffer, self)
        self._input_subscriptions = [
            self.create_subscription(
                kind, INPUTS[name][1],
                partial(self._on_message, name), qos)
            for name, (kind, qos) in TOPIC_SPECS.items()
        ]
        self._service = self.create_service(GetRobotState, '~/get_state', self._query)
        self.create_timer(0.05, self._refresh)
        self.create_timer(5.0, self._log_status)
        self.get_logger().info(
            f'joints={",".join(KNOWN_JOINTS)}; '
            'updates=~/updates/{input}; query=~/get_state')

    def get_state(self):
        with self._lock:
            now = self.get_clock().now().to_msg()
            return RobotState(sampled_at=now, **{
                name: observation.snapshot(now)
                for name, observation in self._observations.items()
            })

    def on_update(self, callback):
        with self._lock:
            self._callbacks.append(callback)

    def _query(self, request, response):
        response.state = self.get_state()
        return response

    @staticmethod
    def _health(s):
        return (s.arrived, s.fresh, s.valid, s.rate_known, s.rate_ok, tuple(s.problems))

    def _emit(self, name, value):
        with self._lock:
            self._last_health[name] = self._health(value.status)
            self._update_publishers[name].publish(value)
            callbacks = tuple(self._callbacks)
        for callback in callbacks:
            try:
                callback(name, copy.deepcopy(value))
            except Exception as error:
                self.get_logger().error(f'update callback failed: {error}')

    def _on_message(self, name, msg):
        with self._lock:
            now = self.get_clock().now().to_msg()
            self._observations[name].update(msg, now)
            value = self._observations[name].snapshot(now)
        self._emit(name, value)

    def _refresh(self):
        changed = []
        with self._lock:
            now = self.get_clock().now().to_msg()
            for name, observation in self._observations.items():
                if self._last_health.get(name) != self._health(observation.status(now)):
                    changed.append((name, observation.snapshot(now)))
        for name, value in changed:
            self._emit(name, value)

    def _log_status(self):
        state = self.get_state()
        parts = []
        for name in INPUTS:
            value = getattr(state, name)
            s = value.status
            health = 'MISSING' if not s.arrived else (
                'STALE' if not s.fresh else 'fresh')
            parts.append(
                f'{name}={health} valid={s.valid} age={s.age_sec:.3f}s '
                f'rate={s.rate_hz:.1f}/{s.expected_rate_hz:g}Hz '
                f'problems={list(s.problems)}')
        parts.append(f'image={state.image.width}x{state.image.height} '
                     f'{state.image.encoding} frame={state.image.frame_id}')
        parts.append(f'calibration_frame={state.calibration.frame_id}')
        parts.append(f'launch_key={state.launch_key.key}')
        parts.append('tf_frames=' + ','.join(
            f'{t.parent_frame}->{t.child_frame}'
            for value in (state.tf, state.tf_static) for t in value.transforms))
        parts.append('tf_lookup=' + str(self.tf_buffer.can_transform(
            'world', 'camera_optical_frame', Time())))
        self.get_logger().info(' | '.join(parts))
