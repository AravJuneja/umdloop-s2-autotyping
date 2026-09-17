"""The node: subscribe to the simulator, publish one normalized view of it.

Consumers can read the state three ways, all backed by the same snapshots.
Subscribe to `~/updates/<input>` to be told when one input changes, call
`~/get_state` for a consistent cross-input snapshot, or register an in-process
callback with `on_update` when you are composed into the same node.
"""

import copy
from functools import partial
import threading

from rclpy.node import Node
from rclpy.time import Time
from robot_state_interfaces.msg import RobotState
from robot_state_interfaces.srv import GetRobotState
from tf2_ros import Buffer, TransformListener

from . import config
from .observation import Observation


class StateNode(Node):

    def __init__(self, **kwargs):
        super().__init__('robot_state', **kwargs)
        # One lock guards the observations, the callback list, and the health
        # cache together, so a snapshot taken for the service cannot catch one
        # input mid-update while another has already moved on.
        self._lock = threading.RLock()
        self._observations = {name: Observation(name) for name in config.INPUTS}
        self._callbacks = []
        # Health as of the last thing we published per input, so the timer can
        # tell a real transition from a tick where nothing changed.
        self._last_health = {}
        # Latched, so a consumer that starts after us immediately sees the
        # current value of every input instead of waiting for the next message.
        self._update_publishers = {
            name: self.create_publisher(spec.observation, f'~/updates/{name}',
                                        config.LATCHED_QOS)
            for name, spec in config.INPUTS.items()
        }
        # Our own TF buffer, fed by the standard listener rather than by our
        # normalized transforms: consumers get real lookups across the tree,
        # not just the individual transforms we happened to receive.
        self.tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self.tf_buffer, self)
        self._input_subscriptions = [
            self.create_subscription(
                spec.ros_type, spec.topic, partial(self._on_message, name), spec.qos)
            for name, spec in config.INPUTS.items()
        ]
        self._service = self.create_service(GetRobotState, '~/get_state', self._query)
        self.create_timer(config.HEALTH_TICK_SEC, self._refresh)
        self.create_timer(config.LOG_TICK_SEC, self._log_status)
        self.get_logger().info(
            f'joints={",".join(config.KNOWN_JOINTS)}; '
            'updates=~/updates/{input}; query=~/get_state')

    def get_state(self):
        """Snapshot every input against a single clock reading.

        All six observations are aged against the same `now`, so their ages
        are comparable to each other rather than to whenever each one was
        individually sampled.
        """
        with self._lock:
            now = self.get_clock().now().to_msg()
            return RobotState(sampled_at=now, **{
                name: observation.snapshot(now)
                for name, observation in self._observations.items()
            })

    def on_update(self, callback):
        """Register an in-process consumer, called with (name, observation)."""
        with self._lock:
            self._callbacks.append(callback)

    def _query(self, request, response):
        response.state = self.get_state()
        return response

    @staticmethod
    def _health(s):
        """Reduce a status to the part worth republishing for.

        Ages and rates drift continuously and would make every timer tick look
        like a change; these fields only move when something actually happened.
        """
        return (s.arrived, s.fresh, s.valid, s.rate_known, s.rate_ok, tuple(s.problems))

    def _emit(self, name, value):
        # Publishing happens under the lock so the health we record always
        # matches the value that went out. Callbacks run outside it: they are
        # arbitrary consumer code, and a slow one must not stall the node.
        with self._lock:
            self._last_health[name] = self._health(value.status)
            self._update_publishers[name].publish(value)
            callbacks = tuple(self._callbacks)
        for callback in callbacks:
            try:
                # Each consumer gets its own copy; one mutating what it
                # receives must not corrupt the next consumer's view.
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
        """Report health changes that no message will announce.

        An input going stale is the absence of a message, so nothing else
        would trigger a publish. Only changed inputs are snapshotted, which
        keeps the tick from copying an unchanged camera frame 20 times a
        second just to republish an identical status.
        """
        changed = []
        with self._lock:
            now = self.get_clock().now().to_msg()
            for name, observation in self._observations.items():
                if self._last_health.get(name) != self._health(observation.status(now)):
                    changed.append((name, observation.snapshot(now)))
        for name, value in changed:
            self._emit(name, value)

    def _log_status(self):
        """One periodic line covering every input, for eyeballing a live run."""
        state = self.get_state()
        parts = []
        for name in config.INPUTS:
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
        # Proves the tree actually connects end to end, which the individual
        # transforms above do not.
        parts.append('tf_lookup=' + str(self.tf_buffer.can_transform(
            'world', 'camera_optical_frame', Time())))
        self.get_logger().info(' | '.join(parts))
