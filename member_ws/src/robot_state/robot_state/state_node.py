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
        # _lock guards the observations, so a snapshot taken for the service
        # cannot catch one input mid-update while another has already moved on.
        # Every snapshot leaves it carrying a sequence number, which is what
        # lets _emit put concurrent snapshots back in order.
        self._lock = threading.RLock()
        self._observations = {name: Observation(name) for name in config.INPUTS}
        self._sequence = 0
        # Rebound rather than mutated, so _emit can read it without a lock.
        self._callbacks: tuple = ()
        # _emit_lock guards everything about publishing: the health and
        # sequence of the last value sent per input, and the publish itself.
        # It is never held while taking _lock, so the two cannot deadlock.
        self._emit_lock = threading.Lock()
        self._last_health = {}
        self._last_sequence = {}
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
            self._callbacks = self._callbacks + (callback,)

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

    def _snapshot(self, name, now):
        """Take a sequenced snapshot. The caller must hold _lock."""
        self._sequence += 1
        return name, self._sequence, self._observations[name].snapshot(now)

    def _emit(self, name, sequence, value):
        # Neither _lock nor the publisher is held while consumer callbacks
        # run: they are arbitrary code, and a slow one must not stall the
        # node. Publishing is not under _lock either, because ~/updates/image
        # is a reliable 2.7 MB topic and a subscriber that falls behind can
        # make the write block for max_blocking_time.
        with self._emit_lock:
            # Snapshots are taken under _lock but published after releasing
            # it, so two of them can arrive here out of order. The sequence
            # number is what keeps a stale one from landing on top of a newer
            # one and leaving the latched topic behind the truth.
            if sequence < self._last_sequence.get(name, 0):
                return
            self._last_sequence[name] = sequence
            self._last_health[name] = self._health(value.status)
            self._update_publishers[name].publish(value)
        for callback in self._callbacks:
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
            pending = self._snapshot(name, now)
        self._emit(*pending)

    def _refresh(self):
        """Report health changes that no message will announce.

        An input going stale is the absence of a message, so nothing else
        would trigger a publish. Only changed inputs are snapshotted, which
        keeps the tick from copying an unchanged camera frame 20 times a
        second just to republish an identical status.
        """
        with self._emit_lock:
            published = dict(self._last_health)
        changed = []
        with self._lock:
            now = self.get_clock().now().to_msg()
            for name, observation in self._observations.items():
                if published.get(name) != self._health(observation.status(now)):
                    changed.append(self._snapshot(name, now))
        for pending in changed:
            self._emit(*pending)

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
        # The names actually arriving, not the ones we expect: if the
        # simulator renames a joint, this line is where it shows up.
        parts.append('joint_names=' + ','.join(state.joints.names))
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
            *config.TF_LOOKUP_CHECK, Time())))
        self.get_logger().info(' | '.join(parts))
