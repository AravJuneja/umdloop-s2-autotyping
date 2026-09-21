"""Type the launch key: world key positions -> joint goals -> presses.

Subscribes to key_projector's latched KeyPositions and robot_state's launch
key, solves one 5-joint pose per character, executes it through arm_control's
MoveJoints action, presses, then publishes /sim/done.

Threading: the run executes on a worker thread so the node's executor keeps
spinning subscriptions and the action client. Blocking the timer callback
deadlocks goal futures.
"""

import json
import threading
import time

from interfaces.action import MoveJoints
from interfaces.msg import JointObservation, KeyPositions, LaunchKeyObservation
from autotype_msgs.msg import EpisodeResult
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String

from . import aim

NAMES = ['base_yaw', 'shoulder_pitch', 'elbow_pitch', 'head_pan', 'head_tilt']
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PANEL_NORMAL = (-1.0, 0.0, 0.0)  # panel faces the arm; refined per-episode below


class Typist(Node):
    def __init__(self):
        super().__init__('typist')
        self._keys = None
        self._keys_seq = 0  # increments on every KeyPositions publish
        self._normal = None
        self._joint_velocities: dict[str, float] = {}
        self._launch_key = None
        self._launch_key_stamp = None
        self._attempted_key = None  # launch key of the episode being typed
        self._attempted_seq = -1  # KeyPositions seq consumed by that attempt
        self._episode_id = 0  # increments on every launch-key copy
        self._attempted_episode = -1  # episode id consumed by that attempt
        self._result_seen_for = -1  # episode id whose /sim/result arrived
        self._settle_seq = -1  # KeyPositions seq currently settling
        self._settle_since = 0.0  # monotonic time the settling seq arrived
        self._episode_over = threading.Event()
        self._active_handle = None
        self._pass_running = False
        self._handle_lock = threading.Lock()
        self.create_subscription(
            KeyPositions, '/key_projector/key_positions', self._on_keys, LATCHED
        )
        self.create_subscription(
            LaunchKeyObservation, '/robot_state/updates/launch_key', self._on_key, LATCHED
        )
        self.create_subscription(EpisodeResult, '/sim/result', self._on_result, LATCHED)
        self.create_subscription(
            JointObservation, '/robot_state/updates/joints', self._on_joints, 10
        )
        self._client = ActionClient(self, MoveJoints, '/arm_control/move_joints')
        self._press = self.create_publisher(Empty, '/arm/press', 10)
        self._decisions = self.create_publisher(String, '/typist/press_decisions', 10)
        self._done_pub = self.create_publisher(Empty, '/sim/done', 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            'keys=/key_projector/key_positions; action=/arm_control/move_joints'
        )

    def _on_keys(self, msg):
        self._keys = {name: (x, y, z) for name, x, y, z in zip(msg.name, msg.x, msg.y, msg.z)}
        if len(msg.normal) == 3:
            self._normal = tuple(float(v) for v in msg.normal)
        self._keys_seq += 1

    def _on_joints(self, msg):
        if msg.names and msg.velocities and len(msg.names) == len(msg.velocities):
            self._joint_velocities = {
                name: float(velocity) for name, velocity in zip(msg.names, msg.velocities)
            }

    def _on_key(self, msg):
        # robot_state force-republishes the latched launch key on /sim/done
        # with a bumped received_stamp, so every episode -- including a
        # same-key one -- arrives here as a distinct copy. A new copy opens
        # a new episode; merely arming it here, the _tick gate (fresh keys
        # + seen result of the previous episode) decides when typing is due.
        stamp = (msg.status.received_stamp.sec, msg.status.received_stamp.nanosec)
        if stamp == self._launch_key_stamp:
            return
        self._launch_key_stamp = stamp
        self._launch_key = msg.key
        self._episode_id += 1
        self._episode_over.clear()
        with self._handle_lock:
            self._active_handle = None

    def _on_result(self, msg):
        # /sim/result arrives after /sim/done ends the episode. It is the
        # only authoritative "this episode is over" signal: gate the next
        # pass on having SEEN it, so a pass can never start typing an
        # episode whose keys have not been published yet (the key projector
        # withholds them until its estimate reconverges post-reset).
        # The in-flight goal (if any) belongs to the finished episode: the
        # sim ignores further commands and the goal never resolves, so
        # cancel it instead of hanging. A newer pass that already owns the
        # arm (its handle replaced ours after our goal finished) must not
        # be cancelled from here.
        del msg
        self._episode_over.set()
        # A /sim/done launch-key copy may arrive before this result. Complete
        # the pass that actually typed, not whichever episode id is current.
        self._result_seen_for = self._attempted_episode
        with self._handle_lock:
            handle = self._active_handle
            if handle is not None:
                self._active_handle = None
        if handle is not None:
            try:
                cancel = handle.cancel_goal_async()
                cancel.add_done_callback(lambda fut: self._on_cancel_done(fut, handle))
            except Exception:
                pass

    def _on_cancel_done(self, future, handle):
        try:
            future.result()
        except Exception:
            pass
        # Only clear our own handle: a superseded newcomer cancelling
        # itself must not release the older pass's goal. The older pass
        # clears it in _on_goal_done when its own result arrives.
        with self._handle_lock:
            if self._active_handle is handle:
                self._active_handle = None

    def _tick(self):
        # Episode gate, three parts: (1) keys newer than the last attempt
        # (the projector withholds them until its estimate reconverges, so
        # anything older is the previous episode); (2) keys settled (no
        # newer publish for ~3 s -- the projector publishes every frame
        # while converging); (3) the previous episode's /sim/result seen
        # (our own /sim/done precedes the reset, so without this the next
        # pass can start on keys that predate the reset). Never start a
        # second pass while one is running: overlapping passes preempt
        # each other's goals and neither finishes.
        if not self._keys or not self._launch_key:
            return
        with self._handle_lock:
            busy = self._pass_running or self._active_handle is not None
        if busy:
            return
        if self._keys_seq == self._attempted_seq:
            return
        if self._keys_seq == self._settle_seq:
            if time.monotonic() - self._settle_since < 3.0:
                return
        else:
            self._settle_seq = self._keys_seq
            self._settle_since = time.monotonic()
            return
        if self._result_seen_for != self._attempted_episode and self._attempted_episode != -1:
            return
        self._attempted_key = self._launch_key
        self._attempted_seq = self._keys_seq
        self._attempted_episode = self._episode_id
        with self._handle_lock:
            self._pass_running = True
        # Blocking the timer callback stalls this node's subscriptions, so
        # the run goes on a worker thread while the executor keeps spinning.
        threading.Thread(target=self._guarded_run, daemon=True).start()

    def _guarded_run(self):
        try:
            self._run()
        except Exception as error:
            self.get_logger().error(f'typing failed: {error}')
        finally:
            with self._handle_lock:
                self._pass_running = False

    def _wait_settled(self, timeout_sec=5.0):
        """Hold until measured joint speeds are under the press MOVING limit.

        The MoveJoints goal reports SETTLED on one good sample; the plant's
        velocity tracking lags the command, so residual motion can remain.
        The press ladder judges MOVING on these same /joint_states
        velocities (INTERFACES.md 9 step 5: 0.02 rad/s), so wait for the
        same condition before pressing. Returns False on timeout or if the
        episode ended while waiting.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._episode_over.is_set():
                return False
            velocities = self._joint_velocities
            if velocities and max(abs(v) for v in velocities.values()) <= 0.02:
                return True
            time.sleep(0.05)
        return False

    def _send(self, q):
        if self._episode_over.is_set():
            raise RuntimeError('episode already finished; not sending goal')
        goal = MoveJoints.Goal()
        goal.name = NAMES
        goal.position = [float(v) for v in q]
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveJoints server unavailable')
        event = threading.Event()
        outcome = {}
        future = self._client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_sent(fut, outcome, event))
        if not event.wait(timeout=90.0):
            raise RuntimeError(f'goal timeout: {q}')
        if outcome.get('state') != 'settled':
            raise RuntimeError(f'goal ended {outcome.get("state")}: {q}')

    def _on_goal_sent(self, future, outcome, event):
        try:
            handle = future.result()
        except Exception as error:
            outcome['state'] = f'send failed: {error}'
            event.set()
            return
        if not handle.accepted:
            outcome['state'] = 'rejected'
            event.set()
            return
        with self._handle_lock:
            if self._active_handle is not None:
                # A newer episode started while this goal was in flight;
                # the old pass must yield to it, not fight it for the arm.
                outcome['state'] = 'superseded by newer episode'
                event.set()
                try:
                    cancel = handle.cancel_goal_async()
                    cancel.add_done_callback(lambda fut: self._on_cancel_done(fut, handle))
                except Exception:
                    pass
                return
            self._active_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(lambda fut: self._on_goal_done(fut, handle, outcome, event))

    def _on_goal_done(self, future, handle, outcome, event):
        with self._handle_lock:
            if self._active_handle is handle:
                self._active_handle = None
        try:
            outcome['state'] = future.result().result.state
        except Exception as error:
            outcome['state'] = f'result failed: {error}'
        event.set()

    @staticmethod
    def _decision(character, target, solved, episode):
        """Structured press decision for the exportable evidence log."""
        message = String()
        if solved is None:
            message.data = json.dumps(
                {
                    'character': character,
                    'decision': 'skip_unreachable',
                    'episode': episode,
                    'target': list(target),
                }
            )
        else:
            q, rng, inc = solved
            message.data = json.dumps(
                {
                    'character': character,
                    'decision': 'press',
                    'episode': episode,
                    'incidence_deg': float(inc),
                    'joints': [float(v) for v in q],
                    'range_m': float(rng),
                    'target': list(target),
                }
            )
        return message

    def _run(self):
        # Snapshot once per episode: the projector keeps publishing while the
        # arm moves, and the camera rides the forearm -- so any publish from
        # mid-pass is computed from a moved camera, not the home view the
        # pass planned from. Per-character re-reads would steer later keys
        # by the arm's own motion.
        keys = dict(self._keys)
        normal = self._normal or PANEL_NORMAL
        episode = self._attempted_episode
        for ch in self._launch_key:
            if ch not in keys:
                raise RuntimeError(f'no key position for {ch!r}')
            solved = aim.solve_for_key(keys[ch], normal)
            self._decisions.publish(self._decision(ch, keys[ch], solved, episode))
            if solved is None:
                target = self._keys[ch]
                self.get_logger().error(
                    f'no reachable pose for {ch!r} '
                    f'target=({target[0]:.4f},{target[1]:.4f},{target[2]:.4f}) '
                    f'normal=({normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f})'
                )
                raise RuntimeError(f'no reachable pose for {ch!r}')
            q, rng, inc = solved
            self.get_logger().info(
                f'{ch}: q={[round(v, 3) for v in q]} inc={inc:.1f} rng={rng:.3f}'
            )
            self._send(q)
            if not self._wait_settled():
                raise RuntimeError('arm did not settle after move')
            # The settle gate above can pass on stale pre-move velocities;
            # a fixed dwell covers the plant lag the samples cannot see.
            time.sleep(1.0)
            self._press.publish(Empty())
            time.sleep(0.5)
        self._done_pub.publish(Empty())
        self.get_logger().info(f'typed {self._launch_key}; /sim/done published')


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = Typist()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
