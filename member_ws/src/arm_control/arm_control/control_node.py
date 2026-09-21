"""The node: a MoveJoints action server, the only publisher of /arm/cmd_joint_velocity.

One execute callback per accepted goal drives the control loop -- reading the
shared joint feed, publishing a velocity command every tick, and finishing
the goal once it settles, is canceled, is preempted by a newer goal, or the
joint feed faults. Everything time-independent about that loop (the gain, the
tolerance, the settle check) lives in `control.py`; this file is only the
wiring: subscriptions, the action server, and the two locks that let the
subscription callback and an executing goal touch the same state safely.
"""

import threading
import time

from autotype_msgs.msg import JointVelocityCommand
from interfaces.action import MoveJoints
from interfaces.msg import ArmStatus, JointObservation
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from . import config, control

# Latched: a late subscriber -- a dashboard, a caller about to send a goal --
# sees the current status immediately instead of waiting for the next
# transition. Same convention robot_state uses for ~/updates/<input>.
STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ArmControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__('arm_control', **kwargs)
        # _state_lock guards everything the subscription callback and an
        # executing goal both touch: the latest joint feed and its health.
        # _goal_lock guards which goal handle is the current one, which is
        # how a newer goal preempts an older one without either thread
        # calling a terminal method on a goal_handle it does not own.
        self._state_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._have_joints = False
        self._joints_healthy = False
        self._joint_problems: list[str] = []
        self._position: dict[str, float] = {}
        self._velocity: dict[str, float] = {}
        self._reported_idle = False
        self._goal_handle = None

        callback_group = ReentrantCallbackGroup()
        self._pub_cmd = self.create_publisher(JointVelocityCommand, '/arm/cmd_joint_velocity', 1)
        self._pub_status = self.create_publisher(ArmStatus, '~/status', STATUS_QOS)
        self.create_subscription(
            JointObservation,
            '/robot_state/updates/joints',
            self._on_joints,
            STATUS_QOS,
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            MoveJoints,
            '~/move_joints',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            handle_accepted_callback=self._on_accepted,
            callback_group=callback_group,
        )
        self.create_timer(
            config.CONTROL_PERIOD_SEC,
            self._publish_idle_zero,
            callback_group=callback_group,
        )
        self.get_logger().info(
            'joints=/robot_state/updates/joints; cmd=/arm/cmd_joint_velocity; '
            'action=~/move_joints; status=~/status'
        )

    # ------------------------------------------------------------- joints

    def _on_joints(self, msg):
        healthy = bool(msg.status.fresh) and bool(msg.status.valid)
        problems = list(msg.status.problems)
        with self._state_lock:
            self._have_joints = True
            was_healthy = self._joints_healthy
            self._joints_healthy = healthy
            self._joint_problems = problems
            if msg.names:
                self._position = dict(zip(msg.names, msg.positions))
                if msg.velocities:
                    self._velocity = dict(zip(msg.names, msg.velocities))
        with self._goal_lock:
            has_goal = self._goal_handle is not None
        if has_goal:
            # The executing goal's own loop reacts to a health change on its
            # next tick, at most one control period away. Publishing a
            # second, possibly conflicting status from here would race it.
            return
        if not self._reported_idle or healthy != was_healthy:
            self._reported_idle = True
            self._publish_status(
                ArmStatus.STOPPED if healthy else ArmStatus.FAULTED,
                problems=problems if not healthy else None,
            )

    # ------------------------------------------------------------- action

    def _on_goal(self, goal_request):
        problems = control.validate_goal(list(goal_request.name), list(goal_request.position))
        with self._state_lock:
            healthy = self._have_joints and self._joints_healthy
        if problems:
            self.get_logger().warn(f'MoveJoints goal rejected: {"; ".join(problems)}')
            return GoalResponse.REJECT
        if not healthy:
            self.get_logger().warn('MoveJoints goal rejected: joint input is not healthy')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        # A newer goal always preempts an older one. The old goal's own loop
        # notices the handle no longer matches and aborts itself -- nothing
        # here touches the old goal_handle directly.
        with self._goal_lock:
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _execute(self, goal_handle):
        target = dict(zip(goal_handle.request.name, goal_handle.request.position))
        result = MoveJoints.Result()
        settled_since = None
        self._publish_status(ArmStatus.MOVING, target=target)
        while True:
            with self._goal_lock:
                if self._goal_handle is not goal_handle:
                    goal_handle.abort()
                    position = self._snapshot_position()
                    return self._finish_result(result, ArmStatus.STOPPED, position)

            if goal_handle.is_cancel_requested:
                self._publish_zero_all()
                self._clear_goal(goal_handle)
                goal_handle.canceled()
                position = self._snapshot_position()
                self._publish_status(ArmStatus.STOPPED, target=target)
                return self._finish_result(result, ArmStatus.STOPPED, position)

            with self._state_lock:
                healthy = self._have_joints and self._joints_healthy
                position = dict(self._position)
                velocity = dict(self._velocity)
                problems = list(self._joint_problems)

            if not healthy:
                self._publish_zero_all()
                self._clear_goal(goal_handle)
                goal_handle.abort()
                self._publish_status(ArmStatus.FAULTED, target=target, problems=problems)
                return self._finish_result(result, ArmStatus.FAULTED, position)

            names = list(config.KNOWN_JOINTS)
            errors = [target[n] - position[n] for n in names]
            velocities = [velocity.get(n, 0.0) for n in names]
            commands = [control.command_velocity(n, target[n], position[n]) for n in names]
            self._publish_command(names, commands)

            now = self.get_clock().now()
            if control.within_tolerance(errors) and control.slow_enough(velocities):
                if settled_since is None:
                    settled_since = now
                dwelt = (now - settled_since).nanoseconds / 1e9
                if dwelt >= config.SETTLE_DWELL_SEC:
                    self._publish_zero_all()
                    self._clear_goal(goal_handle)
                    goal_handle.succeed()
                    self._publish_status(ArmStatus.SETTLED, target=target)
                    return self._finish_result(result, ArmStatus.SETTLED, position)
            else:
                settled_since = None

            feedback = MoveJoints.Feedback(
                state=ArmStatus.MOVING,
                name=names,
                position=[position[n] for n in names],
                velocity=velocities,
                error=errors,
            )
            goal_handle.publish_feedback(feedback)
            self._publish_status(ArmStatus.MOVING, target=target)
            time.sleep(config.CONTROL_PERIOD_SEC)

    def _clear_goal(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is goal_handle:
                self._goal_handle = None

    def _snapshot_position(self):
        with self._state_lock:
            return dict(self._position)

    @staticmethod
    def _finish_result(result, state, position):
        names = list(config.KNOWN_JOINTS)
        result.state = state
        result.name = names
        result.position = [position.get(n, 0.0) for n in names]
        return result

    # ------------------------------------------------------------- output

    def _publish_command(self, names, velocities):
        msg = JointVelocityCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.velocity = velocities
        self._pub_cmd.publish(msg)

    def _publish_zero_all(self):
        """Stop explicitly: name all five joints, command zero velocity on each.

        Omitting a joint would leave it at its last commanded velocity
        (INTERFACES.md 8.5 rule 3) -- stopping the arm means naming every one.
        """
        self._publish_command(list(config.KNOWN_JOINTS), [0.0] * config.JOINT_COUNT)

    def _publish_idle_zero(self):
        """Keep an intentional stop distinct from a dead controller."""
        with self._goal_lock:
            if self._goal_handle is None:
                self._publish_zero_all()

    def _publish_status(self, state, target=None, problems=None):
        with self._state_lock:
            position = dict(self._position)
            velocity = dict(self._velocity)
        names = list(config.KNOWN_JOINTS)
        target = target or position
        msg = ArmStatus(
            stamp=self.get_clock().now().to_msg(),
            state=state,
            name=names,
            position=[position.get(n, 0.0) for n in names],
            velocity=[velocity.get(n, 0.0) for n in names],
            target=[target.get(n, position.get(n, 0.0)) for n in names],
            problems=list(problems) if problems else [],
        )
        self._pub_status.publish(msg)
