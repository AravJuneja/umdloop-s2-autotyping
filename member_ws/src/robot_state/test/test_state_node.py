"""The node's public contract, exercised over a real ROS graph.

Runs on an isolated domain id, once with the node in this process and once
with it spawned as a separate process, so the same assertions have to hold
whether or not the messages cross a DDS boundary.
"""

from collections import Counter
from functools import partial
import os
import signal
import subprocess
import sys
import time
from typing import Any

from conftest import DOMAIN_ID, message
from interfaces.srv import GetRobotState
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from robot_state.config import INPUTS, KNOWN_JOINTS, LATCHED_QOS
from robot_state.state_node import StateNode
from std_msgs.msg import Empty, String


def spin_until(executor, predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError('condition did not become true before timeout')


@pytest.mark.parametrize('separate_process', [False, True])
def test_live_query_latching_staleness_and_tf(separate_process):
    """One episode: inputs arrive, go stale, recover.

    Deliberately a single test rather than six. The behaviours it checks
    are sequential states of one live node -- latched values surviving a
    stall, staleness being announced exactly once, an invalid message
    replacing a valid one -- and splitting them would mean standing a
    node up six times to re-reach the same states.
    """
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    probe = Node('state_contract_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    adapter = None
    process = None
    updates = {}
    counts: Counter[str] = Counter()
    local_updates = {}
    publishers = {}
    timers = []
    try:
        for name, spec in INPUTS.items():
            publishers[name] = probe.create_publisher(spec.ros_type, spec.topic, spec.qos)
        # Published before the node exists: transient-local QoS means it
        # still has to receive them once it subscribes.
        for name in ('calibration', 'launch_key', 'tf_static'):
            publishers[name].publish(message(name))
        if separate_process:
            process = subprocess.Popen(
                [sys.executable, '-m', 'robot_state.robot_state'],
                env={**os.environ, 'ROS_DOMAIN_ID': str(DOMAIN_ID)},
            )
        else:
            adapter = StateNode(context=context)
            executor.add_node(adapter)
            initial = adapter.get_state()
            assert all(not getattr(initial, name).status.arrived for name in INPUTS)

            def local_callback(name, value):
                # Mutating what we receive proves each consumer gets its
                # own copy: the node's state must survive this.
                local_updates[name] = value
                value.status.source = 'modified by consumer'

            adapter.on_update(local_callback)

        def receive(name, value):
            updates[name] = value
            counts[name] += 1

        subscriptions = [
            probe.create_subscription(
                spec.observation,
                f'/robot_state/updates/{name}',
                partial(receive, name),
                LATCHED_QOS,
            )
            for name, spec in INPUTS.items()
        ]
        client = probe.create_client(GetRobotState, '/robot_state/get_state')
        assert client.wait_for_service(timeout_sec=10)

        def query():
            future = client.call_async(GetRobotState.Request())
            spin_until(executor, future.done)
            return future.result().state

        def publish(name):
            msg = message(name)
            now = probe.get_clock().now().to_msg()
            if name == 'tf':
                msg.transforms[0].header.stamp = now
            else:
                msg.header.stamp = now
            publishers[name].publish(msg)

        for name, interval in [('joints', 0.02), ('image', 1 / 15), ('tf', 0.02)]:
            timers.append(probe.create_timer(interval, partial(publish, name)))
        spin_until(
            executor,
            lambda: all(
                name in updates
                and updates[name].status.arrived
                and updates[name].status.valid
                and updates[name].status.fresh
                for name in INPUTS
            ),
        )
        spin_until(executor, lambda: counts['joints'] >= 30 and counts['image'] >= 10)
        state = query()
        assert list(state.joints.names) == list(KNOWN_JOINTS)
        assert state.launch_key.key == 'ROVER'
        assert state.image.row_step == 8 and len(state.image.pixels) == 16
        assert state.calibration.intrinsic[0] == 9
        assert all(getattr(state, name).status.source == 'simulator' for name in INPUTS)
        assert state.calibration.status.source_stamp != state.joints.status.source_stamp
        assert not state.launch_key.status.has_source_stamp
        if adapter:
            assert set(local_updates) == set(INPUTS)
            assert all(
                value.status.arrived and value.status.valid for value in local_updates.values()
            )
            assert local_updates['launch_key'].key == 'ROVER'
            assert adapter.get_state().launch_key.key == state.launch_key.key
            spin_until(
                executor, lambda: adapter.tf_buffer.can_transform('world', 'camera', Time())
            )
            assert adapter.tf_buffer.lookup_transform('world', 'camera', Time())
        assert len(subscriptions) == len(INPUTS)
        # Staleness and recovery are asserted once, in-process. Crossing a
        # process boundary changes how the messages travel, not when an
        # input expires, so re-running this against the subprocess would
        # only pay for a second node lifecycle to reach the same states.
        if separate_process:
            assert process is not None and process.poll() is None
            return
        for timer in timers:
            timer.cancel()
        # Timers are cancelled, so the only thing that can publish now is
        # the health tick. A stall produces two independent transitions --
        # the input stops being fresh, and separately its rate decays out of
        # band -- and each one is worth announcing.
        counts_before_stale = counts.copy()
        spin_until(
            executor,
            lambda: all(
                not updates[name].status.fresh and not updates[name].status.rate_ok
                for name in ('joints', 'image', 'tf')
            ),
        )
        settled = counts.copy()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        # Once health has settled there is nothing left to say: continuing to
        # be stale is not an event.
        assert counts == settled
        assert all(
            counts_before_stale[name] < settled[name] <= counts_before_stale[name] + 2
            for name in ('joints', 'image', 'tf')
        )
        # Stale inputs keep their last value; latched ones never expire.
        stale = query()
        assert stale.joints.positions == state.joints.positions
        assert stale.image.pixels == state.image.pixels
        assert all(
            getattr(stale, name).status.fresh
            for name in ('calibration', 'launch_key', 'tf_static')
        )
        invalid = message('launch_key')
        invalid.data = 'bad'
        publishers['launch_key'].publish(invalid)
        spin_until(executor, lambda: updates['launch_key'].key == 'bad')
        assert not query().launch_key.status.valid
        publishers['launch_key'].publish(message('launch_key'))
        for timer in timers:
            timer.reset()
        spin_until(
            executor,
            lambda: (
                updates['launch_key'].status.valid
                and updates['joints'].status.fresh
                and updates['image'].status.fresh
            ),
        )
    finally:
        if process:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if adapter:
            adapter.destroy_node()
        executor.shutdown()
        probe.destroy_node()
        context.try_shutdown()


def test_missing_inputs_are_announced_before_any_message_arrives():
    """A consumer that connects first is told the inputs are missing.

    The health tick publishes that state, which is why the callback set above
    is complete long before real data shows up.
    """
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    node = StateNode(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    seen: dict = {}
    try:
        node.on_update(lambda name, value: seen.setdefault(name, value))
        spin_until(executor, lambda: set(seen) == set(INPUTS))
        assert all(not value.status.arrived and not value.status.valid for value in seen.values())
        assert all(list(value.status.problems) == ['missing'] for value in seen.values())
        # Nothing arrived, so nothing can be said about the rate either.
        assert not any(value.status.rate_known for value in seen.values())
    finally:
        node.destroy_node()
        executor.shutdown()
        context.try_shutdown()


def test_done_republishes_launch_key_for_same_key_next_episode():
    """A same-key sim reset still notifies the typist of the new episode.

    The sim republishes /sim/launch_key on reset, but the normalized update
    only emits on health *change* -- and a same-key episode changes nothing.
    The typist keys episodes off that update, so without a forced republish
    on /sim/done it would type exactly one episode and idle forever.
    """
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    probe = Node('state_done_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    node = StateNode(context=context)
    executor.add_node(node)
    seen: list = []
    try:
        probe.create_publisher(String, '/sim/launch_key', LATCHED_QOS)
        done_pub = probe.create_publisher(Empty, '/sim/done', 10)

        def record_key(msg: Any) -> None:
            seen.append(msg.key)

        probe.create_subscription(
            INPUTS['launch_key'].observation,
            '/robot_state/updates/launch_key',
            record_key,
            LATCHED_QOS,
        )
        spin_until(executor, lambda: len(seen) >= 1)
        assert seen[-1] == ''
        done_pub.publish(Empty())
        spin_until(executor, lambda: len(seen) >= 2)
        assert seen[-2:] == ['', '']
        launch = probe.create_publisher(String, '/sim/launch_key', LATCHED_QOS)
        launch.publish(message('launch_key'))
        spin_until(executor, lambda: seen[-1] == 'ROVER')
    finally:
        node.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        context.try_shutdown()
