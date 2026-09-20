"""Validation and freshness rules, exercised without a running ROS graph.

These drive `Observation` directly with a synthetic clock, so a test can
step time forward by ten thousand seconds and assert what a latched input
does, which is not something a wall-clock integration test can pin down.
"""

import copy

from builtin_interfaces.msg import Time
from conftest import message, stamp
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from robot_state.config import INPUTS, KNOWN_JOINTS
from robot_state.observation import Observation
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def observe(name, msg=None, now=10.0):
    observation = Observation(name)
    observation.update(message(name) if msg is None else msg, stamp(now))
    return observation


@pytest.mark.parametrize('name', INPUTS)
def test_missing(name):
    status = Observation(name).snapshot(stamp()).status
    assert not status.arrived and not status.fresh and not status.valid
    assert status.age_sec == -1 and status.received_age_sec == -1
    assert status.problems == ['missing']


@pytest.mark.parametrize('name', INPUTS)
def test_round_trip_and_independent_snapshots(name):
    msg = message(name)
    observation = observe(name, msg)
    value = observation.snapshot(stamp())
    assert value.status.valid and value.status.fresh and value.status.arrived
    assert deserialize_message(serialize_message(value), type(value)) == value
    value.status.valid = False
    assert observation.snapshot(stamp()).status.valid
    assert observation.snapshot(stamp(11)).status.received_age_sec == 1


@pytest.mark.parametrize('name', ['joints', 'image', 'tf'])
def test_stale_retains_value_and_source_stamp(name):
    """Going stale must not discard the data, only relabel it.

    A consumer deciding whether to act on old data needs to see the data.
    """
    observation = observe(name)
    fresh = observation.snapshot(stamp())
    stale = observation.snapshot(stamp(11))
    assert stale.status.arrived and stale.status.valid and not stale.status.fresh
    assert stale.status.source_stamp == stamp()
    assert stale.status.age_sec == 1.0
    stale.status = fresh.status
    assert stale == fresh


@pytest.mark.parametrize('name', ['calibration', 'launch_key', 'tf_static'])
def test_latched_values_do_not_expire(name):
    status = observe(name).snapshot(stamp(10000)).status
    assert status.latched and status.fresh


@pytest.mark.parametrize('name', ['joints', 'image', 'tf'])
def test_windowed_inputs_are_not_latched(name):
    status = observe(name).snapshot(stamp()).status
    assert not status.latched and status.freshness_sec > 0


def test_unstamped_key_uses_receive_time_without_inventing_source_stamp():
    status = observe('launch_key').snapshot(stamp(11)).status
    assert not status.has_source_stamp
    assert status.source_stamp == Time()
    assert status.age_sec == 1.0


def test_joints_reordered_by_name_and_owned():
    """Positions follow KNOWN_JOINTS order, and are copied out of the input."""
    msg = message('joints')
    msg.name.reverse()
    observation = observe('joints', msg)
    msg.position[0] = 100.0
    value = observation.snapshot(stamp())
    assert list(value.names) == list(KNOWN_JOINTS)
    assert list(value.positions) == [4.0, 3.0, 2.0, 1.0, 0.0]
    assert list(value.velocities) == [9.0, 8.0, 7.0, 6.0, 5.0]


@pytest.mark.parametrize(
    'field,value',
    [
        ('name', ['unknown'] * 5),
        ('name', [KNOWN_JOINTS[0]] * 5),
        ('position', [0.0]),
        ('velocity', [0.0]),
        ('position', [float('nan')] * 5),
        ('velocity', [float('inf')] * 5),
    ],
)
def test_invalid_joints(field, value):
    msg = message('joints')
    setattr(msg, field, value)
    result = observe('joints', msg).snapshot(stamp())
    assert not result.status.valid
    assert not result.positions


def test_optional_joint_arrays():
    msg = message('joints')
    msg.velocity = []
    assert observe('joints', msg).snapshot(stamp()).status.valid


def test_garbage_effort_does_not_invalidate_positions():
    """Effort is not republished, so it does not get to reject a message."""
    msg = message('joints')
    msg.effort = [float('nan')] * 3
    result = observe('joints', msg).snapshot(stamp())
    assert result.status.valid
    assert list(result.positions) == [0.0, 1.0, 2.0, 3.0, 4.0]


@pytest.mark.parametrize(
    'field,value',
    [
        ('encoding', 'rgb8'),
        ('width', 0),
        ('height', 0),
        ('step', 5),
        ('data', b'123'),
        ('is_bigendian', 2),
    ],
)
def test_invalid_image(field, value):
    msg = message('image')
    setattr(msg, field, value)
    assert not observe('image', msg).snapshot(stamp()).status.valid


def test_padded_bgr_image_preserved():
    value = observe('image').snapshot(stamp())
    assert value.row_step == 8 and bytes(value.pixels) == bytes(range(16))


@pytest.mark.parametrize('name', ['image', 'calibration'])
def test_camera_frame_required(name):
    msg = message(name)
    msg.header.frame_id = ''
    assert not observe(name, msg).snapshot(stamp()).status.valid


@pytest.mark.parametrize(
    'field,value',
    [
        ('width', 0),
        ('distortion_model', 'unknown'),
        ('d', []),
        ('k', [float('nan')] * 9),
        ('k', [0.0] * 9),
    ],
)
def test_invalid_calibration(field, value):
    msg = message('calibration')
    setattr(msg, field, value)
    assert not observe('calibration', msg).snapshot(stamp()).status.valid


@pytest.mark.parametrize('key', ['', 'AB', 'ABCDEFG', 'abc', 'A_B', 'ÄBC', 'ABC\n'])
def test_invalid_key(key):
    assert not observe('launch_key', String(data=key)).snapshot(stamp()).status.valid


@pytest.mark.parametrize('key', ['ABC', 'A12Z90', '123'])
def test_valid_key(key):
    assert observe('launch_key', String(data=key)).snapshot(stamp()).status.valid


def test_transform_validation_and_each_timestamp():
    msg = message('tf')
    other = copy.deepcopy(msg.transforms[0])
    other.child_frame_id = 'head'
    other.header.stamp = stamp(9.99)
    msg.transforms.append(other)
    value = observe('tf', msg).snapshot(stamp())
    assert value.status.valid and value.status.source_stamp == stamp(9.99)
    assert value.transforms[0].source_stamp == stamp()
    msg.transforms[1].transform.rotation.w = 0.0
    invalid = observe('tf', msg).snapshot(stamp())
    assert not invalid.status.valid
    assert invalid.status.source_stamp == stamp()
    assert len(invalid.transforms) == 1
    assert not observe('tf', TFMessage()).snapshot(stamp()).status.valid


def test_delayed_future_and_backward_clock():
    assert not observe('joints', now=11).snapshot(stamp(11)).status.fresh
    assert not observe('joints', now=9).snapshot(stamp(9)).status.valid
    assert not observe('joints').snapshot(stamp(9)).status.fresh
    assert not observe('joints').snapshot(stamp(9)).status.valid
    # Running at 5 Hz instead of 50 is a stream problem, not a data problem:
    # the positions in the message are still exactly what was sent.


def test_freshness_boundary():
    observation = observe('joints')
    assert observation.snapshot(stamp(10.15)).status.fresh
    assert not observation.snapshot(stamp(10.150001)).status.fresh


def test_update_rate_and_recovery():
    """Rate health tracks the stream: good, then degraded, then good again.

    The recovery half matters most -- a rate problem has to clear on its
    own once the publisher catches up, without restarting the node.
    """
    observation = Observation('joints')
    for i in range(60):
        msg = message('joints')
        msg.header.stamp = stamp(10 + i * 0.02)
        observation.update(msg, msg.header.stamp)
    status = observation.snapshot(stamp(11.18)).status
    assert status.rate_known and status.rate_ok and status.valid
    assert status.rate_hz == pytest.approx(50)
    # Long after the stream died the rate is known, and known to be zero.
    dead = observation.snapshot(stamp(20)).status
    assert dead.rate_known and dead.rate_hz == 0 and not dead.rate_ok
    for i in range(3):
        msg.header.stamp = stamp(20 + i * 0.2)
        observation.update(msg, msg.header.stamp)
    # Running at 5 Hz instead of 50 is a stream problem, not a data problem:
    # the positions in the message are still exactly what was sent.
    status = observation.snapshot(stamp(20.4)).status
    assert status.fresh and status.valid and not status.rate_ok
    assert status.problems == []
    for i in range(120):
        msg.header.stamp = stamp(21 + i * 0.02)
        observation.update(msg, msg.header.stamp)
    recovered = observation.snapshot(msg.header.stamp).status
    assert recovered.valid and recovered.rate_ok


def test_rate_decays_while_the_publisher_is_silent():
    """A stalled stream must not keep reporting its last healthy rate."""
    observation = Observation('joints')
    for i in range(50):
        msg = message('joints')
        msg.header.stamp = stamp(10 + i * 0.02)
        observation.update(msg, msg.header.stamp)
    # Nothing arrives after 10.98.
    assert observation.snapshot(stamp(11)).status.rate_hz == pytest.approx(50, rel=0.05)
    decaying = [observation.snapshot(stamp(t)).status.rate_hz for t in (11.5, 12.0, 12.5)]
    assert decaying == sorted(decaying, reverse=True)
    assert decaying[-1] < 25


def test_duplicate_receive_times_do_not_divide_by_zero():
    observation = Observation('joints')
    observation.update(message('joints'), stamp())
    observation.update(message('joints'), stamp())
    status = observation.snapshot(stamp()).status
    assert not status.rate_known
    assert status.rate_hz == 0


def test_invalid_latest_replaces_previous_value():
    """A bad message wins over a good older one, flagged rather than hidden."""
    observation = observe('launch_key')
    observation.update(String(data='bad'), stamp(11))
    result = observation.snapshot(stamp(11))
    assert result.key == 'bad' and not result.status.valid
    assert result.status.arrived and result.status.fresh
