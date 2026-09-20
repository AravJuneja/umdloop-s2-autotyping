"""The control law, exercised as pure functions -- no ROS graph involved.

Every joint constant used here comes straight from arm_control.config, so a
tuning change is reflected automatically instead of drifting out of sync with
a copy pasted into the test.
"""

from arm_control import config, control
import pytest


@pytest.mark.parametrize('name', config.KNOWN_JOINTS)
def test_command_velocity_saturates_at_v_max(name):
    limits = config.LIMITS[name]
    # An error far beyond SATURATION_ERROR_RAD must clip to v_max, in sign
    # too -- overshooting the joint's own speed limit would violate "within
    # its ... velocity limits" regardless of how far off target it is.
    assert control.command_velocity(name, limits.q_max, limits.q_min) == limits.v_max
    assert control.command_velocity(name, limits.q_min, limits.q_max) == -limits.v_max


@pytest.mark.parametrize('name', config.KNOWN_JOINTS)
def test_command_velocity_is_zero_on_target(name):
    limits = config.LIMITS[name]
    midpoint = (limits.q_min + limits.q_max) / 2
    assert control.command_velocity(name, midpoint, midpoint) == 0.0


@pytest.mark.parametrize('name', config.KNOWN_JOINTS)
def test_command_velocity_sign_matches_error_direction(name):
    limits = config.LIMITS[name]
    midpoint = (limits.q_min + limits.q_max) / 2
    assert control.command_velocity(name, midpoint + 0.05, midpoint) > 0
    assert control.command_velocity(name, midpoint - 0.05, midpoint) < 0


def test_within_tolerance_boundary():
    tol = config.POSITION_TOLERANCE_RAD
    assert control.within_tolerance([tol, -tol, 0.0])
    assert not control.within_tolerance([tol + 1e-9, 0.0])


def test_slow_enough_boundary():
    limit = config.SETTLE_VELOCITY_RAD_S
    assert control.slow_enough([limit, -limit, 0.0])
    assert not control.slow_enough([limit + 1e-9, 0.0])


def test_validate_goal_accepts_home_pose():
    assert control.validate_goal(
        list(config.KNOWN_JOINTS), [0.0, 1.4835, -1.9722, 0.0, 0.0]) == []


def test_validate_goal_rejects_missing_joint():
    names = list(config.KNOWN_JOINTS)[:-1]
    problems = control.validate_goal(names, [0.0] * len(names))
    assert problems and 'exactly once' in problems[0]


def test_validate_goal_rejects_duplicate_joint():
    names = [config.KNOWN_JOINTS[0]] + list(config.KNOWN_JOINTS[1:-1]) + [config.KNOWN_JOINTS[0]]
    problems = control.validate_goal(names, [0.0] * len(names))
    assert problems and 'exactly once' in problems[0]


def test_validate_goal_rejects_length_mismatch():
    names = list(config.KNOWN_JOINTS)
    problems = control.validate_goal(names, [0.0] * (len(names) - 1))
    assert problems and 'does not match' in problems[0]


def test_validate_goal_rejects_out_of_limits():
    names = list(config.KNOWN_JOINTS)
    positions = [0.0] * len(names)
    positions[0] = config.LIMITS[names[0]].q_max + 1.0
    problems = control.validate_goal(names, positions)
    assert any('outside' in p for p in problems)


def test_validate_goal_rejects_non_finite():
    names = list(config.KNOWN_JOINTS)
    positions = [0.0] * len(names)
    positions[2] = float('nan')
    problems = control.validate_goal(names, positions)
    assert any('non-finite' in p for p in problems)
