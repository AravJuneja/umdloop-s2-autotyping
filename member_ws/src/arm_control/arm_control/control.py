"""The control law, kept pure: numbers in, numbers out, no ROS and no clock.

Turning a target and the current state into a command is a function of five
numbers per joint; keeping it here, independent of the node, means it can be
tested by handing it numbers instead of standing up a subscription and a
publisher.
"""

from . import config


def clip(value, low, high):
    return max(low, min(high, value))


def command_velocity(name, target, position):
    """Compute the proportional velocity command for one joint, clipped to v_max."""
    limits = config.LIMITS[name]
    error = target - position
    return clip(config.PROPORTIONAL_GAIN[name] * error, -limits.v_max, limits.v_max)


def within_tolerance(errors):
    """Check that every joint's position error is inside POSITION_TOLERANCE_RAD."""
    return all(abs(e) <= config.POSITION_TOLERANCE_RAD for e in errors)


def slow_enough(velocities):
    """Check that every joint's speed is under SETTLE_VELOCITY_RAD_S."""
    return all(abs(v) <= config.SETTLE_VELOCITY_RAD_S for v in velocities)


def in_limits(name, position):
    limits = config.LIMITS[name]
    return limits.q_min <= position <= limits.q_max


def validate_goal(name, position):
    """Everything wrong with a requested pose, or an empty list if none.

    Mirrors robot_state.observation.normalize's shape: a name/position pair
    for each of the five known joints, each appearing exactly once, every
    value finite and inside that joint's hard stops.
    """
    problems = []
    if len(name) != config.JOINT_COUNT or set(name) != set(config.KNOWN_JOINTS):
        problems.append('goal must name each known joint exactly once')
        return problems
    if len(position) != len(name):
        problems.append('goal position array does not match name array')
        return problems
    for joint_name, joint_position in zip(name, position):
        if joint_position != joint_position or joint_position in (float('inf'), float('-inf')):
            problems.append(f'{joint_name}: non-finite target position')
        elif not in_limits(joint_name, joint_position):
            limits = config.LIMITS[joint_name]
            problems.append(
                f'{joint_name}: target {joint_position:.4f} rad outside '
                f'[{limits.q_min:.4f}, {limits.q_max:.4f}]'
            )
    return problems
