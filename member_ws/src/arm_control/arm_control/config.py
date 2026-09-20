"""The contract this node applies: joint order, limits, and control tuning.

Everything the control loop and the action server depend on lives here, the
same convention robot_state's config.py uses -- so the tuning can be reviewed
as a policy instead of reconstructed from literals scattered through the node.
"""

from typing import NamedTuple

# Must match robot_state.config.KNOWN_JOINTS. The two packages share no
# dependency for this constant, so a simulator joint rename needs both
# updated together.
KNOWN_JOINTS = (
    'base_yaw', 'shoulder_pitch', 'elbow_pitch', 'head_pan', 'head_tilt',
)
JOINT_COUNT = len(KNOWN_JOINTS)


class JointLimits(NamedTuple):
    q_min: float
    q_max: float
    v_max: float


# sim/docs/INTERFACES.md section 3.1, converted from degrees to radians.
LIMITS: dict[str, JointLimits] = {
    'base_yaw':       JointLimits(-2.0944, 2.0944, 0.6),
    'shoulder_pitch': JointLimits(-0.5236, 1.7453, 0.6),
    'elbow_pitch':    JointLimits(-2.4435, 0.0,    0.8),
    'head_pan':       JointLimits(-0.7854, 0.7854, 1.0),
    'head_tilt':      JointLimits(-0.6109, 0.6109, 1.0),
}

# Proportional position-to-velocity gain, per joint. Chosen so every joint
# saturates at v_max once its error exceeds SATURATION_ERROR_RAD: a fast
# joint and a slow joint then spend the same final stretch of travel
# decelerating, instead of the fast one slamming into tolerance while the
# slow one is still crawling in. The plant itself ramps commanded velocity
# toward the target at a_max (INTERFACES.md 3.4), so nothing here needs to
# reproduce that ramp -- shrinking the requested velocity as error shrinks is
# what keeps this from overshooting.
SATURATION_ERROR_RAD = 0.2
PROPORTIONAL_GAIN: dict[str, float] = {
    name: limits.v_max / SATURATION_ERROR_RAD for name, limits in LIMITS.items()
}

# The control loop's own rate. Far above the ~10 Hz the 0.10 s watchdog
# requires (INTERFACES.md 8.5), so normal scheduling jitter never comes close
# to lighting it, and matched to /joint_states' 50 Hz so every sample is acted
# on.
CONTROL_RATE_HZ = 50.0
CONTROL_PERIOD_SEC = 1.0 / CONTROL_RATE_HZ

# "A target inside the limits is reached within a tolerance stated in the
# README" (issue #5). One value for every joint -- none of the five is
# promised more precision than the others. This is the number the package
# README states; keep the two in sync.
POSITION_TOLERANCE_RAD = 0.01  # ~0.57 degrees

# "max|velocity| settles under 0.02 rad/s" (issue #5), verbatim.
SETTLE_VELOCITY_RAD_S = 0.02

# A single tick under both thresholds could be noise passing through zero on
# its way past the target. Holding under both for this long before calling a
# goal SETTLED is what makes "without oscillation" a real check rather than a
# lucky sample.
SETTLE_DWELL_SEC = 0.3

# Fault policy: robot_state republishes ~/updates/joints as soon as the input
# goes stale or invalid, inside its own health tick (robot_state/config.py
# HEALTH_TICK_SEC = 0.05 s) -- so watching status.fresh/status.valid on the
# messages we already subscribe to is enough. arm_control keeps no watchdog
# timer of its own on top of that.
