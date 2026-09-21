"""The contract between the simulator and this node, in one place.

Every threshold the node applies to incoming data lives here: which topics we
read, how long each input stays useful, how fast we expect it to arrive, and
how much slop we forgive in the numbers. Keeping them together means the
freshness and validation policy can be reviewed as a policy, instead of being
reconstructed from literals scattered through the callbacks.
"""

from typing import NamedTuple

from interfaces.msg import (
    CalibrationObservation,
    ImageObservation,
    JointObservation,
    LaunchKeyObservation,
    TransformObservation,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# Every observation we publish is derived from the simulator, and says so, so
# a consumer reading a bag can tell our normalized view from a real robot's.
SOURCE = 'simulator'

# The arm exposes exactly these five joints. We republish them in this order
# regardless of the order /joint_states happens to use, so downstream indexing
# is stable; a message naming anything else is rejected rather than reordered.
KNOWN_JOINTS = (
    'base_yaw',
    'shoulder_pitch',
    'elbow_pitch',
    'head_pan',
    'head_tilt',
)
JOINT_COUNT = len(KNOWN_JOINTS)

# Inputs whose value is published once and stays true for the rest of the
# episode. A freshness window of None marks them -- spelled out rather than
# encoded as a negative number, because -1.0 already means "not computed" on
# the age fields and one sentinel should not carry two meanings.
LATCHED = None
NO_EXPECTED_RATE = 0.0
TF_INPUTS = ('tf', 'tf_static')

# --- QoS ------------------------------------------------------------------
# Latched: one sample, delivered to whoever subscribes later. Used both for
# the simulator's write-once topics and for our own ~/updates publishers, so a
# node that starts late still sees the current state instead of waiting for
# the next message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Camera frames: newest wins, dropping is better than queueing.
SENSOR = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# TF batches arrive faster than we consume them and each one matters, so the
# queue is deep enough to absorb a burst without dropping a transform.
TF_DYNAMIC = QoSProfile(depth=100)
TF_STATIC = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class InputSpec(NamedTuple):
    """Everything the node needs to know about one simulator input.

    One entry per input, so a new input cannot be half-registered: the
    subscription, the observation type, and the health policy all come from
    the same record.
    """

    observation: type  # the project-owned message we publish
    ros_type: type  # the message type the simulator publishes
    topic: str  # the topic we subscribe to
    qos: QoSProfile | int
    freshness_sec: float | None  # None means latched for the episode
    expected_rate_hz: float  # 0 means rate health is not meaningful


# Freshness windows are set at roughly three missed messages, which is long
# enough to ride out normal jitter and short enough that a stalled publisher
# is reported before a consumer acts on dead data. TF batches carry the
# oldest-accepted transform's stamp (observation.py), and the sim's TF clock
# can lag the local clock by ~0.1 s under load -- so TF's window must cover
# that skew plus missed messages, or every goal faults mid-execution.
INPUTS: dict[str, InputSpec] = {
    'joints': InputSpec(JointObservation, JointState, '/joint_states', 10, 0.15, 50.0),
    'image': InputSpec(ImageObservation, Image, '/camera/image_raw', SENSOR, 0.25, 15.0),
    'calibration': InputSpec(
        CalibrationObservation,
        CameraInfo,
        '/camera/camera_info',
        LATCHED_QOS,
        LATCHED,
        NO_EXPECTED_RATE,
    ),
    'launch_key': InputSpec(
        LaunchKeyObservation, String, '/sim/launch_key', LATCHED_QOS, LATCHED, NO_EXPECTED_RATE
    ),
    'tf': InputSpec(TransformObservation, TFMessage, '/tf', TF_DYNAMIC, 0.5, 50.0),
    'tf_static': InputSpec(
        TransformObservation, TFMessage, '/tf_static', TF_STATIC, LATCHED, NO_EXPECTED_RATE
    ),
}

# --- Timing tolerances ----------------------------------------------------
# The simulator stamps messages on its own clock. A few milliseconds of skew
# against ours is normal; more than this means the clocks genuinely disagree
# and the age we would report is not trustworthy.
CLOCK_SKEW_TOLERANCE_SEC = 0.05

# Arrivals are counted over this trailing window and divided by it, so a
# publisher that stops decays towards zero instead of reporting its last
# healthy number until the window empties. Short enough that a stall shows up
# within a few health ticks, long enough that one late message does not swing
# the estimate.
RATE_WINDOW_SEC = 2.0

# We refuse to report a rate until we have been listening this long. Without
# it, the first message to arrive would divide by a near-zero period and claim
# an absurd rate, and a stream one message old would be called dead.
RATE_MIN_OBSERVATION_SEC = 0.1

# A stream counts as healthy between half and 1.5x its expected rate. Wide,
# because we are reporting "is this publisher alive and roughly on schedule",
# not measuring jitter.
RATE_TOLERANCE_LOW = 0.5
RATE_TOLERANCE_HIGH = 1.5

# Arrival timestamps retained per input. RATE_WINDOW_SEC at the fastest
# expected rate needs 100; the rest is headroom so a burst cannot evict
# samples that are still inside the window.
ARRIVAL_HISTORY = 500

# How often health is recomputed for inputs that are not currently receiving
# messages. This is the worst-case delay between an input going stale and us
# saying so, so it wants to be well under the shortest freshness window.
HEALTH_TICK_SEC = 0.05
LOG_TICK_SEC = 5.0

# --- Value tolerances -----------------------------------------------------
# Quaternions arrive rounded through float32, so exact unit norm is not
# reachable. Loose enough for that rounding, tight enough to reject an
# all-zero or garbage rotation.
QUATERNION_NORM_TOLERANCE = 0.001

# K[8] is the homogeneous scale term of the camera matrix and should be 1.
# It is computed rather than typed in, so it arrives as ~1.0, not exactly 1.0.
INTRINSIC_SCALE_TOLERANCE = 1e-6

# --- Accepted formats -----------------------------------------------------
# We normalize one image encoding and one distortion model rather than
# converting: anything else is a simulator change we want to hear about.
IMAGE_ENCODING = 'bgr8'
IMAGE_CHANNELS = 3
DISTORTION_MODEL = 'plumb_bob'
DISTORTION_COEFFICIENTS = 5

# The launch key is short, uppercase, and alphanumeric. Anchored so a key with
# trailing whitespace or a newline is rejected rather than silently trimmed.
LAUNCH_KEY_PATTERN = r'[A-Z0-9]{3,6}'

# The pair the periodic log resolves to prove the TF tree connects end to end.
# Simulator-specific, so it belongs here rather than inline in the log call.
TF_LOOKUP_CHECK = ('world', 'camera_optical_frame')
