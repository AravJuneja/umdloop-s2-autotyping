"""Turning raw simulator messages into observations we are willing to stand behind.

Two jobs live here, deliberately separated. `normalize` is pure: it converts
one incoming message into a project-owned value and a list of everything wrong
with it, with no notion of time. `Observation` owns the time-dependent part --
how old the value is, whether it still counts as fresh, and how fast the
stream is running -- so those answers are computed against the clock at the
moment someone asks, not frozen at the moment the message landed.
"""

from collections import deque
import copy
import math
import re

from robot_state_interfaces.msg import ObservationStatus, Transform

from . import config


def nanoseconds(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def seconds(stamp):
    return nanoseconds(stamp) / 1e9


def finite(values):
    return all(math.isfinite(v) for v in values)


def valid_stamp(stamp):
    """Reject stamps that cannot be read as a time at all.

    A negative second or an overflowed nanosecond field means the sender's
    clock is broken, and arithmetic on it would produce an age we would then
    report as if it meant something.
    """
    return stamp.sec >= 0 and stamp.nanosec < 10**9


def normalize(name, msg):
    """Convert one simulator message into (value, source stamp, problems).

    The value is always populated as far as it can be, even when problems are
    found, so a consumer can inspect what actually arrived. The stamp is the
    source's own idea of when the data was captured, or None for inputs that
    carry no timestamp. An empty problems list is what makes a value valid.
    """
    problems = []
    value = config.INPUTS[name].observation()
    stamp = None
    if name == 'joints':
        value.frame_id = msg.header.frame_id
        stamp = msg.header.stamp
        # Set equality plus the length check means each known joint appears
        # exactly once, which is what makes the reorder below safe.
        if (len(msg.name) != config.JOINT_COUNT
                or set(msg.name) != set(config.KNOWN_JOINTS)):
            problems.append('joint names must contain each known joint once')
        # velocity is optional in JointState; present means it must line up
        # with the names, absent means we publish nothing for it. effort is
        # deliberately not checked -- we do not republish it, and rejecting a
        # message over a field we discard would throw away good positions.
        if (len(msg.position) != len(msg.name)
                or len(msg.velocity) not in (0, len(msg.name))):
            problems.append('joint array lengths do not match names')
        if not finite((*msg.position, *msg.velocity)):
            problems.append('non-finite joint data')
        if not problems:
            # Republish in KNOWN_JOINTS order so consumers can index by
            # position without trusting the simulator's ordering.
            order = [msg.name.index(joint) for joint in config.KNOWN_JOINTS]
            value.names = list(config.KNOWN_JOINTS)
            value.positions = [msg.position[i] for i in order]
            value.velocities = [msg.velocity[i] for i in order] if msg.velocity else []
    elif name == 'image':
        stamp = msg.header.stamp
        value.frame_id = msg.header.frame_id
        value.width, value.height = msg.width, msg.height
        value.encoding, value.row_step = msg.encoding, msg.step
        value.pixels = msg.data
        if msg.encoding != config.IMAGE_ENCODING:
            problems.append(f'supported image encoding is {config.IMAGE_ENCODING}')
        # step is the stride in bytes, which may exceed width*channels when
        # rows are padded; the buffer has to match height*step either way.
        if (not msg.width or not msg.height
                or msg.step < msg.width * config.IMAGE_CHANNELS
                or len(msg.data) != msg.height * msg.step):
            problems.append('invalid image dimensions, stride, or byte count')
        if msg.is_bigendian not in (0, 1):
            problems.append('invalid image endian flag')
        # Without a frame the pixels cannot be related to anything in TF.
        if not msg.header.frame_id:
            problems.append('missing image frame')
    elif name == 'calibration':
        stamp = msg.header.stamp
        value.frame_id = msg.header.frame_id
        value.width, value.height = msg.width, msg.height
        value.distortion_model, value.distortion = msg.distortion_model, msg.d
        value.intrinsic = msg.k
        # K is [fx 0 cx; 0 fy cy; 0 0 1]: focal lengths must be positive, the
        # principal point must land inside the image, and the scale term must
        # be 1, or projecting a pixel into the world produces nonsense.
        if (not msg.width or not msg.height or not msg.header.frame_id
                or not finite((*msg.k, *msg.d))
                or msg.k[0] <= 0 or msg.k[4] <= 0
                or abs(msg.k[8] - 1.0) > config.INTRINSIC_SCALE_TOLERANCE
                or not 0 <= msg.k[2] < msg.width or not 0 <= msg.k[5] < msg.height):
            problems.append('invalid calibration dimensions, frame, or matrices')
        if (msg.distortion_model != config.DISTORTION_MODEL
                or len(msg.d) != config.DISTORTION_COEFFICIENTS):
            problems.append(
                f'supported calibration is {config.DISTORTION_MODEL} with '
                f'{config.DISTORTION_COEFFICIENTS} coefficients')
    elif name == 'launch_key':
        value.key = msg.data
        if re.fullmatch(config.LAUNCH_KEY_PATTERN, msg.data) is None:
            problems.append('launch key must be 3-6 ASCII uppercase letters or digits')
    elif name in config.TF_INPUTS:
        # A batch is accepted transform by transform: one bad entry is
        # reported and dropped, the rest still reach consumers.
        children = set()
        if not msg.transforms:
            problems.append('empty transform batch')
        for item in msg.transforms:
            t, q = item.transform.translation, item.transform.rotation
            xyz, xyzw = [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
            # A frame parented to itself, or two transforms claiming the same
            # child, would make the TF tree ambiguous.
            if (not item.header.frame_id or not item.child_frame_id
                    or item.header.frame_id == item.child_frame_id
                    or item.child_frame_id in children
                    or not finite((*xyz, *xyzw))
                    or abs(sum(v * v for v in xyzw) - 1.0)
                    > config.QUATERNION_NORM_TOLERANCE):
                problems.append('invalid transform frames, translation, or quaternion')
                continue
            if not valid_stamp(item.header.stamp):
                problems.append('invalid transform timestamp')
                continue
            children.add(item.child_frame_id)
            value.transforms.append(Transform(
                source_stamp=copy.deepcopy(item.header.stamp),
                parent_frame=item.header.frame_id, child_frame=item.child_frame_id,
                translation=xyz, rotation_xyzw=xyzw,
            ))
        # The batch is only as fresh as its oldest accepted transform, and a
        # rejected one must not be allowed to make the batch look newer.
        if value.transforms:
            stamp = min((t.source_stamp for t in value.transforms), key=seconds)
    else:
        raise ValueError(f'no normalizer for input {name!r}')
    return value, stamp, problems


class Observation:
    """The latest value for one input, plus the history needed to judge it.

    Holds the last message we accepted (valid or not -- a malformed message is
    still the truth about what the simulator is publishing) and the arrival
    times behind it. Nothing here is time-dependent until `status` is called.
    """

    def __init__(self, name):
        spec = config.INPUTS[name]
        self._name = name
        latched = spec.freshness_sec is None
        self.value = spec.observation(status=ObservationStatus(
            source=config.SOURCE, latched=latched,
            freshness_sec=0.0 if latched else spec.freshness_sec,
            expected_rate_hz=spec.expected_rate_hz,
        ))
        self.arrivals: deque[float] = deque(maxlen=config.ARRIVAL_HISTORY)
        self.problems = []

    def update(self, msg, now):
        """Replace the stored value with a newly arrived message.

        The new message wins even when it is invalid: reporting the current
        bad data with valid=False is more useful than silently serving a good
        value the simulator has already moved on from.
        """
        value, stamp, problems = normalize(self._name, msg)
        status = copy.deepcopy(self.value.status)
        status.arrived = True
        status.received_stamp = copy.deepcopy(now)
        status.has_source_stamp = stamp is not None
        if stamp is not None:
            status.source_stamp = copy.deepcopy(stamp)
            if not valid_stamp(stamp):
                problems.append('invalid source timestamp')
            if seconds(stamp) > seconds(now) + config.CLOCK_SKEW_TOLERANCE_SEC:
                problems.append('source timestamp is in the future')
        # A clock that jumped backwards (a sim reset) invalidates every
        # interval in the history, so the rate estimate restarts from here.
        if self.arrivals and seconds(now) < self.arrivals[-1]:
            self.arrivals.clear()
        self.arrivals.append(seconds(now))
        value.status = status
        self.value = copy.deepcopy(value)
        self.problems = problems

    def status(self, now):
        """Judge the stored value against the clock as of `now`.

        Three separable questions, kept as separate fields: did anything ever
        arrive (`arrived`), is what arrived recent enough to act on (`fresh`),
        and was it well-formed (`valid`).
        """
        s = copy.deepcopy(self.value.status)
        if not s.arrived:
            s.problems = ['missing']
            return s
        now_ns = nanoseconds(now)
        s.received_age_sec = (now_ns - nanoseconds(s.received_stamp)) / 1e9
        # Age is measured from the source stamp when there is one, so a
        # message delayed in transit is not mistaken for current data.
        s.age_sec = (now_ns - nanoseconds(
            s.source_stamp if s.has_source_stamp else s.received_stamp)) / 1e9
        # Both ages must be inside the window: a message that just arrived is
        # still stale if the data inside it is old. Latched inputs never expire.
        s.fresh = (s.received_age_sec >= 0
                   and s.age_sec >= -config.CLOCK_SKEW_TOLERANCE_SEC
                   and (s.latched
                        or max(s.age_sec, s.received_age_sec) <= s.freshness_sec))
        problems = list(self.problems)
        if (s.received_age_sec < 0
                or s.age_sec < -config.CLOCK_SKEW_TOLERANCE_SEC):
            problems.append('clock moved behind observation')
        s.rate_known, s.rate_hz = self._rate(now_ns / 1e9)
        # An input with no expected rate is always rate_ok; there is nothing
        # to be wrong about.
        #
        # Rate deliberately does not feed `valid`. Issue #3 asks both that
        # validity cover expected update rates and that old observations stay
        # available and be marked stale, and those cannot both hold: a stalled
        # input has a bad rate, so folding rate in would mark every stale
        # observation invalid and `valid` would stop telling malformed data
        # apart from a quiet publisher. The rate is still checked here and
        # reported with its measured value, just not as a validity failure.
        s.rate_ok = s.expected_rate_hz == 0 or (
            s.rate_known and
            config.RATE_TOLERANCE_LOW * s.expected_rate_hz <= s.rate_hz
            <= config.RATE_TOLERANCE_HIGH * s.expected_rate_hz)
        s.valid = not problems
        s.problems = problems
        return s

    def snapshot(self, now):
        """Return a fully owned copy, so a consumer cannot mutate our state."""
        result = copy.deepcopy(self.value)
        result.status = self.status(now)
        return result

    def _rate(self, now_sec):
        """Estimate arrivals per second over the trailing window.

        Counting over a fixed window rather than over the span of the messages
        themselves is what makes a stalled publisher visible: as its arrivals
        age out, the count falls while the divisor stays put, so the rate
        decays to zero instead of holding at its last healthy value.
        """
        if not self.arrivals:
            return False, 0.0
        window_start = now_sec - config.RATE_WINDOW_SEC
        recent = [t for t in self.arrivals if window_start <= t <= now_sec]
        if self.arrivals[0] > window_start:
            # Still filling the window, so measure from the first message we
            # ever saw. It marks the start of the period rather than an
            # arrival inside it, so it is not counted.
            observed_since, counted = self.arrivals[0], len(recent) - 1
        else:
            observed_since, counted = window_start, len(recent)
        observed_sec = now_sec - observed_since
        # Too short a period makes the estimate meaningless in both
        # directions: one early message would read as a huge rate, and a
        # stream one message old would read as dead.
        if observed_sec < config.RATE_MIN_OBSERVATION_SEC:
            return False, 0.0
        return True, max(counted, 0) / observed_sec
