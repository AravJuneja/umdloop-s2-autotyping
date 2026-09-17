from collections import deque
import copy
import math
import re

from robot_state_interfaces.msg import (
    CalibrationObservation, ImageObservation, JointObservation,
    LaunchKeyObservation, ObservationStatus, Transform, TransformObservation,
)

KNOWN_JOINTS = (
    'base_yaw', 'shoulder_pitch', 'elbow_pitch', 'head_pan', 'head_tilt',
)
# Message type, source topic, freshness window, and expected rate. A negative
# freshness window means the latched value remains useful for the episode.
INPUTS = {
    'joints': (JointObservation, '/joint_states', 0.15, 50.0),
    'image': (ImageObservation, '/camera/image_raw', 0.25, 15.0),
    'calibration': (CalibrationObservation, '/camera/camera_info', -1.0, 0.0),
    'launch_key': (LaunchKeyObservation, '/sim/launch_key', -1.0, 0.0),
    'tf': (TransformObservation, '/tf', 0.15, 50.0),
    'tf_static': (TransformObservation, '/tf_static', -1.0, 0.0),
}


def nanoseconds(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def seconds(stamp):
    return nanoseconds(stamp) / 1e9


def finite(values):
    return all(math.isfinite(v) for v in values)


def normalize(name, msg):
    problems = []
    value = INPUTS[name][0]()
    stamp = None
    if name == 'joints':
        value.frame_id = msg.header.frame_id
        stamp = msg.header.stamp
        if len(msg.name) != 5 or set(msg.name) != set(KNOWN_JOINTS):
            problems.append('joint names must contain each known joint once')
        if (len(msg.position) != len(msg.name)
                or len(msg.velocity) not in (0, len(msg.name))
                or len(msg.effort) not in (0, len(msg.name))):
            problems.append('joint array lengths do not match names')
        if not finite((*msg.position, *msg.velocity, *msg.effort)):
            problems.append('non-finite joint data')
        if not problems:
            order = [msg.name.index(joint) for joint in KNOWN_JOINTS]
            value.names = list(KNOWN_JOINTS)
            value.positions = [msg.position[i] for i in order]
            value.velocities = [msg.velocity[i] for i in order] if msg.velocity else []
    elif name == 'image':
        stamp = msg.header.stamp
        value.frame_id = msg.header.frame_id
        value.width, value.height = msg.width, msg.height
        value.encoding, value.row_step = msg.encoding, msg.step
        value.pixels = msg.data
        if msg.encoding != 'bgr8':
            problems.append('supported image encoding is bgr8')
        if (not msg.width or not msg.height or msg.step < msg.width * 3
                or len(msg.data) != msg.height * msg.step):
            problems.append('invalid image dimensions, stride, or byte count')
        if msg.is_bigendian not in (0, 1):
            problems.append('invalid image endian flag')
        if not msg.header.frame_id:
            problems.append('missing image frame')
    elif name == 'calibration':
        stamp = msg.header.stamp
        value.frame_id = msg.header.frame_id
        value.width, value.height = msg.width, msg.height
        value.distortion_model, value.distortion = msg.distortion_model, msg.d
        value.intrinsic = msg.k
        if (not msg.width or not msg.height or not msg.header.frame_id
                or not finite((*msg.k, *msg.d))
                or msg.k[0] <= 0 or msg.k[4] <= 0 or msg.k[8] != 1
                or not 0 <= msg.k[2] < msg.width or not 0 <= msg.k[5] < msg.height):
            problems.append('invalid calibration dimensions, frame, or matrices')
        if msg.distortion_model != 'plumb_bob' or len(msg.d) != 5:
            problems.append('supported calibration is plumb_bob with five coefficients')
    elif name == 'launch_key':
        value.key = msg.data
        if re.fullmatch(r'[A-Z0-9]{3,6}', msg.data) is None:
            problems.append('launch key must be 3-6 ASCII uppercase letters or digits')
    else:
        children = set()
        if not msg.transforms:
            problems.append('empty transform batch')
        for item in msg.transforms:
            t, q = item.transform.translation, item.transform.rotation
            xyz, xyzw = [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
            if (not item.header.frame_id or not item.child_frame_id
                    or item.header.frame_id == item.child_frame_id
                    or item.child_frame_id in children
                    or not finite((*xyz, *xyzw))
                    or abs(sum(v * v for v in xyzw) - 1.0) > 0.001):
                problems.append('invalid transform frames, translation, or quaternion')
                continue
            if item.header.stamp.sec < 0 or item.header.stamp.nanosec >= 10**9:
                problems.append('invalid transform timestamp')
                continue
            children.add(item.child_frame_id)
            value.transforms.append(Transform(
                source_stamp=copy.deepcopy(item.header.stamp),
                parent_frame=item.header.frame_id, child_frame=item.child_frame_id,
                translation=xyz, rotation_xyzw=xyzw,
            ))
        if value.transforms:
            stamp = min((t.source_stamp for t in value.transforms), key=seconds)
    return value, stamp, problems


class Observation:
    def __init__(self, name):
        value_type, _, expiry, rate = INPUTS[name]
        self._name = name
        self.value = value_type(status=ObservationStatus(
            source='simulator', freshness_sec=expiry, expected_rate_hz=rate,
        ))
        self.arrivals: deque[float] = deque(maxlen=500)
        self.problems = []

    def update(self, msg, now):
        value, stamp, problems = normalize(self._name, msg)
        status = copy.deepcopy(self.value.status)
        status.arrived = True
        status.received_stamp = copy.deepcopy(now)
        status.has_source_stamp = stamp is not None
        if stamp is not None:
            status.source_stamp = copy.deepcopy(stamp)
            if stamp.sec < 0 or stamp.nanosec >= 10**9:
                problems.append('invalid source timestamp')
            if seconds(stamp) > seconds(now) + 0.05:
                problems.append('source timestamp is in the future')
        if self.arrivals and seconds(now) < self.arrivals[-1]:
            self.arrivals.clear()
        self.arrivals.append(seconds(now))
        value.status = status
        self.value = copy.deepcopy(value)
        self.problems = problems

    def status(self, now):
        s = copy.deepcopy(self.value.status)
        if not s.arrived:
            s.problems = ['missing']
            return s
        now_ns = nanoseconds(now)
        s.received_age_sec = (now_ns - nanoseconds(s.received_stamp)) / 1e9
        s.age_sec = (now_ns - nanoseconds(
            s.source_stamp if s.has_source_stamp else s.received_stamp)) / 1e9
        # A recently received message is still stale if its source data is old.
        s.fresh = (s.received_age_sec >= 0 and s.age_sec >= -0.05
                   and (s.freshness_sec < 0
                        or max(s.age_sec, s.received_age_sec) <= s.freshness_sec))
        problems = list(self.problems)
        if s.received_age_sec < 0 or s.age_sec < -0.05:
            problems.append('clock moved behind observation')
        now_sec = now_ns / 1e9
        recent = [t for t in self.arrivals if now_sec - 2.0 <= t <= now_sec]
        # Equal timestamps do not span an interval and cannot define a rate.
        s.rate_known = len(recent) >= 2 and recent[-1] > recent[0]
        s.rate_hz = ((len(recent) - 1) / (recent[-1] - recent[0])
                     if s.rate_known else 0.0)
        s.rate_ok = s.expected_rate_hz == 0 or (
            s.rate_known and 0.5 * s.expected_rate_hz <= s.rate_hz
            <= 1.5 * s.expected_rate_hz)
        if s.expected_rate_hz and s.rate_known and not s.rate_ok:
            problems.append('update rate outside 50-150% of expected rate')
        s.valid = not problems
        s.problems = problems
        return s

    def snapshot(self, now):
        result = copy.deepcopy(self.value)
        result.status = self.status(now)
        return result
