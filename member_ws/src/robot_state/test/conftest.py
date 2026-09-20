"""Message builders shared by the unit and node tests.

Lives here rather than in one of the test modules so that importing a helper
does not create an ordering dependency between test files. pytest guarantees
this directory is importable; a sibling test module is only importable by
accident of how the run was invoked.
"""

import os

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from robot_state.config import KNOWN_JOINTS
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# Derived from the pid so two checkouts, or a simulator left running, cannot
# land on the same graph. Every node in a run must agree, including the one
# the node test spawns as a subprocess.
DOMAIN_ID = 90 + os.getpid() % 10


def stamp(value=10.0):
    """Build a Time from fractional seconds; 10.0 is the default "now".

    Tests pass a different value to move the clock rather than to wait.
    """
    ns = round(value * 10**9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def message(name):
    """Build a minimal valid message for one input, for tests to then corrupt."""
    if name == 'joints':
        msg = JointState(
            name=list(KNOWN_JOINTS),
            position=[0.0, 1.0, 2.0, 3.0, 4.0],
            velocity=[5.0, 6.0, 7.0, 8.0, 9.0],
        )
    elif name == 'image':
        msg = Image(width=2, height=2, encoding='bgr8', step=8, data=bytes(range(16)))
    elif name == 'calibration':
        msg = CameraInfo(
            width=2,
            height=2,
            distortion_model='plumb_bob',
            d=[0.0] * 5,
            k=[9.0, 0.0, 0.5, 0.0, 9.0, 0.5, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[9.0, 0.0, 0.5, 0.0, 0.0, 9.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0],
        )
    elif name == 'launch_key':
        return String(data='ROVER')
    else:
        transform = TransformStamped(child_frame_id='camera')
        transform.header.frame_id = 'world'
        transform.header.stamp = stamp()
        transform.transform.rotation.w = 1.0
        return TFMessage(transforms=[transform])
    msg.header.stamp = stamp()
    msg.header.frame_id = 'camera_optical_frame'
    return msg
