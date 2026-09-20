"""Frames and message builders shared by the tests.

The fixtures are real camera frames, captured from three separate episodes
at q_home with the arm within 0.001 rad of the home pose, so the panel sits
somewhere different in each. Nothing here starts a simulator: a saved PNG is
the whole input.
"""

from pathlib import Path

import cv2
from interfaces.msg import ImageObservation
import numpy as np
import pytest

FIXTURES = sorted((Path(__file__).parent / 'fixtures').glob('home-*.png'))

# One module of white quiet zone around a 4x4 marker makes six modules of
# black-and-white across the printed side.
MARKER_MODULES = 6


@pytest.fixture(params=FIXTURES, ids=lambda path: path.stem)
def frame(request):
    """One saved camera frame, as BGR."""
    return cv2.imread(str(request.param))


def observation(image, valid=True, fresh=True, row_step=None):
    """Wrap a BGR array the way robot_state would hand it to us."""
    message = ImageObservation()
    message.status.source = 'simulator'
    message.status.arrived = True
    message.status.valid = valid
    message.status.fresh = fresh
    message.status.source_stamp.sec = 10
    message.status.source_stamp.nanosec = 500_000_000
    message.frame_id = 'camera_optical_frame'
    message.height, message.width = (int(size) for size in image.shape[:2])
    message.encoding = 'bgr8'
    message.row_step = row_step if row_step is not None else message.width * 3
    if row_step is None:
        message.pixels = image.tobytes()
    else:
        padded = np.zeros((message.height, row_step), np.uint8)
        padded[:, :message.width * 3] = image.reshape(message.height, -1)
        message.pixels = padded.tobytes()
    return message


def render_marker(marker_id, side_px, centre, angle, size=160, jitter=0.0,
                  rng=None, supersample=8):
    """Draw one marker at a known sub-pixel place; return the image and corners.

    Rendered large and averaged down rather than warped straight to size, so
    the edges are area-averaged the way a renderer's would be instead of
    aliased. The returned corners are the marker's true outer corners, which
    is what makes this ground truth rather than another detection.
    """
    module = 40
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_MODULES * module)
    span = MARKER_MODULES * module
    source = np.full((span + 2 * module,) * 2, 255, np.uint8)
    source[module:module + span, module:module + span] = marker
    # Pixel i covers [i - 0.5, i + 0.5], so the black border's outer edge sits
    # half a pixel before its first pixel.
    edge = np.array([[module - .5, module - .5], [module + span - .5, module - .5],
                     [module + span - .5, module + span - .5],
                     [module - .5, module + span - .5]], dtype=np.float32)
    half = side_px / 2
    unit = np.array([[-half, -half], [half, -half], [half, half], [-half, half]],
                    dtype=np.float32)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]], dtype=np.float32)
    corners = unit @ rotation.T + np.array(centre, dtype=np.float32)
    if jitter and rng is not None:
        corners = corners + rng.uniform(-jitter, jitter, corners.shape).astype(np.float32)
    big = (corners + 0.5) * supersample - 0.5
    matrix = cv2.getPerspectiveTransform(edge, big.astype(np.float32))
    canvas = cv2.warpPerspective(
        source, matrix, (size * supersample,) * 2, flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=(0.0, 0.0, 0.0))
    image = cv2.resize(canvas, (size, size), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), corners.astype(np.float64)
