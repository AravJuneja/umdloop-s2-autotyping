"""Finding the panel's markers in one frame, with no ROS in the way.

Nothing here imports rclpy, which is the point: the detection, the decoding
and the overlay can all be exercised against a saved PNG with no simulator,
no graph and no node. The node in detect_node.py is a thin wrapper that
supplies frames and publishes what comes back.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from . import config


@dataclass(frozen=True, eq=False)
class Detection:
    """One marker found in one frame."""

    marker_id: int
    # (4, 2) float array of image points, in the order the marker is printed:
    # its own top-left, top-right, bottom-right, bottom-left. OpenCV rotates
    # the quad to match the decoded orientation, so a marker seen upside down
    # still reports its printed top-left first and the correspondence with
    # the board-frame table in the spec holds without a second guess.
    corners: np.ndarray


@dataclass(frozen=True, eq=False)
class Result:
    """What one frame had to say about the panel."""

    detections: tuple  # Detection, ascending by id
    missing_ids: tuple  # ids from PANEL_MARKER_IDS this frame lacked

    @property
    def complete(self):
        return not self.missing_ids


def make_detector():
    """Build the OpenCV detector once, so frames do not each pay for it."""
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = config.CORNER_REFINEMENT
    dictionary = cv2.aruco.getPredefinedDictionary(config.ARUCO_DICTIONARY)
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def decode(observation):
    """Turn an ImageObservation into a BGR array.

    cv_bridge is not installed and the provided OpenCV is newer than the one
    the system package builds against, so this is a reshape rather than a
    dependency. row_step is honoured instead of assumed to be width * 3: a
    publisher is allowed to pad its rows, and reshaping on width alone would
    silently shear the image if one ever did.
    """
    if observation.encoding != config.IMAGE_ENCODING:
        raise ValueError(f'encoding {observation.encoding!r}, expected {config.IMAGE_ENCODING!r}')
    expected = observation.row_step * observation.height
    if len(observation.pixels) != expected:
        raise ValueError(
            f'{len(observation.pixels)} bytes, expected {expected} '
            f'({observation.height} rows of {observation.row_step})'
        )
    if observation.row_step < observation.width * config.IMAGE_CHANNELS:
        raise ValueError(
            f'row_step {observation.row_step} is short for {observation.width} '
            f'{config.IMAGE_ENCODING} pixels'
        )
    rows = np.frombuffer(observation.pixels, dtype=np.uint8).reshape(
        observation.height, observation.row_step
    )
    packed = rows[:, : observation.width * config.IMAGE_CHANNELS]
    return packed.reshape(observation.height, observation.width, config.IMAGE_CHANNELS)


def detect(image, detector):
    """Find the panel's markers in one BGR frame.

    Ids outside the panel's four are dropped: the dictionary holds fifty and
    only four are on the board. An id detected twice is reported missing
    rather than guessed at -- two quads claiming the same marker means one of
    them is wrong, and nothing in the frame says which.
    """
    corners, ids, _ = detector.detectMarkers(image)
    quads: dict = {}
    if ids is not None:
        for marker_id, quad in zip(ids.ravel().tolist(), corners):
            marker_id = int(marker_id)
            if marker_id in config.PANEL_MARKER_IDS:
                quads.setdefault(marker_id, []).append(quad.reshape(4, 2).astype(np.float64))
    detections = tuple(
        Detection(marker_id=marker_id, corners=found[0])
        for marker_id, found in sorted(quads.items())
        if len(found) == 1
    )
    seen = {detection.marker_id for detection in detections}
    missing = tuple(marker_id for marker_id in config.PANEL_MARKER_IDS if marker_id not in seen)
    return Result(detections=detections, missing_ids=missing)


def draw_overlay(image, result):
    """Draw the detections, their corner order and the missing ids on a copy."""
    overlay = image.copy()
    for detection in result.detections:
        quad = np.round(detection.corners).astype(np.int32)
        cv2.polylines(overlay, [quad], True, config.OUTLINE_COLOR, 1, cv2.LINE_AA)
        centre = detection.corners.mean(axis=0)
        for index, corner in enumerate(detection.corners):
            direction = corner - centre
            norm = float(np.linalg.norm(direction))
            # A degenerate quad has no outward direction; put the label on the
            # corner rather than dividing by zero over a frame we cannot trust.
            if norm > 0:
                direction = direction / norm * config.CORNER_LABEL_OFFSET_PX
            else:
                direction = np.zeros(2)
            _label(overlay, str(index), corner + direction, config.CORNER_COLOR)
        # Clear of the corner labels, which sit about a marker half-diagonal
        # out; at 21 px a marker that is roughly 15 px, and 0 and 1 are up here.
        _label(
            overlay,
            f'id {detection.marker_id}',
            centre + np.array([0.0, -config.ID_LABEL_OFFSET_PX]),
            config.ID_COLOR,
        )
    if result.missing_ids:
        missing = ', '.join(str(marker_id) for marker_id in result.missing_ids)
        _label(overlay, f'missing: {missing}', np.array([10.0, 20.0]), config.MISSING_COLOR)
    return overlay


def _label(image, text, position, color):
    """Centre a short label on a point, clipped to stay inside the frame."""
    (width, height), _ = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, config.FONT_SCALE, config.FONT_THICKNESS
    )
    rows, columns = image.shape[:2]
    origin = (
        int(min(max(position[0] - width / 2, 0), columns - width)),
        int(min(max(position[1] + height / 2, height), rows - 1)),
    )
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        config.FONT_SCALE,
        color,
        config.FONT_THICKNESS,
        cv2.LINE_AA,
    )
