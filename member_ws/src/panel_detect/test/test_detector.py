"""What the detector promises, checked against saved frames.

No simulator, no graph, no node: every input here is either a PNG captured
earlier or a marker this file renders itself.
"""

from conftest import observation, render_marker
import cv2
import numpy as np
from panel_detect import config, detector
import pytest


@pytest.fixture(scope='module')
def find():
    return detector.make_detector()


def test_finds_every_marker_from_the_home_pose(frame, find):
    """All four ids, on a frame from each of three episodes."""
    result = detector.detect(frame, find)
    assert [found.marker_id for found in result.detections] == list(config.PANEL_MARKER_IDS)
    assert result.missing_ids == ()
    assert result.complete
    assert all(found.corners.shape == (4, 2) for found in result.detections)


def test_corners_are_in_printed_order(frame, find):
    """Top-left, top-right, bottom-right, bottom-left of the marker itself.

    The panel is upright in every fixture, so the printed top-left is also
    the corner nearest the image origin; and walking the corners in order
    goes clockwise in image coordinates, which with y pointing down is a
    positive shoelace area.
    """
    for found in detector.detect(frame, find).detections:
        corners = found.corners
        assert np.argmin(corners.sum(axis=1)) == 0
        rolled = np.roll(corners, -1, axis=0)
        area = np.sum(corners[:, 0] * rolled[:, 1] - rolled[:, 0] * corners[:, 1])
        assert area > 0


def test_partial_view_names_what_is_missing(frame, find):
    """A crop reports the markers it still has and names the rest."""
    whole = detector.detect(frame, find)
    left = min(found.corners[:, 0].min() for found in whole.detections)
    right = max(found.corners[:, 0].max() for found in whole.detections)
    # Cut between the left-hand markers and the right-hand ones, with a
    # margin so the cut lands in panel rather than through a marker.
    cut = int((left + right) / 2)
    cropped = detector.detect(np.ascontiguousarray(frame[:, :cut]), find)
    assert [found.marker_id for found in cropped.detections] == [0, 3]
    assert cropped.missing_ids == (1, 2)
    assert not cropped.complete


def test_marker_centres_land_within_a_quarter_pixel(find):
    """The detector's own precision, against corners we know exactly.

    Centres rather than corners: every refinement pulls the outline inward
    by a systematic fraction of a pixel, which cancels at the centre and is
    mostly absorbed by a four-marker fit anyway. The scatter is what limits
    a pose, so the scatter is what gets a bound.
    """
    rng = np.random.default_rng(7)
    offsets = []
    for _ in range(40):
        marker_id = int(rng.integers(0, len(config.PANEL_MARKER_IDS)))
        image, truth = render_marker(
            marker_id=marker_id,
            side_px=float(rng.uniform(19, 24)),
            centre=(80 + float(rng.uniform(-0.5, 0.5)), 80 + float(rng.uniform(-0.5, 0.5))),
            angle=float(rng.uniform(-0.25, 0.25)),
            jitter=0.6,
            rng=rng,
        )
        result = detector.detect(image, find)
        assert [found.marker_id for found in result.detections] == [marker_id]
        found = result.detections[0]
        offsets.append(np.linalg.norm(found.corners.mean(axis=0) - truth.mean(axis=0)))
    assert np.percentile(offsets, 95) <= config.MAX_CENTRE_ERROR_PX


def test_decode_reads_what_robot_state_publishes(frame):
    assert np.array_equal(detector.decode(observation(frame)), frame)


def test_decode_honours_row_padding(frame):
    padded = observation(frame, row_step=frame.shape[1] * 3 + 16)
    assert np.array_equal(detector.decode(padded), frame)


@pytest.mark.parametrize(
    'broken, problem',
    [
        (lambda message: setattr(message, 'encoding', 'rgb8'), 'encoding'),
        (lambda message: setattr(message, 'pixels', b'\x00' * 12), 'bytes'),
        (lambda message: setattr(message, 'row_step', 1), 'bytes'),
    ],
)
def test_decode_refuses_frames_it_cannot_trust(frame, broken, problem):
    message = observation(frame)
    broken(message)
    with pytest.raises(ValueError, match=problem):
        detector.decode(message)


def test_overlay_draws_and_exports(frame, find, tmp_path):
    """The overlay is a copy of the frame, it marks it, and it writes out."""
    overlay = detector.draw_overlay(frame, detector.detect(frame, find))
    assert overlay.shape == frame.shape
    assert not np.array_equal(overlay, frame)
    path = tmp_path / 'overlay.png'
    assert cv2.imwrite(str(path), overlay)
    assert cv2.imread(str(path)).shape == frame.shape


def test_overlay_announces_a_partial_view(frame, find):
    """A frame missing markers says so on the image, not only in the message."""
    half = np.ascontiguousarray(frame[:, : frame.shape[1] // 2])
    partial = detector.detect(half, find)
    assert partial.missing_ids
    banner = detector.draw_overlay(half, partial)[:30, :200]
    assert np.any(banner != half[:30, :200])
