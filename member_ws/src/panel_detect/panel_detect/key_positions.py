"""Estimate the panel's pose from marker detections, and locate its keys.

Panel pose: cv2.solvePnP against the four markers' printed corners (16
correspondences), object points from INTERFACES.md 6.1. There is no `board`
TF frame (INTERFACES.md 5) -- this is the only way to get the pose.

Key positions: INTERFACES.md 6.3/12 and the challenge spec are explicit that
the keyboard's offset inside the panel (key_area_origin) is never provided
and is ours to determine -- it is site configuration, fixed for the whole
deployment rather than resampled per episode (autotype_sim.core.board:
sample_board_pose only draws the panel's pose in world; key_area_origin
comes from the separately-loaded BoardGeometry). So it only needs to be
found once, not every episode, and it is found here by looking, not by
reading a number out of a file:

autotype_sim's own renderer (core/renderer.py: _draw_keyboard) always paints
the keyboard's plate as a perfectly flat bezel colour, with the key grid --
a photo or a grid of labelled caps, either rendering mode -- pasted inside
it at key_area_origin, edge to edge, no bezel of its own. Once the panel
pose is known, warping the image into a metric top-down board-frame view
turns that into a plain image-processing problem: the flat bezel has near
zero local pixel variance and the grid does not, in either rendering mode,
so a variance threshold finds the grid's rectangle. Its extent is exactly
the public 18.25u x 6.25u layout (INTERFACES.md 6.3), so only the
rectangle's *position* is new information; its centre combined with that
known extent is used in place of its own (blur-inflated) edges. Measurements
are averaged across frames -- the true value does not move -- which is
mainly what turns a single noisy contour into something worth trusting.
"""

import os

import cv2
import numpy as np
import rclpy
from autotype_sim.core.keymap import KeyMap, generate_tkl
from interfaces.msg import CalibrationObservation, ImageObservation, MarkerDetections
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from . import detector

# INTERFACES.md 6.1: each marker's own TL/TR/BR/BL corners in the board
# frame (z = 0). MarkerDetection.corners is documented to report a marker's
# printed TL first regardless of image rotation, so corner i here pairs with
# corner i of a detection with the same id without any re-sorting.
MARKER_CORNERS_BOARD = {
    0: [(0.006, 0.006, 0.0), (0.026, 0.006, 0.0), (0.026, 0.026, 0.0), (0.006, 0.026, 0.0)],
    1: [(0.374, 0.006, 0.0), (0.394, 0.006, 0.0), (0.394, 0.026, 0.0), (0.374, 0.026, 0.0)],
    2: [(0.374, 0.149, 0.0), (0.394, 0.149, 0.0), (0.394, 0.169, 0.0), (0.374, 0.169, 0.0)],
    3: [(0.006, 0.149, 0.0), (0.026, 0.149, 0.0), (0.026, 0.169, 0.0), (0.006, 0.169, 0.0)],
}
MARKER_CENTERS_BOARD = {
    0: (0.016, 0.016), 1: (0.384, 0.016), 2: (0.384, 0.159), 3: (0.016, 0.159),
}

# robot_state normalizes every sim input (README: "so the rest of our code
# never has to ask whether a message is recent"); nothing here talks to
# /camera/* directly, matching detect_node.py's own convention.
CALIBRATION_TOPIC = '/robot_state/updates/calibration'
IMAGE_TOPIC = '/robot_state/updates/image'
DETECTIONS_TOPIC = '/panel_detect/detections'

# robot_state publishes every ~/updates/<input> on this exact profile
# (robot_state/config.py: LATCHED_QOS), image included -- a late subscriber
# gets the current value immediately rather than waiting for the next frame.
UPDATES_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# INTERFACES.md 6.1/6.3: public panel size and key standard, not site
# configuration.
PANEL_W_M = 0.400
PANEL_H_M = 0.175
KEY_PITCH_M = 0.01905
GRID_WIDTH_U = 18.25
GRID_HEIGHT_U = 6.25
GRID_WIDTH_M = GRID_WIDTH_U * KEY_PITCH_M
GRID_HEIGHT_M = GRID_HEIGHT_U * KEY_PITCH_M
# generate_tkl needs a key_inset to build valid Key rectangles, but a key's
# centre ((x0+x1)/2) does not depend on it -- it cancels out. Any value
# satisfying 0 < inset < key_pitch/2 is dimensionally inert for our purpose.
NOMINAL_KEY_INSET = 0.0005

# Board-frame top-down resolution used when searching for the key grid.
RECTIFY_PX_PER_M = 3000.0
# Half-width of the square blanked out around each marker centre before
# searching: markers are textured too, and sit far enough from a
# TKL-sized keyboard's plausible placement not to overlap it.
MARKER_MASK_HALF_M = 0.020
BEZEL_LIKE_BGR = (38, 38, 38)
# A single frame's contour is noisy; key_area_origin does not move between
# frames (it is site configuration -- see the module docstring), so this
# many agreeing measurements is trusted before anything is published.
MIN_MEASUREMENTS_TO_TRUST = 5
LOG_EVERY_N_MEASUREMENTS = 25

KEY_POINT_COLOR = (0, 255, 0)
BANNER_COLOR = (0, 255, 255)


def _panel_pose_homography(K, rvec, tvec):
    """Board-frame ``(x, y, 1)`` (metres, z=0) -> image homogeneous coordinates."""
    R, _ = cv2.Rodrigues(rvec)
    return K @ np.hstack([R[:, :2], tvec.reshape(3, 1)])


def _rectify_to_board_frame(image, homography, px_per_m):
    """Fronto-parallel board-frame view: canvas pixel (x, y) is board metre
    (x / px_per_m, y / px_per_m); canvas size is the panel's public extent."""
    canvas_to_board = np.diag([1.0 / px_per_m, 1.0 / px_per_m, 1.0])
    canvas_to_image = homography @ canvas_to_board
    size = (int(round(PANEL_W_M * px_per_m)), int(round(PANEL_H_M * px_per_m)))
    return cv2.warpPerspective(image, canvas_to_image, size, flags=cv2.WARP_INVERSE_MAP)


def _mask_markers(canvas, px_per_m):
    """Blank the marker footprints so their own texture cannot be mistaken
    for the key grid's."""
    masked = canvas.copy()
    half_px = MARKER_MASK_HALF_M * px_per_m
    for cx, cy in MARKER_CENTERS_BOARD.values():
        x0, x1 = int(round(cx * px_per_m - half_px)), int(round(cx * px_per_m + half_px))
        y0, y1 = int(round(cy * px_per_m - half_px)), int(round(cy * px_per_m + half_px))
        masked[max(y0, 0):y1, max(x0, 0):x1] = BEZEL_LIKE_BGR
    return masked


def _find_key_grid_rectangle(canvas, px_per_m):
    """Board-frame ``(x0, y0, width, height)`` metres of the busiest
    rectangular region in ``canvas``, or ``None``."""
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY).astype(np.float32)
    mean = cv2.boxFilter(gray, -1, (9, 9))
    sq_mean = cv2.boxFilter(gray ** 2, -1, (9, 9))
    activity = np.sqrt(np.clip(sq_mean - mean ** 2, 0, None)).astype(np.uint8)
    _, mask = cv2.threshold(activity, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return x / px_per_m, y / px_per_m, w / px_per_m, h / px_per_m


def _extent_is_plausible(width, height, tolerance=0.4):
    return (abs(width - GRID_WIDTH_M) <= tolerance * GRID_WIDTH_M
            and abs(height - GRID_HEIGHT_M) <= tolerance * GRID_HEIGHT_M)


def _origin_is_in_bounds(ox, oy):
    return (0.0 <= ox and ox + GRID_WIDTH_M <= PANEL_W_M
            and 0.0 <= oy and oy + GRID_HEIGHT_M <= PANEL_H_M)


def measure_key_area_origin(image, K, rvec, tvec):
    """One frame's estimate of ``key_area_origin`` (board-frame metres), or
    ``None`` when this frame's contour is not a plausible key grid.

    Trusts the detected rectangle's *centre*, not its edges, combined with
    the known extent: boxFilter + morphological closing systematically
    inflate the detected box by a roughly symmetric margin, which cancels
    out of the centre but would bias each edge individually.
    """
    homography = _panel_pose_homography(K, rvec, tvec)
    canvas = _rectify_to_board_frame(image, homography, RECTIFY_PX_PER_M)
    canvas = _mask_markers(canvas, RECTIFY_PX_PER_M)
    rectangle = _find_key_grid_rectangle(canvas, RECTIFY_PX_PER_M)
    if rectangle is None:
        return None
    x0, y0, width, height = rectangle
    if not _extent_is_plausible(width, height):
        return None
    cx, cy = x0 + width / 2.0, y0 + height / 2.0
    ox, oy = cx - GRID_WIDTH_M / 2.0, cy - GRID_HEIGHT_M / 2.0
    if not _origin_is_in_bounds(ox, oy):
        return None
    return ox, oy


class KeyProjector(Node):

    def __init__(self):
        super().__init__('key_projector')
        self.declare_parameter(
            'overlay_path', '/tmp/panel_detect/key_projection_validation.png')

        self.K = None
        self.D = None
        self.rvec = None
        self.tvec = None
        self.key_names = []
        self.key_points_board = None
        self._key_area_n = 0
        self._key_area_mean = np.zeros(2)
        self._key_area_m2 = np.zeros(2)  # Welford's running sum of squared deviations

        self.create_subscription(
            CalibrationObservation, CALIBRATION_TOPIC, self._on_calibration, UPDATES_QOS)
        self.create_subscription(
            MarkerDetections, DETECTIONS_TOPIC, self._on_detections, 10)
        self.create_subscription(
            ImageObservation, IMAGE_TOPIC, self._on_image, UPDATES_QOS)
        self.get_logger().info(
            f'calibration={CALIBRATION_TOPIC}; detections={DETECTIONS_TOPIC}; '
            f'image={IMAGE_TOPIC}')

    def _on_calibration(self, observation):
        if self.K is not None or not observation.status.valid:
            return
        self.K = np.array(observation.intrinsic, dtype=np.float64).reshape(3, 3)
        self.D = np.array(observation.distortion, dtype=np.float64)
        self.get_logger().info(f'camera intrinsics loaded from {CALIBRATION_TOPIC}')

    def _on_detections(self, msg):
        if self.K is None:
            return
        obj_pts, img_pts = [], []
        for marker in msg.markers:
            board_corners = MARKER_CORNERS_BOARD.get(marker.id)
            if board_corners is None:
                continue
            for obj_c, img_c in zip(board_corners, marker.corners):
                obj_pts.append(obj_c)
                img_pts.append((img_c.u, img_c.v))
        # All four markers required: a partial-view pose from 1-3 markers is
        # possible in principle but is not what this fits today.
        if len(obj_pts) != 16:
            return

        obj_pts = np.array(obj_pts, dtype=np.float64)
        img_pts = np.array(img_pts, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D)
        if not ok:
            self.get_logger().warning(
                'solvePnP failed to converge', throttle_duration_sec=2.0)
            return
        self.rvec, self.tvec = rvec, tvec

        reprojected, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.D)
        errors = np.linalg.norm(reprojected.reshape(-1, 2) - img_pts, axis=1)
        self.get_logger().info(
            f'panel pose updated: reprojection error mean={errors.mean():.3f}px '
            f'max={errors.max():.3f}px',
            throttle_duration_sec=2.0)

    def _update_key_area_estimate(self, image):
        """Fold one frame's key-grid measurement into the running estimate
        and rebuild the keymap from it once enough frames agree."""
        measurement = measure_key_area_origin(image, self.K, self.rvec, self.tvec)
        if measurement is None:
            return
        sample = np.array(measurement)
        self._key_area_n += 1
        delta = sample - self._key_area_mean
        self._key_area_mean += delta / self._key_area_n
        self._key_area_m2 += delta * (sample - self._key_area_mean)
        if self._key_area_n < MIN_MEASUREMENTS_TO_TRUST:
            return
        if self._key_area_n % LOG_EVERY_N_MEASUREMENTS < 1:
            std = np.sqrt(self._key_area_m2 / self._key_area_n)
            self.get_logger().info(
                f'key_area_origin=({self._key_area_mean[0]:.4f}, '
                f'{self._key_area_mean[1]:.4f}) m from {self._key_area_n} frames, '
                f'std=({std[0]:.4f}, {std[1]:.4f}) m')
        self._rebuild_keymap(tuple(self._key_area_mean))

    def _rebuild_keymap(self, key_area_origin):
        geometry = {
            'key_pitch': KEY_PITCH_M,
            'key_area_origin': key_area_origin,
            'key_inset': NOMINAL_KEY_INSET,
        }
        keymap = generate_tkl(geometry)
        self.key_names = [key.name for key in keymap.keys]
        # z = 0: the keyboard is a flat photo rendered onto the panel plane
        # (INTERFACES.md 4, 6.3), not a physical cap with height, so a key's
        # board-frame position needs no z offset.
        self.key_points_board = np.array(
            [[*KeyMap.center(key), 0.0] for key in keymap.keys], dtype=np.float64)

    def _on_image(self, observation):
        status = observation.status
        if not (status.valid and status.fresh) or self.rvec is None:
            return
        try:
            image = detector.decode(observation)
        except ValueError as error:
            self.get_logger().warning(
                f'undecodable frame: {error}', throttle_duration_sec=5.0)
            return

        self._update_key_area_estimate(image)
        if self.key_points_board is None:
            self.get_logger().info(
                f'key grid not yet located ({self._key_area_n} measurements so far)',
                throttle_duration_sec=5.0)
            return

        image_points, _ = cv2.projectPoints(
            self.key_points_board, self.rvec, self.tvec, self.K, self.D)
        overlay = image.copy()
        for name, point in zip(self.key_names, image_points.reshape(-1, 2)):
            u, v = int(round(point[0])), int(round(point[1]))
            if 0 <= u < overlay.shape[1] and 0 <= v < overlay.shape[0]:
                cv2.circle(overlay, (u, v), 3, KEY_POINT_COLOR, -1)
                cv2.putText(
                    overlay, name, (u + 4, v - 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, KEY_POINT_COLOR, 1, cv2.LINE_AA)
        std = np.sqrt(self._key_area_m2 / self._key_area_n)
        cv2.putText(
            overlay,
            f'key_area_origin=({self._key_area_mean[0]:.4f},{self._key_area_mean[1]:.4f})m '
            f'n={self._key_area_n} std=({std[0]:.4f},{std[1]:.4f})m',
            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, BANNER_COLOR, 1, cv2.LINE_AA)

        path = self.get_parameter('overlay_path').value
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, overlay)
        self.get_logger().info(f'wrote key overlay to {path}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = KeyProjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
