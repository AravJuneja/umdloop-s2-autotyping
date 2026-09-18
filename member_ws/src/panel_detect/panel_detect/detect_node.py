"""The node: take frames from robot_state, publish where the markers are.

All of the seeing happens in detector.py. This file is the plumbing around
it -- which topic the frames come from, what gets published, and how a human
gets an overlay out -- so that the detection logic stays testable without a
graph.
"""

import os
import time

import cv2
from interfaces.msg import MarkerDetection, Pixel
from rclpy.node import Node

from . import config, detector


class DetectNode(Node):

    def __init__(self, **kwargs):
        super().__init__('panel_detect', **kwargs)
        self._detector = detector.make_detector()
        self.declare_parameter('overlay_dir', config.DEFAULT_OVERLAY_DIR)
        # The most recent frame and what we found in it, kept so that
        # ~/save_overlay can answer immediately instead of arming a flag and
        # making the caller wait for the next frame to arrive.
        self._last: tuple | None = None
        self._counts = _Counters()
        self._detections = self.create_publisher(
            config.DETECTIONS_TYPE, config.DETECTIONS_TOPIC, config.DETECTIONS)
        self._overlay = self.create_publisher(
            config.OVERLAY_TYPE, config.OVERLAY_TOPIC, config.OVERLAY)
        self._subscription = self.create_subscription(
            config.IMAGE_TYPE, config.IMAGE_TOPIC, self._on_image, config.FRAMES)
        self._service = self.create_service(
            config.SAVE_OVERLAY_TYPE, config.SAVE_OVERLAY_SERVICE, self._save_overlay)
        self.create_timer(config.LOG_TICK_SEC, self._log_status)
        self.get_logger().info(
            f'frames={config.IMAGE_TOPIC}; detections=~/detections; '
            f'overlay=~/overlay; save=~/save_overlay')

    def _on_image(self, observation):
        """Detect in one frame and publish what it showed."""
        self._counts.frames += 1
        status = observation.status
        # robot_state has already decided whether this frame is well-formed
        # and recent. Acting on one it rejected would be reading data the
        # node whose job that is has told us not to trust.
        if not (status.valid and status.fresh):
            self._counts.skipped += 1
            return
        started = time.perf_counter()
        try:
            image = detector.decode(observation)
        except ValueError as error:
            self._counts.skipped += 1
            self.get_logger().warning(f'undecodable frame: {error}', throttle_duration_sec=5.0)
            return
        result = detector.detect(image, self._detector)
        self._counts.elapsed += time.perf_counter() - started
        self._counts.detected += len(result.detections)
        if result.complete:
            self._counts.complete += 1
        self._last = (image, result, observation)
        self._detections.publish(_detections_message(result, observation))
        # The overlay costs a copy of the frame and a few dozen draw calls,
        # so it is only drawn when something is actually listening.
        if self._overlay.get_subscription_count():
            self._overlay.publish(
                _image_message(detector.draw_overlay(image, result), observation))

    def _save_overlay(self, request, response):
        """Write the latest overlay to disk and report where it went."""
        del request
        if self._last is None:
            response.success = False
            response.message = 'no frame received yet'
            return response
        image, result, observation = self._last
        directory = self.get_parameter('overlay_dir').value
        stamp = observation.status.source_stamp
        path = os.path.join(directory, f'overlay-{stamp.sec}-{stamp.nanosec:09d}.png')
        try:
            os.makedirs(directory, exist_ok=True)
            written = cv2.imwrite(path, detector.draw_overlay(image, result))
        except OSError as error:
            written = False
            path = str(error)
        response.success = bool(written)
        response.message = path if written else f'could not write {path}'
        return response

    def _log_status(self):
        """One line per window: what arrived, what we found, what it cost."""
        counts = self._counts
        self._counts = _Counters()
        rate = counts.frames / config.LOG_TICK_SEC
        processed = counts.frames - counts.skipped
        mean_ms = 1000 * counts.elapsed / processed if processed else 0.0
        self.get_logger().info(
            f'frames={counts.frames} ({rate:.1f} Hz) skipped={counts.skipped} '
            f'complete={counts.complete} markers={counts.detected} '
            f'mean={mean_ms:.1f} ms')


class _Counters:
    """One logging window's worth of tallies."""

    def __init__(self):
        self.frames = 0
        self.skipped = 0
        self.complete = 0
        self.detected = 0
        self.elapsed = 0.0


def _detections_message(result, observation):
    """Build the published message from one frame's result."""
    message = config.DETECTIONS_TYPE()
    message.source_stamp = observation.status.source_stamp
    message.frame_id = observation.frame_id
    message.markers = [_marker_message(detection) for detection in result.detections]
    message.missing_ids = list(result.missing_ids)
    return message


def _marker_message(detection):
    """One marker, with its corners in the order detector.py documents."""
    return MarkerDetection(
        id=detection.marker_id,
        corners=[Pixel(u=float(u), v=float(v)) for u, v in detection.corners])


def _image_message(image, observation):
    """Wrap a BGR array as a sensor_msgs/Image carrying the camera's stamp."""
    message = config.OVERLAY_TYPE()
    message.header.stamp = observation.status.source_stamp
    message.header.frame_id = observation.frame_id
    # int() because rosidl type-checks against the builtin and numpy's
    # integers are not a subclass of it.
    message.height, message.width = (int(size) for size in image.shape[:2])
    message.encoding = config.IMAGE_ENCODING
    message.step = message.width * config.IMAGE_CHANNELS
    message.data = image.tobytes()
    return message
