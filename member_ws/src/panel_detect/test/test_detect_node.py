"""The node's contract, over a real graph, once.

Thin on purpose: detection is tested against saved frames in
test_detector.py, and what is left to check here is the wiring -- that a
frame arriving on robot_state's topic comes back out as detections, and
that the save service writes an overlay where it says it did.
"""

import os
import time

from conftest import FIXTURES, observation
import cv2
from interfaces.msg import ImageObservation, MarkerDetections
from panel_detect import config
from panel_detect.detect_node import DetectNode
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

# A band of its own, so a robot_state test running beside this one cannot
# discover our nodes.
DOMAIN_ID = 80 + os.getpid() % 10


def spin_until(executor, predicate, timeout=10, each=None):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if each is not None:
            each()
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('timed out')


def test_a_frame_in_becomes_detections_and_an_overlay_out(tmp_path):
    frame = cv2.imread(str(FIXTURES[0]))
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    try:
        node = DetectNode(context=context)
        node.set_parameters([Parameter('overlay_dir', value=str(tmp_path))])
        probe = rclpy.create_node('probe', context=context)
        received: list = []
        probe.create_subscription(
            MarkerDetections, '/panel_detect/detections', received.append, 10)
        frames = probe.create_publisher(
            ImageObservation, config.IMAGE_TOPIC, config.FRAMES)
        saves = probe.create_client(Trigger, '/panel_detect/save_overlay')
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(probe)

        # Best effort at depth one drops anything published before discovery
        # finishes, which is the point of it; so keep offering the frame.
        message = observation(frame)
        spin_until(executor, lambda: bool(received), each=lambda: frames.publish(message))

        detections = received[0]
        assert [marker.id for marker in detections.markers] == [0, 1, 2, 3]
        assert list(detections.missing_ids) == []
        assert detections.frame_id == 'camera_optical_frame'
        assert detections.source_stamp == message.status.source_stamp
        assert all(len(marker.corners) == 4 for marker in detections.markers)

        future = saves.call_async(Trigger.Request())
        spin_until(executor, future.done)
        response = future.result()
        assert response.success, response.message
        written = cv2.imread(response.message)
        assert written is not None and written.shape == frame.shape
        assert os.path.dirname(response.message) == str(tmp_path)
    finally:
        rclpy.shutdown(context=context)


def test_the_node_ignores_frames_robot_state_rejected(tmp_path):
    """A frame marked invalid is counted and dropped, not detected in."""
    frame = cv2.imread(str(FIXTURES[0]))
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    try:
        node = DetectNode(context=context)
        node.set_parameters([Parameter('overlay_dir', value=str(tmp_path))])
        probe = rclpy.create_node('probe', context=context)
        received: list = []
        probe.create_subscription(
            MarkerDetections, '/panel_detect/detections', received.append, 10)
        frames = probe.create_publisher(
            ImageObservation, config.IMAGE_TOPIC, config.FRAMES)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(probe)

        stale = observation(frame, fresh=False)
        spin_until(executor, lambda: frames.get_subscription_count() > 0)
        for _ in range(20):
            frames.publish(stale)
            executor.spin_once(timeout_sec=0.05)
        assert received == []
    finally:
        rclpy.shutdown(context=context)
