"""What the detector assumes about the panel and the camera, in one place.

Everything here is either a fact from the challenge spec or a threshold we
chose and measured. Keeping them together means the detection policy can be
reviewed as a policy, and means the numbers that came out of a measurement
sit next to a note saying which measurement.
"""

import cv2
from interfaces.msg import ImageObservation, MarkerDetections
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# --- The panel ------------------------------------------------------------
# Four DICT_4X4_50 markers, one per corner, clockwise from top-left as seen
# from the arm (spec 6.1). Anything else the detector finds is not ours: the
# dictionary holds 50 ids and only these four are on the panel.
PANEL_MARKER_IDS = (0, 1, 2, 3)
ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50

# --- Corner refinement ----------------------------------------------------
# Without refinement OpenCV returns integer corners. At the home pose a
# marker is roughly 21 px on a side -- about 4 px per module -- which is
# small enough that the choice of refinement matters more than usual.
#
# All four were measured against synthetic ground truth; the package README
# carries the table. Contour refinement wins on the number that matters,
# marker-centre error, at 0.078 px RMS against 0.126 for sub-pixel and 0.197
# for none. It pays for that with the largest systematic shrink of the
# marker outline (-4.8% on the side), which a four-marker board fit mostly
# ignores: the board's scale comes from the ~500 px between markers, not
# from the 21 px across one. AprilTag refinement is the only one that gets
# the outline right and the only one that moves the centre, by 0.64 px.
#
# Sub-pixel refinement also ignores cornerRefinementWinSize at this marker
# size -- 2, 3 and 5 give bit-identical results -- so there is no window to
# tune here even if we wanted one.
CORNER_REFINEMENT = cv2.aruco.CORNER_REFINE_CONTOUR

# What the synthetic precision test holds us to, at the 95th percentile of
# marker-centre error. Measured p95 is 0.163 px, so this is the measurement
# plus room for a different OpenCV build, not a target we are scraping past.
MAX_CENTRE_ERROR_PX = 0.25

# --- Topics ---------------------------------------------------------------
# Frames come from robot_state rather than straight from the camera, so a
# frame that is malformed or stale has already been judged by the node whose
# job that is, and said so in its status.
IMAGE_TOPIC = '/robot_state/updates/image'
DETECTIONS_TOPIC = '~/detections'
OVERLAY_TOPIC = '~/overlay'
SAVE_OVERLAY_SERVICE = '~/save_overlay'

IMAGE_TYPE = ImageObservation
DETECTIONS_TYPE = MarkerDetections
OVERLAY_TYPE = Image
SAVE_OVERLAY_TYPE = Trigger

# --- QoS ------------------------------------------------------------------
# Newest frame wins. robot_state publishes ~/updates/image reliable and
# transient-local; a best-effort, volatile, depth-1 subscription is
# compatible with that offer and is what makes the middleware drop the frames
# we are too slow for instead of queueing them behind us.
FRAMES = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# Detections are small and every one matters to whoever is estimating pose.
DETECTIONS = QoSProfile(depth=10)
# The overlay is a human-facing debug view, and a stale one is worthless.
OVERLAY = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# --- Accepted formats -----------------------------------------------------
# One encoding, checked rather than converted: anything else is a change in
# the simulator we want to hear about rather than paper over.
IMAGE_ENCODING = 'bgr8'
IMAGE_CHANNELS = 3

# --- Overlay ---------------------------------------------------------------
# BGR, to match the image. Corner labels are drawn outside the marker because
# a marker is only about 21 px across and a digit inside it covers a module.
OUTLINE_COLOR = (0, 255, 0)
CORNER_COLOR = (0, 255, 255)
ID_COLOR = (255, 255, 0)
MISSING_COLOR = (0, 0, 255)
CORNER_LABEL_OFFSET_PX = 10.0
ID_LABEL_OFFSET_PX = 30.0
FONT_SCALE = 0.4
FONT_THICKNESS = 1

# Overlays are written outside the repository by default: a test or a curious
# service call should not leave files in a working tree.
DEFAULT_OVERLAY_DIR = '/tmp/panel_detect'

# --- Logging --------------------------------------------------------------
# One summary line per window, reporting the frame rate we actually managed
# rather than the one the camera offers, since the gap between them is the
# whole question of whether we are keeping up.
LOG_TICK_SEC = 5.0
