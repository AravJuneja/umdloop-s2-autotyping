import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
import cv2
import numpy as np

# Import the custom detections message
from interfaces.msg import MarkerDetections
from autotype_sim.core.keymap import load_geometry, generate_tkl, KeyMap

class KeyProjector(Node):
    def __init__(self):
        super().__init__('key_projector')
        self.K = None
        self.D = None
        self.board_rvec = None
        self.board_tvec = None
        
        geom_path = '/ws/board_geometry.yaml'
        self.get_logger().info(f"Loading official geometry from {geom_path}...")
        geom = load_geometry(geom_path)
        self.keymap = generate_tkl(geom)
        
        self.key_names = []
        centers_3d = []
        
        for key in self.keymap.keys:
            cx, cy = KeyMap.center(key)
            centers_3d.append([cx, cy, 0.0])
            self.key_names.append(key.name)
            
        self.points_3d = np.array(centers_3d, dtype=np.float64)
        
        qos_transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, qos_transient)
            
        # Subscribe directly to the 2D pixel coordinates
        self.detection_sub = self.create_subscription(
            MarkerDetections, '/panel_detect/detections', self.detection_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)

    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info("Camera intrinsics loaded.")

    def detection_callback(self, msg):
        if self.K is None or len(msg.markers) < 4:
            return
            
        # Extract 2D centers of markers 0, 1, 2, 3
        sorted_markers = sorted(msg.markers, key=lambda m: m.id)
        img_pts = []
        for m in sorted_markers:
            cx = sum(c.u for c in m.corners) / 4.0
            cy = sum(c.v for c in m.corners) / 4.0
            img_pts.append([cx, cy])
        img_pts = np.array(img_pts, dtype=np.float64)
        
        # PLACEHOLDERS: Update W and H when your team replies!
        W = 0.450  # Estimated 450mm width
        H = 0.150  # Estimated 150mm height
        obj_pts = np.array([
            [0.0, 0.0, 0.0], # Top-Left (ID 0)
            [W,   0.0, 0.0], # Top-Right (ID 1)
            [W,   H,   0.0], # Bottom-Right (ID 2)
            [0.0, H,   0.0]  # Bottom-Left (ID 3)
        ], dtype=np.float64)
        
        # Compute 3D pose, skipping the TF tree entirely
        success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D)
        if success:
            self.board_rvec = rvec
            self.board_tvec = tvec

    def image_callback(self, msg):
        if self.K is None or self.board_rvec is None:
            self.get_logger().warn("Waiting for board detections to compute pose...", throttle_duration_sec=2.0)
            return 
            
        cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        cv_image = np.copy(cv_image)
        
        try:
            # Project the keys using the live computed pose
            image_points, _ = cv2.projectPoints(self.points_3d, self.board_rvec, self.board_tvec, self.K, self.D)
        except Exception as e:
            self.get_logger().error(f"OpenCV Error: {e}")
            return
        
        # Draw the points and the key names
        for i, (name, p) in enumerate(zip(self.key_names, image_points)):
            u, v = int(p[0][0]), int(p[0][1])
            
            # Print the first key to see where it lands!
            if i == 0:
                self.get_logger().info(f"Key '{name}' projects to pixel: ({u}, {v})")
                
            if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                cv2.circle(cv_image, (u, v), radius=4, color=(0, 255, 0), thickness=-1)
                cv2.putText(cv_image, name, (u + 5, v - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
        cv2.imwrite("key_projection_validation.png", cv_image)
        self.get_logger().info("Saved validation image", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()