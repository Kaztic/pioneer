#!/usr/bin/env python3
"""
ROS2 Object Detection Node

Subscribes to:
- /robot_i/camera/image_raw: Camera image (sensor_msgs/Image)
- /robot_i/odom: Current robot pose (nav_msgs/Odometry)

Publishes:
- /robot_i/object_detection: Object detection reports (pioneer_msgs/DetectionReport)

Implements OpenCV-based color/blob detection for object detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import cv2
import numpy as np
import math
from typing import Optional, Tuple, List
from cv_bridge import CvBridge

# Import custom message
try:
    from pioneer_msgs.msg import DetectionReport
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: DetectionReport message not available")


class ObjectDetectionNode(Node):
    """
    Object detection node for a single robot.
    Uses OpenCV color/blob detection to detect objects in camera images.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize object detection node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'object_detection_node'
        
        if robot_id:
            super().__init__(node_name, namespace=robot_id)
            self.robot_id = robot_id
        else:
            super().__init__(node_name)
            # Extract robot_id from node namespace
            namespace = self.get_namespace()
            if namespace and namespace != '/':
                self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
            else:
                full_name = self.get_fully_qualified_name()
                parts = [p for p in full_name.split('/') if p]
                if len(parts) >= 2:
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"

        # Re-extract robot_id after node is fully initialized
        namespace = self.get_namespace()
        if namespace and namespace != '/':
            self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
        else:
            full_name = self.get_fully_qualified_name()
            parts = [p for p in full_name.split('/') if p]
            if len(parts) >= 2:
                self.robot_id = parts[0]
            if not hasattr(self, 'robot_id') or not self.robot_id or self.robot_id == "None":
                self.robot_id = "robot_1"

        # Parameters
        self.declare_parameter('detection_frequency', 5.0)  # Hz
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_blob_size', 50)  # pixels
        self.declare_parameter('max_blob_size', 50000)  # pixels
        self.declare_parameter('color_lower_hsv', [0, 50, 50])  # Lower HSV bound for color detection
        self.declare_parameter('color_upper_hsv', [180, 255, 255])  # Upper HSV bound
        self.declare_parameter('camera_fov', 1.047)  # 60 degrees in radians
        self.declare_parameter('camera_height', 0.15)  # Camera height above ground (meters)
        self.declare_parameter('object_height', 0.5)  # Assumed object height (meters)

        # Get parameters
        detection_freq = self.get_parameter('detection_frequency').get_parameter_value().double_value
        self.min_confidence = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
        self.min_blob_size = self.get_parameter('min_blob_size').get_parameter_value().integer_value
        self.max_blob_size = self.get_parameter('max_blob_size').get_parameter_value().integer_value
        color_lower = self.get_parameter('color_lower_hsv').get_parameter_value().integer_array_value
        color_upper = self.get_parameter('color_upper_hsv').get_parameter_value().integer_array_value
        self.color_lower_hsv = np.array(color_lower)
        self.color_upper_hsv = np.array(color_upper)
        self.camera_fov = self.get_parameter('camera_fov').get_parameter_value().double_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.object_height = self.get_parameter('object_height').get_parameter_value().double_value

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # State variables
        self.current_image: Optional[np.ndarray] = None
        self.current_odom: Optional[Odometry] = None
        self.last_detection_time = 0.0

        # Subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/camera/image_raw')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/odom')

        # Publisher
        if CUSTOM_MSGS_AVAILABLE:
            self.detection_publisher = self.create_publisher(
                DetectionReport,
                'object_detection',
                10
            )
            self.get_logger().info(f'Publishing to /{self.robot_id}/object_detection')
        else:
            self.get_logger().error('DetectionReport message not available!')

        # Timer for periodic detection
        self.detection_timer = self.create_timer(
            1.0 / detection_freq,
            self.detect_objects
        )

        self.get_logger().info(
            f'Object detection node initialized for {self.robot_id}\n'
            f'  Detection frequency: {detection_freq}Hz\n'
            f'  Min confidence: {self.min_confidence}\n'
            f'  Color range: HSV {self.color_lower_hsv} to {self.color_upper_hsv}'
        )

    def image_callback(self, msg: Image):
        """Update current image."""
        try:
            # Convert ROS Image to OpenCV format
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def odom_callback(self, msg: Odometry):
        """Update current robot pose."""
        self.current_odom = msg

    def detect_objects(self):
        """Perform object detection on current image."""
        if self.current_image is None or self.current_odom is None:
            return

        # Convert RGB to HSV for color detection
        hsv_image = cv2.cvtColor(self.current_image, cv2.COLOR_RGB2HSV)

        # Create mask for color range
        mask = cv2.inRange(hsv_image, self.color_lower_hsv, self.color_upper_hsv)

        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process each contour
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by size
            if area < self.min_blob_size or area > self.max_blob_size:
                continue

            # Calculate bounding box
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate confidence based on blob properties
            # Larger, more circular blobs have higher confidence
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * math.pi * area / (perimeter * perimeter)
            else:
                circularity = 0.0

            # Confidence combines area, circularity, and position
            area_confidence = min(1.0, area / (self.max_blob_size * 0.5))
            circularity_confidence = min(1.0, circularity)
            confidence = (area_confidence * 0.4 + circularity_confidence * 0.6)

            if confidence < self.min_confidence:
                continue

            # Estimate object pose in world coordinates
            object_pose = self.estimate_object_pose(center_x, center_y, w, h)

            if object_pose is None:
                continue

            # Publish detection
            self.publish_detection(object_pose, confidence, center_x, center_y, w, h)

    def estimate_object_pose(self, pixel_x: int, pixel_y: int, width: int, height: int) -> Optional[PoseStamped]:
        """
        Estimate object pose in world coordinates from pixel position.

        Args:
            pixel_x: X pixel coordinate of object center
            pixel_y: Y pixel coordinate of object center
            width: Object width in pixels
            height: Object height in pixels

        Returns:
            PoseStamped with object pose in world frame, or None if estimation fails
        """
        if self.current_image is None or self.current_odom is None:
            return None

        img_height, img_width = self.current_image.shape[:2]

        # Get robot pose
        robot_pose = self.current_odom.pose.pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # Get robot orientation (yaw)
        orientation = robot_pose.orientation
        yaw = self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

        # Calculate angle from center of image
        # Camera FOV is horizontal, so we calculate horizontal angle
        center_x = img_width / 2.0
        pixel_offset = pixel_x - center_x
        angle_offset = (pixel_offset / center_x) * (self.camera_fov / 2.0)

        # Estimate distance using object height
        # If object height in pixels is known, we can estimate distance
        # For now, use a simple heuristic: larger objects are closer
        # This is a simplified model - in reality, we'd need camera calibration
        object_size_pixels = max(width, height)
        # Heuristic: assume object is 0.5m tall, estimate distance
        # Using similar triangles: distance = (object_height * focal_length) / object_size_pixels
        # Approximate focal length from FOV: focal_length ≈ width / (2 * tan(FOV/2))
        focal_length = img_width / (2.0 * math.tan(self.camera_fov / 2.0))
        estimated_distance = (self.object_height * focal_length) / object_size_pixels

        # Clamp distance to reasonable range (0.5m to 10m)
        estimated_distance = max(0.5, min(10.0, estimated_distance))

        # Calculate object position relative to robot
        object_angle = yaw + angle_offset
        object_x = robot_x + estimated_distance * math.cos(object_angle)
        object_y = robot_y + estimated_distance * math.sin(object_angle)

        # Create pose
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = f'{self.robot_id}/odom'
        pose.pose.position.x = object_x
        pose.pose.position.y = object_y
        pose.pose.position.z = self.object_height / 2.0  # Assume object center at half height
        pose.pose.orientation = orientation  # Use robot orientation as object orientation

        return pose

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw angle."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def publish_detection(self, object_pose: PoseStamped, confidence: float, 
                         pixel_x: int, pixel_y: int, width: int, height: int):
        """Publish object detection report."""
        if not CUSTOM_MSGS_AVAILABLE:
            return

        # Create detection report
        detection = DetectionReport()
        detection.header = object_pose.header
        detection.object_pose = object_pose
        detection.confidence = confidence
        detection.object_class = "object"  # Default class
        detection.detection_time = self.get_clock().now().to_msg()

        # Optionally crop image snippet (for now, skip to save bandwidth)
        # In production, you might want to include a small cropped region

        # Publish
        self.detection_publisher.publish(detection)
        
        self.get_logger().info(
            f'Published detection: confidence={confidence:.2f}, '
            f'pose=({object_pose.pose.position.x:.2f}, {object_pose.pose.position.y:.2f})'
        )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = ObjectDetectionNode(robot_id=None)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



