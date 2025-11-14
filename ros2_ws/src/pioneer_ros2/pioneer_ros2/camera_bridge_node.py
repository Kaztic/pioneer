#!/usr/bin/env python3
"""
ROS2 Camera Bridge Node

Reads camera image data from Webots controller files and publishes as sensor_msgs/Image.

This node:
1. Reads camera data from /tmp/pioneer_camera_robot_i.txt files
2. Parses camera image data (robot name, timestamp, width, height, pixel data)
3. Converts to sensor_msgs/Image message format
4. Publishes to /robot_i/camera/image_raw topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import os
import time
import struct
import base64
import numpy as np
import cv2
from cv_bridge import CvBridge
from typing import Optional


class CameraBridgeNode(Node):
    """
    Bridge node for a single robot.
    Should be instantiated once per robot.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize camera bridge node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'camera_bridge_node'
        
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
                # Use fully qualified name to extract namespace
                full_name = self.get_fully_qualified_name()
                parts = [p for p in full_name.split('/') if p]
                if len(parts) >= 2:
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"  # Default

        # Re-extract robot_id after node is fully initialized (if needed)
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

        # File path for this robot's camera data
        self.camera_file = f'/tmp/pioneer_camera_{self.robot_id}.txt'
        
        # Track last read timestamp to avoid republishing same image
        self.last_timestamp = None
        
        # CV Bridge for image format conversion (BGRA to RGB)
        self.bridge = CvBridge()

        # Subscribe to odometry to get robot pose for frame_id
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.current_odom = None
        self.get_logger().info(f'Subscribed to /{self.robot_id}/odom')

        # Publisher for Image
        self.image_publisher = self.create_publisher(
            Image,
            'camera/image_raw',
            10
        )
        self.get_logger().info(f'Publishing to /{self.robot_id}/camera/image_raw')

        # Timer to read and publish images (~10-30 Hz)
        self.read_timer = self.create_timer(0.1, self.read_and_publish_image)  # 10 Hz default

        self.get_logger().info(
            f'Camera bridge node initialized for {self.robot_id}\n'
            f'  Input file: {self.camera_file}\n'
            f'  Publishing to: /{self.robot_id}/camera/image_raw'
        )

    def odom_callback(self, msg: Odometry):
        """Update current odometry for frame_id."""
        self.current_odom = msg

    def read_and_publish_image(self):
        """Read camera file and publish as Image if new data available."""
        if not os.path.exists(self.camera_file):
            return

        try:
            with open(self.camera_file, 'rb') as f:
                # Read first line to check format
                first_line = f.readline().decode('utf-8', errors='ignore').strip()
                if not first_line.startswith('CAMERA:'):
                    return

                # Parse header: CAMERA:robot_name:timestamp:width:height:encoding:
                parts = first_line.split(':')
                if len(parts) < 6:
                    return

                robot_name = parts[1]
                timestamp = float(parts[2])
                width = int(parts[3])
                height = int(parts[4])
                encoding = parts[5] if len(parts) > 5 else 'rgb8'

                # Check if this is a new image (different timestamp)
                if self.last_timestamp is not None and abs(timestamp - self.last_timestamp) < 0.01:
                    return  # Same image, skip

                self.last_timestamp = timestamp

                # Read remaining data (image bytes)
                image_data = f.read()

                # If data is base64 encoded, decode it
                if encoding == 'base64':
                    try:
                        image_data = base64.b64decode(image_data)
                        encoding = 'rgb8'  # Assume RGB after decoding
                    except:
                        self.get_logger().warn('Failed to decode base64 image data')
                        return

                # Handle different image formats
                if encoding == 'bgra8':
                    # Webots returns BGRA (4 bytes per pixel), convert to RGB8
                    expected_size = width * height * 4  # BGRA = 4 bytes per pixel
                    if len(image_data) < expected_size:
                        self.get_logger().warn(
                            f'Incomplete BGRA image data: expected {expected_size} bytes, got {len(image_data)}'
                        )
                        return
                    
                    # Convert BGRA to RGB using OpenCV
                    # Reshape to (height, width, 4) for BGRA
                    bgra_array = np.frombuffer(image_data[:expected_size], dtype=np.uint8).reshape((height, width, 4))
                    # Convert BGRA to RGB
                    rgb_array = cv2.cvtColor(bgra_array, cv2.COLOR_BGRA2RGB)
                    # Flatten back to bytes
                    image_data = rgb_array.tobytes()
                    encoding = 'rgb8'
                elif encoding == 'rgb8':
                    # RGB8 format (3 bytes per pixel)
                    expected_size = width * height * 3
                    if len(image_data) < expected_size:
                        self.get_logger().warn(
                            f'Incomplete RGB image data: expected {expected_size} bytes, got {len(image_data)}'
                        )
                        return
                    image_data = image_data[:expected_size]
                else:
                    self.get_logger().warn(f'Unsupported encoding: {encoding}')
                    return

                # Create Image message
                image_msg = Image()
                
                # Set header
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                if self.current_odom:
                    image_msg.header.frame_id = f'{self.robot_id}/camera_link'
                else:
                    image_msg.header.frame_id = f'{self.robot_id}/base_link'

                # Set image parameters
                image_msg.width = width
                image_msg.height = height
                image_msg.encoding = encoding
                image_msg.is_bigendian = 0
                image_msg.step = width * 3  # RGB8 = 3 bytes per pixel

                # Set image data
                image_msg.data = image_data

                # Publish
                self.image_publisher.publish(image_msg)

                # Log first few images for debugging
                if not hasattr(self, '_publish_count'):
                    self._publish_count = 0
                self._publish_count += 1
                if self._publish_count <= 3:
                    self.get_logger().info(
                        f'Published image: {width}x{height}, encoding={encoding}, '
                        f'size={len(image_data)} bytes'
                    )

        except FileNotFoundError:
            # File doesn't exist yet, that's ok
            pass
        except Exception as e:
            self.get_logger().error(f'Error reading camera file: {e}')


def main(args=None):
    """Main entry point for camera bridge node."""
    rclpy.init(args=args)

    # Let ROS2 handle namespace via --ros-args -r __ns:=/robot_1
    # The node will extract robot_id from its namespace
    node = CameraBridgeNode(robot_id=None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Try to log error, but handle case where ROS2 context is invalid
        error_msg = str(e) if e else "Unknown error"
        try:
            node.get_logger().error(f'Error in camera bridge node: {error_msg}')
        except:
            # If logging fails (e.g., context invalid), print to stderr instead
            import sys
            print(f'Error in camera bridge node: {error_msg}', file=sys.stderr)
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()



