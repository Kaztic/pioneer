#!/usr/bin/env python3
"""
ROS2 LiDAR Bridge Node

Reads LiDAR scan data from Webots controller files and publishes as sensor_msgs/LaserScan.

This node:
1. Reads LiDAR data from /tmp/pioneer_lidar_robot_i.txt files
2. Parses LiDAR scan data (robot name, timestamp, ranges, angles)
3. Converts to sensor_msgs/LaserScan message format
4. Publishes to /robot_i/scan topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import os
import time
from typing import Optional


class LidarBridgeNode(Node):
    """
    Bridge node for a single robot.
    Should be instantiated once per robot.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize LiDAR bridge node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'lidar_bridge_node'
        
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

        # File path for this robot's LiDAR data
        self.lidar_file = f'/tmp/pioneer_lidar_{self.robot_id}.txt'
        
        # Track last read timestamp to avoid republishing same scan
        self.last_timestamp = None

        # Subscribe to odometry to get robot pose for frame_id
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.current_odom = None
        self.get_logger().info(f'Subscribed to /{self.robot_id}/odom')

        # Publisher for LaserScan
        self.scan_publisher = self.create_publisher(
            LaserScan,
            'scan',
            10
        )
        self.get_logger().info(f'Publishing to /{self.robot_id}/scan')

        # Timer to read and publish scans (Webots timestep: 32ms = ~31 Hz)
        self.read_timer = self.create_timer(0.032, self.read_and_publish_scan)

        self.get_logger().info(
            f'LiDAR bridge node initialized for {self.robot_id}\n'
            f'  Input file: {self.lidar_file}\n'
            f'  Publishing to: /{self.robot_id}/scan'
        )

    def odom_callback(self, msg: Odometry):
        """Update current odometry for frame_id."""
        self.current_odom = msg

    def read_and_publish_scan(self):
        """Read LiDAR file and publish as LaserScan if new data available."""
        if not os.path.exists(self.lidar_file):
            return

        try:
            with open(self.lidar_file, 'r') as f:
                line = f.readline().strip()
                if not line or not line.startswith('LIDAR:'):
                    return

                # Parse format: LIDAR:robot_name:timestamp:num_points:angle_min:angle_max:range_min:range_max:range1:range2:...
                parts = line.split(':')
                if len(parts) < 8:
                    return

                robot_name = parts[1]
                timestamp = float(parts[2])
                num_points = int(parts[3])
                angle_min = float(parts[4])
                angle_max = float(parts[5])
                range_min = float(parts[6])
                range_max = float(parts[7])

                # Check if this is a new scan (different timestamp)
                if self.last_timestamp is not None and abs(timestamp - self.last_timestamp) < 0.001:
                    return  # Same scan, skip

                self.last_timestamp = timestamp

                # Parse range values
                if len(parts) < 8 + num_points:
                    self.get_logger().warn(f'Incomplete scan data: expected {num_points} ranges, got {len(parts) - 8}')
                    return

                ranges = []
                for i in range(8, 8 + num_points):
                    range_val = float(parts[i])
                    if range_val < 0:
                        # Invalid reading (marked as -1 in controller)
                        ranges.append(float('inf'))
                    else:
                        ranges.append(range_val)

                # Create LaserScan message
                scan_msg = LaserScan()
                
                # Set header
                scan_msg.header = Header()
                scan_msg.header.stamp = self.get_clock().now().to_msg()
                if self.current_odom:
                    scan_msg.header.frame_id = f'{self.robot_id}/base_link'
                else:
                    scan_msg.header.frame_id = f'{self.robot_id}/odom'

                # Set scan parameters
                scan_msg.angle_min = angle_min
                scan_msg.angle_max = angle_max
                scan_msg.angle_increment = (angle_max - angle_min) / num_points if num_points > 1 else 0.0
                scan_msg.time_increment = 0.0  # Not provided by Webots
                scan_msg.scan_time = 0.032  # Webots timestep (32ms)
                scan_msg.range_min = range_min
                scan_msg.range_max = range_max

                # Set ranges
                scan_msg.ranges = ranges

                # Set intensities (not provided by Webots, set to zero)
                scan_msg.intensities = [0.0] * num_points

                # Publish
                self.scan_publisher.publish(scan_msg)

                # Log first few scans for debugging
                if not hasattr(self, '_publish_count'):
                    self._publish_count = 0
                self._publish_count += 1
                if self._publish_count <= 3:
                    self.get_logger().info(
                        f'Published scan: {num_points} points, '
                        f'angle_range=[{angle_min:.3f}, {angle_max:.3f}], '
                        f'range=[{min(r for r in ranges if r != float("inf")):.2f}, '
                        f'{max(r for r in ranges if r != float("inf")):.2f}]'
                    )

        except FileNotFoundError:
            # File doesn't exist yet, that's ok
            pass
        except Exception as e:
            self.get_logger().error(f'Error reading LiDAR file: {e}')


def main(args=None):
    """Main entry point for LiDAR bridge node."""
    rclpy.init(args=args)

    # Let ROS2 handle namespace via --ros-args -r __ns:=/robot_1
    # The node will extract robot_id from its namespace
    node = LidarBridgeNode(robot_id=None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Try to log error, but handle case where ROS2 context is invalid
        error_msg = str(e) if e else "Unknown error"
        try:
            node.get_logger().error(f'Error in LiDAR bridge node: {error_msg}')
        except:
            # If logging fails (e.g., context invalid), print to stderr instead
            import sys
            print(f'Error in LiDAR bridge node: {error_msg}', file=sys.stderr)
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

