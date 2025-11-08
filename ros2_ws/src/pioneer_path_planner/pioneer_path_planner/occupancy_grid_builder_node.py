#!/usr/bin/env python3
"""
ROS2 Occupancy Grid Builder Node

Processes LaserScan messages into nav_msgs/OccupancyGrid and publishes updates.

This node:
1. Subscribes to /robot_i/scan (sensor_msgs/LaserScan) - LiDAR scans
2. Subscribes to /robot_i/odom (nav_msgs/Odometry) - Robot pose for coordinate transforms
3. Processes scans using ray casting and probabilistic updates
4. Publishes /robot_i/occupancy_map (nav_msgs/OccupancyGrid) - Updated occupancy grid
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
import math
import numpy as np
from typing import Optional, Tuple


class OccupancyGridBuilderNode(Node):
    """
    Occupancy grid builder node for a single robot.
    Implements probabilistic grid mapping with ray casting.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize occupancy grid builder node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'occupancy_grid_builder_node'
        
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
                    self.robot_id = "robot_1"  # Default

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
        self.declare_parameter('grid_resolution', 0.2)  # meters
        self.declare_parameter('map_width', 30.0)  # meters
        self.declare_parameter('map_height', 30.0)  # meters
        self.declare_parameter('origin_x', -15.0)  # meters
        self.declare_parameter('origin_y', -15.0)  # meters
        self.declare_parameter('prob_free', -0.4)  # Log-odds for free cells
        self.declare_parameter('prob_occ', 0.4)  # Log-odds for occupied cells
        self.declare_parameter('prob_threshold', 0.5)  # Threshold for occupied (log-odds)
        self.declare_parameter('publish_frequency', 2.0)  # Hz

        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().double_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().double_value
        self.origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        self.origin_y = self.get_parameter('origin_y').get_parameter_value().double_value
        self.prob_free = self.get_parameter('prob_free').get_parameter_value().double_value
        self.prob_occ = self.get_parameter('prob_occ').get_parameter_value().double_value
        self.prob_threshold = self.get_parameter('prob_threshold').get_parameter_value().double_value

        # Grid dimensions in cells
        self.grid_width = int(math.ceil(self.map_width / self.grid_resolution))
        self.grid_height = int(math.ceil(self.map_height / self.grid_resolution))

        # Initialize log-odds grid (starts at 0 = unknown)
        self.log_odds_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Current robot pose (for coordinate transforms)
        self.current_pose: Optional[Pose] = None

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/scan')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/odom')

        # Publisher
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'occupancy_map',
            10
        )
        self.get_logger().info(f'Publishing to /{self.robot_id}/occupancy_map')

        # Timer to publish grid at fixed frequency
        publish_freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.publish_timer = self.create_timer(
            1.0 / publish_freq,
            self.publish_map
        )

        self.get_logger().info(
            f'Occupancy grid builder node initialized for {self.robot_id}\n'
            f'  Grid resolution: {self.grid_resolution}m\n'
            f'  Map size: {self.map_width}m x {self.map_height}m\n'
            f'  Grid size: {self.grid_width} x {self.grid_height} cells\n'
            f'  Origin: ({self.origin_x}, {self.origin_y})\n'
            f'  Publish frequency: {publish_freq}Hz'
        )

    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry."""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        """
        Process LaserScan and update occupancy grid using ray casting.
        """
        # Log first few scans at ERROR level
        if not hasattr(self, '_scan_callback_count'):
            self._scan_callback_count = 0
        self._scan_callback_count += 1
        if self._scan_callback_count <= 3:
            self.get_logger().error(
                f'[SCAN_RECEIVED] Scan callback #{self._scan_callback_count}: {len(msg.ranges)} points, '
                f'angle_range=[{msg.angle_min:.3f}, {msg.angle_max:.3f}]'
            )
        
        if self.current_pose is None:
            if self._scan_callback_count <= 10:
                self.get_logger().error('[SCAN_WAITING] Waiting for odometry, cannot process scan')
            return  # Need robot pose for coordinate transforms

        # Get robot position in world frame
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Get robot orientation (yaw from quaternion)
        orientation = self.current_pose.orientation
        yaw = self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

        # Process each scan point
        num_points = len(msg.ranges)
        for i in range(num_points):
            range_val = msg.ranges[i]
            
            # Skip invalid readings
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min:
                continue

            # Calculate angle of this scan point in robot frame
            angle = msg.angle_min + i * msg.angle_increment
            
            # Transform to world frame
            world_angle = angle + yaw
            
            # Calculate endpoint in world coordinates
            end_x = robot_x + range_val * math.cos(world_angle)
            end_y = robot_y + range_val * math.sin(world_angle)

            # Ray cast from robot to endpoint
            self.ray_cast(
                robot_x, robot_y,
                end_x, end_y,
                range_val, msg.range_max
            )

    def ray_cast(self, start_x: float, start_y: float, end_x: float, end_y: float,
                 range_val: float, max_range: float):
        """
        Cast a ray from start to end, updating grid cells along the way.
        
        Args:
            start_x, start_y: Ray start position in world coordinates
            end_x, end_y: Ray end position in world coordinates
            range_val: Measured range value
            max_range: Maximum range of sensor
        """
        # Convert to grid coordinates
        start_gx, start_gy = self.world_to_grid(start_x, start_y)
        end_gx, end_gy = self.world_to_grid(end_x, end_y)

        # Get cells along the ray using Bresenham's line algorithm
        cells = self.bresenham_line(start_gx, start_gy, end_gx, end_gy)

        if not cells:
            return

        # Update cells along the ray
        # All cells before the endpoint are free
        # The endpoint cell is occupied
        for idx, (gx, gy) in enumerate(cells):
            # Check bounds
            if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
                continue

            if idx < len(cells) - 1:
                # Free space along ray
                self.log_odds_grid[gy, gx] += self.prob_free
            else:
                # Endpoint is occupied
                self.log_odds_grid[gy, gx] += self.prob_occ

            # Clamp log-odds to reasonable range
            self.log_odds_grid[gy, gx] = max(-10.0, min(10.0, self.log_odds_grid[gy, gx]))

    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> list:
        """
        Bresenham's line algorithm to get all cells between two points.
        
        Returns:
            List of (x, y) grid cell coordinates
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            world_x, world_y: World coordinates (meters)
            
        Returns:
            (grid_x, grid_y) in cells
        """
        grid_x = int((world_x - self.origin_x) / self.grid_resolution)
        grid_y = int((world_y - self.origin_y) / self.grid_resolution)
        return grid_x, grid_y

    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def publish_map(self):
        """Convert log-odds grid to OccupancyGrid message and publish."""
        # Convert log-odds to probability, then to 0-100 scale
        # OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
        occupancy_data = []
        
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                log_odds = self.log_odds_grid[y, x]
                
                if abs(log_odds) < 0.01:  # Unknown (close to zero)
                    occupancy_data.append(-1)
                elif log_odds > self.prob_threshold:  # Occupied
                    # Convert log-odds to probability, scale to 0-100
                    prob = 1.0 - (1.0 / (1.0 + math.exp(log_odds)))
                    occupancy = int(prob * 100)
                    occupancy = min(100, max(0, occupancy))
                    occupancy_data.append(occupancy)
                else:  # Free
                    prob = 1.0 - (1.0 / (1.0 + math.exp(log_odds)))
                    occupancy = int((1.0 - prob) * 100)
                    occupancy = min(100, max(0, occupancy))
                    # For free space, we use 0, but could use lower values
                    occupancy_data.append(0)

        # Create OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = f'{self.robot_id}/odom'

        map_msg.info.resolution = self.grid_resolution
        map_msg.info.width = self.grid_width
        map_msg.info.height = self.grid_height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = occupancy_data

        # Publish
        self.map_publisher.publish(map_msg)

        # Log periodically
        if not hasattr(self, '_publish_count'):
            self._publish_count = 0
        self._publish_count += 1
        occupied_count = sum(1 for x in occupancy_data if x > 50)
        free_count = sum(1 for x in occupancy_data if x == 0)
        unknown_count = sum(1 for x in occupancy_data if x == -1)
        
        # CRITICAL: Log first few publishes and periodically at ERROR level
        if self._publish_count <= 5 or self._publish_count % 50 == 0:
            self.get_logger().error(
                f'[OCCUPANCY_MAP_PUBLISHED] Map #{self._publish_count}: {occupied_count} occupied, {free_count} free, '
                f'{unknown_count} unknown cells'
            )
        else:
            self.get_logger().debug(
                f'Published map: {occupied_count} occupied, {free_count} free, '
                f'{unknown_count} unknown cells'
            )


def main(args=None):
    """Main entry point for occupancy grid builder node."""
    rclpy.init(args=args)

    # Let ROS2 handle namespace via --ros-args -r __ns:=/robot_1
    # The node will extract robot_id from its namespace
    node = OccupancyGridBuilderNode(robot_id=None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Try to log error, but handle case where ROS2 context is invalid
        error_msg = str(e) if e else "Unknown error"
        try:
            node.get_logger().error(f'Error in occupancy grid builder node: {error_msg}')
        except:
            # If logging fails (e.g., context invalid), print to stderr instead
            import sys
            print(f'Error in occupancy grid builder node: {error_msg}', file=sys.stderr)
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



