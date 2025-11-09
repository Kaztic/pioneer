#!/usr/bin/env python3
"""
ROS2 Spatial Analysis Visualizer Node

Subscribes to:
- /robot_i/odom: Robot odometry
- /robot_i/path: Robot planned paths
- /robot_i/frontiers: Frontier detections

Publishes:
- /spatial_analysis/exploration_heatmap: Visualization markers for exploration heatmap
- /spatial_analysis/frontier_distribution: Visualization markers for frontier distribution
- /spatial_analysis/path_efficiency: Visualization markers for path efficiency
- /spatial_analysis/robot_trajectories: Visualization markers for robot trajectories

Creates spatial analysis visualizations for multi-robot exploration.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from typing import Dict, List, Optional, Tuple
from collections import deque
import time


class SpatialAnalysisVisualizerNode(Node):
    """
    Spatial analysis visualizer node for multi-robot exploration.
    Creates heatmaps and visualizations for exploration analysis.
    """

    def __init__(self):
        """Initialize spatial analysis visualizer node."""
        super().__init__('spatial_analysis_visualizer_node')

        # Parameters
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('heatmap_resolution', 0.5)
        self.declare_parameter('heatmap_update_rate', 1.0)
        self.declare_parameter('trajectory_window_seconds', 30.0)
        self.declare_parameter('frontier_window_seconds', 30.0)
        self.declare_parameter('path_window_seconds', 10.0)
        self.declare_parameter('world_bounds_x_min', -15.0)
        self.declare_parameter('world_bounds_x_max', 15.0)
        self.declare_parameter('world_bounds_y_min', -15.0)
        self.declare_parameter('world_bounds_y_max', 15.0)
        self.declare_parameter('fixed_frame', 'robot_1/odom')

        # Get parameters
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.heatmap_resolution = self.get_parameter('heatmap_resolution').get_parameter_value().double_value
        self.heatmap_update_rate = self.get_parameter('heatmap_update_rate').get_parameter_value().double_value
        self.trajectory_window = self.get_parameter('trajectory_window_seconds').get_parameter_value().double_value
        self.frontier_window = self.get_parameter('frontier_window_seconds').get_parameter_value().double_value
        self.path_window = self.get_parameter('path_window_seconds').get_parameter_value().double_value
        self.world_bounds = {
            'x_min': self.get_parameter('world_bounds_x_min').get_parameter_value().double_value,
            'x_max': self.get_parameter('world_bounds_x_max').get_parameter_value().double_value,
            'y_min': self.get_parameter('world_bounds_y_min').get_parameter_value().double_value,
            'y_max': self.get_parameter('world_bounds_y_max').get_parameter_value().double_value,
        }
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value

        # State storage
        self.robot_trajectories: Dict[str, deque] = {}  # robot_id -> deque of (x, y, timestamp)
        self.robot_paths: Dict[str, deque] = {}  # robot_id -> deque of Path messages
        self.frontier_history: deque = deque(maxlen=1000)  # (x, y, timestamp, robot_id)
        self.exploration_heatmap: np.ndarray = None

        # Initialize trajectory storage for each robot
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            max_points = int(self.trajectory_window * 10)  # Assume ~10 Hz update rate
            self.robot_trajectories[robot_id] = deque(maxlen=max_points)
            self.robot_paths[robot_id] = deque(maxlen=100)

        # Initialize heatmap grid
        self._initialize_heatmap()

        # Publishers
        self.heatmap_publisher = self.create_publisher(
            MarkerArray,
            '/spatial_analysis/exploration_heatmap',
            10
        )
        self.frontier_publisher = self.create_publisher(
            MarkerArray,
            '/spatial_analysis/frontier_distribution',
            10
        )
        self.path_efficiency_publisher = self.create_publisher(
            MarkerArray,
            '/spatial_analysis/path_efficiency',
            10
        )
        self.trajectory_publisher = self.create_publisher(
            MarkerArray,
            '/spatial_analysis/robot_trajectories',
            10
        )

        # Subscriptions for each robot
        self.odom_subscriptions = {}
        self.path_subscriptions = {}
        self.frontier_subscriptions = {}

        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            
            # Subscribe to odometry
            self.odom_subscriptions[robot_id] = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )
            
            # Subscribe to paths
            self.path_subscriptions[robot_id] = self.create_subscription(
                Path,
                f'/{robot_id}/path',
                lambda msg, rid=robot_id: self.path_callback(msg, rid),
                10
            )
            
            # Subscribe to frontiers
            self.frontier_subscriptions[robot_id] = self.create_subscription(
                Float32MultiArray,
                f'/{robot_id}/frontiers',
                lambda msg, rid=robot_id: self.frontier_callback(msg, rid),
                10
            )

        # Timer for publishing visualizations
        update_period = 1.0 / self.heatmap_update_rate
        self.visualization_timer = self.create_timer(update_period, self.publish_visualizations)

        self.get_logger().info(
            f'Spatial Analysis Visualizer Node initialized\n'
            f'  Robots: {self.num_robots}\n'
            f'  Heatmap resolution: {self.heatmap_resolution}m\n'
            f'  Update rate: {self.heatmap_update_rate}Hz\n'
            f'  Fixed frame: {self.fixed_frame}'
        )

    def _initialize_heatmap(self):
        """Initialize the exploration heatmap grid."""
        x_size = int((self.world_bounds['x_max'] - self.world_bounds['x_min']) / self.heatmap_resolution)
        y_size = int((self.world_bounds['y_max'] - self.world_bounds['y_min']) / self.heatmap_resolution)
        self.exploration_heatmap = np.zeros((x_size, y_size), dtype=np.float32)
        self.heatmap_shape = (x_size, y_size)

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        grid_x = int((x - self.world_bounds['x_min']) / self.heatmap_resolution)
        grid_y = int((y - self.world_bounds['y_min']) / self.heatmap_resolution)
        grid_x = max(0, min(grid_x, self.heatmap_shape[0] - 1))
        grid_y = max(0, min(grid_y, self.heatmap_shape[1] - 1))
        return grid_x, grid_y

    def _grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (center of cell)."""
        x = self.world_bounds['x_min'] + (grid_x + 0.5) * self.heatmap_resolution
        y = self.world_bounds['y_min'] + (grid_y + 0.5) * self.heatmap_resolution
        return x, y

    def odom_callback(self, msg: Odometry, robot_id: str):
        """Callback for robot odometry updates."""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            timestamp = time.time()

            # Store trajectory point
            if robot_id in self.robot_trajectories:
                self.robot_trajectories[robot_id].append((x, y, timestamp))

            # Update heatmap (increment exploration count at this location)
            grid_x, grid_y = self._world_to_grid(x, y)
            if 0 <= grid_x < self.heatmap_shape[0] and 0 <= grid_y < self.heatmap_shape[1]:
                self.exploration_heatmap[grid_x, grid_y] += 1.0

        except Exception as e:
            self.get_logger().warn(f'Error in odom_callback for {robot_id}: {e}')

    def path_callback(self, msg: Path, robot_id: str):
        """Callback for robot path updates."""
        try:
            if robot_id in self.robot_paths:
                self.robot_paths[robot_id].append((msg, time.time()))
        except Exception as e:
            self.get_logger().warn(f'Error in path_callback for {robot_id}: {e}')

    def frontier_callback(self, msg: Float32MultiArray, robot_id: str):
        """Callback for robot frontier detections."""
        try:
            # Parse frontier points (assuming pairs of x, y coordinates)
            data = msg.data
            timestamp = time.time()
            
            if len(data) % 2 == 0:
                for i in range(0, len(data), 2):
                    x = data[i]
                    y = data[i + 1]
                    self.frontier_history.append((x, y, timestamp, robot_id))
        except Exception as e:
            self.get_logger().warn(f'Error in frontier_callback for {robot_id}: {e}')

    def publish_visualizations(self):
        """Publish all visualization markers."""
        try:
            now = self.get_clock().now()
            current_time = time.time()

            # Publish exploration heatmap
            self._publish_heatmap(now)

            # Publish frontier distribution
            self._publish_frontier_distribution(now, current_time)

            # Publish robot trajectories
            self._publish_trajectories(now, current_time)

            # Publish path efficiency
            self._publish_path_efficiency(now, current_time)

        except Exception as e:
            self.get_logger().error(f'Error publishing visualizations: {e}')

    def _publish_heatmap(self, now):
        """Publish exploration heatmap as marker array."""
        if self.exploration_heatmap is None:
            return

        markers = MarkerArray()
        max_value = np.max(self.exploration_heatmap) if np.max(self.exploration_heatmap) > 0 else 1.0

        # Create markers for each grid cell
        marker_id = 0
        for grid_x in range(self.heatmap_shape[0]):
            for grid_y in range(self.heatmap_shape[1]):
                value = self.exploration_heatmap[grid_x, grid_y]
                if value > 0:
                    x, y = self._grid_to_world(grid_x, grid_y)
                    
                    marker = Marker()
                    marker.header.frame_id = self.fixed_frame
                    marker.header.stamp = now.to_msg()
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = 0.1
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = self.heatmap_resolution * 0.8
                    marker.scale.y = self.heatmap_resolution * 0.8
                    marker.scale.z = 0.1

                    # Color based on exploration count (red = high, blue = low)
                    intensity = value / max_value
                    marker.color.r = float(intensity)
                    marker.color.g = 0.0
                    marker.color.b = float(1.0 - intensity)
                    marker.color.a = 0.5

                    markers.markers.append(marker)
                    marker_id += 1

        if markers.markers:
            self.heatmap_publisher.publish(markers)

    def _publish_frontier_distribution(self, now, current_time):
        """Publish frontier distribution markers."""
        markers = MarkerArray()
        cutoff_time = current_time - self.frontier_window

        # Filter recent frontiers
        recent_frontiers = [
            (x, y, robot_id) for x, y, ts, robot_id in self.frontier_history
            if ts > cutoff_time
        ]

        # Create markers for frontiers
        for i, (x, y, robot_id) in enumerate(recent_frontiers):
            marker = Marker()
            marker.header.frame_id = self.fixed_frame
            marker.header.stamp = now.to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            markers.markers.append(marker)

        if markers.markers:
            self.frontier_publisher.publish(markers)

    def _publish_trajectories(self, now, current_time):
        """Publish robot trajectory markers."""
        markers = MarkerArray()
        cutoff_time = current_time - self.trajectory_window

        # Color palette for different robots
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
        ]

        marker_id = 0
        for robot_idx, (robot_id, trajectory) in enumerate(self.robot_trajectories.items()):
            if not trajectory:
                continue

            # Filter recent trajectory points
            recent_points = [(x, y) for x, y, ts in trajectory if ts > cutoff_time]

            if len(recent_points) < 2:
                continue

            # Create line strip marker for trajectory
            marker = Marker()
            marker.header.frame_id = self.fixed_frame
            marker.header.stamp = now.to_msg()
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            color = colors[robot_idx % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8

            # Add points to line strip
            for x, y in recent_points:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.1
                marker.points.append(point)

            markers.markers.append(marker)
            marker_id += 1

        if markers.markers:
            self.trajectory_publisher.publish(markers)

    def _publish_path_efficiency(self, now, current_time):
        """Publish path efficiency markers."""
        # For now, just publish empty markers
        # This can be extended to show path efficiency metrics
        markers = MarkerArray()
        self.path_efficiency_publisher.publish(markers)


def main(args=None):
    """Main entry point for spatial analysis visualizer node."""
    rclpy.init(args=args)

    node = SpatialAnalysisVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # ROS2 is already shutting down, don't call shutdown again
        pass
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
