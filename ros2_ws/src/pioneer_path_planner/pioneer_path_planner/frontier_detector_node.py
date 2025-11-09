#!/usr/bin/env python3
"""
ROS2 Frontier Detector Node

Subscribes to:
- /robot_i/occupancy_map: Occupancy grid map (nav_msgs/OccupancyGrid)

Publishes:
- /robot_i/frontiers: List of frontier points (geometry_msgs/Point array)

Detects boundaries between known free space and unknown regions (frontiers).
Frontiers are candidate exploration goals where the robot can gain new information.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, Float32MultiArray
import math
import numpy as np
from typing import Optional, List, Tuple


class FrontierDetectorNode(Node):
    """
    Frontier detection node for a single robot.
    Detects frontiers using gradient-based method on occupancy grid.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize frontier detector node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'frontier_detector_node'
        
        if robot_id:
            super().__init__(node_name, namespace=robot_id)
            self.robot_id = robot_id
        else:
            super().__init__(node_name)
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
        self.declare_parameter('frontier_size_threshold', 3)  # Minimum frontier size (cells)
        self.declare_parameter('frontier_distance_threshold', 0.5)  # meters - min distance between frontiers
        self.declare_parameter('information_gain_weight', 1.0)  # Weight for frontier size
        self.declare_parameter('detection_frequency', 2.0)  # Hz

        self.frontier_size_threshold = self.get_parameter('frontier_size_threshold').get_parameter_value().integer_value
        self.frontier_distance_threshold = self.get_parameter('frontier_distance_threshold').get_parameter_value().double_value
        self.information_gain_weight = self.get_parameter('information_gain_weight').get_parameter_value().double_value

        # State
        self.current_map: Optional[OccupancyGrid] = None

        # Subscription
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'occupancy_map',
            self.map_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/occupancy_map')

        # Publisher (using PointStamped for each frontier, but we'll publish array of Points)
        # For simplicity, we'll publish as array of Points via custom topic
        # Actually, let's use a custom message or just publish PointStamped for each frontier
        # For now, let's create a publisher for each frontier as PointStamped
        # Better: publish array of Points (we'll use geometry_msgs/Point[] via a custom message)
        # Simplest: publish multiple PointStamped messages with same timestamp
        # Actually ROS2 doesn't have native Point[] array. We'll publish PointStamped messages
        # Or create a custom message. For now, let's use PointStamped and publish multiple times
        
        # We'll use a simpler approach: publish a list as a custom topic
        # For now, just publish frontier count and log them
        # Best approach: Create a simple custom message or use existing Point array
        # Let's use std_msgs/Float32MultiArray to publish frontier positions
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension
        
        self.frontiers_publisher = self.create_publisher(
            Float32MultiArray,
            'frontiers',
            10
        )
        self.get_logger().info(f'Publishing to /{self.robot_id}/frontiers')

        # Timer for periodic frontier detection
        detection_freq = self.get_parameter('detection_frequency').get_parameter_value().double_value
        self.detection_timer = self.create_timer(
            1.0 / detection_freq,
            self.detect_frontiers_callback
        )

        self.get_logger().info(
            f'Frontier detector node initialized for {self.robot_id}\n'
            f'  Detection frequency: {detection_freq}Hz\n'
            f'  Frontier size threshold: {self.frontier_size_threshold} cells\n'
            f'  Frontier distance threshold: {self.frontier_distance_threshold}m'
        )

    def map_callback(self, msg: OccupancyGrid):
        """Update current occupancy map."""
        self.current_map = msg

    def detect_frontiers_callback(self):
        """Periodic callback to detect and publish frontiers."""
        if self.current_map is None:
            self.get_logger().debug('No occupancy map available yet')
            return

        frontiers = self.detect_frontiers(self.current_map)
        
        if frontiers:
            self.publish_frontiers(frontiers)
            self.get_logger().info(f'Detected {len(frontiers)} frontiers')
        else:
            self.get_logger().debug('No frontiers detected (map might be fully explored or no unknown areas)')

    def detect_frontiers(self, occupancy_map: OccupancyGrid) -> List[Tuple[float, float]]:
        """
        Detect frontiers in occupancy grid.

        Args:
            occupancy_map: nav_msgs/OccupancyGrid message

        Returns:
            List of (x, y) tuples representing frontier centers in world coordinates
        """
        map_info = occupancy_map.info
        width = map_info.width
        height = map_info.height
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        # Convert occupancy grid data to numpy array for easier processing
        # OccupancyGrid is stored row-major: data[i] = grid[row][col] where row = i/width, col = i%width
        grid = np.array(occupancy_map.data).reshape((height, width))

        # Debug: Count cell types
        unique, counts = np.unique(grid, return_counts=True)
        cell_counts = dict(zip(unique, counts))
        free_count = sum(counts[i] for i, val in enumerate(unique) if 0 <= val <= 50)
        unknown_count = cell_counts.get(-1, 0)
        occupied_count = sum(counts[i] for i, val in enumerate(unique) if val > 50)
        
        self.get_logger().debug(
            f'Map stats: {width}x{height}, resolution={resolution:.3f}m, '
            f'Free={free_count}, Unknown={unknown_count}, Occupied={occupied_count}'
        )

        # Detect frontier cells: cells that are FREE (0-50) adjacent to UNKNOWN (-1)
        frontier_cells = []
        
        # Iterate through all cells
        for y in range(1, height - 1):  # Skip boundaries to avoid index errors
            for x in range(1, width - 1):
                # Current cell value
                cell_value = grid[y, x]
                
                # Skip if not free space
                if cell_value < 0 or cell_value > 50:
                    continue
                
                # Check if any neighbor is unknown
                is_frontier = False
                neighbors = [
                    grid[y - 1, x],     # North
                    grid[y + 1, x],     # South
                    grid[y, x - 1],     # West
                    grid[y, x + 1],     # East
                    grid[y - 1, x - 1], # Northwest
                    grid[y - 1, x + 1], # Northeast
                    grid[y + 1, x - 1], # Southwest
                    grid[y + 1, x + 1], # Southeast
                ]
                
                for neighbor in neighbors:
                    if neighbor == -1:  # Unknown
                        is_frontier = True
                        break
                
                if is_frontier:
                    # Convert grid coordinates to world coordinates
                    world_x = origin_x + (x + 0.5) * resolution
                    world_y = origin_y + (y + 0.5) * resolution
                    frontier_cells.append((world_x, world_y))

        self.get_logger().debug(f'Found {len(frontier_cells)} frontier cells')

        # Group nearby frontier cells into clusters
        frontier_clusters = self.cluster_frontiers(frontier_cells, resolution)
        self.get_logger().debug(f'Created {len(frontier_clusters)} frontier clusters')
        
        # Filter clusters by size
        valid_frontiers = []
        for cluster in frontier_clusters:
            if len(cluster) >= self.frontier_size_threshold:
                # Calculate cluster center
                center_x = sum(p[0] for p in cluster) / len(cluster)
                center_y = sum(p[1] for p in cluster) / len(cluster)
                valid_frontiers.append((center_x, center_y))
            else:
                self.get_logger().debug(
                    f'Cluster with {len(cluster)} cells filtered out (threshold: {self.frontier_size_threshold})'
                )

        self.get_logger().debug(f'{len(valid_frontiers)} frontiers passed size threshold')

        # Filter frontiers that are too close to each other
        filtered_frontiers = self.filter_close_frontiers(valid_frontiers, self.frontier_distance_threshold)
        
        if len(valid_frontiers) != len(filtered_frontiers):
            self.get_logger().debug(
                f'Filtered {len(valid_frontiers) - len(filtered_frontiers)} frontiers due to distance threshold'
            )

        return filtered_frontiers

    def cluster_frontiers(self, frontier_cells: List[Tuple[float, float]], resolution: float) -> List[List[Tuple[float, float]]]:
        """
        Group nearby frontier cells into clusters.

        Args:
            frontier_cells: List of (x, y) frontier cell positions
            resolution: Grid resolution in meters

        Returns:
            List of clusters, each cluster is a list of (x, y) positions
        """
        if not frontier_cells:
            return []

        clusters = []
        unassigned = frontier_cells.copy()
        cluster_distance = resolution * 2.0  # Cells within 2 grid cells are in same cluster

        while unassigned:
            # Start new cluster with first unassigned cell
            cluster = [unassigned.pop(0)]
            
            # Find all cells within cluster_distance
            i = 0
            while i < len(unassigned):
                cell = unassigned[i]
                # Check distance to any cell in current cluster
                min_dist = min(
                    math.sqrt((cell[0] - c[0])**2 + (cell[1] - c[1])**2)
                    for c in cluster
                )
                if min_dist <= cluster_distance:
                    cluster.append(unassigned.pop(i))
                    i = 0  # Restart to check against new cluster members
                else:
                    i += 1
            
            clusters.append(cluster)

        return clusters

    def filter_close_frontiers(self, frontiers: List[Tuple[float, float]], min_distance: float) -> List[Tuple[float, float]]:
        """
        Remove frontiers that are too close to each other, keeping the first one.

        Args:
            frontiers: List of (x, y) frontier positions
            min_distance: Minimum distance between frontiers in meters

        Returns:
            Filtered list of frontiers
        """
        if not frontiers:
            return []

        filtered = [frontiers[0]]
        
        for frontier in frontiers[1:]:
            # Check distance to all already-added frontiers
            too_close = False
            for existing in filtered:
                distance = math.sqrt(
                    (frontier[0] - existing[0])**2 + (frontier[1] - existing[1])**2
                )
                if distance < min_distance:
                    too_close = True
                    break
            
            if not too_close:
                filtered.append(frontier)

        return filtered

    def publish_frontiers(self, frontiers: List[Tuple[float, float]]):
        """
        Publish frontiers as Float32MultiArray.

        Args:
            frontiers: List of (x, y) frontier positions
        """
        msg = Float32MultiArray()
        
        # Create data array: [x1, y1, x2, y2, ...]
        data = []
        for x, y in frontiers:
            data.append(float(x))
            data.append(float(y))
        
        msg.data = data
        
        # Set layout: 2 columns (x, y) and N rows (number of frontiers)
        from std_msgs.msg import MultiArrayDimension
        
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "frontiers"
        msg.layout.dim[0].size = len(frontiers)
        msg.layout.dim[0].stride = 2  # 2 values per frontier
        
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "coordinates"
        msg.layout.dim[1].size = 2
        msg.layout.dim[1].stride = 2
        
        msg.layout.data_offset = 0
        
        self.frontiers_publisher.publish(msg)


def main(args=None):
    """Main entry point for frontier detector node."""
    rclpy.init(args=args)

    node = FrontierDetectorNode(robot_id=None)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


