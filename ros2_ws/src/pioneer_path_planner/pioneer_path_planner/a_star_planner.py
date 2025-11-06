#!/usr/bin/env python3
"""
A* Path Planning Algorithm Implementation

This module provides A* path planning on a 2D occupancy grid.
Supports configurable grid resolution, inflation radius, and coordinate transforms.
"""

import math
import heapq
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from enum import IntEnum


class CellState(IntEnum):
    """Occupancy grid cell states"""
    FREE = 0
    UNKNOWN = -1
    OCCUPIED = 100


@dataclass
class GridCell:
    """Represents a cell in the occupancy grid"""
    x: int
    y: int
    state: CellState = CellState.FREE


@dataclass
class Node:
    """A* search node"""
    x: int
    y: int
    g_cost: float  # Cost from start to this node
    h_cost: float  # Heuristic cost from this node to goal
    parent: Optional['Node'] = None

    @property
    def f_cost(self) -> float:
        """Total estimated cost (g + h)"""
        return self.g_cost + self.h_cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __lt__(self, other):
        # For heapq: lower f_cost has higher priority
        if self.f_cost != other.f_cost:
            return self.f_cost < other.f_cost
        return self.h_cost < other.h_cost


class AStarPlanner:
    """
    A* path planner for 2D occupancy grids.
    
    Features:
    - Configurable grid resolution
    - Inflation radius for robot safety
    - 8-connected neighbor exploration
    - Path smoothing
    - Coordinate transformation (world <-> grid)
    """

    def __init__(
        self,
        grid_resolution: float = 0.2,
        inflation_radius: float = 0.5,
        world_bounds: Optional[Tuple[float, float, float, float]] = None,
        allow_unknown: bool = True
    ):
        """
        Initialize A* planner.

        Args:
            grid_resolution: Size of each grid cell in meters (default: 0.2m)
            inflation_radius: Safety radius around obstacles in meters (default: 0.5m)
            world_bounds: (x_min, x_max, y_min, y_max) in world coordinates.
                         If None, will be set dynamically.
            allow_unknown: If True, treat unknown cells as traversable (for exploration).
                          If False, treat unknown cells as obstacles (for navigation safety).
        """
        self.grid_resolution = grid_resolution
        self.inflation_radius = inflation_radius
        self.inflation_cells = int(math.ceil(inflation_radius / grid_resolution))
        self.allow_unknown = allow_unknown

        # World bounds in meters (x_min, x_max, y_min, y_max)
        if world_bounds:
            self.world_bounds = world_bounds
        else:
            # Default: 30m x 30m arena (from pioneer_world.wbt)
            self.world_bounds = (-15.0, 15.0, -15.0, 15.0)

        # Grid dimensions in cells
        self.grid_width = int(math.ceil(
            (self.world_bounds[1] - self.world_bounds[0]) / grid_resolution
        ))
        self.grid_height = int(math.ceil(
            (self.world_bounds[3] - self.world_bounds[2]) / grid_resolution
        ))

        # Initialize grid (all cells FREE by default)
        # Grid is indexed as grid[y][x] where (0,0) is bottom-left
        self.grid: List[List[CellState]] = [
            [CellState.FREE for _ in range(self.grid_width)]
            for _ in range(self.grid_height)
        ]
        
        # Store last failure reason for debugging
        self.last_failure_reason: Optional[str] = None

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid indices.

        Args:
            world_x: X coordinate in meters
            world_y: Y coordinate in meters

        Returns:
            (grid_x, grid_y) tuple of grid cell indices
        """
        grid_x = int((world_x - self.world_bounds[0]) / self.grid_resolution)
        grid_y = int((world_y - self.world_bounds[2]) / self.grid_resolution)

        # Clamp to valid grid bounds
        grid_x = max(0, min(self.grid_width - 1, grid_x))
        grid_y = max(0, min(self.grid_height - 1, grid_y))

        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid indices to world coordinates (center of cell).

        Args:
            grid_x: X grid index
            grid_y: Y grid index

        Returns:
            (world_x, world_y) tuple in meters
        """
        world_x = self.world_bounds[0] + (grid_x + 0.5) * self.grid_resolution
        world_y = self.world_bounds[2] + (grid_y + 0.5) * self.grid_resolution
        return world_x, world_y

    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid cell indices are within bounds."""
        return (0 <= grid_x < self.grid_width and
                0 <= grid_y < self.grid_height)

    def is_cell_free(self, grid_x: int, grid_y: int) -> bool:
        """
        Check if cell is free (not occupied).
        Behavior depends on allow_unknown parameter:
        - If allow_unknown=True: UNKNOWN cells are treated as traversable (exploration mode)
        - If allow_unknown=False: UNKNOWN cells are treated as obstacles (safety mode)
        """
        if not self.is_valid_cell(grid_x, grid_y):
            return False
        cell_state = self.grid[grid_y][grid_x]
        
        # OCCUPIED cells are always blocked
        if cell_state == CellState.OCCUPIED:
            return False
        
        # FREE cells are always allowed
        if cell_state == CellState.FREE:
            return True
        
        # UNKNOWN cells: depends on configuration
        if cell_state == CellState.UNKNOWN:
            return self.allow_unknown
        
        # Default: allow
        return True

    def set_cell_state(self, grid_x: int, grid_y: int, state: CellState):
        """Set the state of a grid cell."""
        if self.is_valid_cell(grid_x, grid_y):
            self.grid[grid_y][grid_x] = state

    def inflate_obstacles(self):
        """
        Inflate occupied cells by inflation_radius.
        Marks cells within inflation_radius of obstacles as OCCUPIED.
        """
        # Create a copy of the grid for reading
        original_grid = [row[:] for row in self.grid]

        # For each occupied cell, mark nearby cells as occupied
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if original_grid[y][x] == CellState.OCCUPIED:
                    # Mark all cells within inflation radius
                    for dy in range(-self.inflation_cells, self.inflation_cells + 1):
                        for dx in range(-self.inflation_cells, self.inflation_cells + 1):
                            nx, ny = x + dx, y + dy
                            distance = math.sqrt(dx**2 + dy**2) * self.grid_resolution
                            if (self.is_valid_cell(nx, ny) and
                                distance <= self.inflation_radius):
                                self.grid[ny][nx] = CellState.OCCUPIED

    def get_neighbors(self, node: Node) -> List[Tuple[int, int, float]]:
        """
        Get valid neighboring cells with movement costs.
        
        Returns:
            List of (nx, ny, cost) tuples for 8-connected neighbors
        """
        neighbors = []
        # 8-connected neighbors (including diagonals)
        directions = [
            (0, 1, 1.0),   # Up
            (1, 0, 1.0),   # Right
            (0, -1, 1.0),  # Down
            (-1, 0, 1.0),  # Left
            (1, 1, math.sqrt(2)),   # Up-Right
            (1, -1, math.sqrt(2)),  # Down-Right
            (-1, -1, math.sqrt(2)), # Down-Left
            (-1, 1, math.sqrt(2)),  # Up-Left
        ]

        for dx, dy, cost in directions:
            nx, ny = node.x + dx, node.y + dy
            if self.is_cell_free(nx, ny):
                neighbors.append((nx, ny, cost))
        return neighbors

    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        Euclidean distance heuristic (admissible for A*).
        
        Args:
            x1, y1: Start cell coordinates
            x2, y2: Goal cell coordinates
            
        Returns:
            Straight-line distance estimate
        """
        dx = (x2 - x1) * self.grid_resolution
        dy = (y2 - y1) * self.grid_resolution
        return math.sqrt(dx**2 + dy**2)

    def plan_path(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path from start to goal using A* algorithm.

        Args:
            start_world: (x, y) start position in world coordinates (meters)
            goal_world: (x, y) goal position in world coordinates (meters)

        Returns:
            List of waypoints in world coordinates, or None if no path found
        """
        # Convert to grid coordinates
        start_grid = self.world_to_grid(start_world[0], start_world[1])
        goal_grid = self.world_to_grid(goal_world[0], goal_world[1])

        # Check if start and goal are free
        if not self.is_cell_free(start_grid[0], start_grid[1]):
            start_state = self.grid[start_grid[1]][start_grid[0]]
            # Store reason for debugging
            self.last_failure_reason = f"Start position not free (state: {start_state})"
            return None
        if not self.is_cell_free(goal_grid[0], goal_grid[1]):
            goal_state = self.grid[goal_grid[1]][goal_grid[0]]
            # Store reason for debugging
            self.last_failure_reason = f"Goal position not free (state: {goal_state})"
            return None

        # Initialize start node
        start_node = Node(
            x=start_grid[0],
            y=start_grid[1],
            g_cost=0.0,
            h_cost=self.heuristic(start_grid[0], start_grid[1],
                                  goal_grid[0], goal_grid[1]),
            parent=None
        )

        # Open set (priority queue) and closed set
        open_set: List[Node] = [start_node]
        heapq.heapify(open_set)
        closed_set: Set[Tuple[int, int]] = set()

        # Track best known costs (for updating nodes in open set)
        best_costs: dict = {(start_node.x, start_node.y): 0.0}

        while open_set:
            # Get node with lowest f_cost
            current = heapq.heappop(open_set)

            # Skip if we've found a better path to this node
            if (current.x, current.y) in closed_set:
                continue

            # Mark as visited
            closed_set.add((current.x, current.y))

            # Check if we reached the goal
            if current.x == goal_grid[0] and current.y == goal_grid[1]:
                # Reconstruct path
                path_grid = []
                node = current
                while node is not None:
                    path_grid.append((node.x, node.y))
                    node = node.parent

                # Reverse to get path from start to goal
                path_grid.reverse()

                # Convert grid coordinates to world coordinates
                path_world = [self.grid_to_world(x, y) for x, y in path_grid]

                # Smooth path (simple smoothing)
                return self.smooth_path(path_world)

            # Explore neighbors
            for nx, ny, move_cost in self.get_neighbors(current):
                if (nx, ny) in closed_set:
                    continue

                # Calculate costs
                tentative_g = current.g_cost + move_cost * self.grid_resolution
                h = self.heuristic(nx, ny, goal_grid[0], goal_grid[1])

                # Check if we've found a better path to this neighbor
                neighbor_key = (nx, ny)
                if neighbor_key in best_costs:
                    if tentative_g >= best_costs[neighbor_key]:
                        continue

                best_costs[neighbor_key] = tentative_g

                # Create neighbor node
                neighbor = Node(
                    x=nx,
                    y=ny,
                    g_cost=tentative_g,
                    h_cost=h,
                    parent=current
                )

                heapq.heappush(open_set, neighbor)

        # No path found - store reason for debugging
        self.last_failure_reason = f"No path found (explored {len(closed_set)} cells)"
        return None

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Simple path smoothing to remove redundant waypoints.
        If we can move directly from point A to point C without hitting obstacles,
        remove point B.
        
        IMPORTANT: Ensures minimum waypoint spacing to prevent path from collapsing
        to just start and goal.

        Args:
            path: List of waypoints in world coordinates

        Returns:
            Smoothed path with fewer waypoints, but with minimum spacing
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]  # Always keep start
        
        # Minimum distance between waypoints (meters)
        # Prevents path from collapsing to just start/goal for long paths
        MIN_WAYPOINT_SPACING = 1.0
        MAX_WAYPOINT_SPACING = 3.0  # Maximum spacing before forcing a waypoint

        # Calculate total path distance
        total_distance = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        # For long paths (>5m), ensure we have intermediate waypoints
        # This prevents path from collapsing to just start/goal
        ensure_intermediates = total_distance > 5.0

        i = 0
        while i < len(path) - 1:
            # Try to skip ahead as far as possible
            best_j = i + 1
            last_distance = 0.0
            
            for j in range(i + 2, len(path)):
                # Calculate distance from current waypoint to candidate
                dx = path[j][0] - path[i][0]
                dy = path[j][1] - path[i][1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Don't skip too far - ensure minimum waypoint spacing
                if distance > MAX_WAYPOINT_SPACING:
                    break
                
                # Check if direct path from path[i] to path[j] is clear
                if self.is_direct_path_clear(path[i], path[j]):
                    best_j = j
                    last_distance = distance
                else:
                    break
            
            # Always keep the goal (last waypoint)
            if best_j >= len(path) - 1:
                # If we're at the last waypoint, include it and stop
                smoothed.append(path[-1])
                break
            
            # For long paths, ensure we don't skip too many waypoints
            if ensure_intermediates and last_distance > MAX_WAYPOINT_SPACING:
                # Find intermediate waypoint to include
                mid_idx = (i + best_j) // 2
                if mid_idx > i and mid_idx < best_j:
                    smoothed.append(path[mid_idx])
                    i = mid_idx
                    continue
            
            smoothed.append(path[best_j])
            
            # If we didn't make progress, force advancement to prevent infinite loop
            if best_j == i + 1:
                i = best_j
            else:
                i = best_j

        # Ensure goal is always included
        if smoothed[-1] != path[-1]:
            smoothed.append(path[-1])

        return smoothed

    def is_direct_path_clear(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        num_samples: int = 10
    ) -> bool:
        """
        Check if a direct path between two points is clear of obstacles.

        Args:
            start: Start position (x, y)
            end: End position (x, y)
            num_samples: Number of points to check along the path

        Returns:
            True if path is clear, False otherwise
        """
        for i in range(num_samples + 1):
            t = i / num_samples
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])

            gx, gy = self.world_to_grid(x, y)
            if not self.is_cell_free(gx, gy):
                return False

        return True

    def update_grid_from_occupancy_map(self, occupancy_map):
        """
        Update internal grid from a nav_msgs/OccupancyGrid message.
        This will be used in Phase 2 when LiDAR data is available.

        Args:
            occupancy_map: nav_msgs/OccupancyGrid message
        """
        if not occupancy_map:
            return

        # Get occupancy map parameters
        map_resolution = occupancy_map.info.resolution
        map_width = occupancy_map.info.width
        map_height = occupancy_map.info.height
        map_origin_x = occupancy_map.info.origin.position.x
        map_origin_y = occupancy_map.info.origin.position.y

        # Check if resolutions match (or are close enough)
        if abs(map_resolution - self.grid_resolution) > 0.01:
            # Resolutions don't match - we'd need to interpolate
            # For now, skip update if resolutions are too different
            return

        # Check if origins match (or are close enough)
        if abs(map_origin_x - self.world_bounds[0]) > 0.01 or \
           abs(map_origin_y - self.world_bounds[2]) > 0.01:
            # Origins don't match - we'd need coordinate transform
            # For now, skip update if origins are too different
            return

        # Update grid cells from occupancy map
        # OccupancyGrid data is stored row-major: data[i] = grid[i/width][i%width]
        for y in range(min(map_height, self.grid_height)):
            for x in range(min(map_width, self.grid_width)):
                # OccupancyGrid is stored row-major, starting from bottom-left
                idx = y * map_width + x
                if idx >= len(occupancy_map.data):
                    continue

                occupancy_value = occupancy_map.data[idx]

                # Map occupancy grid values to CellState:
                # -1 (unknown) -> CellState.UNKNOWN
                # 0-50 (free) -> CellState.FREE
                # 51-100 (occupied) -> CellState.OCCUPIED
                if occupancy_value == -1:
                    state = CellState.UNKNOWN
                elif occupancy_value >= 0 and occupancy_value <= 50:
                    state = CellState.FREE
                elif occupancy_value > 50:
                    state = CellState.OCCUPIED
                else:
                    # Invalid value, skip
                    continue

                # Update planner grid
                if x < self.grid_width and y < self.grid_height:
                    self.grid[y][x] = state

        # Re-inflate obstacles after update
        self.inflate_obstacles()

