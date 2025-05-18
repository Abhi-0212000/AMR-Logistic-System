"""
Window manager module for handling successive trajectory windows.
Uses LaneletMapManager to access lanelet data.
"""

import numpy as np
from typing import Optional, Dict, List, Tuple, Any, NamedTuple
from rclpy.node import Node

from amr_trajectory_planner.lanelet_map_manager import LaneletMapManager
from amr_trajectory_planner.core_modules.centerline_processor import (
    CenterlineProcessor,
    TrajectoryWindow,
)
from amr_trajectory_planner.core_modules.trajectory_optimizer import (
    TrajectoryOptimizer,
)
from amr_trajectory_planner.core_modules.trajectory_blending import TrajectoryBlender
from amr_trajectory_planner.config_types import *


class WindowManager:
    """
    Manages successive trajectory windows with smooth transitions.
    Uses LaneletMapManager to access lanelet data.
    """

    def __init__(self, node: Node):
        """
        Initialize WindowManager with a ROS2 node that contains configuration.

        Args:
            node: ROS2 Node instance with configuration and for logging
        """
        self.node = node
        self.config = self.node.window_config

        # Initialize components
        self.centerline_processor = CenterlineProcessor(self.node)
        self.node.get_logger().info("Processed centerline points of all lanelets")
        self.trajectory_blender = TrajectoryBlender()
        self.optimizer = None

        # Initialize state tracking
        self.reset_state_tracking()
        self.node.get_logger().info("WindowManager initialized")
    
    def reset_state_tracking(self):
        """Reset state tracking variables for a new planning session"""
        self.current_window = None
        self.current_trajectory = None
        self.initial_trajectory = None
        self.trajectories = []
        self.window_start_idx = 0
        self.is_first_window = True
        self.current_window_is_last = False
        self.next_planning_time = None
        self.node.get_logger().info("State tracking variables reset")

    def check_if_last_window(self, window: TrajectoryWindow) -> bool:
        """Determine if this is the last window based on remaining points"""
        # Consider last window if less than 3 waypoints remain
        return len(window.points) <= 2

    def initialize_optimizer(self, window: TrajectoryWindow):
        """Initialize or update trajectory optimizer with window data"""
        if not self.optimizer:
            self.optimizer = TrajectoryOptimizer(
                self.node, window.left_boundary, window.right_boundary
            )
        else:
            # Update existing optimizer with new boundaries
            self.optimizer.update_boundaries(
                window.left_boundary,
                window.right_boundary,
            )
            # Reset optimizer state for new window
            self.optimizer.evaluating_initial_trajectory = True
            self.optimizer.reset_rprop_states()

    def find_nearest_next_waypoint(self, join_position: np.ndarray) -> int:
        """Find index of next waypoint after join point"""
        if not self.centerline_processor or not self.centerline_processor.processed_points:
            self.node.get_logger().error("No processed points available")
            return 0

        processed_points = self.centerline_processor.processed_points
        total_points = len(processed_points)

        # Start searching from the next point after current window start
        start_search_idx = self.window_start_idx + 1 if not self.is_first_window else 0
        self.node.get_logger().info(f"Finding next waypoint, starting from index {start_search_idx}")

        # Check if we have enough remaining points
        remaining_points = total_points - start_search_idx
        if remaining_points < 2:
            self.node.get_logger().warn(f"Only {remaining_points} points remaining - at end of trajectory")
            return total_points - 1  # Return last valid index

        # Find first waypoint after join point in movement direction
        for i in range(start_search_idx, total_points):
            point = np.array(processed_points[i])

            # Direction from join point to this waypoint
            direction = point - join_position

            # Path direction at this point
            if i > 0:
                path_vector = point - np.array(processed_points[i - 1])

                # Check if waypoint is in forward direction
                if np.dot(direction, path_vector) > 0:
                    self.node.get_logger().info(f"Found next waypoint at index {i}")
                    return i

        # Fallback to next sequential index
        next_idx = min(self.window_start_idx + 1, total_points - 1)
        self.node.get_logger().warn(f"No forward waypoint found, using next index {next_idx}")
        return next_idx

    def prepare_window(
        self, robot_position: np.ndarray
    ) -> Tuple[Optional[TrajectoryWindow], Optional[int]]:
        """
        Prepare a trajectory window based on current position or projected join point

        Args:
            robot_position: Current robot position as [x, y] numpy array

        Returns:
            Tuple of (trajectory window, join index)
        """
        if not self.centerline_processor:
            self.node.get_logger().error("Centerline processor not initialized")
            return None, None

        if self.is_first_window:
            # For first window, start at robot position
            self.window_start_idx = 0
            window = self.centerline_processor.get_trajectory_window(
                self.window_start_idx, self.config.lookahead_points
            )
            if window is None:
                self.node.get_logger().error("Failed to get initial window")
                return None, None

            # Replace first point with actual robot position
            window.points[0] = robot_position
            join_idx = 0
        else:
            # Find join point in current trajectory
            join_idx = self._find_join_point(
                self.current_trajectory.planning_points, robot_position
            )

            if join_idx is not None:
                # Get projected position from current trajectory
                join_position = self.current_trajectory.planning_points[join_idx].position

                # Find waypoints starting from join position
                self.window_start_idx = self.find_nearest_next_waypoint(join_position)
                window = self.centerline_processor.get_trajectory_window(
                    self.window_start_idx, self.config.lookahead_points
                )

                if window is None:
                    self.node.get_logger().error("Failed to get next window")
                    return None, None

                # Start window from the join position
                window.points[0] = join_position
            else:
                # Fallback to using current position
                self.node.get_logger().warn("No valid join point found - using current position")
                self.window_start_idx = self.find_nearest_next_waypoint(robot_position)
                window = self.centerline_processor.get_trajectory_window(
                    self.window_start_idx, self.config.lookahead_points
                )
                if window is None:
                    self.node.get_logger().error("Failed to get next window")
                    return None, None
                window.points[0] = robot_position

        # Check if this is the last window
        self.current_window_is_last = self.check_if_last_window(window)
        if self.current_window_is_last:
            self.node.get_logger().info("This is the final window in the trajectory")

        return window, join_idx

    def _find_closest_planning_point(
        self, planning_points: List[PlanningPoint], position: np.ndarray
    ) -> Optional[int]:
        """Find the next closest point ahead of the robot along the trajectory"""
        if len(planning_points) < 2:
            self.node.get_logger().warn("Need at least two planning points for direction comparison")
            return None

        for idx in range(1, len(planning_points)):
            # Vector along the trajectory
            v1 = planning_points[idx].position - planning_points[idx - 1].position
            # Vector from robot to the next point
            v2 = planning_points[idx].position - position

            # Check if the next point is forward using dot product
            if np.dot(v1, v2) > 0:
                self.node.get_logger().info(f"Next forward point found at index {idx}")
                return idx

        # If no point ahead is found, return the last point
        self.node.get_logger().warn("No valid forward point found - returning last point")
        return len(planning_points) - 1

    def _find_join_point(
        self, planning_points: List[PlanningPoint], position: np.ndarray
    ) -> Optional[int]:
        """Find a future join point for trajectory blending"""
        if not planning_points:
            self.node.get_logger().warn("No planning points available")
            return None

        # Find the next point ahead of the robot
        closest_idx = self._find_closest_planning_point(planning_points, position)
        if closest_idx is None:
            return None

        # Calculate projected future time
        planning_delay = self.config.planning_time + self.config.buffer_time
        projected_time = planning_points[closest_idx].time + planning_delay

        # Find the first point reaching the projected time
        for idx in range(closest_idx, len(planning_points)):
            if planning_points[idx].time >= projected_time:
                self.node.get_logger().info(f"Join point found at index {idx}, time: {planning_points[idx].time:.2f}")
                return idx

        # If no valid point found, return the last point
        self.node.get_logger().warn("No suitable join point found, defaulting to last point")
        return len(planning_points) - 1

    def process_window(
        self, robot_position: np.ndarray
    ) -> Tuple[Optional[TrajectoryInfo], Optional[TrajectoryInfo]]:
        """
        Process current window to generate an optimized trajectory

        Args:
            robot_position: Current robot position as [x, y] numpy array

        Returns:
            Tuple of (optimized trajectory, initial trajectory) or (None, None) if failed
        """
        try:
            # Prepare window geometry
            window, join_idx = self.prepare_window(robot_position)

            if window is None or not window.points:
                self.node.get_logger().warn("Invalid window - might be at end of path")
                return None, None

            # Initialize/update optimizer
            self.initialize_optimizer(window)

            # Extract blend data if not first window
            blend_data = None
            if not self.is_first_window and self.current_trajectory and join_idx is not None:
                blend_data = self.trajectory_blender.extract_blend_data(
                    self.current_trajectory, join_idx
                )

            # Update optimizer's spline configuration
            self.optimizer.update_spline_config(
                is_first_window=self.is_first_window,
                is_last_window=self.current_window_is_last,
                blend_data=blend_data,
            )

            # Convert points to numpy arrays
            waypoints = [np.array(point) for point in window.points]
            self.node.get_logger().info(f"Processing window with {len(waypoints)} waypoints")

            # Generate initial trajectory
            initial_trajectory = self.optimizer.evaluate_trajectory(
                waypoints=waypoints,
                tangent_factors=[self.node.trajectory_optimization_config.base_tangent_factor] * len(waypoints),
                distance_map=self.optimizer.distance_map,
                X=self.optimizer.X,
                Y=self.optimizer.Y,
            )

            # Check if initial trajectory is valid
            if initial_trajectory.total_time == float("inf"):
                self.node.get_logger().error("Initial trajectory resulted in collision or is invalid")
                return None, None

            self.node.get_logger().info(
                f"Initial trajectory time range: {initial_trajectory.planning_points[0].time:.2f}s - "
                f"{initial_trajectory.planning_points[-1].time:.2f}s"
            )

            # Optimize trajectory
            best_trajectory, stats = self.optimizer.optimize_trajectory(initial_trajectory)
            self.node.get_logger().info(
                f"Optimization completed: {stats['iterations']} iterations, "
                f"time: {stats['time']:.3f}s, final_cost: {stats['final_cost']:.4f}"
            )

            # Store trajectories for record-keeping
            self.current_window = window
            self.current_trajectory = best_trajectory
            self.initial_trajectory = initial_trajectory
            self.trajectories.append(best_trajectory)

            # Update state
            self.is_first_window = False

            return best_trajectory, initial_trajectory

        except Exception as e:
            import traceback

            self.node.get_logger().error(f"Error processing window: {str(e)}\n{traceback.format_exc()}")
            return None, None

    def is_goal_reached(self) -> bool:
        """Check if goal is reached (last window processed)"""
        return self.current_window_is_last and self.current_trajectory is not None

    def calculate_next_planning_time(self) -> float:
        """
        Calculate when to start planning the next window
        Returns time in seconds from the start of current trajectory
        """
        if not self.current_trajectory:
            return 0.0

        if self.is_first_window:
            # For first window, use last point time directly
            total_time = self.current_trajectory.planning_points[-1].time
        else:
            # For subsequent windows, calculate actual duration
            start_time = self.current_trajectory.planning_points[0].time
            end_time = self.current_trajectory.planning_points[-1].time
            total_time = end_time - start_time

        # Calculate when to start planning next window
        next_time = total_time - (self.config.planning_time + self.config.buffer_time)
        self.next_planning_time = max(0.0, next_time)  # Ensure non-negative time
        return self.next_planning_time

    def get_remaining_distance(self) -> float:
        """Estimate remaining distance to goal"""
        if self.current_window_is_last:
            # If in last window, use the distance to the end of current trajectory
            if self.current_trajectory and len(self.current_trajectory.planning_points) > 1:
                start = self.current_trajectory.planning_points[0].position
                end = self.current_trajectory.planning_points[-1].position
                return np.linalg.norm(end - start)
            return 0.0

        # Otherwise estimate from centerline processor
        if self.centerline_processor and self.centerline_processor.processed_points:
            processed_points = self.centerline_processor.processed_points
            remaining_dist = 0.0

            # Start from current window and sum distances
            for i in range(self.window_start_idx, len(processed_points) - 1):
                p1 = np.array(processed_points[i])
                p2 = np.array(processed_points[i + 1])
                remaining_dist += np.linalg.norm(p2 - p1)

            return remaining_dist

        return 0.0

    def get_remaining_time(self) -> float:
        """Estimate remaining time to goal"""
        if not self.current_trajectory:
            return 0.0

        # If in last window, use time to end of current trajectory
        if self.current_window_is_last:
            if len(self.current_trajectory.planning_points) > 0:
                current_time = self.current_trajectory.planning_points[0].time
                end_time = self.current_trajectory.planning_points[-1].time
                return max(0.0, end_time - current_time)
            return 0.0

        # Use velocity data to estimate remaining time
        remaining_distance = self.get_remaining_distance()

        # Use average velocity from current trajectory
        if len(self.current_trajectory.planning_points) > 1:
            velocities = [p.velocity for p in self.current_trajectory.planning_points]
            avg_velocity = sum(velocities) / len(velocities)
            if avg_velocity > 0.01:  # Avoid division by near-zero
                return remaining_distance / avg_velocity

        # Fallback to simple time estimate
        return remaining_distance / 1.0  # Assume 1 m/s as default

    def get_total_distance(self) -> float:
        """Get the total path distance"""
        if self.centerline_processor and self.centerline_processor.processed_points:
            processed_points = self.centerline_processor.processed_points
            total_dist = 0.0

            for i in range(len(processed_points) - 1):
                p1 = np.array(processed_points[i])
                p2 = np.array(processed_points[i + 1])
                total_dist += np.linalg.norm(p2 - p1)

            return total_dist

        return 0.0
