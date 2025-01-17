"""
Window manager module for handling successive trajectory windows.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

from sympy import Range
from amr_local_planner_py.centerline_processor import CenterlineProcessor, TrajectoryWindow, PreprocessingConfig
from amr_local_planner_py.trajectory_optimizer import TrajectoryOptimizer, TrajectoryOptimizerConfig
from amr_local_planner_py.trajectory_blending import TrajectoryBlender
from amr_local_planner_py.common_types import BlendingConstraints, PlanningPoint, TrajectoryInfo
from amr_local_planner_py.spline_generation import BezierSplineGenerator
from amr_local_planner_py.velocity_profile import VelocityProfileGenerator, ROBOT_CONSTRAINTS, PlanningPoint

@dataclass
class WindowConfig:
    """Configuration for window management"""
    planning_time: float = 1.5    # Time needed for planning (sec)
    buffer_time: float = 0.5      # Extra buffer time (sec)
    lookahead_points: int = 6     # Number of points to look ahead. i.e 6m ahead since waypoints are pre processed to have ~1m spacing
    window_spacing: float = 2.0   # Approx spacing between points (meters)

class WindowManager:
    """Manages successive trajectory windows with smooth transitions"""
    
    def __init__(self, lanelet_dict, lanelet_ids, config: Optional[WindowConfig] = None):
        
        self.config = config or WindowConfig()
        
        # Create preprocessing config based on window config
        preprocessing_config = PreprocessingConfig(
            method="linear",  # Keep default method
            target_spacing=self.config.window_spacing,  # Use window spacing
            bezier_window_size=6,  # Keep other defaults
            bezier_overlap=2,
            spacing_tolerance=0.1
        )
        self.centerline_processor = CenterlineProcessor(lanelet_dict, lanelet_ids, preprocessing_config)
        self.trajectory_blender = TrajectoryBlender()
        
        # Core components
        self.optimizer = None  # Will be initialized with first window
        
        # State tracking
        self.current_window = None
        self.current_trajectory = None
        self.window_start_idx = 0
        self.is_first_window = True
        self.current_window_is_last = False
        self.optimizer_config = TrajectoryOptimizerConfig(
                                                            num_points=150,                    # More points for better accuracy
                                                            safety_margin=0.2,                # Slightly larger safety margin
                                                            check_stride=5,                    # Check more frequently for collisions
                                                            base_tangent_factor=0.5,           # More conservative tangent scaling
                                                            optimization_time_limit=1.0,       # Allow more time for optimization
                                                            distance_map_resolution=0.2,      # Finer distance map grid
                                                            distance_map_window_size=3,        # Larger window for distance map
                                                            spline_points_method="linear"     # More accurate arc length calculation. linear or simpson
                                                        )  # Proper optimizer config
        self.next_planning_time = None
        self.join_point = None  # Will store join point for next window
        
    def check_if_last_window(self, window: TrajectoryWindow) -> bool:
        """Determine if this is the last window"""
        return len(window.points) <= 2  # Consider last window if <= 3 waypoints
        
    def initialize_optimizer(self, window: TrajectoryWindow):
        """Initialize or update optimizer with new window data"""
        if not self.optimizer:
            self.optimizer = TrajectoryOptimizer(
                window.left_boundary,
                window.right_boundary,
                self.optimizer_config
            )
        else:
            # Update existing optimizer with new boundaries
            self.optimizer.update_boundaries(
                window.left_boundary,
                window.right_boundary,
            )
            self.optimizer.evaluating_initial_trajectory = True  # Reset flag for new window
            self.optimizer.reset_rprop_states()  # Reset RPROP states for new window
            
    def calculate_next_planning_time(self):
        """Calculate when to start planning next window"""
        if not self.current_trajectory:
            return None
                
        if self.is_first_window:
            # For first window, use last point time directly
            total_time = self.current_trajectory.planning_points[-1].time
        else:
            # For subsequent windows, calculate actual duration
            start_time = self.current_trajectory.planning_points[0].time
            end_time = self.current_trajectory.planning_points[-1].time
            total_time = end_time - start_time  # Get actual duration
            
        planning_time = self.config.planning_time
        buffer_time = self.config.buffer_time
            
        # Calculate when to start planning next window
        self.next_planning_time = total_time - (planning_time + buffer_time)
        return self.next_planning_time
    
    def find_nearest_next_waypoint(self, join_position: np.ndarray) -> int:
        """
        Find index of next waypoint after join point while maintaining velocity continuity
        
        Args:
            join_position: Position of join point from current trajectory
            
        Returns:
            Index of next waypoint to use
        """
        processed_points = self.centerline_processor.processed_points
        total_points = len(processed_points)
        
        start_search_idx = self.window_start_idx + 1 if not self.is_first_window else 0
        print(f"Finding next waypoint after join point, starting from index {start_search_idx}")
        
        # Check if we have enough remaining points
        remaining_points = total_points - start_search_idx
        if remaining_points < 2:  # Need at least 2 points for next window
            print(f"Warning: Only {remaining_points} points remaining - at end of trajectory")
            return total_points - 1  # Return last valid index
        
        # Find first waypoint after join point in movement direction
        for i in range(start_search_idx, len(processed_points)):
            point = np.array(processed_points[i])
            
            # Get direction from join point to this waypoint
            direction = point - join_position
            
            # Get path direction at this point
            if i > 0:
                path_vector = point - np.array(processed_points[i-1])
                
                # Check if waypoint is in forward direction
                if np.dot(direction, path_vector) > 0:
                    # Found a waypoint in forward direction
                    print(f"Found waypoint at index {i}")
                    return i
        
        # Fallback: return next sequential index
        next_idx = min(self.window_start_idx + 1, len(processed_points) - 1)
        print(f"No forward waypoint found, using next index {next_idx}")
        return next_idx
    
    
    def prepare_window(self, robot_position: np.ndarray) -> TrajectoryWindow:
        """Prepare window geometry based on either current position or projected join point"""
        if self.is_first_window:
            self.window_start_idx = 0
            window = self.centerline_processor.get_trajectory_window(
                self.window_start_idx,
                self.config.lookahead_points
            )
            if window is None:
                print("Failed to get initial window")
                return None, None
            window.points[0] = robot_position
            join_idx = 0
        else:
            # Find join point in current trajectory
            join_idx = self._find_join_point(
                self.current_trajectory.planning_points,
                robot_position
            )
            
            if join_idx is not None:
                # Get projected position
                join_position = self.current_trajectory.planning_points[join_idx].position
                
                # Find waypoints starting from join position
                self.window_start_idx = self.find_nearest_next_waypoint(join_position)
                window = self.centerline_processor.get_trajectory_window(
                    self.window_start_idx,
                    self.config.lookahead_points
                )
                
                if window is None:
                    print("Failed to get next window")
                    return None, None
                
                # Start from projected position
                window.points[0] = join_position
            else:
                print("No valid join point found - fallback to current position")
                self.window_start_idx = self.find_nearest_next_waypoint(robot_position)
                window = self.centerline_processor.get_trajectory_window(
                    self.window_start_idx,
                    self.config.lookahead_points
                )
                if window is None:
                    print("Failed to get next window")
                    return None, None
                window.points.insert(0, robot_position)

        # Check if this is last window
        self.current_window_is_last = self.check_if_last_window(window)
        
        return window, join_idx
    
    # def prepare_window(self, robot_position: np.ndarray) -> TrajectoryWindow:
    #     """Prepare window geometry based on either current position or projected join point"""
    #     if self.is_first_window:
    #         self.window_start_idx = 0
    #         window = self.centerline_processor.get_trajectory_window(
    #             self.window_start_idx,
    #             self.config.lookahead_points
    #         )
    #         if window is None:
    #             print("Failed to get initial window")
    #             return None, None
    #         window.points[0] = robot_position  # Replace first point
    #         join_idx = 0
    #     else:
    #         # Find join point in current trajectory
    #         join_idx = self._find_join_point(
    #             self.current_trajectory.planning_points,
    #             robot_position
    #         )
            
    #         if join_idx is not None:
    #             # Get projected position
    #             join_position = self.current_trajectory.planning_points[join_idx].position
                
    #             # Find waypoints starting from join position
    #             self.window_start_idx = self.find_nearest_next_waypoint(join_position)
    #             window = self.centerline_processor.get_trajectory_window(
    #                 self.window_start_idx,
    #                 self.config.lookahead_points
    #             )
                
    #             if window is None:
    #                 print("Failed to get next window")
    #                 return None, None
                
    #             # Get the first waypoint in window
    #             first_waypoint = np.array(window.points[0])
                
    #             # Calculate distance to first waypoint
    #             dist_to_first = np.linalg.norm(join_position - first_waypoint)
                
    #             if dist_to_first > self.config.window_spacing * 0.5:
    #                 # If distance is significant, prepend join_position
    #                 window.points.insert(0, join_position)
    #             else:
    #                 # Otherwise replace first waypoint
    #                 window.points[0] = join_position
                
    #             # Update window boundaries based on modified points
    #             point_lanelets = [self.centerline_processor._find_point_lanelet(p) for p in window.points]
    #             combined_lanelets = self.centerline_processor._combine_active_lanelets(point_lanelets)
                
    #             # Get updated boundaries
    #             left_bounds, right_bounds = self.centerline_processor._get_window_bounds(combined_lanelets)
    #             window.left_boundary = left_bounds
    #             window.right_boundary = right_bounds
    #             window.active_lanelet_ids = combined_lanelets
                
    #         else:
    #             print("No valid join point found - fallback to current position")
    #             self.window_start_idx = self.find_nearest_next_waypoint(robot_position)
    #             window = self.centerline_processor.get_trajectory_window(
    #                 self.window_start_idx,
    #                 self.config.lookahead_points
    #             )
    #             if window is None:
    #                 print("Failed to get next window")
    #                 return None, None
    #             window.points[0] = robot_position  # Replace first point

    #         # Check if this is last window
    #         self.current_window_is_last = self.check_if_last_window(window)
            
    #         return window, join_idx
    
    def _find_closest_planning_point(self, planning_points: List[PlanningPoint], position: np.ndarray) -> Optional[int]:
        """Find the next closest point ahead of the robot along the trajectory."""
        if len(planning_points) < 2:
            return None  # Need at least two points for direction comparison

        for idx in range(1, len(planning_points)):
            # Vector along the trajectory
            v1 = planning_points[idx].position - planning_points[idx - 1].position
            # Vector from robot to the next point
            v2 = planning_points[idx].position - position

            # Check if the next point is forward using dot product
            if np.dot(v1, v2) > 0:  
                print(f"Next forward point found at index {idx}")
                return idx

        # If no point ahead is found, return the last point
        print("No valid forward point found - returning last point")
        return len(planning_points) - 1

    def _find_join_point(self, planning_points: List[PlanningPoint], position: np.ndarray) -> Optional[int]:
        """Find a future join point for trajectory blending based on projected time."""
        if not planning_points:
            return None

        # Step 1: Find the next point ahead of the robot
        closest_idx = self._find_closest_planning_point(planning_points, position)

        # Step 2: Calculate projected future time
        planning_delay = self.config.planning_time + self.config.buffer_time
        projected_time = planning_points[closest_idx].time + planning_delay

        # Step 3: Iterate forward to find the first point reaching the projected time
        for idx in range(closest_idx, len(planning_points)):
            if planning_points[idx].time >= projected_time:
                print(f"Join point found at index {idx}, time: {planning_points[idx].time:.2f}")
                return idx

        # Step 4: If no valid point found, return the last point
        print("No suitable join point found, defaulting to last point.")
        return len(planning_points) - 1

    
    def process_window(self, robot_position: np.ndarray) -> Optional[TrajectoryInfo]:
        """Process window with proper initialization"""
        try:
            # Don't continue if we already processed last window
            if self.current_window_is_last and self.current_trajectory:
                print("Last window already processed - no more windows needed")
                return None
            
            # Prepare window geometry
            window, join_idx = self.prepare_window(robot_position)
            
            if not window.points:
                print("No points in window - might be at end of path")
                return None
            
            if window is None:
                print("Could not prepare window - might be at end of path")
                if self.current_trajectory:
                    # We have a previous trajectory - mark it as last
                    self.current_window_is_last = True
                return None
            
            # Check if this is last window
            self.current_window_is_last = self.check_if_last_window(window)
            
            # Initialize/update optimizer
            self.initialize_optimizer(window)
            
            # Extract blend data if not first window
            blend_data = None
            if not self.is_first_window and self.current_trajectory:
                if join_idx is not None:
                    blend_data = self.trajectory_blender.extract_blend_data(
                        self.current_trajectory,
                        join_idx
                    )
            
            # Update optimizer's spline configuration
            self.optimizer.update_spline_config(
                is_first_window=self.is_first_window,
                is_last_window=self.current_window_is_last,
                blend_data=blend_data
            )
            
            # Convert points to numpy arrays
            waypoints = [np.array(point) for point in window.points]
            print(f"Wayspoints: {waypoints}")
            
            # Generate initial trajectory
            initial_trajectory = self.optimizer.evaluate_trajectory(
                waypoints=waypoints,
                tangent_factors=[self.optimizer_config.base_tangent_factor] * len(waypoints),
                distance_map=self.optimizer.distance_map,
                X=self.optimizer.X,
                Y=self.optimizer.Y,
            )
            print(f"Inital Trajectory Total Time: {initial_trajectory.total_time}")
            if initial_trajectory.total_time == float('inf'):
                print("Initial Trajectory resulted in collision.")
            else:
                print(f"Initial trajectory stats:\n"
                    f"      Tangent factors: {initial_trajectory.tangent_factors},\n"
                    f"      Time range: {initial_trajectory.planning_points[0].time} - {initial_trajectory.planning_points[-1].time},\n"
                    f"      Planning Points Range: {initial_trajectory.planning_points[0].position} - {initial_trajectory.planning_points[-1].position},\n"
                    f"      Velocity Range: {initial_trajectory.planning_points[0].velocity} - {initial_trajectory.planning_points[-1].velocity}")

            
            # Optimize trajectory
            best_trajectory, stats = self.optimizer.optimize_trajectory(initial_trajectory)
            print(f"Optimization stats: {stats}")
            # best_trajectory = initial_trajectory
            
            # Update state
            self.current_window = window
            self.current_trajectory = best_trajectory
            self.is_first_window = False
            
            return best_trajectory, initial_trajectory
            
        except Exception as e:
            print(f"Error processing window: {str(e)}")
            if self.current_trajectory:
                self.current_window_is_last = True
            return None
    

"""
Usage in main lcal planne node:

def run_local_planner(lanelet_dict, lanelet_ids):
    window_manager = WindowManager(lanelet_dict, lanelet_ids)
    control_rate = 100  # Hz
    rate_period = 1.0/control_rate
    
    while True:  # Will be ROS2 rate-limited loop
        current_time = time.time()  # Will be from ROS
        
        # Get robot position (will be from localization)
        robot_position = np.array([x, y])  # Placeholder
        
        # Check if we need new trajectory
        if window_manager.next_planning_time is None or current_time >= window_manager.next_planning_time:
            # Process window
            trajectory = window_manager.process_window(robot_position)
            
            if trajectory:
                # Publish trajectory
                print("Publishing trajectory...")
                
                # Calculate next planning time
                if window_manager.is_first_window:
                    window_duration = trajectory.planning_points[-1].time  # Total time from start
                else:
                    # Calculate duration of this window
                    start_time = trajectory.planning_points[0].time
                    end_time = trajectory.planning_points[-1].time
                    window_duration = end_time - start_time
                
                window_manager.next_planning_time = (
                    current_time + 
                    window_duration - 
                    (window_manager.config.planning_time + window_manager.config.buffer_time)
                )
            
            # Break if last window completed
            if window_manager.is_last_window:
                print("Reached goal - last window completed")
                break
        
        # Sleep for control rate
        time.sleep(rate_period)

"""