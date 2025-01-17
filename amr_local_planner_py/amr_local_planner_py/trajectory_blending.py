"""
Trajectory blending module for smooth transitions between trajectory windows.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List
from amr_local_planner_py.spline_generation import BezierSplineGenerator
from amr_local_planner_py.velocity_profile import VelocityProfileGenerator, PlanningPoint
from amr_local_planner_py.common_types import BlendingConstraints, PlanningPoint, TrajectoryInfo

class TrajectoryBlender:
    """Handles smooth transitions between trajectory windows"""
    
    def __init__(self):
        """Initialize trajectory blender"""
        pass
    
    def find_join_waypoint(self, points: List[PlanningPoint],
                        join_time: float) -> Optional[int]:
        """
        Find waypoint index closest to join time.
        Returns None if no suitable waypoint found.
        """
        # Find waypoint closest to join time
        for i, point in enumerate(points):
            if point.time >= join_time:
                return i
        return None

    def extract_blend_data(self, current_trajectory: TrajectoryInfo, 
                          join_point_idx: int) -> BlendingConstraints:
        """
        Extract blend data at join point for next window
        Returns:
            Dict containing:
                position: Join point position
                first_derivative: First derivative at join point
                second_derivative: Second derivative at join point
                velocity: Velocity at join point
        """
        # Get join point info
        point = current_trajectory.planning_points[join_point_idx]
    
        # Get segment and parameter info for join point
        segment_idx, param_t, heading = current_trajectory.point_params[join_point_idx]
        segment = current_trajectory.spline_segments[segment_idx]
        
        # Calculate derivatives using control points
        first_deriv = BezierSplineGenerator.calculate_derivative_from_control_points(segment.control_points, param_t, order=1)
        second_deriv = BezierSplineGenerator.calculate_derivative_from_control_points(segment.control_points, param_t, order=2)
        
        return BlendingConstraints(position=point.position,
                                   first_derivative=first_deriv,
                                   second_derivative=second_deriv,
                                   velocity=point.velocity,
                                   time=point.time)

    def calculate_next_window_time(self, 
                                 planning_time: float,
                                 total_time: float,
                                 buffer_time: float = 0.5) -> float:
        """
        Calculate when to start planning next window.
        Returns time in seconds from start of current trajectory.
        """
        return total_time - (planning_time + buffer_time)