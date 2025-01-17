# common_types.py
from dataclasses import dataclass
import numpy as np
from typing import List, Optional, Dict, Tuple

@dataclass
class BlendingConstraints:
    """Continuity constraints for trajectory blending"""
    position: np.ndarray          # Position at join point
    first_derivative: np.ndarray  # First derivative at join point 
    second_derivative: np.ndarray # Second derivative at join point
    velocity: float              # Velocity at join point
    time: float                  # Time at join point

@dataclass
class PlanningPoint:
    """Represents a point along the trajectory with kinematic data."""
    position: np.ndarray          # x, y position
    curvature: float             # curvature at this point
    arc_length: float            # cumulative arc length to this point
    heading: float = 0.0         # heading in radians
    velocity: float = 0.0        # planned velocity
    time: float = 0.0            # time to reach this point
    segment_idx: int = 0         # index of spline segment
    parameter_t: float = 0.0     # parameter t within spline segment
    
    # Store velocities at different stages
    vel_of_iso_constraints: float = 0.0  # velocity from isolated constraints
    
    # Forward pass data
    forward_overlap_bound: float = float('inf')
    forward_trans_bounds: str = ""
    forward_rot_bounds_prime: str = ""
    forward_rot_bounds: str = ""
    forward_valid_bounds: str = ""
    
    # Backward pass data
    backward_overlap_bound: float = float('inf')
    backward_trans_bounds: str = ""
    backward_rot_bounds_prime: str = ""
    backward_rot_bounds: str = ""
    backward_valid_bounds: str = ""

@dataclass
class TrajectoryInfo:
    """Complete trajectory information"""
    waypoints: List[np.ndarray]         # Waypoints defining the path
    tangent_factors: List[float]        # Tangent factors for each waypoint
    spline_points: List[np.ndarray]     # Points along the spline
    planning_points: List[PlanningPoint] # Points with velocity & curvature info
    velocity_profile: List[PlanningPoint] # Complete velocity profile
    total_time: float                   # Total traversal time
    
    # Add spline data needed for blending
    spline_segments: List  # Contains control points
    point_params: List[Tuple[int, float, float]]  # (segment_index, local_parameter, heading) for each point