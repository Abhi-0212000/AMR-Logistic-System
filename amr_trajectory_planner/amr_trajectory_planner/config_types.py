"""
Configuration and data types for AMR trajectory planning.

This module centralizes all configuration classes and data structures
used across the trajectory planning system.
"""

from dataclasses import dataclass, field
import numpy as np
from typing import List, Optional, Dict, Tuple, Any

# Configuration classes

# =========================== Used by WindowManager ===========================
@dataclass
class WindowConfig:
    """Configuration for trajectory window generation"""

    planning_time: float = 1.0  # Time allocated for planning (seconds)
    buffer_time: float = 0.5  # Buffer time for safety (seconds)
    lookahead_points: int = 4  # Number of points to look ahead in the path


# ===============================================================================

# =========================== Used by CenterlineProcessor ===========================
@dataclass
class CenterlineProcessorConfig:
    """Configuration for centerline processing"""

    interpolation_method: str = "spline"  # Method for interpolation ("linear", "spline", "bezier")
    target_spacing: float = 1.0  # Target spacing between processed centerline points (meters)
    spacing_tolerance: float = (
        0.1  # Tolerance for acceptable deviation from target spacing (meters)
    )
    bezier_window_size: int = 5  # Number of points in each window for bezier curve fitting
    bezier_overlap: int = 2  # Number of points to overlap between bezier windows


# ===================================================================================

# =========================== Used by TrajectoryOptimizer ===========================
@dataclass
class CollisionCheckingConfig:
    """Configuration for collision checking"""

    safety_margin: float = 0.2  # Safety margin for collision checking (meters)
    collision_check_interval: int = 5  # Stride for collision checking (check every Nth point)


@dataclass
class RPROPConfig:
    """Configuration for RPROP optimizer"""

    initial_step_size: float = 0.3  # Initial step size
    minimum_step_size: float = 1e-4  # Minimum step size
    maximum_step_size: float = 50.0  # Maximum step size
    increase_factor: float = 1.2  # Factor to increase step size
    decrease_factor: float = 0.5  # Factor to decrease step size


@dataclass
class TrajectoryOptimizerConfig:
    """Configuration for trajectory optimization"""

    trajectory_point_count: int = 150  # Number of equidistant points to be in optimized trajectory
    base_tangent_factor: float = 0.5  # Base scaling factor for spline tangent vectors
    optimization_time_limit: float = 0.8  # Maximum time allowed for optimization (seconds)
    arc_length_calculation_method: str = (
        "linear"  # Method for arc length calculation ("linear", "simpson")
    )
    collision_checking: CollisionCheckingConfig = field(default_factory=CollisionCheckingConfig)
    rprop: RPROPConfig = field(default_factory=RPROPConfig)


# ===================================================================================

# =========================== Used by DistanceMap ===========================
@dataclass
class DistanceMapConfig:
    """Configuration for distance map generation"""

    resolution: float = 0.2  # Resolution of distance map (meters)
    window_size: float = 3.0  # Size of window around path (meters)
    use_sobel: bool = True  # Use Sobel edge detection
    sobel_threshold: float = 1e-6  # Threshold for Sobel edge detection


# ===============================================================================

# =========================== Used by VelocityProfileGenerator ===========================
@dataclass
class RobotConstraintsConfig:
    """Physical constraints of the robot"""

    v_max: float = 1.5  # Maximum translational velocity (m/s)
    omega_max: float = 1.5  # Maximum rotational velocity (rad/s)
    a_t_max: float = 0.5  # Maximum translational acceleration/deceleration (m/s²)
    a_r_max: float = 1.0  # Maximum rotational acceleration/deceleration (rad/s²)
    f_max: float = 40.0  # Maximum centripetal force (N)
    mass: float = 10.0  # Mass of the robot (kg)
    t_react: float = 0.1  # Reaction time of the robot (seconds)
    wheel_base: float = 0.5  # Wheelbase of the vehicle (m)
    min_turning_radius: float = 1.0  # Minimum turning radius (m)


# ========================================================================================


# Data classes for trajectory representation
@dataclass
class BlendingConstraints:
    """Continuity constraints for trajectory blending"""

    position: np.ndarray  # Position at join point
    first_derivative: np.ndarray  # First derivative at join point
    second_derivative: np.ndarray  # Second derivative at join point
    velocity: float  # Velocity at join point
    time: float  # Time at join point


@dataclass
class PlanningPoint:
    """Represents a point along the trajectory with kinematic data."""

    position: np.ndarray  # x, y position
    curvature: float  # curvature at this point
    arc_length: float  # cumulative arc length to this point
    heading: float = 0.0  # heading in radians
    velocity: float = 0.0  # planned velocity
    time: float = 0.0  # time to reach this point
    segment_idx: int = 0  # index of spline segment
    parameter_t: float = 0.0  # parameter t within spline segment

    # Store velocities at different stages
    vel_of_iso_constraints: float = 0.0  # velocity from isolated constraints

    # Forward pass data
    forward_overlap_bound: float = float("inf")
    forward_trans_bounds: str = ""
    forward_rot_bounds_prime: str = ""
    forward_rot_bounds: str = ""
    forward_valid_bounds: str = ""

    # Backward pass data
    backward_overlap_bound: float = float("inf")
    backward_trans_bounds: str = ""
    backward_rot_bounds_prime: str = ""
    backward_rot_bounds: str = ""
    backward_valid_bounds: str = ""


@dataclass
class TrajectoryInfo:
    """Complete trajectory information"""

    waypoints: List[np.ndarray]  # Waypoints defining the path
    tangent_factors: List[float]  # Tangent factors for each waypoint
    spline_points: List[np.ndarray]  # Points along the spline
    planning_points: List[PlanningPoint]  # Points with velocity & curvature info
    velocity_profile: List[PlanningPoint]  # Complete velocity profile
    total_time: float  # Total traversal time

    # Add spline data needed for blending
    spline_segments: List  # Contains control points
    point_params: List[
        Tuple[int, float, float]
    ]  # (segment_index, local_parameter, heading) for each point
