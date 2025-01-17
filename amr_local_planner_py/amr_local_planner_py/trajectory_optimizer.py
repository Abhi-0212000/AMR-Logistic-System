from hmac import new
import re
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import time
from shapely.geometry import Point, Polygon
from amr_local_planner_py.distance_map import DistanceMapGenerator, DistanceMapConfig
from amr_local_planner_py.spline_generation import BezierSplineGenerator, EquidistantPointGenerator, SplineSegment
from amr_local_planner_py.velocity_profile import VelocityProfileGenerator, PlanningPoint, ROBOT_CONSTRAINTS
from amr_local_planner_py.common_types import BlendingConstraints, PlanningPoint, TrajectoryInfo

@dataclass
class RPROPState:
    """RPROP optimization state"""
    delta: float = 0.0             # Current step size
    prev_gradient_sign: float = 0.0 # Previous gradient sign (+1, -1, 0)
    prev_value: float = 0.0        # Previous parameter value
    
    # Constants for RPROP
    delta_0: float = 0.3     # Initial step size
    delta_min: float = 1e-4  # Minimum step size
    delta_max: float = 50.0  # Maximum step size
    eta_plus: float = 1.2    # Increase factor 
    eta_minus: float = 0.5   # Decrease factor
    
@dataclass
class SplineConfig:
    """Configuration for spline generation and optimization"""
    is_first_window: bool = True
    is_last_window: bool = False
    blend_data: Optional[BlendingConstraints] = None

@dataclass
class Parameter:
    """Parameter to optimize"""
    type: str                # "GRADIENT", "PERPENDICULAR", or "TANGENT"
    waypoint_index: int      # Which waypoint
    rprop_state: RPROPState  # RPROP optimization state

# @dataclass
# class TrajectoryInfo:
#     """Complete trajectory information"""
#     waypoints: List[np.ndarray]         # Waypoints defining the path
#     tangent_factors: List[float]        # Tangent factors for each waypoint
#     spline_points: List[np.ndarray]     # Points along the spline
#     planning_points: List[PlanningPoint] # Points with velocity & curvature info
#     velocity_profile: List[PlanningPoint] # Complete velocity profile
#     total_time: float                   # Total traversal time
    
#     # Add spline data needed for blending
#     spline_segments: List[SplineSegment]  # Contains control points
#     point_params: List[Tuple[int, float]]  # (segment_idx, parameter_t) for each point

@dataclass 
class TrajectoryOptimizerConfig:
    """Configuration for trajectory optimization"""
    num_points: int = 250           # Number of points for trajectory discretization
    safety_margin: float = 0.2      # Minimum distance from boundaries (meters)
    check_stride: int = 5           # Skip points for collision checking
    base_tangent_factor: float = 0.5  # Base tangent factor
    gradient_step: float = 1e-4     # Step for numerical gradient
    optimization_time_limit: float = 1.0  # Time limit for optimization in seconds
    
    # Distance map configuration
    distance_map_resolution: float = 0.2  # Grid resolution in meters
    distance_map_window_size: float = 3.0  # Default local window size in meters
    
    # Spline generation configuration
    spline_points_method: str = "simpson"  # Method for arc length calculation
    
    def __post_init__(self):
        """Validate configuration parameters"""
        if self.num_points < 10:
            raise ValueError("num_points must be at least 10")
        if self.safety_margin <= 0:
            raise ValueError("safety_margin must be positive")
        if self.check_stride < 1:
            raise ValueError("check_stride must be at least 1")
        if self.base_tangent_factor <= 0:
            raise ValueError("base_tangent_factor must be positive")
        if self.gradient_step <= 0:
            raise ValueError("gradient_step must be positive")
        if self.optimization_time_limit <= 0:
            raise ValueError("optimization_time_limit must be positive")
        if self.distance_map_resolution <= 0:
            raise ValueError("distance_map_resolution must be positive")
        if self.distance_map_window_size <= 0:
            raise ValueError("distance_map_window_size must be positive")
        if self.spline_points_method not in ["simpson", "linear"]:
            raise ValueError("spline_points_method must be 'simpson' or 'linear'")

class TrajectoryOptimizer:
    def __init__(self, left_boundary: np.ndarray, right_boundary: np.ndarray, 
                 config: Optional[TrajectoryOptimizerConfig] = None):
        """Initialize trajectory optimizer with configuration"""
        self.config = config if config else TrajectoryOptimizerConfig()
        
        # Create distance map with config parameters
        dist_map_config = DistanceMapConfig(
            resolution=self.config.distance_map_resolution,
            window_size=self.config.distance_map_window_size
        )
        self.dist_map_generator = DistanceMapGenerator(dist_map_config)
        self.distance_map, self.X, self.Y, self.extended_left, self.extended_right = self.dist_map_generator.create_distance_map(
            left_boundary, right_boundary)
        
        # Create path polygon for collision 
        # self.path_polygon = Polygon(np.vstack([self.extended_left, np.flip(self.extended_right, axis=0)]))
        self.path_polygon = Polygon(np.vstack([self.extended_left[0], self.extended_right, np.flip(self.extended_left[1:], axis=0), self.extended_left[0]]))
        
        # Initialize velocity profile generator
        self._profile_generator = VelocityProfileGenerator(ROBOT_CONSTRAINTS)
        
        # Window-specific configuration
        self.spline_config = SplineConfig()
        
        self.evaluating_initial_trajectory = True # We will use this flag to skip collision checking for initial trajectory
        
        # Define Sobel kernels for gradient computation
        self.sobel_x = np.array([
            [-1, -2,  0,  2,  1],
            [-4, -8,  0,  8,  4],
            [-6, -12, 0, 12, 6],
            [-4, -8,  0,  8,  4],
            [-1, -2,  0,  2,  1]
        ])
        
        self.sobel_y = np.array([
            [-1, -4, -6, -4, -1],
            [-2, -8, -12, -8, -2],
            [ 0,  0,  0,  0,  0],
            [ 2,  8, 12,  8,  2],
            [ 1,  4,  6,  4,  1]
        ])
        
        # Initialize stats dictionary
        self.stats = {
            'iterations': 0,
            'trajectory_evaluations': 0,
            'collisions': 0,
            'improvements': 0,
            'optimization_time': 0,
            'no_of_for_loops': 0,
        }
        self.rprop_states = {}  
        
    def reset_rprop_states(self):
        """Reset all RPROP states for a new optimization window"""
        self.rprop_states.clear()
    
    def get_rprop_state(self, param_type: str, waypoint_idx: int) -> RPROPState:
        key = (param_type, waypoint_idx)
        if key not in self.rprop_states:
            self.rprop_states[key] = RPROPState()
        return self.rprop_states[key]
    
    def update_spline_config(self, is_first_window: bool, 
                          is_last_window: bool,
                          blend_data: Optional[Dict] = None):
        """Update spline configuration for current window"""
        self.spline_config = SplineConfig(
            is_first_window=is_first_window,
            is_last_window=is_last_window,
            blend_data=blend_data
        )

    def update_boundaries(self, left_boundary: np.ndarray, 
                         right_boundary: np.ndarray):
        """Update boundaries and regenerate distance map"""
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary
        
        self.distance_map, self.X, self.Y, self.extended_left, self.extended_right = self.dist_map_generator.create_distance_map(
            left_boundary, right_boundary)
        
        # Create path polygon for collision checking
        self.path_polygon = Polygon(np.vstack([self.extended_left, np.flip(self.extended_right, axis=0)]))

    def evaluate_trajectory(self, waypoints: List[np.ndarray],
                            tangent_factors: List[float],
                            distance_map: np.ndarray,
                            X: np.ndarray, 
                            Y: np.ndarray) -> TrajectoryInfo:
        """
        Evaluate trajectory defined by waypoints and tangent factors.
        Returns TrajectoryInfo containing all trajectory data.
        """
        try:
            self.stats['trajectory_evaluations'] += 1
            
            # print(f"Tangent Factors: {tangent_factors}")
            # Generate spline
            # Generate spline - Pass blend data for non-first windows
            spline_gen = BezierSplineGenerator(
                waypoints, 
                tangent_factors=tangent_factors,
                blend_constraints=None if self.spline_config.is_first_window 
                                else self.spline_config.blend_data
            )
            point_gen = EquidistantPointGenerator(spline_gen, method=self.config.spline_points_method)
            points, parameters, arc_lengths = point_gen.generate_points(num_points=self.config.num_points)
            print(f"Total Arc Length: {arc_lengths[-1]}")
            
            if self.evaluating_initial_trajectory:
                self.evaluating_initial_trajectory = False
            else:
                # Check collisions
                if self.dist_map_generator.check_path_collision(
                    points, distance_map, X, Y,
                    self.path_polygon, 
                    safety_margin=self.config.safety_margin,
                    check_stride=self.config.check_stride
                ):
                    print("Collision detected")
                    self.stats['collisions'] += 1
                    return TrajectoryInfo(
                        waypoints=waypoints,
                        tangent_factors=tangent_factors,
                        spline_points=points,
                        planning_points=[],
                        velocity_profile=[],
                        total_time=float('inf'),
                        spline_segments=spline_gen.segments,
                        point_params=parameters
                    )
                print("No collision detected")
            
            # Generate planning points with curvature info
            planning_points = []
            for point, param, arc_length in zip(points, parameters, arc_lengths):
                segment_idx, local_t, heading = param
                deriv1 = spline_gen.get_derivative_at_parameter(segment_idx, local_t, order=1)
                deriv2 = spline_gen.get_derivative_at_parameter(segment_idx, local_t, order=2)
                
                denominator = (deriv1[0]**2 + deriv1[1]**2)**(3/2)
                curvature = (deriv1[0]*deriv2[1] - deriv1[1]*deriv2[0]) / denominator if denominator > 1e-10 else 0.0
                
                planning_points.append(PlanningPoint(
                    position=point,
                    curvature=curvature,
                    arc_length=arc_length,
                    heading=heading,
                    segment_idx=segment_idx,
                    parameter_t=local_t
                ))
            
            # Generate velocity profile
            if self.spline_config.is_first_window:
                velocity_profile = self._profile_generator.generate_velocity_profile(
                    planning_points,
                    start_velocity=None,
                    end_velocity=None,
                    start_time=None
                )
            else:
                velocity_profile = self._profile_generator.generate_velocity_profile(
                    planning_points,
                    start_velocity=self.spline_config.blend_data.velocity,
                    end_velocity=0.0 if self.spline_config.is_last_window else None,
                    start_time=self.spline_config.blend_data.time
                )
            total_time = velocity_profile[-1].time if velocity_profile else float('inf')
            
            return TrajectoryInfo(
                waypoints=waypoints,
                tangent_factors=tangent_factors,
                spline_points=points,
                planning_points=planning_points,
                velocity_profile=velocity_profile,
                total_time=total_time,
                spline_segments=spline_gen.segments, # For blending
                point_params=parameters  # For blending
            )
            
        except Exception as e:
            print(f"Error in trajectory evaluation: {e}")
            return TrajectoryInfo(
                waypoints=waypoints,
                tangent_factors=tangent_factors,
                spline_points=[],
                planning_points=[],
                velocity_profile=[],
                total_time=float('inf'),
                spline_segments=[],
                point_params=[]
            )

    def optimize_trajectory(self, initial_trajectory: TrajectoryInfo) -> Tuple[TrajectoryInfo, Dict]:
        """
        Main optimization function
        
        Args:
            initial_trajectory: Initial trajectory to optimize
            
        Returns:
            Tuple containing:
            - Optimized trajectory info
            - Optimization statistics
        """
        
        self.reset_rprop_states()
        
        start_time = time.time()
        
        # Initialize current state from initial trajectory
        # current_trajectory = initial_trajectory
        best_trajectory = initial_trajectory
        
        # Initialize parameters for each inner waypoint
        parameters = []
        for i in range(1, len(initial_trajectory.waypoints)-1):
            parameters.extend([
                Parameter(type="GRADIENT", waypoint_index=i, 
                         rprop_state=self.get_rprop_state("GRADIENT", i)),
                Parameter(type="PERPENDICULAR", waypoint_index=i, 
                         rprop_state=self.get_rprop_state("PERPENDICULAR", i)),
                Parameter(type="TANGENT", waypoint_index=i, 
                         rprop_state=self.get_rprop_state("TANGENT", i))
            ])
        
        # Main optimization loop with time limit from config
        time_left = time.time() - start_time
        while (time.time() - start_time) < self.config.optimization_time_limit:
            self.stats['iterations'] += 1
            made_improvement = False
            
            # Track time remaining
            time_remaining = self.config.optimization_time_limit - (time.time() - start_time)
            if time_remaining <= 0:
                break
            
            # Optimize each parameter
            for param in parameters:
                self.stats['no_of_for_loops'] += 1
                current_trajectory = best_trajectory
                # Skip if we're out of time
                if (time.time() - start_time) >= self.config.optimization_time_limit:
                    break
                
                if param.type == "TANGENT":
                    # print("Optimizing tangent")
                    new_trajectory = self.optimize_tangent(param, current_trajectory)
                    # new_trajectory = best_trajectory
                elif param.type == "GRADIENT":
                    new_trajectory = self.optimize_gradient_direction(
                        param, current_trajectory, self.distance_map, self.X, self.Y)
                #     # new_trajectory = best_trajectory
                    
                elif param.type == "PERPENDICULAR":
                    new_trajectory = self.optimize_perpendicular_direction(
                        param, current_trajectory, self.distance_map, self.X, self.Y)
                    # new_trajectory = best_trajectory
                # else:  # TANGENT
                #     new_trajectory = self.optimize_tangent(param, current_trajectory)
                # print(f"best_trajectory.total_time: {best_trajectory.total_time} & new_trajectory.total_time: {new_trajectory.total_time}")
                if new_trajectory.total_time < best_trajectory.total_time:
                    best_trajectory = new_trajectory
                    made_improvement = True
                    self.stats['improvements'] += 1
                    print(f"Trajectory improved because of {param.type} and waypoint idx {param.waypoint_index} optimization")
                # else:
                #     print("not improved")
            
            # Try waypoint deletion after parameter optimization
            # if len(current_trajectory.waypoints) > 3:
            #     new_trajectory = self.try_waypoint_deletion(current_trajectory)
            #     if new_trajectory.total_time < best_trajectory.total_time:
            #         best_trajectory = new_trajectory
            #         current_trajectory = new_trajectory
            #         self.stats['improvements'] += 1
                    
            #         # Update parameters for new waypoint configuration
            #         parameters = []
            #         for i in range(1, len(current_trajectory.waypoints)-1):
            #             parameters.extend([
            #                 Parameter(type="GRADIENT", waypoint_index=i, 
            #                          rprop_state=RPROPState()),
            #                 Parameter(type="PERPENDICULAR", waypoint_index=i, 
            #                          rprop_state=RPROPState()),
            #                 Parameter(type="TANGENT", waypoint_index=i, 
            #                          rprop_state=RPROPState())
            #             ])
            
            # if not made_improvement:
            #     break

        self.stats['optimization_time'] = time.time() - start_time
        return best_trajectory, self.stats

    def optimize_gradient_direction(self, param: Parameter, trajectory: TrajectoryInfo,
                                dist_map: np.ndarray, grad_x: np.ndarray,
                                grad_y: np.ndarray) -> TrajectoryInfo:
        """
        Optimize waypoint translation in gradient direction.
        Paper section 4.2.1: First parameter - translation in gradient direction
        """
        rprop = param.rprop_state
        if rprop.delta == 0.0:  # If not initialized
            rprop.delta = rprop.delta_0
        waypoint_idx = param.waypoint_index
        current_point = trajectory.waypoints[waypoint_idx]
        # best_trajectory = trajectory

        # Get gradient info
        Sx, Sy = self.apply_sobel_operator(dist_map, current_point)
        alpha = self.calculate_gradient_angle(Sx, Sy)
        if np.isnan(alpha):
            return trajectory

        # Movement vector in gradient direction
        movement_x = np.cos(alpha)
        movement_y = np.sin(alpha)

        # Try RPROP step
        new_waypoints = trajectory.waypoints.copy()
        new_point = current_point + rprop.delta * np.array([movement_x, movement_y])
        
        if not self.path_polygon.contains(Point(new_point)):
            rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
            return trajectory

        new_waypoints[waypoint_idx] = new_point
        new_trajectory = self.evaluate_trajectory(
            new_waypoints,
            trajectory.tangent_factors,
            dist_map, self.X, self.Y
        )

        # Compute improvement
        improvement = trajectory.total_time - new_trajectory.total_time
        current_sign = np.sign(improvement)

        # Update RPROP state
        if rprop.prev_gradient_sign != 0:  # Not first iteration
            sign_correlation = current_sign * rprop.prev_gradient_sign
            
            if sign_correlation > 0:
                # Same direction - increase step size
                rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
            elif sign_correlation < 0:
                # Direction changed - decrease step size
                rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
                rprop.prev_gradient_sign = current_sign
                return trajectory  # Revert and return current trajectory
        
        # Update previous gradient sign
        rprop.prev_gradient_sign = current_sign

        # Return better trajectory if found
        return new_trajectory if improvement > 0 else trajectory

    def optimize_perpendicular_direction(self, param: Parameter,
                                      trajectory: TrajectoryInfo,
                                      dist_map: np.ndarray,
                                      grad_x: np.ndarray,
                                      grad_y: np.ndarray) -> TrajectoryInfo:
        """
        Optimize waypoint translation perpendicular to gradient.
        Paper section 4.2.1: Second parameter - translation parallel to obstacle
        """
        rprop = param.rprop_state
        if rprop.delta == 0.0:  # If not initialized
            rprop.delta = rprop.delta_0
        waypoint_idx = param.waypoint_index
        current_point = trajectory.waypoints[waypoint_idx]
        next_point = trajectory.waypoints[waypoint_idx + 1]
        # best_trajectory = trajectory
        
        # Get gradient info
        Sx, Sy = self.apply_sobel_operator(dist_map, current_point)
        alpha = self.calculate_gradient_angle(Sx, Sy)

        if np.isnan(alpha):
            return trajectory

        # Movement vector orthogonal to gradient (α + π/2)
        movement_x = np.cos(alpha + np.pi/2)
        movement_y = np.sin(alpha + np.pi/2)

        # Ensure movement aligns with travel direction
        # travel_dir = next_point - current_point
        # if np.dot([movement_x, movement_y], travel_dir) < 0:
        #     movement_x = -movement_x
        #     movement_y = -movement_y

        # Try RPROP step
        new_waypoints = trajectory.waypoints.copy()
        new_point = current_point + (rprop.delta) * np.array([movement_x, movement_y])
    
        # Validate new position
        if not self.path_polygon.contains(Point(new_point)):
            rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
            return trajectory
        
        new_waypoints[waypoint_idx] = new_point
        new_trajectory = self.evaluate_trajectory(
            new_waypoints,
            trajectory.tangent_factors,
            dist_map, self.X, self.Y
        )
        
        # Compute improvement
        improvement = trajectory.total_time - new_trajectory.total_time
        current_sign = np.sign(improvement)

        # Update RPROP state
        if rprop.prev_gradient_sign != 0:  # Not first iteration
            sign_correlation = current_sign * rprop.prev_gradient_sign
            
            if sign_correlation > 0:
                # Same direction - increase step size
                rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
            elif sign_correlation < 0:
                # Direction changed - decrease step size
                rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
                rprop.prev_gradient_sign = current_sign
                return trajectory  # Revert and return current trajectory
        
        # Update previous gradient sign
        rprop.prev_gradient_sign = current_sign

        # Return better trajectory if found
        return new_trajectory if improvement > 0 else trajectory

    # def optimize_tangent(self, param: Parameter,
    #                     trajectory: TrajectoryInfo) -> TrajectoryInfo:
    #     """
    #     Optimize tangent elongation factor.
    #     Paper section 4.2.1: Third parameter - tangent magnitude
    #     """
    #     rprop = param.rprop_state
    #     if rprop.delta == 0.0:  # If not initialized
    #         rprop.delta = rprop.delta_0
    #     waypoint_idx = param.waypoint_index
    #     # best_trajectory = trajectory
        
    #     # Try modified elongation factor
    #     new_tangent_factors = trajectory.tangent_factors.copy()
    #     elongation = 1.0 + rprop.delta  # Always positive scalar
    #     new_tangent_factors[waypoint_idx] *= elongation
    #     print(f"New Tangent Factors: {new_tangent_factors} because of param type: {param.type} and waypoint idx: {param.waypoint_index}")
        
    #     # Evaluate new trajectory
    #     new_trajectory = self.evaluate_trajectory(
    #         trajectory.waypoints,
    #         new_tangent_factors,
    #         self.distance_map, self.X, self.Y
    #     )
        
    #     print(f"Trajectory stats during Tangent Optimization:\n"
    #                 f"      Tangent factors: {new_trajectory.tangent_factors},\n"
    #                 f"      Time range: {new_trajectory.planning_points[0].time} - {new_trajectory.planning_points[-1].time},\n"
    #                 f"      Planning Points Range: {new_trajectory.planning_points[0].position} - {new_trajectory.planning_points[-1].position},\n"
    #                 f"      Velocity Range: {new_trajectory.planning_points[0].velocity} - {new_trajectory.planning_points[-1].velocity}")
        
    #     # Compute improvement
    #     improvement = trajectory.total_time - new_trajectory.total_time
    #     print(f"Improvement: {improvement}")
    #     current_sign = np.sign(improvement)
        
    #     # Update RPROP state
    #     if rprop.prev_gradient_sign != 0:  # Not first iteration
    #         sign_correlation = current_sign * rprop.prev_gradient_sign
            
    #         if sign_correlation > 0:
    #             # Same direction - increase step size
    #             rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
    #         elif sign_correlation < 0:
    #             # Direction changed - decrease step size
    #             rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
    #             return trajectory  # Revert and return current trajectory
        
    #     # Update previous gradient sign
    #     rprop.prev_gradient_sign = current_sign
        
    #     # Return better trajectory if found
    #     return new_trajectory if improvement > 0 else trajectory
    
    def optimize_tangent(self, param: Parameter,
                        trajectory: TrajectoryInfo) -> TrajectoryInfo:
        """
        Optimize tangent elongation factor.
        Paper section 4.2.1: Third parameter - tangent magnitude
        """
        rprop = param.rprop_state
        print(f"\nStarting tangent optimization for waypoint {param.waypoint_index}")
        print(f"Initial RPROP state - delta: {rprop.delta}, prev_sign: {rprop.prev_gradient_sign}")
        if rprop.delta == 0.0:  # If not initialized
            rprop.delta = rprop.delta_0
        waypoint_idx = param.waypoint_index
        # best_trajectory = trajectory
        
        # Try modified elongation factor
        new_tangent_factors = trajectory.tangent_factors.copy()
        elongation = 1.0 + rprop.delta  # Always positive scalar
        new_tangent_factors[waypoint_idx] *= elongation
        # print(f"New Tangent Factors: {new_tangent_factors}")
        
        # Evaluate new trajectory
        new_trajectory = self.evaluate_trajectory(
            trajectory.waypoints,
            new_tangent_factors,
            self.distance_map, self.X, self.Y
        )
        print(f"Trajectory stats during Tangent Optimization:\n"
                    f"      Tangent factors: {new_trajectory.tangent_factors},\n"
                    f"      Time range: {new_trajectory.planning_points[0].time} - {new_trajectory.planning_points[-1].time},\n"
                    f"      Planning Points Range: {new_trajectory.planning_points[0].position} - {new_trajectory.planning_points[-1].position},\n"
                    f"      Velocity Range: {new_trajectory.planning_points[0].velocity} - {new_trajectory.planning_points[-1].velocity}")
        
        # Compute improvement
        improvement = trajectory.total_time - new_trajectory.total_time
        print(f"improvement: {improvement}")
        current_sign = np.sign(improvement)
        
        print(f"current_sign: {current_sign} & rprop.prev_gradient_sign: {rprop.prev_gradient_sign}")
        
        # Update RPROP state
        if rprop.prev_gradient_sign != 0:  # Not first iteration
            sign_correlation = current_sign * rprop.prev_gradient_sign
            
            if sign_correlation > 0:
                # Same direction - increase step size
                rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
            elif sign_correlation < 0:
                # Direction changed - decrease step size
                rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
                rprop.prev_gradient_sign = current_sign  # Update sign before returning
                return trajectory  # Revert and return current trajectory
        
        # Update previous gradient sign
        rprop.prev_gradient_sign = current_sign
        print(f"current_sign: {current_sign} & Updated rprop.prev_gradient_sign: {rprop.prev_gradient_sign}")
        # Return better trajectory if found
        print(f"total_time: {trajectory.total_time} & new_trajectory.total_time: {new_trajectory.total_time}")
        print(f"Final RPROP state - delta: {rprop.delta}, prev_sign: {rprop.prev_gradient_sign}\n")
        return new_trajectory if improvement > 0 else trajectory

    def apply_sobel_operator(self, dist_map: np.ndarray, point: np.ndarray) -> Tuple[float, float]:
        """
        Apply 5x5 Sobel operator from paper's Appendix C.
        Returns (Sx, Sy) for gradient computation.
        """
        # Find nearest grid cell indices
        i = np.argmin(np.abs(self.Y[:,0] - point[1]))
        j = np.argmin(np.abs(self.X[0,:] - point[0]))
        
        # Check if we can get 5x5 window
        if not (2 <= i < dist_map.shape[0]-2 and 2 <= j < dist_map.shape[1]-2):
            return np.nan, np.nan
            
        # Get 5x5 window
        window = dist_map[i-2:i+3, j-2:j+3]
        valid_mask = ~np.isnan(window)
        
        # If center point is NaN, gradient undefined
        if np.isnan(window[2,2]):
            return np.nan, np.nan
        
        # Apply kernels only to valid regions
        Sx = np.sum(window[valid_mask] * self.sobel_x[valid_mask])
        Sy = np.sum(window[valid_mask] * self.sobel_y[valid_mask])
        
        return Sx, Sy
        
    def calculate_gradient_angle(self, Sx: float, Sy: float) -> float:
        """
        Calculate gradient angle α as per paper:
        α = arctan(Sy/Sx)     if Sy ≥ 0
        α = arctan(Sy/Sx) + π otherwise
        """
        if np.isnan(Sx) or np.isnan(Sy):
            return np.nan
            
        if abs(Sx) < 1e-10:
            return np.pi/2 if Sy >= 0 else -np.pi/2
        
        alpha = np.arctan(Sy/Sx)
        if Sy < 0:
            alpha += np.pi
            
        return alpha

    def try_waypoint_deletion(self, trajectory: TrajectoryInfo) -> TrajectoryInfo:
        """
        Try removing waypoints based on geometric considerations:
        1. Distance ratio: How much path length changes if point removed
        2. Turn angle: How sharp the turn is at the point
        """
        def calculate_distance(p1: np.ndarray, p2: np.ndarray) -> float:
            """Calculate Euclidean distance between two points."""
            return np.sqrt(np.sum((p2 - p1) ** 2))
    
        def calculate_turn_angle(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
            """Calculate angle between segments p1-p2 and p2-p3."""
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Normalize vectors
            v1_norm = np.sqrt(np.sum(v1 ** 2))
            v2_norm = np.sqrt(np.sum(v2 ** 2))
            
            if v1_norm < 1e-10 or v2_norm < 1e-10:
                return 0.0
            
            # Calculate angle using dot product
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            cos_angle = min(max(cos_angle, -1.0), 1.0)  # Ensure within [-1, 1]
            
            return np.arccos(cos_angle)
        
        best_trajectory = trajectory
        
        if len(trajectory.waypoints) <= 3:  # Need at least start, one inner, end
            return trajectory
        
        # Calculate metrics for each inner waypoint
        deletion_candidates = []
        for i in range(1, len(trajectory.waypoints)-1):
            prev_point = trajectory.waypoints[i-1]
            curr_point = trajectory.waypoints[i]
            next_point = trajectory.waypoints[i+1]
            
            # Calculate metrics
            curr_path_length = (calculate_distance(prev_point, curr_point) + 
                              calculate_distance(curr_point, next_point))
            direct_path_length = calculate_distance(prev_point, next_point)
            
            deletion_candidates.append({
                'index': i,
                'distance_ratio': curr_path_length / direct_path_length,
                'turn_angle': calculate_turn_angle(prev_point, curr_point, next_point)
            })
        
        # Sort candidates by combination of metrics
        # Higher priority for:
        # - distance_ratio closer to 1 (removing point doesn't lengthen path much)
        # - turn_angle closer to π (point nearly collinear with neighbors)
        deletion_candidates.sort(key=lambda x: (
            abs(x['distance_ratio'] - 1.0) +  # Smaller difference from 1.0
            abs(x['turn_angle'] - np.pi)      # Smaller difference from π
        ))
        
        # Try removing points in priority order
        for candidate in deletion_candidates:
            idx = candidate['index']
            
            # Create new waypoints and tangent factors without this point
            new_waypoints = trajectory.waypoints[:idx] + trajectory.waypoints[idx+1:]
            new_tangent_factors = trajectory.tangent_factors[:idx] + trajectory.tangent_factors[idx+1:]
            
            # Evaluate new trajectory
            new_trajectory = self.evaluate_trajectory(
                new_waypoints,
                new_tangent_factors,
                self.distance_map, self.X, self.Y
            )
            
            # Keep if better
            if new_trajectory.total_time < best_trajectory.total_time:
                best_trajectory = new_trajectory
                break  # Conservative: only remove one point at a time
                
        return best_trajectory