import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import time
from shapely.geometry import Point, Polygon
from amr_trajectory_planner.core_modules.velocity_profile import VelocityProfileGenerator
from amr_trajectory_planner.core_modules.distance_map import DistanceMapGenerator
from amr_trajectory_planner.core_modules.spline_generation import BezierSplineGenerator, EquidistantPointGenerator
from rclpy.node import Node
from amr_trajectory_planner.config_types import BlendingConstraints, PlanningPoint, TrajectoryInfo


@dataclass
class RPROPState:
    """RPROP optimization state"""

    delta: float = 0.0  # Current step size
    prev_gradient_sign: float = 0.0  # Previous gradient sign (+1, -1, 0)
    prev_value: float = 0.0  # Previous parameter value

    # Constants for RPROP
    delta_0: float = 0.3  # Initial step size
    delta_min: float = 1e-4  # Minimum step size
    delta_max: float = 50.0  # Maximum step size
    eta_plus: float = 1.2  # Increase factor
    eta_minus: float = 0.5  # Decrease factor


@dataclass
class SplineConfig:
    """Configuration for spline generation and optimization"""

    is_first_window: bool = True
    is_last_window: bool = False
    blend_data: Optional[BlendingConstraints] = None


@dataclass
class Parameter:
    """Parameter to optimize"""

    type: str  # "GRADIENT", "PERPENDICULAR", or "TANGENT"
    waypoint_index: int  # Which waypoint
    rprop_state: RPROPState  # RPROP optimization state


class TrajectoryOptimizer:
    def __init__(
        self,
        node: Node,
        left_boundary: np.ndarray,
        right_boundary: np.ndarray,
    ):
        """Initialize trajectory optimizer with configuration"""
        self.node = node
        self.config = self.node.trajectory_optimization_config

        self.dist_map_generator = DistanceMapGenerator(self.node)
        (
            self.distance_map,
            self.X,
            self.Y,
            self.extended_left,
            self.extended_right,
        ) = self.dist_map_generator.create_distance_map(left_boundary, right_boundary)

        # Create path polygon for collision. 
        # TODO: Check if this can be removed by using lanelet2 directly from self.node
        self.path_polygon = Polygon(
            np.vstack(
                [
                    self.extended_left[0],
                    self.extended_right,
                    np.flip(self.extended_left[1:], axis=0),
                    self.extended_left[0],
                ]
            )
        )

        # Initialize velocity profile generator
        self._profile_generator = VelocityProfileGenerator(self.node)

        # Window-specific configuration
        # TODO: Check if this can be replaced by having a central config or params in main node.
        self.spline_config = SplineConfig()

        self.evaluating_initial_trajectory = True  # Skip collision checking for initial trajectory

        # Define Sobel kernels for gradient computation
        self.sobel_x = np.array(
            [
                [-1, -2, 0, 2, 1],
                [-4, -8, 0, 8, 4],
                [-6, -12, 0, 12, 6],
                [-4, -8, 0, 8, 4],
                [-1, -2, 0, 2, 1],
            ]
        )

        self.sobel_y = np.array(
            [
                [-1, -4, -6, -4, -1],
                [-2, -8, -12, -8, -2],
                [0, 0, 0, 0, 0],
                [2, 8, 12, 8, 2],
                [1, 4, 6, 4, 1],
            ]
        )

        # Initialize stats dictionary
        self.stats = {
            "iterations": 0,
            "trajectory_evaluations": 0,
            "collisions": 0,
            "improvements": 0,
            "optimization_time": 0,
            "no_of_for_loops": 0,
        }
        self.rprop_states = {}

    def reset_rprop_states(self):
        """Reset all RPROP states for a new optimization window"""
        self.rprop_states.clear()
        self.node.get_logger().debug("RPROP optimization states reset")

    def get_rprop_state(self, param_type: str, waypoint_idx: int) -> RPROPState:
        key = (param_type, waypoint_idx)
        if key not in self.rprop_states:
            self.rprop_states[key] = RPROPState(
                delta_0=self.config.rprop.initial_step_size,
                delta_min=self.config.rprop.minimum_step_size,
                delta_max=self.config.rprop.maximum_step_size,
                eta_plus=self.config.rprop.increase_factor,
                eta_minus=self.config.rprop.decrease_factor,
            )
            self.node.get_logger().debug(
                f"Created new RPROP state for {param_type} at waypoint {waypoint_idx}"
            )
        return self.rprop_states[key]

    def update_spline_config(
        self, is_first_window: bool, is_last_window: bool, blend_data: Optional[Dict] = None
    ):
        """Update spline configuration for current window"""
        self.spline_config = SplineConfig(
            is_first_window=is_first_window, is_last_window=is_last_window, blend_data=blend_data
        )
        self.node.get_logger().info(
            f"Updated spline config: first_window={is_first_window}, last_window={is_last_window}, "
            f"blend_data={'provided' if blend_data is not None else 'None'}"
        )

    def update_boundaries(self, left_boundary: np.ndarray, right_boundary: np.ndarray):
        """Update boundaries and regenerate distance map"""
        self.node.get_logger().info(
            f"Updating boundaries: left={len(left_boundary)} points, right={len(right_boundary)} points"
        )
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary

        # Create distance map
        start_time = time.time()
        (
            self.distance_map,
            self.X,
            self.Y,
            self.extended_left,
            self.extended_right,
        ) = self.dist_map_generator.create_distance_map(left_boundary, right_boundary)
        
        # Create path polygon for collision checking
        self.path_polygon = Polygon(
            np.vstack([self.extended_left, np.flip(self.extended_right, axis=0)])
        )
        
        self.node.get_logger().info(
            f"Distance map updated in {time.time() - start_time:.3f}s: "
            f"shape={self.distance_map.shape}, boundary points extended to "
            f"{len(self.extended_left)} left, {len(self.extended_right)} right"
        )

    def evaluate_trajectory(
        self,
        waypoints: List[np.ndarray],
        tangent_factors: List[float],
        distance_map: np.ndarray,
        X: np.ndarray,
        Y: np.ndarray,
    ) -> TrajectoryInfo:
        """
        Evaluate trajectory defined by waypoints and tangent factors.
        Returns TrajectoryInfo containing all trajectory data.
        """
        try:
            self.stats["trajectory_evaluations"] += 1
            self.node.get_logger().debug(f"Evaluating trajectory with {len(waypoints)} waypoints")

            # Generate spline - Pass blend data for non-first windows
            spline_gen = BezierSplineGenerator(
                self.node,
                waypoints,
                tangent_factors=tangent_factors,
                blend_constraints=None
                if self.spline_config.is_first_window
                else self.spline_config.blend_data,
            )
            point_gen = EquidistantPointGenerator(
                self.node, spline_gen, method=self.config.spline_points_method
            )
            points, parameters, arc_lengths = point_gen.generate_points(
                num_points=self.config.num_points
            )
            self.node.get_logger().debug(f"Total Arc Length: {arc_lengths[-1]:.3f}m")

            if self.evaluating_initial_trajectory:
                self.evaluating_initial_trajectory = False
            else:
                # Check collisions
                if self.dist_map_generator.check_path_collision(
                    points,
                    distance_map,
                    X,
                    Y,
                    self.path_polygon,
                    safety_margin=self.config.collision_checking.safety_margin,
                    check_stride=self.config.collision_checking.collision_check_interval,
                ):
                    self.node.get_logger().warn("Collision detected in trajectory")
                    self.stats["collisions"] += 1
                    return TrajectoryInfo(
                        waypoints=waypoints,
                        tangent_factors=tangent_factors,
                        spline_points=points,
                        planning_points=[],
                        velocity_profile=[],
                        total_time=float("inf"),
                        spline_segments=spline_gen.segments,
                        point_params=parameters,
                    )
                self.node.get_logger().debug("No collision detected in trajectory")

            # Generate planning points with curvature info
            planning_points = []
            for point, param, arc_length in zip(points, parameters, arc_lengths):
                segment_idx, local_t, heading = param
                deriv1 = spline_gen.get_derivative_at_parameter(segment_idx, local_t, order=1)
                deriv2 = spline_gen.get_derivative_at_parameter(segment_idx, local_t, order=2)

                denominator = (deriv1[0] ** 2 + deriv1[1] ** 2) ** (3 / 2)
                curvature = (
                    (deriv1[0] * deriv2[1] - deriv1[1] * deriv2[0]) / denominator
                    if denominator > 1e-10
                    else 0.0
                )

                planning_points.append(
                    PlanningPoint(
                        position=point,
                        curvature=curvature,
                        arc_length=arc_length,
                        heading=heading,
                        segment_idx=segment_idx,
                        parameter_t=local_t,
                    )
                )

            # Generate velocity profile
            if self.spline_config.is_first_window:
                velocity_profile = self._profile_generator.generate_velocity_profile(
                    planning_points, start_velocity=None, end_velocity=None, start_time=None
                )
            else:
                velocity_profile = self._profile_generator.generate_velocity_profile(
                    planning_points,
                    startVelocity=self.spline_config.blend_data.velocity,
                    end_velocity=0.0 if self.spline_config.is_last_window else None,
                    start_time=self.spline_config.blend_data.time,
                )
            total_time = velocity_profile[-1].time if velocity_profile else float("inf")
            self.node.get_logger().debug(f"Generated trajectory with total time: {total_time:.3f}s")

            return TrajectoryInfo(
                waypoints=waypoints,
                tangent_factors=tangent_factors,
                spline_points=points,
                planning_points=planning_points,
                velocity_profile=velocity_profile,
                total_time=total_time,
                spline_segments=spline_gen.segments,  # For blending
                point_params=parameters,  # For blending
            )

        except Exception as e:
            self.node.get_logger().error(f"Error in trajectory evaluation: {e}")
            return TrajectoryInfo(
                waypoints=waypoints,
                tangent_factors=tangent_factors,
                spline_points=[],
                planning_points=[],
                velocity_profile=[],
                total_time=float("inf"),
                spline_segments=[],
                point_params=[],
            )

    def optimize_trajectory(
        self, initial_trajectory: TrajectoryInfo
    ) -> Tuple[TrajectoryInfo, Dict]:
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
        self.node.get_logger().info("Starting trajectory optimization")

        start_time = time.time()

        # Initialize current state from initial trajectory
        best_trajectory = initial_trajectory
        self.node.get_logger().debug(f"Initial trajectory time: {best_trajectory.total_time:.3f}s")

        # Initialize parameters for each inner waypoint
        parameters = []
        for i in range(1, len(initial_trajectory.waypoints) - 1):
            parameters.extend(
                [
                    Parameter(
                        type="GRADIENT",
                        waypoint_index=i,
                        rprop_state=self.get_rprop_state("GRADIENT", i),
                    ),
                    Parameter(
                        type="PERPENDICULAR",
                        waypoint_index=i,
                        rprop_state=self.get_rprop_state("PERPENDICULAR", i),
                    ),
                    Parameter(
                        type="TANGENT",
                        waypoint_index=i,
                        rprop_state=self.get_rprop_state("TANGENT", i),
                    ),
                ]
            )
        
        self.node.get_logger().debug(f"Optimizing {len(parameters)} parameters")

        # Main optimization loop with time limit from config
        time_limit = self.config.optimization_time_limit
        self.node.get_logger().info(f"Optimization time limit: {time_limit:.1f}s")
        
        while (time.time() - start_time) < time_limit:
            self.stats["iterations"] += 1
            made_improvement = False

            # Track time remaining
            time_remaining = time_limit - (time.time() - start_time)
            if time_remaining <= 0:
                self.node.get_logger().debug("Time limit reached, stopping optimization")
                break

            # Optimize each parameter
            for param in parameters:
                self.stats["no_of_for_loops"] += 1
                current_trajectory = best_trajectory
                
                # Skip if we're out of time
                if (time.time() - start_time) >= time_limit:
                    break

                if param.type == "TANGENT":
                    self.node.get_logger().debug(f"Optimizing tangent at waypoint {param.waypoint_index}")
                    new_trajectory = self.optimize_tangent(param, current_trajectory)
                elif param.type == "GRADIENT":
                    self.node.get_logger().debug(f"Optimizing gradient direction at waypoint {param.waypoint_index}")
                    new_trajectory = self.optimize_gradient_direction(
                        param, current_trajectory, self.distance_map, self.X, self.Y
                    )
                elif param.type == "PERPENDICULAR":
                    self.node.get_logger().debug(f"Optimizing perpendicular direction at waypoint {param.waypoint_index}")
                    new_trajectory = self.optimize_perpendicular_direction(
                        param, current_trajectory, self.distance_map, self.X, self.Y
                    )

                if new_trajectory.total_time < best_trajectory.total_time:
                    improvement = best_trajectory.total_time - new_trajectory.total_time
                    best_trajectory = new_trajectory
                    made_improvement = True
                    self.stats["improvements"] += 1
                    self.node.get_logger().info(
                        f"Trajectory improved by {param.type} optimization at waypoint {param.waypoint_index}: "
                        f"time reduced by {improvement:.3f}s to {new_trajectory.total_time:.3f}s"
                    )

        # Update statistics
        self.stats["optimization_time"] = time.time() - start_time
        
        # Log optimization results
        self.node.get_logger().info(
            f"Optimization completed in {self.stats['optimization_time']:.3f}s with "
            f"{self.stats['iterations']} iterations, {self.stats['improvements']} improvements"
        )

        return best_trajectory, self.stats

    def optimize_gradient_direction(
        self,
        param: Parameter,
        trajectory: TrajectoryInfo,
        dist_map: np.ndarray,
        grad_x: np.ndarray,
        grad_y: np.ndarray,
    ) -> TrajectoryInfo:
        """
        Optimize waypoint translation in gradient direction.
        Paper section 4.2.1: First parameter - translation in gradient direction
        """
        rprop = param.rprop_state
        if rprop.delta == 0.0:  # If not initialized
            rprop.delta = rprop.delta_0
            self.node.get_logger().debug(f"Initialized gradient optimization delta to {rprop.delta}")
        
        waypoint_idx = param.waypoint_index
        current_point = trajectory.waypoints[waypoint_idx]
        
        self.node.get_logger().debug(f"Optimizing gradient direction for waypoint {waypoint_idx}")

        # Get gradient info
        Sx, Sy = self.apply_sobel_operator(dist_map, current_point)
        alpha = self.calculate_gradient_angle(Sx, Sy)
        if np.isnan(alpha):
            self.node.get_logger().debug("Skipping - gradient angle is undefined")
            return trajectory

        # Movement vector in gradient direction
        movement_x = np.cos(alpha)
        movement_y = np.sin(alpha)
        self.node.get_logger().debug(
            f"Movement direction: [{movement_x:.4f}, {movement_y:.4f}], angle: {alpha:.4f} rad"
        )

        # Try RPROP step
        new_waypoints = trajectory.waypoints.copy()
        new_point = current_point + rprop.delta * np.array([movement_x, movement_y])
        self.node.get_logger().debug(
            f"Trying new point: [{new_point[0]:.4f}, {new_point[1]:.4f}], delta: {rprop.delta:.4f}"
        )

        if not self.path_polygon.contains(Point(new_point)):
            self.node.get_logger().debug("New point outside path boundary, decreasing delta")
            rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
            return trajectory

        new_waypoints[waypoint_idx] = new_point
        new_trajectory = self.evaluate_trajectory(
            new_waypoints, trajectory.tangent_factors, dist_map, self.X, self.Y
        )

        # Compute improvement
        improvement = trajectory.total_time - new_trajectory.total_time
        current_sign = np.sign(improvement)
        self.node.get_logger().debug(f"Improvement: {improvement:.4f}s, sign: {current_sign}")

        # Update RPROP state
        if rprop.prev_gradient_sign != 0:  # Not first iteration
            sign_correlation = current_sign * rprop.prev_gradient_sign
            self.node.get_logger().debug(f"Sign correlation: {sign_correlation}")

            if sign_correlation > 0:
                # Same direction - increase step size
                rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
                self.node.get_logger().debug(f"Increasing delta to {rprop.delta:.4f}")
            elif sign_correlation < 0:
                # Direction changed - decrease step size
                rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
                rprop.prev_gradient_sign = current_sign
                self.node.get_logger().debug(
                    f"Direction changed, decreasing delta to {rprop.delta:.4f} and returning current trajectory"
                )
                return trajectory  # Revert and return current trajectory

        # Update previous gradient sign
        rprop.prev_gradient_sign = current_sign
        
        # Return better trajectory if found
        if improvement > 0:
            self.node.get_logger().debug(f"Improvement found, returning new trajectory")
            return new_trajectory
        else:
            self.node.get_logger().debug(f"No improvement, returning original trajectory")
            return trajectory

    def optimize_perpendicular_direction(
        self,
        param: Parameter,
        trajectory: TrajectoryInfo,
        dist_map: np.ndarray,
        grad_x: np.ndarray,
        grad_y: np.ndarray,
    ) -> TrajectoryInfo:
        """
        Optimize waypoint translation perpendicular to gradient.
        Paper section 4.2.1: Second parameter - translation parallel to obstacle
        """
        rprop = param.rprop_state
        if rprop.delta == 0.0:  # If not initialized
            rprop.delta = rprop.delta_0
            self.node.get_logger().debug(f"Initialized perpendicular optimization delta to {rprop.delta}")
        
        waypoint_idx = param.waypoint_index
        current_point = trajectory.waypoints[waypoint_idx]
        next_point = trajectory.waypoints[waypoint_idx + 1]
        
        self.node.get_logger().debug(f"Optimizing perpendicular direction for waypoint {waypoint_idx}")

        # Get gradient info
        Sx, Sy = self.apply_sobel_operator(dist_map, current_point)
        alpha = self.calculate_gradient_angle(Sx, Sy)

        if np.isnan(alpha):
            self.node.get_logger().debug("Skipping - gradient angle is undefined")
            return trajectory

        # Movement vector orthogonal to gradient (α + π/2)
        movement_x = np.cos(alpha + np.pi / 2)
        movement_y = np.sin(alpha + np.pi / 2)
        self.node.get_logger().debug(
            f"Movement direction: [{movement_x:.4f}, {movement_y:.4f}], "
            f"perpendicular to angle: {alpha:.4f} rad"
        )

        # Try RPROP step
        new_waypoints = trajectory.waypoints.copy()
        new_point = current_point + (rprop.delta) * np.array([movement_x, movement_y])
        self.node.get_logger().debug(
            f"Trying new point: [{new_point[0]:.4f}, {new_point[1]:.4f}], delta: {rprop.delta:.4f}"
        )

        # Validate new position
        if not self.path_polygon.contains(Point(new_point)):
            self.node.get_logger().debug("New point outside path boundary, decreasing delta")
            rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
            return trajectory

        new_waypoints[waypoint_idx] = new_point
        new_trajectory = self.evaluate_trajectory(
            new_waypoints, trajectory.tangent_factors, dist_map, self.X, self.Y
        )

        # Compute improvement
        improvement = trajectory.total_time - new_trajectory.total_time
        current_sign = np.sign(improvement)
        self.node.get_logger().debug(f"Improvement: {improvement:.4f}s, sign: {current_sign}")

        # Update RPROP state
        if rprop.prev_gradient_sign != 0:  # Not first iteration
            sign_correlation = current_sign * rprop.prev_gradient_sign
            self.node.get_logger().debug(f"Sign correlation: {sign_correlation}")

            if sign_correlation > 0:
                # Same direction - increase step size
                rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
                self.node.get_logger().debug(f"Increasing delta to {rprop.delta:.4f}")
            elif sign_correlation < 0:
                # Direction changed - decrease step size
                rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
                rprop.prev_gradient_sign = current_sign
                self.node.get_logger().debug(
                    f"Direction changed, decreasing delta to {rprop.delta:.4f} and returning current trajectory"
                )
                return trajectory  # Revert and return current trajectory

        # Update previous gradient sign
        rprop.prev_gradient_sign = current_sign
        
        # Return better trajectory if found
        if improvement > 0:
            self.node.get_logger().debug(f"Improvement found, returning new trajectory")
            return new_trajectory
        else:
            self.node.get_logger().debug(f"No improvement, returning original trajectory")
            return trajectory

    def optimize_tangent(self, param: Parameter, trajectory: TrajectoryInfo) -> TrajectoryInfo:
        """
        Optimize tangent elongation factor.
        Paper section 4.2.1: Third parameter - tangent magnitude
        """
        rprop = param.rprop_state
        waypoint_idx = param.waypoint_index
        
        self.node.get_logger().debug(
            f"Starting tangent optimization for waypoint {waypoint_idx}, "
            f"initial delta: {rprop.delta:.4f}, prev_sign: {rprop.prev_gradient_sign}"
        )
        
        if rprop.delta == 0.0:  # If not initialized
            rprop.delta = rprop.delta_0
            self.node.get_logger().debug(f"Initialized delta to {rprop.delta}")

        # Try modified elongation factor
        new_tangent_factors = trajectory.tangent_factors.copy()
        elongation = 1.0 + rprop.delta  # Always positive scalar
        new_tangent_factors[waypoint_idx] *= elongation
        self.node.get_logger().debug(f"Applying elongation factor: {elongation:.4f}")

        # Evaluate new trajectory
        new_trajectory = self.evaluate_trajectory(
            trajectory.waypoints, new_tangent_factors, self.distance_map, self.X, self.Y
        )
        
        if not new_trajectory.planning_points:
            self.node.get_logger().debug("No valid planning points in new trajectory")
            return trajectory
            
        self.node.get_logger().debug(
            f"Trajectory stats - Time: {new_trajectory.total_time:.3f}s, "
            f"Points: {len(new_trajectory.planning_points)}, "
            f"Velocity range: {new_trajectory.planning_points[0].velocity:.2f} to "
            f"{new_trajectory.planning_points[-1].velocity:.2f} m/s"
        )

        # Compute improvement
        improvement = trajectory.total_time - new_trajectory.total_time
        current_sign = np.sign(improvement)
        self.node.get_logger().debug(f"Improvement: {improvement:.4f}s, sign: {current_sign}")

        # Update RPROP state
        if rprop.prev_gradient_sign != 0:  # Not first iteration
            sign_correlation = current_sign * rprop.prev_gradient_sign
            self.node.get_logger().debug(f"Sign correlation: {sign_correlation}")

            if sign_correlation > 0:
                # Same direction - increase step size
                rprop.delta = min(rprop.delta * rprop.eta_plus, rprop.delta_max)
                self.node.get_logger().debug(f"Increasing delta to {rprop.delta:.4f}")
            elif sign_correlation < 0:
                # Direction changed - decrease step size
                rprop.delta = max(rprop.delta * rprop.eta_minus, rprop.delta_min)
                rprop.prev_gradient_sign = current_sign  # Update sign before returning
                self.node.get_logger().debug(
                    f"Direction changed, decreasing delta to {rprop.delta:.4f} and returning current trajectory"
                )
                return trajectory  # Revert and return current trajectory

        # Update previous gradient sign
        rprop.prev_gradient_sign = current_sign
        self.node.get_logger().debug(f"Updated prev_gradient_sign to {rprop.prev_gradient_sign}")
        
        # Return better trajectory if found
        if improvement > 0:
            self.node.get_logger().debug(f"Improvement found, returning new trajectory")
            return new_trajectory
        else:
            self.node.get_logger().debug(f"No improvement, returning original trajectory")
            return trajectory

    def apply_sobel_operator(self, dist_map: np.ndarray, point: np.ndarray) -> Tuple[float, float]:
        """
        Apply 5x5 Sobel operator from paper's Appendix C.
        Returns (Sx, Sy) for gradient computation.
        """
        # Find nearest grid cell indices
        i = np.argmin(np.abs(self.Y[:, 0] - point[1]))
        j = np.argmin(np.abs(self.X[0, :] - point[0]))

        # Check if we can get 5x5 window
        if not (2 <= i < dist_map.shape[0] - 2 and 2 <= j < dist_map.shape[1] - 2):
            self.node.get_logger().debug(
                f"Point [{point[0]:.4f}, {point[1]:.4f}] too close to map boundary for Sobel operator"
            )
            return np.nan, np.nan

        # Get 5x5 window
        window = dist_map[i - 2 : i + 3, j - 2 : j + 3]
        valid_mask = ~np.isnan(window)

        # If center point is NaN, gradient undefined
        if np.isnan(window[2, 2]):
            self.node.get_logger().debug(f"Center point is NaN, gradient undefined")
            return np.nan, np.nan

        # Apply kernels only to valid regions
        Sx = np.sum(window[valid_mask] * self.sobel_x[valid_mask])
        Sy = np.sum(window[valid_mask] * self.sobel_y[valid_mask])
        
        self.node.get_logger().debug(f"Sobel operator result: Sx={Sx:.4f}, Sy={Sy:.4f}")
        return Sx, Sy

    def calculate_gradient_angle(self, Sx: float, Sy: float) -> float:
        """
        Calculate gradient angle α as per paper:
        α = arctan(Sy/Sx)     if Sy ≥ 0
        α = arctan(Sy/Sx) + π otherwise
        """
        if np.isnan(Sx) or np.isnan(Sy):
            self.node.get_logger().debug("Cannot calculate gradient angle - Sx or Sy is NaN")
            return np.nan

        if abs(Sx) < 1e-10:
            angle = np.pi / 2 if Sy >= 0 else -np.pi / 2
            self.node.get_logger().debug(f"Sx near zero, using vertical angle: {angle:.4f} rad")
            return angle

        alpha = np.arctan(Sy / Sx)
        if Sy < 0:
            alpha += np.pi
        
        self.node.get_logger().debug(f"Calculated gradient angle: {alpha:.4f} rad")
        return alpha

    def try_waypoint_deletion(self, trajectory: TrajectoryInfo) -> TrajectoryInfo:
        """
        Try removing waypoints based on geometric considerations:
        1. Distance ratio: How much path length changes if point removed
        2. Turn angle: How sharp the turn is at the point
        """
        self.node.get_logger().info(f"Attempting waypoint deletion from trajectory with {len(trajectory.waypoints)} waypoints")

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
            self.node.get_logger().debug("Not enough waypoints for deletion, need at least 3")
            return trajectory

        # Calculate metrics for each inner waypoint
        deletion_candidates = []
        for i in range(1, len(trajectory.waypoints) - 1):
            prev_point = trajectory.waypoints[i - 1]
            curr_point = trajectory.waypoints[i]
            next_point = trajectory.waypoints[i + 1]

            # Calculate metrics
            curr_path_length = calculate_distance(prev_point, curr_point) + calculate_distance(
                curr_point, next_point
            )
            direct_path_length = calculate_distance(prev_point, next_point)
            turn_angle = calculate_turn_angle(prev_point, curr_point, next_point)

            deletion_candidates.append(
                {
                    "index": i,
                    "distance_ratio": curr_path_length / direct_path_length,
                    "turn_angle": turn_angle,
                }
            )
            
            self.node.get_logger().debug(
                f"Waypoint {i} metrics: distance_ratio={curr_path_length / direct_path_length:.4f}, "
                f"turn_angle={turn_angle:.4f} rad"
            )

        # Sort candidates by combination of metrics
        # Higher priority for:
        # - distance_ratio closer to 1 (removing point doesn't lengthen path much)
        # - turn_angle closer to π (point nearly collinear with neighbors)
        deletion_candidates.sort(
            key=lambda x: (
                abs(x["distance_ratio"] - 1.0)
                + abs(  # Smaller difference from 1.0
                    x["turn_angle"] - np.pi
                )  # Smaller difference from π
            )
        )
        
        self.node.get_logger().debug(f"Sorted deletion candidates: {[c['index'] for c in deletion_candidates]}")

        # Try removing points in priority order
        for candidate in deletion_candidates:
            idx = candidate["index"]
            self.node.get_logger().info(
                f"Trying to remove waypoint {idx} with distance_ratio={candidate['distance_ratio']:.4f}, "
                f"turn_angle={candidate['turn_angle']:.4f} rad"
            )

            # Create new waypoints and tangent factors without this point
            new_waypoints = trajectory.waypoints[:idx] + trajectory.waypoints[idx + 1 :]
            new_tangent_factors = (
                trajectory.tangent_factors[:idx] + trajectory.tangent_factors[idx + 1 :]
            )

            # Evaluate new trajectory
            new_trajectory = self.evaluate_trajectory(
                new_waypoints, new_tangent_factors, self.distance_map, self.X, self.Y
            )

            # Check if improvement found
            if new_trajectory.total_time < best_trajectory.total_time:
                improvement = best_trajectory.total_time - new_trajectory.total_time
                self.node.get_logger().info(
                    f"Removing waypoint {idx} improves trajectory time by {improvement:.3f}s"
                )
                best_trajectory = new_trajectory
                break  # Conservative: only remove one point at a time
            else:
                self.node.get_logger().debug(f"Removing waypoint {idx} does not improve trajectory")

        if best_trajectory is not trajectory:
            self.node.get_logger().info(
                f"Waypoint deletion successful, reduced from {len(trajectory.waypoints)} to "
                f"{len(best_trajectory.waypoints)} waypoints"
            )
        else:
            self.node.get_logger().info("No waypoints removed - original trajectory is optimal")

        return best_trajectory
