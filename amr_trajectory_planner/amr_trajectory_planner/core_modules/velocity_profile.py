"""
Velocity profile generation with kinematic constraints.
This module provides classes and functions for generating velocity profiles
that respect kinematic constraints while maintaining smooth transitions.
"""

import time
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import matplotlib.pyplot as plt
from math import sqrt
from rclpy.node import Node
from amr_trajectory_planner.config_types import PlanningPoint, RobotConstraintsConfig


@dataclass
class VelocityRange:
    """Represents valid velocity ranges for constraints."""
    ranges: List[Tuple[float, float]]  # List of (min, max) tuples

    def validate_and_correct_ranges(self) -> None:
        """Validate and correct ranges where min > max."""
        for i, (min_v, max_v) in enumerate(self.ranges):
            if min_v > max_v:
                self.ranges[i] = (max_v, min_v)

    def intersect(self, other: 'VelocityRange') -> 'VelocityRange':
        """Intersect this velocity range with another."""
        self.validate_and_correct_ranges()
        other.validate_and_correct_ranges()
        
        result_ranges = []
        for r1 in self.ranges:
            for r2 in other.ranges:
                min_v = max(r1[0], r2[0])
                max_v = min(r1[1], r2[1])
                if min_v <= max_v:
                    result_ranges.append((min_v, max_v))
        return VelocityRange(ranges=result_ranges)

    def intersect_with_positive_reals(self) -> 'VelocityRange':
        """Intersect the ranges with non-negative real numbers."""
        self.validate_and_correct_ranges()
        positive_ranges = []
        for min_v, max_v in self.ranges:
            if max_v >= 0:
                positive_ranges.append((max(0, min_v), max_v))
        return VelocityRange(ranges=positive_ranges)


class VelocityProfileGenerator:
    """Generates velocity profiles respecting kinematic constraints."""
    
    # Class constants
    EPSILON = 1e-10  # Small value to prevent division by zero
    CURVATURE_EPSILON = 1e-6  # Threshold for considering curvature as zero
    DELTA_S_EPSILON = 1e-6  # Threshold for considering delta_s as zero
    DEFAULT_VELOCITY = 1.0  # Default velocity for remaining time calculation
    
    def __init__(self, node: Node):
        """
        Initialize with robot constraints from node configuration.
        
        Args:
            node: ROS2 node for accessing parameters and logging
        """
        self.node = node
        self.constraints = self._get_constraints_from_config()
        self._validate_constraints()

    def _get_constraints_from_config(self) -> Dict[str, float]:
        """Extract constraints from node configuration."""
        try:
            config = self.node.robot_constraints_config
            self.node.get_logger().debug("Using robot constraints from node configuration")
            
            return {
                'v_max': config.v_max,
                'omega_max': config.omega_max,
                'a_t_max': config.a_t_max,
                'a_r_max': config.a_r_max,
                'f_max': config.f_max,
                'mass': config.mass,
                't_react': config.t_react
            }
        except AttributeError:
            # Fallback to default constraints
            self.node.get_logger().warn(
                "Robot constraints config not found in node, using default values"
            )
            return {
                'v_max': 1.5,          # Maximum translational velocity (m/s)
                'omega_max': 1.5,      # Maximum rotational velocity (rad/s)
                'a_t_max': 0.5,        # Maximum translational acceleration/deceleration (m/s²)
                'a_r_max': 1.0,        # Maximum rotational acceleration (rad/s²)
                'f_max': 40,           # Maximum centripetal force (N)
                'mass': 10,            # Robot mass (kg)
                't_react': 0.1,        # Robot reaction time (s)
            }

    def _validate_constraints(self) -> None:
        """Validate that all required constraints are provided and positive."""
        required_constraints = [
            'v_max', 'omega_max', 'a_t_max', 'a_r_max', 
            'f_max', 'mass', 't_react'
        ]
        for constraint in required_constraints:
            if constraint not in self.constraints:
                error_msg = f"Missing required constraint: {constraint}"
                self.node.get_logger().error(error_msg)
                raise ValueError(error_msg)
                
            if self.constraints[constraint] <= 0:
                error_msg = f"Constraint {constraint} must be positive"
                self.node.get_logger().error(error_msg)
                raise ValueError(error_msg)
        
        self.node.get_logger().debug(
            f"Robot constraints validated: v_max={self.constraints['v_max']}m/s, "
            f"a_t_max={self.constraints['a_t_max']}m/s²"
        )

    def compute_isolated_constraints(self, point: PlanningPoint) -> float:
        """
        Compute maximum velocity based on isolated constraints.
        
        Args:
            point: PlanningPoint instance
            
        Returns:
            float: Maximum allowable velocity at this point
        """
        v_max = self.constraints['v_max']
        
        # Rotational velocity constraint: v ≤ omega_max / |curvature|
        if abs(point.curvature) > self.CURVATURE_EPSILON:
            v_rot = self.constraints['omega_max'] / abs(point.curvature)
            v_max = min(v_max, v_rot)
            
        # Centripetal force constraint: v ≤ sqrt(f_max / (m * |curvature|))
        if abs(point.curvature) > self.CURVATURE_EPSILON:
            v_cent = np.sqrt(self.constraints['f_max'] / 
                          (self.constraints['mass'] * abs(point.curvature)))
            v_max = min(v_max, v_cent)
            
        return v_max

    def compute_accel_overlap_bound(self, point_prev: PlanningPoint, point_curr: PlanningPoint) -> float:
        """
        Compute upper bound for velocity at previous point that guarantees overlap 
        of acceleration constraints at current point.
        
        Args:
            point_prev: Previous point (i-1)
            point_curr: Current point (i)
        
        Returns:
            float: Upper bound for velocity at previous point
        """
        c_i_1 = point_prev.curvature
        c_i = point_curr.curvature
        delta_s = abs(point_curr.arc_length - point_prev.arc_length)
        
        a_t_max = self.constraints['a_t_max']
        a_w_max = self.constraints['a_r_max']
        
        # Case 1: c_i > 0 and c_i_1 >= 0
        if c_i > 0 and c_i_1 >= 0:
            if c_i > c_i_1:  # Case 1.1
                thresh = sqrt((2*delta_s*(((a_w_max) + (c_i*a_t_max))**2)) / (((a_t_max*(c_i + c_i_1)) + (2*a_w_max))*(c_i - c_i_1)))
            elif c_i < c_i_1:  # Case 1.2
                thresh1 = sqrt((8*c_i*a_w_max*delta_s) / ((c_i_1 + c_i)**2))
                
                tmp1 = sqrt((4*c_i*delta_s*((c_i*a_t_max) + (a_w_max))) / ((c_i_1 - c_i)*(c_i_1 + c_i)))
                tmp2 = sqrt((2*delta_s*((c_i*a_t_max) + (a_w_max))**2) / ((c_i_1 - c_i)*((2*a_w_max) + ((c_i_1 + c_i)*a_t_max))))
                thresh_tmp1 = min(tmp1, tmp2)
                
                thresh_tmp2 = min(sqrt((2*a_w_max*delta_s)/(c_i_1)), sqrt(2*a_t_max*delta_s))
                thresh_tmp3 = float('-inf')
                
                tmp = min(((2*a_w_max*delta_s)/(c_i_1)), (2*delta_s*((c_i*a_t_max) - (a_w_max))**2) / ((c_i_1 - c_i)*((2*a_w_max) - ((c_i_1 + c_i)*a_t_max))))
                
                if ((tmp > ((-4*c_i*delta_s*((c_i*a_t_max) - (a_w_max)))/((c_i_1 - c_i)*(c_i_1 + c_i)))) and (tmp > (2*a_t_max*delta_s))):
                    thresh_tmp3 = sqrt(tmp)
                
                thresh = max(max(thresh1, thresh_tmp1), max(thresh_tmp2, thresh_tmp3))
            else:  # Case 1.3: c_i = c_i_1
                thresh = float('inf')
                
        # Case 2: c_i < 0 and c_i_1 <= 0
        elif c_i < 0 and c_i_1 <= 0:
            if c_i > c_i_1:  # Case 2.1
                thresh1 = sqrt((-8*c_i*a_w_max*delta_s) / ((c_i_1 + c_i)**2))
                
                tmp1 = sqrt((-4*c_i*delta_s*((a_w_max) - (c_i*a_t_max))) / ((c_i_1 + c_i)*(c_i_1 - c_i)))
                tmp2 = sqrt((-2*delta_s*(((a_w_max) - (c_i*a_t_max))**2)) / ((c_i_1 - c_i)*((2*a_w_max) - ((c_i_1 + c_i)*a_t_max))))
                thresh_tmp1 = min(tmp1, tmp2)
                
                thresh_tmp2 = min(sqrt((-2*a_w_max*delta_s)/(c_i_1)), sqrt(2*a_t_max*delta_s))
                thresh_tmp3 = float('-inf')
                
                tmp = min((-2*a_w_max*delta_s)/(c_i_1), (-2*delta_s*(((a_w_max) + (c_i*a_t_max))**2)) / ((c_i_1 - c_i)*((2*a_w_max) + ((c_i_1 + c_i)*a_t_max))))
                
                if ((tmp > (-4*c_i*delta_s*((a_w_max) + (c_i*a_t_max)))/((c_i_1 - c_i)*(c_i_1 + c_i))) and (tmp > 2*a_t_max*delta_s)):
                    thresh_tmp3 = sqrt(tmp)
                
                thresh = max(max(thresh1, thresh_tmp1), max(thresh_tmp2, thresh_tmp3))
            elif c_i < c_i_1:  # Case 2.2
                thresh = sqrt((-2*delta_s*((c_i*a_t_max) - (a_w_max))**2) / ((c_i_1 - c_i)*(((c_i + c_i_1)*a_t_max) - (2*a_w_max))))
            else:  # Case 2.3: c_i = c_i_1
                thresh = float('inf')
                
        # Case 3: c_i < 0 and c_i_1 > 0
        elif c_i < 0 and c_i_1 > 0:
            vtwostarpos = sqrt((2*delta_s*a_w_max) / (c_i_1))
            precond = float('inf')
            
            if (c_i_1 + c_i) < 0:
                precond = sqrt((-4*c_i*delta_s*((c_i*a_t_max) - (a_w_max))) / ((c_i_1 - c_i)*(c_i_1 + c_i)))
            
            thresh_tmp = min(precond, sqrt((-2*delta_s*(((c_i*a_t_max) - (a_w_max))**2)) / ((c_i_1 - c_i)*(((c_i_1 + c_i)*a_t_max) - (2*a_w_max)))))
            
            thresh_tmp = max(thresh_tmp, sqrt(2*delta_s*a_t_max))
            thresh = min(thresh_tmp, vtwostarpos)
            
        # Case 4: c_i > 0 and c_i_1 < 0
        elif c_i > 0 and c_i_1 < 0:
            vonestarpos = sqrt((-2*delta_s*a_w_max) / (c_i_1))
            precond = float('inf')
            
            if c_i_1 + c_i > 0:
                precond = sqrt((-4*c_i*delta_s*((a_w_max) + (c_i*a_t_max))) / ((c_i_1 - c_i)*(c_i_1 + c_i)))
            
            thresh_tmp = min(precond, sqrt((-2*delta_s*(((a_w_max) + (c_i*a_t_max))**2)) / ((c_i_1 - c_i)*(((c_i_1 + c_i)*a_t_max) + (2*a_w_max)))))
            
            thresh_tmp = max(thresh_tmp, sqrt(2*delta_s*a_t_max))
            
            thresh = min(thresh_tmp, vonestarpos)
            
        # Case 5: c_i = 0
        elif c_i == 0:
            if c_i_1 > 0:  # Case 5.1
                vtwothatpos = sqrt((2*delta_s*a_w_max) / (c_i_1))
                
                thresh_tmp = max(sqrt(2*delta_s*a_t_max), sqrt((-2*delta_s*(a_w_max**2)) / (c_i_1*((c_i_1*a_t_max) - (2*a_w_max)))))
                
                thresh = min(vtwothatpos, thresh_tmp)
                
            elif c_i_1 < 0:  # Case 5.2
                vonehatpos = sqrt((-2*delta_s*a_w_max) / (c_i_1))
                
                thresh_tmp = max(sqrt(2*delta_s*a_t_max), sqrt((-2*delta_s*(a_w_max**2)) / (c_i_1*((c_i_1*a_t_max) + (2*a_w_max)))))
                
                thresh = min(vonehatpos, thresh_tmp)
            else:
                # c_i = 0 and c_i_1 = 0
                thresh = float('inf')
        
        # Case 6: c_i = 0 and c_i_1 = 0
        else:  # This is redundant with the else in Case 5, but keeping for clarity
            thresh = float('inf')
            
        return thresh

    def get_vel_bounds_due_to_trans_accel_constraints(self, point_prev: PlanningPoint, 
                                                     point_curr: PlanningPoint) -> VelocityRange:
        """
        Calculate velocity range based on translational acceleration constraints.
        
        Args:
            point_prev: Previous point
            point_curr: Current point
            
        Returns:
            VelocityRange: Valid velocity ranges
        """
        v_i_1 = point_prev.velocity
        delta_s = abs(point_curr.arc_length - point_prev.arc_length)
        a_t_max = self.constraints['a_t_max']
        
        # Maximum velocity (Equation 3.30)
        v_max = np.sqrt((v_i_1**2) + (2 * a_t_max * delta_s))
        
        # Minimum velocity (Equation 3.29)
        if (v_i_1**2) > (2 * a_t_max * delta_s):
            v_min = np.sqrt((v_i_1**2) - (2 * a_t_max * delta_s))
        else:
            v_min = 0.0
        
        return VelocityRange(ranges=[(v_min, v_max)])

    def get_vel_bounds_due_to_rot_accel_constraints(self, point_prev: PlanningPoint, 
                                                   point_curr: PlanningPoint) -> VelocityRange:
        """
        Calculate velocity range based on rotational acceleration constraints.
        
        Args:
            point_prev: Previous point
            point_curr: Current point
            
        Returns:
            VelocityRange: Valid velocity ranges
        """
        v_i_1 = point_prev.velocity
        c_i = point_curr.curvature
        c_i_1 = point_prev.curvature
        delta_s = abs(point_curr.arc_length - point_prev.arc_length)
        a_w_max = self.constraints['a_r_max']
        
        def calculate_v1_v2(c_i, c_i_1, v_i_1, delta_s, a_w_max):
            """Calculate v1 and v2 according to equation 3.32"""
            term = np.sqrt((((c_i + c_i_1)**2) * (v_i_1**2)) + 
                         (8*c_i*delta_s*a_w_max))
            v1 = (1/(2*c_i)) * (((c_i_1 - c_i)*v_i_1) + (term))
            v2 = (1/(2*c_i)) * (((c_i_1 - c_i)*v_i_1) - (term))
            return v1, v2
        
        def calculate_v1_v2_star(c_i, c_i_1, v_i_1, delta_s, a_w_max):
            """Calculate v1* and v2* according to equation 3.33"""
            term = np.sqrt((((c_i + c_i_1)**2) * (v_i_1**2)) - 
                         (8*c_i*delta_s*a_w_max))
            v1_star = (1/(2*c_i)) * (((c_i_1 - c_i)*v_i_1) + (term))
            v2_star = (1/(2*c_i)) * (((c_i_1 - c_i)*v_i_1) - (term))
            return v1_star, v2_star
        
        # Handle different cases based on curvature values
        # c_i = 0 case
        if abs(c_i) < self.CURVATURE_EPSILON:
            if abs(c_i_1) < self.EPSILON:
                return VelocityRange(ranges=[(float('-inf'), float('inf'))])
            elif c_i_1 > 0:
                v_hat1 = ((-2*delta_s*a_w_max)/(c_i_1*v_i_1)) - (v_i_1)
                v_hat2 = ((2*delta_s*a_w_max)/(c_i_1*v_i_1)) - (v_i_1)
                return VelocityRange(ranges=[(v_hat1, v_hat2)])
            else:  # c_i_1 < 0
                v_hat1 = ((-2*delta_s*a_w_max)/(c_i_1*v_i_1)) - (v_i_1)
                v_hat2 = ((2*delta_s*a_w_max)/(c_i_1*v_i_1)) - (v_i_1)
                return VelocityRange(ranges=[(v_hat2, v_hat1)])
        
        # Calculate base discriminant
        D = ((c_i + c_i_1)**2) * (v_i_1**2)
        term = 8 * c_i * delta_s * a_w_max
        
        if c_i > 0:
            discriminant = D - term
            v1, v2 = calculate_v1_v2(c_i, c_i_1, v_i_1, delta_s, a_w_max)
            if discriminant < 0:
                return VelocityRange(ranges=[(v2, v1)])
            else:
                v1_star, v2_star = calculate_v1_v2_star(c_i, c_i_1, v_i_1, delta_s, a_w_max)
                return VelocityRange(ranges=[(v2, v2_star), (v1_star, v1)])
        elif c_i < 0:
            discriminant = D + term
            v1_star, v2_star = calculate_v1_v2_star(c_i, c_i_1, v_i_1, delta_s, a_w_max)
            if discriminant < 0:
                return VelocityRange(ranges=[(v1_star, v2_star)])
            else:
                v1, v2 = calculate_v1_v2(c_i, c_i_1, v_i_1, delta_s, a_w_max)
                return VelocityRange(ranges=[(v1_star, v1), (v2, v2_star)])

        return VelocityRange(ranges=[])

    def establish_forward_consistency(self, points: List[PlanningPoint],
                                start_velocity: Optional[float] = None) -> None:
        """
        Establish forward consistency for the velocity profile.
        Ensures velocities can be reached through acceleration from previous points.
        
        Args:
            points: List of planning points
            start_velocity: Optional starting velocity (for blending)
        """
        # Set initial velocity
        if start_velocity is not None:
            # Use provided velocity, but respect isolated constraints
            points[0].velocity = min(start_velocity, points[0].velocity) # Start at min of provided and isolated velocity
            self.node.get_logger().debug(f"Starting velocity set to {points[0].velocity:.2f}m/s (from blending)")
        else:
            points[0].velocity = 0.0  # Default to start at rest
            self.node.get_logger().debug("Starting from rest (velocity = 0)")
        
        self.node.get_logger().debug(f"Establishing forward consistency for {len(points)} points")
        for i in range(1, len(points)):
            prev_point = points[i-1]
            curr_point = points[i]
            
            # Get translational acceleration bounds
            trans_bounds = self.get_vel_bounds_due_to_trans_accel_constraints(
                prev_point, curr_point
            )
            curr_point.forward_trans_bounds = str(trans_bounds.ranges)
            
            # Commented out rotational acceleration constraints
            # Use only translational constraints for now
            
            # Apply isolated constraints
            isolated_vel_range = VelocityRange(ranges=[(0, curr_point.velocity)])
            
            # Find final valid velocity
            intersected_range = trans_bounds.intersect(isolated_vel_range)
            if intersected_range.ranges:
                curr_point.velocity = max(r[1] for r in intersected_range.ranges)
        
        self.node.get_logger().debug("Forward consistency established successfully")

    def establish_backward_consistency(self, points: List[PlanningPoint],
                                 end_velocity: Optional[float] = None) -> None:
        """
        Establish backward consistency for the velocity profile.
        Ensures the robot can decelerate to stop at the end.
        
        Args:
            points: List of planning points
            end_velocity: Optional end velocity (0.0 for last window)
        """
        # Set final velocity
        if end_velocity is not None:
            points[-1].velocity = min(end_velocity, points[-1].vel_of_iso_constraints)
            self.node.get_logger().debug(f"End velocity set to {points[-1].velocity:.2f}m/s (from blending)")
        else:
            # For middle windows, use isolated constraints
            points[-1].velocity = points[-1].vel_of_iso_constraints
            self.node.get_logger().debug(f"End velocity set to {points[-1].velocity:.2f}m/s (from isolated constraints)")
        
        self.node.get_logger().debug(f"Establishing backward consistency for {len(points)} points")
        for i in range(len(points)-2, -1, -1):
            curr_point = points[i]
            next_point = points[i+1]
            
            # Get translational bounds
            trans_bounds = self.get_vel_bounds_due_to_trans_accel_constraints(
                next_point, curr_point
            )
            curr_point.backward_trans_bounds = str(trans_bounds.ranges)
            
            # Commented out rotational acceleration constraints
            # Use only translational constraints for now
            
            # Apply isolated constraints
            isolated_vel_range = VelocityRange(ranges=[(0, curr_point.velocity)])
            
            # Find final valid velocity
            intersected_range = isolated_vel_range.intersect(trans_bounds)
            if intersected_range.ranges:
                for r in intersected_range.ranges:
                    if curr_point.velocity >= r[1]:
                        curr_point.velocity = r[1]
                    else:
                        curr_point.velocity = r[0]
        
        self.node.get_logger().debug("Backward consistency established successfully")

    def apply_velocity_bounds(self, points: List[PlanningPoint]) -> None:
        """
        Apply velocity bounds to ensure smooth transitions.
        
        Args:
            points: List of planning points
        """
        n = len(points)
        self.node.get_logger().debug(f"Applying velocity bounds to {n} points")
        
        # Handle first point (forward bound only)
        if n > 1:
            forward_bound = self.compute_accel_overlap_bound(points[0], points[1])
            points[0].forward_overlap_bound = forward_bound
            points[0].velocity = min(points[0].velocity, forward_bound)
        
        # Handle intermediate points
        for i in range(1, n-1):
            curr_point = points[i]
            next_point = points[i+1]
            prev_point = points[i-1]
            
            # Calculate bounds
            forward_bound = self.compute_accel_overlap_bound(curr_point, next_point)
            backward_bound = self.compute_accel_overlap_bound(curr_point, prev_point)
            
            # Store bounds
            curr_point.forward_overlap_bound = forward_bound
            curr_point.backward_overlap_bound = backward_bound
            
            # Apply the more restrictive bound
            overlap_bound = min(forward_bound, backward_bound)
            curr_point.velocity = min(curr_point.velocity, overlap_bound)
        
        # Handle last point (backward bound only)
        if n > 1:
            backward_bound = self.compute_accel_overlap_bound(points[-1], points[-2])
            points[-1].backward_overlap_bound = backward_bound
            points[-1].velocity = min(points[-1].velocity, backward_bound)
        
        self.node.get_logger().debug("Velocity bounds applied successfully")

    def compute_timestamps(self, points: List[PlanningPoint], 
                      start_time: Optional[float] = None) -> None:
        """
        Compute timestamps for each point using Equation 3.20:
        Δt = (2Δs)/(vi + vi-1)
        
        Args:
            points: List of planning points
            start_time: Optional starting time for blending
                   (None for first window, join_time for others)
        """
        # Set initial time
        points[0].time = start_time if start_time is not None else 0.0
        
        self.node.get_logger().debug(
            f"Computing timestamps for {len(points)} points, starting at t={points[0].time:.2f}s"
        )
        
        for i in range(1, len(points)):
            prev_point = points[i-1]
            curr_point = points[i]
            
            delta_s = abs(curr_point.arc_length - prev_point.arc_length)
            # Avoid division by zero
            avg_velocity = (curr_point.velocity + prev_point.velocity) / 2
            
            if avg_velocity < self.EPSILON:
                # If velocity is near zero, use a small default value to avoid huge time steps
                delta_t = delta_s / 0.1  # Assume 0.1 m/s minimum speed
                self.node.get_logger().warn(
                    f"Near-zero velocity at point {i}, using default speed for timestamp calculation"
                )
            else:
                # Normal case: Δt = (2Δs)/(vi + vi-1)
                delta_t = (2 * delta_s) / (curr_point.velocity + prev_point.velocity)
            
            curr_point.time = prev_point.time + delta_t
        
        total_time = points[-1].time - points[0].time
        self.node.get_logger().debug(f"Timestamps computed, total trajectory time: {total_time:.2f}s")

    def generate_velocity_profile(self, points: List[PlanningPoint],
                            start_velocity: Optional[float] = None,
                            end_velocity: Optional[float] = None,
                            start_time: Optional[float] = None) -> List[PlanningPoint]:
        """
        Generate complete velocity profile through three phases:
        1. Apply isolated constraints
        2. Forward pass to establish acceleration consistency
        3. Backward pass to establish deceleration consistency
        
        Args:
            points: List of planning points
            start_velocity: Optional velocity at start point (for blending)
            end_velocity: Optional velocity at end point (0.0 for last window)
            start_time: Optional starting time (join_time for blending)
        Returns:
            List[PlanningPoint]: Points with computed velocities and timestamps
        """
        if not points:
            self.node.get_logger().error("Cannot generate velocity profile: No points provided")
            return []
            
        start_generation = time.time()
        self.node.get_logger().info(
            f"Generating velocity profile for {len(points)} points "
            f"(start_vel={start_velocity}, end_vel={end_velocity})"
        )
        
        # Phase 1: Apply isolated constraints
        for point in points:
            point.velocity = self.compute_isolated_constraints(point)
            point.vel_of_iso_constraints = point.velocity
        
        self.node.get_logger().debug(
            f"Isolated constraints applied, max velocity: "
            f"{max([p.velocity for p in points]):.2f}m/s"
        )
        
        # Apply velocity bounds (optional - currently commented out)
        # self.apply_velocity_bounds(points)
        
        # Phase 2: Forward pass with optional start velocity
        self.establish_forward_consistency(points, start_velocity)
        
        # Phase 3: Backward pass with optional end velocity
        self.establish_backward_consistency(points, end_velocity)
        
        # Compute timestamps
        self.compute_timestamps(points, start_time)
        
        generation_time = time.time() - start_generation
        self.node.get_logger().info(
            f"Velocity profile generated in {generation_time:.3f}s, "
            f"trajectory duration: {points[-1].time - points[0].time:.2f}s"
        )
        
        return points

    def verify_velocity_profile(self, points: List[PlanningPoint]) -> bool:
        """
        Verify that the velocity profile satisfies acceleration constraints.
        
        Args:
            points: List of planning points
            
        Returns:
            bool: True if profile satisfies constraints
        """
        self.node.get_logger().debug(f"Verifying velocity profile for {len(points)} points")
        is_valid = True
        
        for i in range(1, len(points)):
            prev_point = points[i-1]
            curr_point = points[i]
            
            delta_s = abs(curr_point.arc_length - prev_point.arc_length)
            v_squared_diff = abs(curr_point.velocity**2 - prev_point.velocity**2)
            
            if abs(delta_s) > self.DELTA_S_EPSILON:
                actual_accel = v_squared_diff / (2 * delta_s)
                rounded_accel = round(actual_accel, 1)
                
                if abs(rounded_accel) > self.constraints['a_t_max']:
                    self.node.get_logger().error(
                        f"Acceleration constraint violated at point {i}: "
                        f"actual={actual_accel:.2f}m/s², "
                        f"max allowed={self.constraints['a_t_max']}m/s²"
                    )
                    is_valid = False
        
        if is_valid:
            self.node.get_logger().info("Velocity profile verification passed")
        else:
            self.node.get_logger().warn("Velocity profile verification failed")
        
        return is_valid


class VelocityProfileVisualizer:
    """Utility class for visualizing velocity profiles."""
    
    # Class constants
    DEFAULT_FIGURE_SIZE = (12, 10)
    DEFAULT_SUBPLOT_COUNT = 3
    
    @staticmethod
    def plot_profile(node: Node, points: List[PlanningPoint]) -> None:
        """
        Visualize the velocity profile.
        
        Args:
            node: ROS2 Node for logging
            points: List of planning points
        """
        node.get_logger().info("Generating velocity profile visualization")
        
        arc_lengths = [p.arc_length for p in points]
        velocities = [p.velocity for p in points]
        curvatures = [p.curvature for p in points]
        times = [p.time for p in points]
        
        fig, (ax1, ax2, ax3) = plt.subplots(
            VelocityProfileVisualizer.DEFAULT_SUBPLOT_COUNT, 
            1, 
            figsize=VelocityProfileVisualizer.DEFAULT_FIGURE_SIZE
        )
        
        # Velocity vs Arc Length
        ax1.plot(arc_lengths, velocities, 'b-', label='Velocity')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.set_xlabel('Arc Length (m)')
        ax1.grid(True)
        ax1.legend()
        
        # Curvature vs Arc Length
        ax2.plot(arc_lengths, curvatures, 'g-', label='Curvature')
        ax2.set_ylabel('Curvature')
        ax2.set_xlabel('Arc Length (m)')
        ax2.grid(True)
        ax2.legend()
        
        # Velocity vs Time
        ax3.plot(times, velocities, 'r-', label='Velocity')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.set_xlabel('Time (s)')
        ax3.grid(True)
        ax3.legend()
        
        plt.tight_layout()
        node.get_logger().info("Displaying velocity profile visualization")
        plt.show()
        
    @staticmethod
    def save_to_csv(node: Node, points: List[PlanningPoint], filename: str) -> None:
        """
        Save planning points data to CSV for debugging and analysis.
        
        Args:
            node: ROS2 Node for logging
            points: List of planning points
            filename: Output CSV file path
        """
        try:
            import pandas as pd
            
            data = []
            for i, point in enumerate(points):
                row = {
                    'point_index': i,
                    'position_x': point.position[0],
                    'position_y': point.position[1],
                    'curvature': point.curvature,
                    'heading': point.heading,
                    'arc_length': point.arc_length,
                    'velocity': point.velocity,
                    'time': point.time,
                    'vel_of_iso_constraints': point.vel_of_iso_constraints,
                    'forward_overlap_bound': point.forward_overlap_bound,
                    'backward_overlap_bound': point.backward_overlap_bound,
                    'forward_trans_bounds': point.forward_trans_bounds,
                    'forward_rot_bounds_prime': point.forward_rot_bounds_prime,
                    'forward_rot_bounds': point.forward_rot_bounds,
                    'forward_valid_bounds': point.forward_valid_bounds,
                    'backward_trans_bounds': point.backward_trans_bounds,
                    'backward_rot_bounds_prime': point.backward_rot_bounds_prime,
                    'backward_rot_bounds': point.backward_rot_bounds,
                    'backward_valid_bounds': point.backward_valid_bounds
                }
                data.append(row)
            
            df = pd.DataFrame(data)
            df.to_csv(filename, index=False)
            node.get_logger().info(f"Planning points data saved to {filename}")
        except ImportError:
            node.get_logger().error("Cannot save to CSV: pandas library not available")
        except Exception as e:
            node.get_logger().error(f"Error saving to CSV: {str(e)}")
