"""
Quintic Bézier spline generation with curvature continuity.
This module provides classes and functions for generating quintic Bézier splines
that maintain curvature continuity between segments.
"""

import numpy as np
from rclpy.node import Node
from dataclasses import dataclass
from typing import List, Tuple, Optional, Union
import time
from amr_trajectory_planner.config_types import BlendingConstraints

@dataclass
class SplineSegment:
    """Represents a single quintic Bézier curve segment."""
    control_points: np.ndarray  # Shape (6, 2) for quintic Bézier
    start_waypoint: np.ndarray
    end_waypoint: np.ndarray
    tangent_vectors: Tuple[np.ndarray, np.ndarray]  # Start and end tangents
    second_derivatives: Tuple[np.ndarray, np.ndarray]  # Start and end second derivatives

class BezierSplineGenerator:
    """Generates quintic Bézier splines with curvature continuity."""
    
    # Class constants
    EPSILON = 1e-10  # Small value to prevent division by zero
    FIRST_DERIVATIVE = 1
    SECOND_DERIVATIVE = 2
    BEZIER_DEGREE = 5  # Degree of the Bézier curve (quintic)
    
    def __init__(self, node: Node, waypoints: List[np.ndarray], tangent_factors: Optional[Union[float, List[float]]] = None, blend_constraints: Optional[BlendingConstraints] = None):
        """
        Initialize the spline generator.
        
        Args:
            node: ROS2 Node for logging and parameter access
            waypoints: List of waypoint coordinates
            tangent_factors: Either a single float for all waypoints or list of 
                            factors per waypoint. If None, defaults to base_tangent_factor from config.
            blend_constraints: Optional dict containing:
                'position': np.ndarray - Join point position 
                'first_derivative': np.ndarray - First derivative at join
                'second_derivative': np.ndarray - Second derivative at join
        """
        self.node = node
        self.waypoints = np.array(waypoints)
        self.blend_constraints = blend_constraints
        self.use_heuristic_start = (blend_constraints is None)
        
        # Get base tangent factor from config if available
        base_tangent_factor = 0.5  # Default value
        try:
            base_tangent_factor = self.node.trajectory_optimization_config.base_tangent_factor
            self.node.get_logger().debug(f"Using base tangent factor from config: {base_tangent_factor}")
        except AttributeError:
            self.node.get_logger().debug("No base_tangent_factor config found, using default: 0.5")
        
        # Handle tangent factors
        if tangent_factors is None:
            self.tangent_factors = [base_tangent_factor] * len(waypoints)
        elif isinstance(tangent_factors, float):
            self.tangent_factors = [tangent_factors] * len(waypoints)
        else:
            if len(tangent_factors) != len(waypoints):
                error_msg = "Number of tangent factors must match number of waypoints"
                self.node.get_logger().error(error_msg)
                raise ValueError(error_msg)
            self.tangent_factors = tangent_factors
        
        self.segments: List[SplineSegment] = []
        self._original_tangents: Optional[List[np.ndarray]] = None
        
        # Generate initial spline
        start_time = time.time()
        self._generate_spline()
        generation_time = time.time() - start_time
        self.node.get_logger().debug(f"Spline generation completed in {generation_time:.4f} seconds")
        self.node.get_logger().debug(f"Generated {len(self.segments)} spline segments")

    def _euclidean_distance(self, p1: np.ndarray, p2: np.ndarray) -> float:
        """Calculate Euclidean distance between two points."""
        return np.linalg.norm(p2 - p1)
    
    def _calculate_heuristic_tangents(self) -> List[np.ndarray]:
        """Calculate initial tangent vectors for all waypoints."""
        tangent_vectors = []
        n = len(self.waypoints)

        # First waypoint tangent
        if self.use_heuristic_start:
            # Original heuristic calculation
            distance_01 = self._euclidean_distance(self.waypoints[0], self.waypoints[1])
            first_tangent = self.tangent_factors[0] * distance_01 * (
                self.waypoints[1] - self.waypoints[0]
            ) / distance_01
            tangent_vectors.append(first_tangent)
        else:
            # Use provided first derivative
            tangent_vectors.append(self.blend_constraints.first_derivative)
        
        # Inner waypoint tangents
        for i in range(1, n-1):
            # Calculate distances and direction vectors
            prev_dist = self._euclidean_distance(self.waypoints[i-1], self.waypoints[i])
            next_dist = self._euclidean_distance(self.waypoints[i], self.waypoints[i+1])
            prev_dir = (self.waypoints[i] - self.waypoints[i-1]) / prev_dist
            next_dir = (self.waypoints[i+1] - self.waypoints[i]) / next_dist
            
            # Calculate bisector vector for perpendicular direction
            bisector = (-prev_dir + next_dir)
            if np.linalg.norm(bisector) > self.EPSILON:  # Avoid division by zero
                bisector /= np.linalg.norm(bisector)
                
                # Get perpendicular vector in correct direction
                perpendicular = self._get_correct_perpendicular(bisector, next_dir)
                
                # Scale by minimum distance
                tangent_magnitude = self.tangent_factors[i] * min(prev_dist, next_dist)
                tangent_vectors.append(tangent_magnitude * perpendicular)
            else:
                # If bisector is zero (straight line), use direction vector
                tangent_magnitude = self.tangent_factors[i] * min(prev_dist, next_dist)
                tangent_vectors.append(tangent_magnitude * next_dir)

        # Last waypoint tangent
        distance_last = self._euclidean_distance(self.waypoints[-2], self.waypoints[-1])
        last_tangent = self.tangent_factors[-1] * distance_last * (
            self.waypoints[-1] - self.waypoints[-2]
        ) / distance_last
        tangent_vectors.append(last_tangent)
        
        return tangent_vectors

    def _get_correct_perpendicular(self, bisector: np.ndarray, 
                                 travel_direction: np.ndarray) -> np.ndarray:
        """Select correct perpendicular vector based on travel direction."""
        # Two possible perpendicular vectors
        perp1 = np.array([-bisector[1], bisector[0]])
        perp2 = np.array([bisector[1], -bisector[0]])
        
        # Select vector with positive dot product with travel direction
        return perp1 if np.dot(perp1, travel_direction) > 0 else perp2

    def _calculate_second_derivatives(self, tangents: List[np.ndarray]) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Calculate second derivatives for cubic Bézier approximation."""
        second_derivatives = []
        
        for i in range(len(self.waypoints) - 1):
            if i == 0 and not self.use_heuristic_start:
                # Use provided second derivative for first segment
                start_deriv = self.blend_constraints.second_derivative
                # Calculate end derivative normally
                A, B = self.waypoints[i], self.waypoints[i+1]
                t_A, t_B = tangents[i], tangents[i+1]
                end_deriv = 6*A + 2*t_A + 4*t_B - 6*B
            else:
                # Original calculation for other segments
                A, B = self.waypoints[i], self.waypoints[i+1]
                t_A, t_B = tangents[i], tangents[i+1]
                start_deriv = -6*A - 4*t_A - 2*t_B + 6*B
                end_deriv = 6*A + 2*t_A + 4*t_B - 6*B
            
            second_derivatives.append((start_deriv, end_deriv))
        
        return second_derivatives

    def _calculate_weighted_second_derivatives(self, second_derivatives: List[Tuple[np.ndarray, np.ndarray]]) -> List[np.ndarray]:
        """Calculate weighted second derivatives for smooth transitions."""
        weighted_derivatives = []
        
        # First point
        if self.use_heuristic_start:
            weighted_derivatives.append(second_derivatives[0][0])
        else:
            weighted_derivatives.append(self.blend_constraints.second_derivative)
        
        # Inner points (unchanged)
        for i in range(1, len(self.waypoints) - 1):
            d_prev = self._euclidean_distance(self.waypoints[i-1], self.waypoints[i])
            d_next = self._euclidean_distance(self.waypoints[i], self.waypoints[i+1])
            
            # Distance-based weights
            w_next = d_prev / (d_prev + d_next)
            w_prev = d_next / (d_prev + d_next)
            
            weighted_deriv = (
                w_prev * second_derivatives[i-1][1] + 
                w_next * second_derivatives[i][0]
            )
            weighted_derivatives.append(weighted_deriv)
        
        # Last point (unchanged)
        weighted_derivatives.append(second_derivatives[-1][1])
        
        return weighted_derivatives

    def _generate_control_points(self, tangents: List[np.ndarray], 
                               weighted_second_derivatives: List[np.ndarray]) -> None:
        """Generate control points for each quintic Bézier segment."""
        self.segments.clear()
        
        for i in range(len(self.waypoints) - 1):
            P0 = self.waypoints[i]
            P5 = self.waypoints[i+1]
            t_s = tangents[i]
            t_e = tangents[i+1]
            a_s = weighted_second_derivatives[i]
            a_e = weighted_second_derivatives[i+1]
            
            # Calculate intermediate control points
            P1 = P0 + (1/5) * t_s
            P2 = (1/20) * a_s + 2*P1 - P0
            P4 = P5 - (1/5) * t_e
            P3 = (1/20) * a_e + 2*P4 - P5
            
            control_points = np.array([P0, P1, P2, P3, P4, P5])
            
            # Create segment
            segment = SplineSegment(
                control_points=control_points,
                start_waypoint=P0,
                end_waypoint=P5,
                tangent_vectors=(t_s, t_e),
                second_derivatives=(a_s, a_e)
            )
            self.segments.append(segment)

    def _generate_spline(self) -> None:
        """Generate the complete spline."""
        # Calculate initial tangent vectors
        self.node.get_logger().debug("Calculating tangent vectors...")
        self._original_tangents = self._calculate_heuristic_tangents()
        
        # Calculate second derivatives
        self.node.get_logger().debug("Calculating second derivatives...")
        second_derivatives = self._calculate_second_derivatives(self._original_tangents)
        weighted_second_derivatives = self._calculate_weighted_second_derivatives(second_derivatives)
        
        # Generate control points
        self.node.get_logger().debug("Generating control points...")
        self._generate_control_points(self._original_tangents, weighted_second_derivatives)

    def update_tangent_factor(self, new_factor: float) -> None:
        """
        Update the tangent factor and regenerate the spline.
        
        Args:
            new_factor: New tangent scaling factor
        """
        if new_factor <= 0:
            error_msg = "Tangent factor must be positive"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
            
        self.node.get_logger().info(f"Updating tangent factor to {new_factor}")
        self.tangent_factor = new_factor
        
        if self._original_tangents:
            # Scale existing tangents instead of recalculating
            scaled_tangents = [t * (new_factor/0.5) for t in self._original_tangents]
            
            # Recalculate second derivatives with scaled tangents
            second_derivatives = self._calculate_second_derivatives(scaled_tangents)
            weighted_second_derivatives = self._calculate_weighted_second_derivatives(second_derivatives)
            
            # Update control points
            self._generate_control_points(scaled_tangents, weighted_second_derivatives)
        else:
            # Full regeneration if no original tangents stored
            self.node.get_logger().warn("No original tangents available, regenerating spline")
            self._generate_spline()

    def get_point_at_parameter(self, segment_idx: int, t: float) -> np.ndarray:
        """
        Get point on the spline at given parameter value.
        
        Args:
            segment_idx: Index of the curve segment
            t: Parameter value between 0 and 1
            
        Returns:
            np.ndarray: Point coordinates
        """
        if not 0 <= segment_idx < len(self.segments):
            error_msg = f"Invalid segment index: {segment_idx}, max: {len(self.segments)-1}"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
        if not 0 <= t <= 1:
            error_msg = f"Parameter t must be between 0 and 1, got: {t}"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
            
        ctrl_pts = self.segments[segment_idx].control_points
        point = np.zeros(2)
        
        # Use Bernstein polynomials for quintic Bézier
        for i in range(6):
            coef = self._bernstein_polynomial(5, i, t)
            point += ctrl_pts[i] * coef
            
        return point

    def get_derivative_at_parameter(self, segment_idx: int, t: float, 
                                  order: int = 1) -> np.ndarray:
        """
        Get derivative of specified order at given parameter value.
        
        Args:
            segment_idx: Index of the curve segment
            t: Parameter value between 0 and 1
            order: Order of derivative (1 for first, 2 for second)
            
        Returns:
            np.ndarray: Derivative vector
        """
        if not 0 <= segment_idx < len(self.segments):
            error_msg = f"Invalid segment index: {segment_idx}, max: {len(self.segments)-1}"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
        if not 0 <= t <= 1:
            error_msg = f"Parameter t must be between 0 and 1, got: {t}"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
        if order not in [self.FIRST_DERIVATIVE, self.SECOND_DERIVATIVE]:
            error_msg = f"Only first and second derivatives supported, got order: {order}"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
            
        ctrl_pts = self.segments[segment_idx].control_points
        n = 5  # Degree of quintic Bézier
        
        if order == 1:
            # First derivative
            derivative = np.zeros(2)
            for i in range(n):
                coef = self._bernstein_polynomial(n-1, i, t)
                derivative += n * (ctrl_pts[i+1] - ctrl_pts[i]) * coef
            return derivative
        else:
            # Second derivative
            derivative = np.zeros(2)
            for i in range(n-1):
                coef = self._bernstein_polynomial(n-2, i, t)
                derivative += n * (n-1) * (
                    ctrl_pts[i+2] - 2*ctrl_pts[i+1] + ctrl_pts[i]
                ) * coef
            return derivative
        
    @staticmethod
    def calculate_derivative_from_control_points(control_points: np.ndarray, 
                                               t: float, 
                                               order: int) -> np.ndarray:
        """Static utility method to calculate derivatives from control points"""
        n = len(control_points) - 1  # Degree of Bézier curve
        if order == 1:
            derivative = np.zeros(2)
            for i in range(n):
                coef = BezierSplineGenerator._bernstein_polynomial(n-1, i, t)
                derivative += n * (control_points[i+1] - control_points[i]) * coef
            return derivative
        else:
            derivative = np.zeros(2)
            for i in range(n-1):
                coef = BezierSplineGenerator._bernstein_polynomial(n-2, i, t)
                derivative += n * (n-1) * (
                    control_points[i+2] - 2*control_points[i+1] + control_points[i]
                ) * coef
            return derivative

    @staticmethod
    def _bernstein_polynomial(n: int, i: int, t: float) -> float:
        """Calculate value of Bernstein polynomial."""
        from math import comb
        return comb(n, i) * (1-t)**(n-i) * t**i

class EquidistantPointGenerator:
    """
    Utility class for generating equidistant points along the spline.
    
    This class provides algorithms for generating points with equal arc length spacing along a Bézier
    spline curve, which is important for trajectory planning with consistent speed profiles.
    
    Two methods are supported:
    
    1. Simpson's Rule Approach (more accurate):
       Integrates the speed function using Simpson's rule to calculate accurate arc lengths,
       then finds parameters corresponding to equidistant points.
       
    2. Linear Approach (faster):
       Uses linear approximation between sampled points to estimate arc lengths.
    """
    # Class constants
    DEFAULT_POINTS_PER_SEGMENT = 100
    DEFAULT_TOTAL_POINTS = 50
    MIN_POINTS = 2
    SIMPSON_INTERVALS = 20  # Number of intervals for Simpson's rule
    MAX_PARAMETER = 1.0

    class Method:
        """Enumeration of available arc length calculation methods."""
        LINEAR = "linear"
        SIMPSON = "simpson"
    
    def __init__(self, node: Node, spline: BezierSplineGenerator, method: str = None):
        """
        Initialize the point generator.
        
        Args:
            node: ROS2 Node for logging and parameter access
            spline: BezierSplineGenerator instance
            method: Arc length calculation method ("linear" or "simpson")
                   If None, uses the method from node configuration
        """
        self.node = node
        self.spline = spline
        self._arc_length_table = None
        self._points_per_segment = self.DEFAULT_POINTS_PER_SEGMENT
        
        # Get method from configuration if not specified
        if method is None:
            try:
                method = self.node.trajectory_optimization_config.arc_length_calculation_method
                self.node.get_logger().debug(f"Using arc length calculation method from config: {method}")
            except AttributeError:
                method = self.Method.SIMPSON
                self.node.get_logger().debug(f"No arc length method config found, using default: {method}")
        
        if method not in [self.Method.LINEAR, self.Method.SIMPSON]:
            error_msg = f"Method must be either 'linear' or 'simpson', got: {method}"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
        
        self._method = method
        self.node.get_logger().info(f"Initialized EquidistantPointGenerator with {method} method")
    
    def _arc_length_simpson(self, segment_idx: int, t_start: float = 0, 
                          t_end: float = 1, n: int = SIMPSON_INTERVALS) -> float:
        """
        Calculate arc length of a segment using Simpson's Rule.
        
        Args:
            segment_idx: Index of spline segment
            t_start: Start parameter value
            t_end: End parameter value
            n: Number of intervals (must be even)
            
        Returns:
            float: Arc length of the segment
        """
        # Ensure even number of intervals
        if n % 2 != 0:
            n += 1
            
        h = (t_end - t_start) / n
        t_values = np.linspace(t_start, t_end, n + 1)
        
        # Calculate speed values at each point
        f_values = []
        for t in t_values:
            derivative = self.spline.get_derivative_at_parameter(segment_idx, t)
            speed = np.sqrt(np.sum(derivative ** 2))
            f_values.append(speed)
        
        # Apply Simpson's Rule
        integral = f_values[0] + f_values[-1]
        for i in range(1, n, 2):
            integral += 4 * f_values[i]
        for i in range(2, n-1, 2):
            integral += 2 * f_values[i]
            
        integral *= h / 3
        return integral
    
    def _calculate_arc_length_table(self) -> None:
        """Calculate lookup table for arc lengths along the spline."""
        start_time = time.time()
        if self._method == self.Method.SIMPSON:
            self.node.get_logger().debug("Calculating arc length table using Simpson's rule")
            self._calculate_arc_length_table_simpson()
        else:
            self.node.get_logger().debug("Calculating arc length table using linear approximation")
            self._calculate_arc_length_table_linear()
            
        calc_time = time.time() - start_time
        self.node.get_logger().debug(f"Arc length table calculated in {calc_time:.4f} seconds")
            
    def _calculate_arc_length_table_linear(self) -> None:
        """Calculate arc lengths using simple linear approximation between points."""
        total_points = self._points_per_segment * len(self.spline.segments)
        self._arc_length_table = {
            'global_t': np.zeros(total_points),
            'arc_lengths': np.zeros(total_points),
            'indices': []
        }
        
        cumulative_length = 0
        prev_point = None
        
        for seg_idx, segment in enumerate(self.spline.segments):
            start_idx = seg_idx * self._points_per_segment
            t_local = np.linspace(0, 1, self._points_per_segment)
            
            # Store global t-values
            self._arc_length_table['global_t'][start_idx:start_idx + self._points_per_segment] = (
                seg_idx + t_local
            )
            
            # Calculate arc lengths using linear distances
            for i, t in enumerate(t_local):
                point = self.spline.get_point_at_parameter(seg_idx, t)
                idx = start_idx + i
                
                if prev_point is not None:
                    segment_length = np.linalg.norm(point - prev_point)
                    cumulative_length += segment_length
                
                self._arc_length_table['arc_lengths'][idx] = cumulative_length
                self._arc_length_table['indices'].append((seg_idx, t))
                prev_point = point
                
    def _calculate_arc_length_table_simpson(self) -> None:
        """Calculate arc lengths using Simpson's Rule for higher accuracy."""
        total_points = self._points_per_segment * len(self.spline.segments)
        self._arc_length_table = {
            'global_t': np.zeros(total_points),
            'arc_lengths': np.zeros(total_points),
            'indices': []
        }
        
        cumulative_length = 0
        
        for seg_idx in range(len(self.spline.segments)):
            start_idx = seg_idx * self._points_per_segment
            t_local = np.linspace(0, 1, self._points_per_segment)
            
            # Store global t-values
            self._arc_length_table['global_t'][start_idx:start_idx + self._points_per_segment] = (
                seg_idx + t_local
            )
            
            # Calculate cumulative arc lengths using Simpson's Rule
            for i, t_end in enumerate(t_local):
                if i > 0:  # Skip first point (t=0)
                    t_start = t_local[i-1]
                    segment_length = self._arc_length_simpson(
                        seg_idx, t_start, t_end, n=20  # 20 intervals for each small segment
                    )
                    cumulative_length += segment_length
                
                idx = start_idx + i
                self._arc_length_table['arc_lengths'][idx] = cumulative_length
                self._arc_length_table['indices'].append((seg_idx, t_local[i]))
    
    def generate_points(self, num_points: Optional[int] = None, 
                       spacing: Optional[float] = None) -> Tuple[np.ndarray, List[Tuple[int, float, float]], np.ndarray]:
        """
        Generate equidistant points along the spline.
        
        Args:
            num_points: Number of points to generate
            spacing: Distance between points (alternative to num_points)
            
        Returns:
            Tuple containing:
            - Array of point coordinates
            - List of (segment_index, local_parameter, heading) pairs
            - Array of cumulative arc lengths
        """
        # Initialize arc length table if not already done
        if self._arc_length_table is None:
            self.node.get_logger().debug("Generating arc length lookup table")
            self._calculate_arc_length_table()
        
        arc_lengths = self._arc_length_table['arc_lengths']
        total_length = arc_lengths[-1]
        
        if spacing is not None:
            num_points = max(self.MIN_POINTS, int(total_length / spacing) + 1)
            self.node.get_logger().debug(f"Using spacing {spacing}m, generating {num_points} points")
        elif num_points is None:
            num_points = self.DEFAULT_TOTAL_POINTS  # Default value
            self.node.get_logger().debug(f"Using default point count: {num_points}")
        else:
            self.node.get_logger().debug(f"Generating {num_points} equidistant points")
        
        desired_lengths = np.linspace(0, total_length, num_points)
        points = np.zeros((num_points, 2))
        parameters = []
        
        for i, target_length in enumerate(desired_lengths):
            # Find corresponding global t-value through interpolation
            global_t = np.interp(target_length, 
                               self._arc_length_table['arc_lengths'],
                               self._arc_length_table['global_t'])
            
            # Convert to segment index and local parameter
            segment_idx = int(global_t)
            local_t = global_t - segment_idx
            
            # Handle edge case for last point
            if segment_idx >= len(self.spline.segments):
                segment_idx = len(self.spline.segments) - 1
                local_t = self.MAX_PARAMETER
            
            # Calculate point position
            points[i] = self.spline.get_point_at_parameter(segment_idx, local_t)
            
            # Calculate heading using derivative
            deriv = self.spline.get_derivative_at_parameter(segment_idx, local_t, order=1)
            heading = np.arctan2(deriv[1], deriv[0])
                
            parameters.append((segment_idx, local_t, heading))
        
        self.node.get_logger().info(f"Generated {num_points} equidistant points spanning {total_length:.2f}m")
        return points, parameters, desired_lengths