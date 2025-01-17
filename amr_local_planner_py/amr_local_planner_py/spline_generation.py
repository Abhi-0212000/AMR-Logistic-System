"""
Quintic Bézier spline generation with curvature continuity.
This module provides classes and functions for generating quintic Bézier splines
that maintain curvature continuity between segments.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Union, Dict
import matplotlib.pyplot as plt
from amr_local_planner_py.common_types import BlendingConstraints

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
    
    def __init__(self, waypoints: List[np.ndarray], tangent_factors: Optional[Union[float, List[float]]] = None, blend_constraints: Optional[BlendingConstraints] = None):
        """
        Initialize the spline generator.
        
        Args:
            waypoints: List of waypoint coordinates
            tangent_factors: Either a single float for all waypoints or list of 
                            factors per waypoint. If None, defaults to 0.5.
            blend_constraints: Optional dict containing:
                'position': np.ndarray - Join point position 
                'first_derivative': np.ndarray - First derivative at join
                'second_derivative': np.ndarray - Second derivative at join
        """
        self.waypoints = np.array(waypoints)
        self.blend_constraints = blend_constraints
        self.use_heuristic_start = (blend_constraints is None)
        
        # Handle tangent factors
        if tangent_factors is None:
            self.tangent_factors = [0.5] * len(waypoints)
        elif isinstance(tangent_factors, float):
            self.tangent_factors = [tangent_factors] * len(waypoints)
        else:
            if len(tangent_factors) != len(waypoints):
                raise ValueError("Number of tangent factors must match number of waypoints")
            self.tangent_factors = tangent_factors
        self.segments: List[SplineSegment] = []
        self._original_tangents: Optional[List[np.ndarray]] = None
        
        # Generate initial spline
        self._generate_spline()

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
            if np.linalg.norm(bisector) > 1e-10:  # Avoid division by zero
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
        
        # print(f"Tanget Vectors: {tangent_vectors}")
        
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
        self._original_tangents = self._calculate_heuristic_tangents()
        
        # Calculate second derivatives
        second_derivatives = self._calculate_second_derivatives(self._original_tangents)
        weighted_second_derivatives = self._calculate_weighted_second_derivatives(second_derivatives)
        
        # Generate control points
        self._generate_control_points(self._original_tangents, weighted_second_derivatives)

    def update_tangent_factor(self, new_factor: float) -> None:
        """
        Update the tangent factor and regenerate the spline.
        
        Args:
            new_factor: New tangent scaling factor
        """
        if new_factor <= 0:
            raise ValueError("Tangent factor must be positive")
            
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
            raise ValueError("Invalid segment index")
        if not 0 <= t <= 1:
            raise ValueError("Parameter t must be between 0 and 1")
            
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
            raise ValueError("Invalid segment index")
        if not 0 <= t <= 1:
            raise ValueError("Parameter t must be between 0 and 1")
        if order not in [1, 2]:
            raise ValueError("Only first and second derivatives supported")
            
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

class SplineVisualizer:
    """Utility class for visualizing Bézier splines."""
    
    @staticmethod
    def visualize_spline(spline: BezierSplineGenerator, show_control_points: bool = True,
                        show_waypoints: bool = True, num_points: int = 100) -> None:
        """
        Visualize the Bézier spline.
        
        Args:
            spline: BezierSplineGenerator instance
            show_control_points: Whether to show control points
            show_waypoints: Whether to show original waypoints
            num_points: Number of points to sample per segment
        """
        plt.figure(figsize=(12, 8))
        
        # Plot each segment
        for i, segment in enumerate(spline.segments):
            # Generate points along the curve
            t_values = np.linspace(0, 1, num_points)
            curve_points = np.array([
                spline.get_point_at_parameter(i, t) for t in t_values
            ])
            
            # Plot curve
            plt.plot(curve_points[:, 0], curve_points[:, 1], 'b-', 
                    linewidth=2, label='Spline' if i == 0 else '')
            
            if show_control_points:
                # Plot control points and their connecting lines
                ctrl_pts = segment.control_points
                plt.plot(ctrl_pts[:, 0], ctrl_pts[:, 1], 'r--', alpha=0.3)
                plt.plot(ctrl_pts[:, 0], ctrl_pts[:, 1], 'rx', 
                        label='Control Points' if i == 0 else '')
        
        if show_waypoints:
            # Plot waypoints
            waypoints = spline.waypoints
            plt.plot(waypoints[:, 0], waypoints[:, 1], 'ko', 
                    markersize=8, label='Waypoints')
        
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        plt.title('Quintic Bézier Spline')
        plt.show()

    @staticmethod
    def visualize_curvature(spline: BezierSplineGenerator, num_points: int = 100) -> None:
        """
        Visualize the curvature profile of the spline.
        
        Args:
            spline: BezierSplineGenerator instance
            num_points: Number of points to sample per segment
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Calculate curvature profile
        all_t = []
        all_curvature = []
        colors = plt.cm.rainbow(np.linspace(0, 1, len(spline.segments)))
        
        for i, segment in enumerate(spline.segments):
            t_values = np.linspace(0, 1, num_points)
            t_global = t_values + i
            
            curvature_segment = []
            curve_points = []
            
            for t in t_values:
                # Get point and derivatives
                point = spline.get_point_at_parameter(i, t)
                d1 = spline.get_derivative_at_parameter(i, t, order=1)
                d2 = spline.get_derivative_at_parameter(i, t, order=2)
                
                # Calculate curvature: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
                denominator = (d1[0]**2 + d1[1]**2)**(3/2)
                if denominator > 1e-10:  # Avoid division by zero
                    curvature = (d1[0]*d2[1] - d1[1]*d2[0]) / denominator
                else:
                    curvature = 0
                
                curvature_segment.append(curvature)
                curve_points.append(point)
            
            # Plot curvature for this segment
            ax1.plot(t_global, curvature_segment, color=colors[i], 
                    label=f'Segment {i+1}')
            
            # Plot spline segment
            curve_points = np.array(curve_points)
            ax2.plot(curve_points[:, 0], curve_points[:, 1], color=colors[i], 
                    linewidth=2, label=f'Segment {i+1}')
            
            all_t.extend(t_global)
            all_curvature.extend(curvature_segment)
        
        # Curvature plot formatting
        ax1.set_title('Curvature Profile')
        ax1.set_xlabel('Spline Parameter')
        ax1.set_ylabel('Curvature')
        ax1.grid(True)
        ax1.legend()
        
        # Spline plot formatting
        ax2.set_title('Spline Geometry')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.grid(True)
        ax2.axis('equal')
        ax2.plot(spline.waypoints[:, 0], spline.waypoints[:, 1], 'ko', 
                markersize=8, label='Waypoints')
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
        
        return np.array(all_t), np.array(all_curvature)

class EquidistantPointGenerator:
    """
        Utility class for generating equidistant points along the spline.
        
        Simpson's Rule Approach:
            1. First Level Splitting:
            - Each spline segment is split into n points (default 100)
            - These points represent our "checkpoints" for cumulative arc length

            2. Second Level Splitting (The nested part):
            - For each interval between checkpoints (t_start to t_end)
            - We split that small interval again (default 20 points) for Simpson integration
            - This gives us accurate arc length for just that small piece
            - We don't store these intermediate points
            - We only use them to calculate the arc length integral
            - Formula integrates speed (magnitude of derivative) using these points

            3. Result:
            - Store only the checkpoint parameters and their cumulative arc lengths
            - More accurate because we're actually integrating along the curve
            - Computationally more expensive due to nested sampling

        Linear Approach:
            1. Single Level Splitting:
            - Each spline segment is split into n points (default 100)
            - These are both our checkpoints and calculation points
            - No nested sampling needed

            2. Distance Calculation:
            - Simply calculate straight-line distances between consecutive points
            - Add these up for cumulative arc length
            - Much simpler but less accurate, especially for curved segments

        The Generate Points Method:
        - Works the same way for both approaches
        - Uses the lookup table we created (regardless of how we created it)
        - Interpolates between stored values to find parameters for desired arc lengths
        - Gets actual points using these parameters

        Think of it like this:
        - Linear: Like measuring distance with a ruler between points
        - Simpson: Like rolling a wheel along the actual curve between points

        The key insight is that Simpson's rule does more work upfront to build an accurate arc length table, but once that table exists, both methods use it the same way to generate the final equidistant points.

        This is why the generate_points() function looks identical for both methods - it's just using the lookup table differently. The real difference is in how we built that lookup table in the first place.
    """
    
    class Method:
        """Enumeration of available arc length calculation methods."""
        LINEAR = "linear"
        SIMPSON = "simpson"
    
    def __init__(self, spline: BezierSplineGenerator, method: str = Method.SIMPSON):
        """
        Initialize the point generator.
        
        Args:
            spline: BezierSplineGenerator instance
            method: Arc length calculation method ("linear" or "simpson")
        """
        self.spline = spline
        self._arc_length_table = None
        self._points_per_segment = 100
        if method not in [self.Method.LINEAR, self.Method.SIMPSON]:
            raise ValueError("Method must be either 'linear' or 'simpson'")
        self._method = method
    
    def _arc_length_simpson(self, segment_idx: int, t_start: float = 0, 
                          t_end: float = 1, n: int = 100) -> float:
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
        if self._method == self.Method.SIMPSON:
            self._calculate_arc_length_table_simpson()
        else:
            self._calculate_arc_length_table_linear()
            
    def _calculate_arc_length_table_linear(self) -> None:
        """Calculate arc lengths using simple linear approximation."""
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
        """Calculate arc lengths using Simpson's Rule."""
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
        if self._arc_length_table is None:
            self._calculate_arc_length_table()
        
        arc_lengths = self._arc_length_table['arc_lengths']
        total_length = arc_lengths[-1]
        
        if spacing is not None:
            num_points = max(2, int(total_length / spacing) + 1)
        elif num_points is None:
            num_points = 50  # Default value
        
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
                local_t = 1.0
            
            # Calculate point position
            points[i] = self.spline.get_point_at_parameter(segment_idx, local_t)
            
            # Calculate heading using derivative
            deriv = self.spline.get_derivative_at_parameter(segment_idx, local_t, order=1)
            heading = np.arctan2(deriv[1], deriv[0])
                
            parameters.append((segment_idx, local_t, heading))
        
        return points, parameters, desired_lengths

def main():
    """Example usage of the spline generation module."""
    # Example waypoints
    waypoints = [
        np.array([0., 0.]),
        np.array([1., 1.]),
        np.array([2., 0.]),
        np.array([3., 0.5])
    ]
    
    waypoints = [
        np.array([13.7414, 8.32168]),
        np.array([12.8583, 8.97716]),
        np.array([12.2636, 9.41402]),
        np.array([11.6704, 9.77417]),
        np.array([11.2832, 9.89637]),
        np.array([10.5473, 9.99393])
        # np.array([9.97879, 9.94867]),
        # np.array([9.35417, 9.864]),
        # np.array([8.4243, 9.5917])
    ]
    
    # Create spline
    spline = BezierSplineGenerator(waypoints)
    
    # Visualize
    visualizer = SplineVisualizer()
    visualizer.visualize_spline(spline)
    visualizer.visualize_curvature(spline)
    
    # Generate equidistant points
    point_gen = EquidistantPointGenerator(spline, method=EquidistantPointGenerator.Method.SIMPSON)
    points, params, arc_lengths = point_gen.generate_points(num_points=300)
    print(f"Generated {len(points)} equidistant points")
    print(f"Total arc length: {arc_lengths[-1]:.2f} meters")
    
    # Plot with equidistant points
    plt.figure(figsize=(10, 8))
    visualizer.visualize_spline(spline, show_control_points=False)
    plt.plot(points[:, 0], points[:, 1], 'go', label='Equidistant Points')
    plt.legend()
    plt.show()

# if __name__ == "__main__":
#     main()