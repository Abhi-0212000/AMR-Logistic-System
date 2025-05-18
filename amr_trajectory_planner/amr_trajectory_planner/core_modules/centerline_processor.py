"""
Centerline processor module for trajectory planning.

This module processes lanelet centerlines to:
1. Generate evenly spaced waypoints along a path
2. Create trajectory windows with appropriate boundaries
3. Support both linear and Bézier curve interpolation methods

The CenterlineProcessor is a core component for path planning that transforms
raw lanelet centerline data into properly formatted trajectory data suitable
for optimization and vehicle control.
"""

import numpy as np
from typing import List, Optional
from dataclasses import dataclass
from rclpy.node import Node

from amr_trajectory_planner.config_types import *
from lanelet2.core import BasicPoint2d


class BezierCurve:
    """Utility class for Bézier curve operations"""

    def __init__(self, control_points: np.ndarray):
        self.control_points = np.array(control_points)
        self.degree = len(control_points) - 1

    @staticmethod
    def _bernstein_polynomial(n: int, i: int, t: np.ndarray) -> np.ndarray:
        """Calculate value of Bernstein polynomial efficiently"""
        from math import comb

        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

    def evaluate(self, t: float) -> np.ndarray:
        """Evaluate curve at parameter t using Bernstein polynomials"""
        n = self.degree
        point = np.zeros(2)

        # Use Bernstein polynomials for evaluation
        for i in range(n + 1):
            coef = self._bernstein_polynomial(n, i, t)
            point += self.control_points[i] * coef

        return point

    def evaluate_points(self, t_values: np.ndarray) -> np.ndarray:
        """Evaluate curve at multiple t values efficiently"""
        points = np.zeros((len(t_values), 2))
        for i in range(self.degree + 1):
            coefs = np.array([self._bernstein_polynomial(self.degree, i, t) for t in t_values])
            points += np.outer(coefs, self.control_points[i])
        return points

    def estimate_length(self, num_samples: int = 20) -> float:
        """Estimate curve length with fewer samples for efficiency"""
        t_values = np.linspace(0, 1, num_samples)
        points = np.array([self.evaluate(t) for t in t_values])
        diffs = np.diff(points, axis=0)
        return float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))


class CenterlineProcessor:
    """
    Processes centerline points from lanelets and manages trajectory windows.

    The CenterlineProcessor transforms raw lanelet centerline points into:
    1. Evenly spaced waypoints with consistent spacing
    2. Smoothed path geometry (especially when using Bézier interpolation)
    3. Trajectory windows with appropriate boundary information

    This class forms the foundation for trajectory planning by creating
    nicely structured path data that can be optimized by the trajectory
    optimizer and executed by the motion controller.

    Implementation details:
    - Supports multiple interpolation methods (linear, Bézier)
    - Creates consistent point spacing regardless of input point distribution
    - Tracks which lanelets are active for each trajectory window
    - Extracts boundary information to enforce spatial constraints
    """

    # Class constants to replace magic numbers
    MIN_CURVE_SAMPLES = 10  # Minimum number of samples for any curve
    CURVE_SAMPLING_FACTOR = 2.0  # Oversampling factor (2x the needed density)
    ENDPOINT_MIN_DISTANCE = (
        0.3  # Minimum distance factor (30% of target spacing) to include endpoint
    )
    MIN_CONTROL_POINTS = 3  # Minimum number of control points needed to fit a Bézier curve.
    FINAL_POINT_THRESHOLD = (
        0.5  # Distance threshold for including the final original point in linear interpolation
    )

    def __init__(self, node: Node):
        """
        Initialize with lanelet data

        Args:
            lanelet_dict: Dictionary containing lanelet information
            lanelet_ids: List of lanelet IDs in order
            config: Optional preprocessing configuration
        """
        self.node = node
        self.processed_points = self._process_centerline_points()
        self.node.get_logger().info(
            f"Processed {len(self.processed_points)} centerline points using {self.node.centerline_processor_config.method} method"
        )

    def _get_original_points(self) -> List[List[float]]:
        """Get all original centerline points in order, removing duplicates at lanelet connections"""
        centerline_points = []
        for i, lid in enumerate(self.node.lanelet_ids):
            inverted = self.node.is_inverted[i]
            lanelet = self.node.get_lanelet_by_id(lid, inverted)
            centerline = [[point.x, point.y] for point in lanelet.centerline]
            
            # For all but first lanelet, check if we need to skip the first point (duplicate)
            if centerline_points and i > 0:
                # If the first point of current lanelet is very close to last point of previous lanelet
                last_point = centerline_points[-1]
                first_point = centerline[0]
                
                # Use a small epsilon for floating point comparison
                if abs(last_point[0] - first_point[0]) < 1e-6 and abs(last_point[1] - first_point[1]) < 1e-6:
                    # Skip the first point of current lanelet to avoid duplication
                    centerline.pop(0)
            
            centerline_points.extend(centerline)
        return centerline_points

    def _get_distance(self, p1: List[float], p2: List[float]) -> float:
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def _process_points_linear(
        self, centerline_points: List[List[float]]
    ) -> Optional[List[List[float]]]:
        """
        Process centerline points using linear interpolation to create evenly spaced points.

        This function takes a sequence of points defining a path and generates a new sequence
        where points are evenly spaced at intervals defined by target_spacing. It uses linear
        interpolation between original points to maintain the path shape.

        Args:
            centerline_points: List of [x,y] coordinates defining the original path

        Returns:
            List of [x,y] coordinates with approximately even spacing
        """
        try:
            # --- PHASE 1: Calculate segment distances and cumulative distances ---
            segment_distances = []  # Store lengths of each segment between consecutive points
            cumulative_distances = [0.0]  # Store cumulative distance to reach each point
            total_distance = 0.0  # Total path length

            # Calculate distance metrics for each segment
            for i in range(len(centerline_points) - 1):
                # Calculate Euclidean distance between consecutive points
                dist = self._get_distance(centerline_points[i], centerline_points[i + 1])
                segment_distances.append(dist)
                total_distance += dist
                cumulative_distances.append(total_distance)

            # --- PHASE 2: Generate evenly spaced points along the path ---
            processed_points = []
            # Always include the first point from original path
            processed_points.append(centerline_points[0])

            # Start at first target distance
            target_distance = self.node.centerline_processor_config.target_spacing
            # Continue until we've covered the entire path
            while target_distance < total_distance:
                # --- Find the segment containing our target distance ---
                segment_idx = 0
                # Advance segment_idx until we find the segment containing target_distance
                # This works because cumulative_distances[segment_idx] is the distance at the START of segment
                # and cumulative_distances[segment_idx+1] is the distance at the END of segment
                while (
                    segment_idx < len(cumulative_distances) - 1
                    and cumulative_distances[segment_idx + 1] < target_distance
                ):
                    segment_idx += 1

                # --- Perform linear interpolation within the found segment ---
                # Get the points that define this segment
                p1 = centerline_points[segment_idx]  # Start point of segment
                p2 = centerline_points[segment_idx + 1]  # End point of segment

                # Calculate interpolation parameter t (0 to 1)
                # t represents how far along the segment our target point should be
                segment_start_distance = cumulative_distances[
                    segment_idx
                ]  # Distance to start of segment
                segment_length = segment_distances[segment_idx]  # Length of this segment
                t = (target_distance - segment_start_distance) / segment_length

                # Create the new interpolated point at exactly the target distance
                new_point = [
                    float(p1[0] + t * (p2[0] - p1[0])),  # Interpolated x-coordinate
                    float(p1[1] + t * (p2[1] - p1[1])),  # Interpolated y-coordinate
                ]
                processed_points.append(new_point)

                # Move to next target distance
                target_distance += self.node.centerline_processor_config.target_spacing

            # --- PHASE 3: Ensure final point is included if needed ---
            # Check if the last processed point is significantly different from the original end point
            # This ensures the path reaches the exact destination point
            if (
                self._get_distance(processed_points[-1], centerline_points[-1])
                > self.FINAL_POINT_THRESHOLD
            ):
                processed_points.append(centerline_points[-1])
            return processed_points
        except Exception as e:
            self.node.get_logger().error(f"Error processing linear points: {str(e)}")
            return None

    def _process_points_bezier(
        self, centerline_points: List[List[float]]
    ) -> Optional[List[List[float]]]:
        """
        Process centerline points using Bézier curves to create smooth, evenly spaced points.

        This function:
        1. Divides the path into overlapping windows
        2. Fits a Bézier curve to each window
        3. Samples points from these curves at approximately even spacing
        4. Ensures smooth transitions between curve segments

        The Bézier approach provides better path smoothing than linear interpolation,
        especially for paths with sharp turns or noisy points.

        Args:
            centerline_points: List of [x,y] coordinates defining the original path

        Returns:
            List of [x,y] coordinates that are smoothed and approximately evenly spaced
        """
        try:
            # Convert input to numpy array for efficient operations
            points_array = np.array(centerline_points)
            n_points = len(points_array)

            # Determine window size for Bézier fitting (limited by number of available points)
            window_size = min(self.node.centerline_processor_config.bezier_window_size, n_points)
            processed_points = []

            # --- Track accumulated distance for spacing control ---
            # total_arc_length: tracks accumulated distance since last added point
            # next_sample_distance: distance threshold for adding next point
            total_arc_length = 0.0
            next_sample_distance = 0.0

            # --- Set up sliding window parameters ---
            # We process points in overlapping windows to ensure continuity
            overlap = (
                self.node.centerline_processor_config.bezier_overlap
            )  # Number of points to overlap between windows
            step = window_size - overlap  # How many points to advance each window

            # --- Process each window of points ---
            for window_start in range(0, n_points - overlap, step):
                # Get the points for this window
                window_end = min(window_start + window_size, n_points)
                window_points = points_array[window_start:window_end]

                # --- Check if we have enough points for a meaningful curve ---
                # With fewer than MIN_CONTROL_POINTS (3), a Bézier curve can't
                # create meaningful curvature (with 2 points it's just a line)
                if len(window_points) < self.MIN_CONTROL_POINTS:
                    # Fallback: Process remaining points directly, maintaining spacing
                    for point in window_points:
                        if processed_points:
                            dist = np.linalg.norm(point - processed_points[-1])
                            # Only add point if it maintains minimum spacing
                            if dist >= self.node.centerline_processor_config.target_spacing:
                                processed_points.append(point)
                        else:
                            # Always add first point
                            processed_points.append(point)
                    continue

                # --- Create and sample Bézier curve ---
                # Create Bézier curve using control points from current window
                curve = BezierCurve(window_points)

                # Estimate how many samples we need based on curve length
                # Longer curves need more samples to maintain spacing accuracy
                curve_length = curve.estimate_length()

                # Ensure we have enough samples:
                # - At least MIN_CURVE_SAMPLES (10) for even very short curves
                # - For longer curves, use CURVE_SAMPLING_FACTOR (2.0) times the points needed
                #   to achieve target spacing (oversample to ensure accurate spacing)
                n_samples = max(
                    self.MIN_CURVE_SAMPLES,
                    int(
                        self.CURVE_SAMPLING_FACTOR
                        * curve_length
                        / self.node.centerline_processor_config.target_spacing
                    ),
                )

                # Generate evenly spaced sample points along parametric domain [0,1]
                t_values = np.linspace(0, 1, n_samples)
                # Evaluate curve at these parameter values
                candidate_points = curve.evaluate_points(t_values)

                # --- Process candidate points to achieve even spacing ---
                for point in candidate_points:
                    # Always include the first point
                    if not processed_points:
                        processed_points.append(point.tolist())
                        continue

                    # Calculate distance from last added point
                    dist = np.linalg.norm(point - processed_points[-1])
                    # Accumulate distance
                    total_arc_length += dist

                    # Add point if we've reached or exceeded target spacing
                    if total_arc_length >= next_sample_distance:
                        processed_points.append(point.tolist())
                        # Advance target distance
                        next_sample_distance += (
                            self.node.centerline_processor_config.target_spacing
                        )
                        # Reset accumulator (NOTE: A more accurate approach would be
                        # total_arc_length -= next_sample_distance to preserve remainder)
                        total_arc_length = 0

            # --- Ensure the original endpoint is included ---
            # Check if last processed point is far enough from original endpoint
            if len(processed_points) > 1:
                last_original = points_array[-1]  # Original end point
                last_processed = np.array(processed_points[-1])  # Last point we generated

                # If endpoint is significantly different (> ENDPOINT_MIN_DISTANCE * spacing)
                # then add it to ensure the path reaches the exact destination
                if (
                    np.linalg.norm(last_original - last_processed)
                    > self.ENDPOINT_MIN_DISTANCE
                    * self.node.centerline_processor_config.target_spacing
                ):
                    processed_points.append(last_original.tolist())

            return processed_points
        except Exception as e:
            self.node.get_logger().error(f"Error processing Bézier points: {str(e)}")
            return None

    def _process_centerline_points(self) -> List[List[float]]:
        """Process centerline points using selected method"""
        centerline_points = self._get_original_points()

        if self.config.method == "bezier":
            return self._process_points_bezier(centerline_points)
        else:  # default to linear
            return self._process_points_linear(centerline_points)

    def get_trajectory_window(
        self, window_start_idx: int, lookahead_points: int
    ) -> Optional[TrajectoryWindow]:
        """
        Get trajectory window and its boundaries efficiently using the Lanelet2 API.

        Args:
            window_start_idx: Starting index in processed points
            lookahead_points: Number of points to look ahead

        Returns:
            TrajectoryWindow containing points and boundaries, or None if window cannot be created
        """
        # Check if we have processed points
        if not self.processed_points:
            self.node.get_logger().error("Cannot create window: No processed points")
            return None

        # Extract window points
        end_idx = min(window_start_idx + lookahead_points, len(self.processed_points))
        if end_idx <= window_start_idx:
            self.node.get_logger().error(
                f"Invalid window indices: {window_start_idx} to {end_idx}"
            )
            return None

        window_points = self.processed_points[window_start_idx:end_idx]

        # Track which lanelets are active for this window
        # We use a set for O(1) lookups
        active_lanelet_ids = set()

        # First pass: determine active lanelets
        for point in window_points:
            lanelet2_point = BasicPoint2d(point[0], point[1])

            # Check each lanelet in our route
            for i, lid in enumerate(self.node.lanelet_ids):
                # Get lanelet with proper orientation
                inverted = self.node.is_inverted[i]
                lanelet = self.node.map_manager.get_lanelet_by_id(lid, inverted)

                if not lanelet:
                    continue

                # Check if lanelet contains point
                try:
                    if lanelet.contains(lanelet2_point):
                        active_lanelet_ids.add(lid)
                except Exception as e:
                    self.node.get_logger().debug(
                        f"Error checking if point is in lanelet {lid}: {str(e)}"
                    )

        # Convert to list maintaining original route order
        active_lanelets = [lid for lid in self.node.lanelet_ids if lid in active_lanelet_ids]

        # Extract boundaries in a single efficient pass
        left_boundaries = []
        right_boundaries = []

        for lid in active_lanelets:
            # Find the original index to get inversion status
            i = self.node.lanelet_ids.index(lid)
            inverted = self.node.is_inverted[i]

            # Get lanelet
            lanelet = self.node.map_manager.get_lanelet_by_id(lid, inverted)
            if not lanelet:
                continue

            # Extract boundary points
            # Convert BasicPoint3d objects to numpy arrays efficiently
            left_points = np.array([(p.x, p.y) for p in lanelet.leftBound])
            right_points = np.array([(p.x, p.y) for p in lanelet.rightBound])

            left_boundaries.append(left_points)
            right_boundaries.append(right_points)

        # Check if we found boundaries
        if not left_boundaries or not right_boundaries:
            self.node.get_logger().error("No valid boundaries found for active lanelets")
            return None

        # Combine all boundary segments into continuous boundary lines
        left_combined = np.vstack(left_boundaries)
        right_combined = np.vstack(right_boundaries)

        # Create trajectory window
        return TrajectoryWindow(
            points=window_points,
            left_boundary=left_combined,
            right_boundary=right_combined,
            start_idx=window_start_idx,
            active_lanelet_ids=active_lanelets,
        )
