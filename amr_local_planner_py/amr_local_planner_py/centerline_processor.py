from wsgiref import validate
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from shapely.geometry import Point, Polygon
import matplotlib.pyplot as plt

@dataclass
class PreprocessingConfig:
    """Configuration for centerline preprocessing"""
    method: str = "linear"  # "linear" or "bezier"
    target_spacing: float = 2.0  # Target spacing between processed points
    bezier_window_size: int = 6  # Window size for bezier method
    bezier_overlap: int = 2  # Number of points to overlap between bezier windows
    spacing_tolerance: float = 0.1  # Tolerance for point spacing adjustment

@dataclass
class TrajectoryWindow:
    """Contains data for a trajectory window"""
    points: List[List[float]]
    left_boundary: np.ndarray
    right_boundary: np.ndarray
    start_idx: int
    active_lanelet_ids: List[int]

class BezierCurve:
    """Utility class for Bézier curve operations"""
    def __init__(self, control_points: np.ndarray):
        self.control_points = np.array(control_points)
        self.degree = len(control_points) - 1

    @staticmethod
    def _bernstein_polynomial(n: int, i: int, t: np.ndarray) -> np.ndarray:
        """Calculate value of Bernstein polynomial efficiently"""
        from math import comb
        return comb(n, i) * (t**i) * ((1-t)**(n-i))

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
        return float(np.sum(np.sqrt(np.sum(diffs**2, axis=1))))

class CenterlineProcessor:
    """Processes centerline points and manages trajectory windows"""
    
    def __init__(self, lanelet_dict: Dict, lanelet_ids: List[int], 
                 config: Optional[PreprocessingConfig] = None):
        """
        Initialize with lanelet data
        
        Args:
            lanelet_dict: Dictionary containing lanelet information
            lanelet_ids: List of lanelet IDs in order
            config: Optional preprocessing configuration
        """
        self.lanelet_dict = lanelet_dict
        self.lanelet_ids = lanelet_ids
        self.config = config if config else PreprocessingConfig()
        self.processed_points = self._process_centerline_points()
        self.original_points = self._get_original_points()
        print(f"Processed {len(self.processed_points)} centerline points using {self.config.method} method")
        
    def _get_original_points(self) -> List[List[float]]:
        """Get all original centerline points in order"""
        centerline_points = []
        for lid in self.lanelet_ids:
            if centerline_points and self.lanelet_dict[lid]["centerline_points"][0] == centerline_points[-1]:
                centerline_points.extend(self.lanelet_dict[lid]["centerline_points"][1:])
            else:
                centerline_points.extend(self.lanelet_dict[lid]["centerline_points"])
        return centerline_points

    def _get_distance(self, p1: List[float], p2: List[float]) -> float:
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    
    def _interpolate_point(self, p1: List[float], p2: List[float], distance: float) -> List[float]:
        """Linear interpolation between two points at specified distance from p1"""
        total_dist = self._get_distance(p1, p2)
        if total_dist < 1e-6:
            return p1
        
        t = distance / total_dist
        return [
            p1[0] + t * (p2[0] - p1[0]),
            p1[1] + t * (p2[1] - p1[1])
        ]
    
    def _process_points_linear(self, centerline_points: List[List[float]]) -> List[List[float]]:
        """Process centerline points using linear interpolation"""
        # Calculate distances
        segment_distances = []
        cumulative_distances = [0.0]
        total_distance = 0.0
        
        for i in range(len(centerline_points)-1):
            dist = self._get_distance(centerline_points[i], centerline_points[i+1])
            segment_distances.append(dist)
            total_distance += dist
            cumulative_distances.append(total_distance)
        
        # Create points at target spacing intervals
        processed_points = []
        processed_points.append(centerline_points[0])
        
        target_distance = self.config.target_spacing
        while target_distance < total_distance:
            # Find segment for this target distance
            segment_idx = 0
            while segment_idx < len(cumulative_distances)-1 and cumulative_distances[segment_idx+1] < target_distance:
                segment_idx += 1
            
            # Get bounding points
            p1 = centerline_points[segment_idx]
            p2 = centerline_points[segment_idx + 1]
            
            # Interpolate
            segment_start_distance = cumulative_distances[segment_idx]
            segment_length = segment_distances[segment_idx]
            t = (target_distance - segment_start_distance) / segment_length
            
            new_point = [
                float(p1[0] + t * (p2[0] - p1[0])),
                float(p1[1] + t * (p2[1] - p1[1]))
            ]
            processed_points.append(new_point)
            target_distance += self.config.target_spacing
        
        # Add final point if needed
        if self._get_distance(processed_points[-1], centerline_points[-1]) > 0.5:
            processed_points.append(centerline_points[-1])
            
        return processed_points
    
    def _fit_quintic_bezier(self, points: List[List[float]]) -> BezierCurve:
        """Fit quintic Bézier curve to points"""
        points = np.array(points)
        degree = min(len(points) - 1, 5)
        control_points = points[:degree + 1]
        return BezierCurve(control_points)

    def _adjust_point_spacing(self, points: List[List[float]]) -> List[List[float]]:
        """Adjust point spacing to be closer to target"""
        adjusted_points = [points[0]]
        
        for i in range(1, len(points)):
            prev = np.array(adjusted_points[-1])
            current = np.array(points[i])
            
            distance = np.linalg.norm(current - prev)
            if abs(distance - self.config.target_spacing) <= self.config.spacing_tolerance:
                adjusted_points.append(points[i])
            elif distance > self.config.target_spacing:
                direction = (current - prev) / distance
                new_point = prev + direction * self.config.target_spacing
                adjusted_points.append(new_point.tolist())
                
        return adjusted_points
    
    def _adjust_spacing(self, points: np.ndarray) -> List[List[float]]:
        """Enforce consistent spacing between points"""
        if len(points) < 2:
            return points.tolist()
            
        adjusted_points = [points[0].tolist()]
        total_dist = 0.0
        
        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i-1])
            total_dist += dist
            
            if total_dist >= self.config.target_spacing:
                adjusted_points.append(points[i].tolist())
                total_dist = 0.0
                
        # Always include last point if not too close
        if len(adjusted_points) > 1 and np.linalg.norm(points[-1] - adjusted_points[-1]) > 0.3:
            adjusted_points.append(points[-1].tolist())
            
        return adjusted_points

    # def _process_points_bezier(self, centerline_points: List[List[float]]) -> List[List[float]]:
    #     """Process centerline points using Bézier curves"""
    #     processed_points = []
    #     n_points = len(centerline_points)
    #     window_size = min(6, n_points)
    #     current_idx = 0
        
    #     # Convert to numpy array for faster operations
    #     points_array = np.array(centerline_points)
        
    #     while current_idx < n_points:
    #         # Get window points
    #         end_idx = min(current_idx + window_size, n_points)
    #         if end_idx - current_idx < 3:  # Need at least 3 points
    #             # Add remaining points directly
    #             remaining_points = points_array[current_idx:].tolist()
    #             if processed_points:
    #                 processed_points.extend(remaining_points)
    #             break
            
    #         # Get window points with overlap handling
    #         window_points = points_array[current_idx:end_idx]
            
    #         # Simple length approximation
    #         approx_length = np.sum(np.linalg.norm(np.diff(window_points, axis=0), axis=1))
    #         n_samples = max(3, int(approx_length / self.config.target_spacing))
            
    #         # Fit curve and sample points
    #         curve = BezierCurve(window_points)
    #         t_values = np.linspace(0, 1, n_samples)
    #         sampled_points = [curve.evaluate(t) for t in t_values]
            
    #         # Adjust point spacing
    #         adjusted_points = self._adjust_spacing(sampled_points)
            
    #         # Add points with overlap handling
    #         if current_idx == 0:
    #             processed_points.extend(sampled_points)
    #         else:
    #             # Skip first point to avoid duplicates
    #             processed_points.extend(sampled_points[1:])
            
    #         # Move window forward
    #         current_idx = end_idx - 1  # Overlap by 1 point
            
    #     return processed_points
    
    def _process_points_bezier(self, centerline_points: List[List[float]]) -> List[List[float]]:
        """Process centerline points using Bézier curves with proper overlap handling"""
        points_array = np.array(centerline_points)
        n_points = len(points_array)
        window_size = min(6, n_points)
        processed_points = []
        
        # Variables to track global arc length
        total_arc_length = 0.0
        next_sample_distance = 0.0
        
        # Moving window parameters
        overlap = 2  # Number of points to overlap
        step = window_size - overlap
        
        for window_start in range(0, n_points - overlap, step):
            # Get window points
            window_end = min(window_start + window_size, n_points)
            window_points = points_array[window_start:window_end]
            
            if len(window_points) < 3:  # Need at least 3 points for curve
                # Add remaining points directly with spacing check
                for point in window_points:
                    if processed_points:
                        dist = np.linalg.norm(point - processed_points[-1])
                        if dist >= self.config.target_spacing:
                            processed_points.append(point)
                    else:
                        processed_points.append(point)
                continue
                
            # Create Bézier curve for window
            curve = BezierCurve(window_points)
            
            # Estimate curve length for sampling
            curve_length = curve.estimate_length()
            n_samples = max(10, int(2 * curve_length / self.config.target_spacing))
            
            # Generate candidate points with finer sampling
            t_values = np.linspace(0, 1, n_samples)
            candidate_points = curve.evaluate_points(t_values)
            
            # Process candidate points maintaining global spacing
            for point in candidate_points:
                if not processed_points:
                    processed_points.append(point.tolist())
                    continue
                    
                # Calculate distance from last added point
                dist = np.linalg.norm(point - processed_points[-1])
                total_arc_length += dist
                
                # Add point if we've reached target spacing
                if total_arc_length >= next_sample_distance:
                    processed_points.append(point.tolist())
                    next_sample_distance += self.config.target_spacing
                    total_arc_length = 0  # Reset accumulator
        
        # Ensure we include the last point if it's far enough
        if len(processed_points) > 1:
            last_original = points_array[-1]
            last_processed = np.array(processed_points[-1])
            if np.linalg.norm(last_original - last_processed) > 0.3 * self.config.target_spacing:
                processed_points.append(last_original.tolist())
        
        return processed_points
    
    def validate_point_spacing(self, processed_points: List[List[float]]) -> Dict:
        """Validate spacing between processed points"""
        points = np.array(processed_points)
        distances = []
        
        for i in range(len(points)-1):
            dist = np.linalg.norm(points[i+1] - points[i])
            distances.append(dist)
            
        return {
            'mean_spacing': np.mean(distances),
            'std_spacing': np.std(distances),
            'max_spacing': max(distances),
            'min_spacing': min(distances),
            'total_points': len(points),
            'total_length': sum(distances)
        }
            
    def _process_centerline_points(self) -> List[List[float]]:
        """Process centerline points using selected method"""
        centerline_points = self._get_original_points()
        
        if self.config.method == "bezier":
            return self._process_points_bezier(centerline_points)
        else:  # default to linear
            return self._process_points_linear(centerline_points)
            
    def _find_point_lanelet(self, point: List[float]) -> List[int]:
        """
        Find which lanelets contain the given point, maintaining order based on lanelet_ids.
        
        Returns:
            List[int]: List of lanelet IDs that contain the point, in original lanelet order
        """
        active_lanelets = []
        point = Point(point[0], point[1])
        
        for lid in self.lanelet_ids:
            lanelet = self.lanelet_dict[lid]
            left_bound = lanelet["left_boundary"]
            right_bound = lanelet["right_boundary"]
            
            polygon_points = np.vstack([
                [left_bound[0]],
                right_bound,
                np.flip(left_bound[1:], axis=0),
                [left_bound[0]]
            ])
            
            lanelet_polygon = Polygon(polygon_points)
            if lanelet_polygon.contains(point):
                if lid not in active_lanelets:
                    active_lanelets.append(lid)
                    
        return active_lanelets
    
    def _combine_active_lanelets(self, point_lanelets: List[List[int]]) -> List[int]:
        """
        Combine lists of active lanelets while maintaining order and removing duplicates.
        
        Args:
            point_lanelets: List of lanelet ID lists, one per point
            
        Returns:
            List[int]: Combined list of lanelet IDs in order of first appearance
        """
        seen = set()
        combined = []
        for lanelets in point_lanelets:
            for lid in lanelets:
                if lid not in seen:
                    seen.add(lid)
                    combined.append(lid)
        return combined
    
    def _get_window_bounds(self, active_lanelets: List[int]) -> Tuple[np.ndarray, np.ndarray]:
        """Get combined left and right boundaries for active lanelets"""
        left_bounds = []
        right_bounds = []
        
        for lid in active_lanelets:
            left_bounds.append(self.lanelet_dict[lid]["left_boundary"])
            right_bounds.append(self.lanelet_dict[lid]["right_boundary"])
        
        left_combined = np.vstack(left_bounds)
        right_combined = np.vstack(right_bounds)
        
        return left_combined, right_combined
    
    def get_trajectory_window(self, window_start_idx: int, lookahead_points: int) -> TrajectoryWindow:
        """
        Get trajectory window and its boundaries.
        
        Args:
            window_start_idx: Starting index in processed points
            lookahead_points: Number of points to look ahead
            
        Returns:
            TrajectoryWindow containing points and boundaries
        """
        # Get window points
        end_idx = min(window_start_idx + lookahead_points, len(self.processed_points))
        window_points = self.processed_points[window_start_idx:end_idx]
        
        # Find active lanelets for each point
        point_lanelets = [self._find_point_lanelet(point) for point in window_points]
        
        # Combine while maintaining order
        active_lanelets = self._combine_active_lanelets(point_lanelets)
        
        # Get boundaries
        left_bounds, right_bounds = self._get_window_bounds(active_lanelets)
        
        return TrajectoryWindow(
            points=window_points,
            left_boundary=left_bounds,
            right_boundary=right_bounds,
            start_idx=window_start_idx,
            active_lanelet_ids=active_lanelets
        )
    
    def visualize_window(self, window: TrajectoryWindow):
        """Visualize the trajectory window with both original and processed points"""
        plt.figure(figsize=(15, 10))
        
        # Plot all lanelet boundaries
        for lid in self.lanelet_ids:
            left_bound = self.lanelet_dict[lid]["left_boundary"]
            right_bound = self.lanelet_dict[lid]["right_boundary"]
            
            polygon_points = np.vstack([
                [left_bound[0]],
                right_bound,
                np.flip(left_bound[1:], axis=0),
                [left_bound[0]]
            ])
            plt.plot(polygon_points[:, 0], polygon_points[:, 1], 'lightgray', linewidth=1)
            
            # Add lanelet ID
            centroid_x = np.mean(polygon_points[:, 0])
            centroid_y = np.mean(polygon_points[:, 1])
            plt.annotate(f'{lid}', (centroid_x, centroid_y), ha='center', va='center', 
                        fontsize=8, color='black')
        
        # Plot active boundaries
        plt.plot(window.left_boundary[:, 0], window.left_boundary[:, 1], 'k-', 
                linewidth=2, label='Active Boundaries')
        plt.plot(window.right_boundary[:, 0], window.right_boundary[:, 1], 'k-', 
                linewidth=2)
        
        # Plot original centerline points
        original_points_arr = np.array(self.original_points)
        plt.plot(original_points_arr[:, 0], original_points_arr[:, 1], 'g.', 
                markersize=8, label='Original Points')
        
        # Plot all processed points
        processed_points_arr = np.array(self.processed_points)
        plt.plot(processed_points_arr[:, 0], processed_points_arr[:, 1], 'b.', 
                markersize=4, label='Processed Points')
        
        # Plot window points
        window_points_arr = np.array(window.points)
        plt.plot(window_points_arr[:, 0], window_points_arr[:, 1], 'r.', 
                markersize=10, label='Window Points')
        
        # Number window points
        for i, point in enumerate(window.points):
            plt.annotate(f'{i}', (point[0], point[1]), xytext=(5, 5), 
                        textcoords='offset points')
        
        # Show window start
        start_point = self.processed_points[window.start_idx]
        plt.plot(start_point[0], start_point[1], 'g*', markersize=15, 
                label='Window Start')
        
        # Add legend indicating active lanelets
        plt.figtext(0.02, 0.02, f'Active Lanelets (in order): {window.active_lanelet_ids}',
                   fontsize=10, bbox=dict(facecolor='white', alpha=0.8))
        
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        plt.title(f'Trajectory Window Starting at Index {window.start_idx}\nPreprocessing Method: {self.config.method}')
        plt.show()

    def visualize_preprocessing(self):
        """Visualize preprocessing results with smooth curves"""
        plt.figure(figsize=(20, 10))
        
        # Plot lanelet boundaries
        for lid in self.lanelet_ids:
            left_bound = self.lanelet_dict[lid]["left_boundary"]
            right_bound = self.lanelet_dict[lid]["right_boundary"]
            
            # Plot boundaries in light gray
            plt.plot(left_bound[:, 0], left_bound[:, 1], 'gray', linewidth=1, alpha=0.3)
            plt.plot(right_bound[:, 0], right_bound[:, 1], 'gray', linewidth=1, alpha=0.3)
            
            # Add lanelet ID
            center = np.mean(self.lanelet_dict[lid]["centerline_points"], axis=0)
            plt.annotate(f'L{lid}', (center[0], center[1]), ha='center', va='center')
        
        # Convert points to numpy arrays
        original = np.array(self.original_points)
        processed = np.array(self.processed_points)
        
        # Plot original points and their connections
        plt.plot(original[:, 0], original[:, 1], 'g.-', alpha=0.3,
                label='Original Points', markersize=6, linewidth=1)
        
        # Plot processed points differently based on method
        if self.config.method == "bezier":
            # Plot points
            plt.plot(processed[:, 0], processed[:, 1], 'b.', 
                    label='Processed Points', markersize=4)
            
            # Add dense sampling for smoother curve visualization
            curve_points = []
            window_size = 5
            
            for i in range(0, len(processed) - window_size + 1, window_size - 1):
                window = processed[i:i+window_size]
                if len(window) < 3:
                    continue
                    
                curve = BezierCurve(window)
                t_values = np.linspace(0, 1, 50)
                points = curve.evaluate_points(t_values)
                curve_points.extend(points.tolist())
            
            curve_points = np.array(curve_points)
            plt.plot(curve_points[:, 0], curve_points[:, 1], 'b-', 
                    label='Bezier Curve', linewidth=1.5)
        else:
            # For linear method, just connect points
            plt.plot(processed[:, 0], processed[:, 1], 'b.-',
                    label='Processed Points', markersize=4, linewidth=1.5)
        
        # Add stats box
        stats_text = (
            f"Original points: {len(original)}\n"
            f"Processed points: {len(processed)}\n"
            f"Target spacing: {self.config.target_spacing}m\n"
            f"Method: {self.config.method}"
        )
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
                verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))
        
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        plt.title(f'Centerline Preprocessing - {self.config.method} method')
        plt.show()
        
        
def visualize_preprocessing_comparison(processor: CenterlineProcessor):
    """
    Visualize preprocessing results in both subplot and overlay formats
    """
    # Create figure with two visualization styles
    fig = plt.figure(figsize=(20, 10))
    
    # 1. Subplot style
    ax1 = fig.add_subplot(121)
    ax1.set_title(f'Subplot Comparison\nPreprocessing Method: {processor.config.method}')
    
    # Plot boundaries
    for lid in processor.lanelet_ids:
        left_bound = processor.lanelet_dict[lid]["left_boundary"]
        right_bound = processor.lanelet_dict[lid]["right_boundary"]
        
        ax1.plot(left_bound[:, 0], left_bound[:, 1], 'k-', linewidth=1, alpha=0.5)
        ax1.plot(right_bound[:, 0], right_bound[:, 1], 'k-', linewidth=1, alpha=0.5)
        
        # Add lanelet ID
        center = np.mean(processor.lanelet_dict[lid]["centerline_points"], axis=0)
        ax1.annotate(f'L{lid}', (center[0], center[1]), ha='center', va='center')
    
    # Original points and connections
    original = np.array(processor.original_points)
    ax1.plot(original[:, 0], original[:, 1], 'g.-', label='Original Points',
            markersize=8, linewidth=1)
    
    # Processed points and connections
    processed = np.array(processor.processed_points)
    ax1.plot(processed[:, 0], processed[:, 1], 'b.-', label='Processed Points',
            markersize=4, linewidth=1)
    
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')
    
    # 2. Overlay style
    ax2 = fig.add_subplot(122)
    ax2.set_title(f'Overlay Comparison\nPreprocessing Method: {processor.config.method}')
    
    # Plot boundaries
    for lid in processor.lanelet_ids:
        left_bound = processor.lanelet_dict[lid]["left_boundary"]
        right_bound = processor.lanelet_dict[lid]["right_boundary"]
        
        ax2.plot(left_bound[:, 0], left_bound[:, 1], 'k-', linewidth=1, alpha=0.5)
        ax2.plot(right_bound[:, 0], right_bound[:, 1], 'k-', linewidth=1, alpha=0.5)
        
        # Add lanelet ID
        center = np.mean(processor.lanelet_dict[lid]["centerline_points"], axis=0)
        ax2.annotate(f'L{lid}', (center[0], center[1]), ha='center', va='center')
    
    # Plot original points and connections with lighter color
    ax2.plot(original[:, 0], original[:, 1], 'g.-', alpha=0.3, 
            label='Original Points', markersize=8, linewidth=1)
    
    # Plot processed points and connections with stronger color
    ax2.plot(processed[:, 0], processed[:, 1], 'b.-', 
            label='Processed Points', markersize=4, linewidth=1.5)
    
    # Add stats box
    stats_text = (
        f"Original points: {len(original)}\n"
        f"Processed points: {len(processed)}\n"
        f"Target spacing: {processor.config.target_spacing}m\n"
        f"Method: {processor.config.method}"
    )
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes, 
            verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))
    
    ax2.grid(True)
    ax2.legend()
    ax2.axis('equal')
    
    plt.tight_layout()
    plt.show()
        
def test_centerline_processor(lanelet_dict: Dict = None, 
                        lanelet_ids: List[int] = None,
                        method: str = "linear"):
    """
    Test centerline processor with either provided or generated test data
    
    Args:
        lanelet_dict: Optional dictionary of lanelet data
        lanelet_ids: Optional list of lanelet IDs
        method: Preprocessing method ("linear" or "bezier")
    """
    
    # Create processor with specified method
    config = PreprocessingConfig(
        method=method,
        target_spacing=1.0,
        bezier_window_size=6,
        bezier_overlap=2
    )
    
    processor = CenterlineProcessor(lanelet_dict, lanelet_ids, config)
    
    validate_results = processor.validate_point_spacing(processor.processed_points)
    print("\nValidation results:")
    for key, value in validate_results.items():
        print(f"{key}: {value}")
    
    # Visualize preprocessing results
    visualize_preprocessing_comparison(processor)
    
    # Test window generation
    print("\nTesting window generation...")
    window = processor.get_trajectory_window(0, 6)
    processor.visualize_window(window)
    
    return processor

# if __name__ == "__main__":
    
#     # Example custom data
#     lanelet_dict = {
#         -6995: {
#             "length": 1.87,
#             "centerline_points": [
#                 [19.0998, 4.24597], [17.6539, 5.35312]
#             ],
#             "left_boundary": np.array([ [18.504, 3.38053], [16.979, 4.46012] ]),
#             "right_boundary": np.array([ [19.7237, 5.15215], [18.3057, 6.21025] ])
#         },
#         -7019: {
#             "length": 3.19,
#             "centerline_points": [
#                 [17.6539, 5.35312], [16.9326, 5.90554], [16.2295, 6.43471], [15.7666, 6.79154], [15.1116, 7.28624]
#             ],
#             "left_boundary": np.array([ [16.979, 4.46012], [14.454, 6.4172] ]),
#             "right_boundary": np.array([ [18.3057, 6.21025], [15.7474, 8.06041] ])
#         },
#         -6996: {
#             "length": 3.06,
#             "centerline_points": [
#                 [15.1116, 7.28624], [14.4443, 7.80432], [13.7414, 8.32168], [12.8583, 8.97716]
#             ],
#             "left_boundary": np.array([ [14.454, 6.4172], [13.5285, 7.11906], [12.736, 7.68188], [11.991, 8.23383] ]),
#             "right_boundary": np.array([ [15.7474, 8.06041], [14.6891, 8.88951], [14.0755, 9.38503], [13.389, 9.97357] ])
#         },
#         -6999: {
#             "length": 5.42,
#             "centerline_points": [
#                 [12.8583, 8.97716], [12.2636, 9.41402], [11.6704, 9.77417], [11.2832, 9.89637], [10.5473, 9.99393], [9.97879, 9.94867], [9.35417, 9.864], [8.4243, 9.5917], [7.62356, 9.37511], [6.6296, 8.75324], [5.98117, 8.02015], [5.65965, 7.51788], [5.36672, 7.0406]
#             ],
#             "left_boundary": np.array([ [11.991, 8.23383], [11.7149, 8.42907], [11.2115, 8.73787], [10.7119, 8.85784], [10.3261, 8.61404], [10.0488, 8.26614], [9.81941, 7.89558], [9.50831, 7.46436], [9.17449, 6.98546], [8.78196, 6.48178], [8.5516, 6.15843], [8.25141, 5.77465] ]),
            
#             "right_boundary": np.array([ [13.389, 9.97357], [13.0033, 10.3201], [12.5475, 10.6181], [12.2008, 10.7883], [11.995, 10.872], [11.5086, 11.0697], [11.0918, 11.1913], [10.2724, 11.3168], [9.703, 11.3763], [9.1809, 11.4368], [8.44671, 11.4458], [7.78494, 11.3855], [7.06586, 11.2295], [6.52384, 11.1007], [6.12499, 10.9157], [5.80921, 10.7205], [5.4004, 10.4409], [5.09669, 10.2341], [4.66492, 9.91857], [4.32653, 9.66981], [4.04825, 9.37502], [3.79223, 9.15151], [3.43345, 8.80946] ])
#         },
#         -6985: {
#             "length": 20.10,
#             "centerline_points": [
#                 [5.36672, 7.0406], [4.8454, 6.19118], [3.2809, 3.89322], [2.01286, 2.16787], [0.723955, 0.300428], [-1.12082, -2.192], [-3.33258, -5.28206], [-4.4865, -6.79262], [-6.01396, -8.57034]
#             ],
#             "left_boundary": np.array([ [8.25141, 5.77465], [-3.60881, -10.4589] ]),
#             "right_boundary": np.array([ [3.43345, 8.80946], [-8.4099, -7.15374] ])
#         },
#         -7011: {
#             "length": 17.30,
#             "centerline_points": [
#                 [-6.01396, -8.57034], [-7.19296, -10.0106], [-8.23884, -10.9995], [-9.38688, -11.6126], [-11.1503, -12.2379], [-11.944, -12.4708], [-12.8729, -12.5318], [-13.6901, -12.56], [-15.1697, -12.5302], [-16.9741, -12.2826]
#             ],
#             "left_boundary": np.array([ [-3.60881, -10.4589], [-4.86529, -11.5701], [-5.72838, -12.2248], [-6.78315, -12.7653], [-7.97994, -13.3085], [-9.34334, -13.8078], [-10.7096, -14.1655], [-11.8888, -14.4014], [-13.2611, -14.4523], [-14.0646, -14.5272], [-15.2951, -14.5634], [-16.2532, -14.6061], [-17.2705, -14.6499], [-18.1228, -14.655], [-18.6563, -14.6183], [-19.3209, -14.537], [-19.843, -14.4765] ]),
#             "right_boundary": np.array([ [-8.4099, -7.15374], [-9.97814, -9.26286], [-11.1396, -9.79368], [-12.6838, -10.1313], [-13.5649, -10.1724], [-14.6982, -10.0414] ])
#         },
#         -7010: {
#             "length": 9.57,
#             "centerline_points": [
#                 [-16.9741, -12.2826], [-17.8822, -11.8637], [-18.5469, -11.1803], [-18.9396, -10.4797], [-19.1558, -9.72833], [-19.1452, -9.06693], [-19.088, -8.69983], [-19.0182, -8.30878], [-18.876, -7.71561], [-18.7455, -7.32025], [-18.5185, -6.94107], [-18.247, -6.59919], [-18.1226, -6.44341]
#             ],
#             "left_boundary": np.array([ [-19.843, -14.4765], [-20.0488, -14.2445], [-20.6685, -13.4421], [-21.0998, -12.5887], [-21.2747, -11.5296], [-21.1634, -10.5769], [-21.001, -9.81215], [-20.8453, -9.30137], [-20.4942, -8.50334], [-20.1515, -7.87078], [-19.8544, -7.33357], [-19.5098, -6.80134], [-19.2798, -6.46028], [-19.0833, -6.22024], [-18.8617, -5.91098] ]),
#             "right_boundary": np.array([ [-14.6982, -10.0414], [-15.5572, -9.70419], [-16.1924, -9.31535], [-16.589, -9.01624], [-16.7945, -8.80112], [-16.8765, -8.71528], [-17.0159, -8.56936], [-17.1365, -8.44306], [-17.2571, -8.31679], [-17.3749, -8.19354], [-17.5994, -7.91463], [-17.6912, -7.76296], [-17.695, -7.57412], [-17.6326, -7.4371], [-17.5761, -7.3002], [-17.3544, -6.93731] ])
#         },
#         -6994: {
#             "length": 11.06,
#             "centerline_points": [
#                 [-18.1226, -6.44341], [-17.2651, -5.3695], [-16.446, -4.29063], [-15.6737, -3.23631], [-14.7634, -1.97852], [-13.8303, -0.673066], [-12.8511, 0.692344], [-11.9415, 1.98553], [-11.8437, 2.12914], [-11.7293, 2.32918], [-11.6577, 2.4633]
#             ],
#             "left_boundary": np.array([ [-18.8617, -5.91098], [-12.341, 3.02842] ]), # Original left boundary: [-12.341, 3.02842], [-18.8617, -5.91098]
#             "right_boundary": np.array([ [-17.3544, -6.93731], [-10.9541, 1.74536] ]) # Original right boundary: [-10.9541, 1.74536], [-17.3544, -6.93731]
#         },
#         -7001: {
#             "length": 2.42,
#             "centerline_points": [
#                 [-11.6577, 2.4633], [-11.4953, 2.76773], [-11.3558, 3.19554], [-11.2719, 3.74032], [-11.2616, 3.97076], [-11.2662, 4.2009], [-11.3436, 4.37057], [-11.4012, 4.59375], [-11.4799, 4.82833], [-11.5845, 5.02698], [-11.7932, 5.25602], [-12.0189, 5.44046]
#             ],
#             "left_boundary": np.array([ [-12.341, 3.02842], [-11.943, 3.85099], [-11.7971, 4.25532], [-11.8741, 4.55781], [-12.3797, 5.12927] ]),
#             "right_boundary": np.array([ [-10.9541, 1.74536], [-10.74, 2.29273], [-10.6227, 2.94443], [-10.6393, 3.78239], [-10.7374, 4.25272], [-10.9583, 4.94495], [-11.3114, 5.43384], [-11.8656, 5.91873] ])
#         },
#         -6993: {
#             "length": 1.82,
#             "centerline_points": [
#                 [-12.0189, 5.44046], [-12.2932, 5.69477], [-12.5905, 5.91025], [-12.8963, 6.1049], [-13.1033, 6.24543], [-13.3877, 6.40804], [-13.6214, 6.57407]
#             ],
#             "left_boundary": np.array([ [-12.3797, 5.12927], [-12.6472, 5.3365], [-13.32, 5.83085], [-13.8598, 6.18616] ]),
#             "right_boundary": np.array([ [-11.8656, 5.91873], [-12.3102, 6.25231], [-12.7781, 6.56182], [-13.3896, 6.95113] ])
#         }
        
#     }


#     lanelet_ids = [-6995, -7019, -6996, -6999, -6985, -7011, -7010, -6994, -7001, -6993]
    
#     processor_custom = test_centerline_processor(
#         lanelet_dict=lanelet_dict,
#         lanelet_ids=lanelet_ids,
#         method="bezier"
#     )
    