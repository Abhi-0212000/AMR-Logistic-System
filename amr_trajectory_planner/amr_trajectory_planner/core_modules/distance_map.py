import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points
from typing import Tuple, Optional
from rclpy.node import Node

class DistanceMapGenerator:
    """Class for generating and managing distance maps."""

    def __init__(self, node: Node):
        """Initialize the distance map generator with configuration."""
        self.node = node
        self.config = self.node.distance_map_config

        """
            Original kernels (from paper):

            sobel_x = [              sobel_y = [
            [-1, -2,  0,  2,  1],    [-1, -4, -6, -4, -1],
            [-4, -8,  0,  8,  4],    [-2, -8,-12, -8, -2],
            [-6,-12,  0, 12,  6],    [ 0,  0,  0,  0,  0],
            [-4, -8,  0,  8,  4],    [ 2,  8, 12,  8,  2],
            [-1, -2,  0,  2,  1]     [ 1,  4,  6,  4,  1]
            ]                        ]
            
            Flipped kernels (in code):
            sobel_x = [              sobel_y = [
            [ 1,  2,  0, -2, -1],    [ 1,  4,  6,  4,  1],
            [ 4,  8,  0, -8, -4],    [ 2,  8, 12,  8,  2],
            [ 6, 12,  0,-12, -6],    [ 0,  0,  0,  0,  0],
            [ 4,  8,  0, -8, -4],    [-2, -8,-12, -8, -2],
            [ 1,  2,  0, -2, -1]     [-1, -4, -6, -4, -1]
            ]                        ]
            
            Gradient Direction:
                The original kernels measure gradient from right-to-left (x) and bottom-to-top (y)
                The flipped kernels measure gradient from left-to-right (x) and top-to-bottom (y)
                This is just a convention choice - both will detect edges, just with opposite signs
                
            Why Flip?:
                In most image processing contexts, we typically measure:
                    x-gradient from left-to-right (positive x direction)
                    y-gradient from top-to-bottom (positive y direction)
                This matches the conventional coordinate system where:
                    x increases from left to right
                    y increases from top to bottom
                
            Effect on Results:
                Using original kernels: positive gradient where values decrease
                Using flipped kernels: positive gradient where values increase
                The magnitude of the gradients remains the same
                Only the sign (direction) changes
            
            # For original kernels (right-to-left, bottom-to-top):
            gradient_angles = np.where(
                gradient_y >= 0,
                np.arctan2(-gradient_y, -gradient_x),  # Note the negatives
                np.arctan2(-gradient_y, -gradient_x) + np.pi
            )

            # For flipped kernels (left-to-right, top-to-bottom):
            gradient_angles = np.where(
                gradient_y >= 0,
                np.arctan2(gradient_y, gradient_x),    # No negatives needed
                np.arctan2(gradient_y, gradient_x) + np.pi
            )
        """
        
        # Sobel kernels for gradient calculation
        self.sobel_x = np.array([
            [ 1,  2,  0,  -2, -1],
            [ 4,  8,  0,  -8, -4],
            [ 6, 12,  0, -12, -6],
            [ 4,  8,  0,  -8, -4],
            [ 1,  2,  0,  -2, -1]
        ])
        
        self.sobel_y = np.array([
            [ 1,  4,  6,  4,  1],
            [ 2,  8, 12,  8,  2],
            [ 0,  0,  0,  0,  0],
            [-2, -8,-12, -8, -2],
            [-1, -4, -6, -4, -1]
        ])
        

    def _extend_boundary(self, boundary: np.ndarray, extension_length: float = 2.0, num_extension_points: int = 2) -> np.ndarray:
        """
        Extend a boundary by projecting points with consistent direction.
        
        Args:
            boundary: Nx2 array of boundary points
            extension_length: How far to extend in meters
            num_extension_points: Number of NEW points to add at each end
            
        Returns:
            Extended boundary with consistent extensions
        """
        if len(boundary) < 2:
            return boundary
            
        # Calculate direction vectors using multiple points for stability
        def get_direction_vector(points, front=True):
            # Use first/last 3 points (or all if less than 3) to get average direction
            if front:
                n_points = min(3, len(points))
                vec = points[n_points-1] - points[0]
            else:
                n_points = min(3, len(points))
                vec = points[-1] - points[-n_points]
            return vec / np.linalg.norm(vec)
        
        # Get more stable direction vectors
        front_vec = get_direction_vector(boundary, front=True)
        back_vec = get_direction_vector(boundary, front=False)
        
        # Calculate spacing between points
        spacing = extension_length / num_extension_points
        
        # Create front extension points
        front_points = np.array([
            boundary[0] - front_vec * (spacing * (i + 1))
            for i in range(num_extension_points)
        ])
        
        # Create back extension points
        back_points = np.array([
            boundary[-1] + back_vec * (spacing * (i + 1))
            for i in range(num_extension_points)
        ])
        
        # Combine all points
        extended = np.vstack([
            front_points,    # Extension points at front
            boundary,        # All original points
            back_points      # Extension points at back
        ])
        
        return extended
    
    def create_distance_map(self, 
                          left_boundary: np.ndarray, 
                          right_boundary: np.ndarray,
                          window_center: Optional[np.ndarray] = None,
                          window_size: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Create a distance map for the path defined by boundaries.
        If window_center is provided, creates a local map around that point.
        
        Args:
            left_boundary: Left boundary points
            right_boundary: Right boundary points
            window_center: Optional center point for local window
            window_size: Optional window size (defaults to config value)
            
        Returns:
            Tuple of (distance_map, X, Y, extended_left, extended_right) arrays
        """
        # Convert inputs to numpy arrays
        left_boundary = np.array(left_boundary)
        right_boundary = np.array(right_boundary)
        
        # Extend boundaries
        left_boundary = self._extend_boundary(left_boundary)
        right_boundary = self._extend_boundary(right_boundary)
        
        # Determine map bounds
        if window_center is not None:
            # Local window bounds
            size = window_size if window_size else self.config.window_size
            min_x = window_center[0] - size/2
            max_x = window_center[0] + size/2
            min_y = window_center[1] - size/2
            max_y = window_center[1] + size/2
        else:
            # Global bounds
            min_x = min(np.min(left_boundary[:,0]), np.min(right_boundary[:,0])) - self.config.resolution
            max_x = max(np.max(left_boundary[:,0]), np.max(right_boundary[:,0])) + self.config.resolution
            min_y = min(np.min(left_boundary[:,1]), np.min(right_boundary[:,1])) - self.config.resolution
            max_y = max(np.max(left_boundary[:,1]), np.max(right_boundary[:,1])) + self.config.resolution
        
        # Create grid
        x = np.arange(min_x, max_x, self.config.resolution)
        y = np.arange(min_y, max_y, self.config.resolution)
        X, Y = np.meshgrid(x, y)
        
        # Create geometric objects
        path_polygon = Polygon(np.vstack([left_boundary, np.flip(right_boundary, axis=0)]))
        left_line = LineString(left_boundary)
        right_line = LineString(right_boundary)
        
        # Initialize distance map
        distance_map = np.full_like(X, np.nan)
        
        # Calculate distances for points inside the path
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                point = Point(X[i,j], Y[i,j])
                
                if path_polygon.contains(point):
                    dist_left = point.distance(left_line)
                    dist_right = point.distance(right_line)
                    distance_map[i,j] = min(dist_left, dist_right)
        
        return distance_map, X, Y, left_boundary, right_boundary
    
    def calculate_gradients(self, 
                          distance_map: np.ndarray, 
                          X: np.ndarray, 
                          Y: np.ndarray,
                          left_boundary: Optional[np.ndarray] = None,
                          right_boundary: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate gradients of the distance map using either Sobel operator or geometric method.
        
        Args:
            distance_map: Distance map array
            X, Y: Grid coordinate arrays
            left_boundary, right_boundary: Required only for geometric method
            
        Returns:
            Tuple of (gradient_x, gradient_y) arrays
        """
        if self.config.use_sobel:
            return self._calculate_sobel_gradients(distance_map)
        else:
            return self._calculate_geometric_gradients(distance_map, X, Y, left_boundary, right_boundary)
    
    def _calculate_sobel_gradients(self, distance_map: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Calculate gradients using Sobel operator."""
        # Create a mask for valid regions
        valid_mask = ~np.isnan(distance_map)
        
        # Create a padded version of the distance map
        padded_distance = np.pad(distance_map, 2, mode='edge')
        padded_mask = np.pad(valid_mask, 2, mode='constant', constant_values=False)
        
        # Initialize gradient arrays
        gradient_x = np.zeros_like(distance_map)
        gradient_y = np.zeros_like(distance_map)
        
        # Manual convolution with mask checking
        for i in range(distance_map.shape[0]):
            for j in range(distance_map.shape[1]):
                if valid_mask[i,j]:
                    window = padded_distance[i:i+5, j:j+5]
                    window_mask = padded_mask[i:i+5, j:j+5]
                    
                    if np.sum(window_mask) > 20:
                        gradient_x[i,j] = np.sum(window * self.sobel_x)
                        gradient_y[i,j] = np.sum(window * self.sobel_y)
                    else:
                        gradient_x[i,j] = np.nan
                        gradient_y[i,j] = np.nan
                else:
                    gradient_x[i,j] = np.nan
                    gradient_y[i,j] = np.nan
        
        # Normalize gradients
        magnitude = np.sqrt(gradient_x**2 + gradient_y**2)
        valid_magnitude = magnitude > self.config.sobel_threshold
        gradient_x[valid_magnitude] /= magnitude[valid_magnitude]
        gradient_y[valid_magnitude] /= magnitude[valid_magnitude]
        
        return gradient_x, gradient_y
    
    def _calculate_geometric_gradients(self, 
                                    distance_map: np.ndarray, 
                                    X: np.ndarray, 
                                    Y: np.ndarray,
                                    left_boundary: np.ndarray,
                                    right_boundary: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculates gradients based on geometric distance to nearest boundary points.
            Args:
                distance_map (2D numpy array): Distance map from create_distance_map.
                X (2D numpy array): X-coordinates of the grid points.
                Y (2D numpy array): Y-coordinates of the grid points.
                left_boundary (list of tuple): Left boundary coordinates.
                right_boundary (list of tuple): Right boundary coordinates.
            
            Returns:
                tuple:
                    - gradient_x (2D numpy array): X-component of the gradient.
                    - gradient_y (2D numpy array): Y-component of the gradient.
        """
        # Create geometric objects
        left_line = LineString(left_boundary)
        right_line = LineString(right_boundary)
        path_polygon = Polygon(np.vstack([left_boundary, np.flip(right_boundary, axis=0)]))
        
        # Initialize gradients
        gradient_x = np.full_like(X, np.nan)
        gradient_y = np.full_like(X, np.nan)
        
        # Calculate gradients for points inside the path
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                point = Point(X[i,j], Y[i,j])
                
                if path_polygon.contains(point):
                    nearest_left = nearest_points(point, left_line)[1]
                    nearest_right = nearest_points(point, right_line)[1]
                    
                    dist_left = point.distance(left_line)
                    dist_right = point.distance(right_line)
                    
                    if dist_left < dist_right:
                        dx = point.x - nearest_left.x
                        dy = point.y - nearest_left.y
                    else:
                        dx = point.x - nearest_right.x
                        dy = point.y - nearest_right.y
                    
                    magnitude = np.sqrt(dx*dx + dy*dy)
                    if magnitude > self.config.sobel_threshold:
                        gradient_x[i,j] = dx/magnitude
                        gradient_y[i,j] = dy/magnitude
        
        return gradient_x, gradient_y
    
    def get_point_info(self, 
                      point: np.ndarray, 
                      distance_map: np.ndarray,
                      gradient_x: np.ndarray,
                      gradient_y: np.ndarray,
                      X: np.ndarray,
                      Y: np.ndarray) -> Tuple[float, np.ndarray]:

        """
            Retrieves the distance and gradient at a specific query point.
                Args:
                    point (tuple): Query point (x, y). x, y are floats.
                    distance_map (2D numpy array): Distance to the nearest boundary for each grid point.
                    gradient_x (2D numpy array): X-component of the gradient at each grid point.
                    gradient_y (2D numpy array): Y-component of the gradient at each grid point.
                    X (2D numpy array): X-coordinates of the grid points.
                    Y (2D numpy array): Y-coordinates of the grid points.

                Returns:
                    tuple:
                        - distance (float): Distance to the nearest boundary at the query point.
                        - gradient (tuple): Gradient (x, y) at the query point.

                Notes:
                    The function uses the nearest grid cell to approximate the values for the query point.

            """
        # Find nearest grid cell
        i = np.argmin(np.abs(Y[:,0] - point[1]))
        j = np.argmin(np.abs(X[0,:] - point[0]))
        
        # Check if indices are valid
        if not (0 <= i < distance_map.shape[0] and 0 <= j < distance_map.shape[1]):
            return np.nan, np.array([np.nan, np.nan])
        
        distance = distance_map[i,j]
        gradient = np.array([gradient_x[i,j], gradient_y[i,j]])
        
        # Check if values are valid
        if np.isnan(distance) or np.any(np.isnan(gradient)):
            return np.nan, np.array([np.nan, np.nan])
        
        return distance, gradient
    
    def get_distance_at_point(self, point: np.ndarray, 
                         distance_map: np.ndarray,
                         X: np.ndarray, 
                         Y: np.ndarray) -> float:
        """
        Get interpolated distance value at any point.
        
        Args:
            point: (x,y) coordinates as numpy array
            distance_map: The distance map array
            X, Y: Grid coordinate arrays
            
        Returns:
            float: Distance to nearest boundary
        """
        # Find nearest grid cell
        i = np.argmin(np.abs(Y[:,0] - point[1]))
        j = np.argmin(np.abs(X[0,:] - point[0]))
        
        # Simple nearest neighbor interpolation
        if 0 <= i < distance_map.shape[0] and 0 <= j < distance_map.shape[1]:
            return distance_map[i,j]
        return np.nan
    
    def check_path_collision(self, 
                        points: np.ndarray,
                        distance_map: np.ndarray,
                        X: np.ndarray,
                        Y: np.ndarray,
                        path_polygon: Polygon,
                        safety_margin: float = 0.2,
                        check_stride: int = 3) -> bool:
        """
        Check if path maintains minimum distance from boundaries.
        Uses strided checking for efficiency.
        
        Args:
            points: Nx2 array of (x,y) points
            distance_map: The distance map array
            X, Y: Grid coordinate arrays
            path_polygon: Polygon representing valid path region
            safety_margin: Minimum allowed distance from boundaries
            check_stride: Check every Nth point for efficiency
            
        Returns:
            bool: True if collision detected
        """
        print("Length of points: ", len(points), " In check_path_collision")
        print(f"Point[0]: {Point(points[0])} and {path_polygon.intersects(Point(points[0]))}")
        print(f"Point[0]: {Point(points[0])} and {path_polygon.contains(Point(points[0]))}")
        print(f"Point[-1]: {Point(points[-1])} and {path_polygon.contains(Point(points[-1]))}")
        # Always check first and last points
        if not path_polygon.contains(Point(points[0])):
            print("First point not in path_polygon")
            return True
        if not path_polygon.contains(Point(points[-1])):
            return True
        print("Length of points: ", len(points), " In check_path_collision")
        # Check strided points
        for point in points[::check_stride]:
            # First check if point is inside valid region
            if not path_polygon.contains(Point(point)):
                return True
                
            # Then check distance from boundaries
            dist = self.get_distance_at_point(point, distance_map, X, Y)
            if np.isnan(dist) or dist < safety_margin:
                return True
                
        return False
    
    def visualize_distance_map(self,
                             distance_map: np.ndarray,
                             X: np.ndarray,
                             Y: np.ndarray,
                             gradient_x: Optional[np.ndarray] = None,
                             gradient_y: Optional[np.ndarray] = None,
                             left_boundary: Optional[np.ndarray] = None,
                             right_boundary: Optional[np.ndarray] = None,
                             title: str = "Distance Map"):
        """
        Visualize the distance map with optional gradients and boundaries.
        
        Args:
            distance_map: Distance map array
            X, Y: Grid coordinate arrays
            gradient_x, gradient_y: Optional gradient arrays
            left_boundary, right_boundary: Optional boundary points
            title: Plot title
        """
        plt.figure(figsize=(12, 8))
        
        # Plot distance map
        plt.pcolormesh(X, Y, np.ma.masked_invalid(distance_map),
                      cmap='YlOrRd', shading='auto')
        plt.colorbar(label='Distance to nearest boundary (m)')
        
        # Plot gradients if provided
        if gradient_x is not None and gradient_y is not None:
            stride = 5  # Adjust for visualization density
            plt.quiver(X[::stride, ::stride], Y[::stride, ::stride],
                      gradient_x[::stride, ::stride], gradient_y[::stride, ::stride],
                      scale=20, width=0.003, color='blue', alpha=0.3)
        
        # Plot boundaries if provided
        if left_boundary is not None and right_boundary is not None:
            plt.plot(left_boundary[:,0], left_boundary[:,1], 'k-', label='Left Boundary')
            plt.plot(right_boundary[:,0], right_boundary[:,1], 'k-', label='Right Boundary')
        
        plt.title(title)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()

