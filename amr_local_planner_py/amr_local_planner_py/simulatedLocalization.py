# import rclpy
# from rclpy.node import Node
# from rclpy.timer import Timer
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from amr_interfaces.msg import RobotPose, Trajectory, TrajectoryPoint
# from amr_utils_python.coordinate_transforms_py import CoordinateTransforms, GPSPoint, LocalPoint
# from typing import List, Optional
# from geometry_msgs.msg import Quaternion
# import numpy as np
# import math
# import time


# class AMRDummyLocalizationNode(Node):
#     def __init__(self):
#         super().__init__('amr_dummy_localization_py')

#         # Declare and get parameters
#         origin = self._get_origin_from_params()
#         initial_pos = self._get_robot_initial_pos_from_params()

#         # Check if parameters are valid
#         if not self._are_params_valid(origin, initial_pos):
#             self.get_logger().fatal("Invalid parameters. Node will shut down. Check if you have passed params file as ros argument")
#             rclpy.shutdown()
#             return

#         # Initialize coordinate transformer with origin
#         self.coord_transform = CoordinateTransforms(origin)

#         # Convert initial GPS position to local coordinates
#         local_pos = self.coord_transform.gps_to_local(initial_pos)
#         self.current_pos = local_pos
        
#         # Callback group to allow concurrent callbacks
#         self.callback_group = ReentrantCallbackGroup()

#         # Trajectory tracking attributes
#         self.trajectory: list[TrajectoryPoint] = []
#         self.current_trajectory_index = 0
#         self.trajectory_timer: Timer = None

#         # Robot state
#         self.current_pose = RobotPose()
#         self.current_pose.pose.pose.orientation.w = 1.0  # Identity quaternion


#         # Create publisher and timer
#         # timer is set to run at 100ms intervals i.e 10 times per second = publish robot pose at 10Hz
#         self.publisher = self.create_publisher(RobotPose, 'robot_pose', 10)
#         self.timer = self.create_timer(0.1, self.publish_robot_pose)
        
#         # Create subscriber for trajectory
#         self.trajectory_subscriber = self.create_subscription(
#             Trajectory,  # Use the new Trajectory message type
#             'trajectory',
#             self.trajectory_callback,
#             10,
#             callback_group=self.callback_group
#         )

#         self.get_logger().info(
#             f"Localization started at local coordinates: x={self.current_pos.x:.2f}, y={self.current_pos.y:.2f}"
#         )
        
#     def trajectory_callback(self, msg: Trajectory):
#         """
#         Callback for trajectory subscription.
#         Stops the pose publishing timer and stores the trajectory points.
#         """
#         # Stop the timer that publishes initial robot pose untill we get first trajectory point vector worth 3sec
#         if self.timer:
#             self.timer.cancel()
        
#         # Cancel existing timer if it exists. This timer is responsible for simulating robot current pose based on trajectory
#         if self.trajectory_timer:
#             self.trajectory_timer.cancel()
        
#         # Reset trajectory tracking
#         self.trajectory = msg.points
#         self.current_trajectory_index = 0
        
#         # Log trajectory details
#         self.get_logger().info(
#             f"Received new trajectory with {len(self.trajectory)} points. "
#             f"Total trajectory duration: {self.trajectory[-1].timestamp:.2f} seconds"
#         )
        
#         # Start new trajectory following timer
#         self.trajectory_timer = self.create_timer(
#             0.01,  # 100 Hz
#             self.follow_trajectory,
#             callback_group=self.callback_group
#         )
        
#     def follow_trajectory(self):
#         """
#         Follow trajectory points at 100 Hz.
        
#         Interpolate between current pose and target trajectory point.
#         Publish updated pose.
#         """
#         # Check if we've completed the trajectory
#         if self.current_trajectory_index >= len(self.trajectory):
#             # Stop timer when trajectory is complete
#             self.trajectory_timer.cancel()
#             self.get_logger().info("Trajectory following completed")
#             return

#         # Get current trajectory point
#         current_point = self.trajectory[self.current_trajectory_index]
        
#         # Interpolate pose
#         self._interpolate_and_publish_pose(current_point)
        
#         # Move to next trajectory point
#         self.current_trajectory_index += 1
        
#     def _interpolate_and_publish_pose(self, trajectory_point: TrajectoryPoint):
#         """
#         Interpolate and publish robot pose based on trajectory point.
        
#         This method provides a simple linear interpolation strategy.
#         In a real-world scenario, you'd likely use more sophisticated 
#         motion planning and control algorithms.
#         """
#         # Update pose
#         self.current_pose.header.stamp = self.get_clock().now().to_msg()
#         self.current_pose.header.frame_id = 'map'
        
#         # Position
#         self.current_pose.pose.pose.position.x = trajectory_point.pose.pose.position.x
#         self.current_pose.pose.pose.position.y = trajectory_point.pose.pose.position.y
#         self.current_pose.pose.pose.position.z = trajectory_point.pose.pose.position.z
        
#         # Orientation (if provided in trajectory)
#         if trajectory_point.pose.pose.orientation.w != 0:
#             self.current_pose.pose.pose.orientation = trajectory_point.pose.pose.orientation
        
#         # Velocity and yaw rate
#         self.current_pose.velocity = trajectory_point.velocity
#         self.current_pose.yaw_rate = trajectory_point.yaw_rate
        
#         # Publish pose
#         self.pose_publisher.publish(self.current_pose)

#     def _get_origin_from_params(self) -> GPSPoint:
#         """Retrieve origin GPS coordinates from ROS parameters."""
#         self.declare_parameter('origins.gps_origin.latitude', 0.0)
#         self.declare_parameter('origins.gps_origin.longitude', 0.0)
#         self.declare_parameter('origins.gps_origin.altitude', 0.0)

#         return GPSPoint(
#             self.get_parameter('origins.gps_origin.latitude').get_parameter_value().double_value,
#             self.get_parameter('origins.gps_origin.longitude').get_parameter_value().double_value,
#             self.get_parameter('origins.gps_origin.altitude').get_parameter_value().double_value
#         )

#     def _get_robot_initial_pos_from_params(self) -> GPSPoint:
#         """Retrieve robot's initial GPS position from ROS parameters."""
#         self.declare_parameter('robot_initial_gps_pos.latitude', 0.0)
#         self.declare_parameter('robot_initial_gps_pos.longitude', 0.0)
#         self.declare_parameter('robot_initial_gps_pos.altitude', 0.0)

#         return GPSPoint(
#             self.get_parameter('robot_initial_gps_pos.latitude').get_parameter_value().double_value,
#             self.get_parameter('robot_initial_gps_pos.longitude').get_parameter_value().double_value,
#             self.get_parameter('robot_initial_gps_pos.altitude').get_parameter_value().double_value
#         )

#     def _are_params_valid(self, origin: GPSPoint, initial_pos: GPSPoint) -> bool:
#         """Validate origin and initial GPS parameters."""
#         if origin.latitude == 0.0 and origin.longitude == 0.0:
#             self.get_logger().error("Origin GPS parameters are not set correctly.")
#             return False
#         if initial_pos.latitude == 0.0 and initial_pos.longitude == 0.0:
#             self.get_logger().error("Initial robot GPS position parameters are not set correctly.")
#             return False
#         return True

#     def publish_robot_pose(self):
#         """Publish robot pose message."""
#         msg = RobotPose()

#         # Populate header
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'map'

#         # Set position and orientation in pose
#         msg.pose.pose.position.x = self.current_pos.x
#         msg.pose.pose.position.y = self.current_pos.y
#         msg.pose.pose.position.z = 0.0  # Assuming ground level

#         # Default orientation as identity quaternion (no rotation)
#         msg.pose.pose.orientation.x = 0.0
#         msg.pose.pose.orientation.y = 0.0
#         msg.pose.pose.orientation.z = 0.0
#         msg.pose.pose.orientation.w = 1.0

#         # Set velocity and yaw rate
#         msg.velocity = 0.0  # Set this as appropriate for your system
#         msg.yaw_rate = 0.0  # Set this as appropriate for your system

#         # Publish the message
#         self.publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = AMRDummyLocalizationNode()
#     executor = MultiThreadedExecutor(num_threads=2)
#     executor.add_node(node)
    
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         executor.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration
from amr_interfaces.msg import RobotPose, Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
import numpy as np
from typing import List, Optional, Tuple
import math
import time
import os

class SimulatedLocalizationNode(Node):
    """
    Simulated localization node that:
    1. Initially publishes static robot pose
    2. Follows trajectory points when received
    3. Handles blending between successive trajectory windows
    """
    
    def __init__(self):
        super().__init__('simulated_localization')
        
       # Use same base directory as LocalPlannerNode
        self.save_dir = os.path.join(os.getcwd(), "trajectory_data")
        self.data_dir = os.path.join(self.save_dir, "data")
        self.robot_states_file = os.path.join(self.data_dir, "robot_states.csv")
        
        # Create directory if it doesn't exist
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Initialize CSV file with headers right away
        headers = ['ros_time', 'simulated_time', 'trajectory_start_time', 
                  'position_x', 'position_y', 'heading', 
                  'velocity', 'yaw_rate', 'trajectory_index']
        
        with open(self.robot_states_file, 'w') as f:
            f.write(','.join(headers) + '\n')
            
        # Open file handle for appending
        self.state_file = None
        self.state_save_counter = 0

        self.get_logger().info(f"Will save robot states to: {self.robot_states_file}")
        
        # Initialize callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initial robot state
        # First waypoint from centerline_processor: [19.0998, 4.24597]
        # Second waypoint for initial heading: [17.6539, 5.35312]
        self.initial_position = np.array([19.0998, 4.24597])
        next_waypoint = np.array([17.6539, 5.35312])
        
        # self.initial_position = np.array([12.8583, 8.97716])
        # next_waypoint = np.array([12.2636, 9.41402])
        
        # Calculate initial heading from first two waypoints
        heading_vector = next_waypoint - self.initial_position
        self.initial_heading = math.atan2(heading_vector[1], heading_vector[0])
        
        # Current robot state
        self.current_position = self.initial_position.copy()
        self.current_heading = self.initial_heading
        self.current_velocity = 0.0
        self.current_yaw_rate = 0.0
        
        # Trajectory tracking
        self.current_trajectory: List[TrajectoryPoint] = []
        self.next_trajectory: List[TrajectoryPoint] = []
        self.current_trajectory_start_time: Optional[float] = None
        self.trajectory_blend_time: Optional[float] = None
        self.simulated_time = 0.0  # Time in trajectory frame
        self.current_point_index = 0
        
        # Cleanup parameters
        self.trajectory_cleanup_threshold = 10  # Keep 10 points before current index
        self.cleanup_interval = 20  # Clean up every 20 points
        
        # Publishers
        self.pose_publisher = self.create_publisher(
            RobotPose,
            'robot_pose',
            10
        )
        
        # Subscribers
        self.trajectory_subscriber = self.create_subscription(
            Trajectory,
            'trajectory',
            self.trajectory_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Timers for pose publishing and trajectory following
        # Initial pose publisher (10 Hz)
        self.initial_pose_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_initial_pose,
            callback_group=self.callback_group
        )
        
        # Trajectory follower (100 Hz)
        self.trajectory_timer = None
        
        self.get_logger().info(
            f"Simulated localization started at position: "
            f"[{self.current_position[0]:.4f}, {self.current_position[1]:.4f}], "
            f"heading: {math.degrees(self.current_heading):.2f} degrees"
        )
        
    def save_robot_state(self):
        """Save robot state to CSV file"""
        # Open file if not already open
        if self.state_file is None:
            self.state_file = open(self.robot_states_file, 'a')
        
        # Save every 5th frame
        self.state_save_counter += 1
        if self.state_save_counter % 5 != 0:
            return
            
        state_data = {
            'ros_time': self.get_clock().now().nanoseconds / 1e9,
            'simulated_time': self.simulated_time,
            'trajectory_start_time': self.current_trajectory_start_time,
            'position_x': float(self.current_position[0]),
            'position_y': float(self.current_position[1]),
            'heading': float(self.current_heading),
            'velocity': float(self.current_velocity),
            'yaw_rate': float(self.current_yaw_rate),
            'trajectory_index': self.current_point_index
        }
        
        # Write to file and flush to ensure data is saved
        values = [str(state_data[key]) for key in state_data.keys()]
        self.state_file.write(','.join(values) + '\n')
        self.state_file.flush()  # Force write to disk
        
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        # Close file handle if open
        if self.state_file is not None:
            self.state_file.close()
            self.get_logger().info("Closed robot states file")
        
        super().destroy_node()

    def euler_to_quaternion(self, yaw: float) -> Quaternion:
        """
        Convert yaw angle to quaternion where roll and pitch are 0.
        yaw: rotation around z-axis in radians
        """
        quat = Quaternion()
        half_yaw = yaw * 0.5
        quat.x = 0.0  # Roll component
        quat.y = 0.0  # Pitch component
        quat.z = math.sin(half_yaw)  # z component for yaw
        quat.w = math.cos(half_yaw)  # Scalar component 
        return quat

    def publish_initial_pose(self):
        """Publish initial static robot pose until trajectory is received."""
        msg = RobotPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set pose
        msg.pose.header = msg.header
        msg.pose.pose.position.x = self.initial_position[0]
        msg.pose.pose.position.y = self.initial_position[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.euler_to_quaternion(self.initial_heading)
        
        # Initial state is stationary
        msg.velocity = 0.0
        msg.yaw_rate = 0.0
        
        self.pose_publisher.publish(msg)



    def switch_to_next_trajectory(self):
        """Switch to next trajectory and reset tracking variables."""
        if self.next_trajectory:
            self.current_trajectory = self.next_trajectory
            self.next_trajectory = []
            self.current_trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
            self.trajectory_blend_time = None
            
            self.get_logger().info("Switched to next trajectory")

    def clean_old_trajectory_points(self):
        """Remove trajectory points that we've already passed, keeping some buffer."""
        if len(self.current_trajectory) == 0:
            return
            
        # Only clean up if we have enough points and have moved forward enough
        if (self.current_point_index > self.trajectory_cleanup_threshold and 
            self.current_point_index % self.cleanup_interval == 0):
            
            # Keep some past points for potential interpolation
            points_to_remove = self.current_point_index - self.trajectory_cleanup_threshold
            self.current_trajectory = self.current_trajectory[points_to_remove:]
            self.current_point_index -= points_to_remove
            
            self.get_logger().debug(
                f"Cleaned up {points_to_remove} old trajectory points. "
                f"Now have {len(self.current_trajectory)} points."
            )
            
    def find_trajectory_join_point(self, new_trajectory: List[dict]) -> Optional[int]:
        """
        Find where new trajectory should join with current trajectory.
        Returns index in current trajectory where new trajectory should start.
        """
        if not self.current_trajectory or not new_trajectory:
            return None
            
        # Start searching from recent positions
        search_start = max(0, self.current_point_index - 20)
        search_end = min(len(self.current_trajectory), 
                        self.current_point_index + 20)
                        
        # Get time ranges
        new_traj_start_time = new_trajectory[0]['time']
        new_traj_end_time = new_trajectory[-1]['time']
        
        join_index = None
        min_distance = float('inf')
        
        # Search for closest point within overlapping time range
        for i in range(search_start, search_end):
            curr_point = self.current_trajectory[i]
            
            # Check if point is within new trajectory's time range
            if (curr_point['time'] >= new_traj_start_time and 
                curr_point['time'] <= new_traj_end_time):
                
                # Find closest point in new trajectory
                for new_point in new_trajectory:
                    time_diff = abs(new_point['time'] - curr_point['time'])
                    pos_diff = np.linalg.norm(
                        new_point['position'] - curr_point['position']
                    )
                    
                    # Use weighted combination of time and position difference
                    total_diff = time_diff + 0.5 * pos_diff
                    
                    if total_diff < min_distance:
                        min_distance = total_diff
                        join_index = i
        
        return join_index

    def blend_trajectories(self, join_index: int):
        """
        Blend new trajectory with current trajectory at join point.
        """
        if not self.next_trajectory or join_index is None:
            return
            
        # Get overlapping time range
        join_time = self.current_trajectory[join_index]['time']
        
        # Find corresponding point in new trajectory
        new_start_idx = 0
        for i, point in enumerate(self.next_trajectory):
            if point['time'] >= join_time:
                new_start_idx = i
                break
        
        # Replace points after join index with new trajectory
        self.current_trajectory = (
            self.current_trajectory[:join_index] +
            self.next_trajectory[new_start_idx:]
        )
        
        self.next_trajectory = []
        self.get_logger().info(
            f"Blended trajectories at time {join_time:.2f}s, "
            f"index {join_index}"
        )

    def interpolate_trajectory_point(self, traj_points: List[dict], current_time: float) -> Optional[Tuple]:
        """
        Interpolate between trajectory points for given time.
        Uses index tracking for efficiency.
        """
        if not traj_points:
            self.get_logger().info("No trajectory points available")
            return None
            
        # Debug time info
        try:
            time_range = f"Time range in trajectory: [{traj_points[0]['time']:.2f}, {traj_points[-1]['time']:.2f}]"
            # self.get_logger().info(f"Current time: {current_time:.2f}, {time_range}")
        except Exception as e:
            self.get_logger().error(f"Error accessing trajectory times: {str(e)}")
            return None

        # Handle time before trajectory start
        if current_time < traj_points[0]['time']:
            # self.get_logger().info(f"Current time {current_time:.2f} is before trajectory start {traj_points[0]['time']:.2f}")
            return None
            
        # Handle time after trajectory end
        if current_time > traj_points[-1]['time']:
            # self.get_logger().info(f"Current time {current_time:.2f} is after trajectory end {traj_points[-1]['time']:.2f}")
            return None

        # Start search from current index
        start_idx = max(0, self.current_point_index - 1)
        
        # Find bracketing points
        p1 = None
        p2 = None
        
        for i in range(start_idx, len(traj_points) - 1):
            if (traj_points[i]['time'] <= current_time and 
                traj_points[i + 1]['time'] > current_time):
                p1 = traj_points[i]
                p2 = traj_points[i + 1]
                self.current_point_index = i  # Update current index
                break
        
        if not p1 or not p2:
            return None
            
        # Calculate interpolation factor
        dt = p2['time'] - p1['time']
        if abs(dt) < 1e-6:  # Avoid division by zero
            t = 0.0
        else:
            t = (current_time - p1['time']) / dt
            
        # Ensure t is between 0 and 1
        t = max(0.0, min(1.0, t))
        
        try:
            # Interpolate position
            position = p1['position'] + t * (p2['position'] - p1['position'])
            
            # Calculate heading from positions
            heading_vector = p2['position'] - p1['position']
            heading = math.atan2(heading_vector[1], heading_vector[0])
            
            # Interpolate velocity and yaw_rate
            velocity = p1['velocity'] + t * (p2['velocity'] - p1['velocity'])
            yaw_rate = p1['yaw_rate'] + t * (p2['yaw_rate'] - p1['yaw_rate'])
            
            return position, heading, velocity, yaw_rate
            
        except Exception as e:
            self.get_logger().error(f"Error during interpolation: {str(e)}")
            return None

    def follow_trajectory(self):
        """Main trajectory following loop with trajectory blending."""
        if self.current_trajectory_start_time is None:
            return
            
        current_ros_time = self.get_clock().now().nanoseconds / 1e9
        self.simulated_time = current_ros_time - self.current_trajectory_start_time
        
        # Debug timing info
        self.get_logger().debug(
            f"Current ROS Time: {current_ros_time:.2f}, "
            f"Start Time: {self.current_trajectory_start_time:.2f}, "
            f"Simulated Time: {self.simulated_time:.2f}"
        )
        
        # Check if we have a new trajectory to blend
        if self.next_trajectory:
            join_index = self.find_trajectory_join_point(self.next_trajectory)
            if join_index is not None:
                self.blend_trajectories(join_index)
                self.get_logger().info(f"Blended trajectories at index {join_index}")
        
        # Get interpolated state
        state = self.interpolate_trajectory_point(
            self.current_trajectory, 
            self.simulated_time
        )
        
        if not state:
            # Check if we're beyond trajectory end
            if (len(self.current_trajectory) > 0 and 
                self.simulated_time > self.current_trajectory[-1]['time']):
                # Hold last valid pose
                last_point = self.current_trajectory[-1]
                state = (
                    last_point['position'],
                    math.atan2(last_point['position'][1], last_point['position'][0]),
                    0.0,  # Zero velocity at end
                    0.0   # Zero yaw rate at end
                )
                # self.get_logger().info(
                #     f"Holding last pose at time {self.simulated_time:.2f} "
                #     f"(trajectory ended at {last_point['time']:.2f})"
                # )
            else:
                # No valid trajectory yet
                return
        
        position, heading, velocity, yaw_rate = state
        
        # Clean up old trajectory points periodically
        self.clean_old_trajectory_points()
        
        # Update and publish robot state
        msg = RobotPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.pose.header = msg.header
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.euler_to_quaternion(heading)
        
        msg.velocity = float(velocity)
        msg.yaw_rate = float(yaw_rate)
        
        self.pose_publisher.publish(msg)
        
        # Update current state
        self.current_position = position
        self.current_heading = heading
        self.current_velocity = velocity
        self.current_yaw_rate = yaw_rate
        
        # Save state for animation
        self.save_robot_state()

    def trajectory_callback(self, msg: Trajectory):
        """Handle incoming trajectory messages with proper blending."""
        new_trajectory = []
        for point in msg.points:
            new_trajectory.append({
                'position': np.array([
                    point.pose.pose.position.x,
                    point.pose.pose.position.y
                ]),
                'orientation': point.pose.pose.orientation,
                'velocity': point.velocity,
                'yaw_rate': point.yaw_rate,
                'time': point.timestamp
            })
            
        if not self.current_trajectory:
            # First trajectory - start following it
            self.current_trajectory = new_trajectory
            self.current_trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
            self.current_point_index = 0
            
            # Stop initial pose publisher and start trajectory follower
            self.initial_pose_timer.cancel()
            if not self.trajectory_timer:
                self.trajectory_timer = self.create_timer(
                    0.01,  # 100 Hz
                    self.follow_trajectory,
                    callback_group=self.callback_group
                )
                
            self.get_logger().info(
                f"Starting to follow first trajectory with {len(new_trajectory)} points"
            )
        else:
            # Store for blending
            self.next_trajectory = new_trajectory
            self.get_logger().info(
                f"Received next trajectory with {len(new_trajectory)} points "
                f"from time {new_trajectory[0]['time']:.2f}s "
                f"to {new_trajectory[-1]['time']:.2f}s"
            )

def main(args=None):
    rclpy.init(args=args)
    
    node = SimulatedLocalizationNode()
    
    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()