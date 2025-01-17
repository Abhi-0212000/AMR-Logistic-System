#!/usr/bin/env python3
from curses import window
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration
from amr_interfaces.msg import RobotPose, Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Quaternion

import numpy as np
from typing import List, Optional, Dict, Tuple
import math
import time

# from window_manager import WindowManager, WindowConfig
from amr_local_planner_py.window_manager import WindowManager, WindowConfig
from amr_local_planner_py.trajectory_optimizer import TrajectoryOptimizerConfig
from amr_local_planner_py.common_types import TrajectoryInfo

import os
import pandas as pd
import matplotlib.pyplot as plt
from threading import Lock

class LocalPlannerNode(Node):
    """
    ROS2 node for local trajectory planning with windowed optimization.
    
    Key features:
    - Handles goal-directed trajectory planning
    - Manages windowed trajectory generation
    - Coordinates with localization node
    - Robust error handling and state management
    - Saves trajectory data and visualizations
    - Thread-safe planning operations
    """
    
    def __init__(self):
        super().__init__('local_planner')
        
        # Initialize callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Set up data saving directories
        self.save_dir = os.path.join(os.getcwd(), "trajectory_data")
        self.plots_dir = os.path.join(self.save_dir, "plots")
        self.data_dir = os.path.join(self.save_dir, "data")
        self.robot_states_file = os.path.join(self.data_dir, "robot_states.csv")
        
        # Create directories if they don't exist
        os.makedirs(self.plots_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Thread synchronization
        self.planning_lock = Lock()
        self.is_planning = False
        
        # Window tracking
        self.window_index = 0
        self.window_stats = []
        
        # Initialize trajectory generation parameters
        self.window_config = WindowConfig(
            planning_time=1.0,    # Time needed for planning (sec)
            buffer_time=0.5,      # Extra buffer time (sec)
            lookahead_points=4,   # Points to look ahead (~6m with 1m spacing)
            window_spacing=2.0    # Approx spacing between points (meters)
        )
        
        # Set up optimizer config
        self.optimizer_config = TrajectoryOptimizerConfig(
            num_points=150,
            safety_margin=0.2,
            check_stride=5,
            base_tangent_factor=0.5,
            optimization_time_limit=0.8,
            distance_map_resolution=0.2,
            distance_map_window_size=3,
            spline_points_method="linear"
        )
        
        # Hard-coded lanelet data (for now)
        self.lanelet_dict, self.lanelet_ids = self._initialize_lanelet_data()
        
        # Initialize window manager
        self.window_manager = WindowManager(
            self.lanelet_dict, 
            self.lanelet_ids,
            self.window_config
        )
        self.window_manager.optimizer_config = self.optimizer_config
        
        # State variables
        self.current_robot_pose: Optional[RobotPose] = None
        self.goal_position: Optional[np.ndarray] = None
        self.is_goal_reached: bool = False
        self.last_planning_time: Optional[float] = None
        
        # Publishers
        self.trajectory_publisher = self.create_publisher(
            Trajectory,
            'trajectory',
            10
        )
        
        # Subscribers
        self.pose_subscriber = self.create_subscription(
            RobotPose,
            'robot_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Planning timer
        self.planning_timer = self.create_timer(
            0.1,  # 10 Hz check for planning
            self.planning_loop,
            callback_group=self.callback_group
        )
        
        # Store last waypoint as goal
        last_waypoint = self.window_manager.centerline_processor.processed_points[-1]
        self.goal_position = np.array(last_waypoint)
        
        # Timing parameters
        self.window_sleep_time = 0.15  # 250ms sleep between windows
        self.next_planning_time = None
        
        self.get_logger().info(
            f"Local planner initialized. Goal position set to: "
            f"[{self.goal_position[0]:.4f}, {self.goal_position[1]:.4f}]"
        )
        
    def _initialize_lanelet_data(self) -> Tuple[Dict, List[int]]:
        """Initialize hard-coded lanelet data."""
        lanelet_dict = {
                -6995: {
                "length": 1.87,
                "centerline_points": [
                    [19.0998, 4.24597], [17.6539, 5.35312]
                ],
                "left_boundary": np.array([ [18.504, 3.38053], [16.979, 4.46012] ]),
                "right_boundary": np.array([ [19.7237, 5.15215], [18.3057, 6.21025] ])
            },
            -7019: {
                "length": 3.19,
                "centerline_points": [
                    [17.6539, 5.35312], [16.9326, 5.90554], [16.2295, 6.43471], [15.7666, 6.79154], [15.1116, 7.28624]
                ],
                "left_boundary": np.array([ [16.979, 4.46012], [14.454, 6.4172] ]),
                "right_boundary": np.array([ [18.3057, 6.21025], [15.7474, 8.06041] ])
            },
            -6996: {
                "length": 3.06,
                "centerline_points": [
                    [15.1116, 7.28624], [14.4443, 7.80432], [13.7414, 8.32168], [12.8583, 8.97716]
                ],
                "left_boundary": np.array([ [14.454, 6.4172], [13.5285, 7.11906], [12.736, 7.68188], [11.991, 8.23383] ]),
                "right_boundary": np.array([ [15.7474, 8.06041], [14.6891, 8.88951], [14.0755, 9.38503], [13.389, 9.97357] ])
            },
            -6999: {
                "length": 5.42,
                "centerline_points": [
                    [12.8583, 8.97716], [12.2636, 9.41402], [11.6704, 9.77417], [11.2832, 9.89637], [10.5473, 9.99393], [9.97879, 9.94867], [9.35417, 9.864], [8.4243, 9.5917], [7.62356, 9.37511], [6.6296, 8.75324], [5.98117, 8.02015], [5.65965, 7.51788], [5.36672, 7.0406]
                ],
                "left_boundary": np.array([ [11.991, 8.23383], [11.7149, 8.42907], [11.2115, 8.73787], [10.7119, 8.85784], [10.3261, 8.61404], [10.0488, 8.26614], [9.81941, 7.89558], [9.50831, 7.46436], [9.17449, 6.98546], [8.78196, 6.48178], [8.5516, 6.15843], [8.25141, 5.77465] ]),
                
                "right_boundary": np.array([ [13.389, 9.97357], [13.0033, 10.3201], [12.5475, 10.6181], [12.2008, 10.7883], [11.995, 10.872], [11.5086, 11.0697], [11.0918, 11.1913], [10.2724, 11.3168], [9.703, 11.3763], [9.1809, 11.4368], [8.44671, 11.4458], [7.78494, 11.3855], [7.06586, 11.2295], [6.52384, 11.1007], [6.12499, 10.9157], [5.80921, 10.7205], [5.4004, 10.4409], [5.09669, 10.2341], [4.66492, 9.91857], [4.32653, 9.66981], [4.04825, 9.37502], [3.79223, 9.15151], [3.43345, 8.80946] ])
            },
            -6985: {
                "length": 20.10,
                "centerline_points": [
                    [5.36672, 7.0406], [4.8454, 6.19118], [3.2809, 3.89322], [2.01286, 2.16787], [0.723955, 0.300428], [-1.12082, -2.192], [-3.33258, -5.28206], [-4.4865, -6.79262], [-6.01396, -8.57034]
                ],
                "left_boundary": np.array([ [8.25141, 5.77465], [-3.60881, -10.4589] ]),
                "right_boundary": np.array([ [3.43345, 8.80946], [-8.4099, -7.15374] ])
            },
            -7011: {
                "length": 17.30,
                "centerline_points": [
                    [-6.01396, -8.57034], [-7.19296, -10.0106], [-8.23884, -10.9995], [-9.38688, -11.6126], [-11.1503, -12.2379], [-11.944, -12.4708], [-12.8729, -12.5318], [-13.6901, -12.56], [-15.1697, -12.5302], [-16.9741, -12.2826]
                ],
                "left_boundary": np.array([ [-3.60881, -10.4589], [-4.86529, -11.5701], [-5.72838, -12.2248], [-6.78315, -12.7653], [-7.97994, -13.3085], [-9.34334, -13.8078], [-10.7096, -14.1655], [-11.8888, -14.4014], [-13.2611, -14.4523], [-14.0646, -14.5272], [-15.2951, -14.5634], [-16.2532, -14.6061], [-17.2705, -14.6499], [-18.1228, -14.655], [-18.6563, -14.6183], [-19.3209, -14.537], [-19.843, -14.4765] ]),
                "right_boundary": np.array([ [-8.4099, -7.15374], [-9.97814, -9.26286], [-11.1396, -9.79368], [-12.6838, -10.1313], [-13.5649, -10.1724], [-14.6982, -10.0414] ])
            },
            -7010: {
                "length": 9.57,
                "centerline_points": [
                    [-16.9741, -12.2826], [-17.8822, -11.8637], [-18.5469, -11.1803], [-18.9396, -10.4797], [-19.1558, -9.72833], [-19.1452, -9.06693], [-19.088, -8.69983], [-19.0182, -8.30878], [-18.876, -7.71561], [-18.7455, -7.32025], [-18.5185, -6.94107], [-18.247, -6.59919], [-18.1226, -6.44341]
                ],
                "left_boundary": np.array([ [-19.843, -14.4765], [-20.0488, -14.2445], [-20.6685, -13.4421], [-21.0998, -12.5887], [-21.2747, -11.5296], [-21.1634, -10.5769], [-21.001, -9.81215], [-20.8453, -9.30137], [-20.4942, -8.50334], [-20.1515, -7.87078], [-19.8544, -7.33357], [-19.5098, -6.80134], [-19.2798, -6.46028], [-19.0833, -6.22024], [-18.8617, -5.91098] ]),
                "right_boundary": np.array([ [-14.6982, -10.0414], [-15.5572, -9.70419], [-16.1924, -9.31535], [-16.589, -9.01624], [-16.7945, -8.80112], [-16.8765, -8.71528], [-17.0159, -8.56936], [-17.1365, -8.44306], [-17.2571, -8.31679], [-17.3749, -8.19354], [-17.5994, -7.91463], [-17.6912, -7.76296], [-17.695, -7.57412], [-17.6326, -7.4371], [-17.5761, -7.3002], [-17.3544, -6.93731] ])
            },
            -6994: {
                "length": 11.06,
                "centerline_points": [
                    [-18.1226, -6.44341], [-17.2651, -5.3695], [-16.446, -4.29063], [-15.6737, -3.23631], [-14.7634, -1.97852], [-13.8303, -0.673066], [-12.8511, 0.692344], [-11.9415, 1.98553], [-11.8437, 2.12914], [-11.7293, 2.32918], [-11.6577, 2.4633]
                ],
                "left_boundary": np.array([ [-18.8617, -5.91098], [-12.341, 3.02842] ]), # Original left boundary: [-12.341, 3.02842], [-18.8617, -5.91098]
                "right_boundary": np.array([ [-17.3544, -6.93731], [-10.9541, 1.74536] ]) # Original right boundary: [-10.9541, 1.74536], [-17.3544, -6.93731]
            },
            -7001: {
                "length": 2.42,
                "centerline_points": [
                    [-11.6577, 2.4633], [-11.4953, 2.76773], [-11.3558, 3.19554], [-11.2719, 3.74032], [-11.2616, 3.97076], [-11.2662, 4.2009], [-11.3436, 4.37057], [-11.4012, 4.59375], [-11.4799, 4.82833], [-11.5845, 5.02698], [-11.7932, 5.25602], [-12.0189, 5.44046]
                ],
                "left_boundary": np.array([ [-12.341, 3.02842], [-11.943, 3.85099], [-11.7971, 4.25532], [-11.8741, 4.55781], [-12.3797, 5.12927] ]),
                "right_boundary": np.array([ [-10.9541, 1.74536], [-10.74, 2.29273], [-10.6227, 2.94443], [-10.6393, 3.78239], [-10.7374, 4.25272], [-10.9583, 4.94495], [-11.3114, 5.43384], [-11.8656, 5.91873] ])
            },
            -6993: {
                "length": 1.82,
                "centerline_points": [
                    [-12.0189, 5.44046], [-12.2932, 5.69477], [-12.5905, 5.91025], [-12.8963, 6.1049], [-13.1033, 6.24543], [-13.3877, 6.40804], [-13.6214, 6.57407]
                ],
                "left_boundary": np.array([ [-12.3797, 5.12927], [-12.6472, 5.3365], [-13.32, 5.83085], [-13.8598, 6.18616] ]),
                "right_boundary": np.array([ [-11.8656, 5.91873], [-12.3102, 6.25231], [-12.7781, 6.56182], [-13.3896, 6.95113] ])
            }
        }
        
        lanelet_ids = [-6995, -7019, -6996, -6999, -6985, -7011, -7010, -6994, -7001, -6993]
        # lanelet_ids = [-6999, -6985, -7011, -7010, -6994, -7001, -6993]
        
        return lanelet_dict, lanelet_ids
        
    def pose_callback(self, msg: RobotPose):
        """Handle incoming robot pose updates."""
        self.current_robot_pose = msg
        
        # Check if we've reached the goal
        if self.goal_position is not None:
            current_pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])
            
            dist_to_goal = np.linalg.norm(current_pos - self.goal_position)
            if dist_to_goal < 0.3:  # 30cm threshold
                if not self.is_goal_reached:
                    self.get_logger().info("Goal reached!")
                    self.is_goal_reached = True
                    
    def _trajectory_info_to_msg(self, trajectory: TrajectoryInfo) -> Trajectory:
        """Convert TrajectoryInfo to ROS message."""
        msg = Trajectory()
        
        for point in trajectory.planning_points:
            traj_point = TrajectoryPoint()
            
            # Set timestamp
            traj_point.timestamp = float(point.time)
            
            # Set pose
            traj_point.pose.header.frame_id = "map"
            traj_point.pose.header.stamp = self.get_clock().now().to_msg()
            
            traj_point.pose.pose.position.x = float(point.position[0])
            traj_point.pose.pose.position.y = float(point.position[1])
            traj_point.pose.pose.position.z = 0.0
            
            # Convert heading to quaternion
            quat = Quaternion()
            quat.x = 0.0
            quat.y = 0.0
            quat.z = math.sin(point.heading * 0.5)
            quat.w = math.cos(point.heading * 0.5)
            traj_point.pose.pose.orientation = quat
            
            # Set velocity and yaw rate
            traj_point.velocity = float(point.velocity)
            # traj_point.yaw_rate = float(point.yaw_rate)
            
            msg.points.append(traj_point)
            
        return msg
    
    def save_trajectory_data_for_animation(self, optimized_traj: TrajectoryInfo, 
                        initial_traj: TrajectoryInfo,
                        window_idx: int):
        """
        Save complete trajectory data for one window.
        Args:
            optimized_traj: Optimized trajectory from window manager
            initial_traj: Initial trajectory before optimization
            window_idx: Index of current window
        """
        # window_dir = os.path.join(self.data_dir, f'window_{window_idx}')
        # Get current ROS time when trajectory is published
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Create directory with window index and publish time
        window_dir = os.path.join(self.data_dir, f'window_{window_idx}_time_{current_time:.3f}')
        os.makedirs(window_dir, exist_ok=True)
        
        # 1. Save Initial Trajectory
        initial_points_data = []
        for point in initial_traj.planning_points:
            point_dict = {
                'x': point.position[0],
                'y': point.position[1],
                'velocity': point.velocity,
                'curvature': point.curvature,
                'heading': point.heading,
                'time': point.time,
                'arc_length': point.arc_length
            }
            initial_points_data.append(point_dict)
        
        df_initial = pd.DataFrame(initial_points_data)
        initial_path = os.path.join(window_dir, f'initial_trajectory_{window_idx}.csv')
        df_initial.to_csv(initial_path, index=False)
        
        # 2. Save Optimized Trajectory
        optimized_points_data = []
        for point in optimized_traj.planning_points:
            point_dict = {
                'x': point.position[0],
                'y': point.position[1],
                'velocity': point.velocity,
                'curvature': point.curvature,
                'heading': point.heading,
                'time': point.time,
                'arc_length': point.arc_length
            }
            optimized_points_data.append(point_dict)
        
        df_optimized = pd.DataFrame(optimized_points_data)
        optimized_path = os.path.join(window_dir, f'optimized_trajectory_{window_idx}.csv')
        df_optimized.to_csv(optimized_path, index=False)
        
        # 3. Save Distance Map Data with extended boundaries
        dist_data = {
            'distance_map': self.window_manager.optimizer.distance_map,
            'X': self.window_manager.optimizer.X,  # Coordinate grid X values
            'Y': self.window_manager.optimizer.Y,  # Coordinate grid Y values
            'extended_left': self.window_manager.optimizer.extended_left,
            'extended_right': self.window_manager.optimizer.extended_right
        }
        dist_path = os.path.join(window_dir, f'distance_map_{window_idx}.npz')
        np.savez(dist_path, **dist_data)
        
        # 4. Save Current Window Boundaries
        boundaries = {
            'left_boundary': self.window_manager.current_window.left_boundary,
            'right_boundary': self.window_manager.current_window.right_boundary,
            'active_lanelet_ids': self.window_manager.current_window.active_lanelet_ids
        }
        bound_path = os.path.join(window_dir, f'boundaries_{window_idx}.npz')
        np.savez(bound_path, **boundaries)
        
    def save_trajectory_data(self, trajectory: TrajectoryInfo):
        """Save trajectory data and plots."""
        try:
            # Save planning points data
            points_data = []
            for point in trajectory.planning_points:
                point_dict = {
                    'x': point.position[0],
                    'y': point.position[1],
                    'velocity': point.velocity,
                    'curvature': point.curvature,
                    'heading': point.heading,
                    'time': point.time,
                    'arc_length': point.arc_length
                }
                points_data.append(point_dict)
                
            df = pd.DataFrame(points_data)
            csv_path = os.path.join(self.data_dir, f'window_{self.window_index}_data.csv')
            df.to_csv(csv_path, index=False)
            
            # Generate visualization plots
            plt.figure(figsize=(15, 10))
            
            # Plot trajectory
            plt.subplot(2, 2, 1)
            plt.plot([p.position[0] for p in trajectory.planning_points],
                    [p.position[1] for p in trajectory.planning_points], 'b-', label='Path')
            plt.plot([w[0] for w in trajectory.waypoints],
                    [w[1] for w in trajectory.waypoints], 'ro', label='Waypoints')
            plt.title(f'Window {self.window_index} - Trajectory')
            plt.grid(True)
            plt.legend()
            plt.axis('equal')
            
            # Plot velocity profile
            plt.subplot(2, 2, 2)
            arc_lengths = [p.arc_length for p in trajectory.planning_points]
            velocities = [p.velocity for p in trajectory.planning_points]
            plt.plot(arc_lengths, velocities, 'g-')
            plt.title('Velocity Profile')
            plt.xlabel('Arc Length (m)')
            plt.ylabel('Velocity (m/s)')
            plt.grid(True)
            
            # Plot curvature
            plt.subplot(2, 2, 3)
            curvatures = [p.curvature for p in trajectory.planning_points]
            plt.plot(arc_lengths, curvatures, 'r-')
            plt.title('Curvature Profile')
            plt.xlabel('Arc Length (m)')
            plt.ylabel('Curvature (1/m)')
            plt.grid(True)
            
            # Plot time history
            plt.subplot(2, 2, 4)
            times = [p.time for p in trajectory.planning_points]
            plt.plot(times, velocities, 'b-')
            plt.title('Velocity vs Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity (m/s)')
            plt.grid(True)
            
            plt.tight_layout()
            plot_path = os.path.join(self.plots_dir, f'window_{self.window_index}_plots.png')
            plt.savefig(plot_path)
            plt.close()
            
            # Store window stats
            self.window_stats.append({
                'window_index': self.window_index,
                'sim_time': self.get_clock().now().nanoseconds / 1e9,
                'num_points': len(trajectory.planning_points),
                'duration': trajectory.planning_points[-1].time,
                'start_velocity': trajectory.planning_points[0].velocity,
                'end_velocity': trajectory.planning_points[-1].velocity,
                'max_velocity': max(p.velocity for p in trajectory.planning_points),
                'mean_velocity': np.mean([p.velocity for p in trajectory.planning_points]),
                'total_distance': trajectory.planning_points[-1].arc_length
            })
            
            # Save overall statistics periodically
            if self.window_index % 5 == 0:  # Save every 5 windows
                stats_df = pd.DataFrame(self.window_stats)
                stats_path = os.path.join(self.data_dir, 'overall_statistics.csv')
                stats_df.to_csv(stats_path, index=False)
            
            self.get_logger().info(
                f"Saved trajectory data and plots for window {self.window_index}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error saving trajectory data: {str(e)}")

    # def planning_loop(self):
    #     """
    #     Main planning loop that generates trajectory windows.
    #     Handles timing, trajectory generation, and state management.
    #     """
    #     # Prevent concurrent planning operations
    #     if not self.planning_lock.acquire(blocking=False):
    #         # Planning already in progress
    #         return
            
    #     try:
    #         # Skip if goal already reached
    #         if self.is_goal_reached:
    #             return
                
    #         # Wait for initial pose
    #         if self.current_robot_pose is None:
    #             # Use regular warn with current timestamp
    #             now = self.get_clock().now()
    #             if (now.nanoseconds / 1e9) % 5 < 0.1:  # Throttle to roughly every 5 seconds
    #                 self.get_logger().warn(
    #                     "Waiting for robot pose..."
    #                 )
    #             return
                
    #         current_time = self.get_clock().now().nanoseconds / 1e9
    #         self.get_logger().debug(
    #             f"\nPlanning Loop Status:"
    #             f"\n  Current Time: {current_time:.2f}"
    #             f"\n  Last Planning Time: {self.last_planning_time}"
    #             f"\n  Next Planning Time: {self.window_manager.next_planning_time}"
    #         )
            
    #         # Check if we need a new trajectory
    #         should_plan = (
    #             # First trajectory
    #             self.last_planning_time is None or
    #             # Previous trajectory exists but it's time for next window
    #             (self.window_manager.next_planning_time is not None and 
    #             current_time >= self.window_manager.next_planning_time)
    #         )
            
    #         if not should_plan:
    #             return
                
    #         # Get current robot position
    #         robot_position = np.array([
    #             self.current_robot_pose.pose.pose.position.x,
    #             self.current_robot_pose.pose.pose.position.y
    #         ])
            
    #         # Start planning time measurement
    #         plan_start_time = time.time()
            
    #         try:
    #             # Process window
    #             trajectory = self.window_manager.process_window(robot_position)
    #         except Exception as e:
    #             self.get_logger().error(f"Error processing window: {str(e)}")
    #             return
            
    #         # Check planning time
    #         planning_duration = time.time() - plan_start_time
    #         if planning_duration > self.window_config.planning_time:
    #             self.get_logger().warn(
    #                 f"Planning took longer than configured time: "
    #                 f"{planning_duration:.2f}s > {self.window_config.planning_time:.2f}s"
    #             )
            
    #         if trajectory and trajectory.planning_points:
    #             try:
    #                 # Convert to ROS message
    #                 traj_msg = self._trajectory_info_to_msg(trajectory)
                    
    #                 # Publish trajectory
    #                 self.trajectory_publisher.publish(traj_msg)
                    
    #                 # Update timing
    #                 self.last_planning_time = current_time
                    
    #                 # Log success
    #                 self.get_logger().info(
    #                     f"Published trajectory window {self.window_index}:\n"
    #                     f"  Points: {len(trajectory.planning_points)}\n"
    #                     f"  Duration: {trajectory.planning_points[-1].time:.2f}s\n"
    #                     f"  Planning time: {planning_duration:.3f}s"
    #                 )
                    
    #                 # Save trajectory data
    #                 self.save_trajectory_data(trajectory)
    #                 self.window_index += 1
                    
    #                 # Check for goal
    #                 if self.window_manager.current_window_is_last:
    #                     self.get_logger().info(
    #                         "Generated final trajectory window - approaching goal"
    #                     )
                        
    #                     try:
    #                         # Save final statistics
    #                         stats_df = pd.DataFrame(self.window_stats)
    #                         stats_path = os.path.join(self.data_dir, 'final_statistics.csv')
    #                         stats_df.to_csv(stats_path, index=False)
                            
    #                         # Plot entire path
    #                         self._plot_complete_path()
    #                     except Exception as e:
    #                         self.get_logger().error(f"Error saving final data: {str(e)}")
                            
    #             except Exception as e:
    #                 self.get_logger().error(f"Error publishing trajectory: {str(e)}")
                    
    #         else:
    #             self.get_logger().error(
    #                 f"Failed to generate trajectory for window {self.window_index}"
    #             )
                
    #             if self.window_manager.current_window_is_last:
    #                 self.get_logger().warn(
    #                     "Failed to generate final window - check if goal is reachable"
    #                 )
                    
    #     except Exception as e:
    #         self.get_logger().error(f"Error in planning loop: {str(e)}")
            
    #     finally:
    #         # Always release planning lock
    #         self.planning_lock.release()
    
    def planning_loop(self):
        """Main planning loop that generates trajectory windows."""
        # Prevent concurrent planning operations
        if not self.planning_lock.acquire(blocking=False):
            # Planning already in progress
            return
                
        try:
            # Stop planning if:
            # 1. Goal physically reached
            # 2. Last window was successfully processed
            if (self.is_goal_reached or 
                (self.window_manager.current_window_is_last and self.window_manager.current_trajectory)):
                
                # Cancel the planning timer
                self.planning_timer.cancel()
                self.get_logger().info("Planning completed - timer cancelled")
                
                # Save final statistics if not already saved
                if not hasattr(self, '_final_stats_saved') or not self._final_stats_saved:
                    try:
                        stats_df = pd.DataFrame(self.window_stats)
                        stats_path = os.path.join(self.data_dir, 'final_statistics.csv')
                        stats_df.to_csv(stats_path, index=False)
                        self._plot_complete_path()
                        self._final_stats_saved = True
                    except Exception as e:
                        self.get_logger().error(f"Error saving final data: {str(e)}")
                
                return
                        
            # Wait for initial pose
            if self.current_robot_pose is None:
                # Use regular warn with current timestamp
                now = self.get_clock().now()
                if (now.nanoseconds / 1e9) % 5 < 0.1:  # Throttle to roughly every 5 seconds
                    self.get_logger().warn("Waiting for robot pose...")
                return
                    
            current_time = time.time()  # Use wall time for simpler comparison
                
            # Check if we can plan next window
            should_plan = (
                self.next_planning_time is None or  # First window
                current_time >= self.next_planning_time  # Waited enough after last window
            )
                
            if not should_plan:
                return
                    
            # Get current robot position
            robot_position = np.array([
                self.current_robot_pose.pose.pose.position.x,
                self.current_robot_pose.pose.pose.position.y
            ])
                
            # Start planning time measurement
            plan_start_time = time.time()
                
            try:
                # Process window
                self.get_logger().info(f"Starting to process window {self.window_index}")
                trajectory, initial_trajectory = self.window_manager.process_window(robot_position)
                    
                # Log window processing results
                if trajectory:
                    self.get_logger().debug(
                        f"Window {self.window_index} processing complete:"
                        f"\n  Points: {len(trajectory.planning_points) if trajectory.planning_points else 0}"
                        f"\n  Waypoints: {len(trajectory.waypoints)}"
                    )
                else:
                    self.get_logger().error(f"Window {self.window_index} processing returned None")
                    return  # Don't proceed if window processing failed
                    
            except Exception as e:
                self.get_logger().error(f"Error processing window {self.window_index}: {str(e)}")
                return
                
            # Check planning duration
            planning_duration = time.time() - plan_start_time
                
            if trajectory and trajectory.planning_points:
                try:
                    # Verify trajectory has all required data
                    if not self._verify_trajectory(trajectory):
                        self.get_logger().error(f"Invalid trajectory data for window {self.window_index}")
                        return
                        
                    # Convert to ROS message
                    traj_msg = self._trajectory_info_to_msg(trajectory)
                        
                    # Publish trajectory
                    self.trajectory_publisher.publish(traj_msg)
                        
                    # Set next planning time only after successful publish
                    self.next_planning_time = time.time() + self.window_sleep_time
                        
                    # Log success with timing info
                    self.get_logger().info(
                        f"Published trajectory window {self.window_index}:\n"
                        f"  Points: {len(trajectory.planning_points)}\n"
                        f"  Duration: {trajectory.planning_points[-1].time:.2f}s\n"
                        f"  Planning time: {planning_duration:.3f}s\n"
                        f"  Next window in: {self.window_sleep_time*1000:.0f}ms"
                    )
                    
                    # Save trajectory data for animation
                    self.save_trajectory_data_for_animation(
                        optimized_traj=trajectory,
                        initial_traj=initial_trajectory,
                        window_idx=self.window_index
                    )
                        
                    # Save trajectory data
                    self.save_trajectory_data(trajectory)
                        
                    # Only increment window index after successful processing and saving
                    self.window_index += 1
                        
                    # Check for goal
                    if self.window_manager.current_window_is_last:
                        self.get_logger().info(
                            "Generated final trajectory window - approaching goal"
                        )
                        try:
                            # Save final statistics
                            stats_df = pd.DataFrame(self.window_stats)
                            stats_path = os.path.join(self.data_dir, 'final_statistics.csv')
                            stats_df.to_csv(stats_path, index=False)
                                
                            # Plot entire path
                            self._plot_complete_path()
                        except Exception as e:
                            self.get_logger().error(f"Error saving final data: {str(e)}")
                                
                except Exception as e:
                    self.get_logger().error(f"Error publishing trajectory: {str(e)}")
                    return
                        
            else:
                self.get_logger().error(
                    f"Failed to generate trajectory for window {self.window_index}"
                )
                # Don't update next_planning_time on failure
                return
                    
        except Exception as e:
            self.get_logger().error(f"Error in planning loop: {str(e)}")
                
        finally:
            # Always release planning lock
            self.planning_lock.release()
            
    def _verify_trajectory(self, trajectory: TrajectoryInfo) -> bool:
        """Verify trajectory has all required data."""
        try:
            if not trajectory.planning_points:
                self.get_logger().error("No planning points in trajectory")
                return False
                
            # Check first and last points to verify data
            points_to_check = [trajectory.planning_points[0], trajectory.planning_points[-1]]
            for point in points_to_check:
                # Verify required attributes
                if not hasattr(point, 'velocity') or point.velocity is None:
                    self.get_logger().error("Missing velocity data in trajectory point")
                    return False
                    
                if not hasattr(point, 'position') or point.position is None:
                    self.get_logger().error("Missing position data in trajectory point")
                    return False
                    
                if not hasattr(point, 'heading') or point.heading is None:
                    self.get_logger().error("Missing heading data in trajectory point")
                    return False
                    
                if not hasattr(point, 'time') or point.time is None:
                    self.get_logger().error("Missing time data in trajectory point")
                    return False
                    
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error verifying trajectory: {str(e)}")
            return False

    def _plot_complete_path(self):
        """Generate plot showing complete planned path."""
        try:
            plt.figure(figsize=(15, 10))
            
            # Plot lanelet boundaries
            for lanelet_id in self.lanelet_ids:
                lanelet = self.lanelet_dict[lanelet_id]
                plt.plot(lanelet['left_boundary'][:, 0], 
                        lanelet['left_boundary'][:, 1], 'k-', alpha=0.3)
                plt.plot(lanelet['right_boundary'][:, 0], 
                        lanelet['right_boundary'][:, 1], 'k-', alpha=0.3)
            
            # Plot centerline points
            centerline_points = np.array(self.window_manager.centerline_processor.processed_points)
            plt.plot(centerline_points[:, 0], centerline_points[:, 1], 
                    'b--', alpha=0.5, label='Centerline')
            
            # Plot start and goal
            plt.plot(centerline_points[0, 0], centerline_points[0, 1], 
                    'go', markersize=12, label='Start')
            plt.plot(self.goal_position[0], self.goal_position[1], 
                    'ro', markersize=12, label='Goal')
            
            plt.title('Complete Planned Path')
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.grid(True)
            plt.axis('equal')
            plt.legend()
            
            # Save plot
            plot_path = os.path.join(self.plots_dir, 'complete_path.png')
            plt.savefig(plot_path)
            plt.close()
            
        except Exception as e:
            self.get_logger().error(f"Error plotting complete path: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    node = LocalPlannerNode()
    
    # Use MultiThreadedExecutor for concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()