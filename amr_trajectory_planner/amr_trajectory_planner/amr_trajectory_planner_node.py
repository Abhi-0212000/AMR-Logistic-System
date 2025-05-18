#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
import math
from amr_interfaces.action import NavigateToGoal
from amr_interfaces.msg import RobotPose, Trajectory, TrajectoryPoint

from .amr_trajectory_planner_params import trajectory_planner
from .lanelet_map_manager import LaneletMapManager
from amr_utils_python.coordinate_transforms_py import GPSPoint
from .config_types import *
import numpy as np
from .core_modules.window_manager import WindowManager


class TrajectoryPlannerActionServer(Node):
    def __init__(self):
        super().__init__("trajectory_planner_action_server")

        # Initialize parameter listener
        self.param_listener = trajectory_planner.ParamListener(self)
        self.params = self.param_listener.get_params()

        # Initialize trajectory planner parameters
        self.init_trajectory_planner_configs()

        # Print all parameter values to demonstrate parameter access
        self.print_parameters()

        self.set_up_communication_interfaces()

        # Flag to track if goal is being executed
        self._goal_executing = False

        self.get_logger().info("Trajectory planner action server started")

    def set_up_communication_interfaces(self):
        """Set up the communication interfaces for the trajectory planner"""
        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToGoal,  # The action type
            "navigate_to_goal",  # Action name
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Action server for trajectory planner initialized")

        self.pose_subscriber = self.create_subscription(
            RobotPose,
            "robot_pose",
            self.robot_pose_callback,
            10,
        )
        self.get_logger().info("Subscription to robot pose topic initialized")

        self.trajectory_publisher = self.create_publisher(
            Trajectory,
            "trajectory",
            10,
        )
        self.get_logger().info("Publisher for trajectory topic initialized")

    def robot_pose_callback(self, msg):
        """Store the latest robot pose"""
        # Save the latest robot pose for planning
        self.current_pose = msg
        self.get_logger().debug(f"Received robot pose: ({msg.position.x:.2f}, {msg.position.y:.2f})")

    def init_trajectory_planner_configs(self):
        """Initialize the trajectory planner configurations from parameters"""
        self.window_manager_config = WindowConfig(
            planning_time=self.params.window_manager.planning_time,
            buffer_time=self.params.window_manager.buffer_time,
            lookahead_points=self.params.window_manager.lookahead_points,
        )
        self.centerline_processor_config = CenterlineProcessorConfig(
            interpolation_method=self.params.centerline_processor.interpolation_method,
            target_spacing=self.params.centerline_processor.target_spacing,
            spacing_tolerance=self.params.centerline_processor.spacing_tolerance,
            bezier_window_size=self.params.centerline_processor.bezier_window_size,
            bezier_overlap=self.params.centerline_processor.bezier_overlap,
        )
        self.trajectory_optimization_config = TrajectoryOptimizerConfig(
            trajectory_point_count=self.params.trajectory_optimization.trajectory_point_count,
            base_tangent_factor=self.params.trajectory_optimization.base_tangent_factor,
            optimization_time_limit=self.params.trajectory_optimization.optimization_time_limit,
            arc_length_calculation_method=self.params.trajectory_optimization.arc_length_calculation_method,
            collision_checking=CollisionCheckingConfig(
                safety_margin=self.params.trajectory_optimization.collision_checking.safety_margin,
                collision_check_interval=self.params.trajectory_optimization.collision_checking.collision_check_interval,
            ),
            rprop=RPROPConfig(
                initial_step_size=self.params.trajectory_optimization.rprop.initial_step_size,
                minimum_step_size=self.params.trajectory_optimization.rprop.minimum_step_size,
                maximum_step_size=self.params.trajectory_optimization.rprop.maximum_step_size,
                increase_factor=self.params.trajectory_optimization.rprop.increase_factor,
                decrease_factor=self.params.trajectory_optimization.rprop.decrease_factor,
            ),
        )
        self.distance_map_config = DistanceMapConfig(
            resolution=self.params.distance_map.resolution,
            window_size=self.params.distance_map.window_size,
            use_sobel=self.params.distance_map.use_sobel,
            sobel_threshold=self.params.distance_map.sobel_threshold,
        )
        self.robot_constraints_config = RobotConstraintsConfig(
            v_max=self.params.robot_constraints.v_max,
            omega_max=self.params.robot_constraints.omega_max,
            a_t_max=self.params.robot_constraints.a_t_max,
            a_r_max=self.params.robot_constraints.a_r_max,
            f_max=self.params.robot_constraints.f_max,
            mass=self.params.robot_constraints.mass,
            t_react=self.params.robot_constraints.t_react,
            wheel_base=self.params.robot_constraints.wheel_base,
            min_turning_radius=self.params.robot_constraints.min_turning_radius,
        )

    def print_parameters(self):
        """Print all parameter values from the parameter library"""
        self.get_logger().info("--- AMR Trajectory Planner Parameters ---")

        # Window Manager Parameters
        self.get_logger().info("Window Manager Parameters:")
        self.get_logger().info(f"  planning_time: {self.params.window_manager.planning_time}")
        self.get_logger().info(f"  buffer_time: {self.params.window_manager.buffer_time}")
        self.get_logger().info(
            f"  lookahead_points: {self.params.window_manager.lookahead_points}"
        )

        # Centerline Processor Parameters
        self.get_logger().info("Centerline Processor Parameters:")
        self.get_logger().info(
            f"  interpolation_method: {self.params.centerline_processor.interpolation_method}"
        )
        self.get_logger().info(
            f"  target_spacing: {self.params.centerline_processor.target_spacing}"
        )
        self.get_logger().info(
            f"  spacing_tolerance: {self.params.centerline_processor.spacing_tolerance}"
        )
        self.get_logger().info(
            f"  bezier_window_size: {self.params.centerline_processor.bezier_window_size}"
        )
        self.get_logger().info(
            f"  bezier_overlap: {self.params.centerline_processor.bezier_overlap}"
        )

        # Trajectory Optimization Parameters
        self.get_logger().info("Trajectory Optimization Parameters:")
        self.get_logger().info(
            f"trajectory_point_count: {self.params.trajectory_optimization.trajectory_point_count}"
        )
        self.get_logger().info(
            f"base_tangent_factor: {self.params.trajectory_optimization.base_tangent_factor}"
        )
        self.get_logger().info(
            f"optimization_time_limit: \
                {self.params.trajectory_optimization.optimization_time_limit}"
        )
        self.get_logger().info(
            f"arc_length_calculation_method: \
                {self.params.trajectory_optimization.arc_length_calculation_method}"
        )

        # Collision Checking Parameters
        self.get_logger().info("Collision Checking Parameters:")
        self.get_logger().info(
            f"safety_margin: \
                {self.params.trajectory_optimization.collision_checking.safety_margin}"
        )
        self.get_logger().info(
            f"collision_check_interval: \
                {self.params.trajectory_optimization.collision_checking.collision_check_interval}"
        )

        # RPROP Parameters
        self.get_logger().info("RPROP Parameters:")
        self.get_logger().info(
            f"  initial_step_size: {self.params.trajectory_optimization.rprop.initial_step_size}"
        )
        self.get_logger().info(
            f"  minimum_step_size: {self.params.trajectory_optimization.rprop.minimum_step_size}"
        )
        self.get_logger().info(
            f"  maximum_step_size: {self.params.trajectory_optimization.rprop.maximum_step_size}"
        )
        self.get_logger().info(
            f"  increase_factor: {self.params.trajectory_optimization.rprop.increase_factor}"
        )
        self.get_logger().info(
            f"  decrease_factor: {self.params.trajectory_optimization.rprop.decrease_factor}"
        )

        # Distance Map Parameters
        self.get_logger().info("Distance Map Parameters:")
        self.get_logger().info(f"  resolution: {self.params.distance_map.resolution}")
        self.get_logger().info(f"  window_size: {self.params.distance_map.window_size}")
        self.get_logger().info(f"  use_sobel: {self.params.distance_map.use_sobel}")
        self.get_logger().info(f"  sobel_threshold: {self.params.distance_map.sobel_threshold}")

        # Robot Constraints Parameters
        self.get_logger().info("Robot Constraints Parameters:")
        self.get_logger().info(f"  v_max: {self.params.robot_constraints.v_max}")
        self.get_logger().info(f"  omega_max: {self.params.robot_constraints.omega_max}")
        self.get_logger().info(f"  a_t_max: {self.params.robot_constraints.a_t_max}")
        self.get_logger().info(f"  a_r_max: {self.params.robot_constraints.a_r_max}")
        self.get_logger().info(f"  f_max: {self.params.robot_constraints.f_max}")
        self.get_logger().info(f"  mass: {self.params.robot_constraints.mass}")
        self.get_logger().info(f"  t_react: {self.params.robot_constraints.t_react}")
        self.get_logger().info(f"  wheel_base: {self.params.robot_constraints.wheel_base}")
        self.get_logger().info(
            f"  min_turning_radius: {self.params.robot_constraints.min_turning_radius}"
        )

        self.get_logger().info(f"map_path: {self.params.map_data.map_path}")

    def goal_callback(self, goal_request):
        """
        Accept or reject a goal request.

        Implements a policy of one goal at a time - rejects new goals if one is already executing.
        """
        self.get_logger().info("Received goal request")

        # Check if we already have a goal
        if self._goal_executing:
            self.get_logger().info("Rejecting goal: Another goal is currently being executed")
            return GoalResponse.REJECT

        # Validate the goal request
        if len(goal_request.goal_gps) < 2:
            self.get_logger().error("Rejecting goal: Invalid GPS coordinates")
            return GoalResponse.REJECT

        if len(goal_request.lanelet_ids) == 0:
            self.get_logger().error("Rejecting goal: No lanelet IDs provided")
            return GoalResponse.REJECT

        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle a cancel request for an action goal"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal using window-based trajectory planning"""
        self.get_logger().info("Executing navigation goal...")
        self._goal_executing = True

        # Extract goal details
        goal = goal_handle.request
        feedback_msg = NavigateToGoal.Feedback()
        result = NavigateToGoal.Result()

        try:
            # Initialize map manager
            self.get_logger().info(f"Initializing map with {self.params.map_data.map_path}")
            map_origin_gps = GPSPoint(
                self.params.map_data.gps_origin.latitude,
                self.params.map_data.gps_origin.longitude,
                self.params.map_data.gps_origin.altitude,
            )
            map_manager = LaneletMapManager(self, self.params.map_data.map_path, map_origin_gps)

            # Load map and build routing graph
            if not map_manager.load_map():
                raise RuntimeError("Failed to load lanelet map")
            
            if not map_manager.build_routing_graph():
                raise RuntimeError("Failed to build routing graph")

            self.get_logger().info("Successfully loaded lanelet map and routing graph")
            
            # Process lanelet data for centerline processing
            centerline_points = []
            for i, lanelet_id in enumerate(goal.lanelet_ids):
                # Get the correct inverted flag from the client message
                inverted = goal.is_inverted[i]
                
                self.get_logger().info(f"Loading lanelet {lanelet_id}, inverted: {inverted}")
                
                # Get the lanelet
                lanelet = map_manager.get_lanelet_by_id(lanelet_id, inverted)
                if lanelet is not None:
                    centerline = [[point.x, point.y] for point in lanelet.centerline]
                    centerline_points.extend(centerline)
                    self.get_logger().info(
                        f"Added {len(lanelet.centerline)} points from lanelet {lanelet_id}"
                    )
                else:
                    self.get_logger().error(f"Failed to get lanelet with ID: {lanelet_id}")
                    raise RuntimeError(f"Lanelet {lanelet_id} not found in map")

            # Initialize trajectory planning components
            window_manager = WindowManager(self)
            
            # Wait for a valid robot position
            timeout_count = 0
            while not hasattr(self, 'current_pose') or self.current_pose is None:
                time.sleep(0.1)
                timeout_count += 1
                if timeout_count > 50:  # 5 second timeout
                    raise RuntimeError("Timeout waiting for robot pose data")
            
            # Get robot position as numpy array
            robot_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
            self.get_logger().info(f"Starting trajectory planning from position: {robot_position}")

            # Main planning loop - continue until goal reached
            window_index = 0
            planning_complete = False
            
            while not planning_complete:
                # Check if goal was canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    self._goal_executing = False
                    result.status = GoalStatus(status=GoalStatus.STATUS_CANCELED)
                    result.message = "Navigation canceled"
                    return result

                # Process current window
                try:
                    # Log starting window processing
                    self.get_logger().info(f"Processing window {window_index}")
                    start_time = self.get_clock().now()
                    
                    # Generate trajectory for current window
                    current_trajectory, initial_trajectory = window_manager.process_window(robot_position)

                    # Calculate processing time
                    elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                    
                    # Check if trajectory generation failed
                    if current_trajectory is None:
                        if window_manager.is_goal_reached():
                            # We're done - this is expected
                            planning_complete = True
                            self.get_logger().info("Goal reached - planning complete")
                        else:
                            # Unexpected failure
                            raise RuntimeError(f"Failed to generate trajectory for window {window_index}")
                    else:
                        # Publish trajectory
                        trajectory_msg = self._trajectory_info_to_msg(current_trajectory)
                        
                        self.trajectory_publisher.publish(trajectory_msg)
                        
                        # Update next planning time
                        next_planning_time = window_manager.calculate_next_planning_time()
                        
                        # Log success
                        self.get_logger().info(
                            f"Window {window_index} processed in {elapsed:.3f}s: "
                            f"{len(current_trajectory.planning_points)} points, "
                            f"duration: {current_trajectory.total_time:.2f}s"
                        )

                        # Check if goal reached
                        if window_manager.current_window_is_last:
                            planning_complete = True
                            self.get_logger().info("Final window processed - goal will be reached")

                        # Update and send feedback
                        distance_remaining = window_manager.get_remaining_distance()
                        time_remaining = window_manager.get_remaining_time()
                        feedback_msg.distance_remaining = distance_remaining
                        feedback_msg.estimated_time_remaining = time_remaining
                        goal_handle.publish_feedback(feedback_msg)
                        
                        self.get_logger().info(
                            f"Navigation progress: Distance remaining: {distance_remaining:.1f}m, "
                            f"Time remaining: {time_remaining:.1f}s"
                        )

                        # Get updated robot position for next iteration
                        if hasattr(self, 'current_pose') and self.current_pose is not None:
                            robot_position = np.array([
                                self.current_pose.position.x, 
                                self.current_pose.position.y
                            ])
                        
                        # Wait appropriate time before planning next window
                        if next_planning_time is not None and next_planning_time > 0.0:
                            self.get_logger().info(
                                f"Waiting {next_planning_time:.2f}s before planning next window"
                            )
                            # Sleep in small intervals to allow cancel checks
                            sleep_interval = 0.1
                            for _ in range(int(next_planning_time / sleep_interval)):
                                if goal_handle.is_cancel_requested:
                                    break
                                time.sleep(sleep_interval)
                        
                        # Increment window index
                        window_index += 1
                            
                except Exception as e:
                    import traceback
                    self.get_logger().error(
                        f"Error processing window {window_index}: {str(e)}\n{traceback.format_exc()}"
                    )
                    # Try to continue with next window if possible
                    window_index += 1
                    
                    # If too many consecutive errors, abort
                    if window_index > 3:  # Consider tracking consecutive error count
                        raise RuntimeError(f"Too many planning failures, aborting: {str(e)}")

            # Success case
            goal_handle.succeed()
            result.status = GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)
            result.message = "Navigation completed successfully"
            result.total_distance = window_manager.get_total_distance()
            result.total_time = sum(
                [t.total_time for t in window_manager.trajectories]
            ) if window_manager.trajectories else 0.0

        except Exception as e:
            import traceback
            # Handle all exceptions during execution
            self.get_logger().error(
                f"Goal execution failed: {str(e)}\n{traceback.format_exc()}"
            )
            goal_handle.abort()
            result.status = GoalStatus(status=GoalStatus.STATUS_ABORTED) 
            result.message = f"Failed to plan trajectory: {str(e)}"
            result.total_distance = 0.0
            result.total_time = 0.0

        finally:
            self._goal_executing = False

        return result

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


def main(args=None):
    rclpy.init(args=args)

    trajectory_planner_node = TrajectoryPlannerActionServer()

    # Use MultiThreadedExecutor to handle the action server callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(trajectory_planner_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        trajectory_planner_node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        executor.shutdown()
        trajectory_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
