#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from amr_interfaces.action import NavigateToGoal
from amr_interfaces.msg import RobotPose, Trajectory

from .amr_trajectory_planner_params import trajectory_planner
from .lanelet_map_manager import LaneletMapManager
from amr_utils_python.coordinate_transforms_py import GPSPoint
from .config_types import *

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
        """Execute the goal"""
        self.get_logger().info("Executing goal...")
        self._goal_executing = True

        # Extract goal details
        goal = goal_handle.request
        feedback_msg = NavigateToGoal.Feedback()
        result = NavigateToGoal.Result()

        lanelet_count = len(goal.lanelet_ids)
        self.get_logger().info(f"Planning trajectory through {lanelet_count} lanelets")
        self.get_logger().info(f"Goal GPS: [{goal.goal_gps[0]}, {goal.goal_gps[1]}]")

        # Initialize distance and time (for simulation)
        total_distance = (
            goal.total_distance if goal.total_distance > 0 else 100.0
        )  # Default to 100m if not specified
        total_time_estimate = (
            goal.estimated_time if goal.estimated_time > 0 else 60.0
        )  # Default to 60s if not specified

        try:
            self.get_logger().info(
                f"Initializing Lanelet Map Manager with {self.params.map_data.map_path}"
            )
            map_origin_gps = GPSPoint(
                self.params.map_data.gps_origin.latitude,
                self.params.map_data.gps_origin.longitude,
                self.params.map_data.gps_origin.altitude,
            )
            map_manager = LaneletMapManager(self, self.params.map_data.map_path, map_origin_gps)

            # Load the map
            if not map_manager.load_map():
                self.get_logger().error("Failed to load lanelet map")
                raise RuntimeError("Failed to load lanelet map")

            # Build routing graph
            if not map_manager.build_routing_graph():
                self.get_logger().error("Failed to build routing graph")
                raise RuntimeError("Failed to build routing graph")

            self.get_logger().info("Successfully loaded lanelet map and built routing graph")

            # Process each lanelet from the goal
            self.get_logger().info("Processing lanelets from goal:")
            for i, lanelet_id in enumerate(goal.lanelet_ids):
                # Get the correct inverted flag from the client message
                inverted = goal.is_inverted[i]

                self.get_logger().info(f"Loading lanelet {lanelet_id}, inverted: {inverted}")

                # Get the lanelet
                lanelet = map_manager.get_lanelet_by_id(lanelet_id, inverted)

                if lanelet is not None:
                    # Print some lanelet information
                    self.get_logger().info(f"  - Lanelet ID: {lanelet.id}")

                    # Get lanelet attributes
                    attributes = []
                    for key, value in lanelet.attributes.items():
                        attributes.append(f"{key}: {value}")
                    self.get_logger().info(f"  - Attributes: {', '.join(attributes)}")

                    # Print information about the lanelet points
                    self.get_logger().info(
                        f"  - Centerline point count: {len(lanelet.centerline)}"
                    )
                    self.get_logger().info(f"  - Left bound point count: {len(lanelet.leftBound)}")
                    self.get_logger().info(
                        f"  - Right bound point count: {len(lanelet.rightBound)}"
                    )

                    # Print first and last centerline points if available
                    if len(lanelet.centerline) > 0:
                        first_point = lanelet.centerline[0]
                        last_point = lanelet.centerline[-1]
                        self.get_logger().info(
                            f"  - First centerline point: \
                                ({first_point.x:.2f}, {first_point.y:.2f})"
                        )
                        self.get_logger().info(
                            f"  - Last centerline point: ({last_point.x:.2f}, {last_point.y:.2f})"
                        )

                    # Check if lanelet is navigable by AMR
                    is_navigable = (
                        "amr_navigable" in lanelet.attributes
                        and lanelet.attributes["amr_navigable"] == "yes"
                    )
                    self.get_logger().info(f"  - AMR Navigable: {is_navigable}")

                    # Check if lanelet is one-way
                    is_one_way = (
                        "one_way" in lanelet.attributes and lanelet.attributes["one_way"] == "yes"
                    )
                    self.get_logger().info(f"  - One Way: {is_one_way}")

                    # Check if lanelet is bidirectional
                    is_bidir = (
                        "bidirectional" in lanelet.attributes
                        and lanelet.attributes["bidirectional"] == "yes"
                    )
                    self.get_logger().info(f"  - Bidirectional: {is_bidir}")
                else:
                    self.get_logger().error(f"  - Failed to get lanelet with ID: {lanelet_id}")

            # Continue with the existing simulation code
            self.get_logger().info("Continuing with trajectory execution simulation...")

            # Simulate planning and execution process
            total_steps = 20  # More steps for a smoother simulation
            for i in range(1, total_steps + 1):
                # Check if goal was canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    self._goal_executing = False

                    # Set partial results for canceled goal
                    result.status = GoalStatus(status=GoalStatus.STATUS_CANCELED)
                    result.message = "Navigation canceled by user"
                    result.total_distance = total_distance * (i / total_steps)
                    result.total_time = total_time_estimate * (i / total_steps)
                    return result

                # Simulate some planning/execution work
                time.sleep(0.5)  # Simulate work

                # Calculate remaining progress (simulate as a percentage of completion)
                progress_percent = i / total_steps
                distance_remaining = total_distance * (1.0 - progress_percent)
                time_remaining = total_time_estimate * (1.0 - progress_percent)

                # Update and publish feedback
                feedback_msg.distance_remaining = distance_remaining
                feedback_msg.estimated_time_remaining = time_remaining
                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(
                    f"Navigation progress: {progress_percent*100:.1f}%, "
                    + f"Distance remaining: {distance_remaining:.1f}m, "
                    + f"Time remaining: {time_remaining:.1f}s"
                )

            # Example of setting a successful result
            result.status = GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)
            result.message = "Navigation completed successfully"
            result.total_distance = total_distance
            result.total_time = total_time_estimate

            goal_handle.succeed()
            self.get_logger().info("Goal succeeded")

        except Exception as e:
            # Handle any exceptions during execution
            self.get_logger().error(f"Goal execution failed: {str(e)}")
            goal_handle.abort()
            result.status = GoalStatus(status=GoalStatus.STATUS_ABORTED)
            result.message = f"Failed to plan trajectory: {str(e)}"
            result.total_distance = 0.0
            result.total_time = 0.0

        finally:
            self._goal_executing = False

        return result

    def execute_callback1(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info("Executing goal...")
        self._goal_executing = True

        # Extract goal details
        goal = goal_handle.request
        feedback_msg = NavigateToGoal.Feedback()
        result = NavigateToGoal.Result()

        # Initialize necessary components
        try:
            # Initialize map and planning components
            map_manager = self._initialize_map_manager(goal)
            window_manager = self._initialize_window_manager(map_manager, goal)

            # Planning state variables
            window_index = 0
            planning_complete = False
            robot_position = self._get_initial_position(
                goal
            )  # Either from goal or from pose service

            # Main planning loop
            while not planning_complete:
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self._handle_cancellation(goal_handle, result, window_index)
                    return result

                # Process current window
                try:
                    window_start_time = time.time()

                    # Generate trajectory for current window
                    current_trajectory, initial_trajectory = window_manager.process_window(
                        robot_position, window_index
                    )

                    # Verify trajectory
                    if not self._verify_trajectory(current_trajectory):
                        raise RuntimeError(f"Invalid trajectory data for window {window_index}")

                    # Simulate/execute trajectory
                    self._execute_trajectory(current_trajectory)

                    # Update position for next window
                    robot_position = self._get_updated_position(current_trajectory)

                    # Check if planning is complete
                    if window_manager.is_final_window(window_index):
                        planning_complete = True
                        self.get_logger().info("Final window completed, goal achieved")

                    # Save window data if needed
                    self._save_window_data(current_trajectory, initial_trajectory, window_index)

                    # Increment window index
                    window_index += 1

                    # Update and send feedback
                    self._update_and_send_feedback(
                        goal_handle, feedback_msg, window_index, window_manager, current_trajectory
                    )

                except Exception as e:
                    self.get_logger().error(f"Error processing window {window_index}: {str(e)}")
                    # Decide whether to retry, skip window, or abort based on error type
                    if self._is_fatal_error(e):
                        raise  # Re-throw to be caught by outer try/except
                    else:
                        self.get_logger().warn(f"Attempting to continue with next window")
                        window_index += 1  # Skip problematic window

            # Success case
            goal_handle.succeed()
            result.status = GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)
            result.message = "Navigation completed successfully"
            result.total_distance = window_manager.get_total_distance()
            result.total_time = window_manager.get_total_time()

        except Exception as e:
            # Handle all exceptions during execution
            self.get_logger().error(f"Goal execution failed: {str(e)}")
            goal_handle.abort()
            result.status = GoalStatus(status=GoalStatus.STATUS_ABORTED)
            result.message = f"Failed to plan trajectory: {str(e)}"
            result.total_distance = 0.0
            result.total_time = 0.0

        finally:
            self._goal_executing = False
            self._cleanup_resources()

        return result


def main(args=None):
    rclpy.init(args=args)

    trajectory_planner_node = TrajectoryPlannerActionServer()

    # Use MultiThreadedExecutor to handle the action server callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(trajectory_planner_node)

    try:
        trajectory_planner_node.get_logger().info("Beginning trajectory planner execution loop")
        executor.spin()
    except KeyboardInterrupt:
        trajectory_planner_node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        executor.shutdown()
        trajectory_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
