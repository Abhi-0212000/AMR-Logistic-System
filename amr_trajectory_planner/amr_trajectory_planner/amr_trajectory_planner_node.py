#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from amr_interfaces.action import NavigateToGoal

from .amr_trajectory_planner_params import trajectory_planner


class TrajectoryPlannerActionServer(Node):
    
    def __init__(self):
        super().__init__('trajectory_planner_action_server')
        
        # Initialize parameter listener
        self.param_listener = trajectory_planner.ParamListener(self)
        self.params = self.param_listener.get_params()
        
        # Print all parameter values to demonstrate parameter access
        self.print_parameters()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToGoal,  # The action type
            'navigate_to_goal',  # Action name
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Flag to track if goal is being executed
        self._goal_executing = False
        
        self.get_logger().info('Trajectory planner action server started')

    def print_parameters(self):
        """Print all parameter values from the parameter library"""
        self.get_logger().info("--- AMR Trajectory Planner Parameters ---")
        
        # Window Manager Parameters
        self.get_logger().info("Window Manager Parameters:")
        self.get_logger().info(f"  planning_time: {self.params.window_manager.planning_time}")
        self.get_logger().info(f"  buffer_time: {self.params.window_manager.buffer_time}")
        self.get_logger().info(f"  lookahead_points: {self.params.window_manager.lookahead_points}")
        
        # Centerline Processor Parameters
        self.get_logger().info("Centerline Processor Parameters:")
        self.get_logger().info(f"  interpolation_method: {self.params.centerline_processor.interpolation_method}")
        self.get_logger().info(f"  target_spacing: {self.params.centerline_processor.target_spacing}")
        self.get_logger().info(f"  spacing_tolerance: {self.params.centerline_processor.spacing_tolerance}")
        self.get_logger().info(f"  bezier_window_size: {self.params.centerline_processor.bezier_window_size}")
        self.get_logger().info(f"  bezier_overlap: {self.params.centerline_processor.bezier_overlap}")
        
        # Trajectory Optimization Parameters
        self.get_logger().info("Trajectory Optimization Parameters:")
        self.get_logger().info(f"  trajectory_point_count: {self.params.trajectory_optimization.trajectory_point_count}")
        self.get_logger().info(f"  base_tangent_factor: {self.params.trajectory_optimization.base_tangent_factor}")
        self.get_logger().info(f"  optimization_time_limit: {self.params.trajectory_optimization.optimization_time_limit}")
        self.get_logger().info(f"  arc_length_calculation_method: {self.params.trajectory_optimization.arc_length_calculation_method}")
        
        # Collision Checking Parameters
        self.get_logger().info("Collision Checking Parameters:")
        self.get_logger().info(f"  safety_margin: {self.params.trajectory_optimization.collision_checking.safety_margin}")
        self.get_logger().info(f"  collision_check_interval: {self.params.trajectory_optimization.collision_checking.collision_check_interval}")
        
        # RPROP Parameters
        self.get_logger().info("RPROP Parameters:")
        self.get_logger().info(f"  initial_step_size: {self.params.trajectory_optimization.rprop.initial_step_size}")
        self.get_logger().info(f"  minimum_step_size: {self.params.trajectory_optimization.rprop.minimum_step_size}")
        self.get_logger().info(f"  maximum_step_size: {self.params.trajectory_optimization.rprop.maximum_step_size}")
        self.get_logger().info(f"  increase_factor: {self.params.trajectory_optimization.rprop.increase_factor}")
        self.get_logger().info(f"  decrease_factor: {self.params.trajectory_optimization.rprop.decrease_factor}")
        
        # Distance Map Parameters
        self.get_logger().info("Distance Map Parameters:")
        self.get_logger().info(f"  resolution: {self.params.distance_map.resolution}")
        self.get_logger().info(f"  window_size: {self.params.distance_map.window_size}")
        self.get_logger().info(f"  use_sobel: {self.params.distance_map.use_sobel}")
        self.get_logger().info(f"  sobel_threshold: {self.params.distance_map.sobel_threshold}")
        
        # Robot Constraints Parameters
        self.get_logger().info("Robot Constraints Parameters:")
        self.get_logger().info(f"  max_velocity: {self.params.robot_constraints.max_velocity}")
        self.get_logger().info(f"  max_acceleration: {self.params.robot_constraints.max_acceleration}")
        self.get_logger().info(f"  max_deceleration: {self.params.robot_constraints.max_deceleration}")
        self.get_logger().info(f"  max_jerk: {self.params.robot_constraints.max_jerk}")
        self.get_logger().info(f"  max_lateral_accel: {self.params.robot_constraints.max_lateral_accel}")
        self.get_logger().info(f"  wheel_base: {self.params.robot_constraints.wheel_base}")
        self.get_logger().info(f"  min_turning_radius: {self.params.robot_constraints.min_turning_radius}")

    def goal_callback(self, goal_request):
        """
        Accept or reject a goal request.
        
        Implements a policy of one goal at a time - rejects new goals if one is already executing.
        """
        self.get_logger().info('Received goal request')
        
        # Check if we already have a goal
        if self._goal_executing:
            self.get_logger().info('Rejecting goal: Another goal is currently being executed')
            return GoalResponse.REJECT
        
        # Validate the goal request
        if len(goal_request.goal_gps) < 2:
            self.get_logger().error('Rejecting goal: Invalid GPS coordinates')
            return GoalResponse.REJECT
            
        if len(goal_request.lanelet_ids) == 0:
            self.get_logger().error('Rejecting goal: No lanelet IDs provided')
            return GoalResponse.REJECT
            
        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle a cancel request for an action goal"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info('Executing goal...')
        self._goal_executing = True
        
        # Extract goal details for logging
        goal = goal_handle.request
        lanelet_count = len(goal.lanelet_ids)
        
        self.get_logger().info(f'Planning trajectory through {lanelet_count} lanelets')
        self.get_logger().info(f'Goal GPS: [{goal.goal_gps[0]}, {goal.goal_gps[1]}]')
        
        # Set up feedback and result messages
        feedback_msg = NavigateToGoal.Feedback()
        result = NavigateToGoal.Result()
        
        # Initialize distance and time (for simulation)
        total_distance = goal.total_distance if goal.total_distance > 0 else 100.0  # Default to 100m if not specified
        total_time_estimate = goal.estimated_time if goal.estimated_time > 0 else 60.0  # Default to 60s if not specified
        
        try:
            # Simulate planning and execution process
            total_steps = 20  # More steps for a smoother simulation
            for i in range(1, total_steps + 1):
                # Check if goal was canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
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
                
                self.get_logger().info(f'Navigation progress: {progress_percent*100:.1f}%, ' +
                                      f'Distance remaining: {distance_remaining:.1f}m, ' +
                                      f'Time remaining: {time_remaining:.1f}s')
            
            # Example of setting a successful result
            result.status = GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)
            result.message = "Navigation completed successfully"
            result.total_distance = total_distance
            result.total_time = total_time_estimate

            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
            
        except Exception as e:
            # Handle any exceptions during execution
            self.get_logger().error(f'Goal execution failed: {str(e)}')
            goal_handle.abort()
            result.status = GoalStatus(status=GoalStatus.STATUS_ABORTED)
            result.message = f"Failed to plan trajectory: {str(e)}"
            result.total_distance = 0.0
            result.total_time = 0.0
        
        finally:
            self._goal_executing = False
            
        return result


def main(args=None):
    rclpy.init(args=args)
    
    trajectory_planner_node = TrajectoryPlannerActionServer()
    
    # Use MultiThreadedExecutor to handle the action server callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(trajectory_planner_node)
    
    try:
        trajectory_planner_node.get_logger().info('Beginning trajectory planner execution loop')
        executor.spin()
    except KeyboardInterrupt:
        trajectory_planner_node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        executor.shutdown()
        trajectory_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()