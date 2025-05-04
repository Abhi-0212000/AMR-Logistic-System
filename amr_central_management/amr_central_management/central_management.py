#!/usr/bin/env python3

# Copyright 2025 Abhishek Nannuri
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus, ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.task import Future
from amr_interfaces.msg import RobotPose
from amr_interfaces.srv import AmrGoalManagement, ComputeGlobalPath
from amr_interfaces.action import NavigateToGoal
from amr_utils_python.coordinate_transforms_py import (
    CoordinateTransforms,
    GPSPoint,
    LocalPoint,
)
from datetime import datetime, timedelta
from enum import Enum, auto
from typing import Optional, Dict, Any
from .error_handling import create_error_response, ErrorType
from .amr_central_management_params import amr_central_management


class RobotState(Enum):
    IDLE = auto()
    PLANNING = auto()
    NAVIGATING = auto()
    ERROR = auto()


class CentralManagementNode(Node):
    def __init__(self):
        super().__init__("amr_central_management")

        # Initialize parameter listener
        self.param_listener = amr_central_management.ParamListener(self)
        self.params = self.param_listener.get_params()
        self._reference_gps = GPSPoint(
            self.params.gps_origin.latitude,
            self.params.gps_origin.longitude,
            self.params.gps_origin.altitude,
        )
        self._use_time_based_routing = self.params.use_time_based_routing

        # Initialize states
        self._robot_state = RobotState.IDLE
        self._latest_robot_pose: Optional[RobotPose] = None
        self._last_pose_update: Optional[datetime] = None
        self._current_goal_handle: Optional[ClientGoalHandle] = None
        self._path_plan_data: Optional[Dict[str, Any]] = None

        # Constants
        self.POSE_TIMEOUT = timedelta(seconds=5.0)
        self.SERVICE_TIMEOUT = 5.0
        self.ACTION_SERVER_TIMEOUT = 10.0

        # Callback groups for concurrent execution
        # self._service_group = ReentrantCallbackGroup()
        # self._topic_group = ReentrantCallbackGroup()

        self.local_callback_group = MutuallyExclusiveCallbackGroup()
        self.external_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.topic_callback_group = MutuallyExclusiveCallbackGroup()

        # Load parameters and initialize systems
        self.print_params()
        self._initialize_coordinate_transform()
        self._setup_communication_interfaces()

        # Start periodic state checking
        self.health_check_timer = self.create_timer(
            1.0, self._check_system_health, callback_group=self.timer_callback_group
        )

        self.get_logger().info("Central Management Node initialized successfully")

    def print_params(self):
        """Print the parameters for debugging."""
        self.get_logger().info(
            f"GPS Origin: \
                {self._reference_gps.latitude}, \
                {self._reference_gps.longitude}, \
                {self._reference_gps.altitude}"
        )
        self.get_logger().info(f"Use Time Based Routing: {self._use_time_based_routing}")

    def _initialize_coordinate_transform(self) -> None:
        """Initialize coordinate transformation system."""
        try:
            self._coord_transform = CoordinateTransforms(self._reference_gps)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize coordinate transform: {str(e)}")
            raise

    def _setup_communication_interfaces(self) -> None:
        """Set up ROS communication interfaces with proper error handling."""
        # QoS Profile
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.create_subscription(
            RobotPose,
            "robot_pose",
            self._robot_pose_callback,
            self._qos_profile,
            callback_group=self.topic_callback_group,
        )

        # Services
        self.create_service(
            AmrGoalManagement,
            "amr_goal_management",
            self._handle_goal_request,
            callback_group=self.local_callback_group,
        )

        # Service Clients
        self._path_planner_client = self.create_client(
            ComputeGlobalPath, "global_path_planner", callback_group=self.external_callback_group
        )

        # Action Clients
        self._navigate_action_client = ActionClient(
            self, NavigateToGoal, "navigate_to_goal", callback_group=self.external_callback_group
        )

    def _robot_pose_callback(self, msg: RobotPose) -> None:
        """Update robot pose with timestamp."""
        self._latest_robot_pose = msg
        self._last_pose_update = datetime.now()

    def _check_system_health(self) -> None:
        """Periodic health check of the system."""
        if not self._is_pose_valid():
            self.get_logger().warn("Robot pose data is stale or missing")
            self._robot_state = RobotState.ERROR
        else:
            self.get_logger().info("Robot pose data is valid and recent")
            self.get_logger().info(
                "Setting robot state to IDLE and cancelling health check timer."
            )
            self._robot_state = RobotState.IDLE
            self.health_check_timer.cancel()

    def _is_pose_valid(self) -> bool:
        """Check if the robot pose data is valid and recent."""
        if not self._latest_robot_pose or not self._last_pose_update:
            return False
        return (datetime.now() - self._last_pose_update) <= self.POSE_TIMEOUT

    async def _handle_goal_request(
        self, request: AmrGoalManagement.Request, response: AmrGoalManagement.Response
    ) -> AmrGoalManagement.Response:
        """Handle incoming goal requests with proper error handling and state management."""
        """Handle incoming goal requests."""
        if self._robot_state != RobotState.IDLE:
            return create_error_response(
                ErrorType.BUSY_ERROR, "Another goal is currently being processed"
            )

        if not self._is_pose_valid():
            return create_error_response(ErrorType.POSE_ERROR, "Robot pose data invalid or stale")

        try:
            self._robot_state = RobotState.PLANNING
            # Request path plan
            path_response = await self._request_path_plan(request)

            if path_response.status != 0:
                self._robot_state = RobotState.IDLE
                self.get_logger().error(f"Path planning failed: {path_response.message}")
                return create_error_response(
                    ErrorType.PLANNING_ERROR,
                    f"Path planning failed: {path_response.message}",
                )

            self.get_logger().info("Path planning successful. Sending goal to Local Planner...")

            # Send navigation goal
            goal_accepted = await self._send_navigation_goal(path_response, request)

            if not goal_accepted:
                self._robot_state = RobotState.IDLE
                return create_error_response(
                    ErrorType.GOAL_REJECTED, "Goal rejected by action server"
                )

            response.status = 0
            response.message = "Goal accepted and navigation started"
            response.total_distance = path_response.total_distance
            response.estimated_time = path_response.estimated_time

            self._robot_state = RobotState.NAVIGATING
            return response

        except Exception as e:
            self._robot_state = RobotState.ERROR
            return create_error_response(
                response, f"Unexpected error: {str(e)}", ErrorType.SYSTEM_ERROR
            )

    async def _request_path_plan(
        self, request: AmrGoalManagement.Request
    ) -> ComputeGlobalPath.Response:
        """Request path plan with proper error handling."""
        try:
            if not self._path_planner_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
                self.get_logger().error("Path planner service not available")
                raise RuntimeError("Path planner service not available")

            current_pose = self._get_current_pose_as_gps()

            path_request = ComputeGlobalPath.Request(
                start_latitude=current_pose.latitude,
                start_longitude=current_pose.longitude,
                start_altitude=current_pose.altitude,
                end_latitude=request.goal_latitude,
                end_longitude=request.goal_longitude,
                end_altitude=request.goal_altitude,
                use_time_based_routing=self._use_time_based_routing,
            )
            self.get_logger().info("Requesting path plan from path planner service...")
            self.get_logger().info(f"Path request: {path_request}")

            # Create the future and wait for it explicitly
            future = self._path_planner_client.call_async(path_request)
            # Wait for the future to complete
            return await future

        except Exception as e:
            raise RuntimeError(f"Path planning failed: {str(e)}")

    async def _send_navigation_goal(
        self,
        path_response: ComputeGlobalPath.Response,
        original_request: AmrGoalManagement.Request,
    ) -> bool:
        """Send navigation goal to action server with proper error handling."""
        try:
            # Check if the action server is available before sending the goal
            if not self._navigate_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Navigation action server is not available")
                raise RuntimeError("Navigation action server is not available")

            goal_msg = NavigateToGoal.Goal(
                goal_gps=[
                    original_request.goal_latitude,
                    original_request.goal_longitude,
                    original_request.goal_altitude,
                ],
                lanelet_ids=path_response.lanelet_ids,
                is_inverted=path_response.is_inverted,
                total_distance=path_response.total_distance,
                estimated_time=path_response.estimated_time,
            )

            # Send goal and wait for acceptance
            send_goal_future = self._navigate_action_client.send_goal_async(
                goal_msg, feedback_callback=self._navigation_feedback_callback
            )
            self.get_logger().info("Goal sent and waiting for acceptance confirmation...")
            goal_handle = await send_goal_future

            if not goal_handle.accepted:
                return False

            self.get_logger().info("Goal accepted by action server")

            self._current_goal_handle = goal_handle

            goal_handle.get_result_async().add_done_callback(self._navigation_result_callback)
            self.get_logger().info(
                "Add done callback is called aysnchronously and Waiting for navigation result..."
            )

            return True

        except Exception as e:
            self.get_logger().error(f"Failed to send navigation goal: {str(e)}")
            return False

    def _navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        self.get_logger().debug(f"Navigation feedback: {feedback_msg}")

    def _navigation_result_callback(self, future: Future):
        """Handle navigation result."""
        try:
            result = future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation completed successfully")
            elif result.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("Navigation aborted")
            elif result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Navigation cancelled")

            self._robot_state = RobotState.IDLE
            self._current_goal_handle = None

        except Exception as e:
            self.get_logger().error(f"Error handling navigation result: {str(e)}")
            self._robot_state = RobotState.ERROR

    def _get_current_pose_as_gps(self) -> GPSPoint:
        """Convert current robot pose to GPS coordinates."""
        if not self._latest_robot_pose:
            raise RuntimeError("No valid robot pose available")

        local_point = LocalPoint(
            self._latest_robot_pose.pose.pose.position.x,
            self._latest_robot_pose.pose.pose.position.y,
        )
        return self._coord_transform.local_to_gps(local_point)

    async def cancel_current_goal(self) -> bool:
        """Cancel the current navigation goal if one exists."""
        if not self._current_goal_handle or self._robot_state != RobotState.NAVIGATING:
            self.get_logger().warn("No active goal to cancel")
            return False

        try:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            await cancel_future
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to cancel goal: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = CentralManagementNode()

    # Create MultiThreadedExecutor to handle async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Central Management Node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
