#!/usr/bin/env python3

from enum import Enum
from typing import Optional
from rclpy.node import Node
from amr_interfaces.srv import AmrGoalManagement
from geometry_msgs.msg import Twist

class ErrorType(Enum):
    """Enumeration of possible error types in the AMR system."""
    POSE_ERROR = "Robot pose data invalid or stale"
    SERVICE_ERROR = "Service communication error"
    PLANNING_ERROR = "Path planning failed"
    NAVIGATION_ERROR = "Navigation error"
    BUSY_ERROR = "Robot is currently busy"
    SYSTEM_ERROR = "System error"
    TIMEOUT_ERROR = "Operation timed out"
    GOAL_REJECTED = "Goal rejected by action server"
    EMERGENCY_STOP = "Emergency stop triggered"
    PARAMETER_ERROR = "Invalid parameter"
    TRANSFORM_ERROR = "Coordinate transform error"
    
    def __str__(self) -> str:
        return self.value

class ErrorResponse:
    """Standard error response structure."""
    def __init__(self, error_type: ErrorType, message: str, status_code: int = 1):
        self.error_type = error_type
        self.message = message
        self.status_code = status_code

def handle_error(
    node: Node,
    error_msg: str,
    error_type: ErrorType,
    emergency_stop: bool = False,
    log_level: str = "error"
) -> AmrGoalManagement.Response:
    """
    Centralized error handling for ROS2 node.
    
    Args:
        node: The ROS2 node instance
        error_msg: Detailed error message
        error_type: Type of error from ErrorType enum
        emergency_stop: Whether to trigger emergency stop
        log_level: Logging level ("error", "warn", "info")
    
    Returns:
        AmrGoalManagement.Response with error details
    """
    # Format the error message
    formatted_msg = f"[{error_type.name}] {error_msg}"
    
    # Log the error with appropriate level
    logger = getattr(node.get_logger(), log_level)
    logger(formatted_msg)
    
    # Trigger emergency stop if needed
    if emergency_stop:
        emergency_stop_robot(node)
    
    return create_error_response(error_type, error_msg)

def emergency_stop_robot(node: Node, num_attempts: int = 3) -> bool:
    """
    Sends zero velocity command to stop the robot.
    
    Args:
        node: The ROS2 node instance
        num_attempts: Number of stop commands to send
        
    Returns:
        bool: True if stop commands were sent successfully
    """
    try:
        stop_cmd = Twist()
        for i in range(num_attempts):
            node.cmd_vel_pub.publish(stop_cmd)
            node.get_logger().warn(f"Emergency stop command sent (attempt {i+1}/{num_attempts})")
        return True
    except Exception as e:
        node.get_logger().error(f"Failed to send emergency stop command: {str(e)}")
        return False

def create_error_response(
    error_type: ErrorType,
    error_msg: Optional[str] = None,
    status_code: int = 1
) -> AmrGoalManagement.Response:
    """
    Creates and returns a standardized error response.
    
    Args:
        error_type: Type of error from ErrorType enum
        error_msg: Optional additional error details
        status_code: Response status code (default: 1 for error)
        
    Returns:
        AmrGoalManagement.Response with error details
    """
    response = AmrGoalManagement.Response()
    response.status = status_code
    
    # Construct error message
    if error_msg:
        response.message = f"{error_type}: {error_msg}"
    else:
        response.message = str(error_type)
    
    return response

def check_response_status(response: AmrGoalManagement.Response) -> bool:
    """
    Checks if a response indicates success or failure.
    
    Args:
        response: The response to check
        
    Returns:
        bool: True if response indicates success, False otherwise
    """
    return response.status == 0

def log_error_with_context(
    node: Node,
    error: Exception,
    context: str,
    error_type: ErrorType
) -> None:
    """
    Logs an error with additional context information.
    
    Args:
        node: The ROS2 node instance
        error: The exception that occurred
        context: Description of what was being attempted
        error_type: Type of error from ErrorType enum
    """
    error_msg = f"{context} failed: {str(error)}"
    node.get_logger().error(f"[{error_type.name}] {error_msg}")

def create_service_response(
    success: bool,
    message: str,
    status_code: int = None
) -> AmrGoalManagement.Response:
    """
    Creates a service response with standard format.
    
    Args:
        success: Whether the operation was successful
        message: Response message
        status_code: Optional specific status code
        
    Returns:
        AmrGoalManagement.Response
    """
    response = AmrGoalManagement.Response()
    
    if status_code is not None:
        response.status = status_code
    else:
        response.status = 0 if success else 1
        
    response.message = message
    return response