#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from amr_interfaces.msg import RobotPose
from amr_utils_python.coordinate_transforms_py import GPSPoint, CoordinateTransforms


# Simple function to convert Euler angles to quaternion
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q_w = cy * cp * cr + sy * sp * sr
    q_x = cy * cp * sr - sy * sp * cr
    q_y = sy * cp * sr + cy * sp * cr
    q_z = sy * cp * cr - cy * sp * sr

    return [q_x, q_y, q_z, q_w]


class DummyLocalizationNode(Node):
    def __init__(self):
        super().__init__("dummy_localization_node")

        # Declare parameters with default values
        self.declare_parameter("publish_frequency", 10.0)  # Hz
        self.declare_parameter("frame_id", "map")

        self.map_origin = GPSPoint(50.7153508, 10.4680332, 0.0)  # Dummy GPS origin
        self._coord_transform = CoordinateTransforms(self.map_origin)
        self.current_gps_coords = GPSPoint(50.7154014, 10.4683083, 0.0)  # Dummy GPS coordinates

        # Convert GPS coordinates to UTM coordinates
        self.current_utm_coords = self._coord_transform.gps_to_local(self.current_gps_coords)
        # Hardcoded GPS position from parameters
        self.x = self.current_utm_coords.x
        self.y = self.current_utm_coords.y
        self.z = 0.0

        # Initial orientation (45 degrees in radians)
        self.yaw = math.pi / 4.0

        # Initial velocity and yaw rate
        self.velocity = 0.5  # m/s
        self.yaw_rate = 0.1  # rad/s

        # Create QoS profile for localization data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create publisher
        self.publisher = self.create_publisher(RobotPose, "robot_pose", qos_profile)

        # Create timer for publishing at the specified frequency
        publish_frequency = self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(1.0 / publish_frequency, self.publish_pose)

        self.get_logger().info(
            f"Dummy localization node started, publishing at {publish_frequency} Hz"
        )
        self.get_logger().info(f"Initial position: x={self.x}, y={self.y}, yaw={self.yaw} rad")

    def publish_pose(self):
        """Publish the robot's pose, velocity, and yaw rate"""
        # Get current time for header timestamp
        current_time = self.get_clock().now()
        frame_id = self.get_parameter("frame_id").value

        # Create header for the RobotPose message
        header = Header()
        header.stamp = current_time.to_msg()
        header.frame_id = frame_id

        # Create header for PoseStamped
        pose_header = Header()
        pose_header.stamp = current_time.to_msg()
        pose_header.frame_id = frame_id

        # Create position and orientation
        position = Point(x=self.x, y=self.y, z=self.z)

        # Convert Euler angles to quaternion using our function
        q = euler_to_quaternion(0.0, 0.0, self.yaw)
        orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Create pose
        pose = Pose(position=position, orientation=orientation)

        # Create PoseStamped
        pose_stamped = PoseStamped(header=pose_header, pose=pose)

        # Create and publish RobotPose message
        robot_pose_msg = RobotPose(
            header=header, pose=pose_stamped, velocity=self.velocity, yaw_rate=self.yaw_rate
        )

        self.publisher.publish(robot_pose_msg)

        # Log occasionally (every 5 seconds) to avoid spamming
        # if int(current_time.nanoseconds / 1e9) % 5 == 0:
        #     self.get_logger().info(f'Published pose: x={self.x}, y={self.y}, yaw={self.yaw} rad')


def main(args=None):
    rclpy.init(args=args)

    dummy_localization_node = DummyLocalizationNode()

    try:
        rclpy.spin(dummy_localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_localization_node.get_logger().info("Shutting down dummy localization node")
        dummy_localization_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
