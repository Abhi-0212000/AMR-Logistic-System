cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

# Ensuring correct compiler flags for warnings and errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find essential dependencies, generalized to any machine with ROS 2
find_package(ament_cmake REQUIRED)
# ROS dependencies
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# builtin_interfaces provides some lower-level types like time and duration, which are used by the Header message
find_package(builtin_interfaces REQUIRED)  # Needed for std_msgs/Header

# One-time setup code for ROS 2 interface generation
find_package(rosidl_default_generators REQUIRED)

# Generate the interfaces from .srv files
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/ComputeGlobalPath.srv" # This .srv is not depending on geometry_msgs pkg.
  "srv/AmrGoalManagement.srv"
  "msg/RobotPose.msg"
  "msg/TrajectoryPoint.msg"
  "action/NavigateToGoal.action"
  "msg/Trajectory.msg"
  DEPENDENCIES geometry_msgs std_msgs # Ensure that the generated service can depend on geometry_msgs
)

# Export the rosidl runtime to allow other packages to use the generated service
ament_export_dependencies(rosidl_default_runtime)

# If tests are being run, find and include the testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Standard package declaration for ROS 2
ament_package()
