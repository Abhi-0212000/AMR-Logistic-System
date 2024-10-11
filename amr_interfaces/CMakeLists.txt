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

# One-time setup code for ROS 2 interface generation
find_package(rosidl_default_generators REQUIRED)

# Generate the interfaces from .srv files
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/ComputeGlobalPath.srv" # This .srv is not depending on geometry_msgs pkg.
  DEPENDENCIES geometry_msgs  # Ensure that the generated service can depend on geometry_msgs
)

# Export the rosidl runtime to allow other packages to use the generated service
ament_export_dependencies(rosidl_default_runtime)

# If tests are being run, find and include the testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The following lines allow you to bypass lint checks temporarily
  # You can enable them when your project is ready for final checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Standard package declaration for ROS 2
ament_package()
