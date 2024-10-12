cmake_minimum_required(VERSION 3.8)
project(amr_navigation_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(amr_interfaces REQUIRED)

# Specify where to find the header files
include_directories(include)

add_executable(global_path_planner
    src/global_path_plan_server/global_path_planner.cpp
    src/global_path_plan_server/graph_builder.cpp  # Add this line
    src/global_path_plan_server/amr_traffic_rules.cpp
    src/global_path_plan_server/optimal_path_finder.cpp
    src/utils/common_utils.cpp
    # Add other source files here as needed
)

ament_target_dependencies(global_path_planner 
  rclcpp 
  amr_interfaces  # This links the service interface dependency
  lanelet2_core 
  lanelet2_projection
  lanelet2_io
  lanelet2_routing
  lanelet2_traffic_rules
)


add_executable(global_path_planner_client
    src/global_path_plan_client/global_path_planner_client.cpp
)

ament_target_dependencies(global_path_planner_client 
  rclcpp 
  amr_interfaces  # This links the service interface dependency
)


install(TARGETS

  global_path_planner
  global_path_planner_client

  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
