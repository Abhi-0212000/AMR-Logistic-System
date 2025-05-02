cmake_minimum_required(VERSION 3.8)
project(amr_navigation_system)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Optionally disable compiler-specific extensions

# Specify vcpkg toolchain
# set(CMAKE_TOOLCHAIN_FILE "/home/abhi/vcpkg/scripts/buildsystems/vcpkg.cmake")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
message(STATUS "Found spdlog version: ${spdlog_VERSION}")
find_package(fmt REQUIRED)
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
    src/utils/logger.cpp
    # Add other source files here as needed
)

ament_target_dependencies(global_path_planner
  rclcpp
  spdlog
  fmt
  amr_interfaces  # This links the service interface dependency
  lanelet2_core
  lanelet2_projection
  lanelet2_io
  lanelet2_routing
  lanelet2_traffic_rules
  nlohmann_json
)
target_link_libraries(global_path_planner fmt)

add_executable(global_path_planner_client
    src/global_path_plan_client/global_path_planner_client.cpp
    src/utils/logger.cpp
)

ament_target_dependencies(global_path_planner_client
  rclcpp
  spdlog
  fmt
  nlohmann_json
  amr_interfaces  # This links the service interface dependency
)
target_link_libraries(global_path_planner_client fmt)

# Create a separate library for AMR traffic rules
add_library(amr_traffic_rules SHARED
    src/global_path_plan_server/amr_traffic_rules.cpp
)

ament_target_dependencies(amr_traffic_rules
  lanelet2_core
  lanelet2_traffic_rules
)

# Install the library
install(TARGETS
  global_path_planner
  global_path_planner_client
  amr_traffic_rules  # Add this line
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS

#   global_path_planner
#   global_path_planner_client

#   DESTINATION lib/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
