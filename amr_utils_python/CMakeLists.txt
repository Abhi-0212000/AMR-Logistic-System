cmake_minimum_required(VERSION 3.8)
project(amr_utils_python)

# Set C++ standard to match amr_local_planner
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Set GeographicLib CMake module path exactly as in amr_local_planner
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# Include directories
include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}  # Add this line
)

# Add pybind11 module
pybind11_add_module(coordinate_transforms_py src/coordinate_transforms_py.cpp)

# Link against GeographicLib with full path
target_link_libraries(coordinate_transforms_py PRIVATE 
  ${GeographicLib_LIBRARIES} 
)

# Install Python module
install(TARGETS coordinate_transforms_py
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
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
