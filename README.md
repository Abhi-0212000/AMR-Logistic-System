# AMR-Logistic-System
This repository contains the code and documentation for an Autonomous Mobile Robot (AMR) designed for efficient delivery and logistics operations. The system integrates advanced navigation, sensor fusion, and path planning algorithms to ensure reliable and autonomous delivery within various environments.

## Packages

This repository currently includes two ROS2 packages, with potential for future expansion:

1. **amr_interfaces**:
    - Provides custom ROS2 interface types.
    - Includes one service file:
      - `ComputeGlobalPath.srv`: Facilitates communication between the `global_path_planner` and `global_path_planner_client` nodes in the `amr_navigation_system` package.

2. **amr_navigation_system**:
    - A ROS2 package for autonomous navigation of mobile robots using the Lanelet2 library.
    - Implements core navigation functionalities.
    - **Dependencies**:
      - `rclcpp`: Included with ROS2 Humble installation.
      - `lanelet2`: Must be installed separately.
      - `rclcpp, fmt, nlohmann-json3-dev`
    - **Key Features**:
      - Global path planning using Lanelet2.
      - GPS filtering and nearest-lanelet detection.
      - Map boundary validation and non-navigable area exclusion.
      - Scalable for large-scale environments and custom maps.

## Installation

### 1. Set up ROS2 Humble
Ensure ROS2 Humble is installed on your system. If not, follow the official [ROS2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#install-ros-2-packages).

### 2. Install Lanelet2
Choose one of the following methods:

#### Option A: From Source
```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git

# Build packages
cd ~/ros2_ws
colcon build --packages-up-to lanelet2
```

#### Option B: From Package Manager
```bash
sudo apt install ros-humble-lanelet2
```
For more details, refer to the [Lanelet2 documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2).

### 3. Install Required Dependencies

#### System Dependencies
```bash
# Install spdlog and fmt
sudo apt install libspdlog-dev libfmt-dev

# Install nlohmann-json
sudo apt install nlohmann-json3-dev
```

### 4. Clone the Repository
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Abhi-0212000/AMR-Logistic-System.git
```
Note: This repository is currently private.

### 5. Build the Packages
```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the packages
colcon build --packages-select amr_interfaces amr_navigation_system
```
### System Requirements

- Ubuntu 22.04 (recommended)
- ROS2 Humble
- CMake version 3.8 or higher
- C++17 or higher
- Sufficient disk space (~2GB for all dependencies)

### Post-Installation Setup

1. Source the workspace:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

2. Verify environment setup:
   ```bash
   ros2 pkg list | grep amr_navigation_system
   ```

3. Update the `map_path` variable value in `amr_navigation_system/config/global_path_planner_params.yaml` as per your folder structure. 

For any installation issues, please check the log files or create an issue in the repository.


## Current Folder Structure

```plaintext
ros2_ws
└── src
     ├── AMR-Logistic-System
     │   ├── amr_interfaces
     │   │   ├── msg
     │   │   ├── srv
     │   │   │   └── ComputeGlobalPath.srv
     │   │   ├── CMakeLists.txt
     │   │   └── package.xml
     │   └── amr_navigation_system
     │       ├── config
     │       │   ├── client_params.yaml
     │       │   └── global_path_planner_params.yaml
     │       ├── include/amr_navigation_system
     │       │   ├── global_path_plan_client
     │       │   └── global_path_plan_server
     │       │       ├── amr_traffic_rules.hpp
     │       │       ├── graph_builder.hpp
     │       │       └── optimal_path_planner.hpp
     │       ├── utils
     │       │   └── common_utils.hpp
     │       │   └── logger.hpp
     │       ├── laneletMaps
     │       ├── src
     │       |   ├── global_path_plan_client
     │       |   │   └── global_path_planner_client.cpp
     │       |   └── global_path_plan_server
     │       |       ├── amr_traffic_rules.cpp
     │       |       ├── graph_builder.cpp
     │       |       ├── optimal_path_planner.cpp
     │       |       └── global_path_planner.cpp
     │       ├── utils
     │       │   └── common_utils.hpp
     │       │   └── logger.hpp
     │       │   └── logger.md
     │       ├── CMakeLists.txt
     │       ├── package.xml
     │       └── amr_navigation_system.md 
     └── lanelet2 (Depends on whether you have build the lanelet2 from source code or through pkg manager)
```

## Documentation

For detailed information on using the `amr_navigation_system` package and its functionalities, refer to the inline code documentation and Doxygen comments available in the source files.

Each package contains its own `.md` files with detailed documentation:

`amr_navigation_system/amr_navigation_system.md`
`amr_navigation_system/utils/logger.md`   -    Logging System Documentation

## Future Plans

This package is intended for internal use at Hochschule Schmalkalden for R&D, teaching, and research purposes. It serves as a foundation for future publications and enhancements, with plans to incorporate additional packages and functionalities.

## Acknowledgments

This project utilizes the following open-source resources:
- **Lanelet2**: For map handling and routing. [Lanelet2 GitHub](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- **OpenStreetMap (OSM)**: For generating Lanelet2 maps. [OpenStreetMap](https://www.openstreetmap.org/)

Developed for internal use at Hochschule Schmalkalden for R&D, teaching, and research purposes.

## Support

For additional help:
1. Contact the development team (abhishek.nannuri@outlook.com)