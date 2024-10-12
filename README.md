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
    - **Key Features**:
      - Global path planning using Lanelet2.
      - GPS filtering and nearest-lanelet detection.
      - Map boundary validation and non-navigable area exclusion.
      - Scalable for large-scale environments and custom maps.

## Installation

To use the `amr_navigation_system` package, you need to install the Lanelet2 library. Follow these steps:

1. **Set up ROS2 Humble**:
     - Ensure ROS2 Humble is installed on your system. Follow the official ROS2 installation guide if needed.

2. **Install Lanelet2**:
     - **From Source**:
          1. Fork the Lanelet2 repository and clone it into the `src` folder of your ROS2 workspace (`ros2_ws`):
                ```bash
                cd ~/ros2_ws/src
                git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git
                ```
          2. Build all packages inside the Lanelet2 folder:
                ```bash
                cd ~/ros2_ws
                colcon build --packages-up-to lanelet2
                ```
     - **From Package Manager** (e.g., `apt`):
          - Install using:
                ```bash
                sudo apt install ros-humble-lanelet2
                ```
          - Refer to the [Lanelet2 documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) for more details.

3. **Clone AMR-Logistic-System**:
     - Clone this repository into your ROS2 workspace:
          ```bash
          cd ~/ros2_ws/src
          git clone https://github.com/Abhi-0212000/AMR-Logistic-System.git
          ```
          Note: This repository is currently private.

4. **Build the packages**:
     - Build the AMR-Logistic-System packages:
          ```bash
          cd ~/ros2_ws
          colcon build --packages-select amr_interfaces amr_navigation_system
          ```

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
     │       │   └── default_params.yaml
     │       ├── include/amr_navigation_system
     │       │   ├── global_path_plan_client
     │       │   └── global_path_plan_server
     │       │       ├── amr_traffic_rules.hpp
     │       │       ├── graph_builder.hpp
     │       │       └── optimal_path_planner.hpp
     │       ├── utils
     │       │   └── common_utils.hpp
     │       ├── laneletMaps
     │       ├── src
     │       |   ├── global_path_plan_client
     │       |   │   └── global_path_planner_client.cpp
     │       |   └── global_path_plan_server
     │       |       ├── amr_traffic_rules.cpp
     │       |       ├── graph_builder.cpp
     │       |       ├── optimal_path_planner.cpp
     │       |       └── global_path_planner.cpp
     │       ├── CMakeLists.txt
     │       └── package.xml
     └── lanelet2
```

## Documentation

For detailed information on using the `amr_navigation_system` package and its functionalities, refer to the inline code documentation and Doxygen comments available in the source files.

## Future Plans

This package is intended for internal use at Hochschule Schmalkalden for R&D, teaching, and research purposes. It serves as a foundation for future publications and enhancements, with plans to incorporate additional packages and functionalities.

## Acknowledgments

This project utilizes the following open-source resources:
- **Lanelet2**: For map handling and routing. [Lanelet2 GitHub](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- **OpenStreetMap (OSM)**: For generating Lanelet2 maps. [OpenStreetMap](https://www.openstreetmap.org/)

Developed for internal use at Hochschule Schmalkalden for R&D, teaching, and research purposes.
