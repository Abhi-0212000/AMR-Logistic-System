# AMR Navigation System

This package provides autonomous global path planning functionalities for mobile robots, leveraging the Lanelet2 library for map-based navigation.

## Table of Contents
1. [Features](#features)
2. [Directory Structure](#directory-structure)
3. [Installation & Build Instructions](#installation--build-instructions)
4. [Usage](#usage)
    - [Server Node](#server-node)
    - [Client Node](#client-node)
5. [System Architecture Overview](#system-architecture-overview)
6. [Key Components Explained](#key-components-explained)
    - [Global Path Planner (Server Node)](#1-global-path-planner)
    - [Parameters System](#2-parameters-system)
7. [Debugging Guide](#debugging-guide)
8. [Best Practices](#best-practices)
9. [Contributing](#contributing)
10. [License](#license)
11. [Support](#support)


## Features

- **Global Path Planner Server**: Computes optimal paths using Lanelet2 maps
- **Reusable Logger**: Custom logger based on spdlog supporting `.json`, `/.log` formats
- **Example Client Node**: Demonstrates communication with the global path planner


## Directory Structure

```
amr_navigation_system
├── config
│   ├── client_params.yaml
│   └── global_path_planner_params.yaml
├── include/amr_navigation_system
│   ├── global_path_plan_client
│   └── global_path_plan_server
│       ├── amr_traffic_rules.hpp
│       ├── graph_builder.hpp
│       └── optimal_path_planner.hpp
├── laneletMaps                # Folder for storing OSM map files
├── src
│   ├── global_path_plan_client
│   │   └── global_path_planner_client.cpp  # Example client node
│   └── global_path_plan_server
│       ├── amr_traffic_rules.cpp
│       ├── graph_builder.cpp
│       ├── optimal_path_planner.cpp
│       └── global_path_planner.cpp         # Main server node
├── utils
│   └── common_utils.hpp       # Utility functions/structs shared across the package
│   └── logger.hpp             # Reusable logger component (see logger.md for details)
│   └── logger.md              # This README file for reusing the logger code in other packages
├── CMakeLists.txt
├── package.xml
└── amr_navigation_system.md   # This README file
```

## Installation & Build Instructions

Please follow the **Installation** instructions in main **README.md** file under `amr_navigation_system` package.


## Usage

The Global Path Planner server node provides path computation services based on start and end GPS coordinates.
Replace the `/path/to/global_path_planner_params.yaml` with absolute or relative path to the file `amr_navigation_system/config/global_path_planner_params.yaml` depending on your system and folder management.

### Server Node

The Global Path Planner server node provides path computation services based on start and end GPS coordinates.

```bash
# Launch with custom parameters
ros2 run amr_navigation_system global_path_planner --ros-args --params-file /path/to/global_path_planner_params.yaml
```

### Client Node

The example client node demonstrates interaction with the path planner server.

```bash
# Launch with custom parameters
ros2 run amr_navigation_system global_path_planner_client --ros-args --params-file /path/to/client_params.yaml
```

## Guide to help you understand (High-Level) the code:

### System Architecture Overview

The navigation system follows a client-server architecture:
1. **Server**: Handles path planning requests (`global_path_planner.cpp`)
2. **Client**: Sends requests for path computation (`global_path_planner_client.cpp`)
3. **Supporting Components**: Map handling, path optimization, logging (All the other files.)

### Key Components Explained

#### 1. Global Path Planner
The server is the brain of the system, handling path planning requests. Here's how it works:

```cpp
class GlobalPathPlanner : public rclcpp::Node 
{
    // This is the main class that handles everything
}
```

Key Functions:
- `.yaml` file parameter validations
- **Constructor**: Sets up the server and loads parameters
- **callbackComputeGlobalPath**: Handles path planning requests
- **loadParameters**: Loads configuration from YAML files

#### 2. Parameters System

Please look at the parameter files  `amr_navigation_system/config/global_path_planner_params.yaml`, `amr_navigation_system/config/client_params.yaml` for more details.

### Debugging Guide

#### 1. Understanding System Flow

1. **Initialization Process**:
   ```cpp
   GlobalPathPlanner()  // Constructor
   ├── loadParameters() // Load settings
   ├── checkMapAndCreateDebugDir() // Verify map exists
   └── create_service() // Start the server
   ```

2. **Path Planning Process**:
   ```cpp
   callbackComputeGlobalPath()
   ├── Load map and create graph
   ├── Find nearest road/path points
   ├── Calculate optimal path
   └── Send response to client
   ```

#### 2. Common Issues and Solutions

1. **Map Loading Failures**
   - **Symptom**: Error message about map file not found
   - **Check**: 
     - Is the map path correct in parameters?
     - Does the file exist?
     - Do you have read permissions?

2. **Path Planning Failures**
   - **Symptom**: "Failed to compute path" message
   - **Check**:
     - Are start/end points within bounds? (bounds are defined in global_path_planner_params.yaml)
     - Is the map properly formatted?
     - Check log files for detailed error messages. Logs are stored in `~/.local/share/AMR/logs`

3. **Parameter Issues**
   - **Symptom**: Unexpected behavior or crashes
   - **Check**: 
     - YAML file syntax
     - Parameter values (especially coordinates)
     - Log files for parameter loading errors

#### 3. Using the Logger

**Please refer to the `amr_navigation_system/utils/logger.md` for more datiled understanding of current logging system and guide to reuse it in other packages**

Currently supported in `.json`, `.log` formats.

The system includes a comprehensive logging system:

```cpp
logger_->log_info("Message", __FILE__, __LINE__, __FUNCTION__);
logger_->log_error("Error message", __FILE__, __LINE__, __FUNCTION__);
```

Log files contain:
- Log Level
- Timestamp
- Node name, PKG name
- File and line number
- Function name
- Detailed message
- Thread ID

#### 4. Monitoring Tools

1. **ROS2 Command Line Tools**:
   ```bash
   # List all active nodes
   ros2 node list

   # Check node info
   ros2 node info /amr_global_path_planner

   # Echo service calls
   ros2 service list
   ```

2. **Log File Location**:
   - Default: `~/.local/share/AMR/logs`
   - Format: JSON or plain text (configurable)

### Troubleshooting Steps

1. **When Path Planning Fails**:
   1. Check the log files
   2. Verify input coordinates
   3. Ensure map is loaded correctly
   4. Check if points are within bounds

2. **Configuration Issues**:
   1. Review YAML file syntax
   2. Validate parameter values
   3. Check log for parameter loading errors

3. **System Status Check**:
   ```bash
   # Check if node is running
   ros2 node list | grep path_planner

   # Check service availability
   ros2 service list | grep global_path_planner

   # View node parameters
   ros2 param list /amr_global_path_planner
   ```

### Best Practices

1. **Always check logs first** when troubleshooting
2. **Keep map files in designated directory**
3. **Backup configuration files** before modifications
4. **Test with simple paths** before complex ones
5. **Monitor system resources** during operation


## Support

For additional help:
1. Check the detailed logs in `~/.local/share/AMR/logs`
2. Review the configuration in YAML files
3. Contact the development team (abhishek.nannuri@outlook.com)
