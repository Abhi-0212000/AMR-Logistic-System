AMR Navigation System Logger Documentation
# Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Directory Structure](#directory-structure)
- [Usage Guide](#usage-guide)
    - [Basic Usage](#basic-usage)
    - [Advanced Usage](#advanced-usage)
    - [Configuration](#configuration)
- [API Reference](#api-reference)
- [Examples](#examples)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

## Overview

The AMR Navigation System Logger is a thread-safe, asynchronous logging utility designed for ROS2 nodes. It provides structured logging capabilities with support for both JSON and plain text formats, making it ideal for debugging, monitoring, and system analysis in robotics applications.

## Features

- **Thread-safe Operation**: Ensures safe logging from multiple threads
- **Asynchronous Logging**: Non-blocking log operations
- **Multiple Output Formats**:
    - JSON format for structured logging
    - Plain text format for human-readable logs
- **Configurable Log Levels**:
    - DEBUG (0)
    - INFO (1)
    - WARN (2)
    - ERROR (3)
    - FATAL (4)
- **Automatic Directory Organization**: Logs organized by package and node names
- **Timestamp-based File Creation**: Automatic file creation with precise timestamps
- **Detailed Context Capture**: Records file name, line number, function name, and timestamp
- **Periodic Log Flushing**: Prevents data loss in case of node failure. Depends on queue size (`size_t queue_size = 32;`)

## Dependencies

Add these dependencies to your `package.xml`:
```xml
<depend>spdlog</depend>
<depend>nlohmann-json-dev</depend>
<depend>rclcpp</depend>
```

Add to your `CMakeLists.txt`:
```cmake
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${YOUR_TARGET_NAME}
        src/utils/logger.cpp
        # Add other source files here as needed
)

ament_target_dependencies(${YOUR_TARGET_NAME} 
    rclcpp 
    spdlog
    fmt
    nlohmann_json  
)
target_link_libraries(${YOUR_TARGET_NAME} fmt)
```

## Directory Structure

```plaintext
your_package/
├── include/
│   └── your_package/
│       └── utils/
│           └── logger.hpp
├── src/
│   └── utils/
│       └── logger.cpp
└── config/
        └── params.yaml
```

Generated log directory structure:
```plaintext
base_path/
└── package_name/
        └── node_name/
                └── log_YYYY-MM-DD_HH-MM-SS.json (or .log)
```

## Usage Guide

### Basic Usage

Include the Logger:
```cpp
#include "your_package/utils/logger.hpp"
```

Initialize in Node Constructor:
```cpp
class YourNode : public rclcpp::Node
{
private:
        std::shared_ptr<amr_logging::NodeLogger> logger_;

public:
        YourNode() : Node("your_node_name")
        {
                // Get parameters
                std::string base_path = this->declare_parameter("log_base_path", "~/logs");
                int log_level = this->declare_parameter("log_level", 1);
                std::string file_type = this->declare_parameter("log_file_type", "json");

                // Initialize logger
                logger_ = std::make_shared<amr_logging::NodeLogger>(
                        base_path,
                        "your_package_name",
                        "your_node_name",
                        file_type,
                        log_level
                );
        }
};
```

Log Messages:
```cpp
// Log at different levels
logger_->log_debug("Debug message", __FILE__, __LINE__, __FUNCTION__);
logger_->log_info("Info message", __FILE__, __LINE__, __FUNCTION__);
logger_->log_warn("Warning message", __FILE__, __LINE__, __FUNCTION__);
logger_->log_error("Error message", __FILE__, __LINE__, __FUNCTION__);
logger_->log_fatal("Fatal message", __FILE__, __LINE__, __FUNCTION__);
```

### Advanced Usage

Helper Class Integration

Helper Class Header:
```cpp
class HelperClass {
private:
        std::shared_ptr<amr_logging::NodeLogger> logger_;

public:
        explicit HelperClass(std::shared_ptr<amr_logging::NodeLogger> logger);
        void doSomething();
};
```

Helper Class Implementation:
```cpp
HelperClass::HelperClass(std::shared_ptr<amr_logging::NodeLogger> logger)
        : logger_(logger)
{
        logger_->log_info("Helper class initialized", __FILE__, __LINE__, __FUNCTION__);
}

void HelperClass::doSomething()
{
        logger_->log_info("Doing something", __FILE__, __LINE__, __FUNCTION__);
}
```

### Configuration

YAML Configuration. Refer to the **`amr_navigation_system/config/global_path_planner_params.yaml`**

Create `config/params.yaml`:
```yaml
your_node:
    ros__parameters:
        log_base_path: "~/logs"
        log_level: 1  # INFO
        log_file_type: "json"  # or "txt"
```

## Examples

### JSON Log Format
```json
{
        "timestamp": "2024-10-18_14-30-45.123 UTC",
        "level": "INFO",
        "node_name": "navigation_node",
        "package_name": "amr_navigation",
        "message": "Navigation started",
        "file_name": "navigation_node.cpp",
        "line_number": 42,
        "function_name": "start_navigation",
        "thread_id": "12345"
}
```

### Plain Text Format
```plaintext
2024-10-18_14-30-45.123 UTC [INFO] navigation_node amr_navigation: Navigation started (navigation_node.cpp:42 in start_navigation)
```

## Best Practices

### Log Level Usage:
- **DEBUG**: Detailed information for debugging
- **INFO**: General operational information
- **WARN**: Unexpected but handleable situations
- **ERROR**: Serious issues that need attention
- **FATAL**: Critical errors causing system shutdown

### Memory Management:
- Store logger as `shared_ptr` in node class
- Pass logger to helper classes as `shared_ptr`
- Let destructor handle cleanup

### Thread Safety:
- Logger is thread-safe by design
- No need for additional synchronization
- Avoid holding locks while logging

### Performance Considerations:
- Use appropriate log levels to control verbosity
- Avoid excessive logging in tight loops
- Utilize asynchronous logging for better performance

## Troubleshooting

### Common Issues

#### Permission Denied:
- Ensure write permissions for log directory
- Check if `base_path` is accessible
- Verify user permissions

#### Missing Logs:
- Check log level configuration
- Verify log file path
- Ensure logger initialization success

#### Performance Issues:
- Reduce logging frequency by adjusting `queue_size`
- Increase log level threshold
- Check disk space availability

### Debug Tips

Print log file path during initialization:
```cpp
RCLCPP_INFO(get_logger(), "Log file path: %s", log_file_path_.c_str());
```

Verify logger initialization:
```cpp
if (logger_) {
        RCLCPP_INFO(get_logger(), "Logger initialized successfully");
} else {
        RCLCPP_ERROR(get_logger(), "Logger initialization failed");
}
```
