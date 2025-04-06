# AMR-Logistic-System

This repository contains the code and documentation for an Autonomous Mobile Robot (AMR) designed for efficient delivery and logistics operations. The system integrates advanced navigation, sensor fusion, and path planning algorithms to ensure reliable and autonomous delivery within various environments.

## Containerized Approach

The AMR-Logistic-System now uses a containerized approach with Docker, making deployment and setup much easier across different environments.

## System Requirements

- Ubuntu Linux (recommended: 22.04)
- ROS2 Humble - Install following the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Docker and Docker Compose
- Sufficient disk space (~3GB for Docker images and containers)

## Quick Start Guide

### 1. Create ROS2 Workspace and Clone Repository

```bash
# Create ROS2 workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Abhi-0212000/AMR-Logistic-System.git
```

### 2. Set Up Host-Container Communication

To enable communication between your host system and the Docker container, add the following environment variables to your `~/.bashrc` file:

```bash
# Add these lines to ~/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/.ros/cyclonedds.xml
export ROS_DOMAIN_ID=99  # Choose any number between 1-256
```

Then source your bashrc file:
```bash
source ~/.bashrc
```

### 3. Configure CycloneDDS

Copy the provided CycloneDDS configuration file to your ROS configuration directory:

```bash
# Create the directory if it doesn't exist
mkdir -p ~/.ros

# Copy the configuration file
cp ~/ros2_ws/src/AMR-Logistic-System/cyclonedds.xml ~/.ros/
```

### 4. Install Docker

Install Docker by following the official Docker installation guide for Ubuntu:
[https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

### 5. Launch the Containerized System

```bash
# Navigate to the repository directory
cd ~/ros2_ws/src/AMR-Logistic-System/

# Start the Docker containers
docker compose up
```

## Repository Structure

This repository currently includes two ROS2 packages:

1. **amr_interfaces**:
    - Provides custom ROS2 interface types.
    - Includes `ComputeGlobalPath.srv` service for communication between planning nodes.

2. **amr_navigation_system**:
    - ROS2 package for autonomous navigation using the Lanelet2 library.
    - Implements global path planning, GPS filtering, and map boundary validation.

## Working with the Containerized Environment

### Accessing the Container

To start an interactive shell session in the running container:

```bash
docker exec -it amr-logistic-system-container bash
```

### Running ROS2 Commands on the Host

With the proper communication settings, you can run ROS2 commands on your host that interact with the container:

```bash
# List active ROS2 topics
ros2 topic list

# View node graph
ros2 node list
```

## Troubleshooting

### Communication Issues

- Ensure that the `ROS_DOMAIN_ID` is the same on both host and container
- Verify that the CycloneDDS configuration is correctly set up
- Check that the `cyclonedds.xml` file is in the correct location

### Docker Issues

- Ensure Docker service is running: `sudo systemctl status docker`
- Check Docker logs for errors: `docker logs amr-logistic-system-container`

## Documentation

For detailed information on using the packages and their functionalities, refer to the inline code documentation and additional documentation files:

- `amr_navigation_system/amr_navigation_system.md`
- `amr_navigation_system/utils/logger.md` - Logging System Documentation

## Future Plans

This package is intended for internal use at Hochschule Schmalkalden for R&D, teaching, and research purposes. It serves as a foundation for future publications and enhancements.

## Acknowledgments

This project utilizes the following open-source resources:
- **Lanelet2**: For map handling and routing
- **OpenStreetMap (OSM)**: For generating Lanelet2 maps
- **ROS2 Humble**: Robotics middleware
- **CycloneDDS**: Fast and robust DDS implementation

## Support

For additional help:
- Contact the development team: abhishek.nannuri@outlook.com
