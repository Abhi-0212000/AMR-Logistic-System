# Use ROS2 Humble as base image
FROM ros:humble

# Set non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# COPY ./amr_interfaces /ros2_ws/src/amr_interfaces
# COPY ./amr_navigation_system /ros2_ws/src/amr_navigation_system
COPY ./ /ros2_ws/src/

WORKDIR /ros2_ws

# Install basic development tools and dependencies
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        git \
        build-essential \
        cmake \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already done
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

# Install dependencies using rosdep
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build all packages
RUN . /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# Set the entrypoint directly in the Dockerfile
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
