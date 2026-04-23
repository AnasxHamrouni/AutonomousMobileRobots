FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install required ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-forward-command-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    python3-colcon-common-extensions \
    python3-pip \
    # GL libraries for Gazebo rendering
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install numpy

# Create workspace
WORKDIR /amr_ws/src

# Copy exam package
COPY . /amr_ws/src/exam/

# Build the package
WORKDIR /amr_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select exam"

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
