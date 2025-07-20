FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/workspace

# Install additional packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    vim \
    wget \
    curl \
    build-essential \
    cmake \
    gdb \
    libeigen3-dev \
    libgtest-dev \
    libgazebo11-dev \
    gazebo11 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-moveit \
    ros-humble-control-msgs \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-realtime-tools \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN useradd -m -s /bin/bash ros && \
    usermod -aG sudo ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the ros user
USER ros
WORKDIR /home/ros

# Setup ROS environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias sb='source install/setup.bash'" >> ~/.bashrc && \
    echo "alias ws='cd /workspace'" >> ~/.bashrc

# Set the working directory
WORKDIR $WORKSPACE

# Initialize rosdep
RUN sudo rosdep init || true
RUN rosdep update

# Default command
CMD ["/bin/bash"]
