# Getting Started with ROS2 Robotics Foundation

Welcome to the ROS2 Robotics Foundation! This guide will help you get up and running quickly.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- Docker and Docker Compose
- Git
- At least 4GB RAM and 10GB free disk space

### Optional (for native development)
- ROS2 Humble Desktop Full
- Gazebo 11
- Python 3.10+
- C++17 compatible compiler

## Quick Start (Docker - Recommended)

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd ros2-robotics-foundation
```

### 2. Setup X11 forwarding (for GUI)
```bash
# Allow Docker to access X11
xhost +local:docker

# Create X11 auth file
touch /tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
```

### 3. Build and Start Development Environment
```bash
# Build the Docker image and start container
docker-compose up -d

# Access the development container
docker exec -it ros2_dev bash
```

### 4. Build the Workspace (inside container)
```bash
cd /workspace

# Build all packages
./scripts/build.sh

# Source the workspace
source install/setup.bash
```

### 5. Run Basic System
```bash
# In the container
./scripts/run_basic.sh
```

### 6. Run Simulation (optional)
```bash
# Start simulation with Gazebo
./scripts/run_simulation.sh
```

## Native Installation (Advanced)

### 1. Install ROS2 Humble
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble Desktop Full
sudo apt install ros-humble-desktop-full

# Install additional dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 2. Clone and Build
```bash
# Clone repository
git clone <your-repo-url>
cd ros2-robotics-foundation

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
./scripts/build.sh

# Source workspace
source install/setup.bash
```

## Verification

### Test Basic System
```bash
# In one terminal
ros2 launch foundation_bringup basic_system.launch.py

# In another terminal
ros2 topic list
ros2 service list
ros2 node list
```

You should see topics like:
- `/robot_status`
- `/system_health`
- Various component status topics

### Test Simulation
```bash
# Start simulation
ros2 launch foundation_simulation simulation.launch.py

# In another terminal, check if robot is spawned
ros2 topic echo /robot_description
```

## Next Steps

1. **Explore the Architecture**: Read [Architecture Overview](architecture.md)
2. **Add Components**: Learn how to [Add New Components](adding_components.md)
3. **Follow Tutorials**: Check out the [Tutorials](tutorials/) directory
4. **Customize**: Modify the foundation for your specific needs

## Common Issues

### Docker X11 Issues
If you can't see GUI applications:
```bash
# Reset X11 permissions
xhost +local:docker
export DISPLAY=:0
```

### Build Errors
```bash
# Clean build
./scripts/build.sh --clean

# Update dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Permission Issues
```bash
# Fix file permissions after Docker usage
sudo chown -R $USER:$USER .
```

## Getting Help

- Check the [Documentation](docs/) folder
- Look at [Examples](src/foundation_examples/)
- Create an issue on GitHub
- Join the community discussions

Happy robotics development! 🚀
