# ROS2 Robotics Foundation

A comprehensive, Docker-based ROS2 foundation that serves as a starting point for building modular robotics systems. This repository provides a solid foundation for integrating robot arms, mobile bases, sensors, and other robotics components.

## 🚀 Quick Start

Choose your preferred setup method:

### Option A: Docker Setup (Recommended)

**Prerequisites:**
- Docker and Docker Compose installed
- Git

**Step 1: Clone the repository**
```bash
git clone https://github.com/erolcem/ROS2_foundation.git
cd ROS2_foundation
```

**Step 2: Setup X11 forwarding for GUI (Linux)**
```bash
# Allow Docker to access display
xhost +local:docker

# Create X11 auth file for Docker
touch /tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
```

**Step 3: Build and start the development environment**
```bash
# Build the Docker image and start container
docker-compose up -d

# Check if container is running
docker ps
```

**Step 4: Access the container and build**
```bash
# Enter the development container
docker exec -it ros2_dev bash

# Inside container: Run the setup script
./setup.sh

# Or build manually:
# colcon build --symlink-install
# source install/setup.bash
```

**Step 5: Test the system**
```bash
# Inside container: Launch the basic system
ros2 launch foundation_bringup basic_system.launch.py
```

### Option B: Native Installation (Alternative)

**Prerequisites:**
- Ubuntu 22.04 LTS
- ROS2 Humble Desktop Full

**Step 1: Install ROS2 Humble (if not installed)**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble Desktop Full
sudo apt install ros-humble-desktop-full

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep
```

**Step 2: Clone and build**
```bash
# Clone repository
git clone https://github.com/erolcem/ROS2_foundation.git
cd ROS2_foundation

# Run setup script
./setup.sh

# Or build manually:
# source /opt/ros/humble/setup.bash
# rosdep install --from-paths src --ignore-src -r -y
# colcon build --symlink-install
# source install/setup.bash
```

### 🔧 Troubleshooting Docker

If Docker build fails with GPG errors:

```bash
# Clean Docker cache
docker system prune -a

# Rebuild with no cache
docker-compose build --no-cache

# Alternative: Use pre-built image
docker pull osrf/ros:humble-desktop-full
```

## 🏗️ Architecture

This foundation is built with modularity in mind:

### Core Components
- **foundation_interfaces**: Custom messages, services, and actions
- **foundation_common**: Shared utilities and base classes
- **foundation_bringup**: Launch files and system configuration
- **foundation_description**: Robot description files (URDF/Xacro)
- **foundation_simulation**: Gazebo simulation setup

### Modular Extensions
- **Robot Arms**: Easy integration of manipulator arms
- **Mobile Bases**: Support for various mobile platforms
- **Sensors**: Camera, LiDAR, IMU integration
- **Navigation**: Advanced navigation capabilities
- **Perception**: Object detection and recognition

## 📚 Documentation

- [Getting Started Guide](docs/getting_started.md)
- [Architecture Overview](docs/architecture.md)
- [Adding New Components](docs/adding_components.md)
- [Docker Development](docs/docker_development.md)
- [Tutorials](docs/tutorials/)

## 🔧 Development

### Workspace Structure
```
src/
├── foundation_interfaces/     # Custom interfaces
├── foundation_common/         # Shared utilities
├── foundation_bringup/        # Launch configurations
├── foundation_description/    # Robot descriptions
├── foundation_simulation/     # Simulation setup
└── examples/                  # Example implementations
```

### Contributing
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests and documentation
5. Submit a pull request

## 📋 Features

- ✅ Docker-based development environment
- ✅ Modular architecture for easy extension
- ✅ Comprehensive documentation and tutorials
- ✅ Example implementations
- ✅ CI/CD pipeline setup
- ✅ ROS2 best practices
- ✅ Simulation ready (Gazebo)
- ✅ Navigation stack integration
- ✅ Sensor fusion capabilities

## 🎯 Use Cases

This foundation is perfect for:
- Educational robotics projects
- Research and development
- Prototyping new robotic systems
- Learning ROS2 best practices
- Building commercial robotics applications

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Support

- Create an issue for bug reports or feature requests
- Check the [documentation](docs/) for detailed guides
- Join our community discussions

---

Built with ❤️ for the robotics community
