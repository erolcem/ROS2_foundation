# ROS2 Robotics Foundation

A comprehensive, Docker-based ROS2 foundation that serves as a starting point for building modular robotics systems. This repository provides a solid foundation for integrating robot arms, mobile bases, sensors, and other robotics components.

## 🚀 Quick Start

### Prerequisites
- Docker and Docker Compose
- Git

### Getting Started
```bash
# Clone the repository
git clone <your-repo-url>
cd ros2-robotics-foundation

# Build and start the development environment
docker-compose up -d

# Access the container
docker exec -it ros2_dev bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Run example launch file
ros2 launch foundation_bringup basic_system.launch.py
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
