# ROS2 Foundation Development Scripts

## Quick Setup
```bash
# Build workspace
./scripts/build.sh

# Source workspace
source install/setup.bash

# Run basic system
./scripts/run_basic.sh

# Run simulation
./scripts/run_simulation.sh
```

## Development Workflow

### Building
```bash
# Clean build
./scripts/build.sh --clean

# Build specific packages
colcon build --packages-select foundation_interfaces foundation_common

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing
```bash
# Run all tests
./scripts/test.sh

# Run specific package tests
colcon test --packages-select foundation_common

# Generate coverage report
./scripts/coverage.sh
```

### Docker Development
```bash
# Start development container
docker-compose up -d

# Access container
docker exec -it ros2_dev bash

# Build inside container
cd /workspace && colcon build

# Run simulation in container
docker-compose --profile simulation up
```
