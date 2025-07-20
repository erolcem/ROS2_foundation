#!/bin/bash

# ROS2 Foundation Setup Script
# This script helps you get started with the ROS2 Robotics Foundation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "🚀 ROS2 Robotics Foundation Setup"
echo "=================================="
echo ""

# Check if we're in Docker
if [ -f /.dockerenv ]; then
    echo "✅ Running inside Docker container"
    IN_DOCKER=true
else
    echo "ℹ️  Running on native system"
    IN_DOCKER=false
fi

cd "$WORKSPACE_DIR"

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo "📋 Checking dependencies..."

if [ "$IN_DOCKER" = false ]; then
    # Check ROS2 installation
    if ! command_exists ros2; then
        echo "❌ ROS2 not found. Please install ROS2 Humble first."
        echo "   Follow: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
    
    # Check colcon
    if ! command_exists colcon; then
        echo "❌ colcon not found. Installing..."
        sudo apt update
        sudo apt install -y python3-colcon-common-extensions
    fi
    
    # Check rosdep
    if ! command_exists rosdep; then
        echo "❌ rosdep not found. Installing..."
        sudo apt install -y python3-rosdep
    fi
fi

echo "✅ Dependencies check complete"

# Initialize rosdep if needed
echo "🔧 Setting up rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

# Install workspace dependencies
echo "📦 Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
echo "🔨 Building workspace..."
if [ "$IN_DOCKER" = true ]; then
    source /opt/ros/humble/setup.bash
fi

colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --parallel-workers $(nproc)

echo ""
echo "✅ Build completed successfully!"

# Create convenience script
echo "📝 Creating convenience scripts..."

cat > setup_workspace.sh << 'EOF'
#!/bin/bash
# Convenience script to source the workspace
source install/setup.bash
echo "✅ ROS2 Foundation workspace sourced!"
echo "Try: ros2 launch foundation_bringup basic_system.launch.py"
EOF

chmod +x setup_workspace.sh

echo ""
echo "🎉 Setup Complete!"
echo "=================="
echo ""
echo "Next steps:"
echo "1. Source the workspace:    source install/setup.bash"
echo "   Or use:                  ./setup_workspace.sh"
echo ""
echo "2. Run basic system:        ros2 launch foundation_bringup basic_system.launch.py"
echo "   Or use:                  ./scripts/run_basic.sh"
echo ""
echo "3. Run simulation:          ros2 launch foundation_simulation simulation.launch.py"
echo "   Or use:                  ./scripts/run_simulation.sh"
echo ""
echo "4. Read documentation:      docs/getting_started.md"
echo ""
echo "Happy robotics development! 🤖"
