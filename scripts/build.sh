#!/bin/bash

# ROS2 Foundation Build Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Parse arguments
CLEAN_BUILD=false
PACKAGES=""
CMAKE_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --packages)
            PACKAGES="--packages-select $2"
            shift 2
            ;;
        --debug)
            CMAKE_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=Debug"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

cd "$WORKSPACE_DIR"

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo "Cleaning build artifacts..."
    rm -rf build/ install/ log/
fi

# Update rosdep if needed
echo "Updating rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

# Install dependencies
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "Building workspace..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    $CMAKE_ARGS \
    $PACKAGES

echo "Build completed successfully!"
echo "To use the workspace, run: source install/setup.bash"
