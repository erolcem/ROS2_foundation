#!/bin/bash

# Run basic system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Source the workspace
source install/setup.bash

# Run the basic system
echo "Starting ROS2 Foundation basic system..."
ros2 launch foundation_bringup basic_system.launch.py
