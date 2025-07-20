#!/bin/bash

# Run simulation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Source the workspace
source install/setup.bash

# Run the simulation
echo "Starting ROS2 Foundation simulation..."
ros2 launch foundation_simulation simulation.launch.py world:=foundation_world.sdf
