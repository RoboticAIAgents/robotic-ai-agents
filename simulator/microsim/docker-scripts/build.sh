#!/bin/bash
# Build the MicroSim package inside the Docker container

set -e

echo "======================================"
echo "Building MicroSim in Docker"
echo "======================================"

# Make sure we're in the workspace
cd /workspace

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the package
echo ""
echo "Running colcon build..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

echo ""
echo "======================================"
echo "Build complete!"
echo "======================================"
echo ""
echo "Package is ready to use. Try:"
echo "  ros2 run microsim microsim_node"
echo ""
