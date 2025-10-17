#!/bin/bash
# Run the MicroSim node inside Docker

set -e

cd /workspace

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build if needed
if [ ! -d "install" ]; then
    echo "Building package first..."
    colcon build --symlink-install
fi

# Source workspace
source install/setup.bash

echo "======================================"
echo "Starting MicroSim Node"
echo "======================================"
echo ""
echo "Node will publish to:"
echo "  /drone/odom, /drone/gps, /drone/camera/image_raw"
echo "  /rover/odom, /rover/gps, /rover/range"
echo "  /radio/drone_rx, /radio/rover_rx"
echo ""
echo "Send commands to:"
echo "  /drone/cmd_vel, /rover/cmd_vel"
echo "  /radio/drone_tx, /radio/rover_tx"
echo ""
echo "Services:"
echo "  /microsim/reset, /microsim/pause"
echo ""
echo "======================================"
echo ""

# Run the node
ros2 run microsim microsim_node
