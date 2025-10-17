#!/bin/bash
# Run integration tests inside Docker container

set -e

echo "=== MicroSim Integration Tests (Docker) ==="
echo

# Source ROS 2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /workspace

# Build the package
echo "Building package..."
colcon build --symlink-install --packages-select microsim

# Source the workspace
source install/setup.bash

# Start the node in background
echo "Starting MicroSim node..."
ros2 run microsim microsim_node &
NODE_PID=$!

# Wait for initialization
echo "Waiting for node to initialize (3 seconds)..."
sleep 3

# Run integration tests
echo "Running integration tests..."
python3 -m pytest test/test_integration.py -v

TEST_RESULT=$?

# Cleanup
echo "Stopping node..."
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

if [ $TEST_RESULT -eq 0 ]; then
    echo "✓ All integration tests passed!"
else
    echo "✗ Some integration tests failed"
fi

exit $TEST_RESULT
