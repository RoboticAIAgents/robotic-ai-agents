#!/bin/bash
# Script to run MicroSim integration tests
# These tests start the actual ROS 2 node and verify behavior through topic interactions

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== MicroSim Integration Tests ===${NC}"
echo

# Check if we're in a ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS 2 environment not sourced${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo -e "${YELLOW}Step 1: Building the package...${NC}"
colcon build --symlink-install --packages-select microsim

echo
echo -e "${YELLOW}Step 2: Sourcing workspace...${NC}"
source install/setup.bash

echo
echo -e "${YELLOW}Step 3: Starting MicroSim node in background...${NC}"
ros2 run microsim microsim_node &
NODE_PID=$!
echo "Started node with PID: $NODE_PID"

# Wait for node to initialize
echo "Waiting for node to initialize (3 seconds)..."
sleep 3

# Check if node is still running
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo -e "${RED}Error: MicroSim node failed to start${NC}"
    exit 1
fi

echo
echo -e "${YELLOW}Step 4: Running integration tests...${NC}"
echo

# Run the integration tests
if python3 -m pytest test/test_integration.py -v; then
    TEST_RESULT=0
    echo
    echo -e "${GREEN}✓ Integration tests passed!${NC}"
else
    TEST_RESULT=1
    echo
    echo -e "${RED}✗ Integration tests failed${NC}"
fi

# Clean up: kill the node
echo
echo -e "${YELLOW}Step 5: Cleaning up...${NC}"
echo "Stopping MicroSim node (PID: $NODE_PID)"
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

echo -e "${GREEN}Done!${NC}"
exit $TEST_RESULT
