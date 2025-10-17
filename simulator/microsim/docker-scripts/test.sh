#!/bin/bash
# Run tests inside the Docker container

set -e

echo "======================================"
echo "Running MicroSim Tests in Docker"
echo "======================================"

cd /workspace

# Run Python unit tests (no ROS needed)
echo ""
echo "Running Python unit tests..."
python3 -m pytest test/ -v --tb=short

echo ""
echo "======================================"
echo "All tests passed!"
echo "======================================"
