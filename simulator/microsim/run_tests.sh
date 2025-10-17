#!/bin/bash
# Test runner script for MicroSim

set -e  # Exit on error

echo "======================================"
echo "MicroSim Test Suite"
echo "======================================"
echo ""

# Check if pytest is installed
if ! command -v pytest &> /dev/null; then
    echo "ERROR: pytest not found. Install with:"
    echo "  pip3 install pytest"
    exit 1
fi

# Check if numpy is installed
python3 -c "import numpy" 2>/dev/null || {
    echo "ERROR: numpy not found. Install with:"
    echo "  pip3 install numpy"
    exit 1
}

echo "Running unit tests..."
echo ""

# Run tests with pytest
cd "$(dirname "$0")"

# Run different test categories
echo "1. Running timekeeper tests..."
pytest test/test_timekeeper.py -v

echo ""
echo "2. Running physics tests..."
pytest test/test_physics.py -v

echo ""
echo "3. Running world tests..."
pytest test/test_world.py -v

echo ""
echo "4. Running sensor tests..."
pytest test/test_sensors.py -v

echo ""
echo "5. Running determinism tests..."
pytest test/test_determinism.py -v

echo ""
echo "======================================"
echo "Running all tests together..."
echo "======================================"
pytest test/ -v --tb=short

echo ""
echo "======================================"
echo "Test Summary"
echo "======================================"
pytest test/ --tb=no --quiet

echo ""
echo "All tests passed! ✓"
