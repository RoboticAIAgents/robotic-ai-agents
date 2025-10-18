# MicroSim Test Suite

## Overview

Comprehensive test suite for MicroSim simulator with two types of tests:
- **Unit Tests**: Standalone tests that don't require ROS 2 - fast, deterministic, CI-friendly
- **Integration Tests**: Full system tests that run the actual ROS 2 node and verify behavior

## Test Statistics

- **Unit Tests**: 117 tests across 8 files
- **Integration Tests**: 9 end-to-end scenarios
- **All Passing**: ✓
- **Coverage**: Core simulation logic, ROS 2 integration, sensor publishing, radio communication

## Running Tests

### Unit Tests (Fast, No ROS 2 Required)

```bash
# Run all unit tests
python3 -m pytest test/ -v

# Run specific test file
python3 -m pytest test/test_timekeeper.py -v

# Run with coverage
python3 -m pytest test/ --cov=microsim --cov-report=html

# Use the test runner script
./run_tests.sh
```

**Prerequisites**: `pip3 install pytest numpy`

### Integration Tests (Requires ROS 2)

```bash
# Run integration tests (starts actual ROS 2 node)
./run_integration_tests.sh

# Or in Docker
./docker-scripts/integration_test.sh
```

**Prerequisites**:
- ROS 2 Humble sourced
- Package built with `colcon build`

**What Integration Tests Verify**:
- All topics publish at expected rates (~60 Hz odom, ~10 Hz GPS, etc.)
- Velocity commands cause robots to move correctly
- Radio communication works with latency/jitter/packet loss
- GPS readings match odometry positions
- Camera publishes valid RGB images
- Range sensor responds to obstacles

## Test Organization

### [test_timekeeper.py](test/test_timekeeper.py) (8 tests)

Tests for fixed-step time management:
- ✓ Initialization with correct defaults
- ✓ Custom timestep configuration
- ✓ Time advancement on step()
- ✓ Precision over 600 steps (10 seconds)
- ✓ Reset to initial state
- ✓ Deterministic time sequences

**Key Validation**: Time accumulation maintains precision over long runs (1 hour = 216,000 steps)

### [test_physics.py](test/test_physics.py) (21 tests)

Tests for drone and rover kinematics:

**Drone (6-DOF) - 10 tests**:
- ✓ Forward, lateral, vertical motion
- ✓ Yaw rotation
- ✓ Body-frame velocity commands
- ✓ Velocity clamping to max
- ✓ Ground collision prevention
- ✓ Yaw normalization to [-π, π]

**Rover (Diff-Drive) - 9 tests**:
- ✓ Forward/backward motion
- ✓ Rotation in place
- ✓ Curved trajectories
- ✓ Heading affects motion direction
- ✓ Velocity/omega clamping

**Determinism - 2 tests**:
- ✓ Identical trajectories with same commands
- ✓ Reproducibility over 100 steps

### [test_world.py](test/test_world.py) (16 tests)

Tests for 2D grid world representation:
- ✓ Grid initialization and sizing
- ✓ Default values (height=0, semantic=GROUND)
- ✓ Out-of-bounds queries return safe defaults
- ✓ Setting circular regions (obstacles, hazards, targets, water)
- ✓ Multiple and overlapping regions
- ✓ Coordinate conversion (world ↔ grid)
- ✓ Fine and coarse resolution support
- ✓ Semantic class enum values

### [test_sensors.py](test/test_sensors.py) (18 tests)

Tests for GPS and range sensors:

**GPS - 7 tests**:
- ✓ Update rate timing (10 Hz)
- ✓ Gaussian noise addition
- ✓ Zero-mean noise statistics
- ✓ Seeding for determinism
- ✓ Different seeds produce different noise

**Range Sensor - 9 tests**:
- ✓ Update rate timing (20 Hz)
- ✓ Noise addition
- ✓ Clamping to [0, max_range]
- ✓ Zero-noise mode returns exact values
- ✓ Noise statistics (mean, std dev)

**Determinism - 2 tests**:
- ✓ Seeded GPS produces identical sequences
- ✓ Seeded range sensor produces identical sequences

### [test_determinism.py](test/test_determinism.py) (9 tests)

End-to-end determinism validation:

**System Determinism - 5 tests**:
- ✓ Complete simulation runs are identical with same seed
- ✓ Different seeds produce different sensor noise (but same trajectories)
- ✓ World state queries are deterministic
- ✓ Long simulations (3600 steps = 60s) remain deterministic
- ✓ Command replay produces identical results

**Radio Determinism - 2 tests**:
- ✓ Packet delivery is deterministic with seeding
- ✓ Different seeds produce different packet loss patterns

**Numerical Stability - 2 tests**:
- ✓ Drone position doesn't accumulate floating-point errors (36,000 steps)
- ✓ Time accumulation maintains precision (216,000 steps = 1 hour)

## Test Markers

Tests are tagged with pytest markers for selective execution:

```bash
# Run only unit tests
python3 -m pytest -m unit

# Run only determinism tests
python3 -m pytest -m determinism

# Run integration tests (when ROS 2 is available)
python3 -m pytest -m integration
```

## Continuous Integration

These tests are designed to run in CI environments:
- No ROS 2 dependency
- No GUI/display required
- Fast execution (~2 seconds total)
- Deterministic (no flaky tests)
- Clear pass/fail criteria

## Key Properties Verified

1. **Determinism**: Same seed → same results (critical for ML/RL)
2. **Numerical Stability**: No drift over long simulations
3. **Correct Physics**: Kinematic models behave as expected
4. **Sensor Realism**: Noise statistics match specifications
5. **Boundary Conditions**: Out-of-bounds, clamping, edge cases
6. **API Contracts**: All public methods work as documented

## Integration Test Details

### [test_integration.py](test/test_integration.py) (9 tests)

End-to-end tests that verify the complete system behavior:

**Test Suite**:
- ✓ All topics publishing (odom, GPS, range, camera)
- ✓ Topic rates meet specifications (~60 Hz, ~10 Hz, ~20 Hz)
- ✓ Drone command-to-movement (velocity commands → position changes)
- ✓ Rover command-to-movement (diff-drive kinematics)
- ✓ Radio communication in range (with latency/jitter)
- ✓ GPS position matches odometry
- ✓ Camera image format (128x128 RGB, valid intrinsics)
- ✓ Range sensor responds to obstacles
- ✓ Message timing and synchronization

**How They Work**:
1. Start the MicroSim ROS 2 node in background
2. Create a test node that subscribes to all topics
3. Publish commands and verify expected behavior
4. Check message rates, formats, and content
5. Clean up and report results

**Run Time**: ~30 seconds for full suite

## Contributing

When adding new modules:
1. Create corresponding `test_<module>.py` file
2. Use `@pytest.mark.unit` for standalone tests
3. Use `@pytest.mark.integration` for ROS 2 tests
4. Verify determinism with seeded RNGs
5. Test boundary conditions and error cases
6. Aim for >90% code coverage

Run tests before committing:
```bash
./run_tests.sh
```
