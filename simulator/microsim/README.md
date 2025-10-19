# MicroSim

Minimal, deterministic, real-time ROS 2 simulator for Drone and Rover agents.

## Overview

MicroSim provides a lightweight simulation environment with:
- Fixed-step deterministic execution (60 Hz)
- Two robots: 6-DOF kinematic drone + differential-drive rover
- Sensors: GPS, RGB pinhole camera (drone), forward range sensor (rover)
- Simple radio link model for inter-robot communication
- Clean ROS 2 interface (topics, services, parameters)

## Quick Start

### Option 1: Docker (Recommended) 🐳

**No ROS 2 installation required!** Use Docker for a clean, isolated environment.

```bash
# Start ROS 2 container
docker-compose up -d

# Enter container
docker-compose exec ros2 bash

# Inside container: Build and run
./docker-scripts/build.sh
./docker-scripts/run_node.sh
```

**📖 See [docs/DOCKER.md](docs/DOCKER.md) for complete Docker guide**

### Option 2: Native ROS 2 (macOS/Linux)

**Windows users:** See [docs/WINDOWS_SETUP.md](docs/WINDOWS_SETUP.md) for Windows-specific instructions.

#### Prerequisites

- ROS 2 Humble (via robostack on macOS, or native on Linux)
- Python 3.8+
- NumPy, PyYAML, matplotlib (for visualization)

#### Install on macOS

```bash
# Install miniforge
curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh
bash Miniforge3-MacOSX-arm64.sh

# Create ROS 2 environment
conda create -n ros2_humble
conda activate ros2_humble
conda install -c robostack-staging ros-humble-desktop python=3.11

# Install dependencies
conda install matplotlib numpy pyyaml
```

#### Build

```bash
# Navigate to workspace
cd /path/to/microsim

# Install in editable mode (allows live YAML editing)
pip install -e . --no-deps

# Or use colcon (traditional ROS 2 way)
colcon build --packages-select microsim
source install/setup.bash
```

#### Run

```bash
# Terminal 1: Start simulator
conda activate ros2_humble
source install/setup.bash  # if using colcon
ros2 run microsim microsim_node

# Terminal 2: Launch visualization (macOS-compatible)
conda activate ros2_humble
python3 scripts/viz_2d.py

# Terminal 3: Control the drone interactively
conda activate ros2_humble
python3 scripts/drone_controller.py

# Or send manual velocity commands
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}" --rate 10
```

**Note:** RViz2 has stability issues on macOS. Use the included `scripts/viz_2d.py` for visualization instead.

### Interactive Drone Controller

The `drone_controller.py` script provides keyboard-based control for testing:

```bash
python3 scripts/drone_controller.py
```

**Controls:**
- **W/S** - Forward/Backward
- **A/D** - Left/Right (strafe)
- **Q/E** - Rotate Left/Right
- **R/F** - Up/Down (altitude)
- **+/-** - Increase/Decrease speed multiplier
- **SPACE** - Stop all movement
- **H** - Hover (stop horizontal, maintain altitude)
- **L** - Land (descend slowly)
- **0** - Reset simulation
- **ESC** - Exit controller

The controller publishes commands at 10 Hz and displays real-time velocity feedback.

### Autonomous Controller (AI Development Template)

For building AI-based autonomous controllers, use the reference implementation:

```bash
python3 scripts/autonomous_drone_controller.py
```

This provides a clean template showing:
- ROS 2 sensor integration (odometry, GPS, camera)
- Control loop structure (perceive → think → act)
- Simple waypoint following as baseline
- Clear hooks for AI integration (LLM, RL, vision models)

**See [docs/AI_CONTROLLER_GUIDE.md](docs/AI_CONTROLLER_GUIDE.md) for detailed AI integration patterns.**

The reference controller demonstrates:
- **Perception:** Process sensor data (odometry, GPS, camera)
- **Decision:** AI decision-making hook (replace `simple_decision_logic()`)
- **Action:** Velocity command generation and publishing

Perfect starting point for intelligent drone controllers using AI agents.

## Topics

### Drone
- `/drone/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/drone/odom` (nav_msgs/Odometry) - Ground truth odometry
- `/drone/gps` (sensor_msgs/NavSatFix) - Noisy GPS position
- `/drone/camera/image_raw` (sensor_msgs/Image) - RGB camera (256x256, rgb8)
- `/drone/camera/camera_info` (sensor_msgs/CameraInfo) - Camera intrinsics

### Rover
- `/rover/cmd_vel` (geometry_msgs/Twist) - Velocity commands (linear.x, angular.z)
- `/rover/odom` (nav_msgs/Odometry) - Ground truth odometry
- `/rover/gps` (sensor_msgs/NavSatFix) - Noisy GPS position
- `/rover/range` (sensor_msgs/Range) - Forward-facing distance sensor

### Radio Communication
- `/radio/drone_tx` (std_msgs/String) - Drone transmit (publish here to send to rover)
- `/radio/rover_tx` (std_msgs/String) - Rover transmit (publish here to send to drone)
- `/radio/drone_rx` (std_msgs/String) - Drone receive (subscribe to get messages from rover)
- `/radio/rover_rx` (std_msgs/String) - Rover receive (subscribe to get messages from drone)

Messages experience realistic network effects: latency (50ms ±10ms), packet loss (1%), distance limiting (100m).

## Services

- `~/reset` (std_srvs/Empty) - Reset simulation to initial state
- `~/pause` (std_srvs/Trigger) - Pause/unpause simulation

## TF Tree

```
world
├── drone/base_link
│   └── drone/camera_link
└── rover/base_link
```

## Architecture

- **timekeeper.py** - Fixed-step time management (60 Hz)
- **world.py** - 2D grid world with semantic labels
- **physics.py** - Kinematic models for drone (6-DOF) and rover (diff-drive)
- **sensors.py** - GPS and range sensor implementations
- **camera.py** - RGB pinhole camera with semantic rendering
- **radio.py** - Radio link model for inter-robot communication
- **tf_broadcaster.py** - TF transform publishing
- **microsim_node.py** - Main ROS 2 node

## Documentation

- **[docs/QUICKSTART.md](docs/QUICKSTART.md)** - Get started in 3 steps
- **[docs/USAGE.md](docs/USAGE.md)** - Complete command reference
- **[docs/WORLD_CONFIGURATION.md](docs/WORLD_CONFIGURATION.md)** - Configure world, features, sensors
- **[docs/AI_CONTROLLER_GUIDE.md](docs/AI_CONTROLLER_GUIDE.md)** - Build AI-based controllers
- **[docs/WINDOWS_SETUP.md](docs/WINDOWS_SETUP.md)** - **Windows installation guide**
- **[docs/DOCKER.md](docs/DOCKER.md)** - Docker setup guide
- **[docs/TEST_SUITE.md](docs/TEST_SUITE.md)** - Test documentation

## Features

✓ **Deterministic simulation** - Fixed timestep (60 Hz), seeded RNGs
✓ **Dual robots** - 6-DOF drone + differential-drive rover
✓ **Rich sensors** - GPS (noisy), downward camera (256×256 RGB), range sensor
✓ **Radio link** - Inter-robot communication with latency/jitter/packet loss
✓ **World features** - Obstacles, hazards, targets with semantic labeling
✓ **Live editing** - Modify scenario YAML without reinstalling
✓ **macOS support** - Native visualization using matplotlib
✓ **Comprehensive tests** - 72 unit tests + 8 integration tests, all passing

## License

MIT
