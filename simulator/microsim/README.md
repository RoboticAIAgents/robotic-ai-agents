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

**📖 See [DOCKER.md](DOCKER.md) for complete Docker guide**

### Option 2: Native ROS 2

#### Prerequisites

- ROS 2 (Humble or Iron)
- Python 3.8+
- NumPy, PyYAML

#### Build

```bash
# From workspace root
colcon build --packages-select microsim

# Source the workspace
source install/setup.bash
```

#### Run

```bash
# Start simulator
ros2 run microsim microsim_node

# In another terminal, visualize in RViz2
rviz2

# Send velocity commands (example)
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

## Topics

### Drone
- `/drone/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/drone/odom` (nav_msgs/Odometry) - Ground truth odometry
- `/drone/gps` (sensor_msgs/NavSatFix) - Noisy GPS position
- `/drone/camera/image_raw` (sensor_msgs/Image) - RGB camera (128x128, rgb8)
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

## Development Status

**Sprint 0 Complete** ✓
- Package structure created
- Module skeletons implemented
- Basic node infrastructure

**Sprint 1** (In Progress)
- Core simulation loop
- World grid and obstacles
- Full sensor implementations

**Sprint 2** (Planned)
- YAML scenario loading
- Radio link integration

**Sprint 3** (Planned)
- RGB camera ray-casting and rendering
- CameraInfo publishing

**Sprint 4** (Planned)
- Determinism verification
- Documentation and acceptance tests

## License

MIT
