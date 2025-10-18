# MicroSim Usage Guide

Quick reference for running and controlling the MicroSim simulator.

---

## Starting the Simulator

### On Mac (Native - Recommended)

```bash
# Activate ROS 2 environment
conda activate ros2_humble

# Navigate to workspace
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Source the workspace
source install/setup.bash

# Run the simulator
ros2 run microsim microsim_node
```

### In Docker

```bash
# Navigate to project
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Enter Docker container
docker-compose exec ros2 bash

# Inside Docker:
cd /workspace
source install/setup.bash
ros2 run microsim microsim_node
```

---

## Launching RViz (Visualization)

**Open a new terminal:**

### Option 1: With Saved Configuration (Recommended)

```bash
# Activate ROS 2 environment
conda activate ros2_humble

# Navigate to workspace
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Launch RViz with pre-configured displays
rviz2 -d config/microsim.rviz
```

This will automatically load all the configured displays:

- TF (coordinate frames)
- Drone & Rover Odometry
- World Features (obstacles, hazards, targets)
- Robot Bodies (3D models)
- Camera Feed (raw image from drone)

### Option 2: Basic Launch

```bash
# Activate ROS 2 environment
conda activate ros2_humble

# Launch RViz
rviz2
```

Then manually add displays via the "Add" button in RViz.

### Alternative: 2D Matplotlib Visualization (Recommended for macOS)

**RViz has known stability issues on macOS.** Use this lightweight Python-based alternative instead:

```bash
# Navigate to workspace
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Activate environment
conda activate ros2_humble
source install/setup.bash

# Launch 2D visualization (stable on macOS)
python3 scripts/viz_2d.py
```

This shows:

- **Left panel**: Top-down 2D map with drone (red arrow), rover (blue arrow), and world features
- **Right panel**: Live camera feed from drone
- Movement trails for both robots
- Real-time position display

**Advantages over RViz:**

- No OpenGL issues (uses matplotlib)
- More stable on macOS
- Lower resource usage
- Still shows all important information

### If RViz Crashes on macOS

RViz can crash on macOS due to OpenGL compatibility issues. **We recommend using the 2D matplotlib visualization above instead.**

If you still want to try RViz:

```bash
# Use the launch script with software rendering (slower but more stable)
./launch_rviz.sh

# Or with saved configuration
./launch_rviz.sh -d config/microsim.rviz
```

**Common RViz crash causes on Mac:**

- OpenGL 2.1 limitations (macOS doesn't support newer OpenGL)
- Graphics driver issues
- OGRE rendering engine incompatibility

**Known issue:** Even with software rendering, RViz may still crash with:

```
libc++abi: terminating due to uncaught exception of type std::__1::system_error: mutex lock failed
```

This is a fundamental incompatibility between RViz/OGRE and macOS. **Use the 2D visualization tool instead.**

### Configuring RViz Displays

1. **Set Fixed Frame:**

   - Left panel → "Global Options" → "Fixed Frame" → Change to `world`

2. **Add TF Display (Coordinate Frames):**

   - Click "Add" button (bottom left)
   - "By display type" tab → Select "TF" → Click "OK"

3. **Add Camera Display (Drone Camera View):**

   - Click "Add" → "By topic" tab
   - Expand `/drone/camera/image_raw` → Select "Camera" → Click "OK"

4. **Add Odometry Displays (Robot Paths):**

   - Click "Add" → "By topic"
   - Find `/drone/odom` → Select "Odometry" → Click "OK"
   - Repeat for `/rover/odom`

5. **Adjust Trail Length (Optional):**

   - Expand "Odometry" display in left panel
   - Find "Keep" property → Set to `100` (prevents long trails)

6. **Clear Odometry Trails:**
   - Uncheck the Odometry display
   - Check it again to re-enable with cleared history

---

## Controlling the Robots

### Drone Commands

**Move forward:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

**Fly in circles:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"
```

**Hover and spin in place:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 1.0}}" --once
```

**Spiral upward:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.5}, angular: {z: 0.5}}" --once
```

**Strafe sideways:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {z: 0.0}}" --once
```

**Climb vertically:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {z: 0.0}}" --once
```

**Stop the drone:**

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

### Rover Commands

**Move forward:**

```bash
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

**Drive in circles:**

```bash
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.3}}"
```

**Rotate in place:**

```bash
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"
```

**Stop the rover:**

```bash
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

### Command Velocity Explanation

The `Twist` message has 6 components:

**Linear velocity (m/s):**

- `x` - Forward/backward (drone and rover)
- `y` - Left/right (drone only, rover ignores this)
- `z` - Up/down (drone only)

**Angular velocity (rad/s):**

- `z` - Yaw rotation (turning left/right)

**Note:** By default, commands publish continuously (~10 Hz). Use `--once` flag to send a single command.

---

## Simulation Control

### Reset Simulation

Move robots back to initial positions and zero velocities:

```bash
ros2 service call /microsim/reset std_srvs/srv/Empty
```

**Initial positions:**

- Drone: x=0, y=0, z=1.0, yaw=0
- Rover: x=-5, y=0, theta=0

### Pause/Unpause Simulation

Toggle pause on/off:

```bash
ros2 service call /microsim/pause std_srvs/srv/Trigger
```

---

## Radio Communication

### Send Message from Drone to Rover

```bash
ros2 topic pub /radio/drone_tx std_msgs/msg/String '{data: "Hello from drone"}' --once
```

### Send Message from Rover to Drone

```bash
ros2 topic pub /radio/rover_tx std_msgs/msg/String '{data: "Hello from rover"}' --once
```

### Listen to Received Messages

**Drone receives:**

```bash
ros2 topic echo /radio/drone_rx
```

**Rover receives:**

```bash
ros2 topic echo /radio/rover_rx
```

**Note:** Messages have simulated latency (~50ms ± 10ms), jitter, and 1% packet loss.

---

## Monitoring Topics

### List All Topics

```bash
ros2 topic list
```

### Monitor Topic Rates

```bash
# Check odometry rate (~56 Hz)
ros2 topic hz /drone/odom

# Check GPS rate (~8 Hz)
ros2 topic hz /drone/gps

# Check camera rate (2 Hz)
ros2 topic hz /drone/camera/image_raw

# Check range sensor rate (~19 Hz)
ros2 topic hz /rover/range
```

### Echo Topic Data

```bash
# View drone odometry
ros2 topic echo /drone/odom

# View drone GPS
ros2 topic echo /drone/gps

# View rover range sensor
ros2 topic echo /rover/range

# View camera info
ros2 topic echo /drone/camera/camera_info
```

---

## Available Topics

### Odometry (60 Hz)

- `/drone/odom` - Drone position, orientation, velocities
- `/rover/odom` - Rover position, orientation, velocities

### GPS (10 Hz)

- `/drone/gps` - Drone GPS with noise (WGS-84)
- `/rover/gps` - Rover GPS with noise (WGS-84)

### Sensors

- `/rover/range` - Range sensor (20 Hz, ultrasound)
- `/drone/camera/image_raw` - RGB camera (2 Hz, 128x128 px)
- `/drone/camera/camera_info` - Camera intrinsics

### Radio Communication

- `/radio/drone_tx` - Drone transmit
- `/radio/drone_rx` - Drone receive
- `/radio/rover_tx` - Rover transmit
- `/radio/rover_rx` - Rover receive

### TF Frames

- `/tf` - Transform tree (60 Hz)
- Frames: `world`, `drone/base_link`, `rover/base_link`, `drone/camera_link`

### System

- `/clock` - Simulation time

---

## Running Tests

### Unit Tests (No ROS 2 Required)

```bash
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Run all unit tests
python3 -m pytest test/ -v

# Run specific test file
python3 -m pytest test/test_physics.py -v

# Quick test script
./run_tests.sh
```

### Integration Tests (Requires ROS 2)

**In Docker:**

```bash
cd /workspace
./docker-scripts/integration_test.sh
```

**On Mac:**

```bash
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim
./run_integration_tests.sh
```

---

## Building and Installing

### Build from Source (Mac)

```bash
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Activate ROS 2 environment
conda activate ros2_humble

# Install dependencies
pip install numpy pyyaml pytest
conda install ros-humble-ament-index-python -y

# Build
colcon build

# Source workspace
source install/setup.bash
```

### Editable Install (for Development)

```bash
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim

# Install in editable mode
pip install -e .
```

Changes to Python files are immediately reflected (no rebuild needed).

---

## Quick Start Workflow

**Terminal 1 - Simulator:**

```bash
conda activate ros2_humble
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim
source install/setup.bash
ros2 run microsim microsim_node
```

**Terminal 2 - RViz:**

```bash
conda activate ros2_humble
rviz2
# Configure displays as described above
```

**Terminal 3 - Control:**

```bash
conda activate ros2_humble

# Fly drone in circles
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"

# Drive rover forward
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Stop everything
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Reset to initial positions
ros2 service call /microsim/reset std_srvs/srv/Empty
```

---

## Useful Aliases

Add these to your `~/.zshrc` for quick access:

```bash
# MicroSim shortcuts
alias microsim='conda activate ros2_humble && cd ~/prog/Robots/robotic-ai-agents/simulator/microsim && source install/setup.bash && ros2 run microsim microsim_node'
alias viz='conda activate ros2_humble && rviz2'
alias stop_drone='ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once'
alias stop_rover='ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once'
alias reset_sim='ros2 service call /microsim/reset std_srvs/srv/Empty'
```

Then reload your shell:

```bash
source ~/.zshrc
```

---

## Troubleshooting

### "Package 'microsim' not found"

Make sure you've sourced the workspace:

```bash
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim
source install/setup.bash
```

### Robots don't stop when I cancel the command

Send an explicit stop command:

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### RViz doesn't show topics

1. Make sure the simulator is running
2. Check that `ROS_DOMAIN_ID` is the same (usually 0)
3. List topics to verify: `ros2 topic list`

### Docker and Mac can't see each other's topics

Stop Docker if running everything on Mac:

```bash
docker-compose down
```

### Scenario file not found

The simulator looks for `install/microsim/share/microsim/scenarios/default.yaml`.

If missing, specify explicitly:

```bash
ros2 run microsim microsim_node --ros-args -p scenario_file:=$(pwd)/scenarios/default.yaml
```

---

## Documentation

- **README.md** - Project overview and quick start
- **TEST_SUITE.md** - Test documentation
- **DOCKER.md** - Docker setup and usage
- **PRDv1.1.md** - Product requirements
- **IMPLEMENTATION_PLAN.md** - Development roadmap

---

## Need Help?

- Check logs in the simulator terminal
- Use `ros2 topic list` to see available topics
- Use `ros2 topic info <topic>` for topic details
- Use `ros2 service list` for available services
- Run unit tests to verify installation: `./run_tests.sh`
