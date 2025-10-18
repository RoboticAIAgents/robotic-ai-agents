# MicroSim Quick Start Guide

Get up and running with the MicroSim simulator in 3 steps!

---

## Step 1: Start the Simulator

Open a terminal and run:

```bash
conda activate ros2_humble
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim
source install/setup.bash
ros2 run microsim microsim_node
```

You should see:
```
[INFO] [microsim]: MicroSim node starting...
[INFO] [microsim]: Loaded scenario: Default Scenario
[INFO] [microsim]: MicroSim node initialized successfully
```

---

## Step 2: Launch Visualization

**Open a new terminal** and run:

```bash
conda activate ros2_humble
cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim
source install/setup.bash
python3 scripts/viz_2d.py
```

A matplotlib window will open showing:
- **Left**: Top-down 2D map view
- **Right**: Drone's camera feed

> **Note:** We use matplotlib instead of RViz because RViz crashes on macOS due to OpenGL issues.

---

## Step 3: Control the Robots

**Open a third terminal** and try these commands:

### Make the drone fly in circles:

```bash
conda activate ros2_humble
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
  --rate 10
```

Watch the drone move in the visualization! Press `Ctrl+C` to stop.

### Make the rover drive forward:

```bash
ros2 topic pub /rover/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

### Reset the simulation:

```bash
ros2 service call /sim/reset std_srvs/Empty
```

This returns both robots to their starting positions.

---

## What You're Seeing

### In the 2D Map (Left Panel):
- **Red arrow**: Drone position and heading
- **Blue arrow**: Rover position and heading
- **Green circles**: Obstacles (solid objects)
- **Yellow circles**: Hazards (dangerous zones)
- **Cyan circles**: Targets (goal locations)
- **Faint trails**: Movement history

### In the Camera Feed (Right Panel):
- **Brown/tan**: Ground terrain
- **Gray**: Obstacles
- **Yellow/orange**: Hazards
- **Green/cyan**: Targets
- **Light blue**: Sky (when camera looks up)

The camera is mounted on the drone looking straight down (90° pitch).

---

## Common Commands

### View robot positions:
```bash
ros2 topic echo /drone/odom --once
ros2 topic echo /rover/odom --once
```

### See all available topics:
```bash
ros2 topic list
```

### Monitor camera update rate:
```bash
ros2 topic hz /drone/camera/image
```

### Pause the simulation:
```bash
ros2 service call /sim/pause std_srvs/Empty
```

### Resume the simulation:
```bash
ros2 service call /sim/resume std_srvs/Empty
```

---

## Troubleshooting

### "Package 'microsim' not found"
Make sure you sourced the workspace:
```bash
source install/setup.bash
```

### "No module named 'matplotlib'"
Install matplotlib in your conda environment:
```bash
conda activate ros2_humble
conda install matplotlib
```

### Visualization window is blank
Make sure the simulator is running first (Step 1) before launching visualization.

### Drone/rover not moving
- Make sure you're publishing at `--rate 10` (not just once)
- Check that the simulator is not paused
- Verify topics with: `ros2 topic list`

### Camera showing only sky
The drone starts at 40m altitude. Fly lower or descend:
```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: -5.0}}" --rate 10
```

---

## Next Steps

1. **Read the full documentation:**
   - [USAGE.md](USAGE.md) - Complete command reference
   - [VISUALIZATION_SETUP.md](VISUALIZATION_SETUP.md) - Detailed visualization guide
   - [README.md](README.md) - Architecture and design overview

2. **Experiment with control:**
   - Try different velocity combinations
   - Fly the drone over obstacles and watch the camera
   - Make the rover navigate around obstacles

3. **Modify the scenario:**
   - Edit [scenarios/default.yaml](scenarios/default.yaml)
   - Add more obstacles, hazards, or targets
   - Change robot starting positions

4. **Run the test suite:**
   ```bash
   cd /Users/rcampos/prog/Robots/robotic-ai-agents/simulator/microsim
   pytest tests/ -v
   ```

---

## Quick Reference Card

| Action | Command |
|--------|---------|
| **Start simulator** | `ros2 run microsim microsim_node` |
| **Launch viz** | `python3 scripts/viz_2d.py` |
| **Drone forward** | `linear.x = 2.0` |
| **Drone up** | `linear.z = 1.0` |
| **Drone turn right** | `angular.z = -0.5` |
| **Rover forward** | `linear.x = 1.0` |
| **Rover turn left** | `angular.z = 0.5` |
| **Reset sim** | `ros2 service call /sim/reset std_srvs/Empty` |
| **Pause sim** | `ros2 service call /sim/pause std_srvs/Empty` |

---

**Happy simulating!** 🚁🤖
