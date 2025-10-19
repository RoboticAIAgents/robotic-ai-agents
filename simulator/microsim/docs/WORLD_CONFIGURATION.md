# World Configuration Guide

Complete guide to configuring the MicroSim world: adding structures, setting colors, adjusting parameters, and creating custom scenarios.

## Overview

The world is configured through YAML scenario files located in `scenarios/`. The default configuration is `scenarios/default.yaml`.

## Quick Start

### 1. Edit the Scenario File

```bash
# Edit the default scenario
nano scenarios/default.yaml

# Or create a custom scenario
cp scenarios/default.yaml scenarios/my_custom_world.yaml
```

### 2. Run with Custom Scenario

```bash
# Using pip install (development mode)
ros2 run microsim microsim_node --ros-args \
  -p scenario_file:=$(pwd)/scenarios/my_custom_world.yaml

# Using colcon build
ros2 run microsim microsim_node --ros-args \
  -p scenario_file:=$(pwd)/install/microsim/share/microsim/scenarios/my_custom_world.yaml
```

## Scenario File Structure

```yaml
world:
  # World dimensions and resolution

robots:
  drone:
    # Drone initial position and parameters
  rover:
    # Rover initial position and parameters

sensors:
  # GPS, camera, range sensor configuration

radio:
  # Inter-robot communication settings

features:
  # World structures (obstacles, hazards, targets)
```

## World Parameters

### Basic Configuration

```yaml
world:
  size: [100.0, 100.0]  # World dimensions in meters [x, y]
  resolution: 0.1       # Grid cell size in meters (smaller = more detail)
```

**Resolution guide:**
- `0.5` - Coarse grid (fast, blocky appearance)
- `0.1` - Fine grid (default, good balance)
- `0.05` - Very fine grid (detailed, slower)

The world is a 2D grid with semantic labeling. Smaller resolution = more cells = higher memory but finer detail in camera view.

## Robot Configuration

### Drone Setup

```yaml
robots:
  drone:
    initial_pose:
      x: 0.0      # X position in meters (east/west)
      y: 0.0      # Y position in meters (north/south)
      z: 15.0     # Altitude in meters
      yaw: 0.0    # Heading in radians (0 = east, π/2 = north)
    max_velocity: 3.0   # Maximum linear velocity (m/s)
    max_accel: 2.0      # Maximum acceleration (m/s²)
```

### Rover Setup

```yaml
robots:
  rover:
    initial_pose:
      x: -5.0     # X position in meters
      y: 0.0      # Y position in meters
      theta: 0.0  # Heading in radians
    max_velocity: 1.0   # Maximum linear velocity (m/s)
    max_omega: 1.57     # Maximum angular velocity (rad/s)
```

**Coordinate system:**
- X-axis: East (+) / West (-)
- Y-axis: North (+) / South (-)
- Z-axis: Up (+) / Down (-)
- Yaw/Theta: Counterclockwise from East (0° = East, 90° = North)

## World Features

Features are objects in the world: obstacles, hazards, and targets. They appear in:
- Camera view (with distinct colors)
- 2D/3D visualization (as markers)
- Semantic map (for collision detection)

### Feature Types

#### 1. Obstacles (Gray)

Physical structures that block movement.

```yaml
features:
  - type: obstacle
    position: [10.0, 10.0]  # [x, y] in world frame
    radius: 3.0             # Radius in meters (circular footprint)
    height: 5.0             # Height in meters (for visualization)
```

**Camera color:** Gray (RGB: 128, 128, 128)
**Semantic value:** 1
**Use cases:** Buildings, rocks, trees, walls

#### 2. Hazards (Red)

Dangerous areas (fire, water, rough terrain).

```yaml
features:
  - type: hazard
    position: [-10.0, 15.0]
    radius: 5.0
    height: 0.0  # Often flat (ground-level hazards)
```

**Camera color:** Red (RGB: 255, 0, 0)
**Semantic value:** 2
**Use cases:** Fire zones, water hazards, contaminated areas

#### 3. Targets (Green)

Goals or points of interest.

```yaml
features:
  - type: target
    position: [20.0, -10.0]
    radius: 2.0
    height: 0.0  # Often flat (landing pads, markers)
```

**Camera color:** Green (RGB: 0, 255, 0)
**Semantic value:** 3
**Use cases:** Landing pads, goal locations, items to collect

### Adding Multiple Features

Create varied environments with multiple features:

```yaml
features:
  # Large rock formation
  - type: obstacle
    position: [10.0, 10.0]
    radius: 3.0
    height: 5.0

  # Small boulder
  - type: obstacle
    position: [5.0, 3.0]
    radius: 0.5
    height: 2.0

  # Fire hazard area
  - type: hazard
    position: [-10.0, 15.0]
    radius: 5.0
    height: 0.0

  # Small puddle
  - type: hazard
    position: [2.0, -5.0]
    radius: 1.0
    height: 0.0

  # Landing pad
  - type: target
    position: [20.0, -10.0]
    radius: 2.0
    height: 0.0

  # Small marker
  - type: target
    position: [0.0, 5.0]
    radius: 0.5
    height: 0.0
```

### Feature Design Tips

**Mix sizes for realism:**
```yaml
# Large structures (3-5m radius)
- type: obstacle
  position: [10.0, 10.0]
  radius: 3.0
  height: 5.0

# Medium structures (1-2m radius)
- type: hazard
  position: [5.0, 5.0]
  radius: 1.5
  height: 0.0

# Small features (0.3-0.8m radius)
- type: target
  position: [2.0, 2.0]
  radius: 0.5
  height: 0.0
```

**Height guidelines:**
- `height: 0.0` - Ground-level (landing pads, puddles, flat hazards)
- `height: 1.0-3.0` - Low obstacles (rocks, small structures)
- `height: 4.0-10.0` - Tall obstacles (buildings, trees)

**Spacing:**
- Keep features at least 2× radius apart to avoid overlap
- Leave clear paths for navigation
- Place targets in accessible locations

## Sensor Configuration

### GPS Sensor

```yaml
sensors:
  gps:
    rate_hz: 10.0      # Publishing frequency
    noise_std: 0.1     # Gaussian noise std dev (meters)
```

**Noise guide:**
- `0.0` - Perfect GPS (unrealistic)
- `0.1` - Good consumer GPS
- `1.0` - Poor GPS / indoor
- `5.0` - Severely degraded

### Range Sensor (Rover)

```yaml
sensors:
  range:
    rate_hz: 20.0       # Publishing frequency
    max_range: 10.0     # Maximum detection range (meters)
    noise_std: 0.05     # Measurement noise (meters)
```

### Camera (Drone)

```yaml
sensors:
  camera:
    width: 256          # Image width in pixels
    height: 256         # Image height in pixels
    fov_deg: 60.0       # Field of view in degrees
    rate_hz: 10.0       # Publishing frequency (frames per second)
```

**Resolution vs Performance:**

| Resolution | FPS  | Use Case |
|-----------|------|----------|
| 128×128   | 50+  | Fast testing, low-detail needs |
| 256×256   | 25   | **Recommended** - good balance |
| 512×512   | 6    | High detail, slower |
| 1024×1024 | 1.5  | Very high detail, very slow |

**Rate guide:**
- `2.0 Hz` - Slow updates, minimal CPU
- `10.0 Hz` - Recommended for responsive visualization
- `30.0 Hz` - High rate, may impact simulation performance

## Radio Communication

```yaml
radio:
  max_range: 100.0         # Maximum communication range (meters)
  base_latency: 0.05       # Base latency in seconds (50ms)
  jitter: 0.01             # Latency variation (±10ms)
  packet_loss: 0.01        # Packet loss probability (1%)
```

**Realistic profiles:**

**Good conditions:**
```yaml
radio:
  max_range: 200.0
  base_latency: 0.02
  jitter: 0.005
  packet_loss: 0.001
```

**Poor conditions:**
```yaml
radio:
  max_range: 50.0
  base_latency: 0.1
  jitter: 0.05
  packet_loss: 0.1
```

## Camera Color Reference

The camera renders features with fixed semantic colors:

```python
# Camera rendering colors (in camera.py)
SEMANTIC_COLORS = {
    0: (100, 200, 100),  # Ground - Green
    1: (128, 128, 128),  # Obstacle - Gray
    2: (255, 0, 0),      # Hazard - Red
    3: (0, 255, 0),      # Target - Bright Green
    255: (135, 206, 235) # Sky - Light Blue
}
```

**Note:** Colors are hardcoded in `microsim/camera.py`. To change colors, edit the `SEMANTIC_COLORS` dictionary.

### Changing Feature Colors

To customize colors, edit `microsim/camera.py`:

```python
SEMANTIC_COLORS = {
    0: (100, 200, 100),   # Ground
    1: (255, 165, 0),     # Obstacle - Orange instead of gray
    2: (139, 0, 0),       # Hazard - Dark red
    3: (0, 128, 255),     # Target - Blue instead of green
    255: (135, 206, 235)  # Sky
}
```

Then rebuild:
```bash
colcon build --packages-select microsim
source install/setup.bash
```

## Example Scenarios

### Minimal Testing Environment

```yaml
world:
  size: [50.0, 50.0]
  resolution: 0.2

robots:
  drone:
    initial_pose: {x: 0.0, y: 0.0, z: 5.0, yaw: 0.0}
    max_velocity: 2.0
    max_accel: 1.0

sensors:
  gps:
    rate_hz: 10.0
    noise_std: 0.1
  camera:
    width: 128
    height: 128
    fov_deg: 60.0
    rate_hz: 10.0

features:
  - type: target
    position: [10.0, 0.0]
    radius: 1.0
    height: 0.0
```

### Complex Obstacle Course

```yaml
world:
  size: [100.0, 100.0]
  resolution: 0.1

features:
  # Obstacle forest
  - {type: obstacle, position: [10, 10], radius: 2.0, height: 5.0}
  - {type: obstacle, position: [15, 12], radius: 1.5, height: 4.0}
  - {type: obstacle, position: [12, 15], radius: 1.8, height: 6.0}
  - {type: obstacle, position: [8, 14], radius: 1.2, height: 3.5}

  # Hazard zone
  - {type: hazard, position: [20, 20], radius: 8.0, height: 0.0}
  - {type: hazard, position: [18, 25], radius: 4.0, height: 0.0}

  # Landing pads
  - {type: target, position: [30, 5], radius: 1.5, height: 0.0}
  - {type: target, position: [-10, 20], radius: 2.0, height: 0.0}
  - {type: target, position: [5, -15], radius: 1.0, height: 0.0}
```

### Search and Rescue

```yaml
features:
  # Building ruins (obstacles)
  - {type: obstacle, position: [15, 15], radius: 5.0, height: 8.0}
  - {type: obstacle, position: [20, 10], radius: 4.0, height: 6.0}
  - {type: obstacle, position: [12, 20], radius: 3.0, height: 5.0}

  # Fire/damage zones (hazards)
  - {type: hazard, position: [18, 18], radius: 6.0, height: 0.0}
  - {type: hazard, position: [10, 12], radius: 4.0, height: 0.0}

  # Survivor locations (targets)
  - {type: target, position: [25, 15], radius: 0.5, height: 0.0}
  - {type: target, position: [8, 22], radius: 0.5, height: 0.0}
  - {type: target, position: [14, 8], radius: 0.5, height: 0.0}
```

## Validation and Testing

### Check Scenario Syntax

```bash
# Python YAML validation
python3 -c "import yaml; yaml.safe_load(open('scenarios/my_custom_world.yaml'))"
```

### Visualize Before Running

Look at the 2D visualization to verify feature placement:

1. Start simulator with your scenario
2. Launch `python3 scripts/viz_2d.py`
3. Check top-left 2D map for feature positions
4. Verify no overlaps or unreachable areas

### Common Issues

**Features not visible in camera:**
- Check `height` - may be too low/high
- Verify `position` is within camera FOV
- Ensure drone altitude is appropriate

**Features overlapping:**
- Increase spacing between features
- Check that `radius` values don't cause overlap

**Poor performance:**
- Reduce world `resolution` (increase cell size)
- Decrease camera `width`/`height`
- Lower camera `rate_hz`
- Reduce number of features

## Advanced: Programmatic Generation

For procedurally generated worlds, create scenarios in code:

```python
import yaml
import numpy as np

# Generate random obstacles
def generate_scenario(num_obstacles=10, seed=42):
    np.random.seed(seed)

    features = []
    for i in range(num_obstacles):
        features.append({
            'type': 'obstacle',
            'position': [
                float(np.random.uniform(-40, 40)),
                float(np.random.uniform(-40, 40))
            ],
            'radius': float(np.random.uniform(0.5, 3.0)),
            'height': float(np.random.uniform(2.0, 6.0))
        })

    scenario = {
        'world': {'size': [100.0, 100.0], 'resolution': 0.1},
        'robots': {
            'drone': {
                'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 15.0, 'yaw': 0.0},
                'max_velocity': 3.0,
                'max_accel': 2.0
            }
        },
        'sensors': {
            'gps': {'rate_hz': 10.0, 'noise_std': 0.1},
            'camera': {'width': 256, 'height': 256, 'fov_deg': 60.0, 'rate_hz': 10.0}
        },
        'features': features
    }

    with open('scenarios/generated.yaml', 'w') as f:
        yaml.dump(scenario, f, default_flow_style=False)

generate_scenario()
```

## Summary

**To modify the world:**

1. **Edit** `scenarios/default.yaml` (or create new file)
2. **Change** world size, resolution, robot poses
3. **Add/remove** features with different types, positions, sizes
4. **Adjust** sensor rates, noise, camera resolution
5. **Run** simulator with `--ros-args -p scenario_file:=...`
6. **Verify** in visualization

**Key files:**
- `scenarios/default.yaml` - Main configuration
- `microsim/camera.py` - Feature colors (SEMANTIC_COLORS)
- `microsim/world.py` - World grid logic

Happy world building! 🌍
