# Robotic AI Agents

AI-powered autonomous robot controller development using ROS 2 simulation.

## Project Overview

This repository contains:
- **MicroSim**: Deterministic ROS 2 simulator for drone and rover agents
- **AI Controllers**: Reference implementations for building intelligent autonomous systems
- **Documentation**: Comprehensive guides for simulation, world configuration, and AI integration

## Quick Start

### 1. Navigate to Simulator

```bash
cd simulator/microsim
```

### 2. Install Dependencies

**Windows Users:** See [docs/WINDOWS_SETUP.md](simulator/microsim/docs/WINDOWS_SETUP.md) for complete Windows setup guide (WSL2 + Docker recommended).

**Option A: Docker (Recommended - Linux/macOS/Windows)**
```bash
# See simulator/microsim/docs/DOCKER.md for complete setup
docker-compose up -d
docker-compose exec ros2 bash
```

**Option B: Native (macOS/Linux only)**
```bash
# Install ROS 2 Humble via robostack
conda create -n ros2_humble
conda activate ros2_humble
conda install -c robostack-staging ros-humble-desktop python=3.11
conda install matplotlib numpy pyyaml

# Install package
pip install -e . --no-deps
```

### 3. Run Simulation

```bash
# Terminal 1: Start simulator
ros2 run microsim microsim_node

# Terminal 2: Visualization
python3 scripts/viz_2d.py

# Terminal 3: Interactive control (optional)
python3 scripts/drone_controller.py
```

## Repository Structure

```
robotic-ai-agents/
├── simulator/                          # ROS 2 simulation environment
│   ├── microsim/                       # Main simulator package
│   │   ├── microsim/                   # Python package
│   │   │   ├── microsim_node.py        # Main ROS 2 node
│   │   │   ├── world.py                # 2D grid world
│   │   │   ├── physics.py              # Robot kinematics
│   │   │   ├── camera.py               # RGB pinhole camera
│   │   │   ├── sensors.py              # GPS and range sensors
│   │   │   └── radio.py                # Inter-robot communication
│   │   ├── scripts/                    # Utilities and controllers
│   │   │   ├── viz_2d.py               # Matplotlib visualization
│   │   │   ├── drone_controller.py     # Interactive CLI control
│   │   │   └── autonomous_drone_controller.py  # AI controller template
│   │   ├── scenarios/                  # World configurations
│   │   │   └── default.yaml            # Default world setup
│   │   ├── docs/                       # Documentation
│   │   │   ├── QUICKSTART.md           # Get started in 3 steps
│   │   │   ├── USAGE.md                # Complete reference
│   │   │   ├── WORLD_CONFIGURATION.md  # Scenario customization
│   │   │   ├── AI_CONTROLLER_GUIDE.md  # AI integration patterns
│   │   │   ├── TEST_SUITE.md           # Testing guide
│   │   │   └── DOCKER.md               # Docker setup
│   │   └── README.md                   # Simulator documentation
│   ├── PRDv1.1.md                      # Product requirements
│   ├── IMPLEMENTATION_PLAN.md          # Development roadmap
│   ├── MICROSIM_INTERFACE_CONTRACT.md  # ROS 2 API specification
│   └── ROS_2_RESOURCES.md              # Learning resources
└── README.md                           # This file
```

## Key Features

### MicroSim Simulator

✅ **Deterministic simulation** - Fixed 60 Hz timestep, seeded RNGs
✅ **Dual robots** - 6-DOF kinematic drone + differential-drive rover
✅ **Rich sensors** - GPS (noisy), RGB camera (256×256), range sensor
✅ **Radio link** - Inter-robot communication with realistic network effects
✅ **Semantic world** - Obstacles, hazards, targets with color-coded rendering
✅ **Real-time visualization** - matplotlib-based 2D/3D views + camera feed
✅ **macOS support** - Native visualization without OpenGL issues

### AI Controller Development

✅ **Reference implementation** - Clean template for AI integration
✅ **Sensor integration** - Odometry, GPS, camera callbacks
✅ **Control loop structure** - Perceive → Think → Act pattern
✅ **AI integration points** - Clear hooks for LLM, RL, vision models
✅ **Interactive testing** - CLI-based manual control for validation
✅ **Comprehensive docs** - 5 AI integration patterns with examples

## Documentation

| Document | Description |
|----------|-------------|
| [**simulator/microsim/README.md**](simulator/microsim/README.md) | Main simulator documentation |
| [**docs/QUICKSTART.md**](simulator/microsim/docs/QUICKSTART.md) | Get started in 3 steps |
| [**docs/USAGE.md**](simulator/microsim/docs/USAGE.md) | Complete command reference |
| [**docs/WORLD_CONFIGURATION.md**](simulator/microsim/docs/WORLD_CONFIGURATION.md) | Configure worlds, sensors, features |
| [**docs/AI_CONTROLLER_GUIDE.md**](simulator/microsim/docs/AI_CONTROLLER_GUIDE.md) | Build intelligent controllers |
| [**docs/WINDOWS_SETUP.md**](simulator/microsim/docs/WINDOWS_SETUP.md) | **Windows installation guide** |
| [**docs/DOCKER.md**](simulator/microsim/docs/DOCKER.md) | Docker setup guide |
| [**docs/TEST_SUITE.md**](simulator/microsim/docs/TEST_SUITE.md) | Testing and validation |

## Development

### Running Tests

```bash
cd simulator/microsim
pytest test/
```

### Creating Custom Worlds

Edit `scenarios/default.yaml` or create new scenarios. See [WORLD_CONFIGURATION.md](simulator/microsim/docs/WORLD_CONFIGURATION.md) for details.

```yaml
features:
  - type: obstacle
    position: [10.0, 10.0]
    radius: 3.0
    height: 5.0
```

### Building AI Controllers

Start with the reference implementation:

```bash
cp scripts/autonomous_drone_controller.py scripts/my_ai_controller.py
# Edit simple_decision_logic() to add your AI
python3 scripts/my_ai_controller.py
```

See [AI_CONTROLLER_GUIDE.md](simulator/microsim/docs/AI_CONTROLLER_GUIDE.md) for integration patterns.

## ROS 2 Interface

### Published Topics

**Drone:**
- `/drone/cmd_vel` - Velocity commands (Twist)
- `/drone/odom` - Ground truth odometry (Odometry)
- `/drone/gps` - Noisy GPS (NavSatFix)
- `/drone/camera/image_raw` - RGB camera feed (Image, 256×256)

**Rover:**
- `/rover/cmd_vel` - Velocity commands (Twist)
- `/rover/odom` - Ground truth odometry (Odometry)
- `/rover/gps` - Noisy GPS (NavSatFix)
- `/rover/range` - Forward range sensor (Range)

**Radio:**
- `/radio/drone_tx`, `/radio/drone_rx` - Drone communication
- `/radio/rover_tx`, `/radio/rover_rx` - Rover communication

### Services

- `/sim/reset` - Reset simulation to initial state
- `/sim/pause` - Pause/unpause simulation

## Current Status

**✅ Completed:**
- Core simulation (60 Hz deterministic)
- Drone and rover physics
- All sensors (GPS, camera, range)
- Radio communication model
- 2D/3D visualization
- Interactive CLI controller
- Autonomous controller template
- Comprehensive documentation
- Test suite (72 unit tests passing)

**🎯 Ready for AI Development:**

The simulator is feature-complete and ready for developing intelligent autonomous controllers. See the AI Controller Guide to get started.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests: `pytest test/`
5. Submit a pull request

## License

MIT

## Contact

For questions or issues, please open a GitHub issue.
