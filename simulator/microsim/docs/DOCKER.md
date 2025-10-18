# MicroSim Docker Setup

Complete guide for running MicroSim in Docker with ROS 2 Humble.

## Quick Start (5 Minutes)

### 1. Prerequisites

- Docker Desktop installed and running
- Your workspace cloned locally
- (Optional) XQuartz for RViz visualization

### 2. Start the Container

```bash
cd simulator/microsim

# Start ROS 2 container in background
docker-compose up -d

# Enter the container
docker-compose exec ros2 bash
```

You're now inside a ROS 2 Humble environment with your code mounted at `/workspace`!

### 3. Build and Run

Inside the container:

```bash
# Build the package
./docker-scripts/build.sh

# Run tests
./docker-scripts/test.sh

# Start the simulator
./docker-scripts/run_node.sh
```

### 4. Test It Works

Open another terminal on your Mac:

```bash
# Enter same container
docker-compose exec ros2 bash

# Check topics
ros2 topic list

# Echo odometry
ros2 topic echo /drone/odom

# Send velocity command
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}"
```

---

## Detailed Usage

### Container Management

```bash
# Start container
docker-compose up -d

# Stop container
docker-compose stop

# Stop and remove
docker-compose down

# View logs
docker-compose logs -f

# Restart container
docker-compose restart
```

### Entering the Container

```bash
# Interactive shell
docker-compose exec ros2 bash

# Run a single command
docker-compose exec ros2 ros2 topic list

# Multiple terminals
docker-compose exec ros2 bash  # Terminal 1
docker-compose exec ros2 bash  # Terminal 2
```

### Building the Package

```bash
# Inside container:

# Method 1: Use the script
./docker-scripts/build.sh

# Method 2: Manual
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Note**: Files are symlinked, so changes on your Mac appear immediately in the container!

### Running Tests

```bash
# Inside container:

# Unit tests (no ROS needed)
python3 -m pytest test/ -v

# Or use the script
./docker-scripts/test.sh
```

### Running the Simulator

```bash
# Inside container:

# Method 1: Use the script
./docker-scripts/run_node.sh

# Method 2: Manual
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run microsim microsim_node
```

---

## ROS 2 Commands (Inside Container)

### Topics

```bash
# List all topics
ros2 topic list

# Show topic type
ros2 topic info /drone/odom

# Echo messages
ros2 topic echo /drone/gps
ros2 topic echo /rover/range

# Publish commands
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.1}}"

ros2 topic pub /rover/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"

# Radio communication
ros2 topic pub /radio/drone_tx std_msgs/String "data: 'Hello from drone'"
ros2 topic echo /radio/rover_rx
```

### Services

```bash
# List services
ros2 service list

# Call reset
ros2 service call /microsim/reset std_srvs/Empty

# Pause/unpause
ros2 service call /microsim/pause std_srvs/Trigger
```

### Nodes

```bash
# List nodes
ros2 node list

# Node info
ros2 node info /microsim
```

### TF (Transforms)

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transform
ros2 run tf2_ros tf2_echo world drone/base_link
```

---

## RViz Visualization (Optional)

### Setup X11 Forwarding on Mac

1. **Install XQuartz:**
   ```bash
   brew install --cask xquartz
   ```

2. **Configure XQuartz:**
   - Open XQuartz
   - Go to XQuartz → Preferences → Security
   - Check "Allow connections from network clients"
   - Restart XQuartz

3. **Allow Docker to connect:**
   ```bash
   xhost +localhost
   ```

4. **Set DISPLAY environment variable:**
   ```bash
   # In docker-compose.yml, DISPLAY is already set
   # Or manually:
   export DISPLAY=host.docker.internal:0
   ```

### Run RViz

Inside container:

```bash
# Start RViz
rviz2

# Or with a config (TODO: create microsim.rviz)
rviz2 -d /workspace/config/microsim.rviz
```

**Configure RViz:**
- Fixed Frame: `world`
- Add → RobotModel (for drone and rover URDFs - TODO)
- Add → TF
- Add → Odometry → Topic: `/drone/odom`
- Add → Image → Topic: `/drone/camera/image_raw`
- Add → Camera → Topic: `/drone/camera/camera_info`

---

## Development Workflow

### Recommended Workflow

1. **Edit code on your Mac** (use VS Code, PyCharm, whatever)
   ```bash
   # On Mac
   code simulator/microsim/microsim/
   ```

2. **Test in Docker** (instant - symlinked files)
   ```bash
   # Inside container
   python3 -m pytest test/
   ```

3. **Run simulator in Docker**
   ```bash
   # Inside container
   ./docker-scripts/run_node.sh
   ```

4. **Inspect with ROS tools**
   ```bash
   # Another terminal in same container
   ros2 topic list
   ros2 topic echo /drone/odom
   ```

### File Synchronization

Files are mounted as volumes, so:
- ✅ Changes on Mac appear instantly in container
- ✅ No rebuild needed for Python files (symlinked install)
- ✅ Only rebuild if you change package.xml, setup.py, CMakeLists.txt

### Hot Reload

Since we use `--symlink-install`, Python changes take effect immediately:
1. Edit `microsim_node.py` on Mac
2. Restart node in container (`Ctrl+C`, then re-run)
3. Changes are live!

---

## Troubleshooting

### Container Won't Start

```bash
# Check Docker is running
docker ps

# Check logs
docker-compose logs

# Rebuild from scratch
docker-compose down
docker-compose up -d
```

### RViz Won't Start

```bash
# Check X11 forwarding
echo $DISPLAY  # Should be host.docker.internal:0

# Check XQuartz is running
ps aux | grep Xquartz

# Re-allow X11
xhost +localhost

# Try with error output
rviz2 2>&1 | head -20
```

### Package Won't Build

```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths . --ignore-src -r -y
```

### Topics Not Showing

```bash
# Check node is running
ros2 node list

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be 0

# Check network mode
# docker-compose.yml uses network_mode: host
```

### Permission Issues

```bash
# If files created in container have wrong permissions:

# On Mac (outside container)
sudo chown -R $(whoami) .

# Or run container as your user (add to docker-compose.yml):
# user: "${UID}:${GID}"
```

---

## Advanced Usage

### Headless Mode (No GUI)

For testing only, use the smaller headless image:

```bash
docker-compose --profile headless up -d
docker-compose exec ros2-headless bash
```

### Custom Scenario

```bash
# Edit scenario on Mac
code scenarios/my_scenario.yaml

# Run with custom scenario (TODO: add arg support)
ros2 run microsim microsim_node --scenario /workspace/scenarios/my_scenario.yaml
```

### Multiple Instances

```bash
# Different ROS_DOMAIN_ID per instance
docker run -it --rm \
  -e ROS_DOMAIN_ID=1 \
  -v $(pwd):/workspace \
  osrf/ros:humble-desktop-full bash
```

### Build Custom Image

If you want to bake the package into an image:

```bash
# Build image
docker build -t microsim:latest .

# Run it
docker run -it --rm microsim:latest
```

---

## Performance Tips

1. **Use --symlink-install** - Already in scripts, no rebuild for Python changes

2. **Build in Release mode** - Already in build.sh
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. **Limit Docker resources** - Docker Desktop → Preferences → Resources
   - CPUs: 4+
   - Memory: 4GB+

4. **Use Docker volumes** - Already configured in docker-compose.yml

---

## Clean Up

```bash
# Stop container
docker-compose down

# Remove volumes (if any)
docker-compose down -v

# Remove image (if custom built)
docker rmi microsim:latest

# Prune all Docker
docker system prune -a
```

---

## Summary

**Key Commands:**
```bash
# Start
docker-compose up -d
docker-compose exec ros2 bash

# Build
./docker-scripts/build.sh

# Test
./docker-scripts/test.sh

# Run
./docker-scripts/run_node.sh

# Stop
docker-compose down
```

**Development Loop:**
1. Edit code on Mac
2. Test in container: `python3 -m pytest test/`
3. Run node: `./docker-scripts/run_node.sh`
4. Inspect: `ros2 topic list`, `ros2 topic echo ...`

**Need help?** Check the troubleshooting section or logs: `docker-compose logs -f`
