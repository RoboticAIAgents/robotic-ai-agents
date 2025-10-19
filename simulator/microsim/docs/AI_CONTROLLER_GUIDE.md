# AI Drone Controller Development Guide

This guide shows how to use the reference autonomous controller as a base for building intelligent AI-based drone controllers.

## Overview

The `autonomous_drone_controller.py` provides a clean template demonstrating:

- ROS 2 integration (publishers, subscribers, services)
- Sensor data processing (odometry, GPS, camera)
- Control loop structure (perceive → think → act)
- Velocity command generation
- Mission/waypoint management

## Quick Start

### 1. Run the Reference Controller

```bash
# Terminal 1: Start simulator
ros2 run microsim microsim_node

# Terminal 2: Start visualization
python3 scripts/viz_2d.py

# Terminal 3: Run autonomous controller
python3 scripts/autonomous_drone_controller.py
```

The drone will autonomously follow 6 waypoints in a predefined pattern.

### 2. Understand the Structure

The controller has three main phases in its control loop:

```python
def control_loop(self):
    # STEP 1: PERCEPTION - Process sensor data
    current_pos = self.position
    target_pos = self.waypoints[self.current_waypoint_index]

    # STEP 2: DECISION - AI goes here!
    velocity_command = self.simple_decision_logic(
        current_pos, target_pos
    )

    # STEP 3: ACTION - Execute decision
    self.publish_velocity(*velocity_command)
```

### 3. Add Your AI Agent

Replace `simple_decision_logic()` with your AI implementation.

## AI Integration Patterns

### Pattern 1: LLM-Based High-Level Planning

Use a language model for high-level decision making:

```python
def ai_decision_logic(self, current_pos, obstacles, goal):
    """Use LLM for high-level planning."""

    # Build context for LLM
    situation = {
        'position': current_pos.tolist(),
        'obstacles': obstacles,
        'goal': goal.tolist(),
        'camera_sees': self.describe_camera_view()
    }

    # Query LLM
    prompt = f"""
    You are controlling a drone. Current situation:
    Position: {situation['position']}
    Goal: {situation['goal']}
    Obstacles detected: {situation['obstacles']}

    Decide the next action. Return velocity as [vx, vy, vz, omega].
    """

    response = self.llm_agent.query(prompt)
    velocity = self.parse_llm_response(response)

    return velocity
```

### Pattern 2: Reinforcement Learning Policy

Use a trained RL policy:

```python
def rl_decision_logic(self, observation):
    """Use RL policy for control."""

    # Build observation vector
    obs = np.concatenate([
        self.position,
        self.velocity,
        [self.yaw],
        self.get_target_direction(),
        self.get_obstacle_features()
    ])

    # Query policy
    action = self.rl_policy.get_action(obs)

    # Map action to velocity
    vx, vy, vz, omega = self.action_to_velocity(action)

    return (vx, vy, vz, omega)
```

### Pattern 3: Vision-Based Navigation

Use camera for perception-driven control:

```python
def vision_based_decision(self):
    """Use camera for obstacle avoidance and navigation."""

    if self.latest_camera_image is None:
        return (0, 0, 0, 0)

    # Process camera image
    obstacles = self.detect_obstacles(self.latest_camera_image)
    free_space = self.segment_free_space(self.latest_camera_image)

    # Reactive control based on vision
    if obstacles:
        # Avoid obstacles
        avoidance_vector = self.compute_avoidance(obstacles)
        goal_vector = self.compute_goal_direction()

        # Blend behaviors
        velocity = 0.3 * avoidance_vector + 0.7 * goal_vector
    else:
        # Direct path to goal
        velocity = self.compute_goal_direction()

    return velocity
```

### Pattern 4: Behavior Trees

Hierarchical decision-making:

```python
def behavior_tree_decision(self):
    """Use behavior tree for structured decision making."""

    # Root node: Sequence
    # 1. Check battery
    if self.battery_low():
        return self.land_safely()

    # 2. Check for obstacles
    if self.obstacle_detected():
        return self.avoid_obstacle()

    # 3. Execute mission
    if self.mission_active:
        return self.follow_waypoint()

    # 4. Default: hover
    return (0, 0, 0, 0)
```

### Pattern 5: Hybrid AI

Combine multiple approaches:

```python
def hybrid_ai_decision(self):
    """Combine classical planning with learned components."""

    # High-level planning with LLM
    current_goal = self.llm_planner.get_next_subgoal(
        self.position,
        self.mission_state
    )

    # Mid-level path planning (classical)
    path = self.rrt_planner.plan(
        self.position,
        current_goal,
        self.obstacle_map
    )

    # Low-level control with RL
    next_waypoint = path[0] if path else current_goal
    observation = self.build_observation(next_waypoint)
    action = self.rl_controller.get_action(observation)

    return self.action_to_velocity(action)
```

## Sensor Data Access

### Odometry (Ground Truth)

```python
def odometry_callback(self, msg):
    self.position = np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ])
    self.velocity = np.array([...])
    self.yaw = self.extract_yaw(msg.pose.pose.orientation)
```

### GPS (Noisy Sensor)

```python
def gps_callback(self, msg):
    self.gps_position = np.array([
        msg.latitude,
        msg.longitude,
        msg.altitude
    ])
    # Use for realistic sensor fusion
```

### Camera (RGB Images)

```python
def camera_callback(self, msg):
    self.latest_camera_image = msg

    # Convert to OpenCV/NumPy for processing
    from cv_bridge import CvBridge
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    # Now process with vision AI
    detections = self.yolo_model(cv_image)
```

## Example AI Workflows

### Workflow 1: Obstacle Avoidance with Vision

```python
class VisionBasedController(AutonomousDroneController):
    def __init__(self):
        super().__init__()
        self.obstacle_detector = YOLODetector()

    def camera_callback(self, msg):
        super().camera_callback(msg)

        # Detect obstacles in camera view
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.detected_obstacles = self.obstacle_detector.detect(cv_image)

    def simple_decision_logic(self, current_pos, target_pos, target_yaw):
        # Check for obstacles
        if self.detected_obstacles:
            # Reactive avoidance
            return self.avoid_obstacles()
        else:
            # Continue to goal
            return super().simple_decision_logic(
                current_pos, target_pos, target_yaw
            )
```

### Workflow 2: LLM-Guided Exploration

```python
class LLMExplorationController(AutonomousDroneController):
    def __init__(self):
        super().__init__()
        self.llm = OpenAI_API()
        self.explored_areas = set()

    def control_loop(self):
        # Describe current situation to LLM
        situation = self.describe_environment()

        # Ask LLM for next exploration target
        prompt = f"""
        You are exploring an environment.
        Current position: {self.position}
        Areas explored: {self.explored_areas}

        Choose next exploration target to maximize coverage.
        Return as [x, y, z] coordinates.
        """

        response = self.llm.query(prompt)
        next_target = self.parse_coordinates(response)

        # Navigate to LLM-chosen target
        velocity = self.compute_velocity_to(next_target)
        self.publish_velocity(*velocity)
```

### Workflow 3: Multi-Agent Coordination

```python
class CollaborativeDroneController(AutonomousDroneController):
    def __init__(self, drone_id, team_size):
        super().__init__()
        self.drone_id = drone_id
        self.team_size = team_size

        # Subscribe to teammate positions
        self.team_positions = {}
        for i in range(team_size):
            if i != drone_id:
                self.create_subscription(
                    Odometry, f'/drone{i}/odom',
                    lambda msg, id=i: self.teammate_callback(id, msg), 10
                )

    def simple_decision_logic(self, current_pos, target_pos, target_yaw):
        # Coordinate with team using AI
        team_state = self.get_team_state()
        my_task = self.task_allocator.assign_task(
            self.drone_id, team_state
        )

        # Execute assigned task
        return self.execute_task(my_task)
```

## Tips for AI Development

### 1. Start Simple
- Get the basic controller working first
- Test with simple waypoints
- Add AI incrementally

### 2. Use Simulation for Training
- Perfect for RL training (reset service available)
- No safety concerns for aggressive exploration
- Deterministic for reproducible experiments

### 3. Leverage Available Data
- Odometry: High-rate ground truth (60 Hz)
- GPS: Realistic noisy measurements (10 Hz)
- Camera: Semantic-labeled ground (10 Hz)

### 4. Debug Visualization
- Use RViz/matplotlib visualization to see AI decisions
- Add custom markers for planned paths
- Log AI reasoning for post-analysis

### 5. Performance Considerations
- Control loop runs at 10 Hz by default
- AI inference must complete in <100ms
- Consider async processing for heavy models

## Advanced Topics

### Custom Waypoint Generation

```python
# Instead of hardcoded waypoints, generate from AI
def generate_ai_waypoints(self):
    # Use LLM to plan exploration route
    mission_description = "Survey the area and find landing zones"
    waypoints = self.llm_planner.plan_mission(
        mission_description,
        self.get_world_info()
    )
    self.set_waypoints(waypoints)
```

### Learning from Demonstration

```python
# Record human demonstrations
def record_demonstration(self):
    while recording:
        state = self.get_state()
        action = self.get_human_action()  # from CLI controller
        self.dataset.append((state, action))

    # Train imitation learning model
    self.train_policy(self.dataset)
```

### Online Learning

```python
# Update policy during flight
def control_loop(self):
    state = self.get_state()
    action = self.policy.get_action(state)

    self.publish_velocity(*action)

    # Get reward from environment
    reward = self.compute_reward()

    # Online policy update
    self.policy.update(state, action, reward)
```

## Testing Your AI Controller

```bash
# Test 1: Basic waypoint following
python3 scripts/autonomous_drone_controller.py

# Test 2: With your AI modifications
python3 scripts/my_ai_controller.py

# Test 3: Multiple runs for consistency
for i in {1..10}; do
    python3 scripts/my_ai_controller.py --seed $i
done
```

## Next Steps

1. **Copy the reference controller:**
   ```bash
   cp autonomous_drone_controller.py my_ai_controller.py
   ```

2. **Add your AI imports:**
   ```python
   from your_ai_package import LLMAgent, RLPolicy, VisionModel
   ```

3. **Replace decision logic:**
   - Modify `simple_decision_logic()`
   - Or override `control_loop()` entirely

4. **Test incrementally:**
   - Start with one AI component
   - Validate behavior in simulation
   - Add complexity gradually

5. **Iterate and improve:**
   - Log performance metrics
   - Visualize AI decisions
   - Refine based on results

Happy AI drone controller development! 🚁🤖
