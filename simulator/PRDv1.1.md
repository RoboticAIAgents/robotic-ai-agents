# MicroSim — Product Requirements Document (PRD)

**Version:** v1.1
**Status:** Ground truth
**Date:** 2025-10-16
**Owner:** Dev 1 (Simulation)
**Stakeholders:** Dev 2 (Robot Intelligence), PM, QA

> **Purpose:** Define the authoritative requirements for **MicroSim**, a minimal, deterministic, real-time simulator that exposes a **ROS 2** interface for two robots—a **Drone** and a **Rover**—to be controlled by AI agents. This PRD is the single source of truth for implementation and testing.

---

## Changelog

```
feat(camera): switch drone camera to RGB pinhole model; add CameraInfo, intrinsics, latency/noise params, and palette spec
```

- **Breaking by design (documented):** Drone camera encoding changes **mono8 → rgb8**; new publisher **/drone/camera/camera_info**.
- All other surfaces remain backward-compatible.

---

## 1. Goals & Non-Goals

### 1.1 Primary Goals

- Provide a **deterministic** (seeded), **real-time** simulation with a fixed step scheduler.
- Expose a **clean ROS 2 contract** (topics, services, params) so Dev 2 can develop controllers/policies as if talking to real robots.
- Include **basic sensors**: GPS, down-looking **RGB pinhole** camera (drone), forward range sensor (rover).
- Include a simple **radio link** model for inter-robot messaging with latency/drop.
- Offer a **lightweight 3D view** via RViz2 (and optional web viewer later) without embedding a heavy game engine.

### 1.2 Non-Goals (Deliberately Out of Scope)

- High-fidelity aerodynamics, wheel suspension, friction modeling.
- Multi-robot fleets beyond the two specified agents.
- Complex terrain physics (soft bodies, fluids) or photoreal rendering.
- Full mapping/localization stacks (SLAM); that belongs to Dev 2.

---

## 2. Personas & Key Use Cases

- **Dev 2 (Robot Intelligence):** Writes ROS 2 nodes that consume odometry/sensors and publish velocity commands; uses the radio to coordinate drone↔rover tasks; records rosbag2 for offline debugging.
- **QA/CI:** Runs headless deterministic tests, validates topic schemas, rates, and QoS; checks reproducible bags given a seed.
- **PM/Demo:** Launches the sim + RViz2 with one command to visualize missions.

**Use Cases**

1. Hover & Patrol: Drone executes a square path using `/drone/cmd_vel`, samples down camera at 2 Hz.
2. Obstacle Avoidance: Rover steers using `/rover/range/front` only.
3. Team Task: Drone scans, radios a target location; rover navigates there.

---

## 3. System Overview

MicroSim is a single ROS 2 node `microsim` that:

- Runs a **fixed-step scheduler** (default 60 Hz) for physics & systems.
- Publishes/subscribes **standard ROS 2 messages** only.
- Loads **scenario YAML** to configure terrain, obstacles, noise, and initial poses.
- Provides **services** for reset, pause, stepping, and seeding.
- Supplies **URDFs** (or supports Markers) for RViz2 visualization.

```mermaid
flowchart LR
  subgraph ROS2 Graph
    A[Dev 2 Nodes\n(controllers, planners)] <-- topics/services --> B[MicroSim Node]
    B -->|/tf, /odom, sensors| C[RViz2]
    A <--> |/radio/* via QoS| B
  end
```

---

## 4. Functional Requirements

### 4.1 Robots

#### 4.1.1 Drone (Kinematic)

- **State:** position `(x,y,z)` in ENU `map`, yaw `ψ`; body-frame velocities.
- **Command Input:** `geometry_msgs/Twist` on `/drone/cmd_vel` (body-frame):
  `linear.x` forward (+X), `linear.y` left (+Y), `linear.z` up (+Z), `angular.z` yaw rate (rad/s). Other fields ignored.
- **Limits (defaults):** `|vx|≤5 m/s`, `|vy|≤5`, `|vz|≤3`, `|yaw_rate|≤1.5 rad/s`.
- **Dynamics:** First-order command lag (τ=0.15 s) → kinematic integrate at 60 Hz.
- **Ground Constraint:** `z ≥ ground(x,y) + clearance` (no penetration).
- **Wind:** Optional constant + Gaussian gust in world frame.

#### 4.1.2 Rover (Diff-Drive Kinematic)

- **State:** planar pose `(x,y,θ)` in ENU `map`.
- **Command Input:** `geometry_msgs/Twist` on `/rover/cmd_vel`: use `linear.x` and `angular.z` only.
- **Limits (defaults):** `|v|≤2 m/s`, `|ω|≤1.2 rad/s`, first-order lag τ=0.10 s.
- **Collision:** Stops at occupied cells; no slide/penetration.

### 4.2 Sensors

#### 4.2.1 GPS (Both Robots)

- **Topic:** `.../gps/fix` (`sensor_msgs/NavSatFix`) at 5 Hz.
- **Frame:** `*_gps_link` with `map` as reference origin `(lat0,lon0,alt0)`.
- **Noise:** Gaussian σ=2 m (ENU), optional slow bias random walk; covariance populated.

#### 4.2.2 Down Camera (Drone)

- **Topics:**

  - `/drone/camera/image_raw` — `sensor_msgs/Image`, **encoding=rgb8**, **2 Hz** (configurable).
  - `/drone/camera/camera_info` — `sensor_msgs/CameraInfo`, published with matching header stamps.

- **Projection:** **Pinhole** (perspective). The camera is rigidly mounted to `drone/camera_link` (TF).
- **Intrinsics:** Provided via `CameraInfo` (`K`, `P`, `width`, `height`). Distortion is zero by default (configurable).
- **Resolution (default):** `128×128` (configurable).
- **Content:** Low-fidelity **RGB palette** rendering of ground features using simple primitives (triangles, circles, rectangles) with fixed colors. Nearest occluder wins (height-based z-test).
- **Latency & noise:** Publish latency (mean + jitter) and lightweight visual noise (optional) are applied to approximate real pipelines.
- **Determinism:** With fixed `seed` and identical poses, images are bit-stable (pre-noise, or with noise seeded).

##### 4.2.2.1 Camera RGB Palette (Authoritative)

The simulator maps semantic classes to fixed RGB colors:

| Class ID | Name         | Shape (typical) | RGB Hex |
| -------: | ------------ | --------------- | ------- |
|        0 | Free ground  | fill/background | #C8C8C8 |
|        1 | Obstacle     | circle          | #3CB371 |
|        2 | Hazard       | triangle        | #E74C3C |
|        3 | Target / POI | rectangle       | #3498DB |
|        4 | Water        | fill/patch      | #1ABC9C |

> This mapping is **stable** across runs and scenarios. Colors are applied **after** z-testing (nearest surface by height).

##### 4.2.2.2 Rendering Model (Pinhole)

At each camera tick (2 Hz by default):

1. Query `drone/camera_link` pose in `map` at the image timestamp.
2. For each pixel, cast a ray through the pinhole and intersect with the terrain plane; test against 3D primitives (boxes/cylinders/prisms/billboards) and keep the **nearest** hit (height-aware occlusion).
3. Write the pixel color from the **palette** of the hit class.
4. Apply optional visual noise (tiny Gaussian blur σ≈0.5 px, optional color jitter) and **enqueue latency** (mean±jitter) before publishing.
5. Publish `Image(rgb8)` and a matching `CameraInfo` with valid intrinsics and `frame_id=drone/camera_link`.

#### 4.2.3 Forward Range (Rover)

- **Topic:** `/rover/range/front` (`sensor_msgs/Range`) at 10 Hz.
- **Model:** Single raycast in `base_link` forward direction; returns first hit distance.
- **Noise:** Gaussian σ=0.05 m; min/max range set by params; small dropout rate optional.

### 4.3 Radio Link (Drone ↔ Rover)

- **TX Topics:** `/radio/drone_tx`, `/radio/rover_tx` (`std_msgs/ByteMultiArray`).
- **RX Topics:** `/radio/drone_rx`, `/radio/rover_rx` (`std_msgs/ByteMultiArray`).
- **QoS:** **best_effort**, `depth=1` for TX, `depth=5` for RX.
- **Model:** Per-direction FIFO with latency (mean+jitter), packet drop probability, optional reordering (default off). Ordering is preserved when not dropped.
- **Metrics:** `/radio/metrics` (`diagnostic_msgs/DiagnosticArray`) exposes measured latency, drop %, queue depth.

### 4.4 World & Terrain

- **Domain:** 2.5D grid (square cells; default 1 m resolution).
- **Fields per cell:** elevation `h`, occupied boolean, surface type (for camera shading).
- **Obstacles:** Axis-aligned boxes or cylinders; placed via scenario YAML.
- **Primitives for camera rendering:** Each obstacle/feature carries `{class_id, height, geometry}` where geometry is a circle, rectangle, or triangle in the ground plane (with optional prism height). Rendering picks the nearest intersection (height-aware occlusion). Class IDs map to the palette in §4.2.2.1.
- **Bounds:** Configurable map size (e.g., 200×200 cells).

### 4.5 Time & Scheduling

- **Fixed physics tick:** 60 Hz (parameter `physics_hz`).
- **Sensor publishers:** odom 30 Hz, GPS 5 Hz, range 10 Hz, camera 2 Hz (all configurable).
- **Sim time:** `/clock` published; users may set `use_sim_time:=true` in their nodes.
- **Pause/Step:** When paused, `/clock` advances only on explicit step calls.

### 4.6 Visualization

- **Primary:** RViz2 using **RobotModel** displays from URDFs + TF; add Image/Camera, TF, Odometry, Range, MarkerArray.
- **RViz note:** RViz **displays** the topics; **rendering** occurs in the simulator (RViz is not the renderer).
- **URDFs:** Minimal URDFs for drone & rover provided; meshes optional (primitives acceptable).
- **Markers:** Optional world overlays (obstacles, paths, waypoints) via `visualization_msgs/MarkerArray`.

### 4.7 Recording & Replay

- **rosbag2:** Users can record any topics; playback should reproduce identical messages given same seed + command sequence.

---

## 5. ROS 2 API Contract

### 5.1 Topics (Publishers from MicroSim)

**Drone**

- `/drone/odom` — `nav_msgs/Odometry` @30 Hz, reliable, keep_last=10.
- `/drone/gps/fix` — `sensor_msgs/NavSatFix` @5 Hz, reliable.
- `/drone/camera/image_raw` — `sensor_msgs/Image` (**rgb8**) @2 Hz, reliable, keep_last=2.
- `/drone/camera/camera_info` — `sensor_msgs/CameraInfo` @2 Hz, reliable, keep_last=2.
- `/tf`, `/tf_static` — `tf2_msgs/TFMessage` @60 Hz.

**Rover**

- `/rover/odom` — `nav_msgs/Odometry` @30 Hz, reliable.
- `/rover/gps/fix` — `sensor_msgs/NavSatFix` @5 Hz, reliable.
- `/rover/range/front` — `sensor_msgs/Range` @10 Hz, reliable.
- `/tf`, `/tf_static` — `tf2_msgs/TFMessage` @60 Hz.

**World/Debug (optional)**

- `/world/occupancy` — `nav_msgs/OccupancyGrid` @1 Hz.
- `/world/markers` — `visualization_msgs/MarkerArray` @1–5 Hz.

**Radio Metrics**

- `/radio/metrics` — `diagnostic_msgs/DiagnosticArray` @1 Hz.

### 5.2 Topics (Subscribers consumed by MicroSim)

- `/drone/cmd_vel` — `geometry_msgs/Twist` ≤60 Hz, reliable, keep_last=10.
- `/rover/cmd_vel` — `geometry_msgs/Twist` ≤60 Hz, reliable, keep_last=10.
- `/radio/drone_tx`, `/radio/rover_tx` — `std_msgs/ByteMultiArray`, best_effort, depth=1.

### 5.3 Services (provided by MicroSim)

- `/sim/reset` — `std_srvs/Empty` → reset world to current scenario & seed.
- `/sim/set_seed` — `example_interfaces/SetInt32` → sets RNG seed for next reset.
- `/sim/pause` — `example_interfaces/SetBool` → pause or unpause.
- `/sim/step` — `example_interfaces/SetInt32` → advance N physics ticks when paused.
- `/sim/info` — `std_msgs/String` (JSON blob) via latched publisher or service (implementation choice) with version, scene, rates, seed, noise.

### 5.4 Parameters (on node `microsim`)

```yaml
microsim:
  # Timing
  physics_hz: 60
  odom_hz: 30
  camera_hz: 2 # DEPRECATED: use microsim.drone.camera.hz
  gps_hz: 5
  range_hz: 10

  # Drone
  drone:
    max_vx: 5.0
    max_vy: 5.0
    max_vz: 3.0
    max_yaw_rate: 1.5
    cmd_time_constant: 0.15
    wind_world_xyz: [0.0, 0.0, 0.0]

    # Camera (authoritative)
    camera:
      enabled: true
      hz: 2
      width: 128
      height: 128
      encoding: "rgb8"
      projection: "pinhole" # authoritative
      # Intrinsics: choose either (fx, fy, cx, cy) OR derive from FOV
      intrinsics:
        fx: 120.0
        fy: 120.0
        cx: 64.0
        cy: 64.0
      distortion:
        model: "plumb_bob" # "none" or "plumb_bob"
        D: [0.0, 0.0, 0.0, 0.0, 0.0] # k1, k2, t1, t2, k3
      # Visual noise (very light; all optional)
      noise:
        blur_sigma_px: 0.5 # 0 disables blur
        color_jitter: 0.0 # 0..1 multiplicative jitter
      # Pipeline timing
      latency_ms_mean: 80
      latency_ms_jitter: 30
      drop_probability: 0.01

  # Rover
  rover:
    max_v: 2.0
    max_omega: 1.2
    cmd_time_constant: 0.10

  # Sensors noise (1σ)
  sensors:
    gps_sigma_m: 2.0
    gps_bias_drift_m_per_s: 0.005
    range_sigma_m: 0.05
    # camera_noise_u8: 2            # REMOVED (replaced by camera.noise fields)

  # Radio model
  radio:
    latency_ms_mean: 80
    latency_ms_jitter: 30
    drop_probability: 0.03
    reordering: false

  # World
  world:
    cell_size_m: 1.0
    size_xy_cells: [200, 200]
    origin_wgs84: [9.935, -84.09, 1150.0] # lat, lon, alt (example)

  # Scenario
  scenario: "default"
  seed: 12345
  use_sim_time: true
```

---

## 6. Data & Math Models

### 6.1 Coordinate Frames & Conventions

- **Global:** `map` (ENU; +X East, +Y North, +Z Up).
- **Local:** `odom` (continuous, drift-ful) → `base_link`.
- **Sensors:** `camera_link`, `gps_link`, `range_link` fixed to `base_link` via URDF.

### 6.2 Kinematics (Integration at Δt = 1/physics_hz)

**Drone** (body-frame command → world position)

```
# Clamp cmd to limits, apply first-order filter: u̇ = (u_cmd - u)/τ
v_body = [vx, vy, vz]
Rz = rot_z(ψ)
ṗ_world = Rz * v_body
ψ̇ = yaw_rate
p(t+Δt) = p(t) + ṗ_world*Δt
ψ(t+Δt) = ψ(t) + ψ̇*Δt
z = max(z, ground(x,y) + clearance)
```

**Rover** (diff-drive abstraction)

```
ẋ = v*cosθ
ẏ = v*sinθ
θ̇ = ω
x(t+Δt) = x + ẋΔt; y(t+Δt) = y + ẏΔt; θ(t+Δt) = θ + θ̇Δt
If next cell occupied → stop movement this tick
```

The camera is a fixed joint `base_link → camera_link` (URDF). The sim publishes `map → odom → base_link`, and `robot_state_publisher` provides `base_link → camera_link`.

### 6.3 Sensors

- **GPS:** ENU→WGS-84 via local tangent plane at `origin_wgs84`. Add Gaussian noise and low-rate bias random walk; populate covariance.
- **Camera (Drone):** **Pinhole** projection with a down-looking camera mounted at `drone/camera_link`. The renderer casts per-pixel rays, performs **height-aware occlusion**, and paints **fixed-palette RGB** values. It publishes `sensor_msgs/Image (rgb8)` and `sensor_msgs/CameraInfo` with consistent intrinsics. Optional latency and light visual noise are applied. Deterministic with seed (noise seeded).
- **Range:** Cast a ray from `range_link` forward; find first occupied cell; add Gaussian noise; honor min/max range.

### 6.4 Radio

- For each TX direction, maintain queue with **timestamp + payload**. On update:

  1. Draw latency = `N(latency_mean, jitter)` clipped ≥0.
  2. With prob `drop_probability`, drop packet.
  3. Otherwise deliver to RX at `t_send + latency`.

- Deterministic order: stable RNG seeded by `seed`.

---

## 7. Files, Packages, and Launching

### 7.1 Repo Layout

```
microsim/
  sim/
    __init__.py
    world.py            # grid, obstacles, terrain sampling
    timekeeper.py       # fixed-step + /clock
    components.py       # Pose, Twist, Noise, RadioLink data classes
    systems/
      physics.py        # drone & rover kinematics, collisions
      sensors.py        # gps, camera, range
      radio.py          # latency/drop queues
      tf.py             # frame tree publisher
      ros2_bridge.py    # pubs/subs, QoS, params
  ursdf/
    drone.urdf.xacro
    rover.urdf.xacro
  scenarios/
    default.yaml
    flat.yaml
    obstacles_arena.yaml
  rviz/
    microsim_view.rviz
  launch/
    microsim.launch.py
  tests/
    unit/
    integration/
    acceptance/
  docs/
    PRD.md
```

### 7.2 Launch

- **Command:** `ros2 launch microsim microsim.launch.py scenario:=default seed:=12345`
- **RViz:** `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix microsim)/rviz/microsim_view.rviz`

---

## 8. Scenario YAML Schema (Authoritative)

```yaml
name: "default"
seed: 12345
map:
  cell_size_m: 1.0
  size_xy_cells: [200, 200]
  origin_wgs84: [9.935, -84.09, 1150.0]
  elevation:
    file: "scenarios/heightmaps/flat.npy"
  surface_types:
    default: 0
obstacles:
  - type: box
    center: [50, 60, 0]
    size: [4, 4, 2] # x,y,z meters
  - type: cylinder
    center: [80, 40, 0]
    radius: 2
    height: 1.5
robots:
  drone:
    start:
      map_pose: { x: 20, y: 20, z: 10, yaw: 0.0 }
    camera:
      hz: 2
      resolution: { width: 128, height: 128 }
      projection: pinhole
      intrinsics: { fx: 120.0, fy: 120.0, cx: 64.0, cy: 64.0 }
      distortion: { model: plumb_bob, D: [0, 0, 0, 0, 0] }
      noise: { blur_sigma_px: 0.5, color_jitter: 0.0 }
      latency_ms: { mean: 80, jitter: 30, drop_probability: 0.01 }
    gps:
      hz: 5
      sigma_m: 2.0
  rover:
    start:
      map_pose: { x: 10, y: 10, yaw: 1.57 }
    range:
      hz: 10
      min_m: 0.2
      max_m: 10.0
      sigma_m: 0.05
radio:
  latency_ms_mean: 80
  latency_ms_jitter: 30
  drop_probability: 0.03
wind:
  world_xyz: [0.2, 0.0, 0.0]
```

---

## 9. Visualization Assets

### 9.1 URDF (Minimal Example — Drone)

```xml
<robot name="drone">
  <link name="base_link">
    <visual>
      <geometry><cylinder length="0.05" radius="0.15"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
  </link>
  <link name="camera_link">
    <visual>
      <geometry><box size="0.05 0.05 0.03"/></geometry>
      <material name="blue"><color rgba="0.2 0.4 0.8 1.0"/></material>
    </visual>
  </link>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.10 0.0 -0.05" rpy="0 0 0"/>
  </joint>
  <link name="gps_link">
    <visual>
      <geometry><sphere radius="0.02"/></geometry>
      <material name="yellow"><color rgba="1.0 0.9 0.2 1.0"/></material>
    </visual>
  </link>
  <joint name="base_to_gps" type="fixed">
    <parent link="base_link"/>
    <child link="gps_link"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </joint>
</robot>
```

### 9.2 RViz2 Configuration

- Fixed Frame = `map`
- Displays: `TF`, two `RobotModel`s (namespaced), `Odometry` for each robot, **Image or Camera** (`/drone/camera/image_raw`), `MarkerArray` (`/world/markers`).
- Add a **Camera** or **Image** display for `/drone/camera/image_raw`; verify `frame_id=drone/camera_link` and overlay with TF for sanity.
- Note: RViz **displays** the topics; **rendering** occurs in the simulator.

---

## 10. Non-Functional Requirements

- **Performance:** Maintain ≥0.95 real-time factor with default map (200×200), two robots, and default sensor rates on a mid-range laptop CPU.
- **Determinism:** With fixed `seed` + identical command stream (topic values + timestamps), all published messages (payloads, not wall-clock headers) are **bit-identical** across runs. For the camera, pre-noise images are bit-stable; with noise enabled, images are identical when the noise RNG is seeded.
- **Stability:** Subscribers publishing malformed messages (NaN/Inf) must be ignored for that tick; no crashes.
- **Portability:** Linux primary; macOS/Windows best-effort (Docker allowed). Python 3.11 baseline.
- **Packaging:** ROS 2 ament package with `setup.py`, `package.xml`, `COLCON_IGNORE` as needed; `rosdep` installable.
- **Telemetry:** Optional metrics logger (CSV) for RTF, tick latency, queues.

---

## 11. Testing & Acceptance Criteria

### 11.1 Unit Tests

- Kinematics integration (drone, rover) vs closed-form solutions for constant commands.
- Raycast correctness on synthetic obstacle patterns.
- ENU↔WGS-84 conversion round-trip within tolerance.

### 11.2 Integration Tests

- Topic presence & QoS: all required topics/services exist with specified QoS.
- Sensor rates within ±5% of target (over 30 s window).
- TF tree valid & continuous; frames match URDF links.
- **Camera rate:** `/drone/camera/image_raw` publishes at `2.0 ± 5%` Hz over a 60 s window.
- **CameraInfo consistency:** `CameraInfo.K`/`P` match configuration; width/height match `Image`.
- **Latency distribution:** Inter-arrival minus render time matches configured `{mean, jitter}` within ±10 ms.

### 11.3 Determinism Test

- Given `seed=S`, recorded rosbag2 of a scripted mission must hash-match across two runs (excluding ROS stamp headers if necessary).
- **Camera determinism:** With fixed seed and identical drone poses, **pre-noise** renders are byte-identical; with noise enabled, renders are identical if the noise RNG is seeded.

### 11.4 Acceptance Scenarios

1. **Hover Demo:** Drone holds position for 60 s; camera publishes 2 Hz; GPS covariance sane; no ground penetration.
2. **Obstacle Stop:** Rover drives into wall; halts within one cell of first occupied cell; range sensor stable.
3. **Radio Ping:** Drone sends N=100 messages; observed drop rate within ±1% of parameter; mean latency within ±10 ms.
4. **Camera Occlusion:** Place a 2 m-tall obstacle overlapping a ground patch; verify that pixels projecting to the obstacle area are the obstacle’s class color, not the ground color.
5. **CameraInfo Validity:** For a known grid of world points under the camera, back-project image pixels using `CameraInfo` and confirm projected rays intersect expected map coordinates within 1 pixel.

---

## 12. Extensibility (Plugin Surface)

- **Sensors:** Python class interface `update(dt)` + `publish()`; registered via entry points or a plugin folder.
- **World Hooks:** Allow custom obstacle generators (e.g., random mazes) via scenario YAML `plugin:` entries.
- **Bridges:** Optional Three.js web viewer via `rosbridge_suite` is allowed as a separate package.

---

## 13. Risks & Mitigations

- **Risk:** ROS 2 QoS incompatibility causing silent drops → **Mitigation:** ship a validated RViz profile and example subscriber templates using matching QoS.
- **Risk:** Non-determinism from thread/async timing → **Mitigation:** single-threaded fixed-step core + seeded RNG; isolate I/O.

---

## 14. Versioning & Change Policy

- Semantic versioning: **MAJOR.MINOR.PATCH**.
- This PRD is **v1.1**; camera encoding change (mono8→rgb8) and new `camera_info` topic are the only intentional breaking changes from v1.0. A deprecation alias is not provided (agents must expect `rgb8`).

---

## 15. Open Questions

- Do we need a minimal IMU (for future use) even if controllers don’t require it now?
- Should we include a simple waypoint service for scripted demos?
- Do we need a CSV logger built-in or rely on rosbag2 only?

---

## 16. Glossary

- **ENU:** East-North-Up coordinate convention.
- **URDF:** Unified Robot Description Format—XML robot model for ROS.
- **QoS:** Quality of Service policies for ROS 2 middleware.
- **RTF:** Real-Time Factor (sim time / wall time).
- **CameraInfo:** ROS 2 message describing camera intrinsics (K) and projection P.
- **Pinhole camera:** Perspective camera defined by intrinsics (fx, fy, cx, cy); one ray per pixel.

---

## Appendix A — Example Rover URDF (Minimal)

```xml
<robot name="rover">
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.3 0.15"/></geometry>
      <material name="red"><color rgba="0.8 0.2 0.2 1.0"/></material>
    </visual>
  </link>
  <link name="range_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.06"/></geometry>
      <material name="green"><color rgba="0.2 0.8 0.2 1.0"/></material>
    </visual>
  </link>
  <joint name="base_to_range" type="fixed">
    <parent link="base_link"/>
    <child link="range_link"/>
    <origin xyz="0.25 0.0 0.08" rpy="0 0 0"/>
  </joint>
  <link name="gps_link"/>
  <joint name="base_to_gps" type="fixed">
    <parent link="base_link"/>
    <child link="gps_link"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

## Appendix B — Example Launch (Sketch)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    scenario = LaunchConfiguration('scenario', default='default')
    seed = LaunchConfiguration('seed', default='12345')

    microsim = Node(
        package='microsim', executable='microsim', name='microsim',
        parameters=[{'scenario': scenario, 'seed': seed, 'use_sim_time': True}],
        output='screen')

    drone_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='drone',
        parameters=[{'robot_description': Command(['xacro ', PathJoinSubstitution([...,'ursdf','drone.urdf.xacro'])])}])

    rover_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='rover',
        parameters=[{'robot_description': Command(['xacro ', PathJoinSubstitution([...,'ursdf','rover.urdf.xacro'])])}])

    return LaunchDescription([microsim, drone_rsp, rover_rsp])
```

## Appendix C — RViz Display Checklist

- Fixed Frame = `map`
- `TF` enabled
- Two `RobotModel` displays (drone/rover)
- `Odometry` displays for `/drone/odom`, `/rover/odom`
- **Image/Camera** for `/drone/camera/image_raw`
- `MarkerArray` for `/world/markers`
- Save as `rviz/microsim_view.rviz`
