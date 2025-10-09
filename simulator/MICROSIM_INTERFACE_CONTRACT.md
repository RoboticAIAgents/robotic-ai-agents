# MicroSim – Interface Contract (v0.1)

> This document defines **the external contract** between the MicroSim simulator (Dev 1) and the Robot Intelligence stack (Dev 2). Think of MicroSim as a **black box** that speaks ROS 2. Everything below is guaranteed by the simulator; Dev 2 can implement, test, and CI their agents without knowing simulator internals.

---

## 1) Scope & Roles

- **Robots modeled:** one **Drone** (quadcopter-like, kinematic) and one **Rover** (differential drive, kinematic).
- **Standards:** ROS 2 (Humble/Iron), REP-103 (SI units, ENU), REP-105 (TF tree).
- **Real-time:** The sim runs near **1.0 real-time factor** with fixed-step physics. Simulated time is published on **`/clock`**.
- **You implement:** planners, controllers, policies, tool-using agents, logging, and any higher-level autonomy.
- **You don’t implement:** physics, sensor synthesis, radio/link simulation, scene resets, seeding—those are provided.

---

## 2) Coordinate Frames & Units

- **World:** `map` (ENU: +X East, +Y North, +Z Up).
- **Local odometry:** `odom` (continuous, drift-ful).
- **Robot bases:** `drone/base_link`, `rover/base_link`.
- **Sensors:** `drone/camera_link`, `rover/range_link`, `drone/gps_link`, `rover/gps_link`.
- **Units:** meters, radians, seconds, m/s, rad/s.
- **Yaw:** +Z (counter-clockwise) in ENU.

The simulator publishes a complete **TF tree** (`/tf`, `/tf_static`) at **60 Hz**.

---

## 3) Time & Rates

- **Sim time:** published on **`/clock`**. Set `use_sim_time:=true` in your nodes if you want to sync to sim time.
- **Fixed physics tick:** 60 Hz.
- **Nominal sensor rates (configurable):**

  - Odom: 30 Hz
  - Drone down-camera: **2 Hz** (mono8)
  - GPS: 5 Hz
  - Rover front range: 10 Hz
  - Radio link: event-driven with simulated latency/drop

---

## 4) Topic API (per robot)

### 4.1 Drone

| Direction | Topic                     | Type                    |     Rate | QoS                        | Notes                                                                                                                                                         |
| --------- | ------------------------- | ----------------------- | -------: | -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ⬅︎ cmd    | `/drone/cmd_vel`          | `geometry_msgs/Twist`   |   ≤60 Hz | **reliable**, keep_last=10 | **Body-frame** command: `linear.x` forward (+X), `linear.y` left (+Y), `linear.z` up (+Z), `angular.z` yaw rate. Others ignored. Saturation/clamping applied. |
| ⮕ state   | `/drone/odom`             | `nav_msgs/Odometry`     |    30 Hz | reliable, keep_last=10     | Frame: `odom` → `base_link`. Covariances reflect process + noise.                                                                                             |
| ⮕ sensor  | `/drone/gps/fix`          | `sensor_msgs/NavSatFix` |     5 Hz | reliable, keep_last=5      | WGS-84. Includes position covariance; ~1–3 m 1σ by default.                                                                                                   |
| ⮕ sensor  | `/drone/camera/image_raw` | `sensor_msgs/Image`     | **2 Hz** | reliable, keep_last=2      | `encoding=mono8`, `frame_id=drone/camera_link`, e.g., 64×64. Low-fidelity top-down raster of terrain.                                                         |
| ⮕ tf      | `/tf`, `/tf_static`       | `tf2_msgs/TFMessage`    |    60 Hz | reliable                   | `map→odom→base_link→camera_link/gps_link`.                                                                                                                    |

### 4.2 Rover

| Direction | Topic                | Type                    |   Rate | QoS                    | Notes                                                                                         |
| --------- | -------------------- | ----------------------- | -----: | ---------------------- | --------------------------------------------------------------------------------------------- |
| ⬅︎ cmd    | `/rover/cmd_vel`     | `geometry_msgs/Twist`   | ≤60 Hz | reliable, keep_last=10 | Differential-drive abstraction: use `linear.x` (m/s) and `angular.z` (rad/s). Others ignored. |
| ⮕ state   | `/rover/odom`        | `nav_msgs/Odometry`     |  30 Hz | reliable, keep_last=10 | Frame: `odom` → `base_link`.                                                                  |
| ⮕ sensor  | `/rover/range/front` | `sensor_msgs/Range`     |  10 Hz | reliable, keep_last=5  | `frame_id=rover/range_link`. Single forward raycast; min/max set via params.                  |
| ⮕ sensor  | `/rover/gps/fix`     | `sensor_msgs/NavSatFix` |   5 Hz | reliable, keep_last=5  | WGS-84 with covariance.                                                                       |
| ⮕ tf      | `/tf`, `/tf_static`  | `tf2_msgs/TFMessage`    |  60 Hz | reliable               | `map→odom→base_link→range_link/gps_link`.                                                     |

> **Contract behavior:** Commands are **body-frame** unless stated. The sim applies first-order lag and clamps velocities/turn rates; actual motion is observable via `/odom`.

---

## 5) Inter-Robot “Radio” Link

The “radio” is a lossy half-duplex abstraction with latency, jitter, and drop probability.

| Direction               | Topic             | Type                              | QoS                          | Notes                                                                                                                            |
| ----------------------- | ----------------- | --------------------------------- | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| Drone TX ➜ Rover RX     | `/radio/drone_tx` | `std_msgs/ByteMultiArray`         | **best_effort**, keep_last=1 | Dev 2 publishes bytes here to send **from Drone to Rover**. Sim applies latency/jitter/drop, then delivers to `/radio/rover_rx`. |
| Rover RX (from Drone)   | `/radio/rover_rx` | `std_msgs/ByteMultiArray`         | best_effort, keep_last=5     | What the rover **receives**.                                                                                                     |
| Rover TX ➜ Drone RX     | `/radio/rover_tx` | `std_msgs/ByteMultiArray`         | best_effort, keep_last=1     | Rover→Drone direction. Delivered to `/radio/drone_rx`.                                                                           |
| Drone RX (from Rover)   | `/radio/drone_rx` | `std_msgs/ByteMultiArray`         | best_effort, keep_last=5     | What the drone **receives**.                                                                                                     |
| Link metrics (optional) | `/radio/metrics`  | `diagnostic_msgs/DiagnosticArray` | reliable, keep_last=1        | Per-direction latency (ms), drop %, RSSI model (if enabled).                                                                     |

**Contract behavior:** Payload is opaque bytes; you own serialization (e.g., CBOR/JSON/protobuf). The sim guarantees **ordering per direction**, but may **drop** packets (configurable). No retries are injected by the sim.

---

## 6) World/Map Introspection (Optional, for debugging)

| Topic              | Type                             |   Rate | Notes                                                                                                                                                |
| ------------------ | -------------------------------- | -----: | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/world/occupancy` | `nav_msgs/OccupancyGrid`         |   1 Hz | 2.5D occupancy projection; frame `map`. For visualization/debugging only—your agents shouldn’t rely on it for decisions unless the scenario says so. |
| `/world/markers`   | `visualization_msgs/MarkerArray` | 1–5 Hz | RViz-friendly markers for robots/obstacles.                                                                                                          |

---

## 7) Control & Lifecycle

All control endpoints are **standard ROS 2 services or topics** so you can script experiments and CI jobs. No custom message packages are required.

| Purpose                                   | Name              | Type                          | Semantics                                                                              |
| ----------------------------------------- | ----------------- | ----------------------------- | -------------------------------------------------------------------------------------- |
| Reset world to scenario defaults          | `/sim/reset`      | `std_srvs/Empty`              | Resets poses, RNG, and queues using the **current seed**.                              |
| Set RNG seed (takes effect on next reset) | `/sim/set_seed`   | `example_interfaces/SetInt32` | Set `data=seed`. Returns `success`/`message`.                                          |
| Pause/unpause                             | `/sim/pause`      | `example_interfaces/SetBool`  | `data=true` pauses; `false` resumes.                                                   |
| Step N ticks (when paused)                | `/sim/step`       | `example_interfaces/SetInt32` | Steps **N ≥ 1** physics ticks synchronously.                                           |
| Teleport robot pose (command topic)       | `/drone/set_pose` | `geometry_msgs/PoseStamped`   | Immediate teleport; sim will reflect in TF/odom next tick. Similar: `/rover/set_pose`. |
| Get sim info                              | `/sim/info`       | `std_msgs/String`             | JSON blob: version, rates, scene name, seed, noise knobs.                              |

**Start/stop:** The sim exposes a single node `microsim` and sets `use_sim_time=true` internally (you may set it in your nodes as needed).

---

## 8) Parameters (namespaces & defaults)

All are ROS 2 parameters on the `microsim` node and can be set via launch/YAML.

```yaml
microsim:
  # Timing
  physics_hz: 60
  odom_hz: 30
  camera_hz: 2
  gps_hz: 5
  range_hz: 10

  # Drone command limits
  drone:
    max_vx: 5.0 # m/s
    max_vy: 5.0
    max_vz: 3.0
    max_yaw_rate: 1.5 # rad/s
    cmd_time_constant: 0.15 # s, 1st-order lag

  # Rover command limits
  rover:
    max_v: 2.0 # m/s
    max_omega: 1.2 # rad/s
    cmd_time_constant: 0.10

  # Sensor noise (1σ)
  sensors:
    gps_sigma_m: 2.0
    range_sigma_m: 0.05
    camera_noise_u8: 2 # gray levels

  # Radio model
  radio:
    latency_ms_mean: 80
    latency_ms_jitter: 30
    drop_probability: 0.03
    reordering: false

  # Scenario & seeding
  scenario: "default"
  seed: 12345
```

---

## 9) Guarantees & Edge Cases

- **Determinism:** With a fixed `seed` and identical command sequence/timestamps, the sim is **bit-stable** at message level (excluding wall-time stamps).
- **Saturation & safety:** Commands beyond limits are **clamped**; **NaN/Inf** commands are ignored for that tick.
- **Collision model:** Rover stops at occupied cells; drone clamps altitude to ground clearance.
- **QoS stability:** Changing QoS on your subscribers won’t crash the sim; incompatible QoS will follow ROS rules (you may not receive data).
- **Clock:** When paused, `/clock` advances only on `/sim/step`.

---

## 10) Minimal “Hello Sim” Checklist (Dev 2)

1. **Time:** `ros2 param set /your_node use_sim_time true` (or in launch).
2. **Subscribe:** `/drone/odom`, `/rover/odom`, `/drone/gps/fix`, `/rover/range/front`, `/tf`.
3. **Publish:** `/drone/cmd_vel`, `/rover/cmd_vel` at ≤60 Hz.
4. **Radio:** Publish `std_msgs/ByteMultiArray` to `/radio/drone_tx`, subscribe to `/radio/drone_rx` (and the rover counterparts).
5. **Reset loop:** Call `/sim/reset` to start fresh; optionally set `/sim/set_seed` first.
6. **Record:** `ros2 bag record /clock /tf /drone/* /rover/* /radio/*` for reproducible runs.

---

## 11) Example Message Semantics

- **Drone `cmd_vel` (body-frame):**

  - `linear.x`: forward (+X), `linear.y`: left (+Y), `linear.z`: up (+Z) in m/s.
  - `angular.z`: yaw rate in rad/s. Other angular components ignored.

- **Rover `cmd_vel`:**

  - Use only `linear.x` and `angular.z`. The sim treats it as an ideal diff-drive.

---

## 12) Versioning & Change Policy

- **This contract is v0.1.** Additions will be **backward-compatible** (new topics/params).
- Any breaking changes (topic rename, type change) will bump **major** and ship a compatibility shim for ≥1 release.

---

## 13) What’s Out of Scope (by design)

- No multi-robot beyond the two named agents (for now).
- No high-fidelity aerodynamics/suspension.
- No custom message dependencies required for basic use.
- No built-in mapping/localization libraries; that’s Dev 2’s domain.

---

## 14) Quick RViz Setup (optional)

- Fixed frame: `map`.
- Displays: TF, Odometry (both), Image (`/drone/camera/image_raw`), Range (marker), and MarkerArray (`/world/markers` if enabled).

---

### TL;DR for Dev 2

- **Publish** `Twist` to `/drone/cmd_vel` and `/rover/cmd_vel`.
- **Listen** to odom/GPS/range/camera + `/tf`.
- **Use** `/radio/*` ByteMultiArray for inter-robot comms.
- **Control** runs via `/sim/reset`, `/sim/pause`, `/sim/step`, and `/sim/set_seed`.
- **Assume** ENU, SI units, fixed rates, and deterministic runs with a seed.
