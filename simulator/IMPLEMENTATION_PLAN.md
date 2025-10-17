I can do it for you right now—and then you can hand the result to a coding agent to generate scaffolding/tests from it. Best approach: let me produce a clear, opinionated **implementation plan** aligned to your PRD (including the RGB pinhole camera changes), and use a coding agent for code generation on a per-module basis with tight prompts and acceptance criteria pulled from this plan.

Below is a detailed, copy-paste-ready plan.

---

# MicroSim — Implementation Plan (from PRD v1.1)

## 0) Strategy & Working Agreements

- **Source of truth:** PRD v1.1 (RGB pinhole camera).
- **Language/stack:** Python 3.11, ROS 2 (Humble/Iron), rclpy, numpy. No game engines.
- **Style/quality:** black, isort, ruff, mypy (loose), pytest + `launch_testing`.
- **Determinism:** single-threaded fixed-step loop; separate seeded RNG per subsystem.
- **Build/run:** colcon/ament Python package `microsim`.
- **Visualization:** RViz2 (RobotModel, Image/Camera, TF, MarkerArray). RViz is a viewer only.

---

## 1) Work Breakdown Structure (WBS)

### 1. Core Infrastructure

1. **Repo scaffold & packaging**

   - `package.xml`, `setup.py`, `pyproject.toml`, `colcon.pkg`.
   - Lint/format configs; pre-commit hooks.

2. **Timekeeper**

   - `FixedStepScheduler`: runs at `physics_hz`, publishes `/clock`.
   - Wall-time sync (soft) to maintain ≥0.95 RTF; skip render, never skip physics.

3. **Parameter server**

   - Load ROS params + scenario YAML; overlay precedence: CLI > params YAML > scenario.

4. **Scenario loader**

   - Parse §8 YAML: map, obstacles, robots starts, radio/camera config.

### 2. World & Physics

5. **World grid**

   - 2.5D height grid (float32), occupancy (bool), class_id (uint8).
   - Query helpers: height(x,y), occupied(x,y), class_id(x,y).

6. **Collision & ray utilities**

   - Grid DDA traversal, AABB/cylinder intersection helpers.

7. **Kinematics**

   - Drone (body → world), first-order lag, ground clamp.
   - Rover diff-drive, lag, cell collision stop.

8. **TF/Frames**

   - Publish `map→odom→base_link`; URDF fixed joints for sensors via RSP.

### 3. Sensors

9. **GPS**

   - ENU→WGS-84 (tangent plane), Gaussian noise + bias drift, covariance.

10. **Range (rover)**

    - Forward raycast, min/max, Gaussian noise/dropout.

11. **Camera (drone, RGB pinhole)**

    - Pinhole rays, height-aware occlusion (z-test), fixed palette (§4.2.2.1).
    - `Image(rgb8)` + matching `CameraInfo` (K, P, width, height).
    - Optional post-fx: tiny Gaussian blur (σ≈0.5 px), color jitter (0..1).
    - Publish latency queue (mean±jitter) and optional drop.

### 4. Comms & ROS 2 Bridge

12. **Cmd subscribers**

    - `/drone/cmd_vel`, `/rover/cmd_vel` (QoS reliable, depth 10).

13. **State & sensor publishers**

    - Odom, GPS, Range, Camera, TF, `/world/markers` optional, `/world/occupancy` optional.

14. **Radio link**

    - `/radio/*_tx` (best_effort) → queued → `/radio/*_rx`, metrics.

15. **Sim control services**

    - `/sim/reset`, `/sim/pause`, `/sim/step`, `/sim/set_seed`, `/sim/info`.

### 5. Visualization & Assets

16. **URDFs**

    - Minimal drone & rover URDFs; links for `camera_link`, `gps_link`, `range_link`.

17. **RViz profile**

    - Fixed frame `map`; RobotModels, TF, Odometry, Image/Camera, MarkerArray.

### 6. Determinism, Testing, CI

18. **RNG discipline**

    - `rng_core`, `rng_camera`, `rng_radio`, `rng_sensors`.

19. **Unit tests**

    - Kinematics, DDA ray, ENU↔WGS84, camera rays hit tests, palette assignment.

20. **Integration tests**

    - Topic/QoS presence, rates ±5%, CameraInfo validity, latency distribution, occlusion.

21. **Determinism tests**

    - Pre-noise image bit-stability; seeded-noise stability; rosbag reproducibility.

22. **CI**

    - Lint, type-check, unit tests, selected launch tests headless.

---

## 2) Milestones & Gating Criteria

### Sprint 0 — Project bootstrap (0.5–1 wk)

- ✅ Repo scaffold, black/ruff/mypy/pytest/pre-commit.
- ✅ Minimal `microsim` node that publishes `/clock` and exits cleanly.
- Gate: CI green on lint + one trivial unit test.

### Sprint 1 — Core loop, world, TF, odom (1 wk)

- Implement FixedStepScheduler @60 Hz with `/clock`.
- World grid + scenario loader (flat map).
- Kinematics for drone/rover; publish `/drone/odom`, `/rover/odom`.
- TF: `map→odom→base_link`.
- Gate: RViz shows both RobotModels moving with `/cmd_vel`; odom rate 30 Hz ±5%.

### Sprint 2 — GPS, Range, Radio, YAML polish (1 wk)

- GPS w/ covariance & noise; Range raycast; radio queues + metrics.
- Scenario YAML: starts, obstacles, radio knobs.
- Gate: Integration tests pass for GPS (5 Hz), Range (10 Hz), radio latency histogram within ±10 ms.

### Sprint 3 — Camera (RGB pinhole) + CameraInfo (1–1.5 wk)

- Pinhole renderer (RGB palette, occlusion).
- CameraInfo intrinsics/distortion; latency/blur/jitter.
- RViz Image/Camera display; determinism hooks.
- Gate: Camera 2 Hz ±5%, occlusion test passes, CameraInfo back-projection error ≤1 px, determinism tests pass.

### Sprint 4 — Determinism harness, docs, acceptance (0.5–1 wk)

- Rosbag reproducibility script; acceptance scenarios 1–5 automated.
- RViz profile shipped; docs (README + usage).
- Gate: All PRD acceptance criteria pass; CI stable.

---

## 3) Module-level Design & APIs

### 3.1 `sim/timekeeper.py`

```python
class FixedStepScheduler:
    def __init__(self, hz: float, use_sim_time: bool, clock_pub: rclpy.Publisher):
        self.dt = 1.0 / hz
    def run(self, update_cb: Callable[[float], None]):
        # while rclpy.ok(): tick update_cb(self.dt); publish /clock
```

### 3.2 `sim/world.py`

```python
@dataclass
class WorldConfig:
    cell_size: float
    size_xy: tuple[int,int]
    elevation: np.ndarray  # H[y,x]
    occupancy: np.ndarray  # bool[y,x]
    class_id: np.ndarray   # uint8[y,x]

class World:
    def height(self, x: float, y: float) -> float: ...
    def occupied(self, x: float, y: float) -> bool: ...
    def class_at(self, x: float, y: float) -> int: ...
```

### 3.3 `systems/physics.py`

```python
@dataclass
class DroneState: x: float; y: float; z: float; yaw: float; vx: float; vy: float; vz: float; yaw_rate: float
@dataclass
class RoverState: x: float; y: float; yaw: float; v: float; omega: float

class PhysicsSystem:
    def step(self, dt: float, cmds: Cmds, world: World): ...
```

### 3.4 `systems/sensors.py` (GPS, Range)

```python
class GPSSensor:
    def update(self, state, world, rng): -> NavSatFix
class RangeSensor:
    def update(self, rover_state, world, rng): -> Range
```

### 3.5 `systems/camera.py` (RGB pinhole)

```python
@dataclass
class CameraIntrinsics: fx: float; fy: float; cx: float; cy: float; width: int; height: int; D: list[float]
@dataclass
class CameraTiming: latency_ms_mean: float; latency_ms_jitter: float; drop_probability: float

class PinholeCamera:
    def __init__(self, intrinsics: CameraIntrinsics, timing: CameraTiming, palette: np.ndarray, noise_cfg: NoiseCfg):
        self.queue: deque[Tuple[np.ndarray, rclpy.Time]] = deque()

    def render(self, cam_pose_map, world: World, rng) -> np.ndarray:
        """Return rgb8 image. Steps:
           1) For each pixel (u,v): make ray via K^-1 * [u,v,1].
           2) Intersect with terrain plane, then test primitives; keep nearest (z-test).
           3) Color = palette[class_id]. Optional blur/jitter.
        """

    def make_camera_info(self, stamp) -> CameraInfo:
        # fill K, P, D, width, height, frame_id='drone/camera_link'

    def enqueue_and_maybe_publish(self, img, info, now, rng):
        # draw latency, schedule publish, honor drop_probability
```

**Palette:** fixed table per PRD §4.2.2.1:

```python
PALETTE = np.array([
    [0xC8,0xC8,0xC8],  # Free ground
    [0x3C,0xB3,0x71],  # Obstacle
    [0xE7,0x4C,0x3C],  # Hazard
    [0x34,0x98,0xDB],  # Target/POI
    [0x1A,0xBC,0x9C],  # Water
], dtype=np.uint8)
```

### 3.6 `systems/radio.py`

```python
class RadioLink:
    def tx(self, direction: Literal['drone->rover','rover->drone'], payload: bytes, now: Time): ...
    def update(self, now: Time) -> list[DeliveredPacket]: ...
```

### 3.7 `systems/tf.py`

```python
class TFSystem:
    def publish(self, states: dict, stamp: rclpy.Time): ...
```

### 3.8 `systems/ros2_bridge.py`

- Publishers: odom (30 Hz), gps (5 Hz), range (10 Hz), camera (2 Hz), TF (60 Hz), optional world topics, radio metrics.
- Subscribers: `cmd_vel` x2; radio TX x2.
- Services: reset/pause/step/set_seed/info.

---

## 4) Parameters & Scenario Parsing (binding to PRD)

- Bind `microsim.drone.camera.*` (hz, width, height, encoding=rgb8, projection=pinhole, intrinsics K, distortion model & D, noise blur/color_jitter, latency mean/jitter/drop).
- Carry forward global `radio.*`, `sensors.*` (GPS/range), world grid settings.
- Scenario YAML fields mirror §8 exactly; loader supplies defaults if missing.

---

## 5) QoS Profiles (canonical)

- **Cmd/State:** reliable, keep_last=10.
- **Camera:** reliable, keep_last=2.
- **CameraInfo:** reliable, keep_last=2.
- **Radio TX:** best_effort, keep_last=1; **Radio RX:** best_effort, keep_last=5.
- **TF:** reliable.

---

## 6) Testing Plan (exact checks)

### Unit

- **Kinematics:** constant-command closed-form comparisons (tol 1e-6).
- **Ray DDA:** known grid cases (crossing boundaries, grazing).
- **Camera intrinsics:** `K`/`P` computation; inverse projection numerical sanity.
- **Palette:** class_id→color mapping stable.

### Integration (`launch_testing`)

- **Topic contract:** all topics/services present with expected types/QoS.
- **Rates:** odom 30 Hz, camera 2 Hz, gps 5 Hz, range 10 Hz (±5% over 30–60 s).
- **CameraInfo vs Image:** width/height match; `K` equals config; back-projection error ≤1 px on a synthetic grid.
- **Occlusion:** raised obstacle hides ground pixels in overlap region.
- **Latency:** measured inter-arrival minus render time mean/jitter within ±10 ms of config.
- **Radio:** drop % within ±1%; ordering preserved when not dropped.

### Determinism

- **Pre-noise camera:** byte-identical images across runs (seeded).
- **Seeded noise:** identical images when noise RNG seeded.
- **Rosbag reproducibility:** scripted mission bags hash-match (ignoring time headers if needed).

---

## 7) Performance Plan

- Target ≥0.95 RTF on modest CPU for 200×200 map, 2 robots, default rates.
- Camera renderer:

  - Use integer math where possible; vectorize ray dir table (precompute per pixel in camera frame).
  - Early-out if ray hits obstacle above ground; otherwise ground color.
  - Optional: stride-based loop or numba fallback (behind flag) if profiling demands.

---

## 8) Deliverables by Sprint

- **S0:** repo, CI, hello clock.
- **S1:** moving robots in RViz, odom+TF, YAML loader.
- **S2:** GPS, Range, Radio, obstacles; tests.
- **S3:** Camera + CameraInfo + latency/noise; occlusion tests; RViz config.
- **S4:** Determinism harness, bags, acceptance suite, docs.

---

## 9) Handoff Assets for Dev 2

- RViz config `rviz/microsim_view.rviz`.
- Example agent harness:

  - `agent/rover_avoid.py` (subscribe `/rover/range/front`, publish `/rover/cmd_vel`).
  - `agent/drone_hover.py` (subscribe `/drone/odom`, publish `/drone/cmd_vel`).
  - Radio demo: `agent/radio_ping.py`.

- `ros2 bag` record/replay scripts + sample scenario.

---

## 10) Risks & Mitigations (implementation)

- **Camera perf:** If 2 Hz misses, reduce post-fx and profile; precompute rays; clamp map size.
- **QoS mismatch:** Provide subscriber templates; CI test for incompatible QoS.
- **Determinism leakage:** Avoid multithreaded publishers; seed all RNGs; control publish order.

---

## 11) Prompts for Coding Agents (copy-ready)

**Example — Generate PinholeCamera class**

> Implement `PinholeCamera` in `systems/camera.py` (Python 3.11). Inputs: `CameraIntrinsics`, `CameraTiming`, `palette (uint8[5,3])`, `NoiseCfg`. Methods:
>
> - `render(cam_pose_map, world, rng) -> np.ndarray[rgb8, HxWx3]` per PRD §4.2.2.2.
> - `make_camera_info(stamp) -> CameraInfo` with K, P, D, width/height, `frame_id='drone/camera_link'`.
> - `enqueue_and_maybe_publish(img, info, now, rng)` adds latency (mean±jitter) and optional drop.
>   Constraints: no external rendering libs; pure numpy loops OK; keep deterministic with provided `rng`. Unit tests must pass for occlusion & back-projection.

**Example — Integration test for camera rate & CameraInfo**

> Write a `launch_testing` test that starts `microsim` with a tiny map, subscribes to `/drone/camera/image_raw` and `/drone/camera/camera_info` for 60 s, asserts rate `2.0 ± 5%`, and validates that `K` equals the configured intrinsics.

(Repeat similarly for Timekeeper, Physics, Radio.)

---

## 12) Definition of Done (per component)

- Code + unit tests + integration tests + docs.
- RViz verified visually for sanity.
- CI green on Linux with colcon.
- PRD acceptance criteria satisfied.
