
```
# clone simulator
git clone --recursive https://github.com/henryshum0/YunguProject.git
cd YunguProject

# install depdency
./install_deps.sh

# build
colcon build --symlink-install

# before running the yungu map, put the yungu.glb file
# under VisionFlow-PX4/Tools/simulation/gz/worlds
```

## Running the simulation

Start everything with two terminals:

```bash
# Terminal 1 — simulator + agent + bridge (Gazebo + PX4 SITL + MicroXRCEAgent + gz bridge)
./utils/start_sim.sh

# Terminal 2 — offboard state machine + SUPER planner + RViz
source install/setup.bash
ros2 launch offboard offboard.launch.py
```

`start_sim.sh` picks the vehicle model and map from `config/simulation.yaml`,
waits for PX4 to report "Ready for takeoff", then starts the MicroXRCEAgent and
the GZ↔ROS bridge (logs in `/tmp/yungu_sim/`). Stop everything with `Ctrl+C`,
or run `./utils/stop_sim.sh` if anything lingers.

Everything else — the offboard state machine, SUPER planner, birdview overlay
and both RViz windows — comes up with `offboard.launch.py`.

## Configuration files

All run-time configuration lives in the [`config/`](config/) folder and is read
through the shared [`config/sim_config.py`](config/sim_config.py) helper, so
edits take effect on the next launch (no rebuild needed).

| File | Purpose | Key parameters |
|------|---------|----------------|
| [`config/simulation.yaml`](config/simulation.yaml) | Simulator: Gazebo vehicle model, map/world, uXRCE-DDS port, GZ→ROS bridge topics | `model`, `world`, `gz_version`, `xrce_port`, `bridge.*` |
| [`config/offboard.yaml`](config/offboard.yaml) | Offboard state machine + planner tuning + waypoint following | `visualization`, `update_rate`, `planner_cmd_hz`, `default_height`, `landing_vel`, `landing_z`, `goal_height`, `planner_config`, `waypoint_reached_dist`, `waypoint_hold_time`, `world_offset_x/y/z` |
| [`config/birdview.yaml`](config/birdview.yaml) | Aerial birdview overlay on the ground plane (top-down map reference in RViz) | `extent_x`, `extent_y`, `z`, `offset_x`, `offset_y`, `yaw`, `max_points` |
| [`config/super_planner/gazebo-smooth.yaml`](config/super_planner/gazebo-smooth.yaml) | SUPER planner behaviour (frontend, trajectory optimization, ROG-Map) | `fsm.click_height`, `super_planner.*`, `traj_opt.*`, `astar.*`, `rog_map.*` |

Notable behaviour:

- `simulation.yaml` is the single source of truth for the sim: `model` +
  `world` combine into the PX4 make target `gz_<model>_<world>`
  (e.g. `gz_swan_gamma_v2_yungu`). `./utils/start_sim.sh --help` lists the
  available models/maps.
- `offboard.yaml` `visualization: false` runs headless — no RViz, no birdview
  overlay, and SUPER's marker publishing is turned off.
- `offboard.yaml` `goal_height` overrides SUPER's `fsm.click_height` (goal
  height used by the RViz "2D Goal Pose" tool); `planner_config` selects which
  SUPER config is used.
- Env-var overrides still work for one-off sim runs without editing the file,
  e.g. `PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./utils/start_sim.sh`,
  `XRCE_PORT=2018`, or `HEADLESS=1` (no Gazebo GUI).

### RViz views

`offboard.launch.py` opens two RViz windows together (both optional):

- **birdview** — top-down planning view: aerial map + occupied map + trajectory
  (planning window, `rviz_config:=birdview.rviz` by default)
- **freelook** — free-rotate 3D debug view: corridors, trajectories, markers
  (debug window, `rviz_freelook_config:=freelook.rviz` by default)

Toggle them with `rviz:=true/false` and `rviz_freelook:=true/false`.

### Env-var overrides

Every config value can be overridden on the command line for a one-off run
(without editing the file):

| Variable     | Default                | Description                                          |
|--------------|------------------------|------------------------------------------------------|
| `PX4_MODEL`  | from config (`swan_gamma_v2`) | Gazebo vehicle model / airframe for `make px4_sitl` |
| `PX4_WORLD`  | from config (`yungu`)  | Gazebo map/world (.sdf in the worlds dir)            |
| `XRCE_PORT`  | from config (`8888`)   | UDP port for the MicroXRCEAgent                      |
| `GZ_VERSION` | from config (`harmonic`) | gz-transport version for `ros_gz_bridge` (gz-sim 8) |
| `HEADLESS`   | *(unset)*              | Any non-empty value (e.g. `1`) runs Gazebo without its GUI (server only) |

```bash
# Run a different airframe / world without editing the config
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./utils/start_sim.sh

# Legacy form: pass the full make target directly
PX4_MODEL=gz_swan_gamma_v2_yungu ./utils/start_sim.sh

# Use a non-default uXRCE-DDS port (must match PX4 UXRCE_DDS_PRT)
XRCE_PORT=2018 ./utils/start_sim.sh

# Run Gazebo without its GUI (physics server only) - useful for CI or
# machines without a display
HEADLESS=1 ./utils/start_sim.sh
```

### What `start_sim.sh` launches

| # | Component | Command |
|---|---|---|
| 1 | PX4 SITL + Gazebo | `make px4_sitl gz_<model>_<world>` (from config) |
| 2 | MicroXRCEAgent | `MicroXRCEAgent udp4 -p <xrce_port>` (from config) |
| 3 | GZ ↔ ROS bridge + TF | `utils/gz_bridges/bridge.sh` (topics from config) |

`start_sim.sh` waits for PX4 to report "Ready for takeoff", then starts the
agent and the bridge. Logs go to `/tmp/yungu_sim/` (`px4_sitl.log`,
`xrce_agent.log`, `bridge.log`).

**Stopping:** press `Ctrl+C` to stop everything (the PX4 window is closed and
all spawned processes are killed). If anything lingers — e.g. the `gz-server`
that PX4 detaches into its own session — run:

```bash
./utils/stop_sim.sh
```

### Bridging and visualization

- The gz bridge is independent of RViz: run `utils/gz_bridges/bridge.sh`
  alone (it reads the topics from `config/simulation.yaml`), and launch RViz
  separately via the offboard launch file.
- RViz shows the drone (TF), the lidar point cloud, and a "2D Goal Pose" tool
  that publishes goals to `/goal_pose` (default `rviz:=true`, disable with
  `rviz:=false`).
- The SUPER planner is launched by `offboard.launch.py` using the config from
  `config/offboard.yaml` (`planner_config`, resolved under
  `config/super_planner/`), which subscribes to the super_bridge outputs
  (`/cloud_registered`, `/lidar_slam/odom`).
- **Ground truth odometry** (`/odom`, world frame): the yungudemo branch's
  swan_gamma_v2 carries no top-level `OdometryPublisher`, so the bridge points
  at the model-instance topic `/model/swan_gamma_v2_0/odometry` (see
  `config/simulation.yaml`). The truth path `/gt_path` and the 
  `cmd_record` comparison scripts depend on it.
- **TF bridge QoS**: `tf_bridge.py` subscribes `/lidar_slam/odom` with
  `BEST_EFFORT` to match super_bridge's publisher — a RELIABLE subscription
  silently never receives anything and the `world → base_link` TF stays stale.

## FAST-LIO mode (LiDAR-inertial odometry + PX4 EKF2 fusion)

A one-shot launcher that runs the full stack **with FAST-LIO** feeding its
odometry into PX4's EKF2 (external vision fusion), then the fused PX4
odometry drives the SUPER planner via `super_bridge` — mirroring the
real-hardware setup (no Gazebo ground-truth involved):

```
FAST-LIO /Odometry ──→ fastlio_px4_bridge ──→ /fmu/in/vehicle_visual_odometry
                                                     ↓ PX4 EKF2 fusion
                        PX4 /fmu/out/vehicle_odometry ──→ super_bridge
                                                     ↓
                        /lidar_slam/odom + /cloud_registered ──→ planner
```

```bash
# Full stack: GPU-forced Gazebo + PX4 + FAST-LIO + EKF2 fusion + planner + RViz
./utils/start_fastlio.sh

# Gazebo server-only (no GUI) — RViz still opens for planning visualization
HEADLESS=1 ./utils/start_fastlio.sh

# Skip RViz entirely
NO_RVIZ=1 ./utils/start_fastlio.sh
```

What `utils/start_fastlio.sh` adds over `utils/start_sim.sh`:

| # | Component | Notes |
|---|---|---|
| 1 | `utils/fastlio/gazebo_imu_bridge.py` | PX4 `sensor_combined` (FRD) → `/livox/imu` (ENU): axis flip + rolling time sync |
| 2 | `utils/fastlio/add_time_field.py` | Adds `time` field to the Gazebo cloud (0 for instantaneous scans) |
| 3 | `fast_lio` (`fastlio_mapping`) | LiDAR-inertial odometry, config: `config/fastlio_swan_gamma_effect.yaml` (ikd-Tree incremental map → `/cloud_effected`) |
| 4 | `utils/fastlio/fastlio_px4_bridge.py` | FAST-LIO `/Odometry` (ENU) → `/fmu/in/vehicle_visual_odometry` (NED) for EKF2 |
| 5 | `super_bridge` | PX4 fused odom + raw cloud → `/lidar_slam/odom` + `/cloud_registered` |

PX4 EKF2 external-vision params are set in the swan_gamma_v2 airframe
(`4007_gz_swan_gamma_v2`): `EKF2_EV_CTRL 13` (HPOS+VEL+YAW fusion — **no VPOS**:
FAST-LIO has no absolute height reference, so the barometer keeps the
vertical channel and the SLAM z drift is not propagated into altitude),
`EKF2_EV_DELAY 5`, `EKF2_EVP_NOISE 0.1`, `EKF2_EVV_NOISE 0.1`,
`EKF2_EVA_NOISE 0.05`.

**Verify the fusion pipeline is alive:**

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --qos-reliability best_effort --spin-time 3
ros2 topic echo /lidar_slam/odom --qos-reliability best_effort --spin-time 3
ros2 topic echo /cloud_registered --qos-reliability best_effort --spin-time 3
```

## Point-to-point navigation (interface for search algorithms)

This section is the **only** interface a search/planning algorithm needs.
Once the FAST-LIO stack is up, the drone auto-takes-off to `default_height`
and enters `IDLE`; any ROS 2 node can then command it to fly by publishing a
`geometry_msgs/PoseStamped` — no launch changes, no code changes.

### Quick start

```bash
# Terminal 1 — full FAST-LIO simulation stack (Gazebo + PX4 + FAST-LIO + EKF2)
./utils/start_fastlio.sh

# Terminal 2 — offboard state machine + SUPER planner + RViz
source install/setup.bash
ros2 launch offboard offboard.launch.py
```

Wait for the takeoff to complete (the offboard log shows
`State: TAKEOFF → IDLE`), then publish waypoints. RViz's "2D Goal Pose" tool
already publishes to `/waypoint_pose` (re-targeted in `birdview.rviz` /
`freelook.rviz`), so hand-clicked goals and algorithmic goals go through the
exact same path.

### Goal input topics

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/waypoint_pose` | `geometry_msgs/PoseStamped` | sub: best_effort/volatile | **Batch waypoint input (recommended).** Each message is queued in the offboard waypoint buffer and flown one at a time. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | sub: best_effort/volatile | **Direct single goal.** Same topic SUPER's click-goal subscribes to; the offboard node also publishes here internally to hand the current waypoint to SUPER. |
| `/waypoint_buffer` | `geometry_msgs/PoseStamped` | reliable | Internal channel (`goal_marker_node` → offboard node). Don't publish here directly. |
| `/waypoint_markers` | `visualization_msgs/MarkerArray` | transient_local | Waypoint-buffer feedback: green = queued, yellow = currently pursued, cyan line = route. |

Data flow:

```
    /waypoint_pose ──→ goal_marker_node ──→ /waypoint_buffer ──→ offboard node
  (your algorithm)                              (FIFO queue)         │
                                                                     ↓
                                                       /goal_pose (one at a time)
                                                                     ↓
                                                          fsm_node (SUPER)
                                                                     ↓
                                                    /planning/pos_cmd ──→ offboard ──→ PX4
```

Example publisher (Python):

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        self.pub = self.create_publisher(PoseStamped, "/waypoint_pose", 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"          # ENU world frame, z up
        msg.pose.position.x = 10.0             # east
        msg.pose.position.y = 5.0              # north
        msg.pose.position.z = 5.0              # up [m]
        msg.pose.orientation.w = 1.0           # yaw follows flight direction
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(GoalPublisher())
```

**QoS caveat:** the `/waypoint_pose` subscription is `best_effort` +
`keep_last(1)` — publishing many waypoints faster than the node processes them
drops messages. Publish at **≥ 0.5 s intervals** (as above) and confirm
acceptance via the offboard log (`Waypoint buffered (#N)`) or
`/waypoint_markers`.

### State feedback topics

| Topic | Type | Description |
|---|---|---|
| `/lidar_slam/odom` | `nav_msgs/Odometry` | **Fused odometry** (FAST-LIO → PX4 EKF2), world ENU — the state feedback for your algorithm |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | World-frame lidar cloud (ROG-Map input) |
| `/planning/pos_cmd` | `mars_quadrotor_msgs/PositionCommand` | SUPER's commanded trajectory (position/velocity/acceleration/yaw/yaw_dot), ~100 Hz |
| `/planning_cmd/poly_traj` | `mars_quadrotor_msgs/PolynomialTrajectory` | Polynomial trajectory (MPC heartbeat) |
| `fsm/path` | `nav_msgs/Path` | Planned path (A* → optimization) |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | Raw PX4 local position (NED) |
| `/fmu/out/vehicle_status_v4` | `px4_msgs/VehicleStatus` | Vehicle status (arming state, nav state) |

### Coordinate frames

- **Waypoints, goals and planner output are all ENU world frame**
  (`frame_id: "world"`, x = east, y = north, z = up). Frame conversions
  (ENU → PX4 NED, yaw included) happen automatically inside the offboard node.
- PX4's EKF local frame origin sits at the model spawn pose, which differs
  from the Gazebo world origin when the airframe sets a non-zero
  `PX4_GZ_MODEL_POSE` (swan_gamma_v2 currently spawns at `-4,-2,1.15392`).
  `super_bridge` and the offboard node compensate this automatically via the
  `world_offset_x/y/z` parameters in `config/offboard.yaml` — **keep them in
  sync with the airframe's spawn pose** (all zero when the model spawns at the
  world origin). With the offset applied, `/lidar_slam/odom`,
  `/cloud_registered`, the TF tree and waypoints all agree in the world frame.
- Waypoint arrival is judged by **horizontal** distance to the waypoint
  (`waypoint_reached_dist`); the drone holds `waypoint_hold_time` seconds
  before the next buffered waypoint is handed to SUPER.

### Waypoint-following parameters (`config/offboard.yaml`)

| Key | Default | Description |
|---|---|---|
| `waypoint_reached_dist` | `3.0` m | Horizontal distance below which a waypoint is considered reached |
| `waypoint_hold_time` | `0.0` s | Hover time after reaching a waypoint before starting the next |
| `goal_height` | `5.0` m | Goal altitude for planar goals (overrides SUPER `fsm.click_height`) |
| `planner_cmd_hz` | `80.0` Hz | Cmd rate at which the offboard state machine hands over to the planner |
| `default_height` | `5.0` m | Auto-takeoff altitude (NED) after OFFBOARD |

All of these are launch arguments too, so they can be overridden per run, e.g.
`ros2 launch offboard offboard.launch.py waypoint_reached_dist:=2.0
waypoint_hold_time:=1.0`.

### Landing

```bash
ros2 service call /offboard/land std_srvs/srv/Trigger
```

### Recording & evaluation

`cmd_record` (see the section above) records each goal together with the
commanded trajectory and the fused odometry:

```bash
# Terminal 3 — recorder + live plot (starts on the first goal)
ros2 launch cmd_record record.launch.py

# Plot a saved segment afterwards (defaults to the newest CSV in cmd_log/)
ros2 run cmd_record plot_csv
```

Each goal click writes `cmd_log/goal_<NNN>_<timestamp>.csv` containing the
goal position, the commanded trajectory, and the real odometry.

### FAST-LIO fusion — measured results (2026-08)

Measured against the Gazebo ground truth (`/odom`) in the `yungu` world
(scenario-based attribution):

| Scenario | Horizontal error | Vertical bias | Notes |
|---|---|---|---|
| Static on ground | **0.00 m** | +0.52 m | zero drift, fully stable |
| Hover @ 4.5 m | **0.04 m** | +0.42 m | horizontal is very good |
| Flight (v ≤ 4.7 m/s) | 0.4–2.8 m (∝ speed) | +0.6–1.4 m | lag + vertical drift |
| After stopping | **0.18 m** (recovers) | +1.36 m (persists) | horizontal recovers, vertical does not |

The PX4-EKF2 fused odometry (`/lidar_slam/odom`) tracks FAST-LIO within
**0.03–0.12 m** — EKF2 fully adopts the visual odometry, i.e. the fusion
pipeline works as intended.

Known bias sources (all accounted for, none a FAST-LIO algorithm bug):

1. **Static/hover z bias (+0.42–0.52 m, constant)** — residual frame-origin
   offset between FAST-LIO's `camera_init` (first-frame LiDAR pose) and the
   world frame. The spawn-pose offset is compensated by `world_offset_*`, but
   a small residual (a few cm, vertical only) remains from the EKF/camera_init
   initialization. Horizontal is unaffected (≤ 0.04 m).
2. **In-flight horizontal lag (~0.4–0.7 s, ∝ speed)** — flight bias ≈ speed ×
   0.5–0.6 s; time-shifting the FAST-LIO output by −0.7 s removes ~75 % of the
   bias. Caused by the *simulation* pipeline: ~184 KB point clouds forwarded
   over multiple hops under WSL + PX4 clock vs sim-time skew. Real hardware
   with hardware-stamped sensor data should not show this.
3. **Vertical drift after flight (+1 m, persists)** — the SLAM has no absolute
   height reference; the z offset gets absorbed into the map and stays locked.
   Use a barometer/height source to constrain z on real hardware.

### Recording & plotting (`cmd_record`)

The [`cmd_record`](src/cmd_record) package records the commanded trajectory
together with the real drone odometry — one CSV per goal click — and shows a
live matplotlib plot (plus post-hoc plotting). Run it in a third terminal:

```bash
# Terminal 3 — recorder + live plot (click a goal in RViz to start recording)
source install/setup.bash
ros2 launch cmd_record record.launch.py

# Plot a saved segment afterwards (defaults to the newest CSV in cmd_log/)
ros2 run cmd_record plot_csv
```

Each goal click writes `cmd_log/goal_<NNN>_<timestamp>.csv` containing the goal
position, the commanded trajectory, and the real odometry. Its dependencies
(numpy, matplotlib, and `python3-tk` for the live plot) are installed by
`./install_deps.sh`.

## Integrating with an external search/coverage planner

The point-to-point interface above is the execution endpoint for offline search
planners. A worked example — integrating
[coverage-search-planner](https://github.com/Jocelyn-2005/coverage-search-planner)
(`feature/continuous-lane-planning`), which generates ground-cover video
detection flight plans as `flight_plan.json` (ENU meters) — is documented in
[`docs/coverage-search-integration.md`](docs/coverage-search-integration.md).

In short: convert the plan's waypoints into the world ENU frame (a constant
translation `T` between the planner's local ENU origin and the Gazebo world
origin), then publish them one by one to `/waypoint_pose` (≥ 0.5 s apart,
respecting `turn_in_place` / `hold_time_s` and never skipping
`obstacle_avoidance` segments). See the doc for the full protocol summary,
coordinate calibration methods, an adapter skeleton and the run steps.
