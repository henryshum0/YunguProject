
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

The full stack (Gazebo + PX4 SITL + MicroXRCEAgent + gz bridge) is launched
with a single script:

```bash
# Terminal 1 — simulator + agent + bridge
./src/utils/start_sim.sh

# Terminal 2 — offboard state machine + SUPER planner + RViz
source install/setup.bash
ros2 launch offboard offboard.launch.py
```

### Simulation config (`config/simulation.yaml`)

The vehicle **model**, the Gazebo **map/world**, the GZ version, the uXRCE-DDS
port, and the GZ→ROS **bridge topics** are all chosen in
[`config/simulation.yaml`](config/simulation.yaml) — the single source of truth
for the sim. Edit it, then run `start_sim.sh` with no arguments:

| Key               | Description                                                        |
|-------------------|--------------------------------------------------------------------|
| `model`           | Gazebo vehicle model / PX4 gz airframe (e.g. `x500_lidar`)         |
| `world`           | Gazebo map (.sdf in `VisionFlow-PX4/Tools/simulation/gz/worlds`, e.g. `yungu`) |
| `gz_version`      | Gazebo distro (`harmonic`)                                         |
| `xrce_port`       | UDP port for the MicroXRCEAgent                                    |
| `bridge.enabled`  | Launch the GZ↔ROS `parameter_bridge`                               |
| `bridge.tf_enabled` | Launch the TF bridge (`tf_bridge.py`)                            |
| `bridge.topics`   | GZ topics bridged into ROS 2 (e.g. `/x500_lidar/scan`, `/x500_lidar/scan/points`) |

`model` + `world` combine into the PX4 make target `gz_<model>_<world>`
(e.g. `gz_x500_lidar_yungu`). To see the effective values and the full list of
available models/maps:

```bash
./src/utils/start_sim.sh --help
```

### Env-var overrides

Every config value can be overridden on the command line for a one-off run
(without editing the file):

| Variable     | Default                | Description                                          |
|--------------|------------------------|------------------------------------------------------|
| `PX4_MODEL`  | from config (`x500_lidar`) | Gazebo vehicle model / airframe for `make px4_sitl` |
| `PX4_WORLD`  | from config (`yungu`)  | Gazebo map/world (.sdf in the worlds dir)            |
| `XRCE_PORT`  | from config (`8888`)   | UDP port for the MicroXRCEAgent                      |
| `GZ_VERSION` | from config (`harmonic`) | gz-transport version for `ros_gz_bridge` (gz-sim 8) |
| `HEADLESS`   | *(unset)*              | Any non-empty value (e.g. `1`) runs Gazebo without its GUI (server only) |

```bash
# Run a different airframe / world without editing the config
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./src/utils/start_sim.sh

# Legacy form: pass the full make target directly
PX4_MODEL=gz_x500_lidar_yungu ./src/utils/start_sim.sh

# Use a non-default uXRCE-DDS port (must match PX4 UXRCE_DDS_PRT)
XRCE_PORT=2018 ./src/utils/start_sim.sh

# Run Gazebo without its GUI (physics server only) - useful for CI or
# machines without a display
HEADLESS=1 ./src/utils/start_sim.sh
```

### What `start_sim.sh` launches

| # | Component | Command |
|---|---|---|
| 1 | PX4 SITL + Gazebo | `make px4_sitl gz_<model>_<world>` (from config) |
| 2 | MicroXRCEAgent | `MicroXRCEAgent udp4 -p <xrce_port>` (from config) |
| 3 | GZ ↔ ROS bridge + TF | `src/utils/gz_bridges/bridge.sh` (topics from config) |

`start_sim.sh` waits for PX4 to report "Ready for takeoff", then starts the
agent and the bridge. Logs go to `/tmp/yungu_sim/` (`px4_sitl.log`,
`xrce_agent.log`, `bridge.log`).

**Stopping:** press `Ctrl+C` to stop everything (the PX4 window is closed and
all spawned processes are killed). If anything lingers — e.g. the `gz-server`
that PX4 detaches into its own session — run:

```bash
./src/utils/stop_sim.sh
```

### Bridging and visualization

- The gz bridge is independent of RViz: run `src/utils/gz_bridges/bridge.sh`
  alone (it reads the topics from `config/simulation.yaml`), and launch RViz
  separately via the offboard launch file.
- RViz shows the drone (TF), the lidar point cloud, and a "2D Goal Pose" tool
  that publishes goals to `/goal_pose` (default `rviz:=true`, disable with
  `rviz:=false`).
- The SUPER planner is launched by `offboard.launch.py` using
  `src/SUPER/super_planner/config/gazebo.yaml`, which subscribes to the bridged
  topics (`/x500_lidar/scan/points`, `/odom`).

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
./temp/start_all_fastlio.sh

# Gazebo server-only (no GUI) — RViz still opens for planning visualization
HEADLESS=1 ./temp/start_all_fastlio.sh

# Skip RViz entirely
NO_RVIZ=1 ./temp/start_all_fastlio.sh
```

What `start_all_fastlio.sh` adds over `start_sim.sh`:

| # | Component | Notes |
|---|---|---|
| 1 | `temp/gazebo_imu_bridge.py` | PX4 `sensor_combined` (FRD) → `/livox/imu` (ENU): axis flip + rolling time sync |
| 2 | `temp/add_time_field.py` | Adds `time` field to the Gazebo cloud (0 for instantaneous scans) |
| 3 | `fast_lio` (`fastlio_mapping`) | LiDAR-inertial odometry, config: `temp/fastlio_gazebo.yaml` |
| 4 | `temp/fastlio_px4_bridge.py` | FAST-LIO `/Odometry` (ENU) → `/fmu/in/vehicle_visual_odometry` (NED) for EKF2 |
| 5 | `super_bridge` | PX4 fused odom + raw cloud → `/lidar_slam/odom` + `/cloud_registered` |

PX4 EKF2 external-vision params are set in the x500 airframe
(`4008_gz_x500_lidar`): `EKF2_EV_CTRL 13` (HPOS+VEL+YAW fusion — **no VPOS**:
FAST-LIO has no absolute height reference, so the barometer keeps the
vertical channel and the SLAM z drift is not propagated into altitude),
`EKF2_EV_DELAY 5`, `EKF2_EVP_NOISE 0.1`, `EKF2_EVV_NOISE 0.1`,
`EKF2_EVA_NOISE 0.05`, plus `COM_POWER_OVERRIDE 1` so SITL arming is not
blocked by the power preflight check.

**Verify the fusion pipeline is alive:**

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once --qos-reliability best_effort
ros2 topic echo /lidar_slam/odom --once --qos-reliability best_effort
ros2 topic echo /cloud_registered --once --qos-reliability best_effort
```

### FAST-LIO fusion — measured results (2026-08)

Measured against the Gazebo ground truth (`/odom`) in the `yungu` world,
using [`temp/check_pos4.py`](temp/check_pos4.py) (scenario-based attribution,
CSV output):

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

1. **Static/hover z bias (+0.42–0.52 m, constant)** — frame-origin offset:
   FAST-LIO's `camera_init` origin is the first-frame LiDAR pose, while
   Gazebo's world origin is the spawn pose. Not an error — align the origins
   before comparing with ground truth. Horizontal is unaffected (≤ 0.04 m).
2. **In-flight horizontal lag (~0.4–0.7 s, ∝ speed)** — flight bias ≈ speed ×
   0.5–0.6 s; time-shifting the FAST-LIO output by −0.7 s removes ~75 % of the
   bias. Caused by the *simulation* pipeline: ~184 KB point clouds forwarded
   over multiple hops under WSL + PX4 clock vs sim-time skew. Real hardware
   with hardware-stamped sensor data should not show this.
3. **Vertical drift after flight (+1 m, persists)** — the SLAM has no absolute
   height reference; the z offset gets absorbed into the map and stays locked.
   Use a barometer/height source to constrain z on real hardware.