
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
- The SUPER planner is launched by `offboard.launch.py`; it loads its config
  from `config/super_planner/gazebo-smooth.yaml` (the project-level `config/`
  folder) and subscribes to the bridged topics (`/x500_lidar/scan/points`,
  `/odom`).

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