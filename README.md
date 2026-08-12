
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
./src/utils/start_sim.sh

# Terminal 2 — offboard state machine + SUPER planner + RViz
source install/setup.bash
ros2 launch offboard offboard.launch.py
```

`start_sim.sh` picks the vehicle model and map from `config/simulation.yaml`,
waits for PX4 to report "Ready for takeoff", then starts the MicroXRCEAgent and
the GZ↔ROS bridge (logs in `/tmp/yungu_sim/`). Stop everything with `Ctrl+C`,
or run `./src/utils/stop_sim.sh` if anything lingers.

Everything else — the offboard state machine, SUPER planner, birdview overlay
and both RViz windows — comes up with `offboard.launch.py`.

## Configuration files

All run-time configuration lives in the [`config/`](config/) folder and is read
through the shared [`config/sim_config.py`](config/sim_config.py) helper, so
edits take effect on the next launch (no rebuild needed).

| File | Purpose | Key parameters |
|------|---------|----------------|
| [`config/simulation.yaml`](config/simulation.yaml) | Simulator: Gazebo vehicle model, map/world, uXRCE-DDS port, GZ→ROS bridge topics | `model`, `world`, `gz_version`, `xrce_port`, `bridge.*` |
| [`config/offboard.yaml`](config/offboard.yaml) | Offboard state machine + planner tuning + waypoint following | `visualization`, `update_rate`, `planner_cmd_hz`, `default_height`, `landing_vel`, `landing_z`, `goal_height`, `planner_config`, `waypoint_reached_dist`, `waypoint_hold_time` |
| [`config/birdview.yaml`](config/birdview.yaml) | Aerial birdview overlay on the ground plane (top-down map reference in RViz) | `extent_x`, `extent_y`, `z`, `offset_x`, `offset_y`, `yaw`, `max_points` |
| [`config/super_planner/gazebo-smooth.yaml`](config/super_planner/gazebo-smooth.yaml) | SUPER planner behaviour (frontend, trajectory optimization, ROG-Map) | `fsm.click_height`, `super_planner.*`, `traj_opt.*`, `astar.*`, `rog_map.*` |

Notable behaviour:

- `simulation.yaml` is the single source of truth for the sim: `model` +
  `world` combine into the PX4 make target `gz_<model>_<world>`
  (e.g. `gz_x500_lidar_yungu`). `./src/utils/start_sim.sh --help` lists the
  available models/maps.
- `offboard.yaml` `visualization: false` runs headless — no RViz, no birdview
  overlay, and SUPER's marker publishing is turned off.
- `offboard.yaml` `goal_height` overrides SUPER's `fsm.click_height` (goal
  height used by the RViz "2D Goal Pose" tool); `planner_config` selects which
  SUPER config is used.
- Env-var overrides still work for one-off sim runs without editing the file,
  e.g. `PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./src/utils/start_sim.sh`,
  `XRCE_PORT=2018`, or `HEADLESS=1` (no Gazebo GUI).

### RViz views

`offboard.launch.py` opens two RViz windows together (both optional):

- **birdview** — top-down planning view: aerial map + occupied map + trajectory
  (planning window, `rviz_config:=birdview.rviz` by default)
- **freelook** — free-rotate 3D debug view: corridors, trajectories, markers
  (debug window, `rviz_freelook_config:=freelook.rviz` by default)

Toggle them with `rviz:=true/false` and `rviz_freelook:=true/false`.

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