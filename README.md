
```
# clone simulator
git clone --recursive https://github.com/Renwang-Huang/VisionFlow-PX4.git

# switch to yungu branch
cd VisionFlow-PX4
git checkout yungudemo
cd ..

# install depdency
./install_deps.sh
```

## Running the simulation

The full stack (Gazebo + PX4 SITL + MicroXRCEAgent + gz bridge) is launched
with a single script:

```bash
# Terminal 1 — simulator + agent + bridge
./src/utils/start_sim.sh

# Terminal 2 — offboard state machine + SUPER planner + RViz
ros2 launch offboard offboard.launch.py
```

### What `start_sim.sh` launches

| # | Component | Command |
|---|---|---|
| 1 | PX4 SITL + Gazebo | `make px4_sitl gz_x500_lidar_yungu` |
| 2 | MicroXRCEAgent | `MicroXRCEAgent udp4 -p 8888` |
| 3 | GZ ↔ ROS bridge + TF | `src/utils/gz_bridges/bridge.sh` |

`start_sim.sh` waits for PX4 to report "Ready for takeoff", then starts the
agent and the bridge. Logs go to `/tmp/yungu_sim/` (`px4_sitl.log`,
`xrce_agent.log`, `bridge.log`). Press `Ctrl+C` to stop everything.

### Config via env vars

| Variable     | Default                 | Description                                          |
|--------------|-------------------------|------------------------------------------------------|
| `PX4_MODEL`  | `gz_x500_lidar_yungu`   | Gazebo model / airframe target for `make px4_sitl`   |
| `XRCE_PORT`  | `8888`                  | UDP port for the MicroXRCEAgent                      |
| `GZ_VERSION` | `harmonic`              | gz-transport version for `ros_gz_bridge` (gz-sim 8)  |

```bash
# Run a different airframe / model
PX4_MODEL=gz_x500_lidar_yungu ./src/utils/start_sim.sh

# Use a non-default uXRCE-DDS port (must match PX4 UXRCE_DDS_PRT)
XRCE_PORT=2018 ./src/utils/start_sim.sh

# Explicitly select the gz transport version
GZ_VERSION=harmonic ./src/utils/start_sim.sh
```

### Bridging and visualization

- The gz bridge is independent of RViz: run `src/utils/gz_bridges/bridge.sh`
  alone, and launch RViz separately via the offboard launch file.
- RViz shows the drone (TF), the lidar point cloud, and a "2D Goal Pose" tool
  that publishes goals to `/goal_pose` (default `rviz:=true`, disable with
  `rviz:=false`).
- The SUPER planner is launched by `offboard.launch.py` using
  `config/gazebo.yaml`, which subscribes to the bridged topics
  (`/x500_lidar/scan/points`, `/odom`).