# EGO/PX4 navigation launches

This directory contains a navigation-only launch for the single-drone
EGO-Planner/PX4 adapter and a combined launch that also starts coverage
planning. PX4/Gazebo, the bridge, and `gz_sensor_interface` remain separate.

After building and sourcing the workspace overlay, run:

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
ros2 launch /home/windshape/YunguProject/src/launch/ego_single_drone.launch.py

# Or include on-demand coverage planning as well.
ros2 launch /home/windshape/YunguProject/src/launch/coverage_and_offboard.launch.py
```

The launcher uses the installed `coverage_planner` Yungu example by default:
`yungu_planner.json` and its adjacent `yungu_map.json`.  Select a different
planner JSON with:

```bash
ros2 launch /home/windshape/YunguProject/src/launch/coverage_and_offboard.launch.py \
  config_file:=/absolute/path/to/planner.json
```

Both launch files read the workspace configuration files:

- `src/simulation/config/simulation.yaml`
- `src/navigation/config/offboard/offboard_fsm.yaml`
- `src/navigation/config/offboard/topics.yaml`

The EGO launch uses `/gz/odom_super` and `/gz/point_cloud_super` to maintain a
persistent, horizontally rolling 60 × 60 × 16 m log-odds grid in `world`.
LiDAR hits and free-space rays are fused incrementally; unobserved obstacles
expire after 30 seconds. The retained inflated cloud remains available on
`/grid_map/occupancy_inflate` for RViz. Goals outside that local rolling region
are not supported by this initial setup.

The direct-cloud map assumes odometry and cloud coordinates stay aligned in
`world`. Restart the navigation launch after a localization-frame reset.

The combined launch starts the coverage planning action server and waypoint
queue services; planning a route does not automatically enqueue it to the
vehicle. Use `SearchSkill` or `/waypoint_buffer` to queue a returned route.
