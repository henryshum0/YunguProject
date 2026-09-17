# YunguProject operations reference

This reference holds the lower-level operating detail for the Yungu autonomy
stack. Start with the workspace [README](../README.md) for architecture and a
first-run workflow.

## Configuration

Configuration is external to the ROS package builds, so edits take effect on
the next launch.

| File or directory | Purpose | Important settings |
|---|---|---|
| [`src/simulation/config/simulation.yaml`](../src/simulation/config/simulation.yaml) | PX4/Gazebo model, world, bridge, and uXRCE settings | `model`, `world`, `gz_version`, `xrce_port`, `bridge.*` |
| [`src/simulation/config/gz_sensor_interface.yaml`](../src/simulation/config/gz_sensor_interface.yaml) | Gazebo sensor bridge topics, frames, and extrinsics | `lidar_sensor.*`, `truth_odom.*`, `super_lidar.*` |
| [`src/simulation/config/visualization.yaml`](../src/simulation/config/visualization.yaml) | TF, path, point-cloud, and RViz settings | `frames.*`, `visual_tf.*`, `rviz.*` |
| [`src/navigation/config/offboard/topics.yaml`](../src/navigation/config/offboard/topics.yaml) | Shared EGO/PX4 navigation endpoints | `offboard_fsm.*` and `ego_planner.*` |
| [`src/navigation/config/offboard/offboard_fsm.yaml`](../src/navigation/config/offboard/offboard_fsm.yaml) | Offboard state machine and EGO local-grid settings | takeoff, timeout, local-grid, and EGO motion limits |
| [`src/search/config/`](../src/search/config/) | Coverage planner JSON and reusable map geometry | `*_planner.json`, `*_map.json` |

### Per-run simulation overrides

| Variable or argument | Default | Description |
|---|---|---|
| `PX4_MODEL`, `PX4_WORLD` | `simulation.yaml` | Override the Gazebo airframe or world. Legacy `gz_<model>_<world>` form is accepted. |
| `XRCE_PORT` | `8888` | MicroXRCE agent port. |
| `GZ_VERSION` | `harmonic` | Gazebo transport version used by bridges. |
| `HEADLESS=1` | unset | Run Gazebo server without the GUI. |
| `rviz:=false`, `rviz_freelook:=false` | `true` | Disable either RViz window in the visualization launcher. |

```bash
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./utils/start_sim.sh
HEADLESS=1 ./utils/start_sim.sh
ros2 launch visualization visualization.launch.py rviz:=false
```

## Offboard state machine

`offboard_node` owns PX4 flight-state transitions and forwards EGO's ENU
trajectory commands only while executing an accepted queue goal.

![Offboard FSM state machine](assets/offboard_fsm_state_machine.png)

The diagram source is [`assets/offboard_fsm_state_machine.dot`](assets/offboard_fsm_state_machine.dot).

| State | Behavior |
|---|---|
| `INIT` | Streams a PX4 setpoint and waits for a takeoff request plus valid local position. |
| `TAKEOFF` | Repeatedly requests PX4 OFFBOARD and arm, then climbs directly to the configured height before entering `IDLE`. |
| `IDLE` | Holds the captured PX4 pose and dispatches one queued ENU goal to EGO-Planner. |
| `MOVE` | Forwards fresh EGO `PositionCommand` messages to PX4. It returns to `IDLE` after EGO executes the dispatched goal and reports `WAIT_TARGET`. |
| `LAND` | Requests PX4 native LAND, then waits for PX4 touchdown/disarm before returning to `INIT`. |

```bash
ros2 service call /offboard/takeoff std_srvs/srv/Trigger "{}"
ros2 service call /offboard/land std_srvs/srv/Trigger "{}"
```

Landing interrupts `ARMING`, `TAKEOFF`, `IDLE`, and `MOVE`.

## Navigation and queue interface

Use `/waypoint_buffer` for algorithmic routes. It accepts the entire ordered
batch before the offboard FSM executes it. `/waypoint_buffer/clear` is an abort
action: it removes the active and every queued target, then holds the vehicle.

| Endpoint | Type | Description |
|---|---|---|
| `/waypoint_buffer` | `offboard_fsm/srv/QueueWaypoints` | Queue `geometry_msgs/msg/PoseStamped[]`; the response reports acceptance and count. |
| `/waypoint_buffer/clear` | `offboard_fsm/srv/ClearWaypoints` | Clear active and pending waypoints; response reports count removed. |
| `/waypoint_buffer/status` | `nav_msgs/msg/Path` | Reliable transient-local snapshot: active waypoint first, followed by pending waypoints. |
| `/offboard/takeoff` | `std_srvs/srv/Trigger` | Accept a latched normal takeoff request only from `INIT`; arming remains asynchronous. |
| `/offboard/land` | `std_srvs/srv/Trigger` | Accept native PX4 landing; touchdown/disarm remain asynchronous. |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | Internal handoff from offboard FSM to EGO-Planner. |
| `/ego_planner/state` | `std_msgs/msg/String` | Reliable EGO lifecycle state used to detect goal completion or errors. |
| `/ego_planner/position_cmd` | `quadrotor_msgs/msg/PositionCommand` | EGO ENU trajectory forwarded to PX4 after ENU-to-NED conversion. |

### Frame convention

Navigation waypoints and EGO goals use the `world` ENU frame: x east, y
north, z up; ROS yaw zero faces east and positive rotation is
counter-clockwise. `offboard_fsm` converts ENU position and yaw to PX4 NED at
its output boundary.

Queued `PoseStamped` messages retain their supplied altitude. EGO manual goals
also retain the requested z coordinate.

For batch navigation and ENU/NED conversion, use [`NavigateSkill`](../skills/README.md).
For coverage planning plus explicit queueing, use `SearchSkill`. Direct use of
EGO's manual-goal topic bypasses queue and flight-state safety checks.

## Planning and feedback

| Topic | Type | Description |
|---|---|---|
| `/gz/odom_super` | `nav_msgs/msg/Odometry` | PX4 local odometry converted from NED to ENU for EGO-Planner. |
| `/gz/point_cloud_super` | `sensor_msgs/msg/PointCloud2` | World-frame LiDAR cloud used by EGO's direct point-cloud grid. |
| `/ego_planner/position_cmd` | `quadrotor_msgs/msg/PositionCommand` | EGO position, velocity, acceleration, yaw, and yaw-rate command. |
| `/ego_planner/state` | `std_msgs/msg/String` | EGO high-level state. |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | Raw PX4 NED local position. |
| `/fmu/out/vehicle_status_v4` | `px4_msgs/msg/VehicleStatus` | PX4 arming and navigation state. |

### Persistent rolling EGO map

The workspace EGO launch enables `grid_map/direct_cloud_mode`. It maintains a
world-aligned, horizontally rolling log-odds grid from `/gz/point_cloud_super`
and `/gz/odom_super`; it does not clear the map for every cloud frame. Hit
endpoints increase occupancy evidence, ray traversals clear free space, and a
raw obstacle is inflated only when its occupancy crosses the configured
threshold. With the workspace profile, hit/miss probabilities are `0.70` and
`0.35`, the clamp range is `[0.12, 0.97]`, the occupied threshold is `0.80`,
the map recentres every 10 m, and untouched obstacle evidence expires after
30 s.

The raw and inflated retained windows publish in `world` on
`/grid_map/occupancy` and `/grid_map/occupancy_inflate`. RViz uses the latter.
The cloud and odometry must remain in the same stable `world` frame; reset or
restart navigation after a localization-frame reset instead of retaining map
evidence across that change.

EGO's packaged depth-demo launch keeps depth fusion by default. Its direct
profile is opt-in:

```bash
ros2 launch ego_planner advanced_param.launch.py \
  direct_cloud_mode:=true \
  cloud_topic:=/gz/point_cloud_super \
  odometry_topic:=/gz/odom_super
```

Record and inspect a route with flight monitor:

```bash
ros2 launch flight_monitor record.launch.py
ros2 run flight_monitor plot_csv
```

The recorder starts on the first goal and writes `cmd_log/goal_<NNN>_<timestamp>.csv`.

## Module relationships

![Module dependency graph](assets/module_dependency_graph.png)

The diagram source is [`assets/module_dependency_graph.dot`](assets/module_dependency_graph.dot).

- `gz_sensor_interface` provides Gazebo LiDAR and odometry transformations.
- `offboard_fsm` provides state management, queue services, EGO-goal bridge,
  and the ENU-to-PX4-NED control boundary.
- EGO-Planner plans local trajectories using the sensor interface's world cloud
  and odometry.
- `coverage_planner` is independent of flight execution and returns sparse ENU
  routes through its service.
- `visualization`, `flight_monitor`, and `benchmark` are optional tools.

The visualization `world` frame is anchored at the drone launch origin, as is
the PX4 ENU origin. The ground-truth path is shifted by the spawn offset so it
aligns with LiDAR and sensor-interface output.

## Related references

- [Combined coverage-planner and offboard launcher](../src/launch/README.md)
- [Coverage planner action and map configuration](../src/search/uav-coverage-route-planner/README.md)
- [GUI controls and map behavior](../gui/README.md)
- [Skills API](../skills/README.md)
- [Architectural UML and skill dependency graphs](uml.md)
