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
| [`src/simulation/config/gz_sensor_interface.yaml`](../src/simulation/config/gz_sensor_interface.yaml) | Gazebo sensor bridge topics, frames, and extrinsics | `lidar_sensor.*`, `imu_bridge.*`, `truth_odom.*`, `super_lidar.*` |
| [`src/simulation/config/visualization.yaml`](../src/simulation/config/visualization.yaml) | TF, birdview, path, point-cloud, and RViz settings | `frames.*`, `visual_tf.*`, `birdview.*`, `rviz.*` |
| [`src/simulation/config/birdview.yaml`](../src/simulation/config/birdview.yaml) | Aerial birdview overlay | `extent_*`, `offset_*`, `yaw`, `max_points` |
| [`src/navigation/config/offboard/topics.yaml`](../src/navigation/config/offboard/topics.yaml) | Shared navigation ROS endpoints | `offboard_fsm.*`, `super.*`, `fastlio.*`, `gz_sensor_interface.*`, `visualization.*` |
| [`src/navigation/config/offboard/offboard_fsm.yaml`](../src/navigation/config/offboard/offboard_fsm.yaml) | Offboard state machine and planner integration | `update_rate`, arming/takeoff/landing settings, queue settings, `goal_height`, planner/FAST-LIO configuration |
| [`src/navigation/config/offboard/super_planner/`](../src/navigation/config/offboard/super_planner/) | SUPER A*, ROG-Map, and trajectory optimization | `fsm.*`, `traj_opt.*`, `astar.*`, `rog_map.*` |
| [`src/search/config/`](../src/search/config/) | Coverage planner JSON and reusable map geometry | `*_planner.json`, `*_map.json` |

Set `offboard.visualization: false` in the offboard configuration for a fully
headless navigation run. Use `use_fastlio:=false` with the combined launcher to
test controller/planner behavior without starting FAST-LIO.

### Per-run simulation overrides

| Variable or argument | Default | Description |
|---|---|---|
| `PX4_MODEL`, `PX4_WORLD` | `simulation.yaml` | Override the Gazebo airframe or world. Legacy `gz_<model>_<world>` form is accepted. |
| `XRCE_PORT` | `8888` | MicroXRCE agent port. |
| `GZ_VERSION` | `harmonic` | Gazebo transport version used by bridges. |
| `HEADLESS=1` | unset | Run Gazebo server without the GUI. |
| `rviz:=false`, `rviz_freelook:=false` | `true` | Disable either RViz window in the visualization launcher. |
| `use_fastlio:=false` | `false` in the combined launcher | Do not start FAST-LIO or its PX4 visual-odometry bridge. |

```bash
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./utils/start_sim.sh
HEADLESS=1 ./utils/start_sim.sh
ros2 launch offboard_fsm offboard.launch.py use_fastlio:=false
ros2 launch visualization visualization.launch.py rviz:=false
```

## Offboard state machine

`offboard_node` owns vehicle flight-state transitions. It waits for healthy
odometry, planner readiness, and (when enabled) FAST-LIO before accepting a
takeoff command.

![Offboard FSM state machine](assets/offboard_fsm_state_machine.png)

The diagram source is [`assets/offboard_fsm_state_machine.dot`](assets/offboard_fsm_state_machine.dot).

| State | Behavior |
|---|---|
| `INIT` | Verifies inputs, selects PX4 OFFBOARD mode, then waits for takeoff. If restarted airborne in OFFBOARD with a healthy planner, it resumes in `IDLE`. |
| `ARMING` | Arms with configured retry behavior; failure returns to `INIT`. |
| `TAKEOFF` | Climbs directly with PX4 control to `default_height`, then enters `IDLE`. |
| `IDLE` | Holds position, maintains SUPER readiness, processes terminal goal status, and hands the next queued waypoint to SUPER when ready. |
| `MOVE` | Forwards SUPER `PositionCommand` output to PX4. Planner terminal status, failure recovery, or waypoint completion returns to `IDLE`. |
| `LAND` | Descends directly to `landing_z`, disarms, and returns to `INIT`. |

```bash
ros2 topic pub --once /takeoff_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /land_cmd std_msgs/msg/Bool "{data: true}"
ros2 service call /offboard/land std_srvs/srv/Trigger
```

Landing interrupts `ARMING`, `TAKEOFF`, `IDLE`, and `MOVE`.

## Navigation and queue interface

Use `/waypoint_buffer` for algorithmic routes. It accepts the entire ordered
batch before the offboard FSM executes it. `/waypoint_buffer/clear` is an abort
action: it removes the active and every queued target, holds the vehicle, and
resets SUPER.

| Endpoint | Type | Description |
|---|---|---|
| `/waypoint_buffer` | `offboard_fsm/srv/QueueWaypoints` | Queue `geometry_msgs/msg/PoseStamped[]`; the response reports acceptance and count. |
| `/waypoint_buffer/clear` | `offboard_fsm/srv/ClearWaypoints` | Clear active and pending waypoints; response reports count removed. |
| `/waypoint_buffer/status` | `nav_msgs/msg/Path` | Reliable transient-local snapshot: active waypoint first, followed by pending waypoints. |
| `/waypoint_pose` | `geometry_msgs/msg/PoseStamped` | Manual/RViz single-goal input, bridged into the queue service. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Internal current-goal handoff from offboard FSM to SUPER. |
| `/waypoint_markers` | `visualization_msgs/msg/MarkerArray` | Queue feedback: green queued, yellow active, cyan route. |

### Frame convention

Navigation waypoints and SUPER goals use the `world` ENU frame: x east, y
north, z up; ROS yaw zero faces east and positive rotation is
counter-clockwise. `offboard_fsm` converts ENU position and yaw to PX4 NED at
its output boundary.

Queued `PoseStamped` messages retain their supplied altitude. The configured
manual-goal height only applies to RViz/`/waypoint_pose` goals created by the
goal-marker bridge.

### Direct manual goal example

`/waypoint_pose` is convenient for one-off RViz or shell tests, but algorithms
should use the queue service or `NavigateSkill`.

```python
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("manual_goal_publisher")
        self.publisher = self.create_publisher(PoseStamped, "/waypoint_pose", 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 10.0, 5.0, 5.0
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.timer.cancel()


rclpy.init()
node = GoalPublisher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

For batch navigation and ENU/NED conversion, use [`NavigateSkill`](../skills/README.md).
For coverage planning plus explicit queueing, use `SearchSkill`.

## Planning and feedback

| Topic | Type | Description |
|---|---|---|
| `/gz/odom_super` | `nav_msgs/msg/Odometry` | PX4 local odometry converted from NED to ENU for SUPER. |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | World-frame LiDAR cloud used by ROG-Map. |
| `/planning/pos_cmd` | `mars_quadrotor_msgs/msg/PositionCommand` | SUPER position, velocity, acceleration, yaw, and yaw-rate command. |
| `fsm/planner_state` | `super_planner/msg/PlannerState` | SUPER high-level state. |
| `fastlio/lio_state` | `fast_lio/msg/LioState` | FAST-LIO health. |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | Raw PX4 NED local position. |
| `/fmu/out/vehicle_status_v4` | `px4_msgs/msg/VehicleStatus` | PX4 arming and navigation state. |

Record and inspect a route with flight monitor:

```bash
ros2 launch flight_monitor record.launch.py
ros2 run flight_monitor plot_csv
```

The recorder starts on the first goal and writes `cmd_log/goal_<NNN>_<timestamp>.csv`.

## Module relationships

![Module dependency graph](assets/module_dependency_graph.png)

The diagram source is [`assets/module_dependency_graph.dot`](assets/module_dependency_graph.dot).

- `gz_sensor_interface` provides Gazebo LiDAR/IMU/odometry transformations.
- `offboard_fsm` provides state management, queue services, manual-goal bridge,
  and FAST-LIO-to-PX4 external-vision forwarding.
- SUPER plans local trajectories; FAST-LIO supplies localization when enabled.
- `coverage_planner` is independent of flight execution and returns sparse ENU
  routes through its service.
- `visualization`, `flight_monitor`, and `benchmark` are optional tools.

The visualization `world` frame is anchored at the drone launch origin, as are
FAST-LIO `camera_init` and the PX4 ENU origin. The ground-truth path is shifted
by the spawn offset so it aligns with LiDAR and sensor-interface output.

## Related references

- [Combined coverage-planner and offboard launcher](../src/launch/README.md)
- [Coverage planner service and map configuration](../src/search/uav-coverage-route-planner/README.md)
- [GUI controls and map behavior](../gui/README.md)
- [Skills API](../skills/README.md)
- [Architectural UML and skill dependency graphs](uml.md)
