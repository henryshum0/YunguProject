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
| [`src/detection/config/detection.yaml`](../src/detection/config/detection.yaml) | Detection contract shared by the mock and a real detector | `world.*`, `camera.*`, `topics.*`, `localizer.*` |
| [`src/detection/config/mock_detector.yaml`](../src/detection/config/mock_detector.yaml) | Simulation-only mock detector and its error model | `rate_hz`, `seed`, `visibility.*`, `error_model.*`, `overlay.*` |
| [`src/detection/config/targets.yaml`](../src/detection/config/targets.yaml) | Simulated ground-truth targets and the Gazebo spawn offset | `world_origin_in_gz`, `models`, `targets` |

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

### Bringing the system up

| Command | Starts |
|---|---|
| [`./utils/start_all.sh`](../utils/start_all.sh) | Simulator and vehicle stack in one terminal; `Ctrl+C` stops both. `--gui` adds the operator GUI, `--headless` drops the Gazebo GUI, and `name:=value` arguments pass through to the stack launch. |
| [`src/launch/yungu_stack.launch.py`](../src/launch/yungu_stack.launch.py) | The vehicle stack: sensor bridging, navigation, coverage planning, detection. Each layer is an argument (`sensors`, `navigation`, `detection`, `visualization`, `mock_detector`, `spawn_targets`, `use_fastlio`, `coverage_config`). |
| `./utils/start_sim.sh` | The simulator alone: Gazebo, PX4 SITL, the uXRCE agent, and the Gazebo bridges. |

The individual layer launches still work on their own; the stack launch only
includes them. Startup order does not matter — ROS subscriptions bind late, and
the detection target spawner waits (`spawn_wait`) for the Gazebo world.

Note for anyone adding arguments to the stack launch: launch configurations are
inherited by included descriptions, so a name must not collide with one an
included launch already declares. `offboard.launch.py` uses `planner_config` for
the SUPER trajectory-planner config, which is why the coverage planner JSON is
`coverage_config` here.

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
For coverage planning plus explicit queueing, use `SearchSkill.plan_and_queue`; for the complete
search + navigation + detection mission, use `SearchSkill.call`, the
[`Search mission` GUI tab](../gui/README.md), or
[`run_search_detection_demo.py`](../run_search_detection_demo.py).

## Detection

The detection layer publishes what the camera sees and where that is on the map.
In simulation the detector is `mock_detector`, which projects the ground-truth
targets in `targets.yaml` through the camera model and applies a calibrated error
model; `target_localizer` then places each box on the ground plane, and
`detection_overlay` draws the boxes onto the camera stream as a debug view. All
three are started by one launcher. See [the detection reference](../src/detection/README.md)
for the design, the interface a real detector must meet, and known limitations.

```bash
ros2 launch detection detection.launch.py                  # targets + mock + localizer
ros2 launch detection detection.launch.py spawn:=false     # targets are already in the world
ros2 launch detection detection.launch.py mock:=false      # a real detector publishes instead
ros2 launch detection detection.launch.py overlay:=false   # skip the debug overlay
ros2 run detection spawn_targets --detection-config src/detection/config/detection.yaml \
    --targets-config src/detection/config/targets.yaml \
    --models-dir src/detection/detection/models --remove
```

| Topic | Type | Description |
|---|---|---|
| `/detection/detections` | `vision_msgs/msg/Detection2DArray` | Image-plane boxes in the camera frame, stamped with the frame's capture time. |
| `/detection/detections_world` | `vision_msgs/msg/Detection3DArray` | The same detections on the ground plane in the `map` ENU frame; `id` joins them to the boxes above. |
| `/detection/markers` | `visualization_msgs/msg/MarkerArray` | RViz markers for world detections. |
| `/detection/image_overlay` | `sensor_msgs/msg/Image` | Debug view from `detection_overlay`: the front-camera frame with boxes drawn on it. The GUI's **Show detection boxes** button switches the camera preview to it. |

World-frame detections, and the target positions a search reports, are
**estimates**: a detection box back-projected onto the ground plane. In
simulation `detection.truth.match_to_truth` pairs an estimate with the true
target for reading a result; the detection path itself never uses it.

Detection runs entirely on the wall clock, taking its vehicle pose from
`/gz/odom_super` (already ENU in the launch-origin frame and already restamped
with `now()` by `super_lidar`). Bridged Gazebo topics carry the simulation clock
instead, which is why the pose does not come from `/gz/ground_truth/odom`.

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
- `detection` supplies the simulation mock detector and the target localizer;
  the localizer and the configuration are reused unchanged with a real detector.
- `visualization`, `flight_monitor`, and `benchmark` are optional tools.

The visualization `world` frame is anchored at the drone launch origin, as are
FAST-LIO `camera_init` and the PX4 ENU origin. The ground-truth path is shifted
by the spawn offset so it aligns with LiDAR and sensor-interface output.

## Related references

- [Combined coverage-planner and offboard launcher](../src/launch/README.md)
- [Coverage planner service and map configuration](../src/search/uav-coverage-route-planner/README.md)
- [Detection layer, mock detector, and target localizer](../src/detection/README.md)
- [GUI controls and map behavior](../gui/README.md)
- [Skills API](../skills/README.md)
- [Architectural UML and skill dependency graphs](uml.md)
