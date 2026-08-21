
# YunguProject

ROS 2 (Humble) drone autonomy stack for the Yungu flight test: Gazebo + PX4
SITL simulation, the SUPER trajectory planner, optional FAST-LIO localization
(mirrors real hardware) and a PX4 offboard state machine with waypoint
following. Any external search/coverage planner can command the drone by
publishing waypoints on `/waypoint_pose` — no launch or code changes. Once the
system is up you take off with a `/takeoff_cmd` message (direct PX4 climb);
navigation between waypoints is planner-driven (SUPER), with automatic
planner-failure recovery and direct PX4 landing.

## Prerequisites

- Ubuntu 22.04 with **ROS 2 Humble** (gz-sim 8 / Harmonic)
- The PX4 fork is a git submodule: clone with `--recursive`
- Before running the Yungu map, put `yungu.glb` under
  `VisionFlow-PX4/Tools/simulation/gz/worlds`

## Setup

```bash
git clone --recursive https://github.com/henryshum0/YunguProject.git
cd YunguProject
./install_deps.sh                  # system + ROS + Python deps (idempotent)
colcon build --symlink-install
source install/setup.bash
```

## Running

The stack runs in two layers (two terminals):

```bash
# Terminal 1 — simulation stack: Gazebo + PX4 SITL + MicroXRCEAgent + gz bridge
./utils/start_sim.sh

# Terminal 2 — perception + planning + offboard + RViz
#   FAST-LIO + lidar_merge are launched by this file when enabled in
#   config/offboard.yaml (use_fastlio / use_lidar_merge).
ros2 launch offboard offboard.launch.py
```

Stop with `Ctrl+C`; if anything lingers (e.g. `gz-server` detached into its
own session), run `./utils/stop_sim.sh`. Logs go to `/tmp/yungu_sim/`.

### Per-run overrides

Every config value can be overridden without editing the file:

| Variable / arg | Default | Description |
|---|---|---|
| `PX4_MODEL`, `PX4_WORLD` | from `config/simulation.yaml` | Gazebo airframe + world (`PX4_MODEL=gz_<model>_<world>` legacy form also accepted) |
| `XRCE_PORT` | `8888` | uXRCE-DDS port for the MicroXRCEAgent |
| `GZ_VERSION` | `harmonic` | gz-transport version for `ros_gz_bridge` |
| `HEADLESS=1` | *(unset)* | Run Gazebo without its GUI (server only) |
| `rviz:=false`, `rviz_freelook:=false` | `true` | Toggle the two RViz windows in `offboard.launch.py` |
| `use_fastlio:=...`, `use_lidar_merge:=...` | `true` | Toggle the FAST-LIO layer / lidar_merge in `offboard.launch.py` (same keys as `config/offboard.yaml`) |

```bash
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./utils/start_sim.sh
HEADLESS=1 ./utils/start_sim.sh
ros2 launch offboard offboard.launch.py rviz:=false
ros2 launch offboard offboard.launch.py use_fastlio:=false   # no FAST-LIO layer
```

`offboard.launch.py` also launches the birdview overlay + two RViz windows
(top-down birdview planning view, free-rotate 3D debug view).

## Configuration

All run-time config lives in [`config/`](config/) and is read via
[`config/sim_config.py`](config/sim_config.py) — edits take effect on the next
launch (no rebuild).

| File | Purpose | Key keys |
|---|---|---|
| [`config/simulation.yaml`](config/simulation.yaml) | Sim: model, world, gz version, uXRCE port, GZ→ROS bridge topics | `model`, `world`, `gz_version`, `xrce_port`, `bridge.*` |
| [`config/offboard.yaml`](config/offboard.yaml) | Offboard state machine + planner + waypoint following + FAST-LIO/lidar_merge switches | `visualization`, `update_rate`, `planner_cmd_hz`, `default_height`, `goal_height`, `planner_config`, `cloud_in_topic`, `use_fastlio`, `use_lidar_merge`, `fastlio_config`, `waypoint_reached_dist`, `waypoint_hold_time`, `arm_retry_delay`, `arm_retry_max`, `planner_fail_retry_max`, `planner_reset_delay`, `planner_stall_timeout`, `takeoff_vel`, `landing_vel`, `yaw_align_thresh` |
| [`config/super_planner/`](config/super_planner/) | SUPER planner behaviour (A*, traj opt, ROG-Map) | `fsm.click_height`, `super_planner.*`, `traj_opt.*`, `astar.*`, `rog_map.*` |
| [`config/birdview.yaml`](config/birdview.yaml) | Aerial birdview overlay | `extent_*`, `offset_*`, `yaw`, `max_points` |

Set `offboard.visualization: false` for a fully headless run (no RViz, no
birdview overlay, SUPER markers off). `goal_height` overrides SUPER's
`fsm.click_height` for the RViz "2D Goal Pose" tool.

## Flight states & takeoff/landing

The `offboard_node` runs a planner-driven state machine. On start it waits in
`INIT` until odometry, FAST-LIO and the planner are all healthy and the
vehicle is on the ground and disarmed; it then switches PX4 to **OFFBOARD**
mode and waits for a takeoff command.

```mermaid
stateDiagram-v2
    [*] --> INIT

    INIT --> ARMING : takeoff_cmd
    INIT --> EMERGENCY_STOP : critical_failure (LIO error / planner fail)

    ARMING --> TAKEOFF : armed
    ARMING --> INIT : heartbeat_failed
    ARMING --> INIT : arm_retry_exhausted (3 attempts, 5 s apart)

    TAKEOFF --> IDLE : reached default_height

    IDLE --> MOVE : buffered_goal &amp; can_move_
    IDLE --> PLANNER_FAIL : planner_error
    IDLE --> LAND : land_cmd

    MOVE --> IDLE : goal_reached / paused
    MOVE --> PLANNER_FAIL : planner_error (2 s FAIL flag or stall)
    MOVE --> LAND : land_cmd

    PLANNER_FAIL --> MOVE : planner_reset_ok
    PLANNER_FAIL --> FAILSAFE : 3_consecutive_failures

    FAILSAFE --> INIT : disarm_complete (direct PX4 land)

    LAND --> INIT : disarm_complete (direct PX4 land)

    EMERGENCY_STOP --> INIT : manual_recovery

    note right of LAND : /land_cmd is honoured in every state\nexcept INIT / PLANNER_FAIL / FAILSAFE / LAND
```

| State | Behaviour |
|---|---|
| `INIT` | Verifies odometry / FAST-LIO (`fastlio/lio_state.running`) / planner (`fsm/planner_state`) readiness, sets OFFBOARD, waits for `/takeoff_cmd`. Critical failure → `EMERGENCY_STOP`. |
| `ARMING` | Arms, retrying every `arm_retry_delay` (5 s) up to `arm_retry_max` (3) times; on exhaustion returns to `INIT`. |
| `TAKEOFF` | Direct PX4 vertical climb (no planner) to `default_height` at `takeoff_vel`; on reaching altitude → `IDLE`. |
| `IDLE` | Hovers, waiting for goals. With a buffered goal and the heading pointed toward it (`can_move_`), publishes it and moves to `MOVE`. |
| `MOVE` | Forwards SUPER's `PositionCommand` to PX4. Planner command-rate drop → `IDLE`; planner failure → `PLANNER_FAIL`; goal reached → `IDLE`; `/land_cmd` → `LAND`. |
| `PLANNER_FAIL` | Calls the planner reset service (`/fsm_node/reset`) and re-publishes the current goal. After `planner_fail_retry_max` (3) consecutive failures → `FAILSAFE`. |
| `FAILSAFE` | Lands with direct PX4 commands (no planner), then returns to `INIT`. |
| `LAND` | Direct PX4 landing (no planner) to `landing_z` at `landing_vel`, then disarms and returns to `INIT`. |
| `EMERGENCY_STOP` | Holds position on a critical failure; recovers to `INIT` on a manual recovery signal. |

Take off / land at any time:

```bash
# Take off (only accepted once the system is ready in INIT)
ros2 topic pub --once /takeoff_cmd std_msgs/msg/Bool "{data: true}"

# Land (interrupts any flight except INIT / PLANNER_FAIL / FAILSAFE)
ros2 topic pub --once /land_cmd std_msgs/msg/Bool "{data: true}"
```

## Point-to-point navigation (interface for search algorithms)

This is the **only** interface a search/planning algorithm needs. Once the
stack is up, send a `/takeoff_cmd` to take off; the drone climbs directly to
`default_height` and enters `IDLE`. Then command it by publishing a
`geometry_msgs/PoseStamped`.

### Inputs

| Topic | Type | Purpose |
|---|---|---|
| `/waypoint_pose` | `PoseStamped` | **Batch waypoint input (recommended).** Queued in the offboard waypoint buffer and flown one at a time. RViz's "2D Goal Pose" tool is re-targeted here. |
| `/goal_pose` | `PoseStamped` | **Direct single goal.** Also the internal channel offboard uses to hand the current navigation waypoint to SUPER. |
| `/takeoff_cmd` | `std_msgs/Bool` | **Take off** once the system is ready (`true`). The drone arms and climbs planner-driven to `default_height`. |
| `/land_cmd` | `std_msgs/Bool` | **Land** (`true`). Interrupts any flight and lands planner-driven. Ignored in `INIT` / `PLANNER_FAIL` / `FAILSAFE`. |
| `/waypoint_markers` | `MarkerArray` | Feedback: green = queued, yellow = currently pursued, cyan line = route. |

> QoS caveat: `/waypoint_pose` is `best_effort`/`keep_last(1)` — publish at
> **≥ 0.5 s intervals** and confirm acceptance via the offboard log
> (`Waypoint buffered (#N)`) or `/waypoint_markers`.

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
        msg.header.frame_id = "world"                 # ENU world frame, z up
        msg.pose.position.x, msg.pose.position.y = 10.0, 5.0   # east, north
        msg.pose.position.z = 5.0                     # up [m]
        msg.pose.orientation.w = 1.0                  # yaw follows flight direction
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(GoalPublisher())
```

### Feedback

| Topic | Type | Description |
|---|---|---|
| `/lidar_slam/odom` | `Odometry` | Fused world-frame ENU odometry (FAST-LIO → PX4 EKF2, else Gazebo truth) — the state feedback for your algorithm |
| `/cloud_registered` | `PointCloud2` | World-frame lidar cloud (ROG-Map input) |
| `/planning/pos_cmd` | `PositionCommand` | SUPER's commanded trajectory (pos/vel/acc/yaw/yaw_dot), ~100 Hz |
| `fsm/planner_state` | `super_planner/PlannerState` | High-level planner FSM state (`init` / `wait_goal` / `move` / `fail`) |
| `fastlio/lio_state` | `fast_lio/LioState` | FAST-LIO odometry health (`init` / `running` / `error`) |
| `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | Raw PX4 local position (NED) |
| `/fmu/out/vehicle_status_v4` | `VehicleStatus` | Arming / nav state |

### Frames & waypoint behaviour

- Waypoints, goals and planner output are **ENU world frame** (`frame_id:
  "world"`, x=east, y=north, z=up); ENU→NED conversion (yaw included) happens
  inside the offboard node.
- A waypoint is reached by **horizontal** distance (`waypoint_reached_dist`),
  then held for `waypoint_hold_time` before the next one is handed to SUPER.
- Waypoint parameters double as launch args, e.g.
  `ros2 launch offboard offboard.launch.py waypoint_reached_dist:=2.0
  waypoint_hold_time:=1.0`.

### Landing

The drone lands with a direct PX4 descent (no planner) at `landing_vel` down
to `landing_z`, then disarms. Either command it with the `/land_cmd` topic
(works from any flight state except `INIT` / `PLANNER_FAIL` / `FAILSAFE`), or
use the legacy `~/land` service:

```bash
ros2 topic pub --once /land_cmd std_msgs/msg/Bool "{data: true}"
ros2 service call /offboard/land std_srvs/srv/Trigger   # legacy, same effect
```

### Recording & evaluation

```bash
# Terminal 3 — recorder + live plot (starts on the first goal)
ros2 launch flight_monitor record.launch.py
# Plot a saved segment afterwards (defaults to the newest CSV in cmd_log/)
ros2 run flight_monitor plot_csv
```

Each goal click writes `cmd_log/goal_<NNN>_<timestamp>.csv` (goal position +
commanded trajectory + real odometry).

## Dependency graph

```mermaid
flowchart LR
    subgraph sim["Simulation stack — utils/start_sim.sh"]
        GZ["Gazebo gz-sim"] --> PX4["PX4 SITL<br/>(VisionFlow-PX4)"]
        PX4 <--> XRCE["MicroXRCEAgent"]
        GZ --> BR["ros_gz_bridge<br/>utils/bridge.sh"]
        BR --> TF["tf_bridge"]
    end

    subgraph perc["FAST-LIO + lidar_merge — offboard.launch.py"]
        GZ -->|scan_left/right| LB["lidar_bridge<br/>imu_relay · add_time_field · lidar_merge"]
        LB -->|fused cloud + imu| FL["FAST-LIO<br/>fastlio_mapping"]
        FL -->|/Odometry| FPB["fastlio_px4_bridge"]
        FPB -->|vehicle_visual_odometry| PX4
    end

    subgraph plan["Planning + control — offboard.launch.py"]
        PX4 -->|vehicle_odometry| SB["super_bridge"]
        GZ -->|point cloud| SB
        LB -->|fused cloud| SB
        SB -->|lidar_slam/odom + cloud_registered| FSM["fsm_node<br/>(super_planner)"]
        USER["search / coverage planner"] -->|/waypoint_pose| GM["goal_marker_node"]
        GM -->|/waypoint_buffer| OFF["offboard_node"]
        OFF -->|/goal_pose| FSM
        FSM -->|/planning/pos_cmd| OFF
        OFF -->|TrajectorySetpoint| PX4
        SB --> RViz
        OFF --> RViz
    end
```

| Package | Role |
|---|---|
| `offboard` | offboard_node (state machine), super_bridge, goal_marker_node, fastlio_px4_bridge, birdview_publisher |
| `super_planner` (+ `rog_map`, `mission_planner`) | SUPER planner (`fsm_node`) |
| `lidar_bridge` | imu_relay, add_time_field, lidar_merge, tf_bridge |
| `FAST_LIO` | LiDAR-inertial odometry (`fastlio_mapping`) |
| `flight_monitor` | `cmd_record` recorder + `plot_csv` |
| `px4_msgs`, `mars_quadrotor_msgs` | ROS 2 message definitions |
| `benchmark` | Random gate/pillar obstacle-map generator for planner benchmarks |

Notes:

- `tf_bridge` subscribes `/lidar_slam/odom` with `BEST_EFFORT` to match
  super_bridge's publisher — a reliable subscription never receives anything
  and the `world → base_link` TF goes stale.
- Ground-truth odometry `/odom` comes from the gz model-instance topic
  `/model/swan_gamma_v2_0/odometry` (see `config/simulation.yaml`); it is used
  by the truth path `/gt_path` and `flight_monitor` comparisons.
- FAST-LIO's `fastlio_px4_bridge` feeds PX4 EKF2 external vision
  (`/fmu/in/vehicle_visual_odometry`); the fused
  `/fmu/out/vehicle_odometry` then drives SUPER via `super_bridge`.

## External planner integration

The point-to-point interface above is the execution endpoint for offline
search/coverage planners. A worked example (coverage-search-planner,
`flight_plan.json` → ENU waypoints) is documented in
[`docs/coverage-search-integration.md`](docs/coverage-search-integration.md).
Additional background: [`docs/README_zh.md`](docs/README_zh.md) and
[`docs/utils-fastlio-gz-bridges.md`](docs/utils-fastlio-gz-bridges.md).
