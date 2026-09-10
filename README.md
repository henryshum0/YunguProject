# YunguProject

YunguProject is a ROS 2 Humble autonomy workspace for a simulated PX4 UAV. It
combines Gazebo and PX4 SITL, LiDAR localization, obstacle-aware trajectory
planning, coverage-route planning, and an operator GUI. The default
environment is the Yungu map and `swan_gamma_v2` vehicle.

## How the stack fits together

The workspace is layered so that operators and mission applications use the
top layers, while the ROS packages remain independently usable for development
and integration.

```text
Skills GUI (gui.py)
        │ take off / land, click-to-navigate, click-to-search
        ▼
Python skill interfaces (skills/)
        │ NavigateSkill / SearchSkill, ENU/NED conversion, readiness checks
        ▼
ROS interfaces
        │ PlanCoverage ──► sparse ENU Path ──► QueueWaypoints
        ▼
Search                 Navigation                         Simulation
coverage_planner  →  offboard_fsm → SUPER → PX4    ←  Gazebo / sensor bridge
                         ▲        ▲
                    FAST-LIO   LiDAR/odometry
```

A coverage request is planned by `coverage_planner`, returned as a sparse ENU
`Path`, accepted atomically by `offboard_fsm`, then flown by SUPER and PX4
offboard control. Planning does **not** queue or publish a route by itself;
`SearchSkill` or the GUI's **Plan and queue** action performs that explicit
second step.

## Workspace layout

| Location | Responsibility |
|---|---|
| [`gui/`](gui/) and [`gui.py`](gui.py) | Tkinter operator/test GUI: flight controls, live camera and operations map, navigation, and coverage search. |
| [`skills/`](skills/) | Plain Python ROS client interfaces: `NavigateSkill`, `SearchSkill`, and their primitives. |
| [`src/navigation/`](src/navigation/) | Flight execution: `offboard_fsm`, SUPER, FAST-LIO, Livox driver, PX4 messages, and navigation tools. |
| [`src/search/`](src/search/) | `coverage_planner` plus independent Yungu map and planner configuration. |
| [`src/simulation/`](src/simulation/) | Gazebo-facing sensor interface and simulation configuration. |
| [`src/launch/`](src/launch/) | Cross-package launch orchestration, including coverage planner plus offboard FSM. |
| [`utils/`](utils/) | Simulation lifecycle and bridge helpers, including STL-to-planner-map conversion. |
| [`VisionFlow-PX4/`](VisionFlow-PX4/) | PX4 fork/submodule, Gazebo worlds, and vehicle models. |

## First successful run

### 1. Build the workspace

Ubuntu 22.04 with ROS 2 Humble is the supported platform. Clone recursively so
the PX4 submodule is available. Place `yungu.glb` under
`VisionFlow-PX4/Tools/simulation/gz/worlds` before launching the Yungu world.

```bash
git clone --recursive https://github.com/henryshum0/YunguProject.git
cd YunguProject
./install_deps.sh
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2. Start the runtime layers

Use separate terminals, sourcing ROS and the workspace overlay in each one.

```bash
source /opt/ros/humble/setup.bash
source /path/to/YunguProject/install/setup.bash
```

Replace `/path/to/YunguProject` with this workspace path, then run one of the
following commands in each prepared terminal:

```bash
# Terminal 1: Gazebo, PX4 SITL, MicroXRCE agent, and Gazebo bridges.
./utils/start_sim.sh

# Terminal 2: LiDAR, IMU, and ground-truth sensor bridging.
ros2 launch gz_sensor_interface sensor_sensors.launch.py

# Terminal 3: offboard FSM, SUPER, and the coverage-planning service.
ros2 launch "$PWD/src/launch/coverage_and_offboard.launch.py"

# Terminal 4 (optional): RViz and birdview tools.
ros2 launch visualization visualization.launch.py

# Terminal 5: operator GUI.
python3 gui.py
```

The combined launcher starts `offboard_fsm` and `coverage_planner` but does not
create a coverage plan. In the GUI, choose a rectangle on the map and use
**Plan only** to preview it or **Plan and queue** to submit it for flight. The
GUI's selected map is display-only; choose the same map used by the planner
configuration when testing a route.

Stop with `Ctrl+C`. If a Gazebo process remains, run `./utils/stop_sim.sh`.
Simulation logs are written to `/tmp/yungu_sim/`.

## Use the top-level interfaces

### GUI

Run `python3 gui.py` from the workspace root after a successful build. It
loads the ROS environment automatically and provides:

- confirmed takeoff and land actions;
- ENU/NED waypoint entry and click-to-navigate selection;
- two-click coverage-rectangle planning and optional route queueing;
- a persistent Yungu map with vehicle pose and authoritative waypoint queue;
- a persistent live front-camera preview.

The GUI is an operator/test client only: it does not replace planning,
collision checking, or offboard control. See [`gui/README.md`](gui/README.md)
for controls, map behavior, and camera requirements.

### Python skills

`skills/` is a normal Python package, not a colcon package. It connects to ROS
nodes already running in the background and checks service readiness before
sending a request.

- `NavigateSkill` accepts one or more `(x, y, z, heading_deg)` waypoints in
  ENU or NED, converts them to ENU `PoseStamped` messages, and queues them.
- `SearchSkill` sends four ENU coverage corners to the planner, queues the
  returned path only after planning succeeds, and returns that path.
- Lower-level primitives are available when an application needs planning,
  queueing, or clearing separately.

Both coverage paths and ENU navigation waypoints use ROS ENU orientation:
`x=east`, `y=north`, `z=up`, with yaw zero facing east and positive rotation
counter-clockwise. Read complete API examples in
[`skills/README.md`](skills/README.md).

### ROS mission interfaces

| Endpoint | Type | Purpose |
|---|---|---|
| `/coverage_planner/plan_coverage` | `coverage_planner/srv/PlanCoverage` | Plan four ENU rectangle corners and return a sparse route. |
| `/waypoint_buffer` | `offboard_fsm/srv/QueueWaypoints` | Atomically append an ordered waypoint batch. |
| `/waypoint_buffer/clear` | `offboard_fsm/srv/ClearWaypoints` | Abort the active target and remove pending waypoints. |
| `/waypoint_buffer/status` | `nav_msgs/msg/Path` | Latched queue snapshot: active target, then pending targets. |
| `/takeoff_cmd` | `std_msgs/msg/Bool` | Begin offboard arming and takeoff when the FSM is ready. |
| `/land_cmd` | `std_msgs/msg/Bool` | Interrupt navigation and land. |
| `/waypoint_pose` | `geometry_msgs/msg/PoseStamped` | RViz/manual single-goal input, bridged to the queue service. |

For the state-machine lifecycle, direct ROS examples, feedback topics, and
failure behavior, use the [operations reference](docs/operations.md).

## Package responsibilities

### Simulation and perception

- `gz_sensor_interface` converts and relays Gazebo LiDAR, IMU, and odometry.
- `FAST_LIO` supplies LiDAR-inertial odometry; `livox_ros_driver2` supports the
  physical LiDAR path.
- `VisionFlow-PX4` supplies PX4 SITL, the Yungu Gazebo world, vehicle models,
  and the bridged front-camera image stream.

### Navigation and execution

- `offboard_fsm` owns arming, takeoff, landing, queue services, and the
  ENU-to-PX4-NED boundary.
- SUPER (`super_planner`, `rog_map`, and supporting packages) plans local safe
  trajectories from each active waypoint.
- `px4_msgs` and `mars_quadrotor_msgs` provide ROS message definitions.

### Search and operator tooling

- `coverage_planner` plans obstacle-aware single-UAV coverage routes from the
  configured map and a requested search rectangle. Its package README covers
  the service and JSON schema: [`src/search/uav-coverage-route-planner/README.md`](src/search/uav-coverage-route-planner/README.md).
- `visualization`, `flight_monitor`, and `benchmark` provide RViz/birdview,
  recording, and planner-evaluation utilities.
- [`src/launch/README.md`](src/launch/README.md) documents the combined
  coverage-planner/offboard launcher.

## Configuration and further documentation

Configuration is grouped by owner:

- [`src/simulation/config/`](src/simulation/config/) for Gazebo, bridge,
  sensor, and visualization settings;
- [`src/navigation/config/offboard/`](src/navigation/config/offboard/) for
  offboard FSM, shared topic names, and SUPER settings;
- [`src/search/config/`](src/search/config/) for planner mission settings and
  reusable map geometry.

Configuration changes apply on the next launch without rebuilding. The
[operations reference](docs/operations.md) lists important files and launch
overrides. For package-specific integration details, use the
[skills API](skills/README.md), [combined-launch guide](src/launch/README.md),
and [coverage-planner guide](src/search/uav-coverage-route-planner/README.md).
The [architectural UML and skill dependency graphs](docs/uml.md) show the
project-owned command path and its ROS boundaries.
