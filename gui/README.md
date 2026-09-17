# Skills test GUI

This plain Tkinter application tests the ROS-backed workspace skills. It is not a colcon package.

## Skill-interface tabs

The GUI uses a plug-in tab contract in [`skill_interfaces/base.py`](skill_interfaces/base.py).
Each `SkillInterface` owns its controls, validation, ROS request dispatch, and
persistent-map click/overlay behavior; the application supplies only shared
connection settings, worker/status handling, telemetry, cameras, and the map.

`NavigateSkillInterface` and `CoverageSearchSkillInterface` live in separate
files under [`skill_interfaces/`](skill_interfaces/) and are the defaults. To
add another skill view, implement `SkillInterface` in its own file and pass its
class alongside the defaults when constructing `SkillsTestGui`; no changes to
the shared map or camera layout are required.

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
export PYTHONPATH=/home/windshape/YunguProject:$PYTHONPATH
/usr/bin/python3 /home/windshape/YunguProject/gui/skills_gui.py
```

Start the coverage planner and offboard FSM before using service/action controls. The GUI reports an error
without freezing if a configured service is unavailable.

The connection panel requires the navigation configuration directory
(`src/navigation/config/offboard`) and planner JSON (`src/search/config/yungu_planner.json`).
The GUI loads their validated `SkillRuntimeConfig` before each flight or skill action; this supplies
the frame ID, planner action, waypoint queue/clear services, and takeoff/land services. Camera,
vehicle-odometry, queue-status, and timeout settings remain independently editable.

- **Navigate** accepts one `x, y, z, heading_deg` waypoint per line. Select ENU or NED; the
  existing `NavigateSkill` performs the conversion and queues the full route. Select the Navigate
  tab, then use the persistent operations map to click an ENU position; review the
  editable altitude (default 5 m) and heading (default 0°), then use **Send selected goal**.
  The heading is ROS ENU yaw (0° east, counter-clockwise positive).
  **Send goals to** chooses which robots that goal reaches — the UAV, any ground agent, or
  several at once with **All**. One click can therefore dispatch the whole fleet: the UAV flies
  to the point at the given altitude and heading, and each checked agent walks to the same x/y.
- **Clear route** calls the clear service, aborting the current route and removing queued waypoints.
- **Plan only** calls `PlanSearchPrimitive` using four ENU search corners and displays its path
  without publishing planner waypoint or marker topics.
- **Plan and queue** calls `SearchSkill`, publishes the planner visualization, then displays the
  route after it was accepted by the offboard queue service.
- **Take off** and **Land** call the configured `std_srvs/srv/Trigger` services only after a
  confirmation dialog. A success response means the offboard FSM accepted the request; it does
  not mean that the vehicle has already taken off, landed, or disarmed.
- **Camera feeds** are a persistent sidebar, so they remain visible while navigating or planning.
  There is one feed per robot: the UAV's front camera (`/swan_gamma_v2/front_camera/image`) and
  follow camera (`/swan_gamma_v2/follow_camera/image`), plus the front camera each ground agent
  carries, taken from the agents config rather than typed in. Each has its own ROS
  subscriber/executor, and the GUI lays them out two to a row in aspect-preserving 320×240
  previews. Use **Start / reconnect feeds** after changing a UAV topic; preview work does not
  block planner or waypoint service actions. All simulated feeds require `ros-humble-ros-gz-image`
  and the normal bridge.
- **Ground agents** are optional. If the agents config cannot be loaded — the package is not
  built, say — the GUI runs as a complete UAV client and the Navigate tab reports the reason
  instead of offering agent targets.

## Coverage map selection

The persistent operations map defaults to
`src/search/config/yungu_map.json`. It renders the map origin and occupied footprints directly
from the planner-compatible JSON; no image file or additional Python package is required.

Use **Browse…** to select another map JSON and **Reload** after editing one. The selected map is a
visual aid only: it does not reconfigure the running coverage planner. Select the same map used by
the planner's startup configuration to avoid planning against a different obstacle layout.

The selected tab controls click behavior without hiding the map. On **Navigate**, one click selects
an ENU goal. On **Coverage Search**, two opposite clicks create an axis-aligned ENU rectangle; the
GUI fills the service corners in southwest, southeast, northeast, northwest order. You can still
edit all values manually. **Reset selection** clears only the search rectangle. Successful Plan
only and Plan and queue requests overlay their returned sparse waypoint route in blue.

The operations map remains visible across tab changes and also shows the live vehicle pose from
`/gz/ground_truth/odom` (black heading arrow) and the authoritative offboard route from
`/waypoint_buffer/status`: orange is the active target and purple points are pending waypoints.
Both topics are editable in the connection settings. The vehicle source must already be ENU and
aligned with the selected map; the GUI does not transform frames. The map remains visual only: it
does not check clicked routes for collision or alter the map used by the running planner.

Run the non-graphical import check with:

```bash
/usr/bin/python3 /home/windshape/YunguProject/gui/skills_gui.py --check
```
