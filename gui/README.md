# Skills test GUI

This plain Tkinter application tests the ROS-backed workspace skills. It is not a colcon package.

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
the frame ID, planner action, waypoint queue/clear services, and takeoff/land topics. Camera,
vehicle-odometry, queue-status, and timeout settings remain independently editable.

- **Navigate** accepts one `x, y, z, heading_deg` waypoint per line. Select ENU or NED; the
  existing `NavigateSkill` performs the conversion and queues the full route. Select the Navigate
  tab, then use the persistent operations map to click an ENU position; review the
  editable altitude (default 5 m) and heading (default 0°), then use **Queue selected goal**.
  The heading is ROS ENU yaw (0° east, counter-clockwise positive).
- **Clear route** calls the clear service, aborting the current route and removing queued waypoints.
- **Plan only** calls `PlanSearchPrimitive` using four ENU search corners and displays its path
  without publishing planner waypoint or marker topics.
- **Plan and queue** calls `SearchSkill`, publishes the planner visualization, then displays the
  route after it was accepted by the offboard queue service.
- **Take off** and **Land** publish the existing `Bool(data=True)` commands only after a confirmation
  dialog.
- **Camera feeds** are a persistent sidebar, so they remain visible while navigating or planning.
  The front camera defaults to `/swan_gamma_v2/front_camera/image`; the follow camera defaults to
  `/swan_gamma_v2/follow_camera/image`. Each has its own ROS subscriber/executor, while the GUI
  stacks both in an aspect-preserving native-resolution 640×480 preview. Use **Start / reconnect feeds** after
  changing either topic; preview work does not block planner or waypoint service actions. Both
  simulated feeds require `ros-humble-ros-gz-image` and the normal `utils/start_sim.sh` bridge.

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
