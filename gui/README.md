# Skills test GUI

This plain Tkinter application tests the ROS-backed workspace skills. It is not a colcon package.

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
export PYTHONPATH=/home/windshape/YunguProject:$PYTHONPATH
/usr/bin/python3 /home/windshape/YunguProject/gui/skills_gui.py
```

Start the coverage planner and offboard FSM before using service actions. The GUI reports an error
without freezing if a configured service is unavailable.

The connection panel requires the navigation configuration directory
(`src/navigation/config/offboard`) and planner JSON (`src/search/config/yungu_planner.json`), and
takes an optional detection contract (`src/detection/config/detection.yaml`).
The GUI loads their validated `SkillRuntimeConfig` before each flight or skill action; this supplies
the frame ID, planner service, waypoint queue/clear services, takeoff/land topics, and the detection
topics. Camera, vehicle-odometry, queue-status, and timeout settings remain independently editable.

Detection is only needed by the **Search mission** tab. If its configuration cannot be loaded — the
detection package is not built, say — navigation and coverage planning keep working and the mission
reports the exact reason it cannot start.

- **Navigate** accepts one `x, y, z, heading_deg` waypoint per line. Select ENU or NED; the
  existing `NavigateSkill` performs the conversion and queues the full route. Select the Navigate
  tab, then use the persistent operations map to click an ENU position; review the
  editable altitude (default 5 m) and heading (default 0°), then use **Queue selected goal**.
  The heading is ROS ENU yaw (0° east, counter-clockwise positive).
- **Clear route** calls the clear service, aborting the current route and removing queued waypoints.
- **Plan only** calls `PlanSearchPrimitive` using four ENU search corners and displays its path
  without publishing planner waypoint or marker topics.
- **Plan and queue** calls `SearchSkill.plan_and_queue`, publishes the planner visualization, then
  displays the route after it was accepted by the offboard queue service. It returns as soon as the
  route is queued; it does not wait for the flight.
- **Take off** and **Land** publish the existing `Bool(data=True)` commands only after a confirmation
  dialog. They publish directly rather than through the service worker, so **Land stays available
  while a search mission is flying**.
- **Show detection boxes** switches the camera preview between the raw camera stream and the
  detector's overlay (`/detection/image_overlay`, taken from the detection configuration rather than
  typed in), and back again. The overlay is published by `detection_overlay`, so it needs the
  detection layer running.
- **Front camera** is a persistent sidebar, so it remains visible while navigating or planning.
  It subscribes on its own ROS executor and starts automatically with the configured image topic.
  Use **Start / reconnect preview** after changing that topic; the preview does not block planner
  or waypoint service actions. The default simulated camera requires `ros-humble-ros-gz-image`
  and the normal `utils/start_sim.sh` bridge process. The GUI reserves a native 640×480 preview,
  matching the simulated camera output without downsampling.

## Search mission

The **Search mission** tab runs the complete skill — search plans a route, navigation flies it, and
detection watches while it flies — and reports whether what you were looking for was actually found.

It searches the rectangle from the **Coverage Search** tab, so select the area there (two map
clicks) or type the four corners, then:

The shipped simulated targets sit inside `(20, -20)` to `(60, 20)`, which is a good area to select
first: it is free of buildings, plans cleanly at the configured 15 m, and every target in it is
beyond the camera's reach from the launch point, so the mission has to fly to find anything.

| Control | Meaning |
|---|---|
| **Look for** | What to search for: the group names `person` or `vehicle`, or a comma-separated list of detector classes such as `car, van`. The group names cover the classes a detector confuses with one another. A name the detector cannot report is rejected before anything is queued. |
| **Mission timeout (s)** | Ceiling on the flight. A 40x40 m area at 15 m takes roughly six minutes. |
| **Stop as soon as a target is confirmed** | Abort the rest of the route once a target is confirmed ("go find it"). Clear it to always sweep the whole area and report everything found. A target only confirms once the vehicle is within ~20 m of it, so a distant early sighting does not end the mission with a rough fix — the sweep flies on and confirms it up close, then observes a couple more seconds to refine the position. |
| **Run search mission** | Clears the waypoint queue, plans, queues, and flies. Other service buttons are disabled for the duration; Land is not. |
| **Stop mission** | Calls the mission off: the route is aborted and the result comes back with reason `aborted`. Enabled only while a mission runs. |
| **Clear found targets** | Removes the magenta target markers from the map. |
| **Show ground truth on map** | Simulation aid: draws the true target positions as hollow grey diamonds, to read the estimates against. Off by default, so a search looks the way it would without them. |

While the mission runs the tab reports live progress about once a second — elapsed time, waypoints
flown, detector frames seen, and the targets confirmed so far — and each confirmed target appears on
the operations map as a magenta diamond. Estimates visibly settle as more detections average in.

A single detection never counts as a find: a target is confirmed only when several detections of the
same class agree on a world position, which is what keeps a detector's false positives out of the
result. When the mission ends, the result panel shows `success`, `found`, `termination_reason`
(`found` / `covered` / `timeout` / `aborted` / `not_started`), route progress, detector frame count,
every confirmed target, and the planned route.

A reported target position is an **estimate**: each detection box back-projected onto the ground
plane and averaged over frames, which is all a real system has. In simulation the result also
prints the true position underneath it, together with the target's id, its true class, and the
error — so a class mistake and a localization error are both visible at a glance:

```text
targets (estimate = back-projected from the detections):
  - pedestrian   estimate (   48.06,   -16.45,  -1.65)  hits=4  best_score=0.58
                 truth    (   48.00,   -17.00,  -1.65)  ped_02 [pedestrian]  error 0.55 m
```

The truth comes from the **Ground truth YAML (sim)** setting, which is read only when displaying a
finished result — the mission itself never sees it. Clear that setting on a real vehicle, where
there is no such file, and the truth lines are simply left out. An estimate with no target near it
is called out as a possible false positive.

The vehicle must already be airborne and holding: the mission queues a route, it does not take off.

## Coverage map selection

The persistent operations map defaults to
`src/search/config/yungu_map.json`. It renders the map origin and occupied footprints directly
from the planner-compatible JSON; no image file or additional Python package is required.

Use **Browse…** to select another map JSON and **Reload** after editing one. The selected map is a
visual aid only: it does not reconfigure the running coverage planner. Select the same map used by
the planner's startup configuration to avoid planning against a different obstacle layout.

The selected tab controls click behavior without hiding the map. On **Navigate**, one click selects
an ENU goal. On **Coverage Search** and **Search mission**, two opposite clicks create an
axis-aligned ENU rectangle; the
GUI fills the service corners in southwest, southeast, northeast, northwest order. You can still
edit all values manually. **Reset selection** clears only the search rectangle. Successful Plan
only and Plan and queue requests overlay their returned sparse waypoint route in blue.

The operations map remains visible across tab changes and also shows the live vehicle pose from
`/gz/ground_truth/odom` (black heading arrow), the authoritative offboard route from
`/waypoint_buffer/status` (orange is the active target, purple are pending waypoints), and the
targets a search mission confirmed (magenta diamonds).
Both topics are editable in the connection settings. The vehicle source must already be ENU and
aligned with the selected map; the GUI does not transform frames. The map remains visual only: it
does not check clicked routes for collision or alter the map used by the running planner.

Run the non-graphical import check with:

```bash
/usr/bin/python3 /home/windshape/YunguProject/gui/skills_gui.py --check
```
