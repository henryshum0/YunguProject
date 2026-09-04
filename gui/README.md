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

The connection panel defaults to `map`, `/coverage_planner/plan_coverage`,
`/waypoint_buffer`, `/waypoint_buffer/clear`, `/takeoff_cmd`, `/land_cmd`, and a 10-second timeout.
All values are editable for namespaced or remapped systems.

- **Navigate** accepts one `x, y, z, heading_deg` waypoint per line. Select ENU or NED; the
  existing `NavigateSkill` performs the conversion and queues the full route.
- **Clear route** calls the clear service, aborting the current route and removing queued waypoints.
- **Plan only** calls `PlanSearchPrimitive` using four ENU search corners and displays its path
  without publishing planner waypoint or marker topics.
- **Plan and queue** calls `SearchSkill`, publishes the planner visualization, then displays the
  route after it was accepted by the offboard queue service.
- **Take off** and **Land** publish the existing `Bool(data=True)` commands only after a confirmation
  dialog.

Run the non-graphical import check with:

```bash
/usr/bin/python3 /home/windshape/YunguProject/gui/skills_gui.py --check
```
