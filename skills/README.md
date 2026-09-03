# ROS skill interfaces

This is a plain Python package that talks to ROS 2 nodes already running in the
background. It is not a colcon package.

Prepare an interactive shell by sourcing ROS and the workspace that contains the generated
coverage-planner service:

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
export PYTHONPATH=/home/windshape/YunguProject:$PYTHONPATH
```

Use the primitives from an orchestrator node:

```python
from rclpy.node import Node
from skills import MovePrimitive, NavigateSkill, PlanSearchPrimitive

node = Node("mission_orchestrator")
search = PlanSearchPrimitive(node, frame_id="map")
path = search.call(((10.0, 10.0), (120.0, 10.0), (120.0, 80.0), (10.0, 80.0)))

move = MovePrimitive(node)  # reliable /waypoint_buffer by default
count = move.call(path.poses)
```

`NavigateSkill` is the coordinate-based interface to `MovePrimitive`. Its waypoints use
`(x, y, z, heading_deg)` and can be supplied singly or as a sequence. ENU input is
`(east, north, up, yaw)` with `0°` facing East and counter-clockwise-positive rotation;
NED input is `(north, east, down, yaw)` with `0°` facing North and clockwise-positive rotation.

```python
from skills import NavigateSkill

navigate = NavigateSkill(node, frame_id="map")
navigate.call((10.0, 20.0, 5.0, 90.0), frame="enu")
navigate.call([
    (20.0, 10.0, -5.0, 0.0),
    (25.0, 10.0, -5.0, 45.0),
], frame="ned")
```

The skill transforms NED waypoints to ENU and stores the ENU heading in each generated
`PoseStamped` quaternion before publishing to `/waypoint_buffer`. It returns after publication,
not after physical arrival. The current offboard FSM navigates based on waypoint position.

`SearchSkill` composes the planner and navigation interfaces: it requests a four-corner ENU
search area, queues the returned ENU `Path` through `NavigateSkill`, then returns that path.

```python
from skills import SearchSkill

search_and_navigate = SearchSkill(node, frame_id="map")
path = search_and_navigate.call(((10.0, 10.0), (120.0, 10.0), (120.0, 80.0), (10.0, 80.0)))
```

`PlanSearchPrimitive` calls `/coverage_planner/plan_coverage`, waits for its response, and returns
the sparse `nav_msgs/msg/Path`. Its poses already use ENU positions and ROS ENU-yaw quaternions,
matching ENU output from `NavigateSkill`. It requires exactly four finite, distinct ENU corners
in the configured map frame. `MovePrimitive` only publishes `PoseStamped` goals to the running
`offboard_fsm`; it returns once they have been published, not when the vehicle finishes flying.

Both primitives accept alternate ROS names through their constructors. Calls raise
`SkillTimeoutError` when the planner service is unavailable or does not respond, and
`SkillExecutionError` when the planner rejects an otherwise valid request.
