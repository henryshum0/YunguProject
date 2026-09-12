# ROS skill interfaces

This is a plain Python package that talks to ROS 2 nodes already running in the
background. It is not a colcon package.

## Package layout

- `primitives/` contains ROS client adapters and `primitives/base.py`, the
  `Primitive` base class and common request errors.
- `skills/` contains the composed `Skill` base class, `NavigateSkill`, and
  `SearchSkill`.
- `helper/frames.py` contains reusable ENU/NED waypoint and pose conversions.

The top-level `skills` package continues to export the public classes for the
examples below.

Prepare an interactive shell by sourcing ROS and the workspace that contains the generated
coverage-planner action:

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
export PYTHONPATH=/home/windshape/YunguProject:$PYTHONPATH
```

Every primitive requires one validated workspace configuration. Load it once
from the offboard configuration directory and the coverage-planner startup JSON:

```python
from rclpy.node import Node
from skills import (
    ClearWaypointsPrimitive,
    LandPrimitive,
    MovePrimitive,
    NavigateSkill,
    PlanSearchPrimitive,
    SkillRuntimeConfig,
    TakeoffPrimitive,
)

node = Node("mission_orchestrator")
config = SkillRuntimeConfig.load(
    navigation_config_dir="/home/windshape/YunguProject/src/navigation/config/offboard",
    planner_config_file="/home/windshape/YunguProject/src/search/config/yungu_planner.json",
)

search = PlanSearchPrimitive(node, config=config)
path = search.call(((10.0, 10.0), (120.0, 10.0), (120.0, 80.0), (10.0, 80.0)))

move = MovePrimitive(node, config=config)
count = move.call(path.poses)
cleared = ClearWaypointsPrimitive(node, config=config).call()
takeoff_message = TakeoffPrimitive(node, config=config).call()
land_message = LandPrimitive(node, config=config).call()
```

`NavigateSkill` is the coordinate-based interface to `MovePrimitive`. Its waypoints use
`(x, y, z, heading_deg)` and can be supplied singly or as a sequence. ENU input is
`(east, north, up, yaw)` with `0°` facing East and counter-clockwise-positive rotation;
NED input is `(north, east, down, yaw)` with `0°` facing North and clockwise-positive rotation.

```python
from skills import NavigateSkill

navigate = NavigateSkill(node, config=config)
navigate.call((10.0, 20.0, 5.0, 90.0), frame="enu")
navigate.call([
    (20.0, 10.0, -5.0, 0.0),
    (25.0, 10.0, -5.0, 45.0),
], frame="ned")
```

The skill transforms NED waypoints to ENU and stores the ENU heading in each generated
`PoseStamped` quaternion before queuing the batch through `/waypoint_buffer`. It returns after
the service accepts the route, not after physical arrival. The current offboard FSM navigates
based on waypoint position. Before queuing, it waits up to 10 seconds for the `offboard_fsm`
service and raises `SkillTimeoutError` if the lower-level node is not ready. Call
`navigate.clear()` to abort the active route and remove all queued waypoints through
`/waypoint_buffer/clear`.

`SearchSkill` composes the planner and navigation interfaces: it requests a four-corner ENU
search area, queues the returned ENU `Path` through `NavigateSkill`, then returns that path.

```python
from skills import SearchSkill

search_and_navigate = SearchSkill(node, config=config)
path = search_and_navigate.call(((10.0, 10.0), (120.0, 10.0), (120.0, 80.0), (10.0, 80.0)))
```

`PlanSearchPrimitive` sends a goal to `/coverage_planner/plan_coverage`, receives an immediate
accepted/rejected acknowledgement, then waits for its asynchronous result and returns the sparse
`nav_msgs/msg/Path`. It defaults to a non-publishing preview; pass
`publish_result=True` to refresh the planner waypoint and marker topics. `SearchSkill` does this
automatically before queueing its route. Its poses already use ENU positions and ROS ENU-yaw
quaternions, matching ENU output from `NavigateSkill`. It requires exactly four finite, distinct
ENU corners in the configured map frame. `MovePrimitive` submits `PoseStamped` batches to the running
`offboard_fsm` queue service; it returns once they have been accepted, not when the vehicle
finishes flying. Planner, queue, and clear-service readiness failures raise `SkillTimeoutError`.

`SkillRuntimeConfig` validates `offboard_fsm.yaml`, `topics.yaml`, and the planner JSON before
any ROS client is created. It requires matching ENU frame IDs, derives the planner action as
`/coverage_planner/plan_coverage`, and reads queue, clear, takeoff, land, and queue-status names
from `topics.yaml`. Takeoff and land call `std_srvs/srv/Trigger`; a successful response confirms
FSM acceptance, while flight progress remains asynchronous. Calls raise `SkillTimeoutError` when the planner or offboard service is
unavailable or does not respond, and `SkillExecutionError` when a service rejects a valid request
or a planner action returns a failed result.
