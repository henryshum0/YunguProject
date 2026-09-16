# ROS request interfaces

This is the authoritative client contract for the coverage planner and offboard
FSM. The GUI and the `skills` package use these request interfaces; they do not
publish vehicle commands directly.

## Coverage planning action

`/coverage_planner/plan_coverage` uses
`coverage_planner/action/PlanCoverage`. It is an action because coverage
planning can take longer than a normal service request.

- Goal: `geometry_msgs/msg/PolygonStamped search_area` with exactly four ENU
  corners in the configured map frame, plus `bool publish_result`.
- Goal receipt: accepted/rejected immediately by the action server.
- Feedback: `string stage` (`validating`, `planning`, `publishing`,
  `succeeded`, `failed`, or `cancelled`).
- Result: `bool success`, `string message`, and a sparse
  `nav_msgs/msg/Path waypoints`. A failed result has `success=false`; no route
  should be queued.

By default, the action calculates only. Set `publish_result=true` only when
the planner's Path/MarkerArray visualization should be refreshed.

## EGO/PX4 offboard services

| Endpoint | Type | Request | Successful response | Meaning |
|---|---|---|---|---|
| `/waypoint_buffer` | `offboard_fsm/srv/QueueWaypoints` | `PoseStamped[] waypoints` | `success`, `message`, `queued_count` | Atomically append an ordered ENU route. A queued point becomes EGO's next manual goal only after takeoff. |
| `/waypoint_buffer/clear` | `offboard_fsm/srv/ClearWaypoints` | none | `success`, `message`, `cleared_count` | Drop the active and pending route, stop forwarding EGO commands, and hold the current PX4 pose. |
| `/offboard/takeoff` | `std_srvs/srv/Trigger` | none | `success`, `message` | Accept a takeoff request only from `INIT`. Arming, OFFBOARD entry, and climb occur asynchronously. |
| `/offboard/land` | `std_srvs/srv/Trigger` | none | `success`, `message` | Abort navigation and request native PX4 LAND. Touchdown/disarm remain asynchronous. |

For example:

```bash
ros2 service call /offboard/takeoff std_srvs/srv/Trigger "{}"
ros2 service call /offboard/land std_srvs/srv/Trigger "{}"
```

A successful `Trigger` response confirms command acceptance, not that the
vehicle has become airborne, landed, or disarmed. Observe vehicle telemetry
for physical progress.

## Internal EGO interfaces

The FSM is the only normal client of these internal endpoints:

| Endpoint | Type | Meaning |
|---|---|---|
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | One ENU EGO manual goal, including the requested altitude. |
| `/ego_planner/state` | `std_msgs/msg/String` | Reliable EGO state: `INIT`, `WAIT_TARGET`, planning/executing states, or `ERROR`. The FSM completes a route point only after observing execution followed by `WAIT_TARGET`. |
| `/ego_planner/position_cmd` | `quadrotor_msgs/msg/PositionCommand` | EGO ENU position, velocity, acceleration, yaw, and yaw-rate command forwarded by the FSM to PX4 as NED. |

## Read-only queue state

`/waypoint_buffer/status` is a reliable, transient-local
`nav_msgs/msg/Path` snapshot. Its first pose is the active target and later
poses are pending targets in execution order. It is telemetry, not a command
endpoint.

The queue status is telemetry, not an EGO goal or PX4 command interface.
