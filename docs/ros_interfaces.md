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

## Offboard FSM services

| Endpoint | Type | Request | Successful response | Meaning |
|---|---|---|---|---|
| `/waypoint_buffer` | `offboard_fsm/srv/QueueWaypoints` | `PoseStamped[] waypoints` | `success`, `message`, `queued_count` | Atomically append the ordered route. Empty batches are rejected. |
| `/waypoint_buffer/clear` | `offboard_fsm/srv/ClearWaypoints` | none | `success`, `message`, `cleared_count` | Abort the active target and clear all pending targets. |
| `/offboard/takeoff` | `std_srvs/srv/Trigger` | none | `success`, `message` | Accept a normal FSM takeoff request only while the FSM is in `INIT`; it remains latched while readiness and arming checks run. |
| `/offboard/land` | `std_srvs/srv/Trigger` | none | `success`, `message` | Accept native PX4 landing outside `INIT` and `LAND`. Touchdown and disarm occur asynchronously in the FSM. |

For example:

```bash
ros2 service call /offboard/takeoff std_srvs/srv/Trigger "{}"
ros2 service call /offboard/land std_srvs/srv/Trigger "{}"
```

A successful `Trigger` response confirms command acceptance, not that the
vehicle has become airborne, landed, or disarmed. Observe FSM/vehicle
telemetry for physical progress.

## Read-only queue state

`/waypoint_buffer/status` is a reliable, transient-local
`nav_msgs/msg/Path` snapshot. Its first pose is the active target and later
poses are pending targets in execution order. It is telemetry, not a command
endpoint.

`/waypoint_pose` remains a manual/RViz `geometry_msgs/msg/PoseStamped` input
to `goal_marker_node`; that node calls the queue service on behalf of RViz.
