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

Every primitive requires one validated workspace configuration. Load it once
from the offboard configuration directory and the coverage-planner startup JSON:

```python
from rclpy.node import Node
from skills import (
    ClearWaypointsPrimitive,
    MovePrimitive,
    NavigateSkill,
    PlanSearchPrimitive,
    SkillRuntimeConfig,
)

node = Node("mission_orchestrator")
config = SkillRuntimeConfig.load(
    navigation_config_dir="/home/windshape/YunguProject/src/navigation/config/offboard",
    planner_config_file="/home/windshape/YunguProject/src/search/config/yungu_planner.json",
    # Optional. Required by DetectSkill and by a full SearchSkill mission.
    detection_config_file="/home/windshape/YunguProject/src/detection/config/detection.yaml",
)

search = PlanSearchPrimitive(node, config=config)
path = search.call(((10.0, 10.0), (120.0, 10.0), (120.0, 80.0), (10.0, 80.0)))

move = MovePrimitive(node, config=config)
count = move.call(path.poses)
cleared = ClearWaypointsPrimitive(node, config=config).call()
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

`DetectSkill` watches the detector stream for the classes a mission is looking for. Its input is
what to search for: the group names `person` and `vehicle`, or detector class names such as
`car` and `pedestrian`. `success` means the detection pipeline ran and was observed; use `found`
for whether something was located.

```python
from skills import DetectSkill

detect = DetectSkill(node, config=config)
result = detect.call(("vehicle",), timeout_sec=10.0)
print(result.success, result.message, result.found)
for target in result.targets:
    print(target.class_id, target.position, target.hits, target.best_score)
```

A single detection is never treated as a find. A target is confirmed only once `min_hits`
detections of the same class agree on a world position within `cluster_radius_m`, which is what
keeps a detector's false positives out of a mission result. `start()`/`collect()`/`stop()` expose
the same stream without blocking, which is how `SearchSkill` watches for targets while flying.
World positions come from `target_localizer`; when it is not running, detections are still
reported with `world_position` left as `None`.

`SearchSkill` has two entry points.

`plan_and_queue` is command level: it requests a four-corner ENU search area, queues the returned
ENU `Path` through `NavigateSkill`, and returns that path without waiting for the flight.

```python
from skills import SearchSkill

search = SearchSkill(node, config=config)
path = search.plan_and_queue(((10.0, 10.0), (120.0, 10.0), (120.0, 80.0), (10.0, 80.0)))
```

`call` is the complete mission: plan, queue, watch the detector while the route is flown, and
return a `SearchResult`. It blocks until the route is covered, a target is confirmed, or
`mission_timeout_sec` elapses, and it drives the node's executor while it waits — so do not spin
that node elsewhere at the same time. The vehicle must already be airborne and holding; the skill
queues a route, it does not take off.

```python
result = search.call(
    ((20.0, -20.0), (60.0, -20.0), (60.0, 20.0), (20.0, 20.0)),
    classes=("vehicle",),
    stop_on_first_detection=True,   # abort the rest of the route once a target is confirmed
)
print(result.success, result.message)
print(result.found, result.termination_reason)          # found / covered / timeout / not_started
print(result.waypoints_completed, result.waypoints_total, result.route_completion)
for target in result.targets:
    print(target.class_id, target.position, target.hits)
```

By default a confirmation only counts detections placed within `confirm_within_m` (20 m) of the
vehicle, and after the first confirmation the mission keeps observing for `settle_sec` (2 s) before
stopping. Together these fix the "stop on the first, distant sighting" problem: a target seen far
off — where the back-projected position is unreliable — does not end the mission; the sweep flies
on and confirms the target once it is close, then averages a few more close-range frames into the
reported position. Set `confirm_within_m=None` to count detections at any range (stop on the first
confirmation, wherever it happens), and `settle_sec=0.0` to stop the instant a target is confirmed.

`clear_existing=True` (the default) empties the waypoint queue before planning, so route progress
refers to this mission only. `route_completion` is route progress, not area coverage: the planner
reports the route it designed for the area, not how much ground was actually observed.

`ConfirmedTarget.position` is an **estimate**: the detection boxes back-projected onto the ground
plane and averaged over the frames that agreed. A real detector gives nothing else, so neither does
this. In simulation `detection.truth.match_to_truth` pairs an estimate with the ground-truth target
it refers to, for reading a result afterwards — the mission itself never sees the truth.

A mission lasts minutes, so it can be watched and called off. `on_progress` is invoked about twice a
second with a `SearchProgress` snapshot (elapsed time, waypoints flown, detector frames, targets
confirmed so far), and `stop_requested` is polled every cycle — returning true aborts the route and
ends the mission with `termination_reason == "aborted"`. This is how the GUI's **Search mission** tab
shows live progress and implements its **Stop mission** button.

```python
from threading import Event

cancel = Event()
result = search.call(area, classes=("person",),
                     on_progress=lambda progress: print(progress.waypoints_completed),
                     stop_requested=cancel.is_set)
```

`PlanSearchPrimitive` calls `/coverage_planner/plan_coverage`, waits for its response, and returns
the sparse `nav_msgs/msg/Path`. It defaults to a non-publishing preview; pass
`publish_result=True` to refresh the planner waypoint and marker topics. `SearchSkill` does this
automatically before queueing its route. Its poses already use ENU positions and ROS ENU-yaw
quaternions, matching ENU output from `NavigateSkill`. It requires exactly four finite, distinct
ENU corners in the configured map frame. `MovePrimitive` submits `PoseStamped` batches to the running
`offboard_fsm` queue service; it returns once they have been accepted, not when the vehicle
finishes flying. Planner, queue, and clear-service readiness failures raise `SkillTimeoutError`.

`SkillRuntimeConfig` validates `offboard_fsm.yaml`, `topics.yaml`, the planner JSON, and — when
supplied — `detection.yaml` before any ROS client is created. It requires matching ENU frame IDs
across all three, derives the planner service as `/coverage_planner/plan_coverage`, and reads
queue, clear, takeoff, land, and queue-status names from `topics.yaml`. Without
`detection_config_file`, `config.detection` is `None` and the detection-aware entry points raise
`SkillExecutionError` saying so; everything else works unchanged. Calls raise `SkillTimeoutError` when the planner or offboard service is
unavailable or does not respond, and `SkillExecutionError` when a service rejects a valid request.
