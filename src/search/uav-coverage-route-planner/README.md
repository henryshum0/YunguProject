# coverage_planner

`coverage_planner` is a ROS 2 Humble node that loads static map and planning settings at startup.
Every search boundary, plan result, and optional topic publication is driven by the
`PlanCoverage` service.

The package targets Ubuntu 22.04, ROS 2 Humble, and Python 3.10. It uses system Python—no pip,
virtual environment, or uv is required.

## Build

Install Shapely with apt, then build from the colcon workspace:

```bash
sudo apt update
sudo apt install python3-shapely
cd /home/windshape/YunguProject
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select coverage_planner --symlink-install
source install/setup.bash
```

If this workspace was built with a stale virtual-environment interpreter, add
`--cmake-clean-cache` to the build command once.

## Run and visualize

The required `config_file` parameter is the planner settings JSON. Its `map_file` may be an
absolute path or a path relative to the planner JSON:

```bash
ros2 run coverage_planner coverage_planner_node --ros-args \
  -p config_file:=$(ros2 pkg prefix coverage_planner)/share/coverage_planner/config/example_planner.json
```

The node validates its configuration and waits for a planning request; it does not publish a
route or markers at startup. After a successful request, late subscribers receive the latched
result:

```bash
ros2 topic echo --once /coverage_planner/waypoints
ros2 topic echo --once /coverage_planner/markers
```

## Request a plan by service

The node also exposes `~/plan_coverage`, which resolves to
`/coverage_planner/plan_coverage`. Its type is `coverage_planner/srv/PlanCoverage`.
Send exactly four unclosed ENU corners in `search_area` as a `geometry_msgs/PolygonStamped` whose
`header.frame_id` exactly matches the configured map frame. A successful response returns the
sparse `nav_msgs/Path`. Set `publish_result: true` to also refresh the latched waypoint and
marker topics; it defaults to `false`, which is a non-publishing preview. The point `z` values
are ignored; route altitude still comes from the planner JSON.

```bash
ros2 service call /coverage_planner/plan_coverage coverage_planner/srv/PlanCoverage \
  "{search_area: {header: {frame_id: map}, polygon: {points: [
    {x: 10.0, y: 10.0}, {x: 120.0, y: 10.0},
    {x: 120.0, y: 80.0}, {x: 10.0, y: 80.0}
  ]}}, publish_result: false}"
```

The response has `success`, `message`, and `waypoints`. Invalid frames, anything other than four
points, non-finite coordinates, or self-intersecting quadrilaterals return `success: false`; the
node stays available for the next request.

For the preconfigured RViz view:

```bash
ros2 launch coverage_planner coverage_planner_rviz.launch.py
```

For a custom mission, pass matching configured topics to the RViz launch file:

```bash
ros2 launch coverage_planner coverage_planner_rviz.launch.py \
  config_file:=/absolute/path/to/planner.json \
  waypoints_topic:=/uav_1/coverage_waypoints \
  markers_topic:=/uav_1/coverage_markers
```

## Configuration files

Planner JSON uses schema `1.2`. It contains static flight, camera, route, and output settings;
it does not contain `origin`, `occupied_areas`, or a search area. The service request is the
only source of the search boundary.

```json
{
  "schema_version": "1.2",
  "map_file": "yungu_map.json",
  "frame_id": "map",
  "output_topics": {
    "waypoints": "/coverage_planner/waypoints",
    "markers": "/coverage_planner/markers"
  },
  "flight": {"altitude_m": 25.0},
  "camera": {
    "horizontal_fov_deg": 60.0,
    "vertical_fov_deg": 45.0,
    "side_overlap": 0.3
  }
}
```

Map JSON uses schema `1.0` and contains only reusable map parameters: the home origin and
occupied footprints. It deliberately does not contain a search area, because a coverage boundary
can vary per mission even when the map does not.

```json
{
  "schema_version": "1.0",
  "origin": {"x": 0.0, "y": 0.0},
  "occupied_areas": [
    {
      "id": "collider_001",
      "points": [[1.0, 1.0], [5.0, 1.0], [5.0, 4.0], [1.0, 4.0]]
    }
  ]
}
```

Both files reject unknown fields, non-finite values, invalid/self-intersecting polygons, and
duplicate occupied-area IDs. A polygon needs three distinct points; a repeated closing point is
accepted and normalized. `occupied_areas` defaults to an empty array in the map JSON.

Coordinates are local ENU metres. `origin` is the UAV home position and every published waypoint
uses `flight.altitude_m` as its ENU z coordinate. Occupied footprints are removed from searchable
ground and become horizontally buffered, non-overflyable obstacles.

Defaults in the planner JSON are `frame_id: map`, `/coverage_planner/waypoints` and
`/coverage_planner/markers`, ground elevation `0.0 m`, horizontal clearance `3.0 m`, Global
Scanline generation, automatic scan direction and route optimization, local completion, 99%
patch coverage, and return to origin.

## Generate a Yungu map JSON

The workspace utility converts the supplied binary collider STL to the strict map schema without
additional Python packages. Run it from `/home/windshape/YunguProject`:

```bash
python3 util/stl_to_planner_map.py \
  VisionFlow-PX4/Tools/simulation/gz/worlds/yungu_collider.stl \
  --output src/search/config/yungu_map.json
```

The Yungu world references the mesh at ENU origin with no scale or pose, so no coordinate
transform is applied. The utility emits the 25 raised collider footprints as `collider_001`
through `collider_025`, ignores the flat ground-plane component (height ≤ `0.1 m`), and writes
the map origin as `(0.0, 0.0)`. Override it when needed with `--origin-x` and `--origin-y`; use
`--minimum-height-m` to adjust ground-plane filtering.

## Outputs and failure behavior

`output_topics.waypoints` is a `nav_msgs/msg/Path` containing the sparse planner route: home,
coverage-lane endpoints, connector or obstacle-avoidance vertices, and optional return home.
`output_topics.markers` is a `visualization_msgs/msg/MarkerArray` containing a green search
boundary, red occupied boundaries, and blue waypoint points. Both use reliable,
transient-local, keep-last-1 QoS.

If either JSON cannot be loaded or validated, the node logs a fatal error, publishes neither
topic, and exits nonzero. Invalid or infeasible service requests return `success: false` without
publishing a new result; the node remains available for the next request.

## Test

```bash
cd /home/windshape/YunguProject
source /opt/ros/humble/setup.bash
colcon test --packages-select coverage_planner --event-handlers console_direct+
colcon test-result --verbose
```

The preserved coverage algorithm is described in [`docs/core-algorithm.md`](docs/core-algorithm.md).
