# Navigation visualization

This package supplies RViz support for the current EGO/PX4 navigation stack.
It does not run a planner or control the vehicle.

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
ros2 launch visualization visualization.launch.py
```

The launch starts:

- `visual_tf`, which publishes the `world → body` vehicle transform from the
  configured PX4 ENU odometry;
- `gt_path.py`, which publishes the Gazebo ground-truth flight trail;
- `ego_trajectory_path.py`, which samples EGO's custom B-spline into an RViz
  `nav_msgs/Path`; and
- RViz unless `rviz:=false` is supplied.

The RViz profile displays the configured EGO input cloud, the inflated local
map, the planned B-spline path, and the ground-truth path. EGO publishes the
inflated cloud with volatile durability, so the RViz display intentionally uses
volatile QoS; it will show map updates while EGO is running.

Topic names and trajectory sampling settings are configured in
[`src/simulation/config/visualization.yaml`](../../../simulation/config/visualization.yaml).
The EGO/PX4 bridge currently keeps `/gz/odom_super` and
`/gz/point_cloud_super` as endpoint names; they are EGO's active inputs, not
SUPER planner interfaces.
