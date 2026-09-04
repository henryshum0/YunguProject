# Coverage planner and offboard FSM launcher

This directory contains a workspace-level launch file that starts the
`coverage_planner_node` directly and includes the existing `offboard_fsm`
launch.

After building and sourcing the workspace overlay, run:

```bash
source /opt/ros/humble/setup.bash
source /home/windshape/YunguProject/install/setup.bash
ros2 launch /home/windshape/YunguProject/src/launch/coverage_and_offboard.launch.py
```

The launcher uses the installed `coverage_planner` Yungu example by default:
`yungu_planner.json` and its adjacent `yungu_map.json`.  Select a different
planner JSON with:

```bash
ros2 launch /home/windshape/YunguProject/src/launch/coverage_and_offboard.launch.py \
  config_file:=/absolute/path/to/planner.json
```

The included `offboard_fsm` launcher retains its existing workspace
configuration files:

- `config/simulation.yaml`
- `config/offboard/offboard_fsm.yaml`
- `config/offboard/topics.yaml`
- `config/offboard/fastlio_swan_gamma_effect.yaml`

In particular, its `use_sim_time` value is read from
`config/offboard/offboard_fsm.yaml`; the combined launcher does not override it.

FAST-LIO is enabled by default. Disable it for controller/planner testing with
`use_fastlio:=false`. The combined launch starts the coverage planning service
and waypoint queue services; planning a route does not automatically enqueue it
to the vehicle. Use `SearchSkill` or `/waypoint_buffer` to queue a returned
route.
