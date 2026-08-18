# cmd_record

Goal-triggered recorder for SUPER's command trajectory plus the real drone
odometry. Each time you click a goal, it starts recording the commanded
trajectory (`/planning/pos_cmd`) together with the real odometry
(`/lidar_slam/odom`); when a new goal arrives it saves the previous segment and
starts a new one. Each segment is written to its own CSV under
`<project>/cmd_log/`.

It also runs a **realtime sliding-window plot** (position, velocity,
acceleration, attitude, body rate, real odometry) that refreshes live while the
planner is commanding.

## Features

- **Goal-triggered**: nothing is recorded until a goal is published on
  `/goal_pose` (`geometry_msgs/PoseStamped`).
- **Per-goal segments**: a new goal while recording stops + saves the current
  CSV and starts a fresh segment for the new goal.
- **Automatic stop**: a watchdog stops and saves a segment when the commanded
  trajectory publish rate drops below `min_cmd_rate` (default 10 Hz) — i.e. the
  planner stopped commanding.
- **Goal + trajectory in one file**: every CSV row repeats the goal position, so
  goal and data stay together.
- **Live sliding-window view**: realtime `window_sec`-long scrolling plot with
  **cmd vs real odometry per axis** (X/Y/Z rows; Position/Velocity/Accel/
  Attitude/Body-rate columns). The Body-rate cells show the **commanded vs real
  (odom) body rate**; the Z-row Attitude/Body-rate cells additionally show the
  **yaw command**, the **yaw-rate command** and the **real drone yaw** (from
  odometry), so the yaw tracking response is visible live.
- **Body-rate tracking**: every row records the commanded body rate
  (`/planning/pos_cmd.angular_velocity`) and the **real body rate** (`owx owy
  owz`, from the odometry twist), so the body-rate tracking response is visible
  in the CSV and overlaid in the Body-rate plot cells.
- **Yaw-command tracking**: every row records the commanded yaw
  (`/planning/pos_cmd.yaw`, **wrapped into [-π, π]** so it is directly
  comparable with the real drone yaw), commanded yaw rate (`yaw_dot`) and the
  **real drone yaw** (`oyaw`, extracted from the odometry quaternion).
- **Post-hoc plot**: `plot_csv` replays any saved CSV.

## CSV format

Files: `<project>/cmd_log/goal_<NNN>_<timestamp>.csv`

| col | name | meaning |
|-----|------|---------|
| 1 | `t` | timestamp [s] |
| 2–4 | `gx gy gz` | current goal position [m] (same in every row) |
| 5–7 | `px py pz` | commanded position [m] |
| 8–10 | `vx vy vz` | commanded velocity [m/s] |
| 11–13 | `ax ay az` | commanded acceleration [m/s²] |
| 14–16 | `roll pitch yaw` | commanded attitude [rad] |
| 17–19 | `wx wy wz` | commanded body rate [rad/s] |
| 20–22 | `owx owy owz` | real (odom) body rate [rad/s] |
| 23–24 | `yaw_cmd yaw_dot_cmd` | commanded yaw (wrapped to [-π, π]) + yaw rate [rad, rad/s] |
| 25 | `oyaw` | real drone yaw from odometry [rad] |
| 26–28 | `opx opy opz` | real odometry position [m] |
| 29–31 | `ovx ovy ovz` | real odometry velocity [m/s] |

## Build

```bash
cd <workspace root, e.g. /home/windshape/YunguProject>
source /opt/ros/humble/setup.bash
colcon build --packages-select cmd_record --symlink-install
source install/setup.bash
```

## Run (live recording + plot)

```bash
# direct
ros2 run cmd_record cmd_record_node

# or via launch
ros2 launch cmd_record record.launch.py
```

Then click a goal (RViz 2D Nav Goal → `/goal_pose`). The node starts recording;
each new goal switches to a new CSV. When the planner stops (cmd rate < 10 Hz)
the segment is saved automatically. `Ctrl-C` saves the active segment and exits.

### Parameters

| param | default | description |
|-------|---------|-------------|
| `goal_topic` | `/goal_pose` | goal click topic (`geometry_msgs/PoseStamped`) |
| `cmd_topic` | `/planning/pos_cmd` | SUPER command trajectory topic |
| `odom_topic` | `/lidar_slam/odom` | real drone odometry (`nav_msgs/Odometry`) |
| `log_dir` | *(empty)* | CSV directory; empty = `<project>/cmd_log` |
| `min_cmd_rate` | `10.0` | stop recording when cmd rate drops below this [Hz] |
| `viz_en` | `true` | enable the live sliding-window plot |
| `window_sec` | `20.0` | sliding window length [s] |
| `plot_rate` | `10.0` | live plot refresh [Hz] |
| `use_header_stamp` | `true` | use message header stamp for `t` |

Example overrides:

```bash
ros2 run cmd_record cmd_record_node --ros-args \
  -p window_sec:=30.0 -p min_cmd_rate:=10.0 -p viz_en:=false

ros2 launch cmd_record record.launch.py window_sec:=30.0 min_cmd_rate:=10.0
```

## Visualize a recorded CSV

```bash
# newest goal_*.csv in cmd_log
ros2 run cmd_record plot_csv

# specific file
ros2 run cmd_record plot_csv cmd_log/goal_002_20260805_163854.csv

# zoom into a window and export a PNG (headless-friendly)
ros2 run cmd_record plot_csv <file> --start 0.5 --end 1.5 --save out.png

# or run without ROS
python3 src/cmd_record/cmd_record/plot_csv.py <file>
```

The plot is organized per axis for response comparison: **3 rows = X / Y / Z**
and **5 columns = Position / Velocity / Accel / Attitude / Body rate**. On the
Position and Velocity columns the **commanded (solid blue)** and **real
odometry (dashed red)** curves are overlaid so you can see the tracking error;
the goal position is a dotted gray line on the Position column. Use
`--start`/`--end` to zoom into a window.

The CSVs are plain tables — you can also open them directly in Excel,
pandas, or Matlab for custom analysis.

## Troubleshooting

- **Goal segment created but 0 rows (cmd not recorded)**: SUPER publishes
  `/planning/pos_cmd` with **best-effort** QoS (`QoS(1).best_effort()` in
  `fsm_ros2.hpp`). The recorder subscribes with best-effort QoS to match — a
  default *reliable* subscription is incompatible with a best-effort publisher,
  so DDS never connects and no command is received. If you ever repoint
  `cmd_topic`/`odom_topic`/`goal_topic` at different publishers, keep the
  subscriber best-effort (compatible with both best-effort and reliable
  publishers).
- **No rows while a goal is active**: `fsm_node` only publishes
  `/planning/pos_cmd` while it is following a trajectory (FOLLOW_TRAJ/EMER_STOP).
  When idle/hovering there is no traffic, so nothing is recorded — this is
  expected.
- **Segment not stopped after reaching the goal**: the watchdog stops a segment
  when the cmd rate drops below `min_cmd_rate` for a trailing 1 s window (after
  a 1 s warm-up). If the planner keeps publishing a heartbeat above that rate,
  raise `min_cmd_rate` or set it to your planner's idle rate.

## Behavior notes

- Recording starts only after a goal click; command/odometry before that is
  ignored (odometry is still tracked so it is fresh the moment recording starts).
- A segment is considered "done" when the commanded rate drops below
  `min_cmd_rate` for the trailing 1-second window (with a 1 s warm-up after the
  goal so short bursts don't stop early).
- The first row of a segment may show `nan`/stale odometry (it samples the
  latest known odom at each command) — in normal operation odometry streams
  continuously so this is not an issue.

## Package layout

```
src/cmd_record/
├── package.xml
├── setup.py
├── cmd_record/
│   ├── cmd_record_node.py   # recorder node
│   └── plot_csv.py          # post-hoc CSV visualizer
└── launch/
    └── record.launch.py     # launch file for the node
```
