# map_coord_bridge

覆盖规划器局部 ENU（如 `yungu2030_local_origin`）与仿真 world ENU 之间的
**双向坐标桥**：`p_world = Rz(yaw_deg) * p_map + T`。

两个独立通道：

| 方向 | 输入 | 输出 | 用途 |
|---|---|---|---|
| map → world | `--map-in-topic`（PoseStamped，planner 系，默认 `/map_pose_in`） | `--world-out-topic`（PoseStamped，world 系，默认 `/waypoint_pose`） | 地图系航点/目标直接喂给 offboard 链路（QoS best_effort，匹配 goal_marker） |
| world → map | `--world-in-topic`（Odometry，world 系，默认 `/lidar_slam/odom`） | `--map-out-topic`（Odometry，planner 系，默认 `/map/vehicle_odom`） | 飞机实时位置转到地图系，用于标定验证和覆盖显示 |

## 用法

```bash
# 直接运行
python3 src/map_coord_bridge/map_coord_bridge/map_coord_bridge.py \
  --offset-x -153.4 --offset-y -67.2 --offset-z -23.85

# 或 launch（参数走 MCB_* 环境变量）
MCB_OFFSET_X=-153.4 MCB_OFFSET_Y=-67.2 \
ros2 launch map_coord_bridge map_coord_bridge.launch.py
```

## 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `--offset-x/y/z` | 0 | 平移 `T`（标定方法见 `docs/coverage-search-integration.md` 第 3 节） |
| `--yaw-deg` | 0 | 地图→world 绕 z 旋转（地图与 world 轴不一致时） |
| `--map-in-topic` | `/map_pose_in` | planner 系 PoseStamped 输入 |
| `--world-out-topic` | `/waypoint_pose` | world 系 PoseStamped 输出（可直接飞） |
| `--world-in-topic` | `/lidar_slam/odom` | world 系 Odometry 输入 |
| `--map-out-topic` | `/map/vehicle_odom` | planner 系 Odometry 输出 |

## 验证标定 T

1. 启动仿真栈，飞机稳定后：
   ```bash
   ros2 topic echo /map/vehicle_odom --qos-reliability reliable --spin-time 1
   ```
   读到的位置应 ≈ 规划器 `start` 点（若 T 正确）。
2. 若偏离，调整 `--offset-*` 直到 `/map/vehicle_odom` 读数与 `start` 一致。

## 与 csp_adapter 的关系

- `csp_adapter`：读 `flight_plan.json` 批量发航点（自动执行任务）；
- `map_coord_bridge`：手工/外部把地图系坐标点转发进 world 系（交互式验证、
  单点试飞、RViz 地图系点击），并回报飞机在地图系的位置。
两者共用同一变换（`p_world = Rz·p_map + T`）和同一套 `offset/yaw` 参数；
同时运行时注意 `--world-out-topic` 别都用 `/waypoint_pose`（可用不同 topic）。
