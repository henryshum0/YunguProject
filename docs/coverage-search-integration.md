# 与 coverage-search-planner 的对接指南

本文档说明如何把 [coverage-search-planner](https://github.com/Jocelyn-2005/coverage-search-planner)
（`feature/continuous-lane-planning` 分支）生成的地面覆盖搜索飞行计划，接入
本仓库的**点对点避障导航**（offboard 状态机 + SUPER 规划器）执行。

> 上游仓库当前为纯 Python、无 ROS 依赖，输出版本化 `flight_plan.json`
> （ENU 米制坐标）交给"下层飞控适配器"——本仓库就是那个适配器的执行端。

## 1. 职责划分

| 系统 | 职责 | 产出/接口 |
|---|---|---|
| **coverage-search-planner** | 离线生成覆盖搜索航线（扫描方向、航线集合、避障连接、覆盖复核、航点细分） | `flight_plan.json`（ENU 米制航点 + 航段协议） |
| **YunguProject（本仓库）** | 在线执行：PX4/FAST-LIO 定位、SUPER 点对点避障规划、offboard 状态机 | 消费 `/waypoint_pose`（world ENU），飞抵每个航点 |

两者解耦点：**`flight_plan.json` 文件**。上游只需知道目标区域和地图语义，
下游只需逐点导航，互不依赖对方的运行环境。

```
coverage-search-planner (离线)          YunguProject (在线)
┌─────────────────────────┐            ┌─────────────────────────────┐
│ semantic_map + search   │            │ start_fastlio.sh            │
│ area + planner_config   │            │  ├─ PX4 + FAST-LIO + EKF2   │
│         │               │            │  └─ super_bridge            │
│         ▼               │            │                             │
│ flight_plan.json        │────转换────▶│ csp_adapter (本指南第 4 节) │
│  (ENU 航点+航段)         │  坐标系对齐  │      │                     │
└─────────────────────────┘            │      ▼                     │
                                       │ /waypoint_pose ──→ offboard │
                                       │        └──→ SUPER 避障规划    │
                                       │             └──→ PX4 执行     │
                                       └─────────────────────────────┘
```

## 2. flight_plan.json 协议（v3.0 要点）

协议全文见上游 [docs/flight-controller-interface.md](https://github.com/Jocelyn-2005/coverage-search-planner/blob/feature/continuous-lane-planning/docs/flight-controller-interface.md)，
对接需要的最小字段：

**顶层**：`schema_version: "3.0"`、`coordinate_frame: "ENU"`、`units: "meters"`、
`map_id`（ENU 原点标识）、`waypoints[]`、`route_segments[]`、`lanes[]`、
`summary`（含 `mission_status`）。

**waypoint**（飞控途径点）：

```json
{
  "id": 1,
  "sequence": 1,
  "x": 153.4, "y": 67.2, "z": 25.0,
  "heading_deg": 90.0,
  "speed_mps": 5.0,
  "turn_in_place": false,
  "hold_time_s": 0.0
}
```

- `sequence` 从 1 连续递增，最后一个点速度为 0
- `heading_deg`：0° 指北、90° 指东（ENU 系下即 y 轴方向为 0°，x 轴方向为 90°）
- `turn_in_place=true`：多旋翼应先到点、稳定悬停 `hold_time_s`、原地调头再进入下一航段

**route_segment**：

```json
{
  "segment_id": 3,
  "kind": "coverage_lane",   // coverage_lane | connector | obstacle_avoidance | return_home
  "start_waypoint_id": 5,
  "end_waypoint_id": 18,
  "detection_enabled": true
}
```

- `detection_enabled`：该航段视频分析结果是否计入覆盖（只影响检测统计，不影响飞行）
- `obstacle_avoidance` 段是规划器算好的绕障路径，**不要跳过**

**执行前校验（不满足则拒绝执行）：**

- `schema_version == "3.0"`，`coordinate_frame == "ENU"`
- `summary.mission_status == "ready"`（`infeasible_coverage` 禁止执行）
- 航段连通：每段 `start_waypoint_id` 等于上一段 `end_waypoint_id`（首段从起降点开始）
- 高度/速度在载具能力内（本仿真栈参考值：巡航 ≤ 4 m/s，高度 ≤ 20 m）

## 3. 坐标系对齐（最重要的一步）

两边都是 ENU 米制，但**原点不同**：

| 系统 | 坐标系 | 原点 |
|---|---|---|
| coverage-search-planner | 本地 ENU（`map_id: "yungu2030_local_origin"`） | 语义地图自定义原点，示例 `start: [153.4, 67.2, 25.0]` 为起降点 |
| YunguProject | world ENU（`frame_id: "world"`） | Gazebo world 原点 (0,0,0)；PX4 `vehicle_odometry` 实测已是 world 系（无需平移） |

只要两边的轴方向一致（x 东、y 北、z 上），变换就是一个**常数平移**：

```
p_world = p_planner + T,   T = (t_x, t_y, t_z)
```

### 标定方法

方法 A（推荐，起飞前标定一次）：

1. 在 RViz 里把无人机飞到 planner 地图原点的物理位置（或起飞点 `start` 对应的地标）
2. 读取 `ros2 topic echo /lidar_slam/odom --qos-reliability best_effort --spin-time 1` 得 `p_world`
3. planner 里同一点的坐标是 `p_planner`（地图原点即 (0,0,0)，起飞点即 `start`）
4. `T = p_world - p_planner`，写入适配器配置

方法 B（无地标时）：`start` 起降点即无人机 spawn 后起飞的位置，直接用起飞瞬间
`/lidar_slam/odom` 的读数近似 `T + start`。PX4 `vehicle_odometry` 已是
world 系（2026-08-19 实测），所以起飞点读数直接 ≈ 模型 spawn 位置
`(-4, -2, z)`。

> 若语义地图的 y 轴与 world 不一致（如地图旋转过），还需一个绕 z 的旋转矩阵；
> 先在地图上选两个已知 world 坐标的点解算。

## 4. 适配器节点（csp_adapter）设计

建议新增一个 ROS 2 Python 节点（如 `src/csp_adapter/`），职责：

1. **加载并校验** `flight_plan.json`（第 2 节校验项）
2. **坐标系变换**：`p_world = Rz * p_planner + T`（默认 Rz = 恒等）
3. **按 sequence 顺序逐点发布** `PoseStamped` 到 `/waypoint_pose`
   （world ENU，`orientation` 由 `heading_deg` 换算 yaw：`yaw = heading_deg * π/180`）
4. **尊重航段语义**：
   - 每个 `turn_in_place=true` 的航点：发布后等 `hold_time_s` 再发下一点
     （本仓库 offboard 的 `waypoint_hold_time` 是全局限定，建议在适配器内做）
   - 不跳过 `obstacle_avoidance` 段（SUPER 还会在航段间做局部避障）
5. **进度反馈**：订阅 `/waypoint_markers`（绿=排队、黄=执行中）或
   `/lidar_slam/odom`，确认航点到达后再发下一个（防止 `/waypoint_pose` 缓冲溢出）
6. 到达最后一个航点（`speed_mps == 0` 的 `return_home` 终点）后：
   调用 `ros2 service call /offboard/land std_srvs/srv/Trigger` 降落

骨架代码：

```python
#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CspAdapter(Node):
    def __init__(self, plan_path: str, t_x: float, t_y: float, t_z: float):
        super().__init__("csp_adapter")
        self.pub = self.create_publisher(PoseStamped, "/waypoint_pose", 10)
        plan = json.load(open(plan_path))
        assert plan["schema_version"] == "3.0"
        assert plan["summary"]["mission_status"] == "ready", "计划不可执行"
        # 坐标系变换 T（见第 3 节）
        self.T = (t_x, t_y, t_z)
        self.waypoints = plan["waypoints"]          # 已按 sequence 排序
        self.turn_points = {w["id"]: w for w in self.waypoints if w["turn_in_place"]}
        self.timer = self.create_timer(0.5, self.tick)  # ≥0.5s 发布间隔（QoS 约束）
        self.idx = 0

    def tick(self):
        if self.idx >= len(self.waypoints):
            self.timer.cancel()
            self.get_logger().info("全部航点已发布完毕")
            return
        wp = self.waypoints[self.idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = wp["x"] + self.T[0]
        msg.pose.position.y = wp["y"] + self.T[1]
        msg.pose.position.z = wp["z"] + self.T[2]
        yaw = math.radians(wp["heading_deg"])
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)
        self.get_logger().info(f"waypoint #{self.idx+1}/{len(self.waypoints)} "
                               f"({wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f})")
        # turn_in_place 航点：等待 hold_time_s 再推进（简化：暂停 1 次发布周期）
        if wp["id"] in self.turn_points and self.turn_points[wp["id"]]["hold_time_s"] > 0.5:
            self.timer.cancel()
            self.create_timer(self.turn_points[wp["id"]]["hold_time_s"], self.resume)
            return
        self.idx += 1

    def resume(self):
        self.idx += 1
        self.timer.reset()

def main():
    rclpy.init()
    node = CspAdapter("/path/to/flight_plan.json", t_x=0.0, t_y=0.0, t_z=0.0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
```

> `speed_mps` 说明：本仓库的 offboard + SUPER 按自身轨迹优化飞行，**不消费目标
> 速度**。若要严格限制航段速度，请在适配器内按航段拆分发布节奏（如每
> `segment_length / speed_mps` 秒发一点），或用 `planner_config` 的
> 速度上限约束 SUPER。上游规划时也可把 `coverage_speed_mps` 等设为本栈
> 可接受的值。

## 5. 集成运行步骤

```bash
# ① 生成飞行计划（上游仓库，离线）
cd coverage-search-planner
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
# 检查 results/example_run/flight_plan.json 的 mission_status == ready

# ② 启动仿真栈（终端 1）：Gazebo + PX4 + agent + bridge
./utils/start_sim.sh

# ③ 启动 offboard + SUPER + FAST-LIO + RViz（终端 2）
#    （offboard.launch.py 现直接拉起 FAST-LIO）
source install/setup.bash
ros2 launch offboard offboard.launch.py

# ④ 等待起飞完成（日志出现 State: TAKEOFF → IDLE）

# ⑤ 标定坐标系 T（第 3 节），然后启动适配器（终端 3）
source install/setup.bash
python3 src/csp_adapter/csp_adapter.py \
  --plan results/example_run/flight_plan.json \
  --offset-x <T_x> --offset-y <T_y> --offset-z <T_z>

# ⑥ 结束：适配器发完最后一个航点后，手动或自动降落
ros2 service call /offboard/land std_srvs/srv/Trigger
```

## 6. 注意事项

- **QoS**：`/waypoint_pose` 订阅端为 `best_effort` + `keep_last(1)`，
  发布间隔 ≥ 0.5 s，并确认 offboard 日志出现 `Waypoint buffered (#N)`；
  缓冲队列是 FIFO，逐点执行（`waypoint_reached_dist` 判定到达）。
- **不要改协议语义**：不按航点推断检测状态、不跳过航段、不自动改变
  ENU 轴方向/单位；`mission_status != ready` 时禁止执行。
- **高度一致性**：planner 的 `flight_altitude_m`（示例 25 m）应满足
  SUPER 的飞行能力；若太高，先在 `planner_config.yaml` 中调低
  （如 5–8 m），并与 `offboard.yaml` 的 `goal_height` / `default_height` 保持一致。
- **避障**：SUPER 会在相邻航点间做局部避障（ROG-Map 障碍 + 轨迹优化），
  但它是**两点间**的避障，不是全局重规划；`obstacle_avoidance` 段的绕障
  路径来自上游规划，务必保留。
- **双机**：上游双机计划（`mission_manifest.json`）不在此对接范围；
  本仓库单机执行，双机时空避碰需上层另行解决。
