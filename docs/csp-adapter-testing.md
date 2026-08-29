# csp_adapter 测试指南

本文档说明如何测试 [csp_adapter](../src/csp_adapter/README.md)（覆盖航线计划
`flight_plan.json` → `/waypoint_pose` 适配器）以及它与 offboard 航点链路的
完整集成。对接设计见 [coverage-search-integration.md](coverage-search-integration.md)。

测试分三层，从快到慢：

1. **单元测试** — 纯函数（坐标变换、计划校验），秒级完成，不需要仿真；
2. **离线冒烟测试** — 用构造的计划文件验证节点启动与拒绝逻辑，不连仿真；
3. **仿真集成测试** — 完整链路（规划器 → 适配器 → offboard → SUPER → PX4），
   需要启动整个仿真栈。

---

## 1. 单元测试（不需要仿真）

```bash
cd src/csp_adapter
python3 -m pytest test/test_csp_adapter.py -v
```

预期 **8 个测试全部通过**，覆盖：

| 测试 | 验证内容 |
|---|---|
| `test_translation_only` | 纯平移：`p_world = p_planner + T` |
| `test_rotation_90deg` | 绕 z 旋转 + 平移的组合变换 |
| `test_rotation_preserves_length` | 旋转不改变距离 |
| `test_yaw_to_quat` | `heading_deg` → 绕 z 四元数 |
| `test_valid_plan` | 合法计划通过校验 |
| `test_rejects_wrong_schema` | 非 3.0 协议被拒绝 |
| `test_rejects_non_ready` | `mission_status != ready` 被拒绝 |
| `test_rejects_disconnected_segments` | 航段不连通被拒绝 |

## 2. 离线冒烟测试（不需要仿真）

验证节点入口与**拒绝逻辑**（不做任何飞行）。

### 2.1 入口可用

```bash
source install/setup.bash
ros2 run csp_adapter csp_adapter --help
```

应打印参数列表（`--plan-path`、`--offset-x/y/z`、`--yaw-deg`、`--interval`、
`--no-land`、`--land-delay`）。

### 2.2 合法计划：能启动、能发点

用规划器生成一份真实计划（见 3.1），或手工构造一份最小计划：

```json
{
  "schema_version": "3.0",
  "coordinate_frame": "ENU",
  "units": "meters",
  "map_id": "test_local_origin",
  "waypoints": [
    {"id": 1, "sequence": 1, "x": 0.0, "y": 0.0, "z": 5.0,
     "heading_deg": 0.0, "speed_mps": 0.0, "turn_in_place": false, "hold_time_s": 0.0},
    {"id": 2, "sequence": 2, "x": 10.0, "y": 0.0, "z": 5.0,
     "heading_deg": 90.0, "speed_mps": 3.0, "turn_in_place": false, "hold_time_s": 0.0},
    {"id": 3, "sequence": 3, "x": 10.0, "y": 10.0, "z": 5.0,
     "heading_deg": 180.0, "speed_mps": 3.0, "turn_in_place": true, "hold_time_s": 2.0}
  ],
  "route_segments": [
    {"segment_id": 1, "kind": "coverage_lane",
     "start_waypoint_id": 1, "end_waypoint_id": 3, "detection_enabled": true}
  ],
  "summary": {"mission_status": "ready"}
}
```

存为 `/tmp/test_plan.json`，然后（`--no-land` 避免真实降落调用）：

```bash
source install/setup.bash
ros2 run csp_adapter csp_adapter \
  --plan-path /tmp/test_plan.json --offset-x -4.0 --offset-y -2.0 \
  --interval 0.5 --no-land
```

预期日志：

```text
plan /tmp/test_plan.json: 3 waypoints, 1 segments, 1 turn_in_place points, mission_status=ready
transform: Rz(0.0 deg) + T=(-4.0, -2.0, 0.0)
waypoint 1/3 id=1 world=(-4.0, -2.0, 5.0) heading=0.0 deg, speed=0.0 m/s
turn_in_place at waypoint 3: holding 2.0 s
...
all 3 waypoints published
```

（若当前没有 `/waypoint_pose` 订阅者，日志仍会打印，只是发出去的点没人收。）

### 2.3 非法计划：应拒绝启动

把 2.2 的计划里 `mission_status` 改成 `infeasible_coverage` 再跑，预期**立即报错退出**：

```text
ValueError: flight_plan.json validation failed:
  - summary.mission_status must be 'ready', got 'infeasible_coverage'
```

同样可验证：`schema_version` 改成 `"2.0"`、把 `route_segments` 改成不连通的
（`start_waypoint_id` 对不上上一段 `end_waypoint_id`）都应被拒绝。

## 3. 仿真集成测试（完整链路）

### 3.1 生成飞行计划（离线，一次）

在 `uav-coverage-route-planner/` 里：

```bash
uv sync
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
```

**检查** `results/example_run/flight_plan.json` 的 `summary.mission_status == "ready"`
（`infeasible_coverage` 禁止执行，需要调参数重新规划）。

### 3.2 启动仿真栈

```bash
# 终端 1：模拟器层
./utils/start_sim.sh

# 终端 2：感知 + 规划层（含 offboard + SUPER + RViz）
./utils/start_fastlio.sh
```

等 offboard 日志出现 `State: TAKEOFF → IDLE`（起飞完成、进入空闲态）。

### 3.3 标定坐标系 T

起飞瞬间的 `T + start` 可以用起飞点的 `/lidar_slam/odom` 读数近似（方法 B）：

```bash
ros2 topic echo /lidar_slam/odom --qos-reliability best_effort --spin-time 1
```

记下位置 `p_world`，计划里 `start` 是 `planner_config.yaml` 的 `start` 字段。
`T = p_world - start`。模型 spawn 在世界原点，起飞点读数即 `(0.0, 0.0, ~1.15)`；
以 `start=[153.4, 67.2, 25.0]` 为例，则 `T = (-153.4, -67.2, ~-23.85)`。

> 验证 T 是否正确：把 2.2 的 `test_plan.json` 换成一两个点（比如把 `start` 作为
> 唯一航点）跑一遍，看飞机是否悬停在起飞点上方 0 误差处。或者看 RViz 里
> `/waypoint_markers` 的绿色排队点是否落在 birdview 地图的正确位置。

### 3.4 启动适配器

```bash
# 终端 3
source install/setup.bash
python3 src/csp_adapter/csp_adapter/csp_adapter.py \
  --plan-path results/example_run/flight_plan.json \
  --offset-x <T_x> --offset-y <T_y> --offset-z <T_z>
```

### 3.5 观察与验收清单

| 观察点 | 方法 | 预期 |
|---|---|---|
| 适配器逐点发布 | 终端 3 日志 | `waypoint #N/N ... world=(...)` 每 ≥0.5s 一条 |
| 航点进入缓冲 | offboard 日志 | 出现 `Waypoint buffered (#N)`，N 递增 |
| 缓冲可视化 | RViz（birdview 窗） | 绿色 = 排队、黄色 = 执行中 |
| 飞机实际飞行 | RViz freelook / 鸟瞰图 | 沿航线移动，到达点后悬停再走下一段 |
| 到达判定 | offboard 日志 | 按 `waypoint_reached_dist`（默认 3.0 m）判定到达 |
| turn_in_place | offboard 日志 | 转向点在原地悬停 `hold_time_s` 再出发 |
| 绕障 | RViz | `obstacle_avoidance` 段保留绕行路径（SUPER 只做局部避障） |
| 降落 | 适配器日志 | 最后航点发出后 `--land-delay` 秒调用 `/offboard/land` |
| 轨迹对比（可选） | RViz `/gt_path` vs 航线 | 实际飞行与计划航线贴合 |

**重点检查**：飞机应完整执行 `obstacle_avoidance` 段，不允许 SUPER 抄近路穿过
`detection_enabled=false` 也要保留的绕障段；覆盖任务结束时自动降落而非悬停。

### 3.6 手动结束

如果适配器用了 `--no-land` 或降落失败：

```bash
ros2 service call /offboard/land std_srvs/srv/Trigger
```

## 4. 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| 适配器拒绝启动（ValueError） | 计划非法（schema/status/连通性） | 按报错修计划或重新规划 |
| 日志打印但飞机不动 | 起飞未完成（不在 IDLE 态）；或 QoS 不匹配 | 等 `State: IDLE`；检查发布端是 best_effort（不要手工把代码改成 reliable） |
| 航点丢失、路线跳点 | 发布间隔 <0.5 s | 检查 `--interval`（节点会强制 ≥0.5） |
| 飞到错误位置 | `T` 标定错了 | 用 3.3 的验证方法重新标定；确认地图 y 轴与 world 一致（不一致用 `--yaw-deg`） |
| 速度比计划快 | offboard+SUPER 不消费 `speed_mps` | 在 `config/super_planner/` 的 planner config 里设速度上限；不要用发布节奏限速 |
| 高度太高飞不了 | 计划 25 m 超出 SUPER 能力 | 调低 `planner_config.yaml` 的 `flight_altitude_m`（如 5–8 m），与 `offboard.yaml` 的 `goal_height` 保持一致后重新生成计划 |
| 自动降落没触发 | `/offboard/land` 服务名不对或不在线 | 手动调用 3.6 的服务；确认 offboard 节点存活 |

## 5. 验收速查

- [ ] `pytest` 8/8 通过
- [ ] `--help` 正常
- [ ] 合法计划能逐点发布（≥0.5 s 间隔、best_effort QoS）
- [ ] 非法计划（schema/status/连通性）被拒绝
- [ ] 仿真中 `Waypoint buffered (#N)` 递增
- [ ] 飞机沿计划航线飞行、turn_in_place 原地转向
- [ ] 绕障段完整执行
- [ ] 终点自动（或手动）降落
- [ ] `mission_status != ready` 的计划从未被执行
