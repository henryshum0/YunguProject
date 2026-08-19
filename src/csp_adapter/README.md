# csp_adapter

把 coverage-search-planner 生成的 `flight_plan.json`（协议 v3.0，ENU 米制）
接入本仓库的 offboard 航点链路：逐点发布到 `/waypoint_pose`（world ENU），
由 offboard 缓冲 + SUPER 避障 + PX4 OFFBOARD 执行，终点自动降落。

对接设计、坐标标定方法与校验规则见仓库根目录
`docs/coverage-search-integration.md`。

## 用法

```bash
# 直接运行（推荐，标定偏移后）
python3 src/csp_adapter/csp_adapter/csp_adapter.py \
  --plan-path results/example_run/flight_plan.json \
  --offset-x <T_x> --offset-y <T_y> --offset-z <T_z>

# 或通过 launch（等价参数经环境变量）
CSP_PLAN_PATH=results/example_run/flight_plan.json \
CSP_OFFSET_X=-157.4 CSP_OFFSET_Y=-69.2 \
ros2 launch csp_adapter csp_adapter.launch.py
```

## 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `--plan-path` | 必填 | `flight_plan.json` 路径（`mission_status=ready` 才执行） |
| `--offset-x/y/z` | 0 | 坐标系平移 `T`（标定方法见对接文档第 3 节） |
| `--yaw-deg` | 0 | 地图→world 绕 z 附加旋转（地图与 world 轴不一致时） |
| `--interval` | 0.5 | 航点发布间隔 s（≥0.5，匹配 QoS 约束） |
| `--no-land` | off | 发完最后航点不自动调 `/offboard/land` |
| `--land-delay` | 5.0 | 最后航点发出后多久调用降落服务（s） |

## 行为要点

- 校验 `schema_version=="3.0"`、`coordinate_frame=="ENU"`、
  `summary.mission_status=="ready"`、航段首尾连通，不满足直接拒绝启动；
- 发布端 QoS 为 best_effort（与 `goal_marker_node` 的订阅匹配），
  间隔强制 ≥0.5 s 避免丢点；
- `turn_in_place=true` 的航点发布后暂停 `hold_time_s` 再发下一点；
- `speed_mps` 下游不消费（offboard+SUPER 按自身轨迹飞行），要限速请在
  SUPER 的 planner config 里设速度上限，不要把发布节奏当限速；
- 高度 >20 m 或速度 >4 m/s 的航点只打警告不阻断，按实际载具能力自行判断。
