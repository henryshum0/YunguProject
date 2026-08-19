# 无人机全覆盖航道生成与路径优化器

Coverage Generation and Route Optimization Planner是一个面向无人机园区连续视频检测任务的纯 Python 规划器。目标物是地面上
静止的人和车辆；识别算法本身不在本仓库范围内。理想检测模型规定：目标完整进入相机有效视野一次，即认为可检测。规划采用 2.5D 几何：目标位于地面，相机在固定高度飞行，建筑高度和墙面会遮挡其后的地面。

项目使用本地 ENU 米制坐标，不依赖 ROS、PX4、Gazebo、MAVROS 或 YOLO。规划结果通过版本化 JSON/YAML 协议交给下层飞控适配器。

项目的研究问题统一拆分为两个核心子问题：

```text
Coverage Generation（怎么保证扫全？）
  ├─ Global Scanline：全局铺扫描线，再由边界与障碍裁剪
  └─ Cellular Decomposition：先以 BCD 分解 cells，再生成 cell 内 lanes
                         ↓
              标准化 RouteOptimizationProblem
                         ↓
Route Optimization（扫全以后怎么飞得更优？）
  └─ lane ordering + lane orientation + obstacle-aware transition cost
     Greedy → 2-opt / Or-opt → GTSP / MILP
```

Coverage Generation 必须先满足每个可搜索 patch 的全覆盖约束；Route Optimization
只能在保持覆盖与安全可行的前提下降低空驶、转场、转弯和总任务代价。

## 快速验收

### 1. 克隆并安装

环境要求：Git、[`uv`](https://docs.astral.sh/uv/) 和 Python 3.12。依赖已经锁定在 `uv.lock`，不要安装到系统 Python。

```bash
git clone https://github.com/Jocelyn-2005/coverage-search-planner.git
cd coverage-search-planner
git switch feature/continuous-lane-planning
uv sync
uv run pytest -q
```

预期测试结果为全部通过。

### 2. 打开 Web 验收页面

```bash
uv run uvicorn coverage_planner.web:app --host 127.0.0.1 --port 8000
```

浏览器打开 <http://127.0.0.1:8000>。页面支持：

- 分别绘制两架无人机互不重叠的责任区并调整各自起降点；
- 在 Global Scanline 与 Cellular Decomposition（BCD）之间选择 Coverage Generation 方法；
- 默认使用 Local insertion 完成覆盖补全，并可切换 Full Greedy 进行对照；
- 配置 sweep direction（可留空自动优化）和建筑安全距离；
- 配置相机视场角、目标包络、画面边缘余量和视频分析采样率；
- 配置覆盖、连接、避障速度和飞控途径点最大间距；
- 查看带 ENU 米制刻度的地图、语义建筑、搜索网格、飞行路线和任务回放；
- 下载连续飞行计划和覆盖报告，并生成用于地图分析的 GeoJSON 文件。

验收建议：绘制两个不重叠责任区，点击“生成并优化两架无人机航线”，确认出现两条独立闭合路线；有效检测视野不得覆盖建筑占地及被墙面遮挡的地面。回放仅表示两机同时启动，不进行时空碰撞规避。

Web 红色外框表示航拍底图的有效有色内容边界，黄色虚线表示语义地图的可请求
搜索边界。横轴为 ENU 东向、纵轴为 ENU 北向；每 10 米一个小刻度，每 50 米一个
带数值的主刻度。扫描方向是平行扫描线在地面上的延伸方向，并非相机俯仰角：
`0°` 为南北向扫描线，`90°` 为东西向扫描线；相机当前始终垂直向下。

### 3. 命令行生成验收结果

```bash
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
```

结果写入 `results/example_run/`。静态图可直接打开：

```text
results/example_run/visualization.png
```

## 输入

一次规划由三类输入组成。

| 输入 | 示例 | 含义 |
|---|---|---|
| 语义地图 | `semantic_map.json` | 地图边界、建筑矩形、建筑高度和类别 |
| 用户搜索区域 | `search_area.geojson` 或 Web 手工绘制 | 每架无人机负责检测的 Polygon/MultiPolygon |
| 规划配置 | `planner_config.yaml` | 起点、高度、相机、重叠率、安全距离、速度和采样参数 |

配置字段 `coverage_generation_method` 选择 `global_scanline` 或 `bcd`；字段
`completion_strategy` 默认为 `local_insertion`，可显式设为 `full_greedy`；字段
`scan_pattern` 仅作为旧版本兼容入口，不应再用于新配置。

核心坐标约定：

- 坐标系为局部 ENU；`x` 向东，`y` 向北，`z` 向上；
- 距离、高度和坐标单位均为米；
- 像素坐标只用于经过标定的 Web 显示，不参与核心规划；
- 当前版本只支持固定高度、垂直向下相机，即 `pitch_deg = -90`。

有效搜索地面定义为：

```text
用户搜索区域 ∩ 地图边界 − 建筑占地 − 显式排除区域
```

建筑是否构成飞行障碍由飞行高度、建筑高度、垂直净空和 `allow_overflight_above_buildings` 共同决定。规划器不会自动升高无人机。

云谷示例另外保存了从最终 Blender 视觉网格测得的保守安全体。航点规划使用安全体而不是偏小的原始碰撞核心。经任务口径确认，`collider_building9.001` 和 `collider_building9.002` 均按从地面开始的实体建筑处理；两者之间人工确认的不可达区域记录在 `excluded_search_regions`，不生成检测航线且不计入覆盖率分母。

完整地图格式见 [语义地图格式](docs/semantic-map-schema.md)。

## 输出

| 文件 | 使用方 | 作用 |
|---|---|---|
| `flight_plan.json` | 下层飞控适配器，首选 | 连续视频检测飞行协议 v3 |
| `flight_plan.yaml` | 人工审阅/飞控适配器 | 与 JSON 等价的 YAML |
| `coverage_report.json` | 验收与实验统计 | 初始候选、最终覆盖、航程、非作业距离和未覆盖网格 |
| `patches.geojson` | 地理信息（GIS）/覆盖分析 | 每个地面网格的覆盖状态 |
| `route.geojson` | 地理信息（GIS）/地图显示 | 任务路线折线 |
| `visualization.png` | 人工验收 | 静态规划结果图 |

双机 Web 任务在根目录额外输出 `mission_manifest.json`，并在 `drone_1/`、
`drone_2/` 下分别保存两架无人机的计划和验收文件。清单中的 `mission_status=ready`
仅表示两架飞机各自在人工责任区内的每个可搜索 patch 均达到 `99.99%` 数值覆盖
阈值，不代表责任区并集覆盖整张地图，
也不代表已经完成双机时空避碰。

覆盖报告区分两个阶段：`initial_candidate_metrics` 是补漏和连续视野复核前的初始
候选指标，`final_solution_metrics` 才是最终航线指标。
`unreachable_candidate_point_ids` 表示初始候选点不可安全到达，但可能已经由其他
视野补偿；真正仍未达到覆盖要求的地面只由 `unreachable_patch_ids` 表示。

### 给下层飞控的文件

下层飞控读取 `flight_plan.json`。协议包含：

- `waypoints`：均匀细分后的 ENU 飞控途径点，带高度、航向和目标速度；
- `route_segments`：相邻途径点之间的直线航段；
- `kind`：`coverage_lane`、`connector`、`obstacle_avoidance` 或 `return_home`；
- `detection_enabled`：该航段的视频结果是否计入本责任区检测；
- `lanes`：覆盖航线与其航段编号；
- `control_point_spacing_m`：途径点最大间距；
- `summary`：覆盖与任务效率指标。

低于最低覆盖率的结果会标记为 `mission_status=infeasible_coverage`，不得直接下发执行。尖锐折返航点通过 `turn_in_place=true` 和 `hold_time_s` 要求多旋翼先稳定悬停、原地调整航向，再进入下一航段。

飞控适配器依序跟踪 `waypoints`，相机提供连续视频流；`route_segment.detection_enabled` 只控制检测结果是否计入任务，不是相机快门触发指令。`video_detection.analysis_rate_hz` 是规划时连续视野积分的离散采样率，也可供下游选择视频推理帧率。控制点不是检测触发点。

详细字段和执行状态机见 [下层飞控接口](docs/flight-controller-interface.md)。

## 方法概览

第一层 Coverage Generation 根据地图、相机 footprint、overlap 和安全距离生成满足
全覆盖约束的 lanes。当前并列实现 Global Scanline 与 Cellular Decomposition（BCD）。
第二层 Route Optimization 在 lanes 固定后联合选择 lane ordering 和 lane orientation，
使用障碍感知 transition cost 连接相邻任务。连续视频视野复核通过后，路线才会细分
成下层飞控途径点。

当前 Route Optimization 决定 lane 的访问顺序和正反方向，以障碍感知的连接距离与返航距离为优化目标。覆盖率、安全净空和任务闭合属于硬约束。任务时间、重复覆盖、转弯和能耗等多目标联合优化不属于当前实现。

最终连续视野复核发现残余漏扫时，默认采用 `local_insertion`：保持主 coverage lanes
的顺序和方向不变，只在 lane 边界按障碍感知的最小航程增量插入补全点。该策略避免
补全阶段反复重排整条路线，主要用于降低完整任务的规划时间。`full_greedy` 保留为
可选对照，会在加入补全点后重新组织全部任务点，计算成本通常更高。两种策略均执行
安全连接重建和最终逐 patch 覆盖验收，选择默认策略不改变覆盖判定标准。

- [问题定义与数学模型](docs/optimization-model.md)
- [几何与优化算法设计](docs/algorithm-design.md)
- [Coverage Generation × Route Optimization Benchmark](docs/benchmark-design.md)
- [下层飞控接口](docs/flight-controller-interface.md)
- [后续设计边界](docs/future-design.md)

## 当前能力边界

当前版本是确定性的分层启发式求解器，不宣称获得全局最优解。它已实现双机人工责任区、固定高度正射覆盖、目标完整入镜约束、建筑墙体遮挡、往复式扫描、建筑高度相关避障、可见图最短路、连续视频覆盖评估和飞控途径点导出。暂不处理运动目标、双机时空碰撞、地形跟随、斜视相机、任意建筑网格以及动力学平滑。

项目并列提供两种 Coverage Generation 方法：`GlobalScanlineGenerator` 先生成全局平行
扫描线，再由搜索边界、建筑和安全区裁剪成 lane；`BCDGenerator` 先在扫描拓扑发生
分裂或合并的位置构造单调 cell，再在各 cell 内生成往复式 lane。二者不存在升级或
替代关系，均输出统一的 `CoveragePlan`，再转换为 `RouteOptimizationProblem`。
Route Optimization 对外固定使用 `auto`：不超过 12 条 coverage lanes 时采用
Exact；更大任务采用 Greedy 初始化后的 2-opt + Or-opt 组合启发式。扫描方向自动时
只比较南北 0° 与东西 90°。Coverage Completion 默认采用 `local_insertion`；
`full_greedy` 仅作为显式选择的高成本对照。内部求解器仍用于测试和算法对比。
