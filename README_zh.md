# YunguProject — 云谷无人机仿真平台

基于 ROS 2 (Humble) + PX4 Autopilot + Gazebo (Harmonic) 的四旋翼仿真与真机部署平台，
集成 SUPER 运动规划器，并支持 FAST-LIO 激光惯性里程计与 PX4 EKF2 的紧耦合融合。

## 安装

```bash
# 克隆（含 submodule）
git clone --recursive https://github.com/henryshum0/YunguProject.git
cd YunguProject

# 安装依赖
./install_deps.sh

# 编译
colcon build --symlink-install

# 运行 yungu 地图前，将 yungu.glb 文件放到
# VisionFlow-PX4/Tools/simulation/gz/worlds 目录下
```

## 运行仿真

完整仿真栈（Gazebo + PX4 SITL + MicroXRCEAgent + gz bridge）通过一个脚本启动：

```bash
# 终端 1 — 仿真器 + agent + bridge
./src/utils/start_sim.sh

# 终端 2 — offboard 状态机 + SUPER 规划器 + RViz
source install/setup.bash
ros2 launch offboard offboard.launch.py
```

### 仿真配置（`config/simulation.yaml`）

车辆的**模型**、Gazebo 的**地图/世界**、GZ 版本、uXRCE-DDS 端口、
以及 GZ→ROS **桥接话题**都在 [`config/simulation.yaml`](config/simulation.yaml)
中配置——这是仿真的唯一事实来源。修改后直接运行 `start_sim.sh`（无需参数）：

| 键               | 说明                                                              |
|-------------------|------------------------------------------------------------------|
| `model`           | Gazebo 车辆模型 / PX4 gz 机型（如 `swan_gamma_v2`）                 |
| `world`           | Gazebo 地图（`.sdf` 位于 `VisionFlow-PX4/Tools/simulation/gz/worlds`，如 `yungu`）|
| `gz_version`      | Gazebo 发行版（`harmonic`）                                      |
| `xrce_port`       | MicroXRCEAgent 的 UDP 端口                                       |
| `bridge.enabled`  | 是否启动 GZ↔ROS `parameter_bridge`                               |
| `bridge.tf_enabled` | 是否启动 TF 桥（`tf_bridge.py`）                               |
| `bridge.topics`   | 桥接到 ROS 2 的 GZ 话题（如 `/swan_gamma_v2/scan`、`/swan_gamma_v2/scan/points`）|

`model` + `world` 组合成 PX4 make 目标 `gz_<model>_<world>`
（例如 `gz_swan_gamma_v2_yungu`）。查看生效值与完整模型/地图列表：

```bash
./src/utils/start_sim.sh --help
```

### 环境变量覆盖

每个配置值都可以在命令行覆盖（无需改文件）：

| 变量         | 默认值                  | 说明                                          |
|--------------|------------------------|----------------------------------------------|
| `PX4_MODEL`  | 来自 config（`swan_gamma_v2`） | Gazebo 车辆模型 / `make px4_sitl` 机型       |
| `PX4_WORLD`  | 来自 config（`yungu`）  | Gazebo 地图/世界（worlds 目录下的 .sdf）       |
| `XRCE_PORT`  | 来自 config（`8888`）   | MicroXRCEAgent 的 UDP 端口                    |
| `GZ_VERSION` | 来自 config（`harmonic`）| gz-transport 版本（gz-sim 8）                 |
| `HEADLESS`   | *(未设置)*              | 任意非空值（如 `1`）以无 GUI 方式运行 Gazebo（仅服务器）|

```bash
# 不改配置，使用不同机型/世界
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./src/utils/start_sim.sh

# 旧式写法：直接传完整 make 目标
PX4_MODEL=gz_swan_gamma_v2_yungu ./src/utils/start_sim.sh

# 使用非默认 uXRCE-DDS 端口（必须与 PX4 UXRCE_DDS_PRT 一致）
XRCE_PORT=2018 ./src/utils/start_sim.sh

# 无 GUI 运行 Gazebo（仅物理服务器）——适合 CI 或无显示器环境
HEADLESS=1 ./src/utils/start_sim.sh
```

### `start_sim.sh` 启动的内容

| # | 组件 | 命令 |
|---|---|---|
| 1 | PX4 SITL + Gazebo | `make px4_sitl gz_<model>_<world>`（来自配置）|
| 2 | MicroXRCEAgent | `MicroXRCEAgent udp4 -p <xrce_port>`（来自配置）|
| 3 | GZ ↔ ROS bridge + TF | `src/utils/gz_bridges/bridge.sh`（话题来自配置）|

`start_sim.sh` 等待 PX4 报告 "Ready for takeoff"，然后启动 agent 和 bridge。
日志位于 `/tmp/yungu_sim/`（`px4_sitl.log`、`xrce_agent.log`、`bridge.log`）。

**停止：** 按 `Ctrl+C` 停止所有进程（PX4 窗口关闭，所有子进程被杀）。
如有残留——例如 PX4 分离到独立会话的 `gz-server`——运行：

```bash
./src/utils/stop_sim.sh
```

### 桥接与可视化

- gz bridge 与 RViz 相互独立：可单独运行 `src/utils/gz_bridges/bridge.sh`
  （读取 `config/simulation.yaml` 中的话题），再通过 offboard launch 文件单独启动 RViz。
- RViz 显示无人机（TF）、激光点云，以及 "2D Goal Pose" 工具——
  该工具向 `/goal_pose` 发布目标点（默认 `rviz:=true`，可用 `rviz:=false` 关闭）。
- SUPER 规划器由 `offboard.launch.py` 启动，使用
  `config/offboard.yaml` 中的 `planner_config`（在 `config/super_planner/`
  下解析），订阅 super_bridge 输出（`/cloud_registered`、`/lidar_slam/odom`）。

## FAST-LIO 模式（激光惯性里程计 + PX4 EKF2 融合）

一键启动脚本，运行包含 **FAST-LIO** 的完整栈——FAST-LIO 里程计送入 PX4 的
EKF2（外部视觉融合），融合后的 PX4 里程计再经 `super_bridge` 驱动 SUPER 规划器。
该架构与真机一致（不依赖 Gazebo 真值）：

```
FAST-LIO /Odometry ──→ fastlio_px4_bridge ──→ /fmu/in/vehicle_visual_odometry
                                                     ↓ PX4 EKF2 融合
                        PX4 /fmu/out/vehicle_odometry ──→ super_bridge
                                                     ↓
                        /lidar_slam/odom + /cloud_registered ──→ planner
```

```bash
# 完整栈：GPU 强制 Gazebo + PX4 + FAST-LIO + EKF2 融合 + 规划器 + RViz
./src/utils/start_fastlio.sh

# Gazebo 仅服务器（无 GUI）——RViz 仍会打开以查看规划可视化
HEADLESS=1 ./src/utils/start_fastlio.sh

# 完全跳过 RViz
NO_RVIZ=1 ./src/utils/start_fastlio.sh
```

`src/utils/start_fastlio.sh` 相对 `src/utils/start_sim.sh` 新增的组件：

| # | 组件 | 说明 |
|---|---|---|
| 1 | `src/utils/fastlio/gazebo_imu_bridge.py` | PX4 `sensor_combined`（FRD）→ `/livox/imu`（ENU）：轴翻转 + 滚动时间同步 |
| 2 | `src/utils/fastlio/add_time_field.py` | 为 Gazebo 点云补充 `time` 字段（瞬时扫描填 0）|
| 3 | `fast_lio`（`fastlio_mapping`）| 激光惯性里程计，配置：`config/fastlio_swan_gamma_effect.yaml`（ikd-Tree 增量地图 → `/cloud_effected`）|
| 4 | `src/utils/fastlio/fastlio_px4_bridge.py` | FAST-LIO `/Odometry`（ENU）→ `/fmu/in/vehicle_visual_odometry`（NED）供 EKF2 使用 |
| 5 | `super_bridge` | PX4 融合里程计 + 原始点云 → `/lidar_slam/odom` + `/cloud_registered` |

PX4 EKF2 外部视觉参数在 swan_gamma_v2 机型中设置（`4007_gz_swan_gamma_v2`）：
`EKF2_EV_CTRL 13`（水平位置+速度+偏航融合——**不含 VPOS**：FAST-LIO 无绝对
高度参考，垂直通道交给气压计，SLAM 的 z 漂移不会传导到飞行高度）、
`EKF2_EV_DELAY 5`、`EKF2_EVP_NOISE 0.1`、`EKF2_EVV_NOISE 0.1`、
`EKF2_EVA_NOISE 0.05`。

**验证融合链路是否正常运行：**

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once --qos-reliability best_effort
ros2 topic echo /lidar_slam/odom --once --qos-reliability best_effort
ros2 topic echo /cloud_registered --once --qos-reliability best_effort
```

**无GUI的fastlio启动**
```
HEADLESS=1 ./src/utils/start_fastlio.sh
```

## 点对点导航接口（供搜索算法开发者使用）

本节是搜索/规划算法需要使用的**唯一**接口。FAST-LIO 栈启动后，无人机自动起飞
到 `default_height` 并进入 `IDLE` 状态；此后任何 ROS 2 节点只需发布
`geometry_msgs/PoseStamped` 即可指挥无人机飞行——无需改任何 launch 文件或代码。

### 快速开始

```bash
# 终端 1 — 完整 FAST-LIO 仿真栈（Gazebo + PX4 + FAST-LIO + EKF2）
./src/utils/start_fastlio.sh

# 终端 2 — offboard 状态机 + SUPER 规划器 + RViz
source install/setup.bash
ros2 launch offboard offboard.launch.py
```

等待起飞完成（offboard 日志出现 `State: TAKEOFF → IDLE`），即可发布航点。
RViz 的 "2D Goal Pose" 工具已改向发布到 `/waypoint_pose`（在
`birdview.rviz` / `freelook.rviz` 中配置），所以手动点目标与算法发目标走的是
同一条路径。

### 目标输入话题

| 话题 | 类型 | QoS | 用途 |
|---|---|---|---|
| `/waypoint_pose` | `geometry_msgs/PoseStamped` | 订阅端 best_effort/volatile | **批量航点输入（推荐）**。每条消息进入 offboard 航点缓冲队列，逐点执行。 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 订阅端 best_effort/volatile | **直接单点目标**。与 SUPER click-goal 订阅同一话题；offboard 内部也在此发布，把当前航点逐个交给 SUPER。 |
| `/waypoint_buffer` | `geometry_msgs/PoseStamped` | reliable | 内部通道（`goal_marker_node` → offboard 节点）。不要直接往这里发。 |
| `/waypoint_markers` | `visualization_msgs/MarkerArray` | transient_local | 航点缓冲反馈：绿球 = 排队中，黄球 = 正在执行，青色线 = 航线。 |

数据流：

```
    /waypoint_pose ──→ goal_marker_node ──→ /waypoint_buffer ──→ offboard 节点
  (你的搜索算法)                              (FIFO 队列)              │
                                                                      ↓
                                                       /goal_pose（逐个发送）
                                                                      ↓
                                                          fsm_node（SUPER）
                                                                      ↓
                                                    /planning/pos_cmd ──→ offboard ──→ PX4
```

发布示例（Python）：

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        self.pub = self.create_publisher(PoseStamped, "/waypoint_pose", 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"          # ENU 世界系，z 向上
        msg.pose.position.x = 10.0             # 东
        msg.pose.position.y = 5.0              # 北
        msg.pose.position.z = 5.0              # 高 [m]
        msg.pose.orientation.w = 1.0           # 偏航默认朝飞行方向
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(GoalPublisher())
```

**QoS 注意：** `/waypoint_pose` 的订阅端是 `best_effort` + `keep_last(1)`——
连发速度超过节点处理速度会丢消息。请以**≥ 0.5 s 间隔**发布（如上例），并通过
offboard 日志（`Waypoint buffered (#N)`）或 `/waypoint_markers` 确认入队成功。

### 状态反馈话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/lidar_slam/odom` | `nav_msgs/Odometry` | **融合里程计**（FAST-LIO → PX4 EKF2），世界 ENU 系——算法的主要状态反馈 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 世界系激光点云（ROG-Map 建图输入） |
| `/planning/pos_cmd` | `mars_quadrotor_msgs/PositionCommand` | SUPER 的轨迹指令（位置/速度/加速度/偏航/偏航角速度），约 100 Hz |
| `/planning_cmd/poly_traj` | `mars_quadrotor_msgs/PolynomialTrajectory` | 多项式轨迹（MPC 心跳） |
| `fsm/path` | `nav_msgs/Path` | 规划路径（A* → 优化） |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | PX4 原始局部位置（NED） |
| `/fmu/out/vehicle_status_v4` | `px4_msgs/VehicleStatus` | 飞行器状态（arming、nav_state） |

### 坐标系

- **航点、目标与规划器输出均为 ENU 世界系**（`frame_id: "world"`，
  x 东、y 北、z 上）。坐标系转换（ENU → PX4 NED，含偏航）由 offboard 节点
  内部自动完成。
- 航点到达按**水平**距离判断（`waypoint_reached_dist`）；到达后悬停
  `waypoint_hold_time` 秒，再开始执行缓冲队列中的下一个航点。
- FAST-LIO 的 `camera_init` 原点为第一帧 LiDAR 位置（即起飞点），与
  Gazebo 世界原点存在固定偏移。若算法按地图坐标工作，建议以**起飞点为参考
  原点**。

### 航点跟随参数（`config/offboard.yaml`）

| 键 | 默认值 | 说明 |
|---|---|---|
| `waypoint_reached_dist` | `3.0` m | 水平距离小于该值判定航点到达 |
| `waypoint_hold_time` | `0.0` s | 到达后悬停时间，再执行下一个航点 |
| `goal_height` | `5.0` m | 平面目标的目标高度（覆盖 SUPER `fsm.click_height`）|
| `planner_cmd_hz` | `80.0` Hz | offboard 状态机判定规划器接管的指令频率阈值 |
| `default_height` | `5.0` m | OFFBOARD 后自动起飞高度（NED）|

以上均为 launch 参数，可单次运行覆盖，例如：
`ros2 launch offboard offboard.launch.py waypoint_reached_dist:=2.0
waypoint_hold_time:=1.0`。

### 降落

```bash
ros2 service call /offboard/land std_srvs/srv/Trigger
```

### 记录与评估

`cmd_record`（见上文）把每个目标连同指令轨迹与融合里程计一并记录：

```bash
# 终端 3 — 记录器 + 实时曲线（收到第一个目标后开始）
ros2 launch cmd_record record.launch.py

# 事后绘制某段记录（默认最新 CSV）
ros2 run cmd_record plot_csv
```

每次目标写入 `cmd_log/goal_<NNN>_<timestamp>.csv`，包含目标位置、指令轨迹与
真实里程计。

### FAST-LIO 融合阶段性结果（2026-08）

在 `yungu` 世界中以 Gazebo 真值（`/odom`）为基准（分场景归因）实测：

| 场景 | 水平误差 | 垂直偏差 | 说明 |
|---|---|---|---|
| 地面静止 | **0.00 m** | +0.52 m | 零漂移，完全稳定 |
| 悬停 4.5 m | **0.04 m** | +0.42 m | 水平精度很好 |
| 飞行（v ≤ 4.7 m/s）| 0.4–2.8 m（∝速度）| +0.6–1.4 m | 滞后 + 垂直漂移 |
| 停住后 | **0.18 m**（收敛）| +1.36 m（不回落）| 水平恢复，垂直锁死 |

PX4 EKF2 融合里程计（`/lidar_slam/odom`）与 FAST-LIO 输出相差 **0.03–0.12 m**——
EKF2 完全采纳视觉里程计，融合链路按预期工作。

已定位的三个偏差来源（均有明确归属，均非 FAST-LIO 算法缺陷）：

1. **静止/悬停垂直基准差（+0.42–0.52 m，恒定）** — 坐标系原点差：FAST-LIO
   的 `camera_init` 原点取第一帧 LiDAR 位置，Gazebo world 原点取模型 spawn
   位置。不是错误——与真值对比前应先对齐原点。水平方向不受影响（≤ 0.04 m）。
2. **飞行水平滞后（约 0.4–0.7 s，∝速度）** — 飞行偏差 ≈ 速度 × 0.5–0.6 s；
   将 FAST-LIO 输出按时间平移 −0.7 s 可消除约 75% 偏差。由**仿真管道**造成
   （WSL 下约 184 KB 大点云多跳转发 + PX4 时钟与仿真时钟偏差）。真机使用
   硬件打标传感器数据时不应出现此问题。
3. **飞行后垂直漂移（+1 m，不回落）** — SLAM 无绝对高度参考，z 偏移被吸进
   地图后永久保留。真机建议用气压计/高度源约束 z。

