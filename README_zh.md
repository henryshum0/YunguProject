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
| `model`           | Gazebo 车辆模型 / PX4 gz 机型（如 `x500_lidar`）                 |
| `world`           | Gazebo 地图（`.sdf` 位于 `VisionFlow-PX4/Tools/simulation/gz/worlds`，如 `yungu`）|
| `gz_version`      | Gazebo 发行版（`harmonic`）                                      |
| `xrce_port`       | MicroXRCEAgent 的 UDP 端口                                       |
| `bridge.enabled`  | 是否启动 GZ↔ROS `parameter_bridge`                               |
| `bridge.tf_enabled` | 是否启动 TF 桥（`tf_bridge.py`）                               |
| `bridge.topics`   | 桥接到 ROS 2 的 GZ 话题（如 `/x500_lidar/scan`、`/x500_lidar/scan/points`）|

`model` + `world` 组合成 PX4 make 目标 `gz_<model>_<world>`
（例如 `gz_x500_lidar_yungu`）。查看生效值与完整模型/地图列表：

```bash
./src/utils/start_sim.sh --help
```

### 环境变量覆盖

每个配置值都可以在命令行覆盖（无需改文件）：

| 变量         | 默认值                  | 说明                                          |
|--------------|------------------------|----------------------------------------------|
| `PX4_MODEL`  | 来自 config（`x500_lidar`） | Gazebo 车辆模型 / `make px4_sitl` 机型       |
| `PX4_WORLD`  | 来自 config（`yungu`）  | Gazebo 地图/世界（worlds 目录下的 .sdf）       |
| `XRCE_PORT`  | 来自 config（`8888`）   | MicroXRCEAgent 的 UDP 端口                    |
| `GZ_VERSION` | 来自 config（`harmonic`）| gz-transport 版本（gz-sim 8）                 |
| `HEADLESS`   | *(未设置)*              | 任意非空值（如 `1`）以无 GUI 方式运行 Gazebo（仅服务器）|

```bash
# 不改配置，使用不同机型/世界
PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./src/utils/start_sim.sh

# 旧式写法：直接传完整 make 目标
PX4_MODEL=gz_x500_lidar_yungu ./src/utils/start_sim.sh

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
  `src/SUPER/super_planner/config/gazebo.yaml`，
  订阅桥接话题（`/x500_lidar/scan/points`、`/odom`）。

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
./temp/start_all_fastlio.sh

# Gazebo 仅服务器（无 GUI）——RViz 仍会打开以查看规划可视化
HEADLESS=1 ./temp/start_all_fastlio.sh

# 完全跳过 RViz
NO_RVIZ=1 ./temp/start_all_fastlio.sh
```

`start_all_fastlio.sh` 相对 `start_sim.sh` 新增的组件：

| # | 组件 | 说明 |
|---|---|---|
| 1 | `temp/gazebo_imu_bridge.py` | PX4 `sensor_combined`（FRD）→ `/livox/imu`（ENU）：轴翻转 + 滚动时间同步 |
| 2 | `temp/add_time_field.py` | 为 Gazebo 点云补充 `time` 字段（瞬时扫描填 0）|
| 3 | `fast_lio`（`fastlio_mapping`）| 激光惯性里程计，配置：`temp/fastlio_gazebo.yaml` |
| 4 | `temp/fastlio_px4_bridge.py` | FAST-LIO `/Odometry`（ENU）→ `/fmu/in/vehicle_visual_odometry`（NED）供 EKF2 使用 |
| 5 | `super_bridge` | PX4 融合里程计 + 原始点云 → `/lidar_slam/odom` + `/cloud_registered` |

PX4 EKF2 外部视觉参数在 x500 机型中设置（`4008_gz_x500_lidar`）：
`EKF2_EV_CTRL 15`（位置+垂直+速度+偏航融合）、`EKF2_EV_DELAY 5`、
`EKF2_EVP_NOISE 0.1`、`EKF2_EVV_NOISE 0.1`、`EKF2_EVA_NOISE 0.05`，
以及 `COM_POWER_OVERRIDE 1`（避免 SITL 电源预检阻止解锁）。

**验证融合链路是否正常运行：**

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once --qos-reliability best_effort
ros2 topic echo /lidar_slam/odom --once --qos-reliability best_effort
ros2 topic echo /cloud_registered --once --qos-reliability best_effort
```

**fastlio比较调试**
```
source install/setup.bash
python3 temp/check_pos3.py 60
```
**无GUI的fastlio启动**
```
HEADLESS=1 ./temp/start_all_fastlio.sh
```
### FAST-LIO 融合阶段性结果（2026-08）

在 `yungu` 世界中以 Gazebo 真值（`/odom`）为基准，使用
[`temp/check_pos4.py`](temp/check_pos4.py)（分场景归因、输出 CSV）实测：

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

