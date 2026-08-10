# YunguProject 项目逻辑梳理

> 基于 ROS 2 (Humble) + PX4 Autopilot + Gazebo (Harmonic) 的四旋翼仿真与部署平台。
> 核心：SUPER 运动规划 + FAST-LIO 激光惯性里程计 + PX4 EKF2 视觉融合。

---

## 一、整体架构（数据流）

```
┌────────────────────────── 仿真 / 实机 ──────────────────────────┐
│                                                                │
│  Gazebo (仿真) / 真实传感器 (实机)                              │
│    ├── LiDAR 点云 → /x500_lidar/scan/points                    │
│    └── IMU → PX4 → /fmu/out/sensor_combined                    │
│                        │                                       │
│                        ▼                                       │
│              FAST-LIO 链（temp/ 脚本）                          │
│    ├── gazebo_imu_bridge.py: PX4 IMU (FRD) → ENU /livox/imu    │
│    ├── add_time_field.py: 点云补 time 字段                      │
│    └── fastlio_mapping: 激光惯性里程计 → /Odometry              │
│                        │                                       │
│                        ▼                                       │
│              fastlio_px4_bridge.py                             │
│              /Odometry (ENU) → /fmu/in/vehicle_visual_odometry │
│              （送入 PX4 EKF2 外部视觉融合）                     │
│                        │                                       │
│                        ▼                                       │
│              PX4 EKF2 融合 (IMU+GPS+视觉)                      │
│              → /fmu/out/vehicle_odometry                       │
│                        │                                       │
│                        ▼                                       │
│              super_bridge（offboard 包）                       │
│              → /lidar_slam/odom + /cloud_registered            │
│                        │                                       │
│                        ▼                                       │
│              SUPER planner (fsm_node) → /planning/pos_cmd      │
│                        │                                       │
│                        ▼                                       │
│              offboard_node（状态机）→ PX4 指令                  │
│                        │                                       │
│                        ▼                                       │
│              PX4 飞控执行（Gazebo 仿真 / 实机）                │
└────────────────────────────────────────────────────────────────┘
```

**要点**：FAST-LIO 模式**不依赖 Gazebo 真值**（真值仅用于调试对比），与实机一致。

---

## 二、顶层目录分工

| 目录/文件 | 作用 |
|-----------|------|
| `src/` | ROS 2 工作空间源码包（核心代码） |
| `VisionFlow-PX4/` | PX4 飞控固件（git submodule，yungudemo 分支） |
| `config/` | 仿真配置（`simulation.yaml` 唯一事实来源） |
| `build/` `install/` `log/` | colcon 编译产物与日志（gitignore） |
| `temp/` | 自定义启动脚本与 FAST-LIO 集成（不入主流程） |
| `install_deps.sh` | 依赖安装脚本 |
| `README.md` / `README_zh.md` | 项目文档（中英文） |
| `.gitmodules` | submodule 定义（VisionFlow-PX4、livox_ros_driver2） |

---

## 三、src/ 各包详解

### 1. `src/FAST_LIO/` — 激光惯性里程计
| 文件 | 作用 |
|------|------|
| `src/laserMapping.cpp` | **主算法**：ESKF 状态估计 + ICP 点云配准 + ikd-Tree 地图 + 发布 odom/TF |
| `src/IMU_Processing.hpp` | IMU 预积分（EKF 预测） |
| `src/preprocess.cpp/.h` | 点云预处理（按 `lidar_type` 分发：0=default, 1=Livox, 2=Velodyne...） |
| `include/ikd-Tree/` | 增量 KD 树（地图管理） |
| `include/IKFoM_toolkit/` | ESKF 数学库（状态估计核心） |
| `config/*.yaml` | 各雷达配置（mid360/avia/horizon/velodyne/ouster） |
| `launch/mapping.launch.py` | 标准启动 |

**我们的调用**：`ros2 run fast_lio fastlio_mapping --ros-args --params-file temp/fastlio_gazebo.yaml`
（配置驱动，算法源码未改）

### 2. `src/SUPER/` — 运动规划（HKU MaRS 开源）
| 子包 | 作用 |
|------|------|
| `super_planner/` | **主规划器**：fsm_node（状态机）+ 轨迹优化 + A* 搜索 |
| `rog_map/` | ROG-Map 栅格建图（订阅点云+odom → 占据地图） |
| `mission_planner/` | 任务级规划（航点） |
| `mars_uav_sim/` | 官方无人机仿真（本项目的 x500 不在此） |
| `super_planner/config/*.yaml` | 规划配置（gazebo.yaml=主配置，fastlio_live.yaml=我们的变体） |

**super_planner 关键文件**：
| 文件 | 作用 |
|------|------|
| `src/super_core/super_planner.cpp` | 规划核心（前端+优化） |
| `src/super_core/fsm.cpp` | 状态机（WAIT_GOAL→GENERATE_TRAJ→FOLLOW_TRAJ） |
| `src/super_core/astar.cpp` | A* 路径搜索（0.2s 超时硬编码） |
| `include/ros_interface/ros2/fsm_ros2.hpp` | ROS2 接口（订阅 goal/odom/cloud，发 pos_cmd） |

### 3. `src/offboard/` — PX4 offboard 控制
| 文件 | 作用 |
|------|------|
| `src/offboard_node.cpp` | **状态机**：INIT→ARMING→SET_OFFBOARD→TAKEOFF→IDLE↔PLANNER→LANDING |
| `src/super_bridge.cpp` | **桥接**：PX4 vehicle_odometry → /lidar_slam/odom + 点云 → /cloud_registered |
| `src/goal_marker_node.cpp` | RViz 2D Goal → marker 可视化 |
| `launch/offboard.launch.py` | 启动 offboard_node + super_bridge + fsm_node + RViz |
| `rviz/x500_lidar_paths.rviz` | RViz 配置 |

### 4. `src/px4_msgs/` — PX4 ROS2 消息定义
`msg/*.msg`：`VehicleOdometry`、`OffboardControlMode`、`TrajectorySetpoint`、`SensorCombined` 等。

### 5. `src/livox_ros_driver2/` — 览沃雷达驱动（submodule）
真机用（MID360/HAP 等），仿真不经此（用 Gazebo 模拟雷达）。

### 6. `src/cmd_record/` — 指令记录工具
`cmd_record_node.py`：记录 /planning/pos_cmd 到 CSV；`plot_csv.py`：绘图分析。

### 7. `src/utils/` — 仿真启动工具
| 文件 | 作用 |
|------|------|
| `start_sim.sh` | 仿真三件套：PX4+Gazebo + uXRCE + bridge（读 config/simulation.yaml） |
| `stop_sim.sh` | 停止仿真 |
| `gz_bridges/bridge.sh` | GZ↔ROS bridge + TF |
| `gz_bridges/tf_bridge.py` | TF 广播（world→base_link→lidar_link） |

---

## 四、temp/ 自定义脚本（FAST-LIO 集成）

| 文件 | 作用 |
|------|------|
| `start_all_fastlio.sh` | **一键全栈**：GPU + PX4 + FAST-LIO 链 + EKF2 融合 + planner + RViz |
| `start_all.sh` | 一键全栈（Gazebo 真值模式，调试用） |
| `start_sim_gpu.sh` | GPU 强制仿真 + 控制链（HEADLESS 支持） |
| `gazebo_imu_bridge.py` | PX4 sensor_combined (FRD) → /livox/imu (ENU)：**轴翻转 + 滚动时间同步** |
| `add_time_field.py` | Gazebo 点云补 time 字段（瞬时扫描填 0） |
| `fastlio_px4_bridge.py` | FAST-LIO /Odometry (ENU) → PX4 /fmu/in/vehicle_visual_odometry (NED) |
| `fastlio_gazebo.yaml` | FAST-LIO 仿真配置（lidar_type:0, 外参单位阵） |
| `fastlio_live.yaml` | planner 配置（FAST-LIO 直接定位变体） |
| `x500_fastlio.rviz` | FAST-LIO 模式 RViz 配置 |
| `PROJECT_STRUCTURE.md` | 本文档 |

---

## 五、PX4 关键配置（VisionFlow-PX4 submodule）

`ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_x500_lidar`：
```bash
EKF2_EV_CTRL 15        # 外部视觉融合（位置+垂直+速度+偏航）
EKF2_EV_DELAY 5        # 视觉延迟 [ms]
EKF2_EVP_NOISE 0.1     # 视觉位置噪声
EKF2_EVV_NOISE 0.1     # 视觉速度噪声
EKF2_EVA_NOISE 0.05    # 视觉姿态噪声
COM_POWER_OVERRIDE 1   # SITL 电源预检绕过（允许解锁）
```

`src/modules/uxrce_dds_client/dds_topics.yaml`：uXRCE 桥接话题列表（含 vehicle_odometry / vehicle_visual_odometry）。

---

## 六、启动方式汇总

| 场景 | 命令 |
|------|------|
| 官方仿真（真值） | `./src/utils/start_sim.sh` + `ros2 launch offboard offboard.launch.py` |
| FAST-LIO 全栈 | `./temp/start_all_fastlio.sh` |
| FAST-LIO 无 GUI | `HEADLESS=1 ./temp/start_all_fastlio.sh` |
| GPU 强制仿真+控制 | `./temp/start_sim_gpu.sh` |
| 停止 | `./src/utils/stop_sim.sh` 或 Ctrl+C |

---

## 七、坐标系约定（关键！）

| 坐标系 | 谁在用 | 说明 |
|--------|--------|------|
| `camera_init` | FAST-LIO 世界系 | ENU，原点=LiDAR 第一帧位置 |
| `world` | Gazebo / super_bridge | ENU，仿真世界原点 |
| `NED` | PX4 / offboard | 飞控坐标系（x 北 y 东 z 下） |
| `/lidar_slam/odom` | planner | super_bridge 从 PX4 融合 odom 转换（ENU） |

**转换链**：
- FAST-LIO (ENU/camera_init) → fastlio_px4_bridge (ENU→NED) → PX4 EKF2
- PX4 (NED) → super_bridge (NED→ENU) → planner (ENU)
