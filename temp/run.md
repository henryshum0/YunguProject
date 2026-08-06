# FAST-LIO 接入 PX4 — 完整启动与验证流程

## ═══ 第一部分：一键启动 ═══

### 方式 A：Gazebo 真值模式（已验证可用）
```bash
# 1. 杀掉所有残留
pkill -f start_all; pkill -f px4; pkill -f "gz sim"; pkill -f offboard_node; pkill -f fsm_node; pkill -f rviz2; pkill -f cloud_to_world; pkill -f gazebo_imu; pkill -f add_time; pkill -f fastlio; pkill -f static_transform 2>/dev/null
sleep 3

# 2. 一键启动
./temp/start_all.sh
```
- planner 用 `/odom`（Gazebo 真值）
- RViz 固定帧: world
- 适合验证仿真链路、SUPER 规划

### 方式 B：FAST-LIO 模式（调试中）
```bash
# 1. 杀掉所有残留（同上）
# 2. 一键启动
./temp/start_all_fastlio.sh
```
- planner 用 `/Odometry`（FAST-LIO）
- RViz 固定帧: camera_init
- 适合调试 FAST-LIO 定位

### 起飞
```bash
# PX4 xterm 或新终端：
commander takeoff
commander mode offboard
```

## ═══ 第二部分：PX4 原生 offboard 点对点（不经 SUPER）═══

用于验证 FAST-LIO 定位精度，排除 SUPER 干扰。

```bash
# 终端 A: 持续对比位置（先跑，挂着看）
source install/setup.bash
python3 temp/check_pos.py

# 终端 B: PX4 原生 offboard 导航（起飞 + 飞到 (3,2) @ 1.5m 高度）
source install/setup.bash
python3 temp/px4_offboard_nav.py 3 2 1.5 8
```

参数说明：`px4_offboard_nav.py <x> <y> <高度> <悬停秒数>`
- 坐标是 NED，相对起飞点
- 例：飞到前方 3m、右侧 2m、高度 1.5m，悬停 8 秒

**前提**：无人机需先脱开 SUPER 控制（`commander mode position`），或直接未解锁状态运行（脚本会自动 arm + offboard）。

## ═══ 第三部分：验证 FAST-LIO 定位 ═══

### 1. 位置对比（check_pos.py 输出）
```
  t(s)   |  FAST-LIO (x,y,z)          |  Gazebo (x,y,z)            |  diff(m)
  10.0   |  (2.95, 1.98, 1.50)        |  (3.00, 2.00, 1.50)        |   0.08
```
- diff < 0.3m 且飞行中保持 → FAST-LIO 定位 OK
- diff 持续增大 / 反向 → FAST-LIO 有问题

### 2. 旋转方向（yaw 对比）
```bash
# 终端 A: yaw 对比
source install/setup.bash
python3 temp/compare_yaw.py

# 终端 B: 转 90°
source install/setup.bash
python3 temp/turn_yaw.py 90
```
- 两列 yaw 同方向变化 → 旋转估计正确
- 反向 / 卡死 → 陀螺仪轴问题（IMU bridge 翻转）

### 3. 点云是否固定
- RViz 显示 `/x500_lidar/scan/points_world`
- 无人机旋转时障碍物应固定不动
- 跟着转 → FAST-LIO 位姿错

## ═══ 第四部分：RViz 显示配置 ═══

| 项 | 真值模式 | FAST-LIO 模式 |
|----|---------|--------------|
| 固定帧 | world | camera_init（或 world，已加静态 TF）|
| 点云 | /x500_lidar/scan/points_world | /x500_lidar/scan/points_world |
| QoS | Best Effort | Best Effort |

**注意**：RViz 订阅点云必须用 Best Effort（发布者 cloud_to_world 是 Best Effort），配置已改好。

## ═══ 附：应急停飞 ═══
```
commander mode position    # 脱开 offboard
commander land             # 降落
# 紧急：commander disarm   直接停桨
```

## ═══ 附：常用检查 ═══
```bash
# FAST-LIO 状态
ros2 topic echo /Odometry --once | grep -A3 "position:"
ros2 topic info /Odometry | grep Publisher

# 数据流
ros2 topic echo /livox/imu --once | grep -A3 "linear_acceleration"   # 静止应 +9.81
ros2 topic echo /x500_lidar/scan/points_timed --once | grep -A2 "height:"

# planner 状态
ros2 node list | grep fsm
ros2 topic echo /planning/pos_cmd --once --qos-reliability best_effort

# TF
ros2 run tf2_ros tf2_echo world camera_init   # 应持续输出（静态 TF）
```
