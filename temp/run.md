# FAST-LIO 接入控制回路 — 完整启动与诊断流程

## ═══ 第一部分：完整启动流程 ═══

**终端1: 仿真**
```bash
./temp/start_sim_gpu.sh
# 等 "PX4 SITL is up"
# Gazebo GUI 没自动开的话（进程只有 -s 参数时）：
#   另开终端执行： gz sim -g
```

**终端2: FAST-LIO 链（自动检查数据）**
```bash
source install/setup.bash
./temp/start_fastlio_checked.sh
# 等它打印 FAST-LIO chain READY（或看到 [mapping] 刷屏）
# ⚠️ 这个终端千万别 Ctrl+C（会连带杀 FAST-LIO）
```

**终端3: 控制链（offboard + planner）**
```bash
source install/setup.bash
ros2 launch offboard offboard.launch.py planner_config:=fastlio_live.yaml rviz:=false
# 观察 fsm_node 正常加载、offboard 状态机走完（→IDLE）
```

**终端4: RViz**
```bash
source install/setup.bash
ros2 run rviz2 rviz2 -d temp/x500_fastlio.rviz
# 确认 Fixed Frame = camera_init，点云 = /cloud_registered
```

**PX4 xterm: 起飞**
```bash
commander takeoff
commander mode offboard
```

**RViz 发 goal（3-5m 内近点）**

## ═══ 第二部分：同步打点诊断（乱飞时用）═══

**目的**：同时记录 FAST-LIO 位置 + Gazebo 真值 + planner 指令，判断坐标错位在哪一环。

**⚠️ 重要**：必须用 Python 常驻版（bash 版有 DDS discovery 延迟 bug，录不到数据）。

```bash
# 链路全部就绪 + 起飞悬停后，另开终端
source install/setup.bash
python3 temp/log_sync_check.py 30 /tmp/coord_sync.csv
# 期间发 goal 观察乱飞
# 录完自动退出并打印有数据的行
cat /tmp/coord_sync.csv
```

**CSV 格式**（每行是同一时刻）：
```
t,            fastlio_x,y,z,   truth_x,y,z,   cmd_x,y,z
1785906.123,  0.1,0.2,1.5,     0.1,0.2,1.5,   0.5,0.3,1.5
```

**解读规则**：

| 对比 | 正常 | 异常 |
|------|------|------|
| fastlio vs truth | 接近（scale≈1） | 差 15m → FAST-LIO 漂移 |
| cmd vs fastlio | cmd 指向目标方向 | 反向 → offboard/planner 转换错 |
| cmd vs truth | 一致 | 错位 → 坐标系问题 |

**应急停飞**（乱飞时）：
```
commander mode position    # 脱开 offboard
commander land             # 降落
# 紧急：commander disarm   直接停桨
```

## ═══ 第三部分：点云异常诊断（点云跟着无人机转）═══

**现象**：无人机旋转时，建好的点云跟着转，不是"一边转一边更新地图"。

**可能原因**：
1. **RViz 固定帧错了**（最常见）：Fixed Frame 如果是 body/base_link（跟随无人机的帧），点云自然跟着转
2. **FAST-LIO 位姿没更新**：EKF 卡住/IMU 断了 → 点云用旧位姿变换 → 新扫描点投影错位 → 看起来点云在转

**检查步骤**：
```bash
# 1. 确认 RViz 固定帧
#    Global Options → Fixed Frame 必须是 camera_init

# 2. 无人机原地旋转（慢转），同时看 /Odometry 的四元数是否变化
ros2 topic echo /Odometry --once 2>&1 | grep -A5 "orientation:"
# 转 90° 后四元数应该明显变化（z/w 分量变）
# 如果四元数不变 → FAST-LIO 位姿没更新 → 查 IMU 链路

# 3. 确认 IMU 数据在流（FAST-LIO 依赖它）
ros2 topic echo /livox/imu --once 2>&1 | grep -A4 "angular_velocity"

# 4. 看 FAST-LIO 终端是否还在刷 [mapping]（没刷 = EKF 卡住）
```

## ═══ 附：常用检查命令 ═══

```bash
# FAST-LIO 状态
ros2 topic info /Odometry                  # Publisher ≥ 1
ros2 topic echo /Odometry --once           # 看位置/姿态

# 数据流
ros2 topic echo /livox/imu --once          # IMU bridge
ros2 topic echo /x500_lidar/scan/points_timed --once  # relay

# planner 状态
ros2 node list | grep fsm                   # fsm_node 活着
ros2 topic echo /planning/pos_cmd --once --qos-reliability best_effort

# TF
ros2 run tf2_ros tf2_echo camera_init body  # FAST-LIO TF
