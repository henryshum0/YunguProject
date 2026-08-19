# 传感器数据流胶水层说明(lidar_bridge + offboard + utils)

仿真栈(Gazebo ⇄ ROS 2 ⇄ FAST-LIO ⇄ PX4)之间的胶水层分布在三处:

- **`src/lidar_bridge/`** — ROS 2 包(ament_python),双激光雷达数据流与转发的节点,由 `colcon build` 安装,经 `ros2 run lidar_bridge <node>` 启动。
- **`src/offboard/scripts/`** — FAST-LIO→PX4 桥与可视化辅助节点(由 start_fastlio.sh 直接 python3 启动,不随 offboard.launch.py)。
- **`utils/`** — 顶层 shell 启动器(`start_sim.sh` / `start_fastlio.sh` / `stop_sim.sh` / `bridge.sh`)。

整体数据流(swan_gamma_v2 模型,双侧面激光雷达):

```
Gazebo gpu_lidar ×2 (lidar_left_link / lidar_right_link)
   │  gz topic: /swan_gamma_v2/scan_left/points, /scan_right/points
   ▼
parameter_bridge (ros_gz_bridge, 配置来自 config/simulation.yaml)
   │
   ▼  ROS: /swan_gamma_v2/scan_{left,right}/points
add_time_field.py ×2 ──► /swan_gamma_v2/scan_{left,right}/points_timed
   │
   ▼
lidar_merge.py ──► /swan_gamma_v2/scan/points_fused (base_link 系)
   │
   ▼
FAST-LIO (fast_lio 包 fastlio_mapping, 消费融合点云 + /livox/imu)
   │  /Odometry (camera_init → body)
   ▼
fastlio_px4_bridge.py ──► /fmu/in/vehicle_visual_odometry → PX4 EKF2 (外部视觉)

Gazebo IMU ──► parameter_bridge ──► /livox/imu_raw ──► imu_relay.py ──► /livox/imu ──► FAST-LIO
```

---

## src/lidar_bridge/(新建于 2026-08,原 utils/gz_bridges + utils/fastlio 部分节点)

ROS 2 ament_python 包,集中"双雷达融合/信息转发"类节点。构建:`colcon build --packages-select lidar_bridge`。

### tf_bridge.py
- **作用**:发布 RViz 可视化所需的 TF 树。ros_gz_bridge 只转发消息不发 TF,没有它 RViz 无法显示点云。发布两条变换:
  - 动态 `world → base_link`(订阅 odometry 话题,默认 `/lidar_slam/odom`,BEST_EFFORT 兼容 super_bridge 的 QoS);
  - 静态 `base_link → lidar_link`(默认 z 偏移 0.16 m,可参数化)。
- **启动方式**:`utils/bridge.sh` 按 `bridge.tf_enabled` 启动:`ros2 run lidar_bridge tf_bridge`。
- **依赖的包**:`rclpy`、`nav_msgs`(Odometry)、`geometry_msgs`(TransformStamped)、`tf2_ros`(TransformBroadcaster / StaticTransformBroadcaster)。

### add_time_field.py
- **作用**:给 Gazebo `gpu_lidar` 输出的 PointCloud2 补一个全 0 的 `time` 字段。FAST-LIO 去畸变需要每点 `time` 字段;gz 雷达是瞬时采样(所有点同一时刻),所以补 0 而非伪造线性时间戳,避免 FAST-LIO 做出错误的运动补偿。用 numpy 向量化重排数据(原逐点循环是 20 Hz 单核瓶颈,提速约 20×)。
- **启动方式**:`start_fastlio.sh` 中以 `ros2 run lidar_bridge add_time_field --ros-args -p input_topic:=... -p output_topic:=...` 起两个实例,分别处理左右雷达。
- **依赖的包**:`rclpy`、`sensor_msgs`(PointCloud2 / PointField)、`numpy`。

### lidar_merge.py
- **作用**:双侧面激光雷达融合节点。订阅左右两路 `points_timed`,按外参(左 `(0, +0.40, 0.05, roll -0.6)`、右 `(0, -0.40, 0.05, roll +0.6)`,与 model.sdf 一致)把每帧点云变换到 `base_link`,丢弃 gz gpu_lidar 的 NaN 点,拼接成单帧 PointCloud2 发到 `/swan_gamma_v2/scan/points_fused`。任一侧来新帧即以该帧为准复用另一侧最新帧,输出频率保持扫描频率。
- **启动方式**:`start_fastlio.sh` 中 `ros2 run lidar_bridge lidar_merge`。
- **依赖的包**:`rclpy`、`sensor_msgs`(PointCloud2 / PointField)、`numpy`(标准库 `threading`、`math`)。

### imu_relay.py
- **作用**:IMU 时间戳单调化中继。gz IMU 桥到 `/livox/imu_raw`(与点云同一 sim 时钟),但 250 Hz 下 DDS 投递乱序会造成时间戳回退,FAST-LIO 遇回退时间戳会崩溃/发散("cannot store a negative time point")。此节点把时间戳钳制为严格递增(回退则向后推 1 µs),转发到 `/livox/imu`,并节流打印被钳制的次数便于排查。
- **启动方式**:`start_fastlio.sh` 中 `ros2 run lidar_bridge imu_relay`。
- **依赖的包**:`rclpy`、`sensor_msgs`(Imu)。
- **注意**:输出 QoS 用 RELIABLE 匹配 FAST-LIO 订阅端的默认 QoS。

---

## utils/

顶层 shell 启动器 + 未归包的节点。

### bridge.sh(原 utils/gz_bridges/bridge.sh)
- **作用**:一键启动 GZ ⇄ ROS 桥 + TF 桥。从 `config/simulation.yaml` 的 `bridge.*` 段读取配置,用 `config/sim_config.py` 把要桥接的 topic 列表生成成 ros_gz_bridge 的 `parameter_bridge` 参数文件(临时 yaml),然后拉起 `ros2 run ros_gz_bridge parameter_bridge`。同时按 `bridge.tf_enabled` 决定是否启动 `ros2 run lidar_bridge tf_bridge`。
- **调用**:由 `utils/start_sim.sh` 调用(`BRIDGE_SCRIPT="${SCRIPT_DIR}/bridge.sh"`)。
- **依赖的包/工具**:`ros_gz_bridge`(`parameter_bridge` 可执行)、ROS 2 Humble(`/opt/ros/humble`)、项目内 `config/sim_config.py`、gz-sim 8 Harmonic(`GZ_VERSION` 来自配置)、`lidar_bridge`(TF 节点)。

### src/offboard/scripts/(2026-08 从 utils/fastlio 迁入)
- **fastlio_px4_bridge.py**:把 FAST-LIO 的 `/Odometry`(camera_init → body,ENU)转成 PX4 外部视觉输入 `/fmu/in/vehicle_visual_odometry`(px4_msgs/VehicleOdometry)喂给 EKF2。转换要点:
  - ENU → NED:`pos=(y, x, -z)`、`vel=(vy, vx, -vz)`、四元数左乘 `q_enu_to_ned`(w=0, x=√2/2, y=√2/2, z=0);
  - `pose_frame=POSE_FRAME_NED`、速度用 NED、角速度用 BODY_FRD(EKF2 要求,UNKNOWN 会被拒);
  - 时间戳按 ROS 纪元 µs 填,由 MicroXRCE-DDS Timesync 转 PX4 boot 时间;
  - 协方差取 odom 协方差对角元,缺省回退(位置 0.01、姿态 0.001、速度 0.01)。
  - **启动方式**:`utils/start_fastlio.sh` 直接 `python3 src/offboard/scripts/fastlio_px4_bridge.py`(不随 offboard.launch.py 启动,刻意保持独立)。
  - **依赖的包**:`rclpy`、`nav_msgs`(Odometry)、`px4_msgs`(VehicleOdometry)。
- **gt_path_node.py**:把 Gazebo 真值 `/odom` 累积成 `nav_msgs/Path` 发到 `/gt_path`(world 系),供 RViz 里真值轨迹与 FAST-LIO 的 `/path` 对比(FAST-LIO 起始在 camera_init 系,通过 world→camera_init 静态 TF 对齐)。内存上限 5000 点(≈10 Hz 真值下 8 分钟)。
  - **启动方式**:`utils/start_fastlio.sh` 直接 `python3 src/offboard/scripts/gt_path_node.py`(不随 offboard.launch.py 启动)。
  - **依赖的包**:`rclpy`、`nav_msgs`(Odometry / Path)、`geometry_msgs`(PoseStamped)。

---

## 启动入口汇总

| 文件 | 启动者 | 命令行 |
|---|---|---|
| `utils/bridge.sh` | `utils/start_sim.sh` | `bash .../bridge.sh` |
| `src/lidar_bridge` `tf_bridge` | `bridge.sh`(`tf_enabled=true` 时) | `ros2 run lidar_bridge tf_bridge` |
| `src/lidar_bridge` `imu_relay` | `utils/start_fastlio.sh` | `ros2 run lidar_bridge imu_relay` |
| `src/lidar_bridge` `add_time_field` ×2 | `utils/start_fastlio.sh` | `ros2 run lidar_bridge add_time_field --ros-args -p input_topic:=... -p output_topic:=...` |
| `src/lidar_bridge` `lidar_merge` | `utils/start_fastlio.sh` | `ros2 run lidar_bridge lidar_merge` |
| `offboard` `gt_path_node` | `utils/start_fastlio.sh` | `python3 src/offboard/scripts/gt_path_node.py` |
| `offboard` `fastlio_px4_bridge` | `utils/start_fastlio.sh` | `python3 src/offboard/scripts/fastlio_px4_bridge.py` |
停止时 `start_fastlio.sh` 的 cleanup 会对上述所有节点 `pkill -9 -f <进程名>`(console script 的进程名即节点名,如 `tf_bridge`、`lidar_merge` 等)。
