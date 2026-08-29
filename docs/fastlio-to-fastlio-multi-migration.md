# FAST-LIO → FAST_LIO_MULTI_ROS2 迁移适配清单

> 范围:整个项目从单雷达 `fast_lio` 包切到双雷达 `fast_lio_multi` 包时,所有需要改动的地方(包/配置/数据流/下游消费者/脚本)。
> 节点内部流水线与输出话题的框图见 [fastlio-multi-dataflow.md](fastlio-multi-dataflow.md)。
> 旧版 = 仓库原 `src/FAST_LIO/`(ROS2 单雷达 fork),新版 = [Draxran/FAST_LIO_MULTI_ROS2](https://github.com/Draxran/FAST_LIO_MULTI_ROS2)(ROS2 双雷达,本地克隆在 `src/FAST_LIO_MULTI_ROS2/`,**dev 分支**打了两个补丁:`default_handler` 支持无 time/ring 字段的 Gazebo 点云 + IMU 订阅 QoS 修正)。

---

## 1. 数据流总览(切换前后)

```text
【旧】单雷达融合:
  gz scan_{left,right}/points ── add_time_field ×2 ──► points_timed ──► lidar_merge ──► points_fused (base_link)
                                                                              │
                                                                              ▼
                                                    fast_lio (fastlio_mapping, 消费 1 路融合云 + /livox/imu)
                                                                              │
                                                    /Odometry (camera_init → body) ──► fastlio_px4_bridge → PX4

【新】双雷达内部融合:
  gz scan_{left,right}/points ── lidar_transform ×2 ──► points_base (base_link, 2 路) ──► fast_lio_multi (laserMapping_bundle)
                                                                                          │  /Odometry (camera_init → body)
                                                                                          └─► fastlio_px4_bridge → PX4(不变)
  gz scan_{left,right}/points ── lidar_merge(保留,改吃原始 points)──► points_fused ──► super_bridge → /lidar_slam/odom + /cloud_registered(world 系,不变)
```

IMU 链路两版相同:`/livox/imu_raw ── imu_relay ──► /livox/imu`。

---

## 2. 包级替换

| 项 | 旧 `fast_lio` | 新 `fast_lio_multi` |
|---|---|---|
| 包名 / 可执行 | `fast_lio` / `fastlio_mapping`(单节点) | `fast_lio_multi` / `laserMapping_bundle`、`laserMapping_async`、`laserMapping_adaptive`(三选一,本项目用 bundle) |
| 自定义 msg | `msg/Pose6D.msg`(rosidl 生成) | 删除,`Pose6D` 改为 [common_lib.h](../src/FAST_LIO_MULTI_ROS2/include/common_lib.h) 内 struct |
| 构建结构 | 单 executable + matplotlibcpp/PythonLibs | `fastlio_preprocess` 库(ikd_Tree + preprocess)+ 3 个可执行 |
| 新增依赖 | — | `message_filters`、`tf2_ros`、`tf2_eigen`、`eigen3_cmake_module`、PCL(`common io filters kdtree`)、OpenMP(REQUIRED) |
| 移除依赖 | matplotlibcpp、PythonLibs、`rclcpp_components`、`std_srvs`、`visualization_msgs` | — |
| 启动命令 | `ros2 run fast_lio fastlio_mapping --ros-args --params-file <cfg>` | `ros2 run fast_lio_multi laserMapping_bundle --ros-args --params-file <cfg>` |
| 源码差异 | 单雷达 [laserMapping.cpp] 等 | 新增多雷达路径:preprocess 双路处理(`lidar_num`)、[IMU_Processing.hpp](../src/FAST_LIO_MULTI_ROS2/src/IMU_Processing.hpp) 新增 `UndistortPclMultiLiDAR` + `lidar_num` 参数、`MeasureGroup` 增加 `lidar2 / lidar_beg_time2 / lidar_end_time2`;`use-ikfom.hpp` 两版**完全一致** |

## 3. 配置文件差异(`yaml`)

### 3.1 参数结构变化

- 旧版:顶层扁平参数(`feature_extract_enable`、`point_filter_num`、`filter_size_surf`、`filter_size_map`、`max_iteration`、`cube_side_length`、`runtime_pos_log_enable`、`map_file_path`)+ `common / preprocess / mapping / publish / pcd_save` 子节
- 新版:`common / method / preprocess / mapping / publish / pcd_save` 全部收进子节,`max_iteration` 移到 `common` 下

### 3.2 新增参数(双雷达相关)

| 参数 | 说明 |
|---|---|
| `common.multi_lidar` | `true` 走双雷达融合;`false` 退化为单雷达(只用 1 号) |
| `common.lid_topic2` / `common.imu_topic` | 2 号雷达话题;IMU 话题仍单路(两雷达共用 1 号 IMU 外参) |
| `common.map_frame` | 输出坐标系,默认 `map`;本项目设为 `camera_init` 保持下游兼容 |
| `preprocess.{lidar_type2, point_filter_num2, scan_line2, scan_rate2, timestamp_unit2, blind2}` | 2 号雷达的预处理参数,与 1 号完全平行 |
| `mapping.zero_start_pose` / `zero_start_delay_scans` | 首帧归零 + 延迟帧数 |
| `mapping.extrinsic_imu_to_lidars` | `true`:(T,R,T2,R2) 各雷达独立外参;`false`:(T,R) + `extrinsic_{T,R}_L2_wrt_L1` 雷达间相对外参 |
| `mapping.extrinsic_T2/R2`、`extrinsic_{T,R}_L2_wrt_L1`、`extrinsic_{T,R}_L1_wrt_drone` | 2 号外参 / 雷达间外参 / 可视化用机体外参 |
| `method.*`(voxelized_pt_num_thres、effect_pt_num_ratio_thres、bundle_enabled_tic_thres) | 仅 adaptive 模式在 bundle/async 间切换的阈值 |

### 3.3 删除的参数(新版不支持,需从配置中移除)

`feature_extract_enable`(特征提取)、`filter_size_map`、`runtime_pos_log_enable`、`map_file_path`、`common.time_sync_en`、`common.time_offset_lidar_to_imu`(外部时间同步)、`mapping.fov_degree`、`publish.map_en`、`publish.effect_map_en`。

> 注意:`/Laser_map` 的 publisher 在新节点里创建了但**从不调用**;`/cloud_effected` 在新节点**不存在**。旧版 RViz 里的增量地图显示(见 §5.4)会变空。

### 3.4 语义变化

- `mapping.extrinsic_est_en`:**多雷达模式下不支持在线外参估计**(代码注释 "not supported yet for multi"),须设 `false`
- `publish.scan_publish_en` 等门控保留,但 `publish_visionpose`(`/mavros/vision_pose/pose`)改为由 `common.publish_tf_results` 门控
- `lidar_type: 0`(default_handler,Gazebo 瞬时点云无 time/ring)→ 新版需要 dev 分支补丁;两版都必须为 0

## 4. 传感器数据流适配(`lidar_bridge`)

| 旧 | 新 | 说明 |
|---|---|---|
| `add_time_field.py` ×2(补 time 字段)→ `points_timed` | **删除** | 新版 `lidar_type: 0` 不解析 time 字段,无需补 |
| — | **新增** `lidar_transform.py` ×2 → `points_base` | 按安装外参(与 model.sdf 一致:左 `(0,+0.40,0.05,roll -0.6)`、右 `(0,-0.40,0.05,roll +0.6)`)把每侧点云变换到 base_link(=IMU 系),`fast_lio_multi` 联合去畸变要求两路云都已在 IMU 系 → 配置中外参全 identity |
| `lidar_merge.py` 消费 `points_timed` | 改消费原始 `points` | 仅剩一个用途:给 `super_bridge` 合成 world 系 `/cloud_registered`(ROG-Map / RViz),与 fast_lio_multi 的 `points_base` 双路直连互不干扰 |
| `imu_relay` → `/livox/imu` | 不变 | — |

## 5. 下游消费者适配

| 消费者 | 是否需改 | 说明 |
|---|---|---|
| `fastlio_px4_bridge.py` | **不改** | 仍消费 `/Odometry`(camera_init → body,ENU)。前提是新配置 `map_frame: camera_init` 且 `zero_start_pose: false`(见 §7) |
| `super_bridge`(offboard.launch.py) | 仅注释/默认值说明更新 | 仍消费 `/scan/points_fused`,话题不变 |
| `flight_monitor/monitor.py` | **已改** | 进程监控名单:`fastlio_mapping` → `laserMapping_bundle`,`add_time_field` → `lidar_transform`(+`lidar_merge`) |
| `src/offboard/rviz/fastlio_ikdtree.rviz` | **需改** | `/cloud_effected`、`/Laser_map` 两个显示组在新节点下无数据(见 §3.3),需改用其他话题(如 `/cloud_registered_tf`)或删掉显示组;`/Odometry`、`/path`、`/fsm/path`、`/gt_path`、`/rog_map/inf_occ` 等不受影响 |
| 根 `README.md` | **待改** | 组件表(约 180-181 行)仍是旧描述(add_time_field / fast_lio / fastlio_mapping),需同步 §1 的新流程 |
| `docs/README_zh.md`、`docs/utils-fastlio-gz-bridges.md` | 已更新 | 已改为 MULTI 版本描述 |

## 6. 启动脚本与配置(`utils/start_fastlio.sh`)

| 改动点 | 旧 | 新 |
|---|---|---|
| 配置变量 | `FASTLIO_CONFIG=config/fastlio_swan_gamma_effect.yaml` | `FASTLIO_MULTI_CONFIG=config/fastlio_multi_swan.yaml` |
| 话题变量 | `LIDAR_FUSED_TOPIC=…/scan/points_fused` | `LIDAR_LEFT_TOPIC / LIDAR_RIGHT_TOPIC=…/scan_{left,right}/points_base` |
| 启动节点 | `ros2 run fast_lio fastlio_mapping` | `ros2 run fast_lio_multi laserMapping_bundle` |
| 等待就绪 | `points_fused` 单路 echo | `points_base` **左右两路** echo |
| watchdog / cleanup | `pkill -f "fastlio_mapping"`、`add_time_field` | `pkill -f "laserMapping_bundle"`、`lidar_transform`(+`lidar_merge` 保留) |

launch 文件:新版 `fast_lio_multi.launch.py`(支持 `update_method:=bundle|async|adaptive`、`config:=…` 参数,还自带 static TF 与可选的 pc_utils 包含)——本项目实际不走 launch,**直接用 `ros2 run` 启动**,仅作参考。

## 7. 本项目特有的配置决策(`config/fastlio_multi_swan.yaml`)

- `map_frame: "camera_init"` + `zero_start_pose: false`:`/Odometry` 保持在 camera_init 系,兼容 `fastlio_px4_bridge` 与 `world → camera_init` 静态 TF(与旧版行为一致)
- `scan_publish_en: false`:`/cloud_registered` 由 `super_bridge`(world 系)独占,避免与 MULTI 自己的输出在同一话题上冲突
- `lidar_type / lidar_type2: 0`:Gazebo 瞬时点云(无 time/ring),依赖 dev 分支补丁,不做 deskew
- 全部外参 identity:点云已由 `lidar_transform` 变换到 base_link(=IMU 系)
- `extrinsic_imu_to_lidars: false`(两雷达同外参,不启用雷达间外参分支)

## 8. 构建与验证

```bash
# 1. 删除旧包源码(工作区已删,仅剩 Log/)
# 2. 构建新包
colcon build --packages-select fast_lio_multi lidar_bridge
source install/setup.bash
# 3. 验证进程与话题
ros2 run fast_lio_multi laserMapping_bundle --ros-args --params-file config/fastlio_multi_swan.yaml
ros2 topic echo /Odometry --once
ros2 topic echo /livox/imu --once
# 4. 全链路验证(与旧版相同)
ros2 topic echo /fmu/in/vehicle_visual_odometry --qos-reliability best_effort --spin-time 3
```

## 9. 遗留事项

- [ ] 根 `README.md` 组件表更新为 MULTI 流程(§5)
- [ ] `fastlio_ikdtree.rviz` 中 `/cloud_effected`、`/Laser_map` 显示组处理(§5)
- [ ] 旧 `src/FAST_LIO/` 目录(仅剩 `Log/`)与 git 中已删除文件的清理收尾
