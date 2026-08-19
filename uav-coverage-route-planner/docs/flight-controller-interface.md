# 下层飞控接口说明

## 1. 交付文件

单机适配器读取各无人机目录下的 `flight_plan.json`；`flight_plan.yaml` 内容等价。双机任务根目录另有 `mission_manifest.json`，说明两机同时启动、责任归属和各自文件路径。当前协议为 `schema_version = "3.0"`，坐标系为 ENU，单位为米。

两架无人机的计划在任务开始前都已计算完成，随后独立执行。本版本不提供双机时空碰撞规避；绕障路线可以短暂离开责任区。

## 2. 顶层结构

```json
{
  "schema_version": "3.0",
  "coordinate_frame": "ENU",
  "units": "meters",
  "map_id": "yungu2030_local_origin",
  "video_detection": {},
  "lanes": [],
  "route_segments": [],
  "waypoints": [],
  "summary": {}
}
```

## 3. 连续视频检测配置

```json
{
  "mode": "continuous_video_stream",
  "analysis_rate_hz": 2.0,
  "control_point_spacing_m": 10.0,
  "forward_overlap": 0.3,
  "lane_overlap": 0.3,
  "target_envelope": {"width_m": 2.5, "length_m": 6.0, "height_m": 2.5},
  "image_boundary_margin_ratio": 0.05,
  "building_wall_occlusion": true
}
```

相机在任务中输出连续视频，不存在定频快门。`analysis_rate_hz` 是规划器近似连续视野扫掠时的数值采样率，也可作为下游视频检测的建议推理帧率。`target_envelope` 与画面边缘余量用于保证目标整体入镜。

## 4. 飞控途径点与航段

途径点包含 `id/sequence/x/y/z/heading_deg/speed_mps/turn_in_place/hold_time_s`。`sequence` 从 1 连续递增；0° 指北、90° 指东；最后一点速度为 0。`turn_in_place=true` 时，多旋翼应先到点并稳定保持 `hold_time_s`，完成原地航向调整后再飞下一段。

航段的 `kind` 可为：

| 值 | 含义 |
|---|---|
| `coverage_lane` | 本责任区的主检测航段 |
| `connector` | 检测航线间的连接段 |
| `obstacle_avoidance` | 建筑安全缓冲绕行段 |
| `return_home` | 返回起降点航段 |

`detection_enabled` 表示该航段的视频分析结果是否计入任务覆盖。它不是拍照指令，也不能替代实时安全判断。

## 5. 推荐执行流程

```text
加载并校验双机清单及两份计划
          ↓
确认两机均已就绪，同一任务时刻启动
          ↓
每架飞机独立按 sequence 跟踪 route_segment
          ↓
设置速度和航向，持续输出视频流
          ↓
detection_enabled=true 时运行/采纳检测结果
          ↓
到达 end_waypoint_id 容差内，进入下一航段
          ↓
最后一点悬停或降落
```

任务开始前应校验协议版本、ENU 原点、ID/序号连续性、航段连通性，以及高度和速度是否在载具能力范围内。`mission_status` 不是 `ready` 时禁止执行。由于本规划器没有时空解冲突，实际运行前还必须由上层系统确认两条航线满足现场安全要求。

## 6. 适配器边界

- 不要把途径点理解为单帧检测或拍照触发点；
- 不要根据航点是否为转折点推断检测状态；
- 不要自动改变 ENU 原点、轴方向或单位；
- 不要在未重新安全校验时跳过途径点；
- 不要把规划视野当作实时定位或视觉识别真值；
- 超出飞行走廊或定位质量不足时，应进入载具自身的安全策略。

## 7. 覆盖报告口径

覆盖报告将规划阶段与最终验收分开记录：`initial_candidate_metrics` 是补漏和连续
复核前的候选方案指标，`final_solution_metrics` 是最终可执行航线指标。
`coverage_generation_method` 记录 Coverage Generation 方法，当前为
`global_scanline` 或 `bcd`；`route_optimization_method` 记录固定 lanes 之后采用的
Route Optimization 方法。旧字段 `scan_pattern` 仅为向后兼容保留。
`route_optimization_method` 固定为 `auto`。不超过 12 条 coverage lanes 时采用
Exact；更大任务采用 Greedy + 2-opt + Or-opt 组合启发式。`scan_direction_deg`
仅可为 `0`、`90` 或 `null`；`null` 表示自动比较两个正交方向。
`completion_strategy` 记录覆盖补全的路线组织方式，默认值为 `local_insertion`；
`full_greedy` 仅在调用方显式指定时使用。该字段用于追溯规划过程，不改变飞控对
最终连续航点序列的执行方式。
所有航段保持视频检测开启，任务不会在飞行中动态增加或修改航点。
`unreachable_candidate_point_ids` 仅表示初始候选点无法安全到达，不等于漏检；
最终仍低于覆盖要求的地面只由 `unreachable_patch_ids` 表示。下层系统判断任务是否
可以执行时，应读取 `mission_status` 和最终覆盖指标，不能依据候选点数量自行否决。
