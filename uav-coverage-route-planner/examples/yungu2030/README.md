# Yungu2030 example data

`semantic_map.json` is copied from the original GSI project's
`data/yungu2030_v1/semantic_map.json`. It is a small, runtime-independent ENU
semantic asset used by the planner and integration tests.

`search_area.geojson` is an ENU GeoJSON copy of the semantic map's full search
boundary. It provides a directly runnable user-search input while preserving
the semantic JSON as the source of truth.

`planner_config.yaml` contains reproducible example camera, altitude, overlap,
clearance, start, Coverage Generation method, and sweep-direction settings for
the CLI and Web planner.

The original directory contains a 228 MB 3D GLB mesh. The mesh and all
ROS/Gazebo/PX4 files are intentionally excluded from this pure-Python
repository.

提供的 `1920 × 1080` 航拍 JPEG 带有导出灰边，原始方向与语义 ENU 地图相反。
`overhead_map_rotated_180.jpg` 是旋转 180° 后的标准显示底图，其中向右为东、向上
为北。仿射标定使用分布在 25 个建筑碰撞矩形附近的屋顶边缘拟合，而不是使用灰色
图像外框。ENU `(0, 0)` 约对应像素 `(269.15, 1026.25)`；比例约为东向每米
`4.621` 像素、北向每米 `4.6825` 像素。

`map_calibration.json` 的 `content_bounds_px` 记录真正有色内容的像素范围
`[248, 37, 1782, 1074]`。换算后的 ENU 范围约为东向 `-4.58～327.39` 米、北向
`-10.20～211.27` 米。Web 红框和坐标刻度使用该范围；语义 `search_area` 仍由
`semantic_map.json` 独立定义，两者边界接近但含义不同，不能强制合并。

建筑节点和安全覆盖项是保守的轴对齐碰撞外包框，不是精确屋顶轮廓，因此其边缘
不要求跟随所有屋顶凹凸。
