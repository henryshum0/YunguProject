# 云谷 Yungu2030 语义地图格式

本文记录当前解析器实际支持的输入格式。以下字段来自对源数据的检查，规划器不会把地图范围或建筑数量硬编码到算法中。

顶层字段为 `schema_version`、`world_name`、`coordinate_frame`、`units`、
`search_area`、`nodes`、`building_safety_overrides`、`excluded_search_regions` 和 `metadata`。

- 地图声明 schema `1.0`、坐标系 `ENU`、单位 `meters`；
- `search_area` 当前为四点矩形，实际边界始终从文件读取；
- 示例数据有 43 个节点：25 栋建筑、14 个区域和 4 个交通设施；
- 当前节点形状均为矩形，由 ENU 米制 `min_corner` 和 `max_corner` 描述；
- 节点属性包括 `category`、`type`、`label`、`passability`、`visibility`、
  `elevation_min_m`、`elevation_max_m` 和可选的 `ground_contact`；
- `metadata` 记录真值排除状态和源资产。

`building_safety_overrides` 保存由视觉网格测得的保守 XY 外包范围和垂直高度区间。
存在覆盖项时，飞行避障和建筑地面扣除均使用该安全体，而不是节点中较小的原始
矩形。`ground_contact` 默认为 `true`；只有资产确实悬空且桥下地面仍需检测时才能
设为 `false`。

`excluded_search_regions` 用于保存人工确认的不可达、无需检测地面。每项包含唯一
`id`、显示名称 `label`、排除原因 `reason` 和矩形 `shape`。这些区域会从每架无人机
的责任区中自动扣除，不生成检测航线，也不进入覆盖率分母；若与建筑重叠，面积不会
重复计算。`ground_contact=true` 的建筑同样从地面搜索区扣除，其高度下界应表达为
地面高程。

当前解析器有意严格校验这一已观察格式。增加新图形类型或 schema 版本前，应先检查真实输入并为其增加模型与测试。
