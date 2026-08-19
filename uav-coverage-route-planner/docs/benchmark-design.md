# Coverage Generation × Route Optimization 项目内对比

默认补全策略为 `local_insertion`。`full_greedy` 仅用于项目内对照。策略对比应固定
地图、责任区、相机、飞行高度、Coverage Generation 和初始 Route Optimization，
只切换补全策略，并记录规划时间、补全点数量、逐 patch 覆盖结果、未覆盖面积、
总航程和 transition distance。全局覆盖率不能替代逐 patch 验收结果。

## 1. 研究维度

该对比用于项目内部选择配置和防止性能回归，不用于宣称提出新的算法。Coverage Generation 与 Route Optimization 按下表比较：

| Coverage Generation | 原始顺序 | Greedy | 2-opt/Or-opt | 精确 GTSP/MILP |
|---|---:|---:|---:|---:|
| Global Scanline | 可生成 | 已实现 | 已实现 | 小实例已实现 |
| Cellular Decomposition（BCD） | 可生成 | 已实现 | 已实现 | 小实例已实现 |

两种 Coverage Generation 方法是平行的基础航点生成思路，不构成“基线—改进算法”
关系。它们负责产生标准化 coverage lanes；Route Optimization 只能读取统一的
`RouteOptimizationProblem` 和障碍转场成本矩阵，不能修改覆盖几何。这样才能保证算法
比较使用相同的 lane、起点、安全净空和成本定义。

## 2. 实例层级

1. 合成小实例：矩形、L/U/C 形、单障碍、多障碍、窄通道和断开区域，控制在
   5～15 条 lane，用于证明精确最优并计算 optimality gap；
2. 合成中型实例：20～50 条 lane，用于比较局部搜索质量和计算时间；
3. 云谷裁剪实例：从两架无人机责任区提取小、中、完整三档规模；
4. 外部多边形实例：后续选择 Fields2Benchmark 中适合空中正射覆盖的几何，剥离
   农业车辆特有的地头和转弯模型后测试泛化性。

每个实例必须固定地图、责任区、相机、高度、扫描方向候选、安全距离、起降点和
随机种子。精确对照的第一阶段固定扫描方向和 lane 集合，只优化 lane ordering 与
orientation；第二阶段才比较 sweep direction 的联合枚举。

## 3. 指标

- 最终覆盖率与未覆盖面积；
- coverage lane、transition、避障和返航距离；
- 非检测航程比例；
- 转弯次数、角度加权转弯代价与估计任务时间；
- 最小障碍净空；
- 求解时间、求解状态、最优值、lower bound 和 solver gap；
- 启发式相对已证明最优解的 optimality gap。

若精确求解器证明最优值为 $J^*$，启发式代价为 $J_h$：

$$
\operatorname{gap}_h=\frac{J_h-J^*}{J^*}\times100\%.
$$

若求解超时且只有下界 $LB$ 与 incumbent $UB$，必须分别报告它们以及 solver gap，
不得把 incumbent 称为全局最优解。

## 4. 当前实现状态

- `GlobalScanlineGenerator` 实现 Global Scanline 路线；
- `BCDGenerator` 实现基于扫描拓扑事件的 cell 分解与 cell 内往复路线；
- `LaneJob` 表示具有一个或两个执行方向的覆盖任务；
- `RouteOptimizationProblem` 是所有启发式和精确求解器的统一输入；
- `LaneTransitionCosts` 使用可见图最短路生成统一转场成本矩阵；
- `GreedyLaneRouter` 重现当前障碍距离最近邻基线；
- 两个生成器都已接入统一的路线优化、避障连接、连续视野评估和飞控导出链路；
- `auto` 在 lane 数量不超过 12 时使用 Exact，更大任务使用 Greedy + 2-opt + Or-opt；
- 覆盖补全默认使用 Local insertion，Full Greedy 保留为显式对照选项。

Web 验收界面的“Coverage Generation”可在两条路线之间切换。比较时必须保持责任区、
飞行高度、相机参数、扫描方向、安全距离和后续优化器完全一致。

可视化也必须区分几何层和路由层：实线表示生成器产生的初始 lane，虚线表示优化器
添加的 lane 间转场；BCD 额外显示 cell 边界。不能把相互交叉的转场线统计成重复
coverage lane。所有 BCD cell 共用全局扫描线格架，避免 cell 局部相位造成过密 lane。

补全策略对比的首要指标是规划时间与逐 patch 可行性，其次才是总航程和补全点数量。
Local insertion 的默认地位只表示其计算成本更符合当前交付需求，不表示它在所有实例
上优于 Full Greedy。若任一 patch 低于配置阈值，应如实记录为覆盖不足。
