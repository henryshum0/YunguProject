# Coverage Generation 与 Route Optimization 算法设计

## 1. 问题定义

给定语义地图 \(M\)、用户搜索区域 \(U\)、相机模型 \(C\)、固定飞行高度 \(h\)、起降点 \(s\) 以及建筑安全约束，规划器需要生成一条安全、闭合且可执行的连续飞行任务，使所有可搜索地面满足规定的覆盖要求。

当前系统采用确定性 2.5D 建模：

- 目标视为静止目标；
- 无人机固定高度飞行；
- 相机保持正射，`pitch = -90°`；
- 建筑高度用于飞行避障与视觉遮挡建模；
- 所有规划、补漏与验证均在起飞前完成；
- 当前不包含飞行中的在线重规划。

整体流程为：

```text
Search-space Construction
        ↓
Camera-aware Coverage Geometry
        ↓
Primary Coverage Generation
        ↓
Route Optimization
        ↓
Obstacle-aware Mission Construction
        ↓
Continuous Visibility Evaluation
        ↓
Patch-wise Coverage Validation
        ↓
Coverage Completion if Necessary
        ↓
Final Mission Export
```

---

## 2. 有效搜索区域

真正需要搜索的地面定义为：

$$
\Omega = (U \cap M) \setminus (B \cup E)
$$

其中：

- \(U\)：用户指定搜索区域；
- \(M\)：语义地图有效边界；
- \(B\)：建筑物地面占地；
- \(E\)：显式排除区域。

需要严格区分三个空间：

1. **Search Space**：必须被相机覆盖的地面；
2. **Flight Free Space**：无人机在当前高度允许经过的区域；
3. **Visible Ground**：从当前相机位姿实际可见的地面。

建筑物分别影响这三个空间：

- 建筑占地不参与地面搜索；
- 部分建筑在当前高度构成飞行障碍；
- 建筑墙体会遮挡其后方地面。

因此：

```text
Search Space
≠
Flight Free Space
≠
Visible Ground
```

---

## 3. Camera-aware Coverage Geometry

规划器根据以下参数计算有效地面检测范围：

- 飞行高度；
- 相机水平与垂直 FOV；
- 目标尺寸；
- 图像边缘余量；
- forward overlap；
- side overlap。

系统使用的是满足“目标完整入镜”要求的有效 footprint，而不是直接使用完整物理 FOV。

有效 footprint 进一步决定：

- 沿航线采样间距；
- 相邻 Coverage Lanes 的横向间距。

当前扫描方向定义在 ENU 平面中：

```text
0°  = North–South
90° = East–West
```

扫描方向与相机俯仰角是两个独立概念。

---

## 4. Primary Coverage Generation

Coverage Generation 负责回答：

> **需要生成哪些主扫描航线，才能构成完整的搜索骨架？**

当前实现包含两种并列方法：

```text
                 Effective Search Area
                         │
             ┌───────────┴───────────┐
             │                       │
     Global Scanline                BCD
             │                       │
             └───────────┬───────────┘
                         ↓
                    CoveragePlan
```

两种方法只负责生成 Coverage Geometry，不负责决定最终访问顺序。

---

## 5. Global Scanline

Global Scanline 采用 **line-first** 策略：

```text
Effective Search Area
        ↓
Global Parallel Scanline Lattice
        ↓
Geometry Intersection
        ↓
Coverage Segments
```

首先在整个责任区建立统一的平行扫描线格架，再与搜索边界、孔洞和建筑占地区域求交。

例如：

```text
────────────────────────────
──────────████──────────────
──────────████──────────────
────────────────────────────
```

建筑或孔洞会将一条全局扫描线裁剪成多个 Coverage Segments。

该方法的主要特点是：

- 结构简单；
- 确定性强；
- 扫描线具有统一 spacing 和 phase；
- 易于复现和分析。

---

## 6. Boustrophedon Cellular Decomposition

BCD 全称为 **Boustrophedon Cellular Decomposition**。

其核心思想是 **cell-first**：

```text
Effective Search Area
        ↓
Topology Sweep
        ↓
Boustrophedon Cells
        ↓
Cell-wise Lawnmower Lanes
        ↓
CoveragePlan
```

BCD 沿给定 sweep direction 分析自由空间截面的连通关系。

当自由空间发生：

```text
1 → 2
```

或：

```text
2 → 1
```

等 split / merge 事件时建立新的 Cell。

每个 Cell 内再生成往复式扫描航线：

```text
→→→→→→
      ↓
←←←←←←
↓
→→→→→→
```

因此两种方法的核心区别可以概括为：

```text
Global Scanline:
Line First → Geometry Clipping

BCD:
Topology Decomposition → Cell-wise Lines
```

BCD 只决定覆盖区域如何分解，不决定 Cell 或 Lane 的最终访问顺序。

---

## 7. Coverage Lanes

Coverage Generation 内部可以使用较密集的参考采样点描述相机覆盖：

```text
●──●──●──●──●──●
```

但 Route Optimization 不直接优化所有采样点。

同一连续扫描段被抽象为一条可双向执行的 Coverage Lane：

```text
A ●────────────────● B

Forward : A → B
Reverse : B → A
```

因此需要区分：

```text
Coverage Reference Samples
≠
Route Optimization Nodes
≠
Flight-control Waypoints
```

该抽象将组合优化对象从大量采样点降低为有限数量的可定向 Coverage Lanes。

---

## 8. Route Optimization

Coverage Generation 完成后得到 Lane 集合：

$$
\mathcal{L} = \{L_1,L_2,\ldots,L_K\}
$$

Route Optimization 不再决定“哪里需要搜索”，而只联合优化：

1. **Lane Ordering**：各 Coverage Lane 的访问顺序；
2. **Lane Orientation**：每条 Lane 从哪一端进入。

当前主要优化目标为：

$$
J =
D_{\mathrm{transition}}
+
D_{\mathrm{return}}
$$

其中：

- \(D_{\mathrm{transition}}\)：不同 Coverage Lanes 之间的转场距离；
- \(D_{\mathrm{return}}\)：任务结束后的返航距离。

Coverage 和安全约束均为硬约束。

当前 `auto` 求解策略为：

```text
K ≤ 12
    ↓
Exact Dynamic Programming

K > 12
    ↓
Greedy
    ↓
2-opt
    ↓
Or-opt
```

---

## 9. Obstacle-aware Routing

Lane 之间的转场不能简单使用欧氏距离。

规划器根据建筑安全缓冲区建立 Visibility Graph，并计算障碍约束下的最短路径：

$$
c(i,j)=d_{\mathrm{VG}}(i,j)
$$

其中 \(d_{\mathrm{VG}}\) 为 Visibility Graph 中两点之间的最短可行距离。

Route Optimization 完成以后，系统才构造完整任务中的：

- `coverage_lane`；
- `connector`；
- `obstacle_avoidance`；
- `return_home`。

因此：

```text
Primary Coverage Generation
        ↓
Coverage Lanes
        ↓
Route Optimization
        ↓
Connector / Avoidance / Return
```

Connector 不参与初始 Coverage Generation，因为其具体位置只有在 Lane Ordering 和 Orientation 确定之后才能得到。

---

## 10. Continuous Mission Visibility

最终任务是连续视频搜索任务，而不是离散拍照任务。

因此最终 Coverage 由整条实际飞行轨迹共同产生：

$$
V_{\mathrm{mission}}
=
V_{\mathrm{coverage}}
\cup
V_{\mathrm{connector}}
\cup
V_{\mathrm{avoidance}}
\cup
V_{\mathrm{return}}
$$

也就是说：

- Coverage Lane 产生主要搜索观测；
- Connector 顺路产生额外观测；
- 避障航段产生额外观测；
- 返航航段同样产生额外观测。

这些额外观测虽然不是为了 Coverage Generation 专门生成，但无人机实际经过时相机仍持续工作，因此计入最终 Mission Coverage。

如果 Connector 等航段恰好补上了 Primary Coverage 中的局部缺口，则无需额外生成补漏点。

---

## 11. Building Occlusion

理论 Camera Footprint 不等于实际可见地面。

对于无人机位姿 \(p\)，首先计算理论地面 footprint：

$$
F(p)
$$

再计算建筑物产生的遮挡区域：

$$
O(p)
$$

实际有效可见地面为：

$$
V(p)=F(p)\setminus O(p)
$$

因此系统不仅删除建筑物本身的地面占地，还考虑建筑墙体对其后方地面的遮挡。

示意如下：

```text
UAV
  \
   \ Camera View
    \
     █████ Building
     █████╲
___________╲XXXXXXXX
             Occluded Ground
```

需要注意，遮挡区域依赖无人机当前观察位置，因此建筑后方不存在固定的永久不可见区域。

---

## 12. Patch-wise Coverage Validation

为了避免全局平均覆盖率掩盖局部漏扫，有效搜索区域被划分为多个 Patch：

$$
\mathcal{P}=\{P_1,P_2,\ldots,P_N\}
$$

设最终任务所有有效可见区域的并集为 \(V\)，则 Patch \(P_i\) 的覆盖率定义为：

$$
r_i=
\frac{
\operatorname{Area}(P_i\cap V)
}{
\operatorname{Area}(P_i)
}
$$

任务要求：

$$
r_i \ge \eta,
\qquad
\forall P_i\in\mathcal{P}
$$

当前覆盖阈值为：

$$
\eta=0.9999
$$

因此 Coverage Constraint 是：

> **per-patch hard constraint**

而不是全局平均覆盖率约束。

某一个 Patch 未达到阈值，即使整体平均覆盖率很高，任务仍不能判定为完成。

---

## 13. Coverage Completion

只有在完整 Mission Visibility Evaluation 后仍存在：

$$
r_i < \eta
$$

的 Patch 时，才触发 Coverage Completion。

补漏流程为：

```text
Uncovered Patch
        ↓
Residual Uncovered Geometry
        ↓
Safe Observation Candidates
        ↓
Visibility Gain / Route Insertion Cost
        ↓
Completion Point
        ↓
Re-routing
        ↓
Continuous Visibility Re-evaluation
```

Completion 的目标不是补“缺失航点”，而是补：

> **最终连续任务中实际仍未被有效观察的地面。**

当前提供两种补全路线策略：

- `local_insertion`（默认）：固定初始 Coverage Lanes 的顺序和方向，按障碍最短路增量将补全点插入 lane 之间，不在 lane 内部插点，也不执行补全后的全局重排；
- `full_greedy`（对照）：加入补全点后，将原 Coverage Lanes 与补全点一起重新执行 Greedy 排序，并在最终阶段裁剪冗余补全点。

默认采用 `local_insertion`，因为它将补全限制为对既有路线的局部修改，避免每轮补全
都重新优化完整任务，规划时间更稳定。它不保证得到补全点最少或航程全局最短的结果，
也可能在严格阈值下保留少量未覆盖 patch。两种策略都会重新生成安全 Connector，
并对完整连续任务执行逐 patch 覆盖验证；未达阈值的结果仍明确标记为不可执行。

候选点综合考虑：

- 新增可见面积；
- 插入当前路线带来的额外飞行代价。

因此系统倾向于选择能够以较小航程增量覆盖较大残余区域的位置。

---

## 14. Primary Coverage 与 Mission Coverage

系统需要区分两个概念。

### Primary Coverage

由 Global Scanline 或 BCD 生成的主 Coverage Lanes 所形成的覆盖骨架。

### Mission Coverage

最终完整任务产生的实际连续覆盖：

```text
Primary Coverage
+
Connector Coverage
+
Obstacle-avoidance Coverage
+
Return-home Coverage
+
Completion Coverage
```

最终任务是否满足 Coverage Constraint，以 **Mission Coverage** 为准。

因此整个系统的职责划分为：

```text
Coverage Generation
=
生成主要扫描任务

Route Optimization
=
决定这些任务如何执行

Mission Coverage Validation
=
判断完整真实任务是否真正完成搜索

Coverage Completion
=
修复最终仍然存在的漏扫区域
```

---

## 15. Final Mission Generation

所有 Patch 通过覆盖验证后，最终几何路线被进一步离散为飞控参考点。

相邻控制点满足：

$$
\|q_{i+1}-q_i\| \le d_{\mathrm{control}}
$$

该步骤只进行轨迹插值，不重新改变：

- Coverage Geometry；
- Lane Ordering；
- Lane Orientation；
- Route Topology。

最终只有在所有可搜索 Patch 满足覆盖要求，并且完整路径满足飞行安全约束时：

```text
mission_status = ready
```

否则：

```text
mission_status = infeasible_coverage
```

---

## 16. 核心设计原则

当前算法遵循以下原则：

1. **Coverage First**
   Coverage 是硬约束，不能为了缩短路线而牺牲覆盖。

2. **Safety First**
   Coverage、Connector、Avoidance、Completion 和 Return 均必须满足飞行安全约束。

3. **Separate Coverage from Routing**
   Coverage Generation 决定“需要飞哪些主扫描航线”，Route Optimization 决定“如何组织这些航线”。

4. **Evaluate the Actual Mission**
   最终覆盖必须基于完整连续轨迹、相机模型和建筑遮挡进行验证，而不能仅依据理论扫描线判断。

5. **Use Incidental Observations**
   Connector、避障和返航航段产生的真实观测计入最终 Mission Coverage。

6. **Repair Only Residual Coverage Defects**
   Completion 只处理最终可见性验证后仍然存在的残余未覆盖区域。

---

## 17. 算法流程总结

```text
Semantic Map
Search Area
Camera / Flight Parameters
        ↓
Effective Search Area
        ↓
Camera-aware Footprint
        ↓
Global Scanline / BCD
        ↓
Primary Coverage Lanes
        ↓
Lane Ordering + Orientation
        ↓
Obstacle-aware Routing
        ↓
Coverage + Connector + Avoidance + Return
        ↓
Continuous Visibility Evaluation
        ↓
Building Occlusion
        ↓
Patch-wise Coverage Validation
        ↓
        Covered?
       /       \
     Yes        No
      ↓          ↓
 Final Plan   Completion
                 ↓
              Re-route
                 ↓
             Re-evaluate
```

整个系统的核心思想可概括为：

> **先生成结构化的主 Coverage Backbone，再优化其执行顺序，随后基于完整连续任务计算真实可见覆盖，并只对最终残余漏扫区域进行补全。**
