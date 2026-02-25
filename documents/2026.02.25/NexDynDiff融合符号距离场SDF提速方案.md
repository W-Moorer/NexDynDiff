# NexDynDiff 融合符号距离场（SDF）用于接触提速的可行性分析与实施方案

## 1. 背景与目标

你提出“在当前项目接触检测/接触能量中融合符号距离场（SDF）”的想法是有价值的。结合现有代码结构，该思路更适合作为**混合管线（TMCD + SDF）**，而不是一次性替换现有 IPC 点-边/点-三角形/边-边距离体系。

本方案目标：

1. 判断在当前 NexDynDiff 架构下，SDF 是否能带来实质提速；
2. 给出最小侵入式融合路径；
3. 给出可量化评测指标与里程碑。

---

## 2. 现有接触链路与潜在瓶颈（基于当前代码）

当前关键流程位于接触模型：

- 接触候选更新：`_before_energy_evaluation__update_contacts(...)`
- 摩擦候选更新：`_before_time_step__update_friction_contacts(...)`
- 近距离检测：`_run_proximity_detection(...)`（TMCD）
- 穿透判定：`_run_intersection_detection(...)`

对应代码可见：

- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L365-L527)
- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L528-L770)
- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L249-L265)

### 2.1 性能观察（结构性）

- 近距离检测和候选重建发生在 `before_energy_evaluation` 回调中；
- 牛顿迭代 + 线搜索会多次触发能量评估，因此候选集重建次数高；
- 在高分辨率网格或高接触密度场景，点-三角/边-边候选处理成本上升明显。

这意味着：**只要能降低候选对数量，或者降低每个候选的几何距离计算成本，就有机会提速**。

---

## 3. SDF 融合是否可能提速？结论先行

## 结论（建议）

**可以提速，但需要“分场景、分阶段”使用。**

### 明显可能提速的场景

1. **静态或准静态刚体障碍物 + 可变形体接触**（典型机器人环境接触）；
2. 障碍物三角面很多，TMCD 窄相代价高；
3. 接触主要是点/面接触，边-边退化情况不占主导。

### 可能不提速甚至变慢的场景

1. 大量动态可变形体之间相互接触（SDF频繁重建代价高）；
2. 需要高精度边-边接触法向与二阶信息，且 SDF 分辨率有限；
3. 高分辨率 SDF 造成内存与缓存压力。

### 经验预期（工程估计）

- 若用于**静态刚体环境**的接触筛选与近距查询，接触检测阶段可期待约 **1.3x–3x** 加速；
- 全流程（包含牛顿线性求解）整体加速通常小于检测加速，需以日志分项时间验证。

---

## 4. 建议采用的技术路线：混合 TMCD + SDF

不建议直接替换 TMCD。建议采用三层混合：

1. **宽相**：保留 TMCD/AABB/黑名单机制；
2. **中相（新增）**：SDF 快速裁剪候选（early reject / early accept）；
3. **窄相**：
   - 对通过筛选的候选继续走现有精确几何距离（保证精度）；
   - 或对部分对象对采用 SDF 直接近似距离（可选加速模式）。

这样可以在保证鲁棒性的同时逐步吃到加速收益。

---

## 5. 与当前代码的融合点设计

## 5.1 数据结构新增（建议）

新增模块（建议目录）：

- `nexdyndiff/src/models/interactions/sdf/SignedDistanceField.h`
- `nexdyndiff/src/models/interactions/sdf/SignedDistanceField.cpp`
- `nexdyndiff/src/models/interactions/sdf/SDFQuery.h`

核心结构：

- `SDFGrid`：体素数据、分辨率、局部包围盒、世界变换；
- `SDFObjectBinding`：接触对象 group 与 SDF 的绑定关系；
- `SDFCache`：每帧/每步缓存距离与法向查询结果。

## 5.2 接口嵌入位置（最小侵入）

在 `EnergyFrictionalContact` 中增加：

- `enable_sdf_acceleration` 开关；
- `sdf_filter_margin`, `sdf_narrow_band`, `sdf_rebuild_policy` 参数；
- 对象注册接口：`bind_sdf_to_handler(ContactHandler h, SDFConfig cfg)`。

插入点：

1. `_before_energy_evaluation__update_contacts(...)` 里，TMCD 返回候选后，先做 SDF 过滤再进入当前 push_back 分发。
2. `_before_time_step__update_friction_contacts(...)` 复用同一筛选逻辑，减少摩擦候选。
3. `_is_intermidiate_state_valid(...)` 可选增加 SDF 的 penetration 快速预判（仅辅助，不替代 intersection）。

---

## 6. “符号距离场”与 SymX 的关系设计

你的关键想法是“符号距离场”，建议区分两层含义：

1. **几何层 SDF**：离散网格/解析 SDF，用于快速距离与法向查询；
2. **能量层符号化**：将 `d = phi(x)`（及近似法向）带入 SymX 势能表达式。

建议先做 1，再渐进做 2。

## 6.1 第一阶段（推荐）

- SDF 仅用于候选过滤与初值估计；
- 能量仍用现有 IPC 距离表达式，避免精度风险。

优点：风险低，提速收益快见效。

## 6.2 第二阶段（研究创新）

对“点-静态障碍物”类型接触，定义：

- $d(x)=\phi(x)$，$n(x)=\nabla\phi(x)/\|\nabla\phi(x)\|$；
- 接触势能仍使用当前 barrier 形式：$E_c=b(d)$；
- 摩擦切向投影由 $n(x)$ 构造。

这一步可形成论文创新：SDF-IPC 混合势能与稳定性分析。

---

## 7. 实施分期（建议 3 阶段）

## Phase 0：基线与可观测性（1–2 周）

- 增加分项计时：
  - proximity 检测耗时；
  - 接触候选构建耗时；
  - 摩擦候选构建耗时；
  - 每步候选数量统计。
- 输出 CSV/日志，建立评测基线。

## Phase 1：SDF 过滤器（2–4 周）

- 实现静态刚体三角网格离线 SDF 构建；
- 在 `_before_energy_evaluation__update_contacts` 接入过滤；
- 提供 3 个模式：`off / filter-only / filter+approx-distance`。

验收标准：

- 物理误差可控（穿透、能量、轨迹偏差在阈值内）；
- 接触阶段耗时显著下降。

## Phase 2：符号化接触能量扩展（4–8 周）

- 对“点-静态障碍物”新增 SDF contact energy 分支；
- 给出与原 IPC 的一致性测试和稳定性对比；
- 撰写可发表实验章节（精度-速度 Pareto）。

---

## 8. 评测指标与实验矩阵

## 8.1 指标

- **效率**：step time、proximity time、候选构建 time、Newton 次数；
- **稳定性**：失败步比例、线搜索失败率、穿透事件；
- **精度**：轨迹偏差、能量变化、接触力统计误差。

## 8.2 实验矩阵

1. 静态高模障碍 + 单可变形体；
2. 多刚体 + 可变形体混合接触；
3. 高接触密度堆叠场景；
4. 消融：
   - TMCD-only；
   - TMCD + SDF-filter；
   - SDF-approx（可选）。

---

## 9. 关键风险与规避

1. **SDF 重建成本过高**  
   - 先从静态障碍做起；动态体仅做刚体变换重定位，不重建网格 SDF。

2. **法向误差导致摩擦方向不稳定**  
   - 使用窄带、梯度平滑、与几何法向混合策略。

3. **边-边接触近似误差大**  
   - 保持边-边仍走现有精确几何路径，SDF 只筛选。

4. **与现有参数体系耦合复杂**  
   - 以开关控制渐进上线，保证可回退。

---

## 10. 最终建议（面向你当前研究目标）

如果你的目标是“研究创新 + 工程可落地 + 可投稿”，推荐路线：

- **短期**：先做 `TMCD + SDF 过滤`，以稳健提速为目标；
- **中期**：扩展到 `SDF 符号化接触势能`（点-静态障碍），形成方法学贡献；
- **长期**：探索 SDF 与可微反演/参数辨识联动，形成你的前沿方向主线。

这条路线与 NexDynDiff 当前结构兼容度高，风险可控，且容易产出可量化结果。

---

## 附：建议优先改动位置（开发导航）

- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.h](nexdyndiff/src/models/interactions/EnergyFrictionalContact.h)
- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L365-L770)
- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L249-L265)
- [nexdyndiff/src/core/NewtonsMethod.cpp](nexdyndiff/src/core/NewtonsMethod.cpp#L185-L215)

可先围绕以上文件做最小原型。