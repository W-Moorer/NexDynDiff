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

## 3. 针对关键疑问的技术回答

你提出的两个问题非常关键，这里先给出直接结论。

### 3.1 为什么 TMCD 在当前任务上通常更“精确”？

严格说，TMCD 的优势不是“永远更精确”，而是在你当前这类三角网格接触问题中，它对**接触几何特征**的刻画更直接：

1. **原始几何图元精确距离**  
   TMCD 直接在点/边/三角图元上做最小距离与对应特征配对（如 pt/ee），不是在体素场上插值。

2. **特征级接触分类**  
   现有管线中会显式区分 point-point、point-edge、point-triangle、edge-edge，再进入不同接触能量分支，这有助于保持接触法向与切向构造的一致性。

3. **退化情形专门处理**  
   边-边近共线、近零面积等退化在图元方法里有专门策略；SDF 若分辨率或梯度质量不足，这些情形容易出现法向抖动。

4. **二阶信息与牛顿法耦合更自然**  
   你当前求解器依赖稳定的梯度/Hessian 行为。图元距离链路在接触点局部几何上更可控。

因此，TMCD 的“精度优势”更准确的表述是：**在离散网格接触的特征定位与退化鲁棒性上，通常更稳健**。

### 3.2 你的判断是否成立：若 SDF 足够准确，是否可以达到同等精度？

结论：**成立（在条件满足时）**。如果 SDF 具备足够高分辨率、良好梯度质量和一致的几何重建，其接触距离与法向可以逼近图元法结果。

但工程上要满足这几个条件并不便宜：

- 窄带分辨率要足够高（内存与带宽代价上升）；
- 梯度与法向需平滑且无台阶噪声；
- 薄特征/尖锐边角要防 aliasing；
- 动态物体需频繁更新 SDF（构建成本高）。

所以“理论可达”与“工程可达”之间有成本差，这也是我建议采用混合策略的原因。

### 3.3 是否可以做 SDF-SDF 快速接触能量，规避“仅节点采样”？

结论：**可以，而且这是很好的研究方向**。关键是不要只做“节点 -> 对方 SDF”的点采样，而是做**面/窄带积分或对称采样**。

推荐两个可落地方案：

1. **双向表面积分（surface quadrature）**  
   $$E_c = \int_{\Gamma_A} b(\phi_B(x))\,dA + \int_{\Gamma_B} b(\phi_A(y))\,dA$$
   - 其中 $\Gamma_A,\Gamma_B$ 是两物体表面；
   - 通过三角面高斯积分（或自适应采样）评估，不依赖单节点。

2. **窄带体积分（narrow-band volumetric contact）**  
   $$E_c = \int_{\Omega} w(\phi_A,\phi_B)\,\psi(\phi_A,\phi_B)\,dV$$
   - 在两 SDF 的窄带交叠区域积分；
   - 对穿透深度和法向更平滑，但积分计算更重。

两者都能规避“只看原始节点”的局限；第一种更容易与现有网格数据与 SymX 能量链路对接。

---

## 4. SDF 融合是否可能提速？结论先行

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

## 5. 建议采用的技术路线：混合 TMCD + SDF

不建议直接替换 TMCD。建议采用三层混合：

1. **宽相**：保留 TMCD/AABB/黑名单机制；
2. **中相（新增）**：SDF 快速裁剪候选（early reject / early accept）；
3. **窄相**：
   - 对通过筛选的候选继续走现有图元精确几何距离（保证特征定位精度）；
   - 或对部分对象对采用 SDF-SDF 面积分/窄带积分（可选研究模式）。

这样可以在保证鲁棒性的同时逐步吃到加速收益。

---

## 6. 与当前代码的融合点设计

## 6.1 数据结构新增（建议）

新增模块（建议目录）：

- `nexdyndiff/src/models/interactions/sdf/SignedDistanceField.h`
- `nexdyndiff/src/models/interactions/sdf/SignedDistanceField.cpp`
- `nexdyndiff/src/models/interactions/sdf/SDFQuery.h`

核心结构：

- `SDFGrid`：体素数据、分辨率、局部包围盒、世界变换；
- `SDFObjectBinding`：接触对象 group 与 SDF 的绑定关系；
- `SDFCache`：每帧/每步缓存距离与法向查询结果；
- `SDFContactQuadrature`：面采样点、权重、局部法向与稳定化参数（用于 SDF-SDF 能量）。

## 6.2 接口嵌入位置（最小侵入）

在 `EnergyFrictionalContact` 中增加：

- `enable_sdf_acceleration` 开关；
- `sdf_filter_margin`, `sdf_narrow_band`, `sdf_rebuild_policy` 参数；
- 对象注册接口：`bind_sdf_to_handler(ContactHandler h, SDFConfig cfg)`。
- 可选能量模式：`sdf_contact_mode = {off, filter_only, point_to_sdf, sdf_surface_quadrature}`。

插入点：

1. `_before_energy_evaluation__update_contacts(...)` 里，TMCD 返回候选后，先做 SDF 过滤再进入当前 push_back 分发。
2. `_before_time_step__update_friction_contacts(...)` 复用同一筛选逻辑，减少摩擦候选。
3. `_is_intermidiate_state_valid(...)` 可选增加 SDF 的 penetration 快速预判（仅辅助，不替代 intersection）。

---

## 7. “符号距离场”与 SymX 的关系设计

你的关键想法是“符号距离场”，建议区分两层含义：

1. **几何层 SDF**：离散网格/解析 SDF，用于快速距离与法向查询；
2. **能量层符号化**：将 `d = phi(x)`（及近似法向）带入 SymX 势能表达式。

建议先做 1，再渐进做 2。

## 7.1 第一阶段（推荐）

- SDF 仅用于候选过滤与初值估计；
- 能量仍用现有 IPC 距离表达式，避免精度风险。

优点：风险低，提速收益快见效。

## 7.2 第二阶段（研究创新）

对“点-静态障碍物”类型接触，定义：

- $d(x)=\phi(x)$，$n(x)=\nabla\phi(x)/\|\nabla\phi(x)\|$；
- 接触势能仍使用当前 barrier 形式：$E_c=b(d)$；
- 摩擦切向投影由 $n(x)$ 构造。

这一步可形成论文创新：SDF-IPC 混合势能与稳定性分析。

## 7.3 第三阶段（回答你的第二个问题：SDF-SDF 非节点方案）

对“物体 A 与物体 B 都有 SDF”的场景，建议采用**对称表面积分接触能量**：

$$
E_c = \sum_{q \in Q_A} w_q\,b(\phi_B(x_q)) + \sum_{r \in Q_B} \tilde{w}_r\,b(\phi_A(y_r))
$$

其中：

- $Q_A,Q_B$ 为两侧表面采样点（可由三角面积分点或重建零水平集积分点得到）；
- $w_q,\tilde{w}_r$ 为面积权重；
- 法向可由 $\nabla\phi$ 归一化得到，并用于摩擦切向基构造。

该方案的优点：

- 不依赖原始节点，能覆盖边角与面内接触；
- 对称形式可降低接触偏置；
- 更容易与 SymX 的统一能量表达融合。

注意事项：

- 需要控制采样密度与窄带范围，否则会抵消提速；
- 需加入法向平滑/稳定化，避免摩擦方向噪声。

---

## 8. 实施分期（建议 3 阶段）

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
- 增加 `sdf_surface_quadrature` 原型（小规模场景先验证）；
- 给出与原 IPC 的一致性测试和稳定性对比；
- 撰写可发表实验章节（精度-速度 Pareto）。

---

## 9. 评测指标与实验矩阵

## 9.1 指标

- **效率**：step time、proximity time、候选构建 time、Newton 次数；
- **稳定性**：失败步比例、线搜索失败率、穿透事件；
- **精度**：轨迹偏差、能量变化、接触力统计误差。

建议新增“几何一致性指标”：

- TMCD 最近点法向 与 SDF 法向夹角均值/最大值；
- 接触点偏置（feature location bias）统计。

## 9.2 实验矩阵

1. 静态高模障碍 + 单可变形体；
2. 多刚体 + 可变形体混合接触；
3. 高接触密度堆叠场景；
4. 消融：
   - TMCD-only；
   - TMCD + SDF-filter；
   - point-to-SDF；
   - SDF-surface-quadrature（可选）。

---

## 10. 关键风险与规避

1. **SDF 重建成本过高**  
   - 先从静态障碍做起；动态体仅做刚体变换重定位，不重建网格 SDF。

2. **法向误差导致摩擦方向不稳定**  
   - 使用窄带、梯度平滑、与几何法向混合策略。

3. **边-边接触近似误差大**  
   - 基础模式下保持边-边仍走现有图元精确路径；SDF-SDF 模式优先用于点/面主导场景。

4. **与现有参数体系耦合复杂**  
   - 以开关控制渐进上线，保证可回退。

---

## 11. 最终建议（面向你当前研究目标）

如果你的目标是“研究创新 + 工程可落地 + 可投稿”，推荐路线：

- **短期**：先做 `TMCD + SDF 过滤`，以稳健提速为目标；
- **中期**：扩展到 `SDF 符号化接触势能`（点-静态障碍）；
- **长期**：推进 `SDF-SDF 对称表面积分`，形成“非节点采样”的方法学贡献，并与可微反演联动。

这条路线与 NexDynDiff 当前结构兼容度高，风险可控，且容易产出可量化结果。

---

## 附：建议优先改动位置（开发导航）

- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.h](nexdyndiff/src/models/interactions/EnergyFrictionalContact.h)
- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L365-L770)
- [nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp](nexdyndiff/src/models/interactions/EnergyFrictionalContact.cpp#L249-L265)
- [nexdyndiff/src/core/NewtonsMethod.cpp](nexdyndiff/src/core/NewtonsMethod.cpp#L185-L215)

可先围绕以上文件做最小原型。