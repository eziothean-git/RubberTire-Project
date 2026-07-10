# RubberTire 改进计划（v0.3.0 基线评审）

状态：评审中（已合入 advisor 两轮反馈，见文末处理记录）。基于对 `src/RubberTire/` 全部物理与 UI 代码的通读。

总体判断：物理架构健康——接触 gate、有效质量限幅、2×2 静摩擦块求解器、扰动前馈方向正确。UI 是当前体验短板：80 个参数全部为纯文本输入框、无重置/批量应用、Live 图表在建造模式下基本失效、存在每帧全量重建类性能问题。

运行环境约束：Unity/PhysX，100 Hz 固定步长，Jacobi 式力累积（`AddForce*` 在 FixedUpdate 后统一积分，同帧内 `GetPointVelocity` 看不到本帧已提交的力/冲量）；.NET 3.5 Mono（GC 敏感）。每个轮子是独立 Rigidbody，经 joint 连接底盘。

---

## A. 物理功能性缺口

### A1. 接触只沿"重力向下"射线 → 墙面/天花板/环形轨道失效
证据：`RubberTireScript.cs` `SimulateFixedUpdateAlways` 中 `downDir` 固定为重力在垂直轮轴平面上的投影（约 L278-281），`GatherTopContactSamples` 只沿该方向发射。

后果：
- 侧壁行驶、loop 顶部、翻越垂直障碍时探测不到接触，轮子穿墙/掉落。环形轨道是 Besiege 常见玩法。
- trigger 维护的 `contacts` HashSet（L84）只被用作一个布尔（L253），信息浪费。

方案：轮平面内径向多方向射线扇——以 `downDir` 为中心向两侧扩展（如 ±90°~±180° 内若干根径向射线），命中后仍走现有 per-collider 聚合 + gate + 摩擦管线，下游不改。备选：用 trigger `contacts` 里的 collider 做 `Physics.ComputePenetration` 求接触方向。

待确认风险：
- `pen = R - dist` 对任意径向射线语义仍成立（径向定义），需确认。
- gate fade-in（默认 5 帧）会让侧壁首次接触软 5 帧，可能需要配合调小。
- per-rigidbody `sampleWeight` 归一化（同一 rigidbody 的样本权重和=1）在法向差异大的多方向接触下是否欠支撑，需评审。

### A2. 轮心穿透超过 R 时接触瞬间丢失
证据：`GatherTopContactSamples` 射线从 `center` 发出（L637-645）。重载/高速落地时轮心进入地面 collider 内部，raycast 不命中背面 → 支撑瞬间消失 → 弹射或穿地。

方案：射线起点后退 `center - downDir * (0.5R)`，`maxDist` 相应加 `0.5R`，`pen` 计算扣除 backoff。改动局部、便宜。

### A3. 落地冲击被 contact gate 削弱
证据：`Support.cs` `EvaluateAndApplySupportForce`：`stopCompressionImpulse`（L61-64，按有效质量止住压入速度的耗散项）与弹簧项一起被 `gate` 乘（L74）。fade-in 默认 5 帧，硬着陆前几帧只有 20%→100% 支撑 → 穿透加深 → 随后满弹簧力弹出。

方案：gate 只作用于弹簧/ERP 部分；`stopCompressionImpulse` 直通（仍受 `maximumSafeImpulse` 能量护栏限幅）。gate 的设计意图是抑制"忽有忽无"的能量注入，而止陷项纯耗散、不注入能量。

待确认风险：射线扇/噪声接触下，止陷冲量直通是否引入新的抖动路径。

### A4. 自车 collider 未过滤
证据：`GatherTopContactSamples` 只排除 `c.attachedRigidbody == Rigidbody`（L657，轮子自身）。悬挂压缩时底盘/挡泥板 block 进入射线路径会被当地面，机器内部互相施力。

方案：至少过滤与本 block 直接关节相连的 rigidbody（经 `BlockBehaviour`/joint connectedBody 查询）。注意不能过滤整台机器——踩自己搭的坡道是合法玩法。

### A5. 高转速求解发散（"甩大饼"）——需要真正的 hard cap
证据：用户实测——转速超过约 10×（游戏内倍数）后轮子进动甩摆、求解发散。代码侧根因：
- `maxAngularVelocityLimit` 默认 250 rad/s，UI 上限 1000（Contact tab "Maximum angular velocity"），每步写入 `Rigidbody.maxAngularVelocity`。100 Hz 下 250 rad/s = 每步 2.5 rad（143°）转角，joint 求解器在每步转角 ≳0.5 rad 后守不住轮轴对齐，约束修正力反过来放大误差 → 进动/甩摆。PhysX `maxAngularVelocity` 只钳 |ω| 幅值，不阻止自旋轴漂移，所以现有设置挡不住该发散模态。
- `enableEngineCurve=false` 时驱动扭矩为常数 `enginePeakTorque`，无任何转速切断，能量持续泵入钳位。
- `ApplyAxleSpinStabilization` 仅在无载荷时阻尼**轴向**分量，与甩摆（垂直轴）模态无关。

方案（三层防线）：
1. 不可配置的 `SpinHardCap`：按每步转角 ≤~0.5 rad 推导（100 Hz → ~50 rad/s，留裕量取 60–80，需真机标定）。`Rigidbody.maxAngularVelocity = min(maxAngularVelocityLimit, SpinHardCap)`；UI 范围上限同步下调。
2. 扭矩端速度空间钳位：`ApplyDriveBrake` 的 `AddTorque` 前，`maxTauRemaining = I_axis * max(0, SpinHardCap - |omegaAxis|) / dt`（`I_axis` 复用 `GetInertiaAroundWorldAxis`），仅对与 `omegaAxis` 同号（泵能）方向钳 `tau`，制动方向不限。引擎曲线关闭时也不再无限泵能；`driveTorque` 测试钩子同样受限。
3. 反甩摆主动阻尼：阻尼**相对底盘**角速度中垂直于轮轴的分量 `ωperp = ωrel - (ωrel·axis)·axis`，载荷下同样生效（载荷不阻止 joint 发散），velocity-space 限幅防过冲。注意必须用相对角速度——绝对 ω 的垂直分量包含整车转向/俯仰，直接阻尼会抵抗转向。底盘 body 查询与 B4 复用同一机制。

验证场景：高挡位空转全油门拉满转速；`enableEngineCurve=false` + 大扭矩；跳台落地瞬间高转速触地。

---

## B. 求解器一致性

### B1. 驱动扭矩对静摩擦求解器有一帧滞后
证据：`Drivetrain.cs` `ApplyDriveBrake` 用 `ForceMode.Force` 施加驱动/制动扭矩（L103-104），下一步积分才生效；`Lateral.cs` `TrySolveStaticImpulse` 基于当前滑移速度求解，本帧自加的驱动只能下一帧被扰动估计器（`disturbanceForward/Side`）补偿。坡道起步/起步瞬间有一帧微抖。

方案：驱动/制动扭矩是自己施加的已知量，换算为接触点预期滑移增量前馈进 `targetForward/Side`。

待确认风险：Jacobi 积分语义下前馈量的正确算法，以及如何避免双重计入（既前馈、又被下一帧扰动估计器学到一遍）。

### B2. 静→动交接处刷毛状态不同步
证据：`Lateral.cs` `ApplyLowSpeedStaticConstraint`：静摩擦分支生效（lockBlend≈1）时施加求解器冲量，但 `state.shearDispWorld` 仍按 slipVelocity 独立积分。速度越过 `StaticLockOffSpeed` 切回动态分支的瞬间，刷毛力 `-shearK·shearDisp` 与之前的静摩擦力不连续。

方案：静分支每帧回写 `state.shearDispWorld = -(blendedImpulse/dt)/shearK`，并同步 `FtireFiltered`。

待确认风险：回写是否破坏 `maxShearDisp` clamp 或摩擦椭圆缩放（`GetCombinedFrictionScale` 同时缩 force 和 shearDisp）的假设。

### B3. 滚阻的接触判据与接触模型脱节
证据：`RubberTireScript.cs` L253 用 trigger `contacts.Count` 门控 legacy 滚阻，但整个接触模型已改为 raycast（README 明言不再依赖 trigger contact mask）。trigger 不触发时滚阻永远不生效。

方案：缓存上一帧 `topSamples.Count > 0` 作判据。若 A1 不采用 trigger 方案，可一并删除 `contacts` HashSet 与 trigger 回调。

### B4. 发动机转速混入底盘角速度
证据：`Drivetrain.cs` L79 `omegaAxis = Dot(Rigidbody.angularVelocity, driveAxis)` 是绝对角速度。车辆快速 yaw/翻滚时底盘角速度污染 RPM（转速漂移，极端时误触 redline 断油）。

方案：减去相连底盘 body 沿轮轴的角速度分量（相对角速度）。

---

## C. 代码质量 / 性能（物理侧，低风险清理）

- C1 每帧 delegate 分配：`GatherTopContactSamples` 中 `list.Sort((a, b) => b.pen.CompareTo(a.pen))`（L700、L751）每次分配 `Comparison<T>`，每 collider 每物理帧一次。缓存为 `static readonly Comparison<HitSample>`。
- C2 Debug 对象每渲染帧 SetActive 抖动：`SimulateLateUpdateAlways` 每帧调 `EnsureDebugObjects()`（末尾无条件 `HideDebugObjects()`）再 `ShowDebugObjects()`，整棵 hierarchy 每帧 disable/enable。Hide 应只在首次创建时调。
- C3 `BuildFactorySettings()` 每次调用重建 80 项 + ~160 个 delegate（`FactorySettings.cs`）。构建一次缓存到字段，闭包本就长期有效。**注意：这是 UI 热路径的关键依赖**——`RebuildSettings`、`FactoryCommitSettings`、`ApplyFactorySettings` 都会调它（advisor 确认）。
- C4 死代码：`UpdateColliderStatesFromSamples` 里 `gateDn` 未使用（L773），删除。`GatherTopContactSamples` L668 的 `dist <= 1e-6f` 回退**保留**：可达性存在争议（advisor 指出射线起点位于/贴近 collider 表面时 `RaycastHit.distance` 可能为 0；A2 起点后退后该场景更可能出现），防御分支零成本，加注释说明触发条件即可。
- C5 曲线断点逻辑重复：`FactoryEngineRedlineRpm()` 手工复算 `EvaluateEngineTorque` 内部的 `baseRpm/holdRpm/redline` 推导。抽 `GetEngineCurveBreakpoints(out baseRpm, out holdRpm, out redline)` 共用，防止 UI 图表与物理脱节。

---

## D. 可选模型增强（按性价比排序）

- D1 表面材质摩擦：读 `collider.sharedMaterial.dynamicFriction` 缩放 μ。冰面/沙地立刻有区分度，改动集中在 patch 聚合处。
- D2 载荷敏感度：`μ_eff = μ·(Fref/Fn)^s`，一个参数，让重载车转向特性更可信。
- D3 高速防穿隧：28 m/s @ 100 Hz 每步 28 cm，薄地面/坡顶可能整步跨过。沿速度方向加一根长度 `|v|·dt` 的补充射线。

---

## E. UI / 参数工作区（体验重点）

现状：`RubberTireFactoryUI.cs`（650 行）+ `FactorySettings.cs` 绑定层。单面板 880×640，5 个 Tab，左列滚动参数行（InputField/Toggle），右侧自绘曲线图（`RubberTireCurveGraphic : MaskableGraphic`）。

### E1. 参数行升级为 slider + 输入框混合控件
证据：`CreateFloatRow` 只生成 `InputField`（ContentType.DecimalNumber）。`RubberTireFactorySetting.Min/Max` 已存在但只用于 clamp，用户看不到合法范围，也无法"拖着找手感"。

方案：每行 slider（粗调）+ 输入框（精调）。跨多个数量级的参数（`springK` 0–200k、`enginePeakPower` 0–2M、`shearK` 1k–200k）用对数刻度 slider。`RubberTireFactorySetting` 加 `bool LogScale` 元数据。依赖 UIFactory3 是否提供 Slider prefab——需先验证（`Make.Prefab("UIFactory3", "Slider", ...)`）；没有则用 `UnityEngine.UI.Slider` 手搭配色。

### E2. Tab 内分组 + 条件可见性
证据：Engine tab 有 28 行（发动机曲线/8 个齿比/制动/油门整形/滚阻）平铺在 514px 视口里（可见约 13 行）；`gearCount=5` 时仍显示 8 行齿比；`enableGearbox=false` 时齿比行照旧全部显示。

方案：`RubberTireFactorySetting` 加 `Group` 字段，渲染成可折叠小节头（Engine / Gearbox / Brakes / Response / Rolling）；齿比行数跟随 `gearCount`；被上游开关禁用的行灰显（如 combined-slip 关闭时的 grip scale）。相关开关变化时局部刷新可见性，不整页重建。

### E3. 恢复默认 + 批量应用到全车轮子
证据：无任何重置入口——调乱参数只能删块重放。设置是 per-block 机器数据（`rtFactoryConfig`），一台四轮车要逐轮调参。

方案：
- 首次 `BuildFactorySettings` 时快照各字段初始值（即代码默认值）作为 defaults；提供"Reset tab"/"Reset all"按钮。
- "Apply to all RubberTire wheels" 按钮：把当前轮子的序列化配置直接写入机器上所有其他 `RubberTireWheelScript` 的 `factoryConfig.Value`（格式本就为整体字符串，天然支持）。这是四轮车调参体验的最大单点提升。

### E4. Basic / Advanced 分级
证据：80 个参数全量展示，其中 gate 帧数、normal filter alpha、TTL、per-collider TopK 等属求解器策略，普通用户不该碰也看不懂。

方案：`RubberTireFactorySetting` 加 `bool Advanced`，默认隐藏，面板角落一个 "Show advanced" 开关。首次上手的心智负担直接减半。

### E5. 仿真期面板可用性（Live 功能现在基本是死的）
证据：`FindSelectedWheel` 依赖 `BlockMapper.IsOpen`（建造模式）；而 `FactoryCurrentEngineRpm` 在 `!IsSimulating` 时返回 -1，live RPM 线、摩擦椭圆实时工作点只在仿真时有数据。两个条件在原版流程里互斥 → live 图表功能大概率从未真正可见。[INFERENCE] 需真机确认 BlockMapper 能否在仿真中打开。

方案：仿真期提供快捷键切换面板显示，target 取最后选中的轮子（或第一个 RubberTire block）；仿真期把参数行设为只读或允许实时调参（调参走现有 `FactoryCommitSettings` 管线，物理侧 `FactoryPullSettings` 每步已在拉取，天然支持热调）。实时调参 + live 工作点是这个 Mod 作为"参数工作区"的核心卖点，当前等于没交付。

### E6. 图表可读性
证据：`DrawEngine`/`DrawSupport`/`DrawTire` 无任何轴数值标注；torque/power 各自独立归一化（同图不同尺度、无刻度）；发动机曲线的关键转折（baseRpm/powerHoldRpm/redline）无标记；Contact/Visual tab 的 496×514 图表区只放一行说明文字。

方案：
- 轴端数值标签（min/max + 单位），发动机图加 base/hold/redline 竖线标记（复用 C5 抽出的断点函数）。
- 图例用色块而非文字描述颜色。
- 增加小型 live 遥测条：当前挡位（`currentGear` 需暴露只读访问器）、throttle01/brake01 双条、滑移速度、法向载荷。配合 E5 才有意义。
- Contact tab 的空图表区改为 live 接触样本表（per-sample pen/Fn/gate 条形），调接触参数时有直接反馈。

### E7. 疑似 bug：场景切换后 UI 永久消失
证据：`TryBuildUI` 用 `readyRequested` 一次性闩锁；`root` 挂在 `Make.ScreenCanvas` 下。若场景切换销毁 ScreenCanvas（连带 root），`Update` 里 `root == null` 走 `TryBuildUI()`，但 `readyRequested` 已为 true → 直接 return，UI 到进程结束都不再重建。[INFERENCE] 取决于 UIFactory 的 canvas 生命周期，需真机验证：进出关卡/切场景后面板是否还在。

方案：`root == null` 时重置 `readyRequested = false`；且 `TryBuildUI` 先判 `Make.ScreenCanvas != null`——canvas 已存在则直接 `BuildUI()`，未就绪才注册 `Make.OnReady`（`OnReady` 已触发过后重注册可能不再回调，先判 canvas 对两种语义都正确）。

### E8. UI 性能批次（advisor 反馈核实后合并）
1. `chart.SetVerticesDirty()` 每帧调用（`Update` L81）→ 每帧全量重新三角化（96 段曲线 + 双 96 段椭圆 ≈ 数百 quad）并触发 canvas rebuild。改为仅在数据变化时置脏：参数变更（`OnSettingChanged`）、Tab 切换、live 值超过阈值变化（约 10Hz 节流）。
2. `RebuildSettings` 每次 Tab 切换/目标变化 Destroy 全部行再重建（含 `BuildFactorySettings` 的 160 个 delegate）→ 改为按 Tab 分组一次性构建、`SetActive` 切换；配合 C3 缓存设置列表。
3. `FactoryPullSettings` 在 UI `Update` 每帧调用。核实：它有 `String.Equals` 早退，**不做**每帧序列化往返（advisor #1 的机制描述不成立），成本是 ~2KB ordinal 比较；仍应移入 0.2s `RefreshBindings` 节拍，消除无意义比较。
4. `FactoryCommitSettings` 设 `.Value` 触发 `Changed` → `ApplyFactorySettings` 把刚序列化的字符串再全量解析 + 跑 80 个 setter（单次编辑的冗余往返）。修法：commit 期间置 `factoryApplyingConfig` 抑制自触发重放，并直接更新 `factoryLastAppliedConfig`。**不采用**"只提交变更单项"（会破坏单记录版本化格式）。
5. `FormatValue` 显示端精度丢失回写：`magnitude >= 10000` 用 `"0"` 格式取整显示，用户聚焦输入框后回车会把取整值经 `SetFloat` 写回，真值精度经 UI 往返丢失。修法：仅在解析值 ≠ 当前值（epsilon 比较）时才 `SetFloat`；或聚焦时切换为 `"R"` 全精度显示。**不采用** advisor 建议的 `"N1"` 千位分隔格式——`TryParseValue` 用 `NumberStyles.Float`（不含 `AllowThousands`），显示值将无法反解析。
6. `RefreshBindings` 5Hz 无条件 `input.text = FormatValue(...)`：每次分配字符串（~50 字段 × 5Hz）。先比较再赋值。

### E9. 单位与说明
参数标签已含部分单位，但 "Recovery gain"、"Low-speed creep blend"、"Compression stop fraction" 等对用户不可解。`RubberTireFactorySetting` 加 `Tooltip` 字段，行 hover 或选中时在图表区下方显示一行说明。成本低，配合 E4 大幅降低上手门槛。

---

## 落地顺序（v2，UI 提级）

| 批次 | 内容 | 理由 |
|---|---|---|
| 1 ✅ | A5 + A2 + A3 + B3 + C1/C2/C4 | **已实现并编译验证（2026-07-10，Roslyn csc 零错误零警告，dll 已同步至 `RubberTire/`）**；A5 为用户实测发散，最高优先 |
| 2 ✅ | E7 + E8 + C3 | UI 正确性/性能地基 |
| 3 ✅ | E1 + E2 + E3 + E4 | 调参体验核心四件套（slider/分组/重置/分级） |
| 4 ✅ | E5 + E6 + C5 + E9 | 仿真期 live 工作区 + 图表可读性 |
| 5 ✅ | A1（径向扇形接触） | 默认 8 向径向射线 |
| 6 ✅ | B1 + B2 | 前馈 + 刷毛同步 |
| 7 ✅ | A4、B4、D1/D2/D3 | 全部实现（D1/D2 默认中性关闭） |

已知风险汇总：
- A1 改射线布局后 gate fade-in 帧数可能需调小，否则侧壁首次接触软 5 帧。
- A3/B1/B2 涉及能量与稳定性边界，落地前需在真机上做落地/坡起/起步-加速交接三个场景的对比验证。
- E1 依赖 UIFactory3 Slider prefab 存在性，E5/E7 依赖真机验证 BlockMapper 与 canvas 生命周期行为——批次 3/4 开工前先做这三项验证。
- A5 的 `SpinHardCap` 数值与横向阻尼系数需真机标定（每步转角 ≤0.5 rad 只是出发点）；横向阻尼必须基于相对底盘角速度，否则会抵抗转向。

批次 1 实现记录（待真机验证项）：
- A5 取值：`SpinHardCap = 60 rad/s`（internal const），`WobbleDampingFraction = 0.25/step`，反甩摆接合窗口 `[0.5·cap, cap]`（按 |ωrel| 计）。三个数待真机标定。
- A5 扭矩钳位同时覆盖 `ApplyDriveBrake` 与 `driveTorque` 测试钩子；两者独立钳位，理论最坏叠加超出 headroom 一倍，由 PhysX `maxAngularVelocity` 兜底。
- A5 反甩摆经 `GetComponent<Joint>().connectedBody` 取直接关节父级；无 joint 时退化为绝对角速度（仅 |ω|>30 rad/s 才接合，风险可忽略）。
- A2 新副作用：射线起点上移 0.5R 后，位于轮心上方 0.5R 内的自车 collider（紧凑轮罩）可能被采成 pen≈R 的伪接触并按穿透排序占据 Top-N 席位——该几何本身已与轮体相交，属边缘情况，A4（自车过滤）实现后彻底消除。
- A3 数学不变式：`stop ≤ m·|v⁻| ≤ maximumSafeImpulse`（recovery≥0、damping∈[0,1]），直通不会越过能量护栏；仍保留防御性 `Min`。
- 旧存档兼容：`maxW` 存值 >60 的机器加载时被 setter 钳到 60（FactoryFloat 的 Min/Max clamp），行为即新 hard cap，无迁移代码。

批次 2–7 实现记录（2026-07-10，Roslyn csc 全量编译零错误零警告，dll 已同步；游戏内验证待 Windows）：
- E1：slider 为 uGUI 原语自建（track+handle），**不依赖 UIFactory Slider prefab**——该前置验证项因此消除。宽量程参数用三次幂响应（`Curved` 标记）而非对数刻度：Min=0 时对数无定义，三次幂低端细腻且可达 Max。
- 编译坑：游戏全局命名空间有 `Slider` 类型遮蔽 `UnityEngine.UI.Slider`，用 `UISlider` 别名解决。
- E2/E4：`RubberTireFactorySetting` 增加 `Group/Advanced/Curved/VisibleWhen/Tooltip/Default*` 元数据与链式构造（`.In/.Adv/.Curve/.When/.Tip`）；行按组折叠（ASCII "+/-" 头），条件可见（齿比随 gearCount、依赖开关灰隐）；Advanced 默认隐藏，会话级静态记忆。
- E3：默认值在 SafeAwake 首次构建时快照（机器数据应用之前，即代码默认值）；"Apply to All" 以 `transform.root` 判同机，直接复制整条序列化记录。
- E5：F9 切换仿真期面板，目标取首个 `IsSimulating` 的轮子实例（1s 重扫）。仿真期编辑写入 sim 实例字段=实时调参，但不回写建造机器（UI 默认提示中注明）。
- E6：坐标轴数值/断点标记（base/hold 竖线，复用 C5 `GetEngineCurveBreakpoints`）、GEAR/THR/BRK/RPM 遥测条、Contact tab per-sample pen+gate 列表（新增 `FactoryContactSampleCount/FactoryContactSample` 访问器）。
- E9：悬停提示用只实现 `IPointerEnter/Exit` 的自定义组件——完整 `EventTrigger` 会吞掉 ScrollRect 的滚轮/拖拽事件。
- E8：图表只在参数变化或仿真期 10Hz 置脏；行构建一次、按 Tab `SetActive` 复用、换轮子按索引重绑 delegate；commit 合批到 0.2s 节拍并在目标切换时强制冲刷；输入框回车只在解析值与真值差异超 epsilon 时写回。
- B1：前馈量 = 本步自施加轮轴扭矩（drive+brake+测试钩子，`pendingDriveAxisTorque`）经逆惯量在接触点的滑移增量投影；扰动估计器同步减去上一步前馈（`lastFeedForward*`），避免双重计入。
- B2：静摩擦分支每帧回写 `shearDispWorld = -F_applied/shearK`（含 maxShearDisp 钳制）并同步 `FtireFiltered`。
- D1/D2：经 per-patch 环境量 `activeMuScale`（`MuStaticEff/MuKineticEff`）进入全部限幅路径；D1 默认关（0.6 中性参考待 Windows 标定），D2 默认 s=0 中性。
- A1：默认 `radialRayCount=8`（45° 间隔），=1 即旧行为；聚合穿透在多射线时改用加权平均。D3 速度射线在 |v|·dt>0.25R 时追加。
- B4/A5.3/A4 共用 `GetComponent<Joint>().connectedBody` 父级查询（每次仿真缓存一次）。
- 旧存档兼容：新增键 rayR/surfMu/muLS/muLRef 缺失时保持默认值，无迁移代码。

---

## Advisor 反馈处理记录

### 第 1 轮（nit）
- C4 `dist <= 1e-6f` "不可达"判断被质疑 → **采纳**：保留防御分支 + 注释，C4 缩减为仅删 `gateDn`。

### 第 2 轮（concern，UI 五项）
- #1 `FactoryPullSettings` 每帧序列化往返 → **机制描述不成立**（有 `String.Equals` 早退，实际是每帧 2KB 字符串比较），但移入 0.2s 节拍的建议采纳（E8.3）。
- #2 `SetVerticesDirty` 每帧 → **采纳**（E8.1）。
- #3 `RebuildSettings` Destroy churn → **采纳**（E8.2）。
- #4 单字段编辑触发全量序列化 → **现象属实、修法修正**：不改单项提交格式，改为抑制 commit 的自触发重放（E8.4）。
- #5 `FormatValue` 用 `"N1"` → **建议有害，已修正**：千位分隔符会导致反解析失败；真实问题是取整显示值经回车写回（E8.5）。
- C3 是 UI 热路径关键依赖 → **采纳**，已在 C3 标注。

### 第 3/5 轮（concern，E7 修复方案）
- `Make.OnReady` 已触发过则重注册可能不回调 → **采纳**：`TryBuildUI` 先判 `Make.ScreenCanvas != null` 直接 `BuildUI()`，未就绪才注册 `OnReady`；配合 `root == null` 时重置 `readyRequested`。

### 第 4 轮（nit，A5 机制补充）
- 垂直轴分量主动阻尼、载荷下生效 → **机制采纳、公式修正**：advisor 的 `ωperp = ω - (ω·axis)·axis` 用的是绝对角速度，会阻尼整车转向/俯仰；必须改用相对底盘的 `ωrel`（A5.3）。
- `AddTorque` 前按 `I·(cap-|ω|)/dt` 钳扭矩 → **采纳**，限定为只钳与 ω 同号（泵能）方向，制动不受限（A5.2）。
