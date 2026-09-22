# Epic：悬架模板化架构（模板 → 子系统 → 试验台总成）

- 任务编号：20260922-suspension-template-architecture
- 创建日期：2026-09-22
- 形态：epic
- 状态：**规划中**（12 个子任务全部 TODO，未开始实施）
- 真源：本目录 SUBTASKS.csv；子任务相对路径均相对此 Epic 目录解析。

## 状态与原始需求

本轮只交付规划，**不实施代码**。用户原话（按轮次，逐条为需求原文）：

1. 「我想把 K/C 模式给通用化，任何的仿真都可以由用户选择 K 或 C 模式」
2. 「在模型装配时声明是 K 模式还是 C 模式，如果用 K 模式则使用 joint，如果是 C 模式则使用 bushing；各个悬架类型的 K、C 模式的运动副、衬套的定义都是预先设定好的，比如下摆臂在 K 模式三个连接点都是运动副，在 C 模式则只有前后点是衬套。模型装配好后也可以任意切换 K、C 模式。」
3. 「我设想的对于任意仿真应该是不同总成+试验台的组合，比如 kc_quasi_static 是单轴总成+悬架 KC 试验台、vehicle_kc 是整车总成（整车总成又相当于两个单轴总成+其他子总成）+悬架 KC 试验台、axle_dynamic 是单轴总成+悬架 KC 试验台（和 kc_quasi_static 的区别应该只有求解器、轮胎的区别）；重点关注 kc_quasi_static 和 axle_dynamic，这两个应该只有求解器和轮胎区别，所以也应该尽可能通用化。」
4. 「不同类型的总成运动副类型都要统一，所以运动副类型都可以用在任意总成，也就是说所有总成公用一个底层」
5. 「对于 axle_dynamic 与 kc_quasi_static，这两个完全可以合并为一个仿真，可以选择准静态和动态模式」
6. 「可以参考 adams car 的模板概念，由专家定义多套模板（使用者无需关注），模板里在同一个点可以建立运动副和衬套，在 K 模式只激活运动副、C 模式只激活衬套，这样的话切换 K、C 模式会很方便，衬套、弹簧、减振器等弹性元件由外部属性文件控制属性，也就是说可以加载不同属性文件达到不同刚度、阻尼等等特性」
7. 「零刚度占位衬套可以改为使用模板定义好的默认属性代替」
8. 「所有仿真的输出由总成+试验台定义，总成带有它所定义的输出，试验台也带有它独有的输出，这些输出是最小单位输出，后续的衍生结果都是使用这些输出做运算得到的（比如 KC 指标、操稳指标等等），可以引入类似 adams car 的 request 功能，即可以通过这些基本输出自定义任何输出」
9. 「准静态 接受 fiala/pac2002/native_brush 只是使用其垂直刚度和轮胎尺寸、质量，相当于模型退化」
10. 「轮胎质量归属到轮胎」
11. 「彻底改三层」
12. 「可以重录基线」
13. 「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到悬架实验总成，前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成」
14. 「先制定计划，不具体实现代码」（本轮交付边界）

### 用户裁决登记（本轮四项）

| 编号 | 问题 | 用户裁决 | 影响 |
|---|---|---|---|
| D1 | 准静态轮胎力律如何落地 | **不新增内核力律**：准静态接受 `fiala`/`pac2002`/`native_brush`，只使用其垂向刚度与轮胎尺寸、质量，相当于**模型退化** | 04/09 子任务；内核只加质量字段，不加力律 |
| D2 | 轮胎质量归属 | **归属轮胎**（选择"乙"：tire 成为独立惯量来源，求解器显式耦合） | 08 子任务；动 C++ 内核与契约文档 |
| D3 | 架构层次 | **彻底改三层**：模板 → 子系统 → 仿真总成 | 全 Epic |
| D4 | 基线重录 | **可以重录基线** | 各子任务在 PROGRESS 中逐项登记重录范围与前后值 |

## Goal

**G1（副底座统一）**：所有总成共用同一份运动副定义与编码。作者层不再存在「某总成只支持 N 种副」的截断；任意总成可用任意副；副表与内核 `contract_registry.cpp` 的行数表由测试强制一致。

**G2（三层架构）**：建立「模板（专家定义）→ 子系统（模板实例化）→ 仿真总成（子系统 + 试验台）」三层，替代现有 `(assembly, family)` 两层的枚举结构。模板声明 parts / joints / bushings / springs / dampers / 属性槽 / 输出；子系统粒度为用户裁定的四类——**左右悬架、转向、轮胎、车身**。两种组装都必须成立：**悬架实验总成**（左右悬架+转向+轮胎+悬架试验台，单轴，04/05/10 承接）与**整车实验总成**（前后悬架+转向+车身+轮胎+整车 KC 试验台，11 承接）。

**G3（K/C 由模板列激活）**：模板中每个连接点可**同时声明运动副列与衬套列**；模式是**实例化时的选择**，装配后可在两模式间任意切换，且切换不改变几何、体身份、连接点位置。激活规则（**已按实测修正，不得按字面简化**）：某一列存在才激活该列，另一列不被激活；**只有 joint 列、没有 bushing 列的点（如臂外点、拉杆两端、齿条导轨）在两个模式下都保留其 joint**——实测 C 模式下这 9 个 joint 仍然存在（`benchmark_axle.py` 实测 C `constraints=9`：外点 4 + 拉杆 4 + rack 导轨 1），C 模式并非「只保留衬套」。C 模式激活的衬套刚度来自模板默认属性或属性文件，**不再使用零刚度占位**。

> 说明：用户原话「在 K 模式只激活运动副、C 模式只激活衬套」需要按此精确理解——它描述的是**同一连接点上有两列时的取舍**，不是「C 模式丢弃所有 joint」。计划首版曾按字面写「C 模式只激活衬套列、未激活列不产出任何对象」，会丢掉 C 模式下必须保留的 9 个 joint，与实测及 04 冻结的 C 基线冲突（审核 `cac8e120` 的 B2）。

**G4（属性文件）**：衬套、弹簧、减振器等弹性元件的属性由外部属性文件提供；同一模板加载不同属性文件得到不同刚度/阻尼特性，模板本身不变。

**G5（准静态与动态合并）**：`kc_quasi_static` 与 `axle_dynamic` 合并为**同一个仿真**，由 study 选择准静态或动态。准静态 study 使用与动态相同的轮胎力律选择（`fiala`/`pac2002`/`native_brush`），但只让垂向刚度与轮胎尺寸、质量生效（模型退化）。轮胎质量归属轮胎本身。

**G6（输出声明与 request）**：总成与试验台各自声明自己的**最小单位输出**；所有衍生结果（KC 指标、操稳指标等）由最小单位输出运算得到。提供类似 Adams Car Request 的自定义输出机制。现有 `report/` 指标重述为内置衍生输出，既有结果不变。

**G7（总成 × 试验台正交组合）**：总成与试验台独立注册，运行 = 选一个总成 + 选一个试验台。现有 7 个组合的默认行为保持可用；新增组合（如单轴 + 四立柱）无需新写驱动坐标与结果排布。

**G8（基线重录受控）**：D4 授权重录基线，但**逐阶段、逐项**重录并登记——记明哪个基线、因哪一步、重录前后的值。禁止一次性全量重录掩盖回归。

### Non-Goals

- 不新增悬架类型的硬点集。麦弗逊/多连杆模板只留**注册接口与结构**，不替用户编硬点数据。
- 不改积分器、K/C 稳态算法、步长控制、牛顿/线搜索的算法本身（G5 与 D2 引入的轮胎惯量耦合除外）。
- 不改轮胎力律的物理定义（只改质量归属与"准静态只取垂向"的激活范围）。
- 不改 ABI 七符号与版本常量（`suspension_kernel_{run,capabilities,contract_version}` + `axle/vehicle/mb_core_abi_version` + `mb_core_run`）；不新增导出符号。
- 不改变现有公开 API 的调用方式（`run_case`/`run_dynamic_case`/`run_vehicle_dynamics` 保持可用）。
- 不扩大 Adams 等价范围，不把未实现的整车 Adams 对标标为通过。
- 不新增图表/UI/第三方依赖。

## 事实与修正（制定计划前实测）

以下为制定本计划时实测确认的事实，实施时应复核：

- **内核副注册表已有 10 项**（`cpp/src/contract/contract_registry.cpp:23-30`）：`spherical`3 / `revolute`5 / `fixed`6 / `prismatic`5 / `universal`4 / `cylindrical`4 / `inplane`1 / `convel`4 / `driven_translation`1 / `driven_rotation`1。**内核侧无需改动即可支持任意总成使用任意副**。
- **截断点在 kc 作者层**：`cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 只映射 `BallJoint`→`spherical`、`RevoluteJoint`→`revolute`、`PrismaticJoint`→`prismatic`，其余抛 `NativeKcError`。而装配层 `preparation/assembly/types.py` 已有 8 种副类、schema `IdealJointSpec`（`schema/model.py:100-109`）也允许 8 种。**统一副底座的实质是拆掉这处截断，不是新建底座。**
- **`constant_velocity → convel` 改名在两处各写一遍**：`cases/axle_dynamic.py:292`、`cases/vehicle_dynamic.py:79`，应收口到一处。
- **生效的 native 边界是文档契约，不是 struct ABI**：`kernel/__init__.py:186-205` 的 `run_contract` 传两份 JSON payload 给 `suspension_kernel_run`（`argtypes` 全是 `c_uint8`/`c_size_t` 指针，`kernel/__init__.py:125-135`）。因此 D2「轮胎质量进内核」主要落在**契约文档字段 + 内核解析 + 求解器惯量耦合**，不必然破 struct 布局；但 `AxleInput`/`VehicleInput` 仍是内核内部结构（`kernel_contract_run.cpp:313-380` 填充），轮胎数组若加质量字段仍需同步。
- **`vertical_linear` 是死名字**：它在注册表 `kTires`（`contract_registry.cpp:36`）里，但内核实际解析表只有 `{"native_brush",0},{"pac2002",1},{"fiala",3}`（`cpp/src/cases/contract_model.cpp:948-950`），枚举 `VehicleTireModelKind`（`cpp/include/mb_model/enums.hpp:61-70`）也只有这 4 项。**D1 裁决正确绕开了这一点**：不新增力律。
- **垂向退化在内核已有底座**：`cpp/src/tire/fiala/forces.cpp:20-29` 的 `fiala_elastic_force` 在无 `deflection_curve` 时就是 `tire.k * penetration`。准静态"只取垂向"可通过作者层参数选择实现，无需改力律。
- **内核 `Tire` 结构体当前无质量字段**（`cpp/include/mb_model/types.hpp:101-146`：radius/k/c/mu_*/brush_k_*/relaxation_*/model_kind/state_slot_width/contact_mass/maxwell_enabled）。质量现挂在 body 上：`WheelSpec.mass`（`schema/vehicle.py:60`，`axial_inertia` 在 `:61`）在 `preparation/assembly/vehicle.py:653,722` 合并进轮端刚体。**D2 是真正的数据结构改动。**
- **契约 schema 对 tire 是闭合的**：`packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json` 的 tire 定义 `additionalProperties: false`，`required: [name, model, body]`，`properties: [blob, body, model, name, parameters]`。因此 08 新增轮胎质量字段**必须同时改契约 schema**，否则文档校验先失败。
- **08 的 Python 侧落点跨越子任务写范围**：轮胎质量字段的发射点在 `cases/vehicle_dynamic.py:327`（`_tire_entry` 附近）与 `cases/axle_dynamic.py:342`，数据源在 `preparation/vehicle_dynamic.py` 的轮胎 spec 与 `schema/vehicle.py:60`。这些属 02 与 04-06 的写范围，与 08 并行会冲突。规划口径：**08 只改 C++ 内核 + 契约 schema + 契约文档读取 + native 镜像**，用文档级 fixture 证明字段被接受/解析与质量守恒；Python 侧发射落点在 02/04-06 完成后串行补齐（或并入 09）。
- **kc 模型文档当前完全不发轮胎**：实测 `"tires" in doc` 为 `False`（`cases/kc_quasi_static/contract.py:133-150` 的顶层键无 `tires`）。kc 侧虽有 `VerticalTireElement`（`preparation/assembly/front_axle.py:561-574`），但不进文档。
- **C 模式零刚度占位衬套实测**：`benchmark_axle.json`（symmetric_proxy）下 K 模式 `constraints=13 / bushings=0 / 文档 elements=0`；C 模式 `constraints=9 / ideal_constraints=17 / bushings=8 / 文档 elements=8`，8 条衬套刚度范数**全为 0.0**（`front_axle.py:737`、`:802` 的 `stiffness=np.zeros((6,6))`）。
- **KC 与 axle_dynamic 的差异不止"求解器与轮胎"**（实测七项）：结果对象不同（kc 走 `StateResult`/`ResultBundle` 且元件力在 Python 侧重算 `api.py:751-791`；axle_dynamic 走 `AxleDynamicsResult` + 内核 7 类输出块）、模型 schema 不同、轮胎有无不同、K/C 有无不同、case 结构不同、时间网格语义不同。**唯一真正共享的是 `AxleSolverSettings` + `kernel/solver.py` 序列化器、driven coordinate 机制、同一 dispatcher 与求解器。**
- **现有子系统边界粒度**：`build_front_axle`（`front_axle.py:625`）一个函数里同时产出左右悬架（body/point/constraint）、转向（rack/tie rod，`:838-903`）、轮胎（`:561-574`）、以及 C 模式的占位衬套。用户裁定的四类子系统需从此函数拆出。
- **`symmetric_proxy` 与 `explicit` 两条拓扑**：`schema/model.py:173` 的 `topology`；`symmetric_proxy` 不允许 `joints`（`:193-196`），副由硬点自动生成；`explicit` 走 `_build_explicit_axle`（`:387`）由用户声明副，其 `_explicit_constraint`（`:271-315`）**已支持全部 8 种副**。
- **基线文件位置**：`packages/suspension_multibody/tests/data/` 下的 `kc_baseline/`、`kc_perf_baseline.json`、`kc_perf_baseline_native.json`、`dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/`。C++ 侧 `packages/suspension_kernel/layering_baseline.json`（145 边 / 118 头文件边）。
- **`report/` 现有指标规模**：`report/metrics/{axle,case_specific,common,vehicle}.py` 共 27 个函数定义，是 G6「重述为内置衍生输出」的迁移对象。

## 三层目标结构

```
模板 Template（专家定义，使用者无需关注）
  ├── parts：有哪些部件
  ├── connections：每个连接点同时声明 joint 列与 bushing 列
  ├── 弹性元件槽位：spring / damper / bushing 的引用
  ├── property_slots：属性文件需要提供什么
  └── outputs：模板自带的输出声明
        ↓ 实例化（填属性文件 + 选 K/C）
子系统 Subsystem（用户裁定的四类）
  ├── 左右悬架（suspension）
  ├── 转向（steering）
  ├── 轮胎（tire）
  └── 车身（chassis）
        ↓ 与试验台组装
仿真总成 SimulationAssembly
  ├── 悬架实验总成 = 左右悬架 + 转向 + 轮胎 + 悬架试验台（单轴）
  └── 整车实验总成 = 前后悬架 + 转向 + 车身 + 轮胎 + 整车KC试验台（整车）
        （整车总成 ≈ 两个单轴悬架子系统 + 车身子系统，见需求第 3 条）
        ↓ 运行（study：准静态 / 动态）
结果 → 最小单位输出（总成输出 ∪ 试验台输出）→ 衍生输出（request）
```

## 子任务分解与依赖

见 `SUBTASKS.csv`。串行主线与并行支线：

```
01 基线冻结
  ↓
02 统一副底座            ← 纯重构，零基线风险
  ↓
03 模板与子系统数据模型
  ├→ 04 子系统拆分（悬架/转向/轮胎/车身）
  │      ↓
  │   05 模板实例化与 K/C 列激活
  │      ↓
  │   06 属性文件机制
  │      ↓
  │   09 准静态/动态 study 合并
  │      ↓
  │   10 试验台抽取与正交组合
  └→ 07 输出声明与衍生输出（可与 04-06 并行）
01 → 08 轮胎质量归属与内核耦合（与 03-07 并行，写范围不相交）
                              ↓
                           11 整车实验总成组装 + 整车侧质量落点
                              ↓
07;10;11 ↓
12 终局验收
```

**并行约束**：03 与 04 都触及 `preparation/assembly/types.py`，不得同时进行。08 只写 C++ 内核与 `native/` 镜像，与 03-07 的 Python 写范围不相交。

## 冻结约束

- **「不得重录基线」已由 D4 解除**，但改为**逐阶段受控重录**：每次重录必须在子任务 PROGRESS 中登记「基线文件 + 导致重录的步骤 + 重录前后的值 + 判定为等价/非等价的依据」。禁止先改基线让门变绿。
- **ABI 七符号与版本常量不变**（除非 08 证实必须追加结构字段，且必须 append-only 并在 PROGRESS 中单独裁决）。
- **固定编译器、浮点选项、线程、后端与计算顺序**；纯结构阶段要求现有动态数组逐位一致。
- **现有 7 个 (总成, 试验台) 组合的默认行为保持可用**；新增能力通过显式选择新模板/新模式/新组合生效。
- **旧内部 import 路径属删除范围**，但**包级公开 API 与历史结果读取保持**。
- C++ 模块分层 DAG 与 `check_module_layering.py --strict --final` 保持绿；新增 C++ 代码必须落进现有 23 模块，不得新增反向边。

## 验证协议

**01 实测现状**：运行现有全量测试、动态哈希、K/C parity、family parity、ABI 符号与版本门，记录失败/skip 原因与当前基线值。中间证据置会话 scratch；冻结夹具与清单是交付物。

**02**：副表与内核 `contract_registry.cpp` 行数由测试强制一致（新增断言）；副表覆盖 8 种真实副 + 2 种 driven；拆截断后现有 3 种副的文档编码**逐字节不变**；构造 5 个负例（未知副名、缺轴、轴退化、行数不符、driven 混入副表）均须失败。

**03**：模板数据结构单元测试；`ConnectionDefinition` 必须同时可带 joint 列与 bushing 列；属性槽声明与输出声明可序列化/反序列化；缺列、双空列、重复 role 均须报错。

**04**：四类子系统可独立实例化；与现役 `build_front_axle` 的产物**逐项对照**（body 集合、点数、约束集合、连接表），差异必须为零或逐项登记理由。

**05**：同一模板实例化 K 与 C，几何/体身份/连接点/驱动坐标**完全一致**，仅 joint 与 bushing 列不同；`with_mode` 幂等；`with_mode("C")` 结果与直接实例化 C **逐位一致**（这是防"第二套实现"的关键断言）。

**06**：同一模板 + 同一属性文件 → 逐位一致；换属性文件 → 刚度/阻尼按属性变化而模板不变；属性缺失、类型错误、越界值均须报错。

**07**：输出声明解析；总成输出 ∪ 试验台输出的合并与冲突检测；衍生输出只用最小单位输出求值（不得旁路读内核）；现有 `report/` 27 个指标函数重述后结果与旧实现**逐值一致**。

**08**：契约文档新增轮胎质量字段；内核 `Tire` 加 mass/inertia；求解器惯量耦合；**质量守恒类断言**（轮胎质量从 body 迁到 tire 后，整车/整轴总质量与世界质心不变）；ABI 七符号与版本门保持；两包构建与隔离 wheel 复验。

**09**：同一模型装配 + `study="quasi_static"` 与 `study="dynamic"` 两种运行；准静态下只取垂向刚度与尺寸/质量的断言（对照动态结果的轮胎侧向/纵向输出为零或未激活）；准静态与动态共享同一装配入口的证据。

**10**：总成 × 试验台矩阵可查询；现有 7 个组合行为不变；构造无效组合（试验台要求总成不提供的坐标）须在准备阶段报错并点名；新增至少一个此前不存在的组合（单轴 + 四立柱或单轴 + 随机路面）并跑通。

**11 独立终局命令**（逐条记录退出码）：

```text
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --all-packages ruff check .
uv run --all-packages ty check .
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

## Done-When

独立逐条确认 G1–G8：

1. **G1**：副表与内核行数表由测试强制一致；任意总成可声明任意副（有测试证明 kc 能发出 `universal`/`cylindrical`/`convel`/`inplane`/`fixed`，且内核接受）；作者层无残留的 N 种截断。
2. **G2**：三层结构存在且可运行——`templates/`、`subsystems/`、试验台三层各自有注册表与实例化路径；四类子系统（悬架/转向/轮胎/车身）可独立实例化；**两种组装都成立**：悬架实验总成（单轴）与整车实验总成（前后悬架+转向+车身+轮胎+整车 KC 试验台）。
3. **G3**：模板中一个连接点同时带 joint 列与 bushing 列；K/C 为实例化选择；`with_mode` 切换后几何/体身份/连接点/驱动坐标不变、仅激活列变；`with_mode("C")` 与直接实例化 C 逐位一致；C 模式衬套刚度**不再全为零**（有测试断言非零且来自模板/属性文件）。
4. **G4**：属性文件可加载；同模板 + 不同属性文件得到不同刚度/阻尼而模板不变；模板可直接给数值（现役行为）也可给引用。
5. **G5**：准静态与动态是同一仿真的两个 study；同一装配入口；准静态只激活垂向刚度与尺寸/质量，力律选择（`fiala`/`pac2002`/`native_brush`）对两者一致可用。
6. **G6**：总成输出与试验台输出各自声明并有合并规则；衍生输出只用最小单位输出求值；现有 27 个 `report/` 指标重述后结果逐值一致；可自定义新输出而无需改包代码。
7. **G7**：总成 × 试验台矩阵可查询；现有 7 个组合行为保持；无效组合在准备阶段报错点名；至少一个新组合跑通。
8. **轮胎质量归属（需求 10）**：有**独立于 08 自证**的验证——轴侧与整车侧都把质量写进 tire entry，且整车/整轴总质量与世界质心不变（不采信 08 自身的守恒断言作为唯一证据）。
9. **G8**：每次基线重录都有登记（文件 + 步骤 + 前后值 + 等价性判定）；无"先改基线让门变绿"的痕迹；`ABI 七符号与版本（15/30/1/1）`未变或变更有独立裁决记录。

**端到端独立验收（不依赖子任务自证）**：用**同一个模板**完成下列四件事，全部通过才算 Goal 达成——

```text
(a) 同一个轴的模板，K 模式跑一次、C 模式跑一次 → 只差激活列
(b) 同一个轴模板 + 同一属性文件，准静态 study 跑一次、动态 study 跑一次
(c) 换一份属性文件重跑 (b)，刚度/阻尼按属性变化而模板与几何不变
(d) 总成 × 试验台矩阵任取两个组合跑通，其中一个此前不存在；
    并用最小单位输出 + 自定义表达式算出一个新指标
(e) 整车实验总成（前后悬架+转向+车身+轮胎+整车 KC 试验台）跑通一次，
    并证明它与悬架实验总成共享同一套子系统定义
(f) 轮胎质量归属：轴侧与整车侧各验证一次质量来自 tire 而非轮端 body
```

**数值门为独立项**：`dynamic_hash_sentinel`、`kc_parity_check`、`case_parity_check` 三门在本 Epic 期间的每次基线重录后都必须重新通过，且重录本身有登记。每行 DONE 不代替这些条件。

## 风险与回退

- **三层重构的风险**：模板/子系统/试验台三层会重写 `preparation/` 与 `simulation/` 的组织方式。缓解：03-05 分步落地，每步保持现有入口可用；旧路径以适配器方式保留到 10 之后再决定删除。
- **轮胎质量改动（D2=乙）的风险**：求解器显式耦合轮胎惯量，可能改变数值结果并触及动态字节门。缓解：先做质量守恒类断言（总质量与质心不变），再评估数值差异；差异必须登记为"物理改变或数值路径改变"，不得默默吸收。
- **准静态发轮胎的风险**：kc 从"无轮胎"变为"有轮胎（垂向激活）"，`kc_baseline/` 必然变。缓解：先建立"垂向激活等价于原 VerticalTireElement"的对照，再重录。
- **旧路径删除的风险**：不可逆。缓解：删除集中在最后阶段，删除前完成调用方迁移与前置门禁；删除范围只覆盖**已无生产调用者**的部分。
- 真实 Adams 执行需现有安装/许可；默认使用已冻结证据与渲染测试，缺少真实执行明确记录，**不能声称整车数值等价**。
- 最终仍有不相关既有失败则独立列明；任何新增失败阻断完成。

## 目录与命名

按 taskmaster v5 的 Epic + Full Single 结构组织；日期前缀遵循技能示例及仓库既有命名惯例。

```text
.codex-tasks/20260922-suspension-template-architecture/
├── EPIC.md
├── SUBTASKS.csv
├── PROGRESS.md
└── tasks/
    ├── 20260922-01-baseline/
    ├── 20260922-02-joint-foundation/
    ├── 20260922-03-template-model/
    ├── 20260922-04-subsystems/
    ├── 20260922-05-instantiation/
    ├── 20260922-06-properties/
    ├── 20260922-07-outputs/
    ├── 20260922-08-tire-mass/
    ├── 20260922-09-study-merge/
    ├── 20260922-10-rigs/
    ├── 20260922-11-vehicle-assembly/
    └── 20260922-12-acceptance/
```

每个子目录均含 SPEC.md、TODO.csv、PROGRESS.md、raw/。临时脚本与中间日志写会话 scratch；raw/ 为可归档交付证据预留，不存虚构或未执行结果。父 SUBTASKS.csv 管子任务状态，子 TODO.csv 管具体步骤，禁止相互替代。本轮规划不将任何实施任务置为 DONE。
