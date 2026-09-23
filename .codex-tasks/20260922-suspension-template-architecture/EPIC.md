# Epic：悬架模板化架构（模板 → 子系统 → 试验台总成）

- 任务编号：20260922-suspension-template-architecture
- 创建日期：2026-09-22
- 形态：epic
- 状态：**实施中**（01/02/03 DONE；04 已开工——`subsystems/` 已落 3 个文件与现役产物快照，13 步仍 TODO；05-12 未开工）
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
15. 「针对G2：悬架实验总成可以没有转向系统，试验台要更具所选子系统自动适配接口」（第二轮补强；本轮仍只规划，不实施代码）
16. 「我想把制动系统和驱动系统以及轮胎也分别实现为子系统，当前已经实现的简单驱动和制动系统，在子系统里可以作为主动力矩作用在车轮子系统上」（第三轮；本轮仍只规划，不实施代码）
17. 「当前的简单制动/驱动子系统不需要有刚体，具体参考adams car的实现；悬架实验总成不需要制动、驱动子系统，整车总成必须要」
18. 「只做「分配声明 + 轮端力矩」；补卡钳几何参数（brake_mu/piston_area/effective_piston_radius）以便与 Adams 简单版逐参数对标」
19. 「在单轴侧，车轮作为悬架试验台的一部分，同样参考adams car」
20. 「当前制动和驱动子系统可以做成简化实现，但是一定要有能力在后续的模板中可以扩展成带刚体的复杂形式」（本轮核心架构约束）

### 用户裁决登记（D1-D4，首轮；D5-D7 见下方第二轮）

| 编号 | 问题 | 用户裁决 | 影响 |
|---|---|---|---|
| D1 | 准静态轮胎力律如何落地 | **不新增内核力律**：准静态接受 `fiala`/`pac2002`/`native_brush`，只使用其垂向刚度与轮胎尺寸、质量，相当于**模型退化** | 04/09 子任务；内核只加质量字段，不加力律 |
| D2 | 轮胎质量归属 | **归属轮胎**（选择"乙"：tire 成为独立惯量来源，求解器显式耦合） | 08 子任务；动 C++ 内核与契约文档 |
| D3 | 架构层次 | **彻底改三层**：模板 → 子系统 → 仿真总成 | 全 Epic |
| D4 | 基线重录 | **可以重录基线** | 各子任务在 PROGRESS 中逐项登记重录范围与前后值 |
| D5 | 可选转向的范围（需求 15） | **只放开单轴悬架实验总成**：转向在单轴侧成为可缺席子系统；整车侧 `SteeringSystemSpec`（`schema/vehicle.py:233`）与 `required_bodies`（`:249-259`）维持必填，`_build_steering`（`preparation/vehicle_dynamic.py:227`）不改，5 个 vehicle family 行为不变 | 04（单轴侧新增路径）、11（登记不对称） |
| D6 | 试验台接口的自适应方式 | **自适应收缩**：总成不提供转向时，rack 驱动轴与 rack 相关输出整体消失，K 网格降维为纯轮跳，试验台仍可跑（结果对象形状随之变化） | 10 |
| D7 | 需求 15 的承接落点 | **并入现有子任务，不新增编号**：04 加「转向子系统可缺席」、10 加「试验台按总成能力自适应」、11 登记整车侧不放开、12 增加无转向端到端判据 | 04/10/11/12 |
| D8 | 制动/驱动子系统的承接与粒度 | **并入现有子任务**（04 定义、10 接试验台输入、11 整车组装），不新增编号。粒度 = **0 刚体**，只做「分配声明 + 轮端力矩」；可用性：**悬架实验总成不含制动/驱动，整车实验总成必须含** | 04/10/11 |
| D9 | 单轴侧车轮归属 | **单轴侧车轮归悬架试验台**（对标 Adams `__MDI_SUSPENSION_TESTRIG` 的 `testrig_tire_property_file='RIGID_WHEEL'`/`testrig_wheel_radius`/`tire_stiffness`）；单轴总成**不产出** `wheel.body`，整车侧维持 wheel 子系统。故 04 不产出单轴轮体，10 承接试验台侧车轮 | 04/10/11 |
| D10 | 制动参数对标范围 | **只取力矩相关子集**：`brake_mu` / `piston_area` / `effective_piston_radius` / `front_brake_bias` / `max_brake_value`；转子几何（`rotor_hub_wheel_offset`/`rotor_hub_width`/`rotor_width`/`brake_reduction`）属详细版，不进简化版 | 04 |
| D11 | 简化版的可扩展性（**本轮核心**） | 简化制动/驱动必须是**可替换的提供者**，不是一次性写死：同一子系统 role 下**模板可替换**——简化模板只发轮端力矩（0 刚体），后续复杂模板可产出刚体（卡钳/转子/动力总成体/差速器）。对标 Adams 实证：`_brake_system_4Wdisk.tpl`（0 体）与 `_brake_system_4Wdisk_calipers.tpl`（4 体）**同属 `MAJOR_ROLE='brake_system'`**；`_powertrain.tpl`/`_driveline_fwd.tpl`/`_driveline_rwd.tpl` 同属 powertrain role | 03/04/10/11 |

## Goal

**G1（副底座统一）**：所有总成共用同一份运动副定义与编码。作者层不再存在「某总成只支持 N 种副」的截断；任意总成可用任意副；副表与内核 `contract_registry.cpp` 的行数表由测试强制一致。

**G2（三层架构）**：建立「模板（专家定义）→ 子系统（模板实例化）→ 仿真总成（子系统 + 试验台）」三层，替代现有 `(assembly, family)` 两层的枚举结构。模板声明 parts / joints / bushings / springs / dampers / 属性槽 / 输出；子系统粒度为用户裁定的**六类**——**左右悬架、转向、车轮、车身、制动、驱动**（原四类「左右悬架/转向/轮胎/车身」按需求 16 扩为六类，「轮胎」更名为「车轮」以涵盖轮体与轮胎，并新增制动、驱动）。**各子系统的可用性矩阵（按需求 17，对标 Adams 装配实证）**：

| 子系统 | 悬架实验总成（单轴） | 整车实验总成 |
|---|---|---|
| 左右悬架 | ✅ | ✅（前+后） |
| 转向 | ✅ **可缺席**（需求 15/D5） | ✅ 必填 |
| 车轮 | **由试验台提供**（D9，对标 Adams 单轴装配不含 wheel 子系统） | ✅ wheel 子系统 |
| 车身 | ✅（固定 chassis） | ✅（真车身） |
| 制动 | ❌ **不需要**（需求 17） | ✅ **必须要** |
| 驱动 | ❌ **不需要**（需求 17） | ✅ **必须要** |

两种组装都必须成立：**悬架实验总成**（左右悬架+转向+车轮（试验台侧）+悬架试验台，单轴，04/05/10 承接）与**整车实验总成**（前后悬架+转向+车身+车轮+制动+驱动+整车 KC 试验台，11 承接）。**子系统在总成内是可选的（需求 15，裁决 D5-D7）**：**悬架实验总成可以没有转向系统**——转向缺席时 rack 体、rack 中心点、rack 导轨与两侧拉杆及其 4 个球铰整体不产出，其余子系统与几何不变；**试验台按所选子系统自动适配接口**——总成不提供的坐标，试验台不得硬要求，而应收缩其驱动轴与输出，不得让运行失败、也不得把不存在的输出填零。整车实验总成侧**不放开转向**（D5）：`schema/vehicle.py` 的 `SteeringSystemSpec` 与 `required_bodies` 维持必填，5 个 vehicle family 行为不变。

**G2b（简化制动/驱动的可扩展性，需求 20 / D11）**：制动与驱动子系统的**简化实现必须能无痛升级为带刚体的复杂实现**。落地方式 = **同一子系统 role、可替换的模板**（对标 Adams：同 `MAJOR_ROLE='brake_system'` 下 `_brake_system_4Wdisk.tpl` 0 体、`_brake_system_4Wdisk_calipers.tpl` 4 体）：简化模板只声明分配参数并发轮端力矩（0 刚体）；复杂模板可产出卡钳/转子/动力总成体/差速器刚体与相应约束。**要求**：(1) 子系统 role 与对外接口（轮端力矩通道、参数槽、输出声明）在两种模板下**同一套**，装配层与试验台**不得因模板是简化版还是复杂版而分支**；(2) 复杂模板**只需新增模板 + 注册**，不得改动试验台、装配层或内核；(3) 有测试证明"用复杂模板替换简化模板不需要改试验台与装配代码"。

**G3（K/C 由模板列激活）**：模板中每个连接点可**同时声明运动副列与衬套列**；模式是**实例化时的选择**，装配后可在两模式间任意切换，且切换不改变几何、体身份、连接点位置。激活规则（**已按实测修正，不得按字面简化**）：某一列存在才激活该列，另一列不被激活；**只有 joint 列、没有 bushing 列的点（如臂外点、拉杆两端、齿条导轨）在两个模式下都保留其 joint**——实测 C 模式下这 9 个 joint 仍然存在（`benchmark_axle.py` 实测 C `constraints=9`：外点 4 + 拉杆 4 + rack 导轨 1），C 模式并非「只保留衬套」。C 模式激活的衬套刚度来自模板默认属性或属性文件，**不再使用零刚度占位**。

> 说明：用户原话「在 K 模式只激活运动副、C 模式只激活衬套」需要按此精确理解——它描述的是**同一连接点上有两列时的取舍**，不是「C 模式丢弃所有 joint」。计划首版曾按字面写「C 模式只激活衬套列、未激活列不产出任何对象」，会丢掉 C 模式下必须保留的 9 个 joint，与实测及 04 冻结的 C 基线冲突（审核 `cac8e120` 的 B2）。

**G4（属性文件）**：衬套、弹簧、减振器等弹性元件的属性由外部属性文件提供；同一模板加载不同属性文件得到不同刚度/阻尼特性，模板本身不变。

**G5（准静态与动态合并）**：`kc_quasi_static` 与 `axle_dynamic` 合并为**同一个仿真**，由 study 选择准静态或动态。准静态 study 使用与动态相同的轮胎力律选择（`fiala`/`pac2002`/`native_brush`），但只让垂向刚度与轮胎尺寸、质量生效（模型退化）。轮胎质量归属轮胎本身。

**G6（输出声明与 request）**：总成与试验台各自声明自己的**最小单位输出**；所有衍生结果（KC 指标、操稳指标等）由最小单位输出运算得到。提供类似 Adams Car Request 的自定义输出机制。现有 `report/` 指标重述为内置衍生输出，既有结果不变。

**G7（总成 × 试验台正交组合）**：总成与试验台独立注册，运行 = 选一个总成 + 选一个试验台。现有 7 个组合的默认行为保持可用；新增组合（如单轴 + 四立柱）无需新写驱动坐标与结果排布。

**G8（基线重录受控）**：D4 授权重录基线，但**逐阶段、逐项**重录并登记——记明哪个基线、因哪一步、重录前后的值。禁止一次性全量重录掩盖回归。


**G9（子系统 role 与模板解耦，需求 20）**：每个子系统 role（`suspension`/`steering`/`wheel`/`chassis`/`brake`/`drive`）与**实现模板**分离：role 定义接口（几何挂点、参数槽、输出、力矩通道），模板定义实现（是否产出刚体、产出哪些）。同一 role 下可注册多个模板，装配时按名选择。这是 G2b 的机制基础，也是「简化现在、复杂后续」不返工的前提。

**接口的强制方式（已定死）**：**单一 `Template` 数据结构 + 声明式 `RoleSpec` 校验**，**不用「基类 + 六个 role 子类」**。所有子系统模板共用**同一种模板格式**，区别只在 role 标签、刚体/joint/属性/输出等内容；`Template` 结构不因 role 而变，新增 role 只需新增一份 `RoleSpec`。对标 Adams：其 `.tpl` 同样只有一种文件格式，`MAJOR_ROLE` 只是一个字段。`RoleSpec` 字段为 `required_mounts`/`required_slots`/`outputs`/`has_torque_channel`；校验在 `register()` 与 `instantiate()` 两处，失败须点名缺哪个挂点或槽位。

**role 与可用性正交**：role 不表达可选性——「转向在单轴可缺席」「制动/驱动仅整车可用」都是**总成层**的声明（D5/D8），不是 role 的属性。
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
- **现有子系统边界粒度**：`build_front_axle`（`front_axle.py:625`）一个函数里同时产出左右悬架（body/point/constraint）、转向（rack/tie rod，`:838-903`）、轮胎（`:561-574`）、以及 C 模式的占位衬套。用户裁定的六类子系统需从此函数拆出（制动/驱动为整车侧新增；车轮在整车侧由 `_add_wheel` 产出、单轴侧归试验台）。
- **`symmetric_proxy` 与 `explicit` 两条拓扑**：`schema/model.py:173` 的 `topology`；`symmetric_proxy` 不允许 `joints`（`:193-196`），副由硬点自动生成；`explicit` 走 `_build_explicit_axle`（`:387`）由用户声明副，其 `_explicit_constraint`（`:271-315`）**已支持全部 8 种副**。
- **基线文件位置**：`packages/suspension_multibody/tests/data/` 下的 `kc_baseline/`、`kc_perf_baseline.json`、`kc_perf_baseline_native.json`、`dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/`。C++ 侧 `packages/suspension_kernel/layering_baseline.json`（145 边 / 118 头文件边）。
- **转向在 Python 侧是强制子系统（需求 15 的改动面）**：整车侧 `VehicleModel.steering: SteeringSystemSpec` 必填无默认（`schema/vehicle.py:233`），`symmetric_proxy` 轴还强制含 `rack`/`tie_rod_L`/`tie_rod_R` 刚体（`:249-259`，缺一即 `raise`）；`prepare_vehicle_run` 无条件调 `_build_steering`（`preparation/vehicle_dynamic.py:227`）。**单轴侧同样无条件**：`front_axle.py:643-647` 无条件建 `rack` 体、`:655-660` 每侧无条件建 `tie_rod_{side}`、`:838-873` 每侧两个球铰、`:874-903` rack 中心点与 `rack_guide`；`rack_center` 硬点经 `_ALIASES["rack_center"]`（`:144`）参与 `_lookup`。内核侧不强制（`std::vector<SteeringActuator>` 可空、`kernel/solver.py` 无转向字段），硬依赖全落在 schema 校验与装配层。
- **试验台接口当前是硬编码的，不是自适应的**：KC 试验台无条件要 `rack` 体与 `rack.center` 点（`cases/kc_quasi_static/contract.py:203-221` 的 `rack_drive`/`rack_neutral`），`:288-290` 的 `axis_map["rack"]` 用 `next(...)` 取第一个 `rack_*` 名字（**无 rack 驱动时抛 `StopIteration`，这是无转向总成的直接崩溃点**）；K 侧 `api.py:334-338` 的 `_K_COORDINATES` 写死 `{"left","right","rack"}` 并被 `_k_grid`（`:344-389`，含 `:362-375` 对称简写与 `:376-388` 三元组两条分支）消费，结果侧 `:440` 解包三元组、`:453-457` 写 `drives["rack_displacement"]`、`:273` 无条件取 `"rack"` 位姿、`:256` 写时间序列 metric。10 子任务原验收只到「试验台声明要求 + 无效组合报错点名」，缺「按总成实际能力收缩接口」这一层。
- **无转向在既有代码里的唯一先例是 `explicit` 拓扑**（`front_axle.py:446` 仅在 `"rack" in bodies` 时处理 rack；`symmetric_proxy` 无任何开关），既有测试与基线也**没有**任何无转向装配路径（`tests/model/test_front_axle.py:38-44` 直接断言 `"rack" in assembly.bodies`）。因此需求 15 是**新增路径**，不得藉此改动默认路径；`rack_fixed_to_chassis` 与 `rack_housing` 两个既有分支的语义保持不变。
- **`report/` 现有指标规模**：`report/metrics/{axle,case_specific,common,vehicle}.py` 共 27 个函数定义，是 G6「重述为内置衍生输出」的迁移对象。
- **Adams 官方实现（安装目录 `C:\Program Files\MSC.Software\Adams\2024_1` 实测，第三轮取证）**：
  - **制动子系统两档，同 role 不同模板**：`acar_concept.cdb/subsystems.tbl/` 下两个简化版子系统的 `[PART_ASSEMBLY]` 段数 = **0**（全文只有 `[PARAMETER]`），但**参数集与取值各不相同，不得混为一谈**：
    - `default_brakes.sub`（模板 `_brake_system_4Wdisk.tpl`）：**17 项**，含 `front_brake_bias` **0.6**、`front_brake_mu` 0.4、`front_effective_piston_radius` **135.0**、`front_piston_area` **2500.0**、`front_rotor_hub_wheel_offset` 25.0、`front_rotor_hub_width` 40.0、`front_rotor_width` -25.0、`max_brake_value` 100.0、`brake_reduction` 0.0、`front_brake_left_side_bias` 1.0、`kinematic_flag` 0，后轴同族（`rear_effective_piston_radius` 120.0、`rear_piston_area` 2500.0）。
    - `sedan_brake_system.sub`（模板同为 `_brake_system_4Wdisk.tpl`）：**15 项**，取值不同——`front_brake_bias` **0.65**、`front_effective_piston_radius` **145.0**、`front_piston_area` **3000.0**、`front_rotor_hub_wheel_offset` **-15.0**、`rear_piston_area` 2000.0；**没有** `brake_reduction` 与 `front_brake_left_side_bias`。
    - `convertible_brake_system.sub`（复杂版，模板 `_brake_system_4Wdisk_calipers.tpl`）有 **4** 个 `[PART_ASSEMBLY]`（`front_caliper` 质量 **0.0**、`front_rotor` 6.72kg、`rear_caliper` 0.0、`rear_rotor` 5.70kg）+ `[LINK_GEOMETRY]` rotor/rotor_hub + 液压曲线 `master_cylinder_pressure`/`rear_brake_line_pressure`。另有 `_detailed_brake.tpl`（90KB，更复杂的一档）。
    **三者 `MAJOR_ROLE` 同为 `brake_system`**。**连复杂版的卡钳质量都是 0.0**——它只作 marker/几何载体。
  - **制动力矩公式**（本项目 `handling_step_steer_dynamic.adm:8875-8889` 的 `SFORCE/33` 原文，前左轮）：`2.0*2500.0*IF(0:0,1.0,0.0)*0.6*VARVAL(96)*1.0*0.1*0.4*145.0*STEP(VARVAL(281),-10.0D,1,10.0D,-1)` = 2（pad 数）×2500（`front_piston_area`）×左右侧选择×0.6（`front_brake_bias`）×`VARVAL(96)`（制动需求，`testrig.vas_brake_demand`）×1.0（效率）×**0.1**×0.4（`front_brake_mu`）×145.0（`front_effective_piston_radius`）×按轮速符号反向。后轴用 `(1.0-0.6)` 与 `130.0`（`SFORCE/31`、`/34`）。
    **两个必须登记的未核实项（不得当成已核实）**：
    1. 常数 `0.1` 不等于 `1/max_brake_value`（后者为 0.01），且同样出现在管路压力式（`VARIABLE/274-276`）中，属模板内固定归一化因子，来源待核实。
    2. **公式常数与哪个基准 `.sub` 对不上**：该 `.adm` 的 2500/0.6 取自 `default_brakes.sub` 口径，而 145.0 取自 `sedan_brake_system.sub` 口径（`default_brakes.sub` 是 135.0）。即该源 `.adm` 是用一套**混合参数**生成的，D10 的「与 Adams 简单版逐参数对标」**没有唯一基准文件**——实施时须先确定以哪一个 `.sub` 为基准（建议 `default_brakes.sub`，因其是 `_brake_system_4Wdisk.tpl` 的默认装配），并在 PROGRESS 中登记该选择。
  - **驱动的简化/复杂两态**：Adams 的 `_powertrain.tpl` 是有刚体的（`TR_Powertrain.sub`：`powertrain` 300kg + `diff_output` 2kg×2 + 发动机悬置衬套 + 差速器曲线 `MDI_viscous.dif` + 变速器/离合参数）；`help/adams_car/appendix/drivelines.html` 明确 `pvs_driveline` 参数控制 driveline 组件激活（**0=Inactive, 1=Active**）。即 Adams 同时支持「失活 + 轮端力矩」与「带刚体传动链」两态。
  - **单轴装配不含 wheel/brake/powertrain 子系统**：`acar/examples/vehicles/achassis_gs.vdb/assemblies.tbl/acar_gs_front.asy` 的全部构成是 `[SUBSYSTEM] suspension/front` + `[SUBSYSTEM] steering/front` + `[TESTRIG] USAGE='__MDI_SUSPENSION_TESTRIG'`；车轮由试验台提供，证据是 `[PARAMETER]` 段含 `testrig_tire_property_file='RIGID_WHEEL'`、`testrig_wheel_radius=300.0`、`tire_stiffness=200.0`（注释「Used by suspension testrig tire」）以及 `brake_ratio=0.55`、`drive_ratio=0.5`。对照整车装配 `acar_gs_full.asy` 有 `wheel`（前/后）、`powertrain`、`brake_system`、`body` 全部子系统。**这印证了需求 17（单轴不要制动/驱动）与需求 19（单轴车轮归试验台）**。
  - **车轮是独立 Major Role**：`shared_car_database.cdb/subsystems.tbl/TR_Front_Tires.sub` 的 `MAJOR_ROLE='wheel'`、模板 `_handling_tire.tpl`；整车装配用 `Major Role : wheel` 块。但**不参与单轴装配**。

## 三层目标结构

```
模板 Template（专家定义，使用者无需关注）
  ├── parts：有哪些部件
  ├── connections：每个连接点同时声明 joint 列与 bushing 列
  ├── 弹性元件槽位：spring / damper / bushing 的引用
  ├── property_slots：属性文件需要提供什么
  └── outputs：模板自带的输出声明
        ↓ 实例化（填属性文件 + 选 K/C）
子系统 Subsystem（用户裁定的六类；role 与模板解耦，见 G9）
  ├── 左右悬架（suspension）
  ├── 转向（steering）              ← 单轴侧可缺席（需求 15）
  ├── 车轮（wheel；含轮体与轮胎）    ← 单轴侧由试验台提供（D9），整车侧为子系统
  ├── 车身（chassis）
  ├── 制动（brake）                 ← 仅整车；简化模板 0 刚体，可换复杂模板（G2b）
  └── 驱动（drive）                 ← 仅整车；简化模板 0 刚体，可换复杂模板（G2b）
        ↓ 与试验台组装
仿真总成 SimulationAssembly
  ├── 悬架实验总成 = 左右悬架 + [转向（可缺席）] + 车轮（试验台提供）+ 悬架试验台（单轴）
  │     （不含制动、驱动；对标 Adams `acar_gs_front.asy`）
  └── 整车实验总成 = 前后悬架 + 转向 + 车身 + 车轮 + 制动 + 驱动 + 整车KC试验台（整车）
  └─ 试验台按所选子系统自适应接口：总成不提供的坐标，试验台收缩而不硬要求（需求 15）
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
 ├→ 04 子系统拆分（悬架/转向/车轮/车身/制动/驱动）+ 转向可缺席 + 简化制动/驱动（可替换模板）
  │      ↓
  │   05 模板实例化与 K/C 列激活
  │      ↓
  │   06 属性文件机制
  │      ↓
  │   09 准静态/动态 study 合并
  │      ↓
  │   10 试验台抽取与正交组合 + 接口自适应 + 单轴侧车轮（需求 15/19）
  └→ 07 输出声明与衍生输出（可与 04-06 并行）
01 → 08 轮胎质量归属与内核耦合（与 03-07 并行，写范围不相交）
                              ↓
                           11 整车实验总成组装 + 整车侧质量落点 + 制动/驱动接入 + 转向不对称登记
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
- **需求 15 不得改变现有 `model_dump` 输出**：结果 provenance 的 `model_hash` 来自 `model.model_dump(mode="json")`（`api.py:116`、`:284`）；无转向的声明方式若新增会进 dump 的字段，会改变所有现有模型哈希与结果字节。参见 04 SPEC 的相应约束。
- **需求 15 不改默认路径**：含转向的悬架实验总成与 5 个 vehicle family 的行为逐位不变；无转向是**新增的可选组合**，通过显式声明该总成不含转向子系统生效，不得靠改默认值或放宽校验来实现。
- **需求 20 的可扩展性不得被简化实现堵死**：简化制动/驱动**不得**把「无刚体」写进 role 接口或装配层的分支条件。role 接口必须对两种模板同构；装配层与试验台不得出现 `if simplified:` 之类的分支。复杂模板落地时只允许新增模板与注册，不允许改试验台/装配层/内核。

## 验证协议

**01 实测现状**：运行现有全量测试、动态哈希、K/C parity、family parity、ABI 符号与版本门，记录失败/skip 原因与当前基线值。中间证据置会话 scratch；冻结夹具与清单是交付物。

**02**：副表与内核 `contract_registry.cpp` 行数由测试强制一致（新增断言）；副表覆盖 8 种真实副 + 2 种 driven；拆截断后现有 3 种副的文档编码**逐字节不变**；构造 5 个负例（未知副名、缺轴、轴退化、行数不符、driven 混入副表）均须失败。

**03**：模板数据结构单元测试；`ConnectionDefinition` 必须同时可带 joint 列与 bushing 列；属性槽声明与输出声明可序列化/反序列化；缺列、双空列、重复 role 均须报错。

**04**：六类子系统可独立实例化；与现役 `build_front_axle` 的产物**逐项对照**（body 集合、点数、约束集合、连接表），差异必须为零或逐项登记理由。
**需求 15 的可选性证据（04）**：同一模型去掉转向子系统后，其余产物与含转向产物**逐项差集一致**——`bodies` 少 `rack`/`tie_rod_{L,R}`、`points` 少 rack 与拉杆相关条目、`constraints`/`ideal_constraints` 少 4 个球铰与 `rack_guide`，其余（左右悬架、轮胎、车身、硬点镜像）逐位不变；且默认路径（含转向）逐位不变。

**05**：同一模板实例化 K 与 C，几何/体身份/连接点/驱动坐标**完全一致**，仅 joint 与 bushing 列不同；`with_mode` 幂等；`with_mode("C")` 结果与直接实例化 C **逐位一致**（这是防"第二套实现"的关键断言）。

**06**：同一模板 + 同一属性文件 → 逐位一致；换属性文件 → 刚度/阻尼按属性变化而模板不变；属性缺失、类型错误、越界值均须报错。

**07**：输出声明解析；总成输出 ∪ 试验台输出的合并与冲突检测；衍生输出只用最小单位输出求值（不得旁路读内核）；现有 `report/` 27 个指标函数重述后结果与旧实现**逐值一致**。

**08**：契约文档新增轮胎质量字段；内核 `Tire` 加 mass/inertia；求解器惯量耦合；**质量守恒类断言**（轮胎质量从 body 迁到 tire 后，整车/整轴总质量与世界质心不变）；ABI 七符号与版本门保持；两包构建与隔离 wheel 复验。

**09**：同一模型装配 + `study="quasi_static"` 与 `study="dynamic"` 两种运行；准静态下只取垂向刚度与尺寸/质量的断言（对照动态结果的轮胎侧向/纵向输出为零或未激活）；准静态与动态共享同一装配入口的证据。

**10**：总成 × 试验台矩阵可查询；现有 7 个组合行为不变；构造无效组合（试验台要求总成不提供的坐标）须在准备阶段报错并点名；新增至少一个此前不存在的组合（单轴 + 四立柱或单轴 + 随机路面）并跑通。
**需求 15 的自适应证据（10）**：同一个悬架 KC 试验台在含转向与不含转向两种总成上都能跑；不含转向时 `axis_map` 不含 rack 轴、K 网格降维为纯轮跳、结果对象无 rack 相关通道；`cases/kc_quasi_static/contract.py:288-290` 的 `next(...)` 与 `api.py:334-338` 的 `_K_COORDINATES` 不得再抛 `StopIteration`（须收缩或给出点名的可读报错）。

**12 独立终局命令**（逐条记录退出码）：
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

独立逐条确认 G1–G9：

1. **G1**：副表与内核行数表由测试强制一致；任意总成可声明任意副（有测试证明 kc 能发出 `universal`/`cylindrical`/`convel`/`inplane`/`fixed`，且内核接受）；作者层无残留的 N 种截断。
2. **G2**：三层结构存在且可运行——`templates/`、`subsystems/`、试验台三层各自有注册表与实例化路径；**六类子系统**（悬架/转向/车轮/车身/制动/驱动）可独立实例化；**两种组装都成立**：悬架实验总成（单轴：左右悬架+[转向可缺席]+车轮（试验台提供），**不含制动/驱动**）与整车实验总成（前后悬架+转向+车身+车轮+制动+驱动+整车 KC 试验台）；**且子系统在总成内可选（需求 15）**——悬架实验总成可在**没有转向系统**的情况下构造与运行，试验台按总成实际提供的子系统自适应收缩接口。
3. **G3**：模板中一个连接点同时带 joint 列与 bushing 列；K/C 为实例化选择；`with_mode` 切换后几何/体身份/连接点/驱动坐标不变、仅激活列变；`with_mode("C")` 与直接实例化 C 逐位一致；C 模式衬套刚度**不再全为零**（有测试断言非零且来自模板/属性文件）。
4. **G4**：属性文件可加载；同模板 + 不同属性文件得到不同刚度/阻尼而模板不变；模板可直接给数值（现役行为）也可给引用。
5. **G5**：准静态与动态是同一仿真的两个 study；同一装配入口；准静态只激活垂向刚度与尺寸/质量，力律选择（`fiala`/`pac2002`/`native_brush`）对两者一致可用。
6. **G6**：总成输出与试验台输出各自声明并有合并规则；衍生输出只用最小单位输出求值；现有 27 个 `report/` 指标重述后结果逐值一致；可自定义新输出而无需改包代码。
7. **G7**：总成 × 试验台矩阵可查询；现有 7 个组合行为保持；无效组合在准备阶段报错点名；至少一个新组合跑通；**试验台按总成实际提供的子系统自适应收缩接口**——无转向总成下 rack 驱动轴与 rack 输出整体消失、K 网格降维为纯轮跳，试验台仍可跑（不以内部异常或填零代替）。
8. **轮胎质量归属（需求 10）**：有**独立于 08 自证**的验证——轴侧与整车侧都把质量写进 tire entry，且整车/整轴总质量与世界质心不变（不采信 08 自身的守恒断言作为唯一证据）。
9. **G8**：每次基线重录都有登记（文件 + 步骤 + 前后值 + 等价性判定）；无"先改基线让门变绿"的痕迹；`ABI 七符号与版本（15/30/1/1）`未变或变更有独立裁决记录。
10. **G2b（需求 20，本轮核心）**：简化制动/驱动是**可替换的提供者**，不是一次性写死——同一 role 下简化模板（0 刚体）与复杂模板（带刚体）**接口同构**；装配层与试验台**无** `if simplified` 类分支；有测试证明「用复杂模板替换简化模板不需要改试验台与装配代码」。
11. **G9（role 与模板解耦）**：六个 role 各自定义接口（几何挂点、参数槽、输出、力矩通道），模板定义实现；同 role 可注册多模板并按名选择。
12. **制动/驱动可用性矩阵（需求 17）**：悬架实验总成**不含**制动/驱动且有测试锁定；整车实验总成**必须含**且有测试锁定。制动参数（`brake_mu`/`piston_area`/`effective_piston_radius`/`front_brake_bias`/`max_brake_value`）与 Adams 简单版**逐参数对标**，力矩公式按 `SFORCE/31-34` 实测原文；式中常数 `0.1` 的来源已核实或明确登记为未核实项。

**端到端独立验收（不依赖子任务自证）**：用**同一个模板**完成下列九件事，全部通过才算 Goal 达成——

```text
(a) 同一个轴的模板，K 模式跑一次、C 模式跑一次 → 只差激活列
(b) 同一个轴模板 + 同一属性文件，准静态 study 跑一次、动态 study 跑一次
(c) 换一份属性文件重跑 (b)，刚度/阻尼按属性变化而模板与几何不变
(d) 总成 × 试验台矩阵任取两个组合跑通，其中一个此前不存在；
    并用最小单位输出 + 自定义表达式算出一个新指标
(e) 整车实验总成（前后悬架+转向+车身+车轮+制动+驱动+整车 KC 试验台）跑通一次，
    并证明它与悬架实验总成共享同一套子系统定义
(f) 轮胎质量归属：轴侧与整车侧各验证一次质量来自 tire 而非轮端 body
(g) 无转向的悬架实验总成：同一模板组装一个**不含转向子系统**的单轴总成 + 悬架 KC 试验台，跑通一次；证明 rack 驱动轴与 rack 相关输出整体消失、K 网格降维为纯轮跳，且同一试验台在含转向总成上行为不变（不是报错绕过）
(h) 制动/驱动的简化→复杂可替换性（需求 20）：用同一 role 下的**复杂模板**替换简化模板，
    装配层与试验台代码零改动即可跑通；且简化模板下整车总成的轮端力矩与 Adams 简单版逐参数对标
(i) 可用性矩阵：悬架实验总成不含制动/驱动、整车实验总成含制动/驱动，两侧都有测试锁定
```

**数值门为独立项**：`dynamic_hash_sentinel`、`kc_parity_check`、`case_parity_check` 三门在本 Epic 期间的每次基线重录后都必须重新通过，且重录本身有登记。每行 DONE 不代替这些条件。

## 风险与回退

- **三层重构的风险**：模板/子系统/试验台三层会重写 `preparation/` 与 `simulation/` 的组织方式。缓解：03-05 分步落地，每步保持现有入口可用；旧路径以适配器方式保留到 10 之后再决定删除。
- **轮胎质量改动（D2=乙）的风险**：求解器显式耦合轮胎惯量，可能改变数值结果并触及动态字节门。缓解：先做质量守恒类断言（总质量与质心不变），再评估数值差异；差异必须登记为"物理改变或数值路径改变"，不得默默吸收。
- **准静态发轮胎的风险**：kc 从"无轮胎"变为"有轮胎（垂向激活）"，`kc_baseline/` 必然变。缓解：先建立"垂向激活等价于原 VerticalTireElement"的对照，再重录。
- **旧路径删除的风险**：不可逆。缓解：删除集中在最后阶段，删除前完成调用方迁移与前置门禁；删除范围只覆盖**已无生产调用者**的部分。
- 真实 Adams 执行需现有安装/许可；默认使用已冻结证据与渲染测试，缺少真实执行明确记录，**不能声称整车数值等价**。
- 最终仍有不相关既有失败则独立列明；任何新增失败阻断完成。
- **简化实现堵死升级路径的风险（需求 20）**：若把「无刚体」写进 role 接口或装配层分支，后续复杂模板将无法只靠新增模板落地。缓解：03 建立 role/模板解耦结构、04 的简化实现只作为**一个模板**存在，并在 04/11 各设一条「复杂模板替换不改装配层」的测试；D11 的验收是硬门。

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
