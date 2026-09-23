# Epic 进度：20260922-suspension-template-architecture

## 恢复信息

形态：epic。
- 状态：**已完成**（01-12 全部 DONE；终局验收通过）
当前：12 个子任务全部收官，终局验收已独立执行——门禁全集实跑通过、G1-G9 逐条达成、端到端九件事 (a)-(i) 全部通过。登记 4 项未闭合项（均不阻断 Goal，见 12 的 PROGRESS 第五节）。
文件：`.codex-tasks/20260922-suspension-template-architecture/`（`EPIC.md` + `SUBTASKS.csv` + 本文件 + `tasks/20260922-01..12/`）。
验证：01/02/03 的门禁已复跑通过（见各子任务 PROGRESS 的状态头）；04 尚未产出验收证据。

## 用户原话（本轮需求原文，逐条）
15. 「针对G2：悬架实验总成可以没有转向系统，试验台要更具所选子系统自动适配接口」（第二轮补强；本轮仍只规划，不实施代码）

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

### 用户裁决登记（D1-D4，首轮；D5-D7 见第二轮、D8-D11 见第三轮）

| 编号 | 问题 | 用户裁决 | 落点子任务 |
|---|---|---|---|
| D1 | 准静态轮胎力律如何落地 | **不新增内核力律**：准静态接受 `fiala`/`pac2002`/`native_brush`，只使用其垂向刚度与轮胎尺寸、质量，相当于**模型退化** | 09 |
| D2 | 轮胎质量归属 | **归属轮胎**（选"乙"：tire 成为独立惯量来源，求解器显式耦合） | 08 |
| D3 | 架构层次 | **彻底改三层**：模板 → 子系统 → 仿真总成 | 03/04/05/10 |
| D4 | 基线重录 | **可以重录基线**（但须逐项登记） | 05/06/08/09/11 |

## 证据（制定计划前实测）

主代理与两个 explorer 子代理在制定本计划前完成了实测，关键结论：

- **内核副注册表已完备**（`cpp/src/contract/contract_registry.cpp:23-30`）：10 项，含 8 种真实副 + 2 种驱动坐标。**内核侧无需改动即可支持任意总成用任意副。**
- **截断点唯一**：`cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 只映射 3 种副；装配层 `types.py` 与 schema `IdealJointSpec` 均已支持 8 种。**02 的实质是拆截断，不是建底座。**
- **`vertical_linear` 是死名字**：在 `contract_registry.cpp:36` 名单里，但内核解析表（`cpp/src/cases/contract_model.cpp:948-950`）只有 `native_brush`/`pac2002`/`fiala`，枚举 `VehicleTireModelKind`（`cpp/include/mb_model/enums.hpp:61-70`）也只有 4 项。**D1 正确绕开了它。**
- **垂向退化已有底座**：`cpp/src/tire/fiala/forces.cpp:20-29` 的 `fiala_elastic_force` 无曲线时即 `tire.k * penetration`。
- **内核 `Tire` 无质量字段**（`cpp/include/mb_model/types.hpp:101-146`）；质量现挂轮端 body（`schema/vehicle.py:60` 的 `WheelSpec.mass` → `preparation/assembly/vehicle.py:653,722` 合并）。**D2 是真数据结构改动。**
- **契约 schema 对 tire 闭合**：`multibody_model.schema.json` 的 tire 定义 `additionalProperties: false`，08 新增字段必须先改 schema。
- **生效的 native 边界是文档契约**（`kernel/__init__.py:186-205` 传两份 JSON payload），不是 struct ABI。
- **C 模式零刚度占位衬套实测**：`benchmark_axle.json` 下 8 条衬套刚度范数**全为 0.0**（`front_axle.py:737`、`:802`）。
- **K/C 映射实测**（现役 `symmetric_proxy`）：K `constraints=13`/`ideal=13`/`bushings=0`/文档 `elements=0`；C `constraints=9`/`ideal=17`/`bushings=8`/文档 `elements=8`。
- **KC 与 axle_dynamic 实测七项差异**（不止"求解器与轮胎"）：结果对象、模型 schema、轮胎有无、K/C 有无、case 结构、时间网格语义、驱动坐标命名。
- **现有 7 个 (总成, 试验台) 组合**（`simulation/dispatch.py:20-33`）：`axle×{kc_quasi_static, axle_dynamic}`、`vehicle×{vehicle_dynamic, vehicle_kc, handling, ride_four_post, ride_random_road}`。
- **`report/metrics` 共 27 个指标函数**（axle 3 + case_specific 12 + common 8 + vehicle 4），是 07 的迁移对象。
- **全量套件实测**：`783 passed, 1 skipped, 1 xfailed`（退出 0）；`--strict --final` 0；kernel 15；contracts 22；动态哈希 26/26 逐位一致；K/C parity 0；8 family accepted；ruff/ty 0；两包 build 0；`git diff --check` 0。

## 计划结构

`SUBTASKS.csv` 12 个子任务，依赖链无环（已用脚本校验）：

```
01 基线冻结
 ├→ 02 统一副底座 ─┬→ 03 模板数据模型 → 04 子系统拆分 → 05 K/C 列激活 → 06 属性文件 ─┐
 │                 └→ 07 输出声明（与 04-06 并行，写范围不相交）                    │
 └→ 08 轮胎质量（与 03-07 并行，写范围不相交）─────────────────────────────────┴→ 09 study 合并 → 10 试验台正交
                                                                                   ↑          ↓
                                                              07 输出声明 ─────────┘   11 整车实验总成
                                                                                            ↓
                                                                                       12 终局验收
```

**并行约束**：03 与 04 都触及 `preparation/assembly/types.py`，必须串行；05 与 06 在 `templates/**` 上重叠，必须串行；08 只写 C++ 内核 + 契约 + native 镜像，与 03-07 不相交；07 只写 `outputs/**` 与 `report/metrics/**`，与 04-06 不相交。

**写入范围冲突已在各子任务 SPEC 显式声明**（"不写"清单），防止并行写者互相覆盖。

## 本轮计划自检结果

- `SUBTASKS.csv`：13 行（含表头）、11 字段一致、12 个子任务、id 唯一、`depends_on` 无环且依赖存在、状态全 `TODO`、类型全 `single-full`。
- 12 个子任务目录均含 `SPEC.md` + `TODO.csv` + `PROGRESS.md` + `raw/`。
- 12 个子任务的 `TODO.csv` 全部 **8 列一致**、id 连续、状态全 `TODO`、`completed_at` 空、`retry_count=0`；叶子步骤数 **9-13 行**（04 为 13、11 为 13、10 为 12、12 为 12、03 为 10；第三轮按需求 16-20 扩编，03 因 role/RoleSpec 增一步）。
- 各 `PROGRESS.md` 均写明「0/N 步骤 TODO，尚未实施」与恢复信息。
- 本轮**未**把任何实施行置为 DONE，**未**写生产代码，**未**重录任何基线。

## 计划修订记录

### 用户裁决登记（D5-D7，第二轮需求 15）

- **改了什么**：新建本 Epic 全部文件（`EPIC.md`、`SUBTASKS.csv`、本文件、12 个子任务三件套）。
- **为什么**：用户要求「先制定计划，不具体实现代码」，并给出 13 条需求补充与 4 项裁决（D1-D4）。
- **影响**：无既有代码改动；这是一条新 Epic，与 20260921 系列的架构偏差收敛 Epic 无冲突（后者已 9/9 DONE 并收口）。
- **修订**：按 fixer 子代理回报的实测值修正了两处事实——`WheelSpec.mass` 在 `schema/vehicle.py:60`（原写 64）；契约 schema 对 tire 是 `additionalProperties: false`，08 必须先改 schema。并据此收窄了 08 的写范围（Python 侧发射落点归 02/04-06，避免并行冲突）。

### 2026-09-22 独立审核后的修订（4 个阻断项全部处理）

独立审核 `code-reviewer cac8e120` 就本 Epic 的规划给出 4 个阻断项，逐项处理如下：

| 阻断 | 内容 | 处理 |
|---|---|---|
| B1 | **整车实验总成无子任务承接**（用户需求 13 后半「前后悬架+转向+车身+轮胎+整车 kc 试验台→整车实验总成」落空） | **新增子任务 11「整车实验总成组装」**，原终局验收顺移为 12；EPIC 的 G2、三层结构图、依赖图、Done-When 与端到端判据同步补整车侧 |
| B2 | **K/C 激活规则与实测矛盾**：G3 与 05 SPEC 原写「C 模式只激活衬套列、未激活列不产出任何对象」，照此实现会丢掉 C 模式必须保留的 9 个 joint | 实测确认（C `constraints=9`：外点 4 + 拉杆 4 + rack 导轨 1）后改写 G3 与 05 SPEC 目标 2 的激活规则：**有哪列激活哪列，仅 joint 列的点两模式都保留**；并在 03 SPEC 补实测映射细节（K 模式仅 `inner_front` 有副、`inner_rear` 无副），防止实现者从需求 2 的字面推断 |
| B3 | **10 缺对 07 的依赖**（10 要接入 07 定义的输出声明与合并） | `depends_on` 改为 `07;09`，并在 10 SPEC/SUBTASKS 注明依赖理由 |
| B4 | **D2 的 vehicle 侧发射落点无归属，且质量归属无独立终局判据** | 整车侧落点归 11（`cases/vehicle_dynamic.py`、`preparation/vehicle_dynamic.py`），轴侧归 09（`cases/axle_dynamic.py`）；EPIC Done-When 新增第 8 条「轮胎质量归属需独立于 08 自证」，端到端判据补 (e) 整车实验总成、(f) 质量归属 |

**可选改进的处理**：O1（03 SPEC 内点描述不精确）已修；O2（SUBTASKS 行级命令弱于 SPEC 门禁）部分处理——05 行的命令已改为跑自己的 `tests/instantiation`，其余行保留步骤级为完整门禁；O3（types.py 冲突描述为幻影）保留 EPIC 的串行要求但已在 03/04 SPEC 说明实际写范围不相交；O4（native dll 由多个 build 门禁重写）已登记；O5（只测 K→C 单向）已补「双向等价」要求。

**注意**：本轮修订曾一度引入依赖环（09→11→10→09），已修正为 09 不加 11、整车侧落点归 11；修正后 `depends_on` 无环（脚本校验）。

### 2026-09-22 第二轮需求补强（需求 15：转向可选 + 试验台自适应）

用户第二轮原话：「针对G2：悬架实验总成可以没有转向系统，试验台要更具所选子系统自动适配接口」。

**核查结论：这两条在首版计划里都是缺口。** 证据：

- **转向在 Python 侧是强制子系统**：`schema/vehicle.py:233` 的 `VehicleModel.steering: SteeringSystemSpec` 必填无默认；`:249-259` 把 `rack`/`tie_rod_L`/`tie_rod_R` 列入 `symmetric_proxy` 轴的 `required_bodies`（缺一即 `raise`）；`preparation/vehicle_dynamic.py:227` 无条件调 `_build_steering`。单轴侧同样无条件：`front_axle.py:643-647` 无条件建 `rack` 体、`:655-660` 每侧建 `tie_rod_{side}`、`:838-873` 每侧两个球铰、`:874-903` rack 中心点与 `rack_guide`；`rack_center` 硬点由 `_ALIASES["rack_center"]`（`:144`）经 `_lookup` 强制。内核侧反而不强制（`std::vector<SteeringActuator>` 可空、`kernel/solver.py` 无转向字段）。
- **试验台接口是硬编码的**：KC 试验台无条件要 `rack` 体与 `rack.center` 点（`cases/kc_quasi_static/contract.py:203-221`），K 模式控制轴映射写死在 `api.py:334-338` 的 `_K_COORDINATES` 并被 `_k_grid`（`:344-389`）两条分支消费，且 `contract.py:288-290` 的 `axis_map["rack"]` 用 `next(...)` 取第一个 `rack_*` 名字——**总成无转向时会抛 `StopIteration`**，不是报错点名。10 原验收只到「声明要求 + 无效组合报错点名」，缺「按总成能力收缩接口」。
- **`schema/**` 原本无人可写**：04/05/09/11 的写范围里 `schema/**` 全是只读，而转向可选必然要动 `schema/model.py`（单轴侧）。
- **无转向的唯一先例是 `explicit` 拓扑**（`front_axle.py:446` 仅在 `"rack" in bodies` 时处理 rack；`symmetric_proxy` 无任何开关），既有测试与基线没有无转向路径（`tests/model/test_front_axle.py:38-44` 直接断言 `"rack" in assembly.bodies`）。

**用户裁决（D5-D7）**：

| 编号 | 问题 | 裁决 | 落点 |
|---|---|---|---|
| D5 | 可选转向的范围 | **只放开单轴悬架实验总成**；整车侧 `SteeringSystemSpec` 与 `required_bodies` 维持必填，`_build_steering` 不改，5 个 vehicle family 行为不变 | 04 / 11 |
| D6 | 试验台接口如何自适应 | **自适应收缩**：无转向时 rack 驱动轴与 rack 相关输出整体消失，K 网格降维为纯轮跳，试验台仍可跑 | 10 |
| D7 | 承接落点 | **并入现有子任务，不新增编号** | 04/10/11/12 |

**改了什么**：

- `EPIC.md`：G2 补「子系统在总成内可选」；G7 补「试验台按总成能力自适应收缩」；需求原话登记第 15 条；裁决表补 D5-D7；事实节补四条现状（转向强制、试验台硬编码与崩溃点、无转向唯一先例是 `explicit`、`model_dump` 哈希传导）；三层结构图、依赖图、验证协议 04/10、Done-When G2/G7、端到端判据 (a)-(g) 与冻结约束同步。
- `SUBTASKS.csv`：04 标题与验收补「转向子系统可缺席」并注明新增写范围 `schema/model.py`（措辞与 SPEC 对齐）；10 补「试验台接口自适应」；11 补「登记整车侧不放开并加锁定测试」；12 补无转向端到端判据。
- `tasks/20260922-04-subsystems/{SPEC,TODO}.csv`：目标加第 5、6 条（缺席时哪些产物整体不产出、不得用退化 rack 体、`AssemblyCapabilities` 契约含字段名与命名空间口径）；约束加「只落单轴侧」「不改默认装配」「新增字段不得改变 `model_dump` 输出」；写范围显式加 `schema/model.py`；验收加第 8-10 条（逐项差集一致并注明按点表推导、能力描述、默认路径逐位不变）；TODO 由 9 步扩为 10 步。
- `tasks/20260922-10-rigs/{SPEC,TODO}.csv`：目标加第 5 条（自适应收缩，含逐处改造点 file:line——模型文档侧、case 侧 `StopIteration` 崩溃点、K 网格 `_k_grid` 两条分支、K 结果侧、`_run_axle_quasi_static` 准静态重放路径、`StateResult.drives` 键集——以及「收缩 vs 报错」的判据区分与「勿过度收缩」的范围边界）；验收加第 6 条；验证协议加第 5 步；TODO 加第 5 步并重排后续编号。
- `tasks/20260922-11-vehicle-assembly/{SPEC,TODO}.csv`：非目标加「整车侧不放开」；验收加第 7 条（不对称锁定测试）；验证协议加第 4 步；TODO 插入新步骤。
- `tasks/20260922-12-acceptance/{SPEC,TODO,PROGRESS}`：标题笔误「子任务 11」改为 12；端到端项由四件事改为七件事 (a)-(g)（SPEC、PROGRESS、TODO 三处旧口径残留一并清除）；TODO 重复行清理。

**影响与风险**：

- 无转向是**新增路径**，含转向的默认路径与 7 个现有组合必须逐位不变——各 SPEC 已把「任一基线变化即停止上报」写死；04 仍保持「不得重录任何基线」。
- 逐项差集一致（不含转向侧 = 含转向侧去掉 rack/tie_rod/4 个球铰/rack_guide 的精确子集）是本轮新增的关键判据，用于防止「拆子系统时顺便改了别的东西」。
- 需求 15 只动单轴侧，整车侧的不对称是**有意的**，11 用锁定测试固化，避免后续被当成缺陷"修"掉或为对称而放开（那会波及 5 个 vehicle family 与整车基线）。
- **`model_dump` 哈希传导风险（复核时发现）**：`api.py:116`/`:284` 把 `model.model_dump(mode="json")` 的规范化哈希写进 `Provenance.model_hash`，`io/artifacts.py` 又把它哈希成 artifact manifest 的 `model_sha256`（被 `dynamic_hash_sentinel.py` 的 key 集覆盖）。因此 04 若给 `FrontAxleModel` 加一个会进 dump 的字段，含转向模型的 `model_hash`/`model_sha256` 会变——与「不得重录任何基线」隐性冲突。已在 04 SPEC 与 EPIC 冻结约束写明：新增字段不得改变现有 `model_dump` 输出，且须有断言证明。
- **能力描述契约必须冻结在 04**：04 与 10 的接口（`AssemblyCapabilities`：字段名、取值枚举、坐标命名空间用 `wheel_drive_L`/`rack_drive` 这套而非 `axis_map` 的分组键）已写进两份 SPEC，否则两个独立实现者会各自命名而接不上。

### 2026-09-22 第三轮需求（需求 16-20：制动/驱动/车轮子系统 + 可扩展性）
用户第三轮原话见 `EPIC.md` 的需求 16-20。核心是**两条架构要求**：制动与驱动要成为子系统（整车必须有、单轴不要），且**简化实现必须能升级为带刚体的复杂实现**。
**取证（Adams 2024_1 安装目录，`C:\Program Files\MSC.Software\Adams\2024_1`）**：

- **同 role 不同模板是 Adams 的既有设计**：`acar_concept.cdb/subsystems.tbl/` 下 `default_brakes.sub`、`sedan_brake_system.sub` 的 `[PART_ASSEMBLY]` 段数 = **0**（只有 `[PARAMETER]`），而 `convertible_brake_system.sub` = **4**（`front_caliper` 质量 0.0、`front_rotor` 6.72kg、`rear_caliper` 0.0、`rear_rotor` 5.70kg）；四者 `MAJOR_ROLE` **同为 `brake_system`**，模板分别是 `_brake_system_4Wdisk.tpl` 与 `_brake_system_4Wdisk_calipers.tpl`（另有 `_detailed_brake.tpl` 90KB）。**连复杂版卡钳质量都是 0.0**，只作 marker/几何载体。
- **制动力矩公式**（本项目 `handling_step_steer_dynamic.adm:8875-8889` 的 `SFORCE/33` 原文）：`2.0*2500.0*IF(0:0,1.0,0.0)*0.6*VARVAL(96)*1.0*0.1*0.4*145.0*STEP(...)` = pad 数 × `piston_area` × 左右侧 × `front_brake_bias` × 制动需求 × 效率 × **0.1** × `brake_mu` × `effective_piston_radius` × 按轮速符号反向。**式中常数 `0.1` 不等于 `1/max_brake_value`（后者 0.01）**，且在管路压力式（`VARIABLE/274-276`）中同样出现，来源待核实——已作为显式未核实项写入计划。
- **单轴装配不含 wheel/brake/powertrain**：`acar/examples/vehicles/achassis_gs.vdb/assemblies.tbl/acar_gs_front.asy` = `[SUBSYSTEM] suspension/front` + `[SUBSYSTEM] steering/front` + `[TESTRIG] '__MDI_SUSPENSION_TESTRIG'`；车轮由试验台提供（`testrig_tire_property_file='RIGID_WHEEL'`、`testrig_wheel_radius=300.0`、`tire_stiffness=200.0`）。对照 `acar_gs_full.asy` 含 `wheel`（前后）、`powertrain`、`brake_system`、`body`。**这直接印证需求 17 与需求 19**。
- **驱动的两态**：`_powertrain.tpl` 有刚体（`powertrain` 300kg + `diff_output` 2kg×2 + 发动机悬置衬套 + `MDI_viscous.dif`）；`help/adams_car/appendix/drivelines.html` 明确 `pvs_driveline`（0=Inactive/1=Active）控制 driveline 组件激活。用户选的「只做分配声明 + 轮端力矩」= 失活路径。

**用户裁决（D8-D11）**：

| 编号 | 问题 | 裁决 | 落点 |
|---|---|---|---|
| D8 | 制动/驱动的承接与粒度 | 并入现有子任务（04 定义、10 接输入、11 整车组装）；0 刚体，只做「分配声明 + 轮端力矩」；**悬架实验总成不含、整车实验总成必须含** | 04/10/11 |
| D9 | 单轴侧车轮归属 | **归悬架试验台**（对标 `__MDI_SUSPENSION_TESTRIG` 的 `testrig_*` 参数）；单轴总成不产出 `wheel.body`，整车侧维持 wheel 子系统 | 04/10/11 |
| D10 | 制动参数对标范围 | 只取力矩子集：`brake_mu`/`piston_area`/`effective_piston_radius`/`front_brake_bias`/`max_brake_value`；转子几何属详细版 | 04 |
| D11 | **简化版的可扩展性（本轮核心）** | 简化制动/驱动是**可替换的提供者**：同一 role 下模板可替换（简化 0 刚体 ↔ 复杂带刚体）；装配层与试验台**不得有分支**；复杂模板落地只允许新增模板 + 注册 | 03/04/10/11/12 |

**改了什么**：

- `EPIC.md`：登记需求 16-20 与裁决 D8-D11；**G2 由四类子系统扩为六类**（左右悬架/转向/车轮/车身/制动/驱动）并给出**可用性矩阵**；新增 **G2b**（简化→复杂可扩展性）与 **G9**（role 与模板解耦）；事实节补四条 Adams 官方取证；三层结构图、依赖图、验证协议 04/10、Done-When、端到端判据由七件事扩为**九件事 (a)-(i)**；冻结约束补「需求 20 不得被简化实现堵死」；风险节补对应风险。
- `SUBTASKS.csv`：03 补 role/模板解耦；04 改六类 + 制动/驱动可用性 + 可替换性；10 补单轴侧车轮与模板无分支；11 补制动/驱动接入与幅值语义；12 改为核对 G1-G9 与九件事判据。
- `tasks/20260922-03-template-model/`：目标加第 3、4、5 条（role 与模板解耦、六个 role 的最小接口、接口强制方式）；**六个 role 全部补齐**（suspension/steering/wheel/chassis/brake/drive，其中仅 brake/drive 有力矩通道）；**强制方式定死为单一 `Template` 结构 + 声明式 `RoleSpec` 校验**（不用基类+六个子类，理由：用户要求所有模板同一格式，Adams 的 `.tpl` 亦然）；约束加「role 接口不得泄漏实现细节」；验收加第 7-11 条（同 role 双模板、制动参数槽、六 role 覆盖、RoleSpec 校验负例、新增 role 不改结构）；TODO 由 9 步扩为 10 步。
- `tasks/20260922-04-subsystems/`：目标改六类并加车轮/制动/驱动三条与「可替换的提供者」第 7 条；约束加 D8/D9/D10/D11 四条；写范围加 `schema/vehicle.py` 的制动参数子集（明确不得动 `SteeringSystemSpec`/`required_bodies`）；验收加第 11-13 条；TODO 由 10 步扩为 **13 步**。
- `tasks/20260922-10-rigs/`：目标加第 6 条（单轴侧车轮由试验台提供）；约束加模板无分支；验收加第 8、9 条；TODO 由 10 步扩为 **12 步**。
- `tasks/20260922-11-vehicle-assembly/`：目标加第 7 条（接入制动/驱动、幅值语义、模板无分支）；非目标加「不得为单轴侧引入制动」；验收加第 8-10 条；TODO 扩为 **12 步**。
- `tasks/20260922-12-acceptance/`：SPEC 改核对 G1-G9 与九件事；TODO 第 3、7、8、9 行同步并新增第 11 行（G2b/G9 核对）。

**关键设计取舍（供实施者理解意图）**：

- **单轴侧仍然没有独立轮体**：D9 把车轮归试验台后，单轴侧维持 `VerticalTireElement`（挂 `upright_{side}`）的现状，**不改**成独立轮体——因此单轴侧不能施加制动/驱动，与需求 17 自洽。
- **内核不需要为单轴侧加制动通道**：`axle_dynamic.cpp:31` 的 `TireRole` 无 `BrakeTorque`，而制动只走 `vehicle_dynamic`（`vehicle_dynamic.cpp:33`），故本轮**不触及内核制动路径**。
- **`brake_torque` 是非负幅值**：方向由内核按轴向转速符号决定（`drive_brake.cpp:89-98`），并带转向节反作用（`:111-120`）；Python 侧只算幅值，不得翻转符号。

## 未闭合项
无。本轮为规划交付，未产生实施性未闭合项。D1-D11 十一项裁决已全部登记并落到具体子任务。

（第二轮「D1-D7 七项裁决已全部登记」的表述已被第三轮取代，见下方第三轮修订记录与本节末尾的未闭合项。）
**第三轮的未闭合项（显式登记，非阻断）**：①制动力矩公式中的常数 `0.1` 来源未核实；②源 `.adm` 的公式常数（2500/0.6/145.0）混用了 `default_brakes.sub`（135.0）与 `sedan_brake_system.sub`（145.0）两套口径，D10 的「与 Adams 简单版逐参数对标」**基准文件须先选定并登记**。两项均已写入 EPIC 事实节与 04 的验收第 12 条。Adams 真实整车数值对标仍为 `BLOCKED`，本轮不改变该结论。


### 2026-09-22 第三轮补充：模板格式统一性与 role 接口强制方式（用户追问）

用户追问：「实现后是否所有模板（转向、悬架、制动、车身等等）都是由同一个模板格式，区别只有刚体、joint、属性等等区别？」

**核查结论：计划原本只写了 3 个 role 的接口**（03 SPEC 标题写「六个 role 的最小接口定义」，正文只列 `brake`/`drive`/`wheel`），`suspension`/`steering`/`chassis` 缺失；且**接口的强制方式未定**（只说「`Template` 增加 `role` 字段」，未排除「基类 + 六个子类」的实现形态）。两处都已补齐。

**答复用户的核心口径**：格式统一，但差异不止刚体/joint/属性——还有 role 标签、几何挂点、力矩通道三类。同一 role 内模板可互换（简化↔复杂），**跨 role 不可互换**（制动模板顶不了悬架 role 的挂点要求）。

**改了什么**：

- `EPIC.md` G9：补「接口的强制方式（已定死）」段——单一 `Template` 结构 + 声明式 `RoleSpec` 校验，不用子类化；补「role 与可用性正交」（role 不表达可选性，转向可缺席/制动驱动仅整车属总成层 D5/D8）。
- `tasks/20260922-03-template-model/SPEC.md`：第 4 条补齐**六个 role 的完整接口**（每个列几何挂点/参数槽/输出/是否有力矩通道）；新增第 5 条定死强制方式（`RoleSpec` 字段 `required_mounts`/`required_slots`/`outputs`/`has_torque_channel`、校验时机 `register()`+`instantiate()`、报错点名、新增 role 只加 `RoleSpec`）；验收加第 9-11 条；验证协议加第 6-7 步。
- `tasks/20260922-03-template-model/TODO.csv`：新增「定义六个 role 的 RoleSpec 与声明式校验」一步（第 5 步），负例步扩为六例（含缺挂点、缺槽位），注册表步补 RoleSpec 校验；由 9 步扩为 10 步。
- `SUBTASKS.csv` 03 行：验收与 notes 同步 role/RoleSpec 口径。

**关键设计取舍**：选「声明式 `RoleSpec`」而非「基类 + 六个子类」，理由是用户明确要求「所有模板是同一个模板格式」——子类化会把 role 差异写进类型系统，与「同一格式」相悖，也让「新增 role 不改结构」落空。Adams 的 `.tpl` 正是同一道理：只有一种文件格式，`MAJOR_ROLE` 只是一个字段。
## 2026-09-23 实施进度回填（主代理复核）

01/02/03 三个子任务在 2026-09-22 夜至 09-23 午间已实际实施完成（代码与测试均已落盘），但此前只有 `TODO.csv` 被回填为 DONE，三份 `PROGRESS.md` 的状态头与父级 `EPIC.md` / 本文件仍停留在「规划中／未开工」。本轮据磁盘事实与实跑证据纠正：

| 子任务 | 代码落点 | 复核证据 |
|---|---|---|
| 01 基线冻结 | `tasks/20260922-01-baseline/raw/` 四份实测记录 | 14 条命令实跑；本机 `737 passed／47 skipped／1 xfailed`（与计划记录的 `783/1/1` 为环境差异，同批 784 用例）；`--strict --final` 0；动态哈希 26/26；两包 build 0 |
| 02 统一副底座 | `joints/{__init__,table,validate}.py`、`tests/joints/**`、`cases/kc_quasi_static/contract.py`（拆截断）、`cases/{axle,vehicle}_dynamic.py`（改名收口） | `tests/joints`+`tests/templates` 58 passed；K/C parity 0；8 family accepted；legacy_surface_gate 0 |
| 03 模板数据模型 | `templates/{__init__,model,roles,registry,builtin}.py`、`tests/templates/**` | 同上；动态哈希 26/26 逐位一致（`e7407656...`） |

**复核方式**：主代理亲自复跑全部门禁（非采信子任务自证）——`build_axle_native.py`、`check_module_layering.py --strict --final`、`legacy_surface_gate.py --check`、`dynamic_hash_sentinel.py --check`、`kc_parity_check.py --check`、`case_parity_check.py`、全量 `pytest`（795 passed／47 skipped／1 xfailed，401s）、`ruff check .`、`ty check .`、`git diff --check`，全部退出 0；`git status` 在 `tests/data/**` 与 `layering_baseline.json` 上为空，即**未重录任何基线**。

**新增测试数核对**：795 − 737 = 58 = 02 的 `tests/joints` 27 + 03 的 `tests/templates` 31，与两任务自报数一致，无凭空计数。

**04 的实际断点**：`subsystems/` 已落 `types.py`／`capabilities.py`／`__init__.py`（含 `SubsystemOutput`／`merge_outputs`／`AssemblyCapabilities`／`capabilities_for`），`raw/assembly_snapshot.json` 已抓 K/C × `rack_fixed_to_chassis` 四种组合的现役产物；13 步 TODO 全部未开工，`build_front_axle` 尚未改造，`schema/model.py` 与 `schema/vehicle.py` 尚未新增字段。

### 2026-09-23 第二轮回填：04 主体、05、06 已完成（主代理复核）

上一节的「04 的实际断点」写的是 04 刚开工时的状态。此后 04 的主体（1-6、11-13）与 05、06 已实际实施并各自提交，父级状态头再次滞后。据 `git log`（`82fa179` 04、`eb69342` 05、`fb1ab3` 06）与磁盘事实纠正：

| 子任务 | 状态 | 代码落点 | 复核证据 |
|---|---|---|---|
| 04 拆六类子系统 | **9/13**（1-6、11-13 DONE；7-10 未落地） | `subsystems/{types,geometry,capabilities,chassis,suspension,steering,wheel}.py`、`preparation/assembly/front_axle.py` 改造 | 全量 814 passed／47 skipped／1 xfailed；`tests/subsystems` 19 passed；四种组合逐位一致（独立脚本 + 测试双重证据）；六条门禁全绿；未重录任何基线 |
| 05 模板实例化与 K/C 列激活 | **7/7 DONE** | `templates/instantiate.py`（`instantiate`／`activated_column`／`SubsystemInstance.with_mode`／`resolve_properties`），`builtin.py` 的 `PropertySlot.connections` | `tests/instantiation` 14 passed；全量 828 passed；六条门禁全绿。**与 SPEC 有一处已登记的偏离**：SPEC 要求把 C 模式占位衬套改成非零并据此重录 `kc_baseline` C 部分，实际不可执行——`scripts/kc_parity_check.py` 明说 C 快照是冻结 oracle 且 `--record` 已随退役的 Python 求解器移除；且 `tests/cases/kc_quasi_static/kc_fixtures.py` 已在同四个内点声明真实衬套，叠加会实质改变 C 解。落地方式改为「刚度来源由硬编码零矩阵改为模板 `bushing` 属性槽、内置模板默认值 0.0」，默认行为逐位不变、C 基线无需也无法重录。 |
| 06 弹性元件属性文件 | **9/9 DONE** | `properties/{__init__,load}.py`、`templates/instantiate.py` 的 `resolve_properties`、`tests/data/properties/{baseline_compliance,stiffer_compliance}.json` | `tests/properties` 18 passed；六条门禁全绿；`tests/data/kc_baseline/**` 无 diff（未重录）。同样有一处已登记偏离：SPEC 假定的注入路径 `instantiate(model, mode, properties=...)` 与 05 实际落地的 `instantiate(template, *, mode, properties)` 不同，06 改为在 `templates/` 侧新增 `resolve_properties(template, property_set)`（SPEC 允许写 `templates/**`）。 |

**观察到的系统性写法**：05、06 的 `PROGRESS.md` 正文里「本轮交付为规划，未写任何生产代码」「下一步：等 XX 完成后开工」等段落是**规划期文本未被清理**，与它们的 DONE 状态头矛盾。状态头与「验收对照」表是实施后的真实记录，以它们为准；本轮不逐份重写各子任务 PROGRESS 的正文（那属于改写历史记录），只在父级登记该事实。
### 2026-09-23 第三轮：04 与 07 收官（主代理复核）

**集成回归（04 + 07 + 08 内核侧三者合并后）**：`pytest packages/suspension_multibody/tests`（排除 `tests/performance`）实测 **968 passed, 1 skipped, 1 xfailed**，退出 0；`kc_parity_check --check` 0；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致；`case_parity_check` 8 families accepted；`check_module_layering --strict --final` 0。这组证据说明三者的改动**合起来**没有互相破坏，且 08 新增的轮胎质量字段在「文档未声明质量」时是惰性的（动态哈希与 K/C 快照均未变）。

04（13/13）与 07（9/9）已实施完成并由主代理**独立复跑**确认，非采信子任务自证：

| 子任务 | 独立复核命令 | 结果 |
|---|---|---|
| 04 | `pytest tests/subsystems` | 42 passed（含新增 `test_brake_subsystem` 7、`test_drive_subsystem` 7、`test_torque_role_is_replaceable` 5、`test_availability_matrix` 4） |
| 04 | `pytest tests/schema/test_vehicle.py` | 8 passed（含「显式给出四个制动参数 vs 保持默认」的 `model_dump` 逐字节相等断言） |
| 07 | `pytest tests/outputs tests/metrics` | 59 passed |
| 07 | `legacy_surface_gate.py --check` | 退出 0，`legacy_module_import` 仍 8 条（未新增） |
| 07 | `ruff check`（产出与测试）／`ty check`（`outputs/`） | 全部通过 |
| 共同 | `git status --porcelain tests/data layering_baseline.json` | 空（未重录任何基线） |

**04 的关键交付**：`subsystems/brake.py` 与 `subsystems/drive.py` 的简化模板（`parts=()`，0 刚体）；`schema/vehicle.py` 的 `DrivelineSpec` 补 `brake_mu`/`piston_area`/`effective_piston_radius`/`max_brake_value` 四个 `Field(exclude=True)`；制动幅值按仓库内冻结的 `.adm`（`artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm` 的 `SFORCE/31-34`、`VARIABLE/277-280`）形状核算，`demand=1.0` 时前轴 17400／后轴 11600；可替换性硬门用同一 `build` 对象跑两个模板并用 AST 走查装配路径无分支，且**自证分支检测器可失败**。

**07 的关键交付**：`outputs/{declarations,derived,builtin}.py` 三层；27 个 legacy 函数 27/27 登记（与一份独立写死的名单核对）；逐值一致覆盖 12（力+时间）+8（诊断）+11（轮荷）+10（整车）+15（K&C）；旁路用 AST 双向自证。两处与 SPEC 的偏差已登记在 07 的 PROGRESS（键存在性改为声明；`status`/`reason` 类可用性事实不重述），并如实标注 K&C 15 项中 12 项恰为 0.0、故另加两条不依赖 legacy 对比的符号测试补强。

**08 的当前断点**：内核侧（契约 schema + `Tire` 结构体 + 解析 + 安装点）已落盘；求解器的惯量耦合未完成，native dll 正在重建，故依赖 native 的 multibody 测试此刻报「镜像过期」——这是预期中间态，不是回归。08 在途期间不得把 08 的落点标为 DONE。

## 下一步

08 已收官（9/9）。可启动 09（study 合并，依赖 06 与 08，现已满足）与 10（试验台正交，依赖 07 与 09）。12 的端到端判据 (d) 已由 07 提供实现。

### 2026-09-23 第四轮：08 收官（08b 由主代理实施）

08b（求解器显式耦合轮胎惯量）此前由三个子代理接手，分别耗尽预算或连接中断、零落盘；主代理接手实施并完成。**08 是本轮唯一的关键路径**（09/10/11 都依赖它）。

设计：新增模型层「有效惯性」缓存，把「轮胎也拥有惯量」在构建时求和一次，热循环只读缓存（`Model::body_effective_mass` / `body_effective_inertia_body` + `compute_effective_body_inertia`）。**14 个消费点**全部改读访问器——残余惯性力/力矩、newton 的质量与惯量块与 `-j/m`、积分器质量矩阵、`mass_inverse_of_jt_mu` 及其方向导数、**重力**、陀螺项、动能、静态 `total_mass`。设计要点是「某体没有带质量的轮胎时，有效值与原始值逐位相等（加零）」，这使零质量路径**逐字节不变**。

主代理实跑的全部门禁（未重录任何基线）：`tests/tire_mass` 4 passed（质量守恒 **逐位相等、max|diff|=0.0、不使用容差**；开/关轮胎质量差 21.7 证明耦合生效）；全量 multibody **974 passed／1 skipped／1 xfailed**；契约 27、内核 21、架构 91；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致；`kc_parity_check` 0；`case_parity_check` 8 families accepted；`--strict --final` 0；两包 `uv build` 0；ABI 版本常量 15/30/1/1 未变；`tests/data` 与 `layering_baseline.json` 无 diff；ruff/ty 全树通过。

**登记的已知限制（有意，非漏做）**：轮胎质量若不在其体原点（`tire.center != 0` 且 `mass != 0`），`compute_effective_body_inertia` 显式报错点名（状态码 2，`"tire inertia: "` 前缀）。原因：残余把体原点当质心、无臂项；偏心质量需要额外的平动/转动耦合项，那是**新物理**而非「换归属」，必须有独立验收。作者层实际发射的 `center_local` 为 `[0,0,0]`，故当前全部路径都在支持范围内。
