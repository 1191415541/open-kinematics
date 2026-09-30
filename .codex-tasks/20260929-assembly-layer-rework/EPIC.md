# Epic：物理装配执行层改造（总成文件驱动的通用装配引擎）

- 任务编号：20260929-assembly-layer-rework
- 创建日期：2026-09-29
- 形态：epic
- 状态：**已结项（2026-09-29）**。`SUBTASKS.csv` 的 01/02/03/04/04b/05/06/07 全部 `DONE`，各子任务 `TODO.csv` 全部 `DONE`；Done-When (a)–(g) 逐条实跑通过（(b) 附已登记的点 label 拼写残余差异），G1–G6 全部有证据。最终验收：全量 1531 passed / 1 skipped / 1 xfailed、architecture 147、contracts+kernel 65、ruff/ty 全过、三架构门 OK、`dynamic_hash_sentinel --check` 26 artifact 逐字节一致（未重录）、K/C probe + `--actual-dir` parity OK、case_parity 8 families、kc_perf 在预算内、`snapshot.py --check` 零差异、`approved_deltas.json` 全程 `[]`；证据见 `tasks/20260929-07-acceptance/raw/final_validation.md` 与本目录 `PROGRESS.md`
- 真源：本目录 `SUBTASKS.csv`；子任务相对路径均相对此 Epic 目录解析。

## 状态与原始需求

用户本轮给出一份架构评审：**模板 / 子系统 / 总成文件三层抽象已经是数据驱动的，但底层物理装配执行层（`vehicle_assembly.py`、`si_assembly.py`）仍残留早期单轴硬编码的演化痕迹**。原文按痛点逐条摘录（R1–R5 为需求原文要点，P 为其中的改造建议）：

- **R1**：「装配执行层未直接消费总成文件，而是通过硬编码的 `VehicleModel` 中转」——`compose_vehicle_runtime(model: VehicleModel)` 强制要求历史遗留 Python 类；`VehicleModel` 写死 `front_axle` / `rear_axle`；`authoring` 侧不得不写胶水把总成文件里的子系统拼成两轴对象再传进去。「导致的后果：无法支持 3 轴卡车、拖挂铰接车、单轮/三轮试验平台。」
  - **P1**：装配引擎改为**直接遍历总成文件声明的子系统清单**，按 `placement_role` 动态实例化并打前缀，不再依赖 `front_axle` / `rear_axle` 属性。
- **R2**：「跨子系统连接仍靠『隐式字符串替换』，缺少显式的『通讯器/接口（Communicators/Ports）』匹配机制」——`body_map = {old: model.chassis.name if old in ("chassis", "ground") else f"{prefix}{old}" ...}` 这类"只要叫 ground/chassis 就换成车身名"的黑盒规则；「用户在总成文件里看不出悬架究竟是插在车身的哪个插座上」。
  - **P2**：把 `modeling/ports.py` 提升为主装配逻辑；悬架只暴露 `mount_port`，装配层读总成文件里的接口映射完成真正插接，消除 `body_map` 的猜测。
- **R3**：「轮端刚体与轮胎的生命周期不对称（单轴生成轮胎，整车先删后建）」——单轴装配不生成车轮刚体却直接绑 `VerticalTireElement`；整车装配调用单轴后把单轴轮胎力元全部删掉，再另建 4 个 `wheel` 刚体重新挂轮胎。「单轴与整车的车轮模型无法共用同一个子系统文件。」
  - **P3**：车轮子系统统一为「刚体 + 轮胎」，单轴与整车以相同方式挂到悬架轮毂；**唯一差异在试验台**（K 台刚性夹持车轮、轮胎不起作用；C 垫板机与轮胎接触、轮胎弹性生效）；「不要在装配阶段删建零件，而应该在试验台施力阶段决定激活哪种受力」。
- **R4**：「对称性写死在底层，阻碍了不对称机构与模块复用」——`suspension.py` 与 `si_assembly.py` 深度绑定 `_SIDES = ("L", "R")`；用户诉求是**不对称悬架**（NASCAR 椭圆赛道车、特种工程车）与**单侧独立子系统文件**（`front_suspension_L.subsystem.json` / `_R.subsystem.json` 分别引用）。
  - **P4**：把镜像提升为**可选配置项**；子系统底层按单侧/单单元编译，总成文件决定"引用对称镜像模板"还是"分别引用左右独立文件"。
- **R5**：「试验台挂接（`rig_link.py`）对被测总成内部构件的篡改」——`_reown_tires` 把原本属于悬架/转向节的轮胎力元所有者转让给试验台的虚拟夹具刚体（carrier）。
  - **P5**：试验台是**外加约束与外加载荷**，被测总成 Runtime 应当**不可变**；试验台与总成只通过轮端接触点（contact patch）或轮心固定夹具建立约束，禁止跨模块篡改零件所有权。

**本轮交付边界（用户明确）**：「根据痛点和改造建议制定改造计划」——只交付规划，不实施代码。

## 用户裁决登记（D1–D7，2026-09-29 已全部裁决）

以下 7 项是计划里**方向性**的选择点。D1–D3 由用户于 2026-09-29 明确裁决，D4–D7 按本计划建议无异议采纳；「建议（理由）」一列保留原建议，裁决结论写在该列的行首，裁决原文要点见 `PROGRESS.md` 的第二轮记录。

| 编号 | 问题 | 建议（理由） | 影响 |
|---|---|---|---|
| D1 | 装配引擎的入口形态 | **已裁决（用户）**：**严禁原地改 `VehicleModel` schema**；新增**文档驱动装配器**（直接消费 `AssemblyDocument`）作为未来唯一的装配主管道，原生支持任意轴数与子系统拓扑；`VehicleModel` 降级为**向下兼容适配器**（绞杀者模式）。依据：原地改 schema 会让 `model_dump` 漂移、7 组合字节一致性失效，连锁瘫痪既有算例与 API | 02/03 |
| D2 | R3/P3 与既有裁决 D9 的冲突：单轴侧车轮体是否收回车轮子系统？ | **已裁决（用户）：不反转 D9，严禁重录 `kc_baseline`**。采用**文件层统一 + 装配期刚性凝结**：单轴与整车可引用**同一份** `wheel.subsystem.json`；装配**单轴侧**（`RigSpec.supplies_wheels=True` 的读数：K/C 与轴动态）时把车轮刚体与轮毂刚体在内存中**刚性凝结（condensation）**，求解自由度拓扑与既有 K/C 基线逐位不变。**判据已于 2026-09-29 复核**：文件层统一后原拟判据「车轮子系统本次不产出车轮刚体」不再成立，改按试验台/请求声明判定（`supplies_wheels=True` 即单轴侧凝结、整车侧不凝结），单轴动态侧同样凝结，故 `dynamic_hash_baseline` 的 13 个轴侧用例必须继续逐字节不变。依据：AGENTS.md 第 7 节禁止重录 `kc_baseline`（"物理结果逐位不变是物理没有变的唯一证据"）；Adams `acar_gs_front.asy` 单轴装配同样不带独立车轮刚体，轮端载荷直接作用在轮毂/转向节上 | 04 |
| D3 | 试验台契约是否反转 | **已裁决（用户）：确认反转**。删除 `rig_link.py` 的所有权篡改（`_reown_tires`），断言改为验证**被测总成不可变（immutability）**。依据：断言内部实现细节是反模式；试验台是外部激励与夹持，只应通过外加约束或接触力与被测物交互；作用线与力路径不变时数学物理严格等价 | 05 |
| D4 | 接口配对粒度 | **已按建议采纳**：复用并提升 `connections/matcher.py`（explicit > role/capabilities/labels），并把 bindings 真正转成两点间的运动副/衬套 | 02 |
| D5 | 对称默认值 | **已按建议采纳**：镜像仍为默认（文件只写一侧、装配建两侧），不对称与单侧独立文件走显式声明 | 06 |
| D6 | 3 轴/拖挂/单轮三轮验收边界 | **已按建议采纳**：三者都进硬判据；拖挂由同一机制（端口配对 + 现有副类型的铰接副）承接 | 07 |
| D7 | 基线重录口径 | **已按建议采纳，并被 D2 收紧**：`kc_baseline` **不得重录**（D2 用凝结保住它）；其余基线若确需变化，必须逐项登记并先给出独立于结果字节的物理等价判据（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径） | 全 Epic |

## Goal

**G1（文件即真源）**：装配执行层以总成文件声明的**子系统条目清单**为唯一驱动。装配路径上不再有"前轴/后轴"这一对硬编码属性，也不再有"整车必然四角四轮"的隐含假设；3 个及以上悬挂子系统、非 front/rear 的放置、单侧/单轮总成都能装配。判据：(a) `grep -n "front_axle\|rear_axle"` 在装配路径（`subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`、`preparation/`）无命中；(b) **单轮（单侧悬架）与三轮（两悬架 + 一个单侧）各有一个最小装配用例跑通**，不只是"能构造"。

**G2（显式接口配对）**：跨子系统连接只通过**显式接口配对**建立——子系统声明它提供什么接口（port）、需要什么接口（requirement），总成文件写出配对（留空时按角色/能力唯一匹配）。**终局硬判据**：装配层不再有"名字等于 `chassis`/`ground` 就改写成车身"这类规则（`grep` 在装配路径无命中）；车身侧刚体改名（如 `subframe`）后同一份总成文件仍能装配；配对歧义与悬空配对报错点名。过渡期的兼容开关必须在 03 之前删除并登记。

**G3（统一轮端：文件统一 + 求解拓扑按读数凝结）**：单轴与整车**引用同一份车轮子系统文件**（车轮刚体 + 轮胎声明），装配阶段不再"删掉单轴轮胎、再建车轮刚体"（`vehicle_assembly.py` 的类型过滤删除）。**求解拓扑按读数决定**：单轴 K/C 试验台是轮心驱动工况，装配期把车轮刚体与轮毂刚体**刚性凝结**，自由度拓扑与 `kc_baseline` 逐位不变（D2）；整车侧保持独立车轮刚体与 native 轮胎。K/C 与动态的差异由**读数**决定，不由装配阶段增删零件实现。

**G4（试验台非侵入）**：接入试验台前后，**被测总成 Runtime 的实体逐项一致**（bodies/points/constraints/elements 的集合与名字）；试验台只贡献**外加约束与外加载荷**（轮心夹具、接触点、作动器），不转让、不改写被测零件的所有权。

**G5（可选对称）**：镜像（mirroring）成为**可声明选项**。子系统文件可以只描述一侧（默认镜像），也可以左右各自成文件被总成分别引用；两种写法装配出的实体集合一致。默认路径仍走镜像，故现有组合产物不变。

**G6（零回归与受控变化）**：**04 不得改变任何既有产物**——单轴侧（K/C 与轴动态）的凝结必须让自由度拓扑与 `kc_baseline`、`dynamic_hash_baseline` 的轴侧用例逐位不变，这是 D2 的硬门。**05 允许改变产物**（试验台不再改写被测实体），但必须：(i) 登记变化（文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据）并写入 01 交付的 `raw/approved_deltas.json`——登记项必须与差异**四项精确匹配**（`product`/`pointer`/`before`/`after`）并带 `reason`/`registered_by`/`evidence`，`registered_by` 必须是 **05**（只有 05 被允许改变既有产物；登记缺字段或归属不符即 `--check` 退出 4，未命中登记的差异退出 1）；(ii) 先给出独立于结果字节的物理等价判据并列出允许差异；(iii) 数值门与快速集重跑通过。静态门与快速集在每次落地后必须重新通过。

## Non-Goals

- **不改 C++ 内核**：本轮只改 Python 装配层与作者层；内核 ABI 七符号与版本常量（15/30/1/1）不变，`check_module_layering.py --strict --final` 保持绿。
- **不实现具体车型物理**：3 轴与拖挂只交付**机制 + 最小端到端用例**——3 悬挂整车与「牵引车 + 挂车」铰接各一个，由 03 交付、07 复验（D6 把它们都定为硬判据）。铰接副**必须先用现有副类型实现**；只有实测证明现有副类型不可行时，才登记为内核范围并提请用户裁决，不得静默降级为「登记即收口」。
- **不改轮胎力律与求解器数值路径**：只改"轮胎/车轮由谁产出、谁承载"，不改本构。
- **不删除 `VehicleModel`**：降级为适配器，历史读取与既有调用者保持可用。
- **不重构输出/指标/试验台输出声明**：属 20260922 Epic 的范围。
- **不改模板数据模型**（`templates/model.py` 的 parts/connections/属性槽）：本轮只改装配执行层如何使用它；接口配对用的是模板已声明的 `ports`。

## 事实与修正（制定计划前实测）

以下每条都带 file:line 与原文，**先核实再写计划**；带「修正」的条目是用户陈述与代码现状不完全一致之处，计划按代码现状写。

**F1（R1 核实 ✓）** `subsystems/vehicle_assembly.py:122` 的入口签名就是 `compose_vehicle_runtime(model: VehicleModel, mode, request)`；`:184-187` 把两轴硬编码成 `("front", model.front_axle, "front_")` / `("rear", model.rear_axle, "rear_")`（两轴条目在 `:185-186`）。`schema/vehicle.py:244` 的 `VehicleModel` 字段就是 `front_axle` / `rear_axle`（`:252-253`），且 `:262-265` 的校验器要求 `wheels` 恰为四个角（`WheelSpec.name` 是 `Literal["front_left", ...]`，`:46`）。文件→模型的胶水是 `authoring/solver.py:632 file_axles_from`（在 `:622` 被 `assembly_request_for` 使用）与 `authoring/vehicle.py:82 vehicle_model_from`。

**F2（R1 补充事实）** 「3 轴」今天**在更早的一层就被拒绝**：`connections/policy.py:202-217` 的 `full_vehicle` 规则写死 `role_counts={"suspension": 2, "chassis": 1, "steering": 1, "brake": 1, "drive": 1}` 与 `required_placements={("suspension","front"), ("suspension","rear")}`，由 `check_assembly_shape`（`:252-296`）逐条执行；`PLACEMENT_ROLES`（`authoring/documents.py:62-64`）与 `assembly.schema.json` 的 `assembly_kind` 枚举只有 `suspension_axle` / `full_vehicle`。**契约 schema 本身就是一道前置墙**：`packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json` 的 `:18`（条目层 `additionalProperties: false`）不允许条目新增字段、`:23` 的 placement 枚举不含 `middle`、`:46` 的整车 `required` 只列 `chassis`/`wheels`/`steering`（**单车身，拖挂的两车身总成在此被挡**）、`:50` 把整车轮数限定为四——这四处都在装配逻辑之前生效。全仓无 `triaxle|tandem|third_axle` 命中。**所以 G1 同时要动"文档形状规则"与契约 schema，不只是装配函数。**

**F3（R1 的另一面）** `authoring/vehicle.py:82 vehicle_model_from`（文件→`VehicleModel` 的唯一入口）**在生产代码里没有调用者**，只在 `tests/authoring/test_vehicle_assembly_documents.py` 闭环。即"文件驱动的整车"今天还不是生产路径——这既是 R1 的成因，也是本 Epic 的落地机会（新入口不必与既有生产调用者兼容）。

**F4（R2 核实 ✓ + 修正）** 用户引用的 `body_map` 在 `subsystems/vehicle_assembly.py:212-228`；**同一条规则散落在 4 处**：`preparation/vehicle_dynamic.py:373`（`name = model.chassis.name if body.name == "chassis" else f"{prefix}{body.name}"`）、`subsystems/vehicle_parts.py:172-173`（焊合根命名 `if "chassis" in component: return "chassis"`）、`subsystems/vehicle_parts.py:414`（`referenced: set[str] = {"chassis"}`）；配套白名单两处：`authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES`、`subsystems/suspension.py:255 _FOREIGN_STEMS`。
**修正**：匹配机制**已经存在**——`connections/matcher.py:102 match_requirements(requirements, candidates, *, explicit=...)`（explicit 显式映射优先；0 候选按 required/optional 分别报错或记录消失；多候选抛 `AmbiguousBindingError`），已被 `subsystems/composition.py:258`（子系统贡献之间）与 `:347`（`_bind_rig`）调用。**但它今天只服务"试验台绑定"与"轮心需求"，且 `composition.py:302-305` 只产出 bindings 记录、不据此建立跨子系统的物理插接**。所以 P2 的落点不只是"把 matcher 接上"，而是：把总成文件的配对写进文档 + 把 bindings 真正转成两点间的运动副/衬套。
另：`modeling/ports.py` / `templates/ports.py` 的 `ChannelPort` / `declaration_to_port` / `Assembly.all_ports()` **无外部消费者**（预备性代码），落地时要决定是启用还是删除。

**F5（R3 核实 ✓ + 修正）** 「单轴生成轮胎」核实：`subsystems/wheel.py:33-41` 的 `build()` 在单轴侧**不产出任何 body**，`tires()`（`:87-111`）把 `model.tires` 声明成 `ResolvedElement`，挂在"声明 `wheel_center` 点的那个 body"上（内置模板是 `wheel_hub_{side}`，否则退回 `upright_{side}`）；真正的 `VerticalTireElement` 由 `subsystems/element_build.py:136-144` 构造，字段 `wheel_body` / `wheel_center_local`。「整车删掉再建」核实：`subsystems/vehicle_assembly.py:233-240` 按类型过滤掉单轴的 `VerticalTireElement`，车轮刚体与挂接由 `subsystems/vehicle_parts.py:458 _add_wheel`（车轮体 `:510-515`、挂接 `:525-541`：mount 是轮毂时 weld、否则 revolute）建立。

**F5b（D2 的可用先例，实测）** 仓库里已有两处"把一个刚体凝结进另一个刚体"的现成实现，04 应复用而不是新造：`subsystems/vehicle_parts.py` 的 `_merge_fixed_wheel`（`:572-610`；`wheel.mount_joint_kind == "fixed"` 时按复合质量属性把车轮的质量/惯量并入 mount body 且不建独立车轮体，分支在 `:482-508`）与同文件的 `_fuse_welded_bodies`（`:133+`，由 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复）。注意 20260921 Epic 的 A3 裁决把**整车侧**生产路径改为"不凝聚、weld 交内核 `fixed` 关节"，那是**另一处**的取舍，与本 Epic 单轴 K/C 侧要凝结不冲突，但必须在 04 的 PROGRESS 里点名两者关系。凝结的**检测判据**（"轮心驱动工况"）由 04 的 SPEC 定；现成可用的判据是"车轮子系统在本次装配中不产出车轮刚体"（单轴路径，见 F5）。
**修正**：整车**不会**再挂一个 `VerticalTireElement`——整车轮胎走 **native tire ABI**（`preparation/vehicle_dynamic.py:1009-1025` 用 `assembly.wheel_body_names[...]` 定位体），且 `:900-903` 明确拒绝 `VerticalTireElement`。所以真正的差异不是"轮胎力元被删又建"，而是**同一种物理在两条路径上用两种表示**（垂向力元 vs native 轮胎）。这恰好支持 P3 的落法：统一**实体**（车轮刚体 + 轮胎声明由 wheel 子系统产出），把"哪种受力激活"交给读数——`cases/kc_quasi_static/contract.py` 的 `TIRE_MODES={"pad"}` / `ELASTIC_MODES` 已经是这个思路的现成先例（K/C 读数已经决定轮胎是否进文档）。

**F6（R4 核实 ✓ + 修正）** `_SIDES` 今天在 `subsystems/si_assembly.py:56`（被 `:117`、`:189`、`:208`–`:228`、`:286`、`:353` 消费）；**`subsystems/suspension.py` 今天已无此符号**（早先的记录已过期）。固定双侧展开另有 `subsystems/wheel.py:114-116 sides()`（返回 `types.SIDES`）与 `subsystems/element_build.py:206`、`:213` 的 `for side in ("L","R")`——这三处是 06 必须收口的消费点。
**修正**：镜像**不是文档开关**——`subsystem.schema.json` / `template.schema.json` 里没有 `mirror`/`mirrored` 字段，镜像是纯代码行为（`mirror_hardpoints` / `side_hardpoints` / `context.mirror`）。当前契约是「**文件只写一侧，装配建两侧**」（`tests/authoring/test_vehicle_assembly_documents.py:202-214`）。"左右分别成文件"今天会被 F2 的 `role_counts` 拒绝（两个 `suspension` 且 placement 非 front/rear）。对称性测试只有 `tests/model/test_symmetry.py`（12 行），无任何不对称/单侧用例。

**F7（R5 核实 ✓ + 冲突登记）** `subsystems/rig_link.py` 的 `_reown_tires`（`:315-348`）把轮胎力元的所有者改写成 `wheel_carrier_{side}` 并把 `wheel_center_local` 清零；`merge_rig_link`（`:243-268`）用它替换原元素；carrier 是自由体、由 `WeldJoint` 焊到"声明 wheel_center 的那个 body"（`:211-218`）。试验台是否供轮由 `rigs/rig.py:73-110` 的 `RigSpec.supplies_wheels` 声明（`kc_quasi_static` 为 True）。实体注入开关 `SUSPENSION_MULTIBODY_RIG_ENTITIES`（`RIG_ENTITIES_SWITCH` 定义在 `subsystems/si_assembly.py:521`，`_rig_entities_enabled()` 紧随其后，默认开），应用点在 `:467-482`。
**冲突**：`tests/subsystems/test_rig_link.py:172-199`（`test_the_tire_moves_to_the_bench_wheel`）**断言的是相反的事实**——「the tire must be re-owned, not duplicated」且 `all(body.startswith("wheel_carrier_"))`。本 Epic 落地 R5 必须反转这条契约（D3）。
另：除 `link_wheel_supplying_rig` 外，还会改写实体的开关有两处——`SUSPENSION_MULTIBODY_CONDENSE_WELDS`（`vehicle_parts.py:113-130`）与 `SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES`（`:408-455`），以及文档层的 `cases/vehicle_kc.py:132-137`（删 `steering_actuator`、追加驱动关节）。

**F8（门禁与基线现状，本 Epic 的 01 起点；本轮实测）** 快速集 `1015 passed / 1 xfailed`；`tests/cases` 100 passed；`tests/architecture` 147 passed；`tests/adams` 160 passed / 47 skipped（skip 全是"Adams 参考 artifacts 不在本副本"的环境跳过）；`suspension_kernel` + `suspension_contracts` 60 passed；`ruff` / `ty` / `legacy_surface_gate` / `check_module_layering --strict --final` / `check_composable_release` 全绿；数值门 3/3 绿（`dynamic_hash_sentinel` 26 artifacts 逐字节一致、`case_parity_check` 8 families accepted、`kc_perf_gate` 在预算内），其中 `vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 是**经用户授权**在 2026-09-29 重录的两项。

**F9（可用先例）** 本仓库刚落地过一个同性质的改造：**K/C 读数的轮毂自旋锁**——装配体保留 `RevoluteJoint`，而**读数文档**把它写成刚性连接（`cases/kc_quasi_static/contract.py::_quasi_static_joints`）。这就是 P3「由读数决定激活哪种受力、不在装配阶段增删零件」的现成范式，04 直接沿用同一分层。

## 目标结构（改造后）

```text
总成文件 (.assembly.json)
  ├── assembly_kind + 子系统条目清单（N 个，每条 ref/functional_role/placement_role/接口配对/overrides）
  └── 接口配对段（谁提供哪个 port、谁需要哪个 port，可显式可留空由装配层唯一匹配）
        ↓ 解析为统一总成图谱
通用装配引擎 UniversalAssembler
  ├── 1. 逐条实例化子系统（无前后轴硬编码；按 placement_role 打前缀；镜像与否按条目声明）
  ├── 2. 显式接口配对（match_requirements：explicit > role/capabilities/labels；歧义即报错）
  └── 3. 统一轮端（wheel 子系统产出车轮刚体 + 轮胎声明，挂到悬架轮毂）
        ↓
被测总成 Runtime（不可变：装配完成后不再被试验台改写）
        ↓ 外加约束 / 外加载荷（只增不改）
试验台 Rig（轮心夹具 / 接触点 / 作动器；供轮与否由 RigSpec.supplies_wheels 声明）
        ↓
study（K / C / 准静态 / 动态）→ 决定激活哪种受力（垂向力元 vs native 轮胎；joint 列 vs bushing 列）
        ↓
结果（最小单位输出）→ 衍生输出
```

## 子任务分解与依赖

见 `SUBTASKS.csv`。串行主线与并行支线：

```text
01 冻结现状事实与判据（含"现有产物逐位不变"的字符化测试）
  ↓
02 显式接口配对落地（把 match_requirements 提升为跨子系统插接的唯一通道 + 文档配对段）
  ↓
03 通用装配引擎（条目清单驱动；VehicleModel 降级为适配器；文档形状规则放开 N 轴）
  ├→ 04 轮端生命周期统一（wheel 子系统产出车轮体+轮胎；读数决定激活）
  │     ↓
  │   05 试验台非侵入（试验台只外加约束/载荷；反转 test_rig_link 契约）
  └→ 06 可选对称（镜像成为条目声明；左右独立文件与镜像两种写法等价）
                              ↓
                          07 终局独立验收
```

**并行与写范围约束**：
- 04 与 06 都触及 `subsystems/si_assembly.py` 与 `subsystems/suspension.py` → **不得并行**；06 的 `depends_on` 设为 05，调度上必然后置。
- 03 与 04 都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py` → 必须串行（04 依赖 03）。
- **契约 schema 是 02/03 的共同前置**：`packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json` 今天禁止新增配对字段、不接受 `middle` 放置、把整车轮数限定为四、整车 `required` 只列一个 `chassis`；02 先改**配对段**、03 再改**放置段 + 轮数段 + 整车 `required`（多车身）段**，不得并行（`suspension_contracts` 是独立包，有自己的测试）。
- 02 的写范围（`connections/`、`modeling/ports.py`、`authoring/documents.py`）与 03 的写范围（`subsystems/vehicle_assembly.py`、`authoring/vehicle.py`、`connections/policy.py`）不相交，但 03 依赖 02 定下的配对契约，故仍串行。
- 05 单独拥有 `subsystems/rig_link.py` 与 `rigs/`；任何其它子任务不得改这两个路径。
- **段级划分（2026-09-29 复核后定）**：`preparation/`（含 `preparation/vehicle_dynamic.py` 的 7 处 `model.front_axle`/`model.rear_axle` 访问，见 `:188`、`:207`、`:210`、`:341`、`:368-369`）整体归 03——G1(a) 按字面口径要求整个 `preparation/` 目录 `grep "front_axle\|rear_axle"` 零命中，故 `VehicleModel` 适配器（`VehicleModel` → 装配图谱）必须落在 `preparation/` 之外，`preparation/` 只读图谱；04 在 `preparation/vehicle_dynamic.py` 上只改**轮端内容段**（`:900-903` 的拒收与 `:1009-1025` 的轮胎构建），且必须在 03 之后（同一文件，串行）。`subsystems/wheel.py` 与 `subsystems/element_build.py` 由 04 拥有**轮端实体段**、由 06 拥有**单侧展开段**（`wheel.py:114-116 sides()`、`element_build.py:206/213` 的 `for side in ("L","R")`）。同一文件被两行拥有时必须串行，且各自只改本段。
- **`authoring/documents.py` 的分段归属**：02 拥有**配对段**、03 拥有**放置与角色枚举段**（`PLACEMENT_ROLES` 与 `:139` 的放置校验——只改契约 schema 不够，文档读取这一层会先拒掉 `middle`）、06 拥有**子系统文档的可选对称声明段**；按 02 → 03 → 06 串行，各自只改本段。

## 冻结约束

- **ABI 七符号与版本常量（15/30/1/1）不变**；`packages/suspension_kernel/scripts/check_module_layering.py --strict --final` 保持 0 环；本轮不新增 C++ 模块、不新增反向边。
- **`model_dump(mode="json")` 的产物不得被改变**：`api.py:116`、`:284` 用它算 `model_hash`。因此 G1 走"新增文档驱动入口 + `VehicleModel` 适配器"，不改 `VehicleModel` 的字段形状（D1）。
- **现有 7 个 (总成, 试验台) 组合的默认行为保持可用**；凡改变产物的子任务必须显式登记。
- **不得新增 skip/xfail**；`tests/adams` 的 47 个环境 skip 是既有的，不得增长。
- **基线重录按 D7（并被 D2 收紧）**：`kc_baseline` **不得重录**——单轴 K/C 的凝结必须让它逐位不变；其余基线若确需变化，逐项登记「文件 + 步骤 + 前后值 + 等价性判定」并先给出独立于结果字节的物理等价判据（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径）；质量与质心相同不足以证明等价；禁止先改基线让门变绿。
- **每步落地后必须重跑**：`just check-fast`（ruff/ty/三个架构门/快速集/另两包）与数值门三项；结构改动后必须重跑 `tests/architecture`。
- **装配层不得出现按名字猜身份的规则**：`"chassis"` / `"ground"` / 前缀拼接这类字符串改写是 R2 的消除对象；替代品是端口配对（G2）。过渡期的兼容路径必须以显式开关存在，并在各自所属行（02 自有路径 / 03 / 04）内删除并登记；终局判据是 `grep` 在装配路径（`subsystems/`、`preparation/`、`connections/`、`authoring/`）无命中，由 07 复验。
- **不得把"简化/试验台专用"的分支写进 role 接口**（沿用 20260922 Epic 的 G2b 口径）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。

## 验证协议

**01（冻结现状）**：实测并落盘五份证据——(a) 现有 7 个组合的装配产物快照（每个供轮试验台各自成键、整车侧共用 `vehicle`；字段含 bodies/points/constraints/ideal_constraints/elements/connections 的集合与名字），作为后续每步的"逐位不变"判据；(b) 5 条痛点的锚点清单（file:line + 原文，即 F1–F7 的复核）；(c) 当前门禁与数值门的实测值与退出码；(d) "3 轴会被哪一层拒绝"的负例实测（构造一份 3 悬挂的总成文件，记录 `check_assembly_shape` 的报错原文）；(e) **快照的确定性与比对口径**——快照必须可重复生成（载荷里不含生成时间等易变字段，生成时间只写进 `raw/` 的说明文件；序列化用排序键、集合转排序列表），且 `snapshot.py --check` 采用「未变化部分逐项相等 + 已登记差异」口径：差异必须被 `raw/approved_deltas.json` 的某条登记**四项精确匹配**（没有前缀覆盖），登记项必须带 `reason`/`registered_by`/`evidence` 且 `registered_by` 为 05，登记不合法即退出 4、未命中即退出 1；`_meta.coverage_boundary` 写明整车侧 5 个 rig 的试验台绑定不在本快照覆盖范围内。**禁止依据旧任务 DONE 结论**。

**02（接口配对）**：`match_requirements` 成为跨子系统插接的唯一通道，且 bindings 真正转成两点间的运动副/衬套（不是只产出记录）——(a) 构造一份显式配对的悬架↔车身总成文件，装配产物与今天的 `body_map` 结果**逐项一致**（体/点/约束集合与名字，含约束类型与端点）；(b) 把车身侧刚体改名为 `subframe`（不在任何硬编码集合里）后，**同一份配对文件仍能装配**；(c) 配对缺失且候选不唯一时抛 `AmbiguousBindingError`，配对指向不存在的 port 时报错点名；(d) **本行自有路径**（`connections/`、`subsystems/composition.py`、`authoring/documents.py`、`modeling/ports.py`、`templates/ports.py`）里的 `"chassis"` / `"ground"` 字符串改写规则全部消除，过渡开关已删除并登记——另三处锚点（`subsystems/vehicle_assembly.py:212-228`、`subsystems/vehicle_parts.py:172-173` 与 `:414`、`preparation/vehicle_dynamic.py:373`）归 03，全装配路径的终局 `grep` 判据由 03 收口、07 复验；(e) 配对段写进 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`，且 `suspension_contracts` 包自己的测试同步更新通过。

**03（通用装配引擎）**：(a) 一份 **3 悬挂子系统**（front/middle/rear 放置）的整车总成文件装配成功并跑通一次 study；(b) `grep "front_axle\|rear_axle"` 在装配路径（`subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`）**与整个 `preparation/` 目录**无命中（G1(a) 字面口径；适配器落在 `preparation/` 之外，`preparation/` 只读图谱）；(c) `VehicleModel` 经适配器路径装配出的产物与 01 快照**逐项一致**（这是零回归的硬门）；(d) `check_assembly_shape` 对 N 轴、非 front/rear 放置的规则改为数据驱动，且有正例（3 轴）与负例（缺放置、重复放置）各自失败/成功的断言；(e) `assembly.schema.json` 接受 `middle` 放置、N 轴/非四轮声明**与整车 `required` 的多车身形态**，且 `authoring/documents.py` 的放置与角色枚举段同步放开（只改 schema 会在文档读取层被拒；契约包测试同步）；(f) **拖挂铰接总成**：同一总成内两个车身侧体（牵引车 + 挂车）经**现有副类型**的铰接副连接并跑通一次——这要求形状规则的角色数规则与契约 schema 的整车 `required`（`:46`，今天只列一个 `chassis`）支持多车身；(g) 单轮（单侧悬架）与三轮（两悬架 + 一个单侧）的最小装配用例**归 06 交付**（单侧展开机制在 06 的写范围），07 复验；(h) **整车动态基线不得漂移**：`preparation/` 图谱化之后，`dynamic_hash_sentinel.py --check` 的 26 个 artifact（含整车侧 8 例与轴侧 13 例）必须仍逐字节一致，本轮不得重录。

**04（统一轮端）**：(a) 单轴总成与整车总成都**读入同一份** wheel 子系统文件（文件读取链打通；今天 `subsystems/wheel.py:44-59` 只读内置模板），两侧都由它产出车轮刚体 + 轮胎声明；(b) 装配阶段不再出现 `isinstance(element, VerticalTireElement)` 的过滤（`grep` 无命中），该过滤的删除归属本行（`subsystems/vehicle_assembly.py`）；(c) **凝结的硬门（D2）**：**单轴侧**（`RigSpec.supplies_wheels=True` 的读数：K/C 与轴动态）在装配期把 wheel 子系统产出的车轮刚体与轮毂刚体**刚性凝结**，装配后的实体集合、约束行数与自由度不变，且 **`kc_baseline` 与 `dynamic_hash_baseline` 的轴侧用例逐位不变**——K/C 用 `kc_native_probe.py` + `kc_native_c_probe.py` 生成 actual 再带 `--actual-dir` 比对（不是不带 `--actual-dir` 的自比较），轴侧动态用 `dynamic_hash_sentinel.py --check`。**判据不得再用「车轮子系统本次不产出车轮刚体」**（文件层统一后该判据失效，2026-09-29 已复核），改用试验台/请求声明（`supplies_wheels`）；(d) 凝结复用既有机制（`_merge_fixed_wheel` / `_fuse_welded_bodies`），并在 PROGRESS 里点名与 20260921 Epic A3（整车侧不凝聚）的关系；(e) 读数决定受力：K 台（轮心推压）下轮胎不参与、C 垫板机下轮胎弹性生效——两侧各有断言，沿用 F9 的分层（装配体不变、文档决定）；(f) 本行对 `preparation/vehicle_dynamic.py` 只改**轮端内容段**（`:900-903`、`:1009-1025`），模型访问段归 03 且必须在 03 之后。

**05（试验台非侵入）**：(a) 接入试验台前后，被测总成 runtime 的 `bodies` / `points` / `constraints` / `elements` **逐项比较所有权、参数与几何值**（承载体名、`wheel_center_local`、约束端点与类型、点坐标），只允许新增试验台自己的实体——只比实体名集合不够，因为 `_reown_tires` 正是"名字不变、所有权变"；(b) `_reown_tires` 被删除（`grep` 无命中），`test_rig_link.py` 的契约反转为"不改写被测实体"且有断言；(c) 试验台只通过轮心夹具/接触点建立约束；(d) 现有 7 组合在 `SUSPENSION_MULTIBODY_RIG_ENTITIES=1` 下产物与 01 快照的差异**逐项登记**（文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据）并写入 `raw/approved_deltas.json`，每条登记要与差异**四项精确匹配**且 `registered_by` 写 05——未命中登记的差异必须让 `snapshot.py --check` 退出 1；**注意覆盖边界**：01 的快照对整车侧 5 个 rig 只记录与试验台无关的装配体（`_meta.coverage_boundary`），故这 5 个 rig 必须由本行自己的「接入前后运行时逐项对照」来证明非侵入，不能只用快照；(e) `RigSpec.supplies_wheels` 的语义改写后，**既有 rig 声明如何解释**有明确规则并登记（供轮 = 提供轮体，不再等于"夺走被测轮胎"）。

**06（可选对称）**：(a) 同一悬架模板，总成文件写"一个条目 + 镜像"与写"左右两个条目 + 各自文件"两种方式，装配产物**逐项比较坐标与数值**（体/点/约束的集合、名字、点坐标、约束端点）后一致；(b) 不对称硬点（左右不同坐标）能装配并跑通一次，且断言**独立文件不被二次镜像**（比较装配后的点坐标与文件写的坐标）；(c) 默认（不声明）仍走镜像，产物与 01 快照逐项一致；(d) `SIDES` 的消费点全部改为按总成声明取值——含 `subsystems/si_assembly.py:56`、`subsystems/wheel.py:114-116 sides()`、`subsystems/element_build.py:206/213` 的固定双侧展开（后两者属本行写范围），`grep "_SIDES"` 在装配路径无残留硬编码（过渡收口必须在 07 之前删除并登记）；(e) 单侧文件的**坐标系、镜像方向与命名契约**（`side_hardpoints`/`mirror_hardpoints` 的语义、`placement_role` 取值、左右独立文件的命名规则）写进文档并有测试；(f) **单轮（单侧悬架）与三轮（两悬架 + 一个单侧）各装配并跑通一次**（G1 判据 (b)，由本行兑现）。

**07（终局独立验收）**：见 Done-When 的端到端清单，逐条实跑并记录退出码与产物差异。命令至少包含：`uv run --no-sync pytest packages/suspension_multibody/tests -q`（全量）、`uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q`、`uv run --no-sync pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q`、`uv run --no-sync ruff check .`、`uv run --no-sync ty check .`、三个架构门脚本、`dynamic_hash_sentinel --check`、`case_parity_check`、`kc_perf_gate --check`，以及 Done-When 的 (a)-(g) 各自的专用用例。此外：(i) (g) 拖挂铰接必须由本行独立复跑（03 交付的用例只作线索）；(ii) `raw/approved_deltas.json` 逐条审计——每条登记都要与 05 的 `PROGRESS.md` 里记录的逐步变化对得上、都要有独立于结果字节的物理等价判据，且不得用前缀式登记掩盖未声明的变化（脚本已无前缀语义，登记不合法直接退出 4）；(iii) 7 个组合的产物差异不得出现「未登记而通过」的情形。

**数值门为独立项**：`dynamic_hash_sentinel --check`、`case_parity_check`、`kc_perf_gate --check` 在本 Epic 期间每次基线重录后都必须重新通过；重录本身有登记。**`kc_parity_check.py` 不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），不构成证据**——需要 K/C 等价对标时必须先跑生产者（`kc_native_probe.py` + `kc_native_c_probe.py`）再带 `--actual-dir artifacts/kc-native-probe` 判定。每行 DONE 不代替这些条件。

## Done-When

**逐条确认 G1–G6**：

1. **G1**：装配引擎由条目清单驱动；3 悬挂总成装配并跑通；装配路径无 `front_axle`/`rear_axle`；文档形状规则支持 N 轴（正例+负例各一）。
2. **G2**：跨子系统连接只经显式配对；车身侧刚体改名后同一份文件仍可装配；`"chassis"` 字符串规则全部消除或收口登记；歧义/悬空配对报错点名。
3. **G3**：单轴与整车共用同一 wheel 子系统文件产出轮端实体；装配阶段无删建轮胎；K/C 与动态的差异由读数决定，两侧各有断言；质量守恒不变。
4. **G4**：试验台接入前后被测实体逐项一致；`_reown_tires` 消除；`test_rig_link` 契约反转并有断言。
5. **G5**：镜像与"左右独立文件"两种写法产物一致；不对称硬点可装配；默认路径产物不变。
6. **G6**：7 个既有组合默认可用；每次产物变化都有登记；静态门、快速集、数值门在每次落地后重新通过；无新增 skip/xfail。

**端到端独立验收（不依赖子任务自证，07 逐条实跑）**：

```text
(a) 3 轴整车总成（3 个悬架子系统 + 车身 + 转向 + 车轮 + 制动 + 驱动）装配并跑通一次；
    实体清单与文件清单一一对应，装配路径 grep 无 front_axle/rear_axle
(b) 同一份总成文件，悬架条目分别写"镜像"与"左右独立文件"，两次装配产物**逐项比较点坐标与约束端点**后一致
(c) 单轴总成与整车总成引用同一份 wheel 子系统文件；装配阶段无"删轮胎再建车轮"；
    单轴 K/C 装配期刚性凝结车轮与轮毂，自由度拓扑与 kc_baseline 逐位不变（D2）
(d) 试验台非侵入：接入前后被测 runtime 逐项比较所有权、参数与几何值后一致；差异只在外加约束/载荷
(e) 显式配对：文件写出悬架→车身挂点配对即按配对连；车身刚体改名 subframe 后同一文件仍装配
(f) 零回归：7 个既有组合的文档与数值门全绿；所有基线重录都有登记
(g) 拖挂铰接：同一总成内两个车身侧体（牵引车与挂车）经现有副类型的铰接副连接并跑通一次（由 03 交付、本行独立复跑）；
    只有在实测证明现有副类型不可行时，才登记为内核范围并提请用户裁决，不得以「登记」代替运行证据
```

## 风险与回退

- **03 换入口的风险**：装配引擎换驱动方式会让现有产物漂移。缓解：01 的字符化快照作硬门；新入口与适配器路径并行，先证明适配器路径产物逐项一致再切换。
- **03 的 preparation 图谱化风险（2026-09-29 复核新增）**：G1(a) 按字面口径要求整个 `preparation/` 零 `front_axle`/`rear_axle` 命中，这意味着 1268 行的整车动态准备改为消费装配图谱，而它的 `dynamic_hash_baseline` 是冻结基线。缓解：适配器（`VehicleModel` → 图谱）留在 `preparation/` 之外并先测通；分步切换（先 bushings/rack 两处判定，再两轴遍历）；每一步都跑 `dynamic_hash_sentinel.py --check`，任何一位变化即退回；本轮**不得重录**任何动态基线（`vehicle_dynamics_baseline/sha256.json` 已于 2026-09-29 经用户授权重录过一次，本轮不再动）。
- **04 的风险（D2 后已大幅降低）**：文件层统一不能让求解拓扑漂移——若凝结后 `kc_baseline` 有任何一位变化，即凝结不等价，必须退回并先证明等价（这是本行唯一的硬门）；`_fuse_welded_bodies` 与 20260921 Epic A3 的口径关系必须登记清楚，避免两处"凝聚"语义混淆。
- **05 轮胎归属回迁的风险**：C 垫板机的轮胎作用点与力路径会变。缓解：先做"试验台夹具 vs 轮胎"两条力路径的对照，再决定 `supplies_wheels` 的新语义；变化逐项登记。
- **06 对称可配置的风险**：镜像路径被改动会波及所有既有几何断言。缓解：默认值保持镜像，不对称走显式声明；01 快照保证默认不变。
- **接口配对（02）的风险**：把配对写成硬性要求会让所有既有总成文件失效。缓解：配对段**可选**——留空时按角色/能力唯一匹配（即今天的推断行为），显式配对是覆盖手段；只有"候选不唯一"才报错。
- **拖挂/3 轴若需新副类型**：属内核范围（ABI 与版本常量），本 Epic 明确不做；以 Non-Goal 与 (g) 的登记形式收口。
- **既有失败**：本 Epic 起点（F8）全绿；任何新增失败阻断完成，不相关既有失败独立列明。

## 目录与命名

```text
.codex-tasks/20260929-assembly-layer-rework/
├── EPIC.md
├── SUBTASKS.csv
├── PROGRESS.md
└── tasks/
    ├── 20260929-01-freeze/            现状冻结与判据
    ├── 20260929-02-port-binding/      显式接口配对
    ├── 20260929-03-universal-assembler/ 通用装配引擎
    ├── 20260929-04-wheel-lifecycle/   轮端生命周期统一
    ├── 20260929-05-rig-non-invasive/  试验台非侵入
    ├── 20260929-06-optional-symmetry/ 可选对称
    └── 20260929-07-acceptance/        终局独立验收
```

本轮为**规划轮**：`EPIC.md` + `SUBTASKS.csv` + `PROGRESS.md` 是交付物，**同时落盘全部 7 个子任务目录**（每个含 `SPEC.md` / `TODO.csv` / `PROGRESS.md` / `raw/`），使 `SUBTASKS.csv` 的 `task_dir` 真实存在、Epic 可执行且可冷启动恢复。子任务的 `SPEC.md` 写"要做什么、写哪些路径、判据与证据落在哪"，`TODO.csv` 是它的步骤表（初始 `TODO`），`PROGRESS.md` 是它的恢复块；子任务开工时按 SPEC 展开步骤，**不得**用规划文本冒充实施记录。临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据，不存虚构结果。父 `SUBTASKS.csv` 管子任务状态，子 `TODO.csv` 管具体步骤，禁止相互替代。除 01 已开工的第一行外，本轮不将任何子任务置为 `DONE`。
