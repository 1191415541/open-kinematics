# 子任务 04：从 build_front_axle 拆出六类子系统（含简化制动/驱动与转向可缺席）

## 目标

把现役 `build_front_axle`（`packages/suspension_multibody/src/suspension_multibody/preparation/assembly/front_axle.py:625-926`）拆成**六个**可独立实例化的子系统，粒度由用户裁决，并让**转向子系统在悬架实验总成中可选**（需求 15，裁决 D5）、**制动/驱动以可替换的简化模板落地**（需求 20，裁决 D8/D11）。

用户原话：「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到悬架实验总成，前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成」（需求 13）；「我想把制动系统和驱动系统以及轮胎也分别实现为子系统，当前已经实现的简单驱动和制动系统，在子系统里可以作为主动力矩作用在车轮子系统上」（需求 16）；「当前的简单制动/驱动子系统不需要有刚体，具体参考adams car的实现；悬架实验总成不需要制动、驱动子系统，整车总成必须要」（需求 17）；「在单轴侧，车轮作为悬架试验台的一部分，同样参考adams car」（需求 19）；「当前制动和驱动子系统可以做成简化实现，但是一定要有能力在后续的模板中可以扩展成带刚体的复杂形式」（需求 20）。

1. 新建 `suspension_multibody/subsystems/` 包，按 03 建立的 **role** 定义**六类**子系统，各有明确输入与输出对象：
   - **左右悬架（suspension）**：body / point / constraint 的生成，含 `symmetric_proxy` 的 UCA/LCA/upright 与镜像逻辑。现状落点：`front_axle.py:53-67`（`mirror_hardpoints` / `side_hardpoints`）、`:655-665`（每侧四体）、`:668-685`（挂点表 `mount_data`）、`:686-750`（UCA 内点，K 用 `RevoluteJoint` `:706-716`、C 用 `BallJoint` `:719-723` + 占位衬套 `:724-739`）、`:751-815`（LCA 内点，K `:771-781`、C `:784-788` + `:789-804`）、`:816-837`（外点 `BallJoint`）。同时归属左右悬架侧的弹性元件：springs `:525-541`、dampers `:542-560`、stops `:575-590`、anti-roll bar `:591-601`（**跨左右**：`left_body="upright_L"`、`right_body="upright_R"`，故只能归"左右悬架"这一含双侧的子系统）。**注意**：`tie_rod_{side}` 归转向子系统，不要因其出现在挂点表里而误划给悬架。
   - **转向（steering，可缺席）**：rack 体（`:643-647`）、tie rod 两端连接（`tie_rod_{side}` 体 `:655-660`；`rack_tie_joint_{side}` `:846-848`、`tie_upright_joint_{side}` `:849-851`）、rack 固定/导轨（`:874-903`：`model.rack_fixed_to_chassis` 为真时 `WeldJoint(rack_fixed_to_chassis)` `:881-888`，否则 `PrismaticJoint(rack_guide)` `:889-900`；仅当 `"rack_housing" not in bodies` 才追加 `:901-903`）。**缺席规则见目标第 5 条。**
   - **车轮（wheel，含轮体与轮胎）**：涵盖轮体与轮胎；原「轮胎」子系统按需求 16 更名为「车轮」。**可用性按 D9 不对称**：整车侧为子系统（现状 `_add_wheel`，`vehicle.py:598-688`，造 `wheel.body` + `RigidBody` + `RevoluteJoint` 自转副 + 中心/接地点）；**单轴侧不产出 `wheel.body`**——车轮由悬架试验台提供（`testrig_tire_property_file`/`testrig_wheel_radius`/`tire_stiffness`，由 10 承接）。单轴侧现有 `VerticalTireElement`（`front_axle.py:561-574`，来自 `model.tires`，`wheel_body=f"upright_{side}"`、`wheel_center_local` 取自 `wheel_center` 硬点）在本步**维持现状不改为独立轮体**。
   - **制动（brake，仅整车；简化模板 0 刚体）**：本步落地**一个简化模板**——只声明分配参数、只发逐轮制动力矩，**不产出任何刚体**（对标 Adams `default_brakes.sub`：`[PART_ASSEMBLY]` = 0）。参数按 D10 取力矩子集：`brake_mu` / `piston_area` / `effective_piston_radius` / `front_brake_bias` / `max_brake_value`（现状已有 `front_brake_bias`/`maximum_brake_torque`，见 `schema/vehicle.py:174-175`，须补前者三项并说明与 `maximum_brake_torque` 的关系）。力矩施加沿用内核既有 `brake_torque` role（`cpp/src/cases/vehicle_dynamic.cpp:33`）——**制动信号是非负幅值，方向由内核按轴向转速符号决定**（`cpp/src/element/drive_brake.cpp:89-98`），Python 侧不得自行定方向。
   - **驱动（drive，仅整车；简化模板 0 刚体）**：本步落地**一个简化模板**——只声明分配参数、只发逐轮驱动力矩，**不产出任何刚体**（对标 Adams `pvs_driveline=0` 失活路径，`help/adams_car/appendix/drivelines.html`）。参数沿用现有 `DrivelineSpec` 语义：`driven_wheels` / `drive_split` / `maximum_drive_torque`（`schema/vehicle.py:169-190`）。力矩施加沿用内核既有 `wheel_torque` role（`vehicle_dynamic.cpp:32`）。
   - **制动/驱动的可用性（需求 17 / D8）**：**悬架实验总成不含**制动与驱动（对标 Adams `acar_gs_front.asy` 只有 suspension+steering+TESTRIG）；**整车实验总成必须含**（对标 `acar_gs_full.asy` 有 `Major Role : brake_system` 与 `powertrain`）。本步只定义与实现子系统本体，接入由 10（试验台输入）与 11（整车组装）承接。
   - **车身（chassis）**：chassis 体与 `MassSpec`。实测：轴侧 `front_axle.py:644` 直接造 `RigidBody("chassis", fixed=True)`，**不消费** `MassSpec`（`schema/model.py:28-60`，字段在 `:165`；`front_axle.py` 只读 `RigidBodySpec.mass`，见 `:174`、`:191`）；整车侧 `preparation/assembly/vehicle.py:99` 的 `build_vehicle` 用 `_body_from_spec(model.chassis)` 造 chassis，再由 `:118-128` 把轴侧 chassis 重命名并入整车体表。
2. 六类子系统各自声明输入（模型 / 硬点 / 上游子系统产物）与输出（`RigidBody` 集合、点表 `dict[tuple[str,str], np.ndarray]`、`Connection`、`Constraint`、`Element`），互不隐含共享可变状态；单独构造不经过 `build_front_axle`。
3. `build_front_axle` 改为**由六类子系统组合**而成，组合顺序复现现役追加顺序；同时**保留为适配器**：签名 `build_front_axle(model, mode="K")` 与返回类型 `FrontAxleAssembly`（`:82-95`）不变，直到 11 之后再定是否删除。
4. 建立"组合路径 vs 现役实现"的逐位一致判据，以及逐项对照表（body 集合与顺序、`points`、`hardpoints`、`connections`、`constraints`、`ideal_constraints`、`bushings`、`elements` 名称序、驱动坐标定义），落 `raw/`。
5. **转向子系统可缺席（需求 15 / D5）**：单轴悬架实验总成允许**不声明转向子系统**。缺席时下列产物**整体不产出**，其余子系统与几何逐位不变：
   - `rack` 刚体（现役无条件建：`:643-647` 的 `"rack" in body_specs` 只决定是否取 `MassSpec`，不决定建不建）；
   - 每侧 `tie_rod_{side}` 刚体与两个球铰 `rack_tie_joint_{side}`/`tie_upright_joint_{side}`（`:655-660`、`:838-873`）；
   - rack 中心点 `points[("rack","center")]`、`points[("chassis","rack_center")]`（`:874-880`）与 `rack_guide`/`rack_fixed_to_chassis` 约束（`:881-903`）；
   - 相应地 `rack_center` 硬点不再必填（现由 `_ALIASES["rack_center"]`（`:144`）经 `_lookup` 强制要求）。
   **不做**「建一个退化 rack 体但不接约束」的替代实现——那会让无转向总成仍带一个悬空刚体，与「总成实际提供哪些子系统」的语义不符，也会污染 10 的坐标收缩判断。
   **整车侧不放开**（D5）：`schema/vehicle.py:233` 的 `SteeringSystemSpec` 必填、`:249-259` 的 `required_bodies` 含 `rack`/`tie_rod_L`/`tie_rod_R`、`preparation/vehicle_dynamic.py:227` 无条件调 `_build_steering`，一律不动；5 个 vehicle family 行为不变。
6. **对外暴露「总成实际提供的子系统与坐标系集合」，且契约在本步冻结**：10 要按总成能力收缩试验台接口（D6），本步须提供可查询的能力描述。**契约（两个独立实现者必须能对上，故此处定名）**：
   - **`rack_axis`/`rack_fixed_to_chassis` 无转向时的处置**：`schema/model.py:175-176` 两字段带默认值；无转向总成下它们**语义悬空但保留默认即可**，不必删除或校验报错——删除会破坏现有模型构造，校验报错会让「同一模型切到无转向」不成立。
   - 对象：`AssemblyCapabilities`（dataclass，不可变），由总成构造时产出并挂在总成上；
   - `subsystems: frozenset[str]`：取值限于**固定六类枚举** `{"suspension", "steering", "wheel", "chassis", "brake", "drive"}`（与 G2 的六类一致；**用 `wheel` 不用 `tire`**，与 03 的 role 名一致）。单轴总成的该集合**不含** `"steering"`（可缺席，D5）、**不含** `"brake"`/`"drive"`（需求 17/D8）、**含** `"wheel"`（由试验台提供，D9）；整车总成的该集合**含** `"brake"`/`"drive"`；
   - `drive_coordinates: frozenset[str]`：总成**实际可驱动**的坐标名，命名空间**以模型文档中实际使用的驱动坐标名为准**——即 `wheel_drive_L`/`wheel_drive_R`/`rack_drive`/`rack_neutral` 这套（`cases/kc_quasi_static/contract.py:189-221` 写入 `entry["name"]` 的名字），**不是** case 侧 `axis_map` 的分组键名（`"wheel"`/`"rack"`）；无转向总成不含任何 `rack_*` 名；
   - 判定规则（10 与测试都用这一条）：坐标是否可用**只由** `coordinate in capabilities.drive_coordinates` 决定；**禁止**按 `"rack" in bodies` 之类探测名字推断（也禁止用 `axis_map` 的键名当坐标名，两者命名空间不同，混用即接口错位）；
   - 诊断用附带信息（可选，不得作为判据）：在场体的名称集合。
7. **简化制动/驱动必须是可替换的提供者，不得堵死复杂模板（需求 20 / D11，本步最关键的架构约束）**：
   - 简化实现只是 `brake`/`drive` role 下的**一个模板**，不是这两个 role 本身。role 接口（参数槽、力矩通道、输出声明、几何挂点）对简化与复杂实现**同构**；
   - **装配层与试验台不得出现任何「是否简化版」的分支**：不得有 `if simplified:`、不得按模板名判断、不得按「有没有刚体」判断；
   - 复杂模板（带卡钳/转子/动力总成体/差速器）落地时，**只允许新增模板 + 注册**，不允许改 `subsystems/`、试验台、装配层或 C++ 内核；
   - 本步须留一条**可执行的证明**：用同一 role 下的第二个模板（测试内构造的、声明产出一个刚体的最小桩模板）替换简化模板，装配路径**代码零改动**即可跑通。**"装配路径"指本步可触达的那条**——即 `subsystems/` 的子系统组装入口与 `build_front_axle`/`build_vehicle` 的组合路径（制动/驱动虽仅整车可用，但 04 定义的 role 组装路径对它们同构，故可在 04 内用桩模板验证）。这条测试是本步的硬门，也是 11 与 12 判据 (h) 的前提。
   - **对标 Adams 实证**：同 `MAJOR_ROLE='brake_system'` 下 `_brake_system_4Wdisk.tpl`（0 体，见 `default_brakes.sub` 17 项参数）与 `_brake_system_4Wdisk_calipers.tpl`（4 体，见 `convertible_brake_system.sub`）并存；powertrain role 下 `_powertrain.tpl`（有刚体）与 `_driveline_fwd.tpl`/`_driveline_rwd.tpl` 并存。注意 `sedan_brake_system.sub` 与 `default_brakes.sub` 参数集不同（15 项 vs 17 项），见 EPIC 事实节的登记。

## 非目标

- 不改 K/C 语义（05 做）：本步不把零刚度占位衬套改成模板属性，`front_axle.py:737`、`:802` 的 `stiffness=np.zeros((6,6))` 保持。
- 不接线模板（03 只建结构，05 接线）：本步不读 `templates/`。
- 不改几何生成规则、硬点别名表（`:114-155`）与镜像规则（`:53-67`、`:229-237`）。
- 不删除 `build_front_axle`。
- 不改 `symmetric_proxy` / `explicit` 两条拓扑的语义（`explicit` 走 `_build_explicit_axle` `:387`）。
- 不实现属性文件（06）与输出声明（07）。

## 约束
- **需求 15 只落在单轴侧**：本步不触碰 `schema/vehicle.py` 的 `SteeringSystemSpec`（`:233`）与 `required_bodies`（`:249-259`），也不改 `preparation/vehicle_dynamic.py:227` 的 `_build_steering`；若为通过本步而要放宽整车侧校验，即为越界。
- **不改默认装配行为**：含转向的默认路径逐位不变；本步新增的是可选缺席路径，不是把默认改成可缺。
- **新增字段不得改变现有 `model_dump` 输出（覆盖 `schema/model.py` 与 `schema/vehicle.py` 两处）**：`api.py:116`/`:284` 把 `model.model_dump(mode="json")` 的规范化哈希写进 `Provenance.model_hash`（并进结果 bundle 与 checkpoint），`io/artifacts.py` 又把它哈希成 artifact manifest 的 `model_sha256`（被 `dynamic_hash_sentinel.py` 的 key 集覆盖）。因此：
  - 转向缺席的声明方式**不得**是「给 `FrontAxleModel` 加一个会出现在 dump 里的新字段」——那会改变所有现有模型的 `model_hash`，进而改变结果字节；
  - **制动参数三个新字段（`brake_mu`/`piston_area`/`effective_piston_radius`）同样受此约束**：`VehicleModel.driveline`（`schema/vehicle.py:234`）是常驻字段，若给 `DrivelineSpec` 加普通字段，`VehicleModel.model_dump` 会多出键，改变整车 `model_hash` 并直接撞本步「不得重录任何基线」（含 `vehicle_dynamics_baseline`）。必须采用 `model_dump` 排除机制（如 `exclude=True`、或放入既有的 `parameters` 之类不参与 dump 的容器），或证明这些字段不改变含转向模型的 dump；
  - 可行方向：由**子系统声明/组装层**表达缺席（模板或总成层的子系统清单），属性类参数走被 `model_dump` 排除的字段。
  - 无论哪种，都必须有测试断言现有模型的 `model_dump(mode="json")` 逐字节不变（两个 schema 各一条）。
- **不得修改 `templates/**`**：那是 03 的写范围，本步只读其结构。
- 本步与 03 都触及 `preparation/assembly/types.py`，**必须串行**，不得并行。
- **产物逐位一致**：组合产物与现役实现必须是 `np.array_equal` 级别的一致（点表逐元素、集合逐顺序），不是"数值近似"。
- **元素顺序不得变**：`_runtime_elements`（`:515-622`）先 `for side in ("L","R")`（`:524`）逐侧追加 springs(`:525-541`) → dampers(`:542-560`) → tires(`:561-574`) → stops(`:575-590`)，循环后追加 ARB(`:591-601`)，再在 C 模式追加 `model.bushings`(`:602-622`)；`build_front_axle` 最后把 8 条衬套再接在尾部（`:910-911`）。车轮子系统产出必须**插回该位置**，不得追加到末尾。
- **不得新增 legacy import**：`tests/architecture/legacy_surface_gate.py` 的 MODE_MIGRATION 只容忍注册表内的既有条目（`tests/architecture/legacy_surface_registry.json` 的 `entry[1]` = `preparation/assembly/front_axle.py` 导入 `elements`，`entry[2]` = `vehicle.py`）。新模块导入退役的 `elements` / `core` / `model` / `analysis` / `metrics` 会成为未注册 finding 并失败。因此新 `subsystems/` 包**不得直接导入 `elements`**：元素构造留在已注册的 `front_axle.py` / `vehicle.py`（子系统返回声明数据，元素在注册文件里构造），且 `legacy_surface_gate.py --check` 必须**在不新增注册条目**的前提下保持绿。
- 不引入新依赖。
- **不得重录任何基线**：`tests/data/kc_baseline/**`、`kc_perf_baseline.json`、`kc_perf_baseline_native.json`、`dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/`、`packages/suspension_kernel/layering_baseline.json` 全部保持原字节。
- **需求 20/D11 的可扩展性不得被堵死（本步硬约束）**：简化制动/驱动**不得**把「无刚体」写进 role 接口或装配层。装配层与试验台不得出现 `if simplified:`、按模板名判断、按「有没有刚体」判断等分支；复杂模板落地时只允许新增模板 + 注册。见目标第 7 条。
- **需求 17/D8 的可用性不对称**：悬架实验总成不含制动/驱动、整车实验总成必须含；两侧都要有测试锁定，不得为「对称」而给单轴侧加制动/驱动。
- **需求 19/D9**：单轴侧**不产出** `wheel.body`（车轮归试验台，10 承接）；本步不得顺手把单轴侧 `VerticalTireElement` 改成独立轮体——那会动 `axle_dynamics_baseline` 与 `kc_baseline`，且超出本步范围。
- **不触碰内核的制动/驱动路径**：制动只走 `vehicle_dynamic` 家族（`axle_dynamic.cpp:31` 的 `TireRole` 无 `BrakeTorque`），本步与后续都不需要为单轴侧加制动通道；`wheel_torque`/`brake_torque` 两个 role 内核已具备（`vehicle_dynamic.cpp:32-33`）。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/subsystems/**`
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/front_axle.py`（改为六类子系统组合；保留 `build_front_axle` 签名与 `FrontAxleAssembly` 字段）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/vehicle.py`（chassis 侧接线；`build_vehicle` 行为不变）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/__init__.py`（仅当需要导出新符号，且不得改变现有导出）
  - 新增测试 `packages/suspension_multibody/tests/subsystems/**`
  - `packages/suspension_multibody/src/suspension_multibody/schema/model.py`（**仅需求 15 所需**：转向缺席的声明方式与相应校验；不得改动 `symmetric_proxy`/`explicit` 拓扑语义与其他字段；新增字段须满足下面的 `model_dump` 约束）
  - `packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py`（**仅需求 17/D8 的制动参数子集所需**：补 `brake_mu`/`piston_area`/`effective_piston_radius` 三个参数槽，并说明与现有 `maximum_brake_torque` 的关系；**不得改动** `SteeringSystemSpec`（`:233`）与 `required_bodies`（`:249-259`）——那是 D5 的整车侧不放开；新增字段同样须满足 `model_dump` 约束）
  - 对照表与快照证据：`tasks/20260922-04-subsystems/raw/**`
- 只读：`templates/**`（03）、`schema/**`（**`schema/model.py` 与 `schema/vehicle.py` 除外，见可写**）、`cases/**`、`elements/**`、`preparation/assembly/types.py`、`preparation/vehicle_dynamic.py` 的 `_build_steering` 与 `_validate_steering_topology`（仅作「整车侧不放开」的对照证据，禁止修改）、父级 `EPIC.md` / `SUBTASKS.csv`。
- 不写：`templates/**`、`outputs/**`（07）、`report/**`（07）、`tests/data/**`（基线）、`packages/suspension_kernel/**`、父级计划文件（归主代理）。

## 依赖

- 前置：03（模板与双列连接点数据结构；本步只读其结构，不接线）。02 已给出统一副表与副编码。
- 后续：05（模板实例化与 K/C 列激活——在 04 的子系统之上做实例化）。

## 验收标准

1. 六类子系统可独立实例化：每类有自己的构造函数与明确的输入/输出对象，测试可在不经过 `build_front_axle` 的情况下单独构造成功。
2. **与现役产物逐项对照，`benchmark_axle.json` 基线上差异为零**。本 SPEC 制定时实跑的现役值（实施时先复核再对照）：
   - `bodies`：10 个，顺序 `['chassis','rack','upper_arm_L','lower_arm_L','upright_L','tie_rod_L','upper_arm_R','lower_arm_R','upright_R','tie_rod_R']`；
   - `points`：36 条；K 与 C 两模式的键序与数值完全一致；
   - `connections`：16 条，顺序 `['uca_mount_L_inner_front','uca_mount_L_inner_rear','lca_mount_L_inner_front','lca_mount_L_inner_rear','upper_arm_L_outer_joint','lower_arm_L_outer_joint','rack_tie_joint_L','tie_upright_joint_L']` 后接同序的 `_R` 组；
   - **K 模式**：`constraints=13` / `ideal_constraints=13` / `bushings=0` / `elements=0`；`constraints` 顺序为每侧 `[RevoluteJoint:uca_mount_{S}_inner_front, RevoluteJoint:lca_mount_{S}_inner_front, BallJoint:upper_arm_{S}_outer_joint, BallJoint:lower_arm_{S}_outer_joint, BallJoint:rack_tie_joint_{S}, BallJoint:tie_upright_joint_{S}]`（S=L 后 R），末尾 `PrismaticJoint:rack_guide`；
   - **C 模式**：`constraints=9` / `ideal_constraints=17` / `bushings=8` / `elements=8`；`ideal_constraints` 顺序为每侧 `[uca_mount_{S}_inner_front, uca_mount_{S}_inner_rear, lca_mount_{S}_inner_front, lca_mount_{S}_inner_rear, upper_arm_{S}_outer_joint, lower_arm_{S}_outer_joint, rack_tie_joint_{S}, tie_upright_joint_{S}]`，末尾 `PrismaticJoint:rack_guide`；`elements` 与 `bushings` 名称序为 `uca_bushing_{S}_inner_front, uca_bushing_{S}_inner_rear, lca_bushing_{S}_inner_front, lca_bushing_{S}_inner_rear`（L 后 R）；
   - `hardpoints`：模型硬点 + 每侧 `{name}__{side}` 副本（`:666-667`）+ `RACK_CENTER`（`:904-906`）。
3. **组合路径与现役实现逐位一致**（纯重构判据）：`build_front_axle(model, mode)` 在组合实现下与本步改动**之前**现场抓取并留在 `raw/` 的现役产物逐位一致；至少覆盖 K、C 两模式与 `rack_fixed_to_chassis` 真/假两个分支。
4. **驱动坐标定义不变**：kc 的驱动坐标由装配点表推导（`cases/kc_quasi_static/contract.py:177-222`，`:209` 取 `assembly.point("upright_{side}","wheel_center")`、`:219` 取 `assembly.point("rack","center")`），因此 `points` 的键与值逐位一致即证明驱动坐标定义一致；有测试断言。
5. `explicit` 拓扑路径行为不变：`_build_explicit_axle`（`:387`）与 `_runtime_elements_explicit`（`:314`）的产物逐位一致。
6. 现役门禁保持绿且**未重录任何基线**：`kc_parity_check.py --check`、`case_parity_check.py`、`dynamic_hash_sentinel.py --check`、`check_module_layering.py --strict --final`、`legacy_surface_gate.py --check`（不新增注册条目）、三套 pytest、ruff、ty、`git diff --check`。
7. **未消费事实如实登记**：`MassSpec` 在轴侧装配中从未被读取（`schema/model.py:165` 定义、`front_axle.py` 不引用），车身子系统必须保留这一差异并在 `raw/` 登记，不得顺手"修正"为消费 `MassSpec`。
8. **转向可缺席且逐项差集一致**：同一模型分别构造含转向与不含转向的单轴总成，逐项对照 `bodies`/`points`/`hardpoints`/`connections`/`constraints`/`ideal_constraints`/`bushings`/`elements`/驱动坐标定义——不含转向侧应为含转向侧**去掉** `rack`、`tie_rod_{L,R}`、4 个拉杆球铰与 `rack_guide`（及 rack 相关点/连接/硬点）后的**精确子集**，其余条目逐位相同。多删或少删均失败。**注意**：此处「驱动坐标定义」只能按**装配点表**（`points` 的键与值）差集推导——`cases/kc_quasi_static/contract.py:177-222` 的 `_driven_coordinates` 在无 rack 时会先于本步失败（`:215-221` 无条件取 `assembly.point("rack","center")`），故本步的差集断言不得经由该函数；其收缩属 10 的范围。
9. **能力描述可查询**：不含转向的总成报告「不含 steering 子系统、不提供 rack 坐标」，含转向的总成报告相反；该能力描述是 10 的输入，不得靠 `bodies` 名字猜测。
10. **含转向的默认路径逐位不变**：`rack_center` 仍必填；K 模式 13/13/0/0 与 C 模式 9/17/8/8 的实测值不变；`rack_fixed_to_chassis` 真/假两个分支与 `rack_housing` 分支行为不变。
11. **简化制动/驱动的可替换性（需求 20 / D11，本步硬门）**：同一 `brake`/`drive` role 下，用第二个模板（测试内构造的最小桩模板，声明产出 1 个刚体）替换简化模板，装配路径**代码零改动**即可跑通；并断言 role 接口字段集与装配层中**不存在**任何「是否简化版」的分支判据。
12. **制动参数与 Adams 简单版逐参数对标（D10）**：`brake_mu`/`piston_area`/`effective_piston_radius`/`front_brake_bias`/`max_brake_value` 均可声明且往返一致；有测试按 `SFORCE/31-34` 的公式形状核对逐轮力矩（含 `front_brake_bias` 前后分摊与按轮速符号反向）。**基准文件须先确定**：源 `.adm` 的常数（2500/0.6/145.0）混用了 `default_brakes.sub`（135.0）与 `sedan_brake_system.sub`（145.0）两套口径，实施时须选定一个 `.sub` 作基准并在 PROGRESS 登记；常数 `0.1` 的来源须核实或明确登记为未核实项。
13. **可用性矩阵（需求 17 / D8）**：有测试断言悬架实验总成的子系统集合**不含** `brake`/`drive`，整车实验总成的集合**含**二者。

## 验证协议

1. 子系统逐个落地，每落一类立刻跑一次 `tests/subsystems` 中该类测试（禁止攒到最后一起验）。
2. 每拆一类，立即与该类在现役实现中的产物逐项对照（body / 点 / 约束 / 连接 / 元素位置）。
3. 组合完成后跑逐位一致测试（K、C、`rack_fixed_to_chassis` 三组）。
4. 门禁顺序：`build_axle_native.py` → `kc_parity_check.py --check` → `case_parity_check.py` → `dynamic_hash_sentinel.py --check` → `legacy_surface_gate.py --check` → `check_module_layering.py --strict --final`。
5. 收尾：三套 pytest、`ruff check .`、`ty check .`、`git diff --check`，并确认 `tests/data/**` 与 `layering_baseline.json` 无 diff。
6. 转向缺席落地后：跑「含转向 vs 不含转向」逐项差集测试与「含转向默认路径逐位不变」测试；既有断言（`tests/model/test_front_axle.py`、`tests/cases/kc_quasi_static/**`）走默认含转向路径，**不应改动**——若必须改动即为回归。

**本步是纯重构：任一基线发生变化即停止并上报**——那说明拆子系统时误改了产物，而不是"预期重录"。
