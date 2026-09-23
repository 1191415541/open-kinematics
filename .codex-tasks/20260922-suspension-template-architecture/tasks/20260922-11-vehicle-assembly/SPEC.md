# 子任务 11：整车实验总成组装（前后悬架 + 转向 + 车身 + 车轮 + 制动 + 驱动 + 整车 KC 试验台）

## 目标

承接用户裁决中「整车实验总成」这一组装，补上三层架构在**整车侧**的落点。

用户原话：「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到**悬架实验总成**，**前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成**」。
1. **两种组装都成立**：
   - 悬架实验总成 = 左右悬架 + [转向（可缺席）] + 车轮（试验台提供）+ 悬架试验台（单轴；由 04/05/10 承接）；**不含制动与驱动**（需求 17）；
   - **整车实验总成 = 前后悬架 + 转向 + 车身 + 车轮 + 制动 + 驱动 + 整车 KC 试验台**（本任务承接）。
2. **后悬架的落点**：现有 `build_vehicle`（`preparation/assembly/vehicle.py:95`）对前后轴各调一次 `build_front_axle`（`:116`）并加 `front_`/`rear_` 前缀重命名（`:135-148`）。本任务把「前后两个悬架子系统 + 车身子系统」的组装显式化为整车总成，而不是藏在 `build_vehicle` 内部。
3. **车身子系统在整车的角色**：轴侧 chassis 是固定体（`front_axle.py:644` 造 `RigidBody("chassis", fixed=True)`），整车侧由 `build_vehicle` 用 `_body_from_spec(model.chassis)` 造真车身（`vehicle.py:99`）并把轴侧 chassis 并入（`:118-128`）。本任务明确这一交接。
4. **整车 KC 试验台接入**：`vehicle_kc` 试验台（现由 `cases/vehicle_kc.py` + `VehicleKcCompiler` 承担）与整车总成组装成整车实验总成。
5. **整车侧轮胎质量发射落点**（补审核 B4）：08 只做内核侧（C++ 内核 + 契约 schema + 文档读取 + native 镜像），Python 侧把质量写进 tire entry 的发射点在整车侧是 `cases/vehicle_dynamic.py:327`（`_tire_entry`）与 `preparation/vehicle_dynamic.py` 的轮胎 spec，数据源在 `schema/vehicle.py:60` 的 `WheelSpec.mass`。这些落点归本任务补齐，使需求 10「轮胎质量归属到轮胎」在**轴侧与整车侧都成立**（轴侧归 09）。
6. **与现有行为的等价**：整车实验总成的产出必须与现役 `build_vehicle` 的产出**逐项一致**（本任务是重构，不是改物理）。
7. **接入制动与驱动子系统（需求 17 / D8）**：整车实验总成**必须**含制动与驱动（对标 Adams `acar_gs_full.asy` 的 `Major Role : brake_system` 与 `Major Role : powertrain`）。接入方式：总成声明这两个子系统在场，试验台/study 提供输入，力矩经 role 的通道下发到车轮。
   - 力矩通道沿用内核既有 role：驱动 `wheel_torque`、制动 `brake_torque`（`cpp/src/cases/vehicle_dynamic.cpp:32-33`）；现役落点是 `preparation/vehicle_dynamic.py:1439-1512` 的 `_build_wheel_torque_signals` 与 `cases/vehicle_dynamic.py:530-535` 的 blob 表发射。
   - **制动方向不得由 Python 定**：`brake_torque` 是**非负幅值**，方向由内核按轴向转速符号决定（`cpp/src/element/drive_brake.cpp:89-98`），并带转向节反作用（`:111-120`）。本步只负责按参数算出幅值。
   - **需求 20/D11**：整车组装对制动/驱动的处理**不得**因模板是简化版还是复杂版而分支；须有一条「用复杂桩模板替换简化模板、本步代码零改动」的测试。


## 非目标
- 不改 `vehicle_dynamic`/`vehicle_kc`/`handling`/`ride_*` 的物理语义与数值。
- 不重写整车动力学装配（`preparation/vehicle_dynamic.py` 的 steering/road/torque 采样、`_build_joints`、`_condense_welded_bodies`/`_fuse_welded_bodies`）——本任务只在子系统层重组，这些保持原样。
- 不做试验台正交组合的完整抽取（10 的范围）；本任务只要求整车实验总成能接上整车 KC 试验台。
- 不删除 `build_vehicle`（保留为适配器，直到 12 之后再定）。
- **转向可选性在整车侧不放开（需求 15 / D5）**：整车侧 `SteeringSystemSpec` 必填（`schema/vehicle.py:233`）、`required_bodies` 含 `rack`/`tie_rod_L`/`tie_rod_R`（`:249-259`）、`prepare_vehicle_run` 无条件调 `_build_steering`（`preparation/vehicle_dynamic.py:227`）。本任务**维持这一不对称**：单轴悬架实验总成可无转向（04 落地），整车实验总成必须有转向；不得为了"对称"而放开整车侧校验（那会波及 5 个 vehicle family 的行为与基线）。
- 不重写轮胎质量归属的内核侧实现（08 的范围）；本任务只补整车侧的发射落点。
- **不得为单轴侧引入制动/驱动**（需求 17）：单轴侧不需要制动通道，因此**不要求**改内核 `axle_dynamic.cpp` 的 `TireRole`（该表无 `BrakeTorque`，`axle_dynamic.cpp:31`）；若本步顺手给单轴侧加，即为越界。
- **需求 20/D11**：不得按模板名、有无刚体、`simplified` 之类判据分支；复杂模板落地时只允许新增模板 + 注册，不得改本步代码。

## 现状事实（制定计划时实测，实施时复核）

- `build_vehicle`（`preparation/assembly/vehicle.py:95`）：对 front/rear 各调 `build_front_axle(axle_model, mode=mode)`（`:116`），把各轴 `constraints`/`ideal_constraints`/`elements`/`connections` 经 `_rename_dataclasses`/`_rename_connections` 改名后合并（`:135-148`）。
- `VehicleAssembly`（`vehicle.py:45-62`）字段：`mode, bodies, state, points, constraints, ideal_constraints, elements, connections, wheel_specs, wheel_centers, wheel_body_names, wheel_rotations_local, axle_assemblies, body_aliases`。**无独立 bushing 字段**（衬套藏在 `elements` 里）；每轴原始 `FrontAxleAssembly` 由 `axle_assemblies` 保留（`:61`）。
- 轴间无 K/C 分支，`mode` 只是原样透传（`:116`）。
- `_condense_welded_bodies`（`:253`）与 `_fuse_welded_bodies`（`:273`）与 K/C **无关**：前者只看环境变量 `SUSPENSION_MULTIBODY_CONDENSE_WELDS`（`:268`）；后者按 `WeldJoint` 做并查集融合（`:275-305`）并同步转换 `BushingElement`（`:472-485`）。**A3 后默认不凝聚**，weld 以 `kind="fixed"` 送 native。
- `vehicle_kc` 不自己决定 K/C，直接委托 `prepare_vehicle_run`（`preparation/vehicle_kc.py:87`）。
- 整车动态的装配模式由 `_select_assembly_mode(model, case.suspension_mode)`（`preparation/vehicle_dynamic.py:221-222`）决定，`auto` → 有 bushing 数据选 C 否则 K。

## 约束

- **产出等价**：整车实验总成的输出与现役 `build_vehicle` 输出逐项一致（体集合与顺序、`points`、`constraints`、`ideal_constraints`、`elements`、`connections`、`wheel_specs`、`body_aliases`）。这是纯重构的判据。
- **保留 `build_vehicle` 签名与返回类型**（`VehicleAssembly`），与 04 保留 `build_front_axle` 的做法一致。
- **不改变整车系列基线**：`vehicle_dynamics_baseline/sha256.json`（8 case）、`case_parity_check` 的 `vehicle_dynamic` family、`vehicle_kc` family 都必须保持不变。若变化，先判断是"重构回归"还是"物理改变"——前者必须修掉，后者须单独裁决并登记。
- 本任务与 04 都触及 `preparation/assembly/vehicle.py`，**必须串行**（04 先完成）。
- 不引入新依赖。

## 范围与文件归属

- 可写：
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/**`（新增整车总成组装；04 建立的包）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/vehicle.py`（改为由子系统组合；保留 `build_vehicle` 签名与 `VehicleAssembly` 字段）
  - `packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py`（仅整车侧轮胎质量发射落点：`_tire_entry` 附近）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py`（仅轮胎 spec 的质量落点）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/__init__.py`（仅当需要导出新符号，不得改变现有导出）
  - 新增测试 `packages/suspension_multibody/tests/vehicle_assembly/**`（含整车侧转向不对称的锁定测试）
  - 本任务 `raw/**`（逐项对照表与快照证据）
- 只读：`templates/**`（03）、`joints/`（02）、`properties/**`（06）、`outputs/**`（07）、`rigs/**`（10）、`schema/**`、`cases/**`、父级计划文件。
- 不写：`outputs/**`、`report/**`（07）；C++ 内核与契约 schema（08）；`cases/vehicle_dynamic.py` 的 case 结构与非质量字段（09/10）；`cases/axle_dynamic.py` 的质量落点（09）；基线文件（预期不需要重录）。

## 依赖

- 前置：**10**（试验台已抽取；整车总成要接整车 KC 试验台）。传递覆盖 01-09。
- 后续：12（终局验收要求 Goal 含整车实验总成）。

## 验收标准

1. **前后悬架 + 车身 + 车轮 + 转向 + 制动 + 驱动可组装为整车总成**：整车实验总成 = 上述子系统 + 整车 KC 试验台，可用且有明确入口。
2. **两种组装并存**：悬架实验总成（单轴）与整车实验总成（整车）都能构造，且共享同一套子系统定义（不是两套代码）。
3. **与现役 `build_vehicle` 产出逐项一致**：逐项对照表落 `raw/`，差异为零或逐项登记理由。
4. **整车系列基线不变**：`vehicle_dynamics_baseline/sha256.json` 8 case 逐位一致；`case_parity_check.py` 的 `vehicle_dynamic`/`vehicle_kc` family 通过；若变化须判断并登记。
5. **整车侧轮胎质量发射落点补齐**：`cases/vehicle_dynamic.py` 与 `preparation/vehicle_dynamic.py` 把质量写进 tire entry；质量守恒（整车总质量与质心不变）有断言。
6. **车身子系统的角色明确**：轴侧 chassis（固定体）与整车车身（`_body_from_spec(model.chassis)`）的交接有测试证明（车身体身份、质量、重命名规则）。
7. **登记整车侧不对称并加锁定测试**：有测试断言整车模型缺少转向（`steering` 缺失或 rack/tie_rod 刚体缺失）时**仍然校验失败**，并指明这是 D5 的有意不对称；该测试是「整车侧不放开」的证据，不是待修缺陷。
8. **制动与驱动在整车总成接入（需求 17 / D8）**：整车实验总成的子系统集合含 `brake`/`drive`；有测试断言二者在场且力矩按 role 通道下发到车轮。
9. **制动幅值语义正确**：断言下发给内核的 `brake_torque` 为**非负**幅值且不自行翻转符号（方向由内核决定）；有对照现役 `_build_wheel_torque_signals` 的逐值一致断言。
10. **需求 20/D11**：用复杂桩模板替换简化制动/驱动模板后，本步（整车组装）代码零改动即可跑通。
11. 门禁：`--strict --final` 保持绿；ruff/ty 通过；`dynamic_hash_sentinel`/`kc_parity` 保持绿。

## 验证协议

1. 子系统组装落地后：跑「两种组装并存」与「整车产出逐项一致」测试。
2. 接线到 `build_vehicle` 后：跑整车系列既有测试（`tests/vehicle`、`tests/cases` 的 vehicle 部分）。
3. 接入整车 KC 试验台后：跑 `vehicle_kc` 端到端。
4. 整车侧不对称锁定测试落地后：断言缺转向的整车模型仍被拒绝，且 5 个 vehicle family 行为不变。
5. 收尾：`case_parity_check.py`（8 family）、`dynamic_hash_sentinel.py --check`、`kc_parity_check.py --check`、三套 pytest、ruff、ty、`--strict --final`、`git diff --check`。
6. 制动/驱动接入后：跑整车实验总成端到端，断言二者在场、力矩通道下发、幅值非负；并对照现役 `_build_wheel_torque_signals` 逐值一致。
7. 需求 20/D11 落地后：用复杂桩模板替换简化模板，断言整车组装代码零改动即可跑通。

**若 `vehicle_dynamics_baseline/sha256.json` 失效**：先判断是重构回归还是物理改变。前者必须修掉；后者必须单独裁决并登记——不得以"重构"为名顺手重录整车基线。
