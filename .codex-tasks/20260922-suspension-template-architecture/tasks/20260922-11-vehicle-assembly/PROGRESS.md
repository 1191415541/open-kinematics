- 任务：整车实验总成组装（前后悬架 + 转向 + 车身 + 轮胎 + 整车 KC 试验台）
- 形态：single-full（Epic 子任务）
- 进度：0/13 步骤 TODO，尚未实施
- 当前：未开工。前置 10（试验台抽取）未完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-11-vehicle-assembly/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 10 已完成：试验台已抽取，可接整车 KC 试验台。
- 04 已完成：单轴六类子系统（左右悬架/转向/车轮/车身/制动/驱动）已拆出并可独立实例化。
- 05 已完成：模板实例化与 K/C 列激活可用。
- **本任务与 04 都触及 `preparation/assembly/vehicle.py`，必须串行（04 先完成）**。

## 本任务补的是什么（审核发现的缺口）

首次审核（`code-reviewer cac8e120`）指出：用户需求第 13 条的**后半段**——「前后悬架加转向加车身加轮胎加整车 kc 实验台得到**整车实验总成**」——在首版计划里无人承接。04 只从前轴 `build_front_axle` 拆四类子系统（没有后悬架），10 明确声明「不做总成组合」，05 无 vehicle 组装落点。因此本任务被补入，编号 11，原终局验收顺移为 12。

## 本步对应的用户需求原文

> 「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到悬架实验总成，前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成」

需求第 3 条亦提到：「vehicle_kc 是整车总成（**整车总成又相当于两个单轴总成+其他子总成**）+悬架 KC 试验台」。

## 本任务的现状事实（制定计划时实测，实施时复核）

- `build_vehicle`（`preparation/assembly/vehicle.py:95`）：对 front/rear 各调一次 `build_front_axle(axle_model, mode=mode)`（`:116`）。
- 装配合并与重命名：各轴 `constraints`/`ideal_constraints`/`elements`/`connections` 经 `_rename_dataclasses`/`_rename_connections` 加 `front_`/`rear_` 前缀后合并（`:135-148`）。
- `VehicleAssembly`（`:45-62`）字段：`mode, bodies, state, points, constraints, ideal_constraints, elements, connections, wheel_specs, wheel_centers, wheel_body_names, wheel_rotations_local, axle_assemblies, body_aliases`。**无独立 bushing 字段**（衬套藏在 `elements` 里）；每轴的原始 `FrontAxleAssembly` 由 `axle_assemblies` 保留。
- 轴间无 K/C 分支，`mode` 原样透传（`:116`）。
- 车身体交接：轴侧 chassis 是固定体（`front_axle.py:644` 的 `RigidBody("chassis", fixed=True)`）；整车侧由 `build_vehicle` 用 `_body_from_spec(model.chassis)` 造真车身（`:99`），再把轴侧 chassis 并入（`:118-128`）。
- `_condense_welded_bodies`（`:253`）与 `_fuse_welded_bodies`（`:273`）**与 K/C 无关**：前者只看 `SUSPENSION_MULTIBODY_CONDENSE_WELDS`（`:268`）；后者按 `WeldJoint` 并查集融合（`:275-305`）并同步转换 `BushingElement`（`:472-485`）。A3 后默认不凝聚，weld 以 `kind="fixed"` 送 native。
- `vehicle_kc` 不自己决定 K/C，委托 `prepare_vehicle_run`（`preparation/vehicle_kc.py:87`）。
- 整车装配模式由 `_select_assembly_mode(model, case.suspension_mode)`（`preparation/vehicle_dynamic.py:221-222`）决定。

## 本步的放行 gate

1. **与现役 `build_vehicle` 产出逐项一致**：这是纯重构的判据；对照表落 `raw/`。
2. **整车系列基线不变**：`vehicle_dynamics_baseline/sha256.json`（8 case）逐位一致；`case_parity_check` 的 `vehicle_dynamic`/`vehicle_kc` family 通过。
3. **两种组装共享同一套子系统定义**：悬架实验总成（单轴）与整车实验总成（整车）不得是两套代码。

## 基线重录台账（实施时填写；本步预期不应重录）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `vehicle_dynamics_baseline/sha256.json` | **应为否** | — | — | — | 纯重构，产出应逐项一致 |
| `case_parity_check.py` 的 vehicle family | **应为否** | — | — | — | 同上 |
| `dynamic_hash_baseline.json`（26 artifact） | **应为否** | — | — | — | axle 侧不受本步影响 |

**若整车基线失效**：先判断是"重构回归"还是"物理改变"。前者必须修掉；后者必须单独裁决并登记——**不得以"重构"为名顺手重录整车基线**。

## 全量套件基线（主代理实测）

`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` → `783 passed, 1 skipped, 1 xfailed`（退出 0）。

## 下一步

等 10 完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 11 行状态由主代理回填。
