# 06 步骤 1：作者层迁移矩阵与调用者冻结

依据：EPIC「Python 迁移矩阵」、`VALIDATION.md:284-302`「报告契约缺口」表、以及本轮对 `packages/suspension_multibody` 的逐符号只读勘察（生产调用者 = `src/` 与 `scripts/` 内引用，不含 `tests/`）。本表是 06 后续步骤的删除/迁移判据：**未闭合行不得删除**。

## 0. 前置事实（影响本任务的执行边界）

- `kernel/capabilities.py` **不存在**（`kernel/` 现只有 `__init__.py`、`native.py`、`solver.py`）。内核能力读取的实现实际在 `pac2002_scope.py:170-215` 的 `_kernel_scope()`（`@lru_cache`，内部 `from .kernel.native import load_library` 后读 `suspension_kernel_capabilities`），惰性常量经 `__getattr__`（`pac2002_scope.py:229-247`）解析。故「能力读取迁 `kernel/capabilities.py`」是**新建文件**，不是移动既有文件。
- `preparation/__init__.py:16` 当前 `__all__: tuple[str, ...] = ()`（零导出），不存在需迁出的公开名字。
- `preparation/` 7 个模块中只有 2 个触及旧归属：`kc_quasi_static.py:25`（`..model`）、`vehicle_dynamic.py:37-58`（`..core`、`..elements`、`..model`）；其余 5 个已是纯 `axle_dynamics.schema` / `cases` / `schema` / `simulation` 依赖。
- 02 门禁 `tests/architecture/legacy_surface_gate.py` 的 registry（`legacy_surface_registry.json`，70 条）**只能收缩**；迁移模式容忍已登记项，`--final` 拒绝一切旧导入与旧包。

## 1. model/ 作者层装配

| 符号 | 定义 | 生产调用者（src/） | 新归属 | 处置 |
|---|---|---|---|---|
| `build_front_axle` | `model/front_axle.py:618` | `api.py:51,101,188`；`preparation/kc_quasi_static.py:25,78`；`model/vehicle.py:31,106`；`adams/reference.py:14,69`；`adams/strict_c.py:35,152`；`adams/strict_k.py:16,225` | `preparation/assembly/front_axle.py` | 迁移（调用方同步改写） |
| `FrontAxleAssembly` | `model/front_axle.py:76` | `api.py:51,403,486,566,592,615,738`；`preparation/kc_quasi_static.py:25,59,73,75,80`；`model/vehicle.py:29,51,96` | 同上 | 迁移 |
| `Connection` | `model/front_axle.py:64` | 仅 `model/` 包内 + `model/vehicle.py:28` | 同上 | 迁移（包内符号） |
| `side_hardpoints` | `model/front_axle.py:54` | `adams/strict_c.py:35,943`；`analysis/vehicle_physics.py:11,206` | 同上 | 迁移 |
| `mirror_hardpoints` | `model/front_axle.py:46` | **无生产调用者**（仅 `model/__init__.py:7` 再导出） | 同上 | 迁移后按 08 判定 |
| `build_vehicle` | `model/vehicle.py:85` | `preparation/vehicle_dynamic.py:58,222`；`analysis/vehicle_physics.py:10,110,151`；`adams/full_vehicle_model.py:3165,3167`（惰性） | `preparation/assembly/vehicle.py` | 迁移 |
| `VehicleAssembly` | `model/vehicle.py:36` | `preparation/vehicle_dynamic.py:58,330,393,591,621,701,976,1168`；`analysis/vehicle_physics.py:10,171,181`；`analysis/time_domain_physics.py:10,146,161,183` | 同上 | 迁移 |

scripts 侧另有 4 个脚本直连 `suspension_multibody.model`：`case_parity_check.py:297,316,334`、`kc_native_c_probe.py:39,168`、`kc_native_probe.py:27,98`、`kc_perf_gate.py:62,72,74`——与本任务同步改写，并相应收缩 registry。

**结论**：`model/` 的 7 个符号全部有明确新归属且调用者已闭合；`mirror_hardpoints` 无生产调用者但保留迁移（不作为独立删除判据）。

## 2. core/ 数据对象（迁移到 `preparation/assembly/types.py`）

| 符号 | 定义 | 生产调用者 | 处置 |
|---|---|---|---|
| `PointCoincidence` | `core/constraints.py:127` | `preparation/vehicle_dynamic.py:44,627` | 数据对象迁 `types.py` |
| `BallJoint` | `:152` | 构造 `model/front_axle.py:280,713,778,816,839,842`；`cases/kc_quasi_static/convert.py:7,93`；isinstance `preparation/vehicle_dynamic.py:627` | 同上 |
| `WeldJoint` | `:159` | 构造 `model/front_axle.py:282,444,875`；isinstance `model/vehicle.py:244,251,394`；`preparation/vehicle_dynamic.py:629` | 同上 |
| `DistanceConstraint` | `:191` | isinstance `preparation/vehicle_dynamic.py:42,643` | 同上 |
| `RevoluteJoint` | `:224` | 构造 `model/front_axle.py:284,699,764`；`model/vehicle.py:14,638`；`cases/kc_quasi_static/convert.py:7,110`；isinstance `cases/kc_quasi_static/contract.py:33,113`；`preparation/vehicle_dynamic.py:631` | 同上 |
| `UniversalJoint` | `:267` | 构造 `model/front_axle.py:288`；isinstance `preparation/vehicle_dynamic.py:635` | 同上 |
| `ConstantVelocityJoint` | `:314` | 构造 `model/front_axle.py:292`；isinstance `preparation/vehicle_dynamic.py:637` | 同上 |
| `CylindricalJoint` | `:376` | 构造 `model/front_axle.py:301`；isinstance `preparation/vehicle_dynamic.py:639` | 同上 |
| `InPlaneJoint` | `:437` | 构造 `model/front_axle.py:303`；isinstance `preparation/vehicle_dynamic.py:641` | 同上 |
| `PrismaticJoint` | `:477` | 构造 `model/front_axle.py:286,467,885`；isinstance `cases/kc_quasi_static/contract.py:113`；`preparation/vehicle_dynamic.py:633,1252` | 同上 |
| `CoordinateDrive` | `:548` | isinstance `preparation/vehicle_dynamic.py:40,643` | 同上 |
| `RigidBody` | `core/rigid_body.py:13` | `api.py:42,598`；`elements/assembly.py:17`；`elements/elastic.py:10`；`model/front_axle.py:19,80`；`model/mass.py:18` | 同上 |
| `RigidBodyState` | `core/rigid_body.py:48` | `api.py:43,566,593,609,613,614,738`；`elements/assembly.py:17,25,40`；`elements/elastic.py:10,215`；`model/front_axle.py:20,81` | 同上 |
| `ConstraintSystem` | `core/constraints.py:567` | 仅 `core/reactions.py:9,24`（`reactions` 自身无生产调用者）；实例化仅存在于 `tests/core/` | 不迁（随 `reactions.py` 归 08 删除判定） |

**`residual` / `jacobian`**（必须不随数据对象保留）：

- 定义散落各 joint 类内：`residual` 于 `constraints.py:118,136,168,201,235,278,328,387,447,488,557` 与 `:586`（`ConstraintSystem`）；`jacobian` 于 `:122,144,178,207,246,288,342,400,455,513,561` 与 `:593`。
- 生产期唯二消费者：`core/constraints.py:590`（`ConstraintSystem.residual`）、`:601`（`ConstraintSystem.jacobian`）、`core/reactions.py:33,43`（`constraint.residual`）、`:44`（`constraint.jacobian`）。
- `core/reactions.py` 对外函数 `recover_reactions`（`:23`）、`body_equilibrium_wrench`（`:61`）**无生产调用者**（仅 `tests/core/test_reactions.py:11,12,13,32,33`）。

**结论**：数据对象全部有归属；`residual`/`jacobian` 迁入 `types.py` 时**不得携带**，其唯一生产消费者是即将删除的 `reactions.py`/`ConstraintSystem`。

## 3. core/spatial.py 的输入侧 / 结果侧拆分

| 符号 | 定义 | 调用者 | 侧别 |
|---|---|---|---|
| `SE3` | `spatial.py`（广泛使用） | `model/*`、`api.py`、`preparation/*` | 两面共用（保留在 `core` 之外的中性位置：`preparation/geometry.py` 与 `results/geometry.py` 各自需要的部分） |
| `cross3` | `:58` | `constraints.py:22,77,78,100,244,508`；`rigid_body.py:9,117`；`elements/elastic.py:13,227,525` | 输入侧 |
| `skew` | `:43` | `constraints.py:26,102,108,256-259,523-526`；`model/mass.py:19,48` | 输入侧 |
| `quaternion_*` / `rotation_vector_to_quaternion` | `:91,:98,:104,:132,:209` | `constraints.py:23,24,25,172,173,176,498,499,501`；`api.py:44,625`；`elements/elastic.py:14,15,411,412`；`adams/strict_c.py:31,32,33,289,290,295`；`analysis/vehicle_kc_time_domain.py:8,100`；`scripts/case_parity_check.py:921,994,997` | 输入侧为主，`api.py:625`/`vehicle_kc_time_domain.py:100` 属结果侧表述 |
| `wrench_global_to_local` | `spatial.py` | `elements/assembly.py:18,51`（装配广义力输入）；`api.py:45,751`（报告 local_load） | **跨两侧**——本任务须按调用点分别归位 |
| `wrench_local_to_global` | `spatial.py` | 仅 `core/reactions.py:11,55` | 结果侧（唯一调用者在无生产调用者的文件中） |
| `normalize_quaternion` | `:65` | 仅 `spatial.py` 内部 | 内部 |
| `twist_local_to_global` / `wrench_translation_tangent` | `:380,:398` | 仅 `tests/core/test_spatial.py:8,11,43,50` | 无生产调用者 |
| `wrench_matrix` | `:390` | **无任何调用者** | 无生产调用者 |

**结论**：结果侧现役只有 `wrench_local_to_global`，且其唯一调用点位于无生产调用者的 `reactions.py`——因此 `results/geometry.py` 的实际需求限于报告侧变换（`api.py:751` 的 `wrench_global_to_local`）。调用方向必须复核：`report`（07 新建）不得反向调用 `preparation`。

## 4. analysis/time_signals.py（迁 `preparation/signals.py`）

| 符号 | 定义 | 调用者 | 处置 |
|---|---|---|---|
| `time_grid` | `analysis/time_signals.py:10` | `api.py:35,193`；`analysis/vehicle_kc_time_domain.py:20,48` | 迁 `preparation/signals.py` |
| `motion` | `:22` | `api.py:35,190,191,192` | 同上 |
| `loads_at_time` | `:37` | `api.py:35,253`；`analysis/vehicle_kc_time_domain.py:20,45` | 同上 |
| `wrenches_at_time` | `:42` | `api.py:35,211` | 同上 |
| `target_body` | `:57` | 仅 `time_signals.py:46`（模块内） | 同上（内部辅助） |

**结论**：属输入信号采样，非报告；调用者 2 处（`api.py`、`vehicle_kc_time_domain.py`），本任务同步改写。`analysis` 目录本体删除归 08。

## 5. pac2002_scope.py（能力读取 → `kernel/capabilities.py`）

| 符号 | 定义 | 生产调用者 | 新归属 |
|---|---|---|---|
| `validate_pac2002_native_scope` | `pac2002_scope.py:304` | `schema/dynamic.py:10,268`；`axle_dynamics/schema.py:11,998`；`scripts/run_native_tire_rig.py:49,705` | schema 校验辅助 → `schema/` |
| `pac2002_unsupported_native_reasons` | `:264` | `adams/full_vehicle_model.py:28,638`；`scripts/audit_pac2002_tire_scope.py:33,68`；tests 多处 | Adams 证据说明 → `adams/` |
| `PAC2002_NATIVE_IMPLEMENTED_FEATURES` | `:72` | `adams/full_vehicle_model.py:25,650` | 同上 |
| `PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES` | `:120` | `adams/full_vehicle_model.py:26,651` | 同上 |
| `PAC2002_SUPPORTED_NATIVE_USE_MODES` | **无落盘定义**（`_KERNEL_SOURCED` 惰性） | `adams/full_vehicle_model.py:27,653`；`scripts/audit_pac2002_tire_scope.py:31,107` | 能力读取 → `kernel/capabilities.py` |
| `_kernel_scope` | `:170`（lru_cache；`:184-215` 读 `suspension_kernel_capabilities`） | `pac2002_scope.py:233,272` | `kernel/capabilities.py`（保持惰性） |
| `__getattr__` | `:229` | 惰性常量访问 | 随能力读取迁移 |

**结论**：能力读取（`_kernel_scope` + 惰性常量 + `__getattr__`）迁 `kernel/capabilities.py` 且**保持惰性**；schema 校验辅助与 Adams 证据说明分别归 `schema` 与 `adams`；旧顶层模块文件本体的删除归 08。不得扩展支持模式。

## 6. elements 力元件（A1 口径：待删除登记）

| 符号 | 定义 | 生产调用者 |
|---|---|---|
| `LinearSpringElement.evaluate` | `elements/elastic.py:253-291` | 经 `evaluate_generalized_forces` |
| `StaticDamperElement.evaluate` | `:310-335` | 同上 |
| `BushingElement.evaluate` | `:443-494` | 同上 |
| `PointWrenchElement.evaluate` | `:520-534` | 同上 |
| `VerticalTireElement.evaluate` | `:546-560` | 同上 |
| `AntiRollBarElement.evaluate` | `:573-592` | 同上 |
| `BumpStopElement.evaluate` | `:607-644` | 同上 |
| `GravityElement.evaluate` | `:655-659` | 同上 |
| `evaluate_generalized_forces` | `elements/assembly.py:24-54`（`__all__:21`） | **唯一入口 `api.py:47,742`** |
| `ForceEvaluation` | `elements/base.py:10-19` | `elements/assembly.py:19,29,35,41,42`；`elements/elastic.py:17` + 各 `evaluate` 返回 |

**结论（结合 05 步骤 7 实测）**：这条通道在 `api.py:742` 之后无更深下游。05 步骤 7 已实测 native 通道与 Python 力律存在**结构性差异**（固定体端无事实、承载端力矩差 0.1%–0.29%、K 模式无元件事实），据此**阻断切换**，删除条件未满足。故本任务第 6 项按 A1 处置：**保留 + 显式待删除登记**（登记见 `raw/step6_pending_deletion_registration.md`），不得删除。另注意 `preparation/vehicle_dynamic.py:50-57` 也 import 这些元件类（生产装配侧），随第 2 项一并对齐。

## 7. 未闭合项（不得删除）

| 项目 | 状态 | 原因 |
|---|---|---|
| `elements/elastic.py`、`elements/assembly.py` 的力律 | **保留** | 05 步骤 7 阻断（见第 6 节） |
| `core/constraints.py` 的 `residual`/`jacobian` | 迁 `types.py` 时剥离（不携带）；`reactions.py` 删除归 08 | 消费者仅 `reactions.py`/`ConstraintSystem` |
| `core/rank.py`（`RankDiagnostic`/`scale_jacobian`/`diagnose_rank`）、`core/reactions.py`、`model/mass.py`（`BodyMassProperties`/`body_mass_properties`/`mass_matrix`/`spatial_bias_wrench`） | 无生产调用者 | 实现删除归 08，物理断言转 native 测试 |
| `analysis/benchmarks.py`、`analysis/time_domain_physics.py`、`analysis/vehicle_physics.py` 的 `compute_vehicle_roll_centers`、`metrics/case_specific.py` 的 `case_metric_for`/`registered_case_metric_families`、`metrics/common.py` 的 `residual_norm` | 无生产调用者 | 归 07/08 判定（不以「无内部调用」擅自删除公开能力） |
