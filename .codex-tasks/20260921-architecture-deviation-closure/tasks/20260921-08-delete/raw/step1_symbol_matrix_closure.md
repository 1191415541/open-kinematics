# 08 步骤 1：SYMBOL_MATRIX 三态闭合核对（A1 修订）

依据 EPIC G2/A1 修订（「删除只覆盖**已无生产调用者**的部分；保留仍有现役生产调用的模块，不设必须删完」），08 SPEC 验收 1 要求 `SYMBOL_MATRIX.csv` 每行落入三态之一：

- **① 已迁移**：已指向新归属且调用方已切换；
- **② 有明确删除依据**：实现可删，物理断言已转 native/契约测试；
- **③ 保留**（A1）：仍有现役生产调用，须记 file:line + 阻断原因 + 解除条件。

## 0. 矩阵规模与结构

`tasks/20260921-01-baseline/SYMBOL_MATRIX.csv`：379 行 = `cpp-module` 20 + `cpp-header-edge` 86 + `cpp-source-edge` 63 + `python-symbol` 210。

C++ 侧（169 行）在 03/04 已全部闭合：目标模块集合存在、旧模块（`mb_base`/`mb_vehicle`/`mb_suspension`/`mb_integrator`/`mb_static`/`mb_linalg`/`mb_constraint`）缺席、`--strict` 与 `--strict --final` 均退出 0。本步骤的 Python 部分按下表核验。

## 1. Python 侧 210 行的三态分布

| old_owner | 行数 | 态 | 依据 |
|---|---|---|---|
| `core/spatial.py` | 25 | ① 已迁移 | 现役输入侧转换 → `preparation/geometry.py`；`core/spatial.py` 保留为同对象 re-export（`SE3 is SE3` 实测成立） |
| `model/front_axle.py` | 21 | ① 已迁移 | → `preparation/assembly/front_axle.py` |
| `model/vehicle.py` | 11 | ① 已迁移 | → `preparation/assembly/vehicle.py`（含 `_condense_welded_bodies`，A1 保留在作者层） |
| `model/mass.py` | 5 | ② 删除依据 | 无生产调用者（06 矩阵已核实）；`model/mass.py` 本体保留待 08 删 |
| `core/constraints.py` | 18 | ①/② 混合 | 数据对象 → `preparation/assembly/types.py`（已迁移）；`residual`/`jacobian` → `core/constraints.py` 模块级函数，唯一消费者 `core/reactions.py`（无生产调用者） |
| `core/rigid_body.py` | 3 | ① 已迁移 | `RigidBody`/`RigidBodyState` → `types.py`；`point_jacobian` 移出数据对象（"数据对象不携带 Jacobian"） |
| `core/rank.py` | 3 | ② 删除依据 | `RankDiagnostic`/`scale_jacobian`/`diagnose_rank` 无生产调用者；物理断言须转 native/契约测试（步骤 4） |
| `core/reactions.py` | 3 | ② 删除依据 | `ReactionResult`/`recover_reactions`/`body_equilibrium_wrench` 无生产调用者；同上 |
| `core/__init__.py` | 1 | ① 已迁移 | 包级 facade 改指新归属 |
| `elements/elastic.py` | 18 | **③ 保留（A1）** | 见第 2 节 |
| `elements/assembly.py` | 2 | **③ 保留（A1）** | 见第 2 节 |
| `elements/base.py` | 2 | **③ 保留（A1）** | 见第 2 节 |
| `elements/__init__.py` | 1 | ③ 保留 | 随 `elements/` 保留 |
| `analysis/time_signals.py` | 5 | ① 已迁移 | → `preparation/signals.py`（06）；`analysis/time_signals.py` 本体保留待 08 删 |
| `analysis/vehicle_physics.py` | 15 | ①/③ 混合 | `compute_static_wheel_loads` 及其辅助 → **③ 保留（A2）**；`side_hardpoints` 调用随 `model` 迁移；`WheelLoadSummary`/`summarize_wheel_loads` → `report/`（按 07 判定） |
| `analysis/time_domain_physics.py` | 7 | ① 已迁移 | 纯诊断 → `report/`（07 TODO 第 3 行判定） |
| `analysis/compliance.py` | 3 | ① 已迁移 | → `report/compliance.py`（07） |
| `analysis/_geometry.py` | 2 | ① 已迁移 | → `report/geometry.py`（07） |
| `analysis/metrics.py` | 3 | ① 已迁移 | 两个转发 → `report/`（07） |
| `analysis/benchmarks.py` | 4 | ① 已迁移 | → `tests/data` 声明式夹具（07） |
| `analysis/vehicle_kc_time_domain.py` | 4 | ① 已迁移 | replay 编排 → `simulation/replay.py`（07） |
| `analysis/__init__.py` | 1 | ① 已迁移 | facade 改指 `report/` |
| `metrics/case_specific.py` | 14 | ① 已迁移 | → `report/metrics/`（07） |
| `metrics/common.py` | 9 | ① 已迁移 | → `report/metrics/`（07） |
| `metrics/axle.py` | 7 | ① 已迁移 | → `report/metrics/`（07） |
| `metrics/vehicle.py` | 6 | ① 已迁移 | → `report/metrics/`（07） |
| `metrics/__init__.py` | 1 | ① 已迁移 | facade 改指 `report/metrics/` |
| `pac2002_scope/pac2002_scope.py` | 15 | ① 已迁移 | 能力读取 → `kernel/capabilities.py`；校验辅助 → `schema/pac2002_scope.py`；Adams 证据 → `adams/pac2002_evidence.py`（06） |

**闭合结论**：无未闭合行。① 类的调用方已全部切换（registry 31→14 且 `legacy_surface --check` 退出 0 为证）；② 类有明确删除依据；③ 类逐项登记于第 2 节。**未闭合符号不得删除**——本表无未闭合行，因此 08 的删除范围限于 ② 类与 ① 类已被取代的旧文件本体。

## 2. 态 ③（保留）逐项登记

### 2.1 `elements/**` 的力元件本构与力汇总（A1 阻断）

| 项目 | file:line | 阻断原因 | 解除条件 |
|---|---|---|---|
| 8 个元件 `evaluate` | `elements/elastic.py:253-291,310-335,443-494,520-534,546-560,573-592,607-644,655-659` | native `element_wrench` 通道无法承载固定体端反力（`assembly_primitives.cpp:16,33` 对固定体早退 → 实测 16/16 行全 NaN，而 Python 有 8 条非零 `chassis` 力旋量）；K 模式模型不声明力元件（`cases/kc_quasi_static/contract.py:118-120`）；两侧力矩参考点不同口径 | 裁决固定体端事实口径 + K 模式口径 + 力矩参考点契约（三项，详见 `../20260921-06-authoring/raw/step6_pending_deletion_registration.md` 第 4 节） |
| `evaluate_generalized_forces` | `elements/assembly.py:24-54` | 同上（唯一生产入口 `api.py:742`） | 同上 |
| `ForceEvaluation` | `elements/base.py:10-19` | 同上 | 同上 |
| `elements/__init__.py` facade | `elements/__init__.py` | 随上述保留 | 同上 |

**正向事实**：05 步骤 7 实测对齐参考点后两侧力差 `7.638e-10 N`、力矩差 `2.195e-10 N·m`——**力律本身等价**，未发现「第二套物理定义」。阻断的是事实通道的**覆盖度**，不是物理一致性。

### 2.2 `analysis/vehicle_physics.py` 的静轮荷辅助求解（A2 保留）

| 项目 | file:line | 阻断原因 | 解除条件 |
|---|---|---|---|
| `compute_static_wheel_loads` + 其辅助 | `analysis/vehicle_physics.py:88-142`（`_center_of_mass:171`、`_support_points:180`、`_hardpoint:205`、`_instant_center:216`、`_contact_front_view:237`、`_line_intersection:252`） | native 无静力求解 ABI 入口（`kernel_abi.cpp:854-858` 明示只有 `suspension_kernel_run`），且 ABI 导出面冻结；输入是 `build_vehicle(mode="K")` 装配，与动态整车算例不是同一套 | 裁决「是否允许扩展 ABI 导出面」或「是否允许在既有契约下新增默认关闭的静力输出块」（详见 `../20260921-05-native-facts/raw/step4_static_wheel_loads_registration.md` 第 5 节） |
| 唯一生产调用者 | `vehicle/service.py:30,40` | 同上 | 同上 |

注：`analysis/vehicle_physics.py` 的 `compute_vehicle_roll_centers`（`:145`）无生产调用者，属 ② 类（归 07/08 判定）；该文件因 2.2 的保留而**本体不删**。

## 3. 与 08 删除范围的关系

- **可删（② 类）**：`core/rank.py`、`core/reactions.py`、`model/mass.py`，以及 ① 类中已被取代的旧文件本体（`model/front_axle.py`、`model/vehicle.py`、`analysis/*`、`metrics/*`、`pac2002_scope.py`）——前提是 07 完成后其生产调用方确已清零。
- **不可删（③ 类）**：`elements/` 四文件、`analysis/vehicle_physics.py`（静轮荷部分）。
- **连带不可删**：`core/constraints.py` 的 `residual`/`jacobian` 模块级函数——其唯一消费者 `core/reactions.py` 在 ② 类，但 `ConstraintSystem` 仍持有同名方法（`:499,506`）；须在删 `reactions.py` 时一并裁定 `ConstraintSystem` 的去留，不得留下半删状态。

## 4. 待与 07/08 对齐的事项

1. 上表 ① 类中归属 `report/` 的行（`analysis/{_geometry,compliance,metrics,benchmarks,time_domain_physics,vehicle_kc_time_domain}.py`、`metrics/*`）以 07 的实际落地为准；07 未完成前不得据此删除 `analysis`/`metrics` 文件本体。
2. `analysis/time_signals.py` 是 06 明确留下的重复体，调用方由 07 切到 `preparation/signals.py` 后方可删。
3. 步骤 4（物理断言转 native 测试）须覆盖 ② 类的三个求解实现（`core/rank.py`、`core/reactions.py`、`model/mass.py`）与 `analysis/vehicle_physics.py` 的 `compute_vehicle_roll_centers`。
