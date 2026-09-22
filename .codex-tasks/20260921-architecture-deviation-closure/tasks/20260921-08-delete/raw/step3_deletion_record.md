# 08 步骤 3：删除记录、保留登记与两处口径冲突

依据 EPIC A1/A2 修订（「删除只覆盖**已无生产调用者**的部分；仍有现役生产调用的模块保留，不设必须删完」）执行。
本文件是 08 SPEC 验收 3 要求的「保留项登记」载体：**删除后扫描的每一处残留都能在本文件找到条目**。

## 1. 删除清单

删除范围严格限于任务书列出的条目，未扩大。调用方清零依据来自
`raw/step1_symbol_matrix_closure.md`（矩阵三态）与删除前扫描 `raw/scan_before_head.json`。

### 1.1 无生产调用者的求解实现（② 类）

| 文件 | 行数 | 清零依据 |
|---|---|---|
| `src/suspension_multibody/core/rank.py` | 60 | `RankDiagnostic`/`scale_jacobian`/`diagnose_rank` 的唯一调用方是 `core/__init__.py` facade 与 `tests/core/test_reactions.py`；无生产调用 |
| `src/suspension_multibody/core/reactions.py` | 65 | `ReactionResult`/`recover_reactions`/`body_equilibrium_wrench` 同上；生产侧无导入（`grep -rn "recover_reactions\|body_equilibrium_wrench"` 仅命中自身、facade 与该测试） |
| `src/suspension_multibody/model/mass.py` | 95 | `mass_matrix`/`spatial_bias_wrench`/`body_mass_properties`/`BodyMassProperties` 无生产调用者（06 矩阵已核实），唯一调用方是 `tests/model/test_dynamic_mass.py` |

### 1.2 已被 06/07 取代的旧归属文件本体（① 类）

| 文件 | 现归属 | 调用方切换证据 |
|---|---|---|
| `src/suspension_multibody/model/{__init__,front_axle,mass,vehicle}.py` | `preparation/assembly/{front_axle,vehicle,types}.py`（06） | 生产侧无 `..model` 导入（唯二调用方 `analysis/vehicle_physics.py:10-11` 本步已切到 `..preparation.assembly`）；`model/__init__.py` 无残留生产导入 |
| `src/suspension_multibody/metrics/{__init__,axle,case_specific,common,vehicle}.py` | `report/metrics/`（07） | `api.py`、`axle_dynamics/contract_run.py`、`vehicle/service.py` 已由 07 切走（`legacy_surface_registry.json` 记录），本步复扫为 0 |
| `src/suspension_multibody/pac2002_scope.py` | `kernel/capabilities.py` + `schema/pac2002_scope.py` + `adams/pac2002_evidence.py`（06） | 全仓无 `suspension_multibody.pac2002_scope` 导入；活代码走 `schema/pac2002_scope` |
| `src/suspension_multibody/analysis/_geometry.py` | `report/geometry.py`（07） | `adams/reference.py`、`cases/kc_quasi_static/workflow.py` 已切走 |
| `src/suspension_multibody/analysis/compliance.py` | `report/compliance.py`（07） | `api.py` 已切走 |
| `src/suspension_multibody/analysis/metrics.py` | `report/metrics/`（07） | 同上 |
| `src/suspension_multibody/analysis/benchmarks.py` | `tests/data/benchmark_axle.json` + `tests/benchmark_fixture.py`（07） | 门禁脚本直接读 JSON，不再导入模块 |
| `src/suspension_multibody/analysis/time_domain_physics.py` | `report/time_domain_physics.py`（07） | 无生产导入 |
| `src/suspension_multibody/analysis/vehicle_kc_time_domain.py` | `simulation/replay.py`（07） | `api.py` 已切走 |
| `src/suspension_multibody/analysis/time_signals.py` | `preparation/signals.py`（06） | `analysis/vehicle_kc_time_domain.py:20` 已由 07 切到 `preparation.signals`；本步复扫为 0 |
| `packages/suspension_multibody/tests/core/test_reactions.py` | 物理断言转 native 契约测试 | 见 `raw/step4_assertion_coverage.md` |

`analysis/` 与 `elements/` 的**包目录本体保留**（其保留理由见第 2 节），因此 SPEC 目标 1 中「整目录删除」不适用于这两者——这正是 A1 修订的内容。

### 1.3 删除后清理的生成物

`src/suspension_multibody/{model,metrics}/` 删除后仅剩 `__pycache__`（未跟踪生成物），已一并删除，
否则 `legacy_surface_gate.py:present_legacy_packages()` 仍会因目录存在而报 `legacy package still present`。

## 2. 保留登记（file:line + 阻断原因 + 解除条件）

### 2.1 `elements/**`（A1）

| 项目 | file:line | 阻断原因 | 解除条件 |
|---|---|---|---|
| 8 个元件 `evaluate` | `elements/elastic.py:253,310,443,520,546,573,607,655` | native `element_wrench` 通道无法承载固定体端反力（`packages/suspension_kernel/cpp/src/element/assembly_primitives.cpp:16,33` 对固定体早退） | 裁决固定体端事实口径 + K 模式力元件声明 + 力矩参考点契约 |
| `evaluate_generalized_forces` | `elements/assembly.py:24` | 同上 | 同上 |
| `ForceEvaluation` | `elements/base.py:11` | 同上 | 同上 |
| 生产调用者 | `api.py:49`、`preparation/assembly/front_axle.py:17`、`preparation/assembly/vehicle.py:20`、`preparation/vehicle_dynamic.py:37` | 同上 | 同上 |

### 2.2 `core/**`（A1 连带保留）

| 项目 | file:line | 阻断原因 | 解除条件 |
|---|---|---|---|
| `core/spatial.py`（整个模块） | 被 `elements/assembly.py:18`、`elements/elastic.py:11` 导入 | `elements/` 本构依赖 spatial 代数；删除会连带破坏 A1 保留项 | 与 2.1 同时解除 |
| `core/rigid_body.py`（整个模块） | 被 `elements/assembly.py:17`、`elements/elastic.py:10` 导入；`point_jacobian` 被 `core/constraints.py` 使用 | 同上 | 同上 |
| `core/constraints.py` 声明 re-export + `residual`/`jacobian` | `core/constraints.py:168`（`residual`）、`:279`（`jacobian`） | 任务书把 `core/constraints.py` 列为「本体保留」；其模块级函数的消费者不止 `core/reactions.py`——`ConstraintSystem` 自身就是消费者（见第 3 节） | 与 2.1 同时解除，并把 residual/Jacobian 内核迁到 native `mb_joint` |
| `ConstraintSystem` | `core/constraints.py:481`，方法 `:500`（`residual`）、`:507`（`jacobian`） | 无生产调用者，但任务书「只删下列、不要扩大范围」的删除清单不含 `constraints.py`；删除它必须连 `tests/core/test_constraints.py` 的断言一起处置 | 与 2.1 同时解除 |
| `core/__init__.py` facade | 已改为只导出仍存在的内容（去掉 `RankDiagnostic`/`ReactionResult`/`diagnose_rank`/`recover_reactions`/`scale_jacobian`/`body_equilibrium_wrench`） | — | — |

### 2.3 `analysis/vehicle_physics.py`（A2）

| 项目 | file:line | 阻断原因 | 解除条件 |
|---|---|---|---|
| `compute_static_wheel_loads` + 辅助 | `analysis/vehicle_physics.py:58`（辅助 `:141,150,175,186,207,222`） | native 无静力求解 ABI 入口（`kernel_abi.cpp` 只导出 `suspension_kernel_run`），且 ABI 导出面冻结；输入是 `build_vehicle(mode="K")` 装配 | 裁决「是否允许扩展 ABI 导出面」或「是否允许新增默认关闭的静力输出块」 |
| 唯一生产调用者 | `vehicle/service.py:30,40` | 同上 | 同上 |
| **`compute_vehicle_roll_centers`** | `analysis/vehicle_physics.py:115`，调用 `build_vehicle(...)` 于 `:121` | **无生产调用者**，但迁 `report/` 会引入 `report -> preparation` 反向边（report 边界禁止调用 preparation，见 `legacy_surface_gate.py` 的 `report_preparation_call`） | 给滚转中心几何一个不依赖 preparation 的输入（仅硬点数据），或让它读 native 的几何事实 |
| `analysis/__init__.py` facade | 已改为只导出 `RollCenterResult`/`StaticWheelLoadResult`/`compute_static_wheel_loads`/`compute_vehicle_roll_centers` | 随 2.3 保留 | 同上 |
| `WheelLoadSummary`/`summarize_wheel_loads` 的重复定义 | 本步已删除 `analysis/vehicle_physics.py` 中的副本，改为从 `report/wheel_loads.py` 导入（`:24`） | 同一对象不留两份定义 | — |
| `wheel_load_metrics` 兼容转发 | 本步已删除（原 `analysis/vehicle_physics.py:81-85`） | 「不留转发壳」：全仓无调用者 | — |

### 2.4 测试侧残留（保留包的覆盖测试）

| 项目 | file:line | 说明 |
|---|---|---|
| `tests/core/test_constraints.py` | `:5`、`:20` | 覆盖保留的 `core/constraints.py`（`ConstraintSystem` 与残差/Jacobian 内核） |
| `tests/core/test_rigid_body.py` | `:5`、`:11` | 覆盖保留的 `core/rigid_body.py`（含 `point_jacobian`） |
| `tests/core/test_spatial.py` | `:5` | 覆盖保留的 `core/spatial.py` |
| `tests/elements/test_elements.py` | `:6`（core）、`:13`（elements） | 覆盖 A1 保留的力元件本构 |
| `tests/model/test_force_assembly.py` | `:5` | 覆盖 A1 保留的元件构造（`elements` 导入） |
| `tests/physics/test_vehicle_physics.py` | `:5` | 覆盖 A2 保留的 `compute_static_wheel_loads`/`compute_vehicle_roll_centers`（任务书要求该文件保留不动） |

`tests/model/`、`tests/metrics/`、`tests/analysis/`、`tests/results/` 等**目录本体保留**，
只把导入切到新归属；目录名不随包名改动（SPEC 非目标：不删除测试目录本身）。

## 3. `core/constraints.py` 的裁定：保留 `ConstraintSystem`（不留半删状态）

任务书要求「删 `reactions.py` 时一并裁定 `ConstraintSystem` 的去留，不得留下半删状态」。裁定如下：

- **保留** `core/constraints.py` 全文（含模块级 `residual:168`/`jacobian:279` 与 `ConstraintSystem:481`）。
- 理由 1：任务书把 `core/constraints.py` 与 `core/rigid_body.py`、`core/spatial.py` 一并列为「本体保留」，
  且第 3 步删除清单「只删下列」不含该文件；删除它属于扩大范围。
- 理由 2：任务书的前提「`residual`/`jacobian` 的唯一消费者是 `core/reactions.py`」与实测不符——
  `ConstraintSystem.residual:500`/`jacobian:507` 就是这两个函数的消费者。若只删模块级函数而保留
  `ConstraintSystem`，模块立刻 `NameError`，那才是「半删状态」。因此二者只能同去同留。
- 理由 3：`ConstraintSystem` 确无生产调用者，其消费者是保留的 `tests/core/test_constraints.py`。
  按 A1，无生产调用者但被测试覆盖的公开能力属于「登记后保留」，而非擅自删除（SPEC 禁止以「无内部调用」删公开能力）。
- 解除条件：`core/spatial.py`/`core/rigid_body.py` 随 2.1 一起解除时，把残差/Jacobian 内核迁到
  native `mb_joint`，同时删除 `core/constraints.py` 与 `tests/core/test_constraints.py`。

**未完成项（明确记录）**：任务书「删除 `core/constraints.py` 的 `residual`/`jacobian` 模块级函数」这一条**未执行**，
原因是它与同一条约束的「`core/constraints.py` 本体保留」「不得半删」「只删下列」三处冲突（见上述理由）。
这是需要主代理裁决的口径冲突，不是遗漏。

## 4. `--check --final` 与 A1 的口径冲突（需主代理裁决，不得放宽门禁）

`uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check --final` **退出 1**，残留为：

| 类别 | 条目 | 是否 A1/A2 保留项 |
|---|---|---|
| `legacy package still present` | `src/suspension_multibody/analysis`、`.../core`、`.../elements` | 是（第 2 节） |
| `legacy_module_import`（test scope） | `tests/core/test_constraints.py:5,20`、`tests/core/test_rigid_body.py:5,11`、`tests/core/test_spatial.py:5`、`tests/elements/test_elements.py:6,13`、`tests/model/test_force_assembly.py:5`、`tests/physics/test_vehicle_physics.py:5` | 是（第 2.4 节） |

结论：**`--check --final` 的终局口径（`LEGACY_PACKAGES = (core, elements, model, analysis, metrics)`
一个目录都不许存在、任何 scope 的旧导入都失败）与 A1「不设必须删完」直接冲突**。

- 该门禁的自身测试也把冲突写死了：`tests/architecture/test_legacy_surface_gate.py:114-119`
  （`test_migration_mode_passes_and_final_mode_fails_on_the_live_tree`）**断言 final 模式必须退出 1**
  且输出含 `legacy package still present`。因此「让 `--final` 退出 0」与「`tests/architecture` 全绿」
  二者不可同时成立。
- 本任务**未**放宽门禁、未加豁免、未改 `LEGACY_PACKAGES`、未改该测试。这是需要主代理裁决的偏差：
  可选口径是「A1 之后 `--final` 只应拒绝已无生产调用者的旧包（`model`/`metrics`，现已为零）并接受
  `core`/`elements`/`analysis` 的登记保留」，但这属于改门禁语义，超出 fixer 的授权。
- 就本任务的删除目标而言，`--final` 的**非保留项**已全部消失：`model` 与 `metrics` 既不在
  `present_legacy_packages()` 结果里，也没有任何导入。

## 5. 扫描器口径问题：`pac2002_scope` 裸词误报（1 条）

删除后扫描的唯一非保留项残留是
`packages/suspension_multibody/README.md:59 [text_reference/pac2002_scope]`。

- 该文本实际引用的是**活模块** `suspension_multibody.schema.pac2002_scope`（本步把原文的裸 `` `pac2002_scope` ``
  修正为 `` `schema/pac2002_scope` ``，以反映 06 之后的新归属）。
- 扫描器 `legacy_reference_scan.py:107-109` 的文本规则是「包限定前缀」+ `pac2002_scope` 裸词例外
  （理由见其 docstring 33-39 行：该词无英文读法）。该规则在 `pac2002_scope` 同时命名一个**活模块**之后
  无法区分两者，因此这是一处**误报**，不是保留项，也不是遗漏。
- 按 SPEC「不为了让扫描通过而放宽扫描范围或加豁免名单」，本任务**未**改扫描器、**未**为通过而复写文档。
  处置建议（需主代理裁决）：要么接受这 1 条误报并在 09 的零残留判据里排除 `schema/pac2002_scope` 形态，
  要么由扫描器把「包限定前缀」规则推广到 schema 子模块（同样属于改扫描器，未授权）。
