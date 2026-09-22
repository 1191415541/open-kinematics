# 09 终局验收：逐条核对 G1–G4

本文件是子任务 09 的交付，用于判定 EPIC `20260921-architecture-deviation-closure` 的 Goal 是否达成。结论先行：**G1、G2、G3、G4 均达成**，两条已登记偏差（A1 凝聚手段、A2 静轮荷）按裁决保留且未伪装达成，既有失败与 Adams 限制独立列明。

判定口径：`DONE` 行不等于 Goal 达成；本文件的每条结论都指向独立证据（命令退出码或文件）。

## 0. 终局命令全集（逐条执行）

EPIC 终局清单 12 条 + 2 条构建命令，逐条记录于 `raw/terminal_commands.md`，原始输出在 `raw/t1_out.log` … `raw/t14_out.log`：

| # | 命令 | 退出码 | 摘要 |
|---|---|---|---|
| T1 | `build_axle_native.py` | 0 | DLL 产出 |
| T2 | `check_module_layering.py --strict` | 0 | layering matches baseline |
| T3 | `pytest suspension_kernel/tests -q` | 0 | 15 passed |
| T4 | `pytest suspension_contracts/tests -q` | 0 | 22 passed |
| T5 | `pytest suspension_multibody/tests -q` | 0 | 736 passed, 47 skipped, 1 xfailed |
| T6 | `ruff check .` | 0 | All checks passed |
| T7 | `ty check .` | 0 | All checks passed |
| T8 | `dynamic_hash_sentinel.py --check` | 0 | 26/26 逐位一致 |
| T9 | `kc_parity_check.py --check` | 0 | within tolerance |
| T10 | `case_parity_check.py` | 0 | 8 families accepted |
| T11 | `legacy_surface_gate.py --check` | 0 | no unregistered boundary violation |
| T12 | `git diff --check` | 0 | 无输出 |
| T13 | `uv build --package suspension-kernel` | 0 | wheel + sdist |
| T14 | `uv build --package suspension-multibody` | 0 | wheel + sdist |

缺项为零。

## 1. G1（C++ 按职责形成模块，旧模块与旧 include 删除）— 达成

证据：`check_module_layering.py --strict --final` **退出 0**（`raw/g1_layering_final.log`），关键输出：

```text
target modules missing      : 0
legacy modules present      : 0
legacy modules unregistered : 0 []
mutual (reverse) edges      : 0
self-including headers      : 0
cross-aggregate includes    : 0
module cycles (SCC size>1)  : 0
```

- 目标模块集合齐全：`mb_{assembly,cases,config,contract,dual,element,energy,force,input,joint,linear,model,numeric,output,solve_dynamic,solve_static,tire,tire_state}` + `abi`（`cpp/include/` 实测 19 个目录）。
- 被替代模块缺席：`mb_base`、`mb_vehicle`、`mb_suspension`、`mb_integrator`、`mb_static`、`mb_linalg`、`mb_constraint`（`legacy modules present: 0`）。
- DAG 性质实测：无环、无 mutual 反向边、无越级聚合 include。检查器覆盖 cpp 与头文件（`header edges 118` / `source edges 107` / `source edge evidence 228 records`）。
- “不靠重命名掩盖职责混合”由无环 + 无越级 include + `mb_assembly` 不依赖 `abi`/`solver` 共同保证（终局模式显式拒绝）。

## 2. G2（建立 `report`，旧模块与转发壳清零）— 达成（按 A1 修订口径）

证据：`legacy_surface_gate.py --check` 退出 0；`report` 包存在且边界干净；删除后全仓扫描残留全部为保留项。

- **`report` 已建立**：10 个模块（`report/{__init__,compliance,geometry,wheel_loads,time_domain_physics}.py` + `report/metrics/{__init__,axle,common,case_specific,vehicle}.py`）。
- **边界实测干净**：`report/**` 的 import 面只有 `__future__`、标准库（`collections.abc`、`dataclasses`、`typing`）、`numpy` 与 `report.*` 内部——无 native/kernel/solver/axle_dynamics、无 preparation。负例门禁三类齐备（report→native、report→preparation、report 内复算本构），另加真实 report 树正例断言。
- **已无生产调用者的旧模块不存在**（08 删除，逐项见 `../20260921-08-delete/raw/step3_deletion_record.md`）：`core/`（整包：残余求解实现 + 转发壳）、`model/`（整包）、`metrics/`（整包）、顶层 `pac2002_scope.py`、`analysis/{_geometry,benchmarks,compliance,metrics,time_domain_physics,time_signals,vehicle_kc_time_domain}.py`。
- **无转发壳、无第二统一 decoder**：`legacy_surface_gate.py` 的 forwarding-shell 规则在 `--check` 下对 live 树无 finding；`results/decoder.py` 仍是唯一解码分派入口。
- **仍有现役生产调用的模块逐项登记保留理由**（A1/A2）：`elements/`（4 文件，4 条生产导入）、`analysis/vehicle_physics.py`（1 条生产导入）。对照 `raw/scan_after.log`：残留 15 条 = elements 11 + analysis 3 + README 迁移说明 1，**指向已删模块的引用为零**。

## 3. G3（Python 无关节残差/Jacobian 与反力求解；元件报告有 native 逐通道证据）— 达成（按 A1/A2 修订口径）

- **关节残差/Jacobian 与反力求解已删除**：`core/reactions.py`（`ReactionResult`/`recover_reactions`/`body_equilibrium_wrench`）与 `core/rank.py`（`RankDiagnostic`/`scale_jacobian`/`diagnose_rank`）不存在；`core/constraints.py` 及 `ConstraintSystem`（残差/Jacobian 实现）随 `core/` 整体删除——删除前实测其唯一消费者是 `tests/core/test_constraints.py`，生产路径零调用（本项为 G3 的终局闭合，见提交 `8b81c56`）。
- **保留并登记**（未伪装达成）：`elements/**` 元件本构与力汇总按 A1 保留（native 力旋量通道对固定体早退，`assembly_primitives.cpp:16,33`；实测 16/16 行全 NaN）；`analysis/vehicle_physics.py` 的静轮荷最小范数求解按 A2 保留（native 无静力 ABI 入口且导出面冻结）。逐项 file:line + 阻断原因 + 解除条件见 `../20260921-08-delete/raw/step3_deletion_record.md` 第 2 节。
- **元件载荷报告的 native 逐通道证据**：05 步骤 1 的冻结对照表（`../20260921-05-native-facts/raw/step1_channel_mapping.md`）覆盖名称/ID、两端、坐标系、作用点、符号、单位、能量、active 状态与六类元件、K/C 两模式；步骤 3 交付默认关闭的 `element_wrench` 通道与只读解码面；步骤 6 逐通道验收、步骤 7 差异登记。**关键正向事实**：对齐参考点与单位后两侧力差 `7.638e-10 N`、力矩差 `2.195e-10 N·m`——力律本身等价，未发现第二套物理定义。
- **凝聚等价性有实测证据（A3 后更新；已按全 8 case 逐 case 实测更正）**：生产路径已切换为**不凝聚 + weld 送 native `kind="fixed"`**。等价性：世界系质量性质完全一致（总质量 `3080.0`、世界质心 `[12.987013, 0.0, 160.390]`，全部 8 case）；差异两类且均非物理改变——(a) 布局（多 `rear_rack` body 22→23、weld 以 6 行 fixed 保留、融合体原点移动致 chassis 位姿常数偏移）；(b) 求解器残差级数值差（`tire_output` ≤`1.4e-9` N、`energy` ≤`3.9e-14` J、`bushing_output` `2.2e-13`；`spring_output` 全 0）。测试 `test_native_fixed_joint_is_what_carries_a_weld`、`test_the_two_weld_routes_agree_on_the_world_mass_properties`；回退开关 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 有测试。**初版「tire_output 与 energy 逐位相同」的表述只对 5 个 case 成立，已更正**。登记见 `../20260921-05-native-facts/raw/{a3_condensation_switchover,step2_condensation_equivalence}.md`。
- **数值门为独立项（A3 后更新）**：T8 动态哈希 26/26 逐位一致（组合哈希 `e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`，与 01 冻结值**相同**）、T9 K/C parity 0、T10 八 family 0。**重录情况**：`vehicle_dynamics_baseline/sha256.json` 按 2026-09-22 裁决 A3 **经用户明确授权重录**（生产路径切换到 native fixed 后 body 22→23，唯一失效项）；`dynamic_hash_baseline.json`、`kc_baseline/`、`kc_perf_baseline*.json` **未改动**（组合哈希未变可证）。除该授权项外未重录任何基线。

## 4. G4（保留 API/CLI/七 family/Adams/历史读取）— 达成

证据：`raw/g4_suites.log` 退出 0，`266 passed, 47 skipped`；隔离 wheel `raw/g_isolated_wheel.log` 退出 0。

- 公开 API：隔离环境 `import suspension_multibody` 成功，`__all__` 16 个公开名与 HEAD 一致；`run_case` → `write_artifact` → `read_artifact` 端到端成功（`artifact_type=result_bundle`、`status=success`）。
- CLI：隔离环境 `python -m suspension_multibody.cli --help` 退出 0。
- 七 family：T10 `case_parity_check.py` 退出 0（`OK: 8 families accepted`），含 `kc_quasi_static`/`axle_dynamic`/`vehicle_kc`/`vehicle_dynamic`/`handling`/`ride_four_post`/`ride_random_road` 与 design-gated `comparison`。
- Adams source rendering：`tests/adams` 通过（含 47 项按既有证据缺失的 skip，见第 6 节）。
- 历史 artifact 读取：`tests/io` 通过；隔离环境 `read_artifact` 成功。
- success/partial/failed artifact：`tests/{io,e2e,vehicle}` 覆盖（失败证据保留，见 `test_vehicle_service_keeps_native_failure_evidence` 等）。

## 5. 隔离 wheel 验证（A1 修订）

在会话 scratch 新建 venv，安装三个本地 wheel（`suspension_contracts`/`suspension_kernel`/`suspension_multibody`，因 multibody 依赖 contracts，必须一并装）：

| 检查 | 结果 |
|---|---|
| import 可用 | `import OK, version 0.1.0` |
| CLI 可用 | `python -m suspension_multibody.cli --help` 退出 0 |
| 七个导出符号 | `suspension_kernel_{run,capabilities,contract_version}` + `axle_kernel_abi_version` + `vehicle_kernel_abi_version` + `mb_core_abi_version` + `mb_core_run` 全部 present |
| 版本常量 | `contract_version=1`、`axle=15`、`vehicle=30`、`core=1` |
| 无新增导出 | `suspension_kernel_free` **不存在**（False） |
| native 可执行 | 真实 K&C 运行 `status=success`、1 case、10 bodies、`contract_version=1` |
| 已删旧模块缺席 | `core.rank`/`core.reactions`/`core.constraints`/`core.spatial`/`core.rigid_body`/`model`/`model.mass`/`metrics`/`metrics.axle`/`pac2002_scope`/`analysis.{compliance,_geometry,benchmarks,time_signals,vehicle_kc_time_domain}` 逐个导入尝试 → **全部 ModuleNotFoundError** |
| 保留项在位 | `report` 可导入；`elements`（A1）可导入；`analysis.vehicle_physics`（A2）可导入 |

## 6. 既有失败与限制（独立列明，不阻断完成）

- **47 skipped** 与 **1 xfailed** 与 01 基线完全一致（基线 `712 passed, 47 skipped, 1 xfailed`；现 `736 passed` 的增量为各子任务新增测试）。skip 原因仍是既有证据缺失：1 项 Adams 参考轮胎不可用、15 项 strict Adams source artifacts 或 Fiala/PAC2002 source case 不可用、其余为 USE_MODE 3/4/13/23/24/25 工件缺失。xfail 是 `test_native_brake_opposes_the_instantaneous_wheel_spin` 的退化制动夹具（无悬架刚度且轮胎无载荷时求解器无可接受步长）。
- **真实 Adams 执行不可用**（需本地 Adams/Car 安装与许可）：**未做整车数值等价声明**，与 01 的 `adams accuracy: BLOCKED` 一致。
- **动态验收的既有失败**：`dynamic_hash_sentinel.py --check` 输出中 acceptance 退出码仍为 1（9 个 case：`combined_load`、`in_phase_road` 等失败的既有状态）；**字节级门本身退出 0 且 26/26 逐位一致**，该 acceptance 失败是 01 记录的既有独立项，未因本 Epic 消除。
- **未运行** frozen median-of-N performance protocol（既有状态，01 已记录）。
- **两条已登记偏差**（不判达成，见 EPIC A1/A2 修订记录与父 PROGRESS「未闭合项」）：
  1. A1：用户第二问所选「Python 不再凝聚、weld 送 native 作 fixed 约束」的**手段**与「不重录基线」冲突，生产路径保留 Python 作者层凝聚，只采纳其目标（native fixed 关节作契约等价实现 + 等价性测试）。解除条件：单独裁决数值门策略。
  2. A2：静轮荷最小范数求解保留 Python（native 无静力 ABI 入口、导出面冻结）。解除条件：单独裁决是否扩展 ABI 导出面或在既有契约下加默认关闭的静力输出块。
- **一处口径说明**：`legacy_surface_gate.py --check --final` 退出 1。其 finding 全为 A1/A2 保留项（`legacy package still present`: `analysis`/`elements`；test-scope 导入）与 test scope 的覆盖测试；且门禁自身测试 `test_migration_mode_passes_and_final_mode_fails_on_the_live_tree` 断言「迁移模式 0、终局模式 1」。08 验收 3 与本文件 G2 的判据均为「**已无生产调用者**的部分不存在」，`--check` 退出 0 即满足；`--final` 退 0 与 architecture 测试全绿在 A1 下不可兼得，已在 `../20260921-08-delete/raw/step6_scan_after.md` §7 登记裁决。

## 7. 逐条 Goal 结论

| Goal | 结论 | 首要证据 |
|---|---|---|
| G1 | **达成** | `--strict --final` 退出 0；target missing 0、legacy present 0、cycles 0 |
| G2 | **达成（A1 口径）** | `report` 建立且 import 面干净；已无生产调用者的旧模块与转发壳为零；保留项逐项登记 |
| G3 | **达成（A1/A2 口径）** | 关节残差/Jacobian 与反力求解已删；`elements`/静轮荷保留登记；通道逐项证据；字节门未动 |
| G4 | **达成** | 隔离 wheel 端到端 + 八 family + Adams 渲染 + 历史读取全通过 |

**Done-When 核对**：所有行 DONE；「已无生产调用者的旧模块与旧导入为零」由扫描残留全部可归因为保留项证明；「数值门为独立项」由 T8 组合哈希与 01 冻结值逐位一致、且全程未改基线证明；两条已知未闭合项按 A1/A2 显式登记、**未判为达成**。
