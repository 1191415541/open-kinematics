# 07 第 7 项：回归与验收证据

全部命令在仓库根 `C:\杂件\open-kinematics` 执行，退出码为实测值。

## 1. 组合门禁（SPEC 验证协议）

| 命令 | 退出码 | 摘要 |
|---|---|---|
| `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/analysis packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cli -q` | 0 | `42 passed` |
| `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q` | 0 | `91 passed` |
| `uv run python packages/suspension_multibody/scripts/case_parity_check.py` | 0 | `OK: 8 families accepted`（kc_quasi_static worst error/tolerance `0.000186038`，其余 family 逐位一致/独立展开一致） |
| `uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` | 0 | `736 passed, 47 skipped, 1 xfailed` |
| `uv run --all-packages ruff check .` | 0 | `All checks passed!` |
| `uv run --all-packages ty check .` | 0 | `All checks passed!` |
| `git diff --check` | 0 | 无空白错误 |

## 2. 数值门（未改任何基线）

| 命令 | 退出码 | 摘要 |
|---|---|---|
| `uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | 0 | `artifacts hashed: 26`；`combined sha256: e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`（与基线逐位一致）；`OK: dynamic output matches the frozen baseline byte-for-byte` |
| `uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict` | 0 | `header edges 118 / source edges 107`；`legacy modules present 0`；`module cycles 0`；`OK: layering matches the recorded baseline` |

`dynamic_hash_baseline.json`、`kc_*_baseline*`、`vehicle_dynamics_baseline/` 均未修改。

## 3. 测试计数与基线口径

- 冻结前实测（HEAD，用 `git archive HEAD` 复制 tests 树到 `tmp_baseline_tests` 后 collect-only）：**772 collected**；实跑基线 **724 passed / 47 skipped / 1 xfailed**。
- 本任务后：**784 collected**；实跑 **736 passed / 47 skipped / 1 xfailed**，退出 0。
- 差值 **+12 = 新增测试数**（`tests/simulation/test_replay.py` 9 条 + `tests/architecture/test_legacy_surface_gate.py` 3 条），**新增失败 0**。
- 注意：任务书给出的基线「722 passed」是 `a734038`（06 主提交）时的数字；`393ee46`（06 收口）又加了 2 条 architecture 断言（`test_unified_simulation_boundaries.py`），HEAD 的真实基线是 724。已按 724 对账，未发现任何回退。

## 4. legacy_surface 门禁与 registry

| 命令 | 退出码 | 摘要 |
|---|---|---|
| `uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | 0 | findings 38（迁移前 55）；`OK: no unregistered Python boundary violation` |
| 同上 `--final` | 1 | **预期**（终局判据在 08：legacy 包仍在） |

registry 条目：**14 → 5**（只能收缩，无豁免、无放宽扫描范围）。删除的 9 条全部是 07 切走的调用方：

- `scripts/{case_parity_check,kc_native_probe,kc_perf_gate}.py` × `analysis`（改读 `tests/data/benchmark_axle.json`）
- `src/.../api.py` × `analysis`(×2)、`metrics`
- `src/.../axle_dynamics/contract_run.py` × `metrics`
- `src/.../cases/kc_quasi_static/workflow.py` × `analysis`
- `src/.../vehicle/service.py` × `metrics`
- `src/.../adams/reference.py` × `analysis`

保留的 5 条：`api.py`/`preparation/assembly/{front_axle,vehicle}.py`/`preparation/vehicle_dynamic.py` 的 `elements`（A1 作者层元件构造，05/06 登记），以及 `vehicle/service.py` 的 `analysis`（A2 保留的 `compute_static_wheel_loads`）。

## 5. report 边界负例门禁（SPEC 验收 1）

`tests/architecture/legacy_surface_gate.py` 新增两类规则（原 02 扫描器扩展，未放宽任何既有规则）：

- `report_preparation_import` / `report_preparation_call`：`report` 内 import 或调用 preparation/authoring（`PREPARATION_TOKENS`/`PREPARE_NAMES`）。
- `report_constitutive_call`：`report` 内求值元件力律（`CONSTITUTIVE_NAMES`）。

三类负例与正例（`tests/architecture/test_legacy_surface_gate.py`）：

| 类别 | 测试 | 覆盖 |
|---|---|---|
| report→native | `test_report_calling_native_or_solving_is_detected`（02 已有，复用） | import + solve 调用 |
| report→preparation | `test_report_importing_or_running_preparation_is_detected`（新增） | import + `build_front_axle`/`time_grid` 调用，迁移与 final 模式均失败 |
| report 内复算本构 | `test_report_recomputing_a_constitutive_law_is_detected`（新增） | `spring_force(...)` 调用，规则唯一命中 |
| 正例（真实 report 树） | `test_the_live_report_tree_stays_inside_its_boundary`（新增） | 真实 `report/` 10 个模块、零 boundary finding；并独立于扫描器规则解析全部 import，native/kernel/solver/axle_dynamics 与 preparation 命中数均为 `[]` |

`report` 的 import 面实测：`__future__`、`collections.abc`、`dataclasses`、`typing`、`numpy`、`suspension_multibody.report.*`。无 kernel/native/solver/axle_dynamics，无 preparation，无 results，无本构调用。

## 6. 实现中发现并处理的问题

### 6.1 `api.py` 的 import 顺序（必须处理，已处理）

`api.py` 原先第一个包级 import 是 `from .analysis.compliance import ...`，它顺带把作者层元件链按安全顺序装载。07 把 `analysis` 从 `api.py` 移除后，字母序上的第一个 import 变成 `from .axle_dynamics.schema import ...`，触发一条**既有但一直被遮蔽**的循环：

```
axle_dynamics → results.decoder → results.vehicle → preparation.vehicle_dynamic
  → elements → core → preparation.assembly.types → preparation.assembly.__init__
  → front_axle → (elements 半初始化) ImportError: AntiRollBarElement
```

实测证据：`report` 侧的 `from .preparation.assembly import ...` 作为入口可安全装载（`python -c "import suspension_multibody.preparation.assembly"` 成功），而 `axle_dynamics`/`elements`/`results` 作为入口会失败。

处理：`api.py` 把 `from .preparation.assembly import FrontAxleAssembly, build_front_axle` 放在包级 import 块之前（带 8 行说明 + `# isort: skip`，ruff 0）。该约束在 08 删除 `elements` 后自然消失，注释里写明了这一点。**未修改任何 06 的只读文件**（`preparation/**`、`results/**` 除 SPEC 授权的 `results/timeseries.py` 外未动）。

### 6.2 与 SPEC「不写 api.py」的冲突（已按任务书执行）

07 SPEC 的「不写」清单含 `api.py`，但 SPEC 目标 4 与验收要求「生产调用方切到新入口」，且任务书明确要求 `api.py:150` 的调用点改用新入口、registry 只能收缩。两者不可同时满足，按**任务书**执行：`api.py` 仅改 3 处 import 来源（`report.compliance`、`report.metrics`、`simulation.replay`）与 6.1 的装载顺序，未改任何调用逻辑、返回值或公开签名。

## 7. 未完成 / 遗留（交 08）

1. `analysis/vehicle_physics.py` 的 `compute_vehicle_roll_centers`（`:145`）：无生产调用者，但调用 `build_vehicle(...)`（`:151`）即执行 preparation，**不能**迁 `report`；SPEC 又不允许以「无内部调用」删除。本任务保留原状，**未做 A2 式登记**——08 删除 `analysis/` 前需要显式裁决（删除 / 保留到某新归属 / 补登记）。
2. `analysis/time_signals.py` 与 `preparation/signals.py` 目前是两份同源实现（06 的搬迁方式），`analysis/time_domain_physics.py` 仍用 `analysis/vehicle_physics.py` 的 `summarize_wheel_loads` 副本；两者都随 08 删除 `analysis/` 收敛。
3. `report` 尚未被任何 CLI/公开 API 直接暴露（`report/__init__` 只服务包内与调用方 import）；SPEC 非目标要求不改公开 API/CLI，故未新增导出。
