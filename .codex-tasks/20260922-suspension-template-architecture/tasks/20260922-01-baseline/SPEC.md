# 子任务 01：冻结现状基线与可执行验证命令

## 目标

在不动任何生产代码的前提下，**本轮实测**现有全部门禁的退出码与当前基线值，形成 02–11 每个子任务都可引用的冻结判据：

1. 冻结命令集（14 条）并逐条记录实测退出码：

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --all-packages ruff check .
uv run --all-packages ty check .
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

**不得依据旧任务（20260921 系列）的 DONE 结论**——旧任务的 `VALIDATION.md` 命令集与本 Epic 不同（本 Epic 增加了两包 `uv build` 与 `git diff --check`），必须本轮实测。

2. 现状事实清单（本 Epic 三个关键面的现状证据，每条要有 file:line）：

- `raw/joint_inventory.md`：8 种真实副（`spherical`/`revolute`/`fixed`/`prismatic`/`universal`/`cylindrical`/`inplane`/`constant_velocity`）在**装配层**、**schema**、**作者层文档编码**三处的落点，并点名截断点（已知 `cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 只映射 3 种）。
- `raw/tire_mass_inventory.md`：轮胎/轮端质量的现状路径（`WheelSpec.mass` → `preparation/assembly/vehicle.py:653,722` 合并进轮端刚体；内核 `Tire` 结构体无质量字段）。
- `raw/baseline_values.md`：各基线文件的当前值——`kc_baseline/`、`kc_perf_baseline.json`、`kc_perf_baseline_native.json`、`dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/`；C++ 侧 `packages/suspension_kernel/layering_baseline.json`。

3. `raw/baseline_commands.md`：命令原文 + 实测退出码 + 结果摘要 + 证据文件路径，成为 02–11 的唯一判据来源。

4. 记录既有失败与 skip/xfail 原因，作为后续「新增失败为零」的对照底线。

## 非目标

- 不做任何结构改动，不改生产代码、测试、schema、基线文件、`CMakeLists.txt`。
- **不重录任何基线**，只记录当前值。
- 不把未实测的值写入交付物；不为不可执行的命令编造参数或产物路径。
- 不用旧任务（`20260917`/`20260919`/`20260920`/`20260921`）的 DONE 记录代替本次实测。

## 约束

- 每条命令必须实际执行并记录：完整命令、实际退出码、结果摘要、证据文件路径。退出码非零也是实测结果，必须原样记录并说明是否属预期阻断（如 `--strict --final` 的当前缺口）。
- 无法执行的命令必须单列并标注它阻断哪些子任务；不得用 `echo SKIP`、占位参数或「预计通过」代替。
- 脚本参数只能通过 `--help` 与现有构建配置核实，不得虚构 flag。
- `raw/` 只存**已执行**的实测证据；大体积中间产物与临时脚本放会话 scratch，禁止预填未执行的证据。
- 本任务无前置子任务，但开工前不得引入任何依赖旧任务结论的假设。

## 范围与文件归属

- 可写：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-01-baseline/raw/**`（四份清单与关键命令日志）、本任务自身 `SPEC.md`/`TODO.csv`/`PROGRESS.md`。
- 只读：`packages/**`（含 `packages/suspension_kernel/layering_baseline.json`、`packages/suspension_multibody/tests/data/**`）、父 `EPIC.md`、`SUBTASKS.csv`。
- 不写：任何生产代码、测试、schema、基线文件；父级 `EPIC.md`/`SUBTASKS.csv`/`PROGRESS.md` 归主代理。

## 依赖

- 无前置，是串行主线的起点。
- 02–11 全部依赖本任务：未冻结判据前不得开始。02 的 `PROGRESS.md` 已明确引用 `raw/baseline_commands.md`（命令与退出码）与 `raw/joint_inventory.md`（8 种副的现状落点与截断点 file:line）；08 引用 `raw/tire_mass_inventory.md` 与 `raw/baseline_values.md`。

## 验收标准

1. `raw/baseline_commands.md` 中 14 条命令每条都有实测退出码（非空、非「预计」）与结果摘要；无法执行的命令单列并标注阻断范围。
2. `raw/joint_inventory.md` 覆盖 8 种副在三处的落点，每条有 file:line，并点名 `cases/kc_quasi_static/contract.py:44-48` 的截断（只映射 `BallJoint`/`RevoluteJoint`/`PrismaticJoint`）。
3. `raw/tire_mass_inventory.md` 给出 `WheelSpec.mass`（`schema/vehicle.py:60`，`axial_inertia` 在 `:61`）→ `preparation/assembly/vehicle.py:653,722` 的合并路径与总质量/质心聚合点（`preparation/assembly/vehicle.py:374-397`），并实测确认内核 `Tire`（`cpp/include/mb_model/types.hpp:101-146`）无质量字段。
4. `raw/baseline_values.md` 给出 6 个 multibody 基线 + 1 个 C++ `layering_baseline.json` 的当前值/哈希/规模。
5. 既有失败、skip、xfail 独立列明并各带原因，形成后续对照底线。
6. 未改动任何生产代码与基线文件：`git diff --check` 干净，且 `git status` 中不存在本任务对 `packages/**` 的改动（`.codex-tasks/*` 已被 `.gitignore` 忽略，交付物本身不出现在 diff 中）。

## 验证协议

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
uv run --all-packages ruff check .
uv run --all-packages ty check .
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv build --package suspension-kernel
uv build --package suspension-multibody
git diff --check
```

上述参数以本任务 `--help` 实测结果为准，实测后把命令原文与退出码写入 `raw/baseline_commands.md`；02–11 只引用该文件中的命令原文，不得自行改写参数或容差。
