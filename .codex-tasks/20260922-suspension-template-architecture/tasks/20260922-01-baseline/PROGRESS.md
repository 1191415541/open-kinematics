- 任务：冻结现状基线与可执行验证命令
- 形态：single-full（Epic 子任务）
- 进度：9/9 步骤 DONE（2026-09-23 复核）
- 当前：已完成并提交入父级 `SUBTASKS.csv` 第 01 行 DONE。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-01-baseline/`
- 验证：14 条命令逐条实跑（见 `raw/baseline_commands.md`）；本机实测 `737 passed／47 skipped／1 xfailed`（与计划记录的 `783/1/1` 差异已定性为环境差异，同批 784 用例）、`--strict --final` 0、动态哈希 26/26 逐位一致（`e7407656...`）、K/C parity 0、8 family accepted、两包 build 0、ruff/ty 0、`git diff --check` 0。

## 恢复信息

**本任务已实施完成，交付物为 `raw/` 下四份实测记录（`baseline_commands.md`／`joint_inventory.md`／`tire_mass_inventory.md`／`baseline_values.md`）。** 以下为开工前须核验的约束，已全部满足：

开工前必须核验：

- 父 `EPIC.md` 的冻结约束有效：ABI 七符号与版本（15/30/1/1）不变；`check_module_layering.py --strict --final` 保持绿是 11 的收尾判据之一；D4 允许**受控**重录基线。
- 本任务**不重录任何基线**，只记录当前值。
- **20260921 系列的 DONE 结论不得引用**：其 `VALIDATION.md` 的命令集与本 Epic 不同（本 Epic 增加两包 `uv build` 与 `git diff --check`），且该系列已按 A1/A2/A3 修订过口径。

## 交付物

- `raw/baseline_commands.md`：14 条命令原文 + 实测退出码 + 结果摘要 + 证据路径。
- `raw/joint_inventory.md`：8 种副在装配层 / schema / 作者层文档编码三处的落点与截断点。
- `raw/tire_mass_inventory.md`：轮胎/轮端质量现状路径。
- `raw/baseline_values.md`：各基线文件当前值 / 哈希 / 规模。
- `raw/` 内关键命令日志（大体积中间产物放会话 scratch）。

## 制定计划时确认的现状事实（实施时须实测复核并落 file:line）

- 截断点：`packages/suspension_multibody/src/suspension_multibody/cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 只映射 `BallJoint`→`spherical`、`RevoluteJoint`→`revolute`、`PrismaticJoint`→`prismatic`。
- schema 已允许 8 种副：`packages/suspension_multibody/src/suspension_multibody/schema/model.py:100-109` 的 `IdealJointSpec.kind`。
- 内核 `Tire` 无质量字段：`packages/suspension_kernel/cpp/include/mb_model/types.hpp:101-146`。
- 质量现挂轮端 body：`packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py:60` 的 `WheelSpec.mass`（`axial_inertia` 在 `:61`）→ `preparation/assembly/vehicle.py:653,722` 合并进轮端刚体；总质量/质心聚合在 `preparation/assembly/vehicle.py:374-397`。
- 基线文件位置：`packages/suspension_multibody/tests/data/` 下的 `kc_baseline/`、`kc_perf_baseline.json`、`kc_perf_baseline_native.json`、`dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/`；C++ 侧 `packages/suspension_kernel/layering_baseline.json`。
- 生效的 native 边界是文档契约而非 struct ABI：`kernel/__init__.py:186-205` 的 `run_contract` 传两份 JSON payload。

## 下一步

从 `TODO.csv` 第 1 行开始逐条实测并写 `raw/`。本任务完成后由主代理把父 `SUBTASKS.csv` 第 01 行置为 DONE；02、08 依赖本任务，07 经 02 间接依赖。父级 `SUBTASKS.csv` 状态由主代理回填，本任务不改。
