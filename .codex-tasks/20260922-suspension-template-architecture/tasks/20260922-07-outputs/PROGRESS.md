- 任务：输出声明与衍生输出（request 机制）
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 02（统一副底座）未完成；未满足依赖前不得开始。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-07-outputs/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码，`raw/` 为空（不预填未执行的证据）。**

开工前必须核验：

- 02 已完成（本任务不依赖副表本身，但父级依赖声明如此；02 会写 `tests/architecture` 之外的门禁与 `tests/joints`，与本任务写范围不相交）。
- 01 的 `raw/baseline_commands.md` 已冻结 `legacy_surface_gate.py --check` 与 `tests/metrics` 的当前退出码，作为本任务「未引入新增失败」的对照底线。
- 父 `EPIC.md` 的冻结约束有效：ABI 七符号与版本（15/30/1/1）不变；本任务不重录任何基线。

## 制定计划时确认的现状事实（实施时须复核）

- 迁移对象共 27 个函数：`report/metrics/axle.py` 3、`case_specific.py` 12、`common.py` 8、`vehicle.py` 4（`grep -n "^def " report/metrics/*.py` 实测 27）。
- 现有 `report/` 同级模块：`report/__init__.py`、`compliance.py`、`geometry.py`、`time_domain_physics.py`、`wheel_loads.py`、`metrics/`；`report/` 的边界由 `tests/architecture/legacy_surface_gate.py` 与 `legacy_surface_registry.json` 登记并拦截。
- 现有 `tests/metrics/test_metrics.py` 是旧实现的回归，重述后必须保持全绿且新增逐值一致对照。
- `packages/suspension_multibody/src/suspension_multibody/outputs/` 目前不存在（`templates/`、`subsystems/` 也不存在，属 03/04 写范围）；本任务新建 `outputs/` 不会与它们冲突。

## 写范围与并行约束（硬条件）

- 本任务只写 `outputs/**`、`report/metrics/**`、`tests/outputs/**`、`tests/metrics/**`。
- **不得**写 `preparation/`、`subsystems/`、`templates/`（04–06 写范围）、`packages/suspension_kernel/**`（08 写范围）。
- 07 的 `outputs/` 是 08 的显式禁写目录，两者写范围不相交，可并行。

## 下一步

等 02 完成后，从 `TODO.csv` 第 1 行开始：先清点 27 个函数并固化对照输入集，再建声明模型、合并与冲突检测、衍生输出求值器（含旁路负例），最后分两批重述 27 个指标并逐值对照。父 `SUBTASKS.csv` 第 07 行的状态由主代理回填。
