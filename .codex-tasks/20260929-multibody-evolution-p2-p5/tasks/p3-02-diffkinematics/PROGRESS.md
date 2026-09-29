# PROGRESS：p3-02 微分运动学引擎（速度旋量与空间瞬轴）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-02`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p3-02-diffkinematics
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-02-diffkinematics/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `TODO`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #1 — 引擎输入输出契约冻结（约束集合与点表 → 速度旋量与空间瞬轴）
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on = p3-01`（`SUBTASKS.csv` 的 `p3-02`）；跨阶段顺序为「阶段三在第一阶段完成后开工」（`EPIC.md` 行 211），且**前置 S1**——阶段一 Epic 01–07 全部 `DONE` 之前本行不得置 `IN_PROGRESS`（`EPIC.md` 前置一节：19 行实施行受此约束，四个只读冻结行不受限）
  - **D3（`EPIC.md` 行 62）**：先走 **Python 侧数值微分，不改 ABI**。内核 `mb_joint/functions.hpp:49-66` 的 `constraint_jacobian` **未过 ABI**；`suspension_kinematics/jacobians.py` 是另一包（SymPy 生成）且未被 multibody 引用，两解算器产品不得互相导入（F9，`EPIC.md` 行 130）。
  - **本行是 p3-03 与 p3-04 的共同前置，且二者要并行**（`EPIC.md` 行 220）——本行**必须**把引擎公开接口（输入/输出形状、瞬轴表示、容差）冻结在 `raw/engine_contract.md`，否则两个并行行无法对接。
  - **本行的写范围含 `vehicle/roll_centers.py` 的调用段**，与 p3-03（同文件主体重构）**串行**（`SUBTASKS.csv` 的 `p3-02` `notes`）；本行不得在 p3-03 之前删硬点别名表。
  - **F7（`EPIC.md` 行 126）**：`compute_vehicle_roll_centers` 在生产代码里没有任何调用者——本行不能靠「未被调用」推断零影响，必须主动证明不改变 K/C 读数。
- **Known issues**:
  - **四种构型今天能否构造尚未实测**（`EPIC.md` 行 314 与行 34）：本行的解析解验证需要 p3-01 的 `raw/config_refusals.md` 结论；该文件尚不存在（p3-01 未开工）。开工前必须先读它。
  - **`vehicle → subsystems` 依赖加重**（`EPIC.md` 行 315）：新引擎读装配运行时的依赖方向未经 `tests/architecture/test_import_boundaries.py` 验证；不通过则改为调用方注入约束集合与点表。
  - 数值微分的步长与截断误差**未实测**（`EPIC.md` 行 251(c) 要求与今日 `_instant_center` 对照且差异有已解释来源）。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py`（尤其 `:59-67` 硬点别名表、`:81-99 _instant_center`）与 `packages/suspension_multibody/tests/physics/test_vehicle_physics.py`，再读 p3-01 已落盘的 `raw/config_refusals.md` 与 `raw/anchors_roll_centers_static_loads.md`，确定本行引擎可用的构型集合；随后冻结引擎签名与返回值形状到 `raw/engine_contract.md`，并先跑 `tests/architecture/test_import_boundaries.py` 验证依赖方向可落地。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记**已执行**的结果，不存虚构结果；口径见 `EPIC.md` 行 355）。
