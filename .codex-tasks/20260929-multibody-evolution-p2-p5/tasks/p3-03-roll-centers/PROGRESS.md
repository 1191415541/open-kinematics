# PROGRESS：p3-03 通用滚转中心（摆脱硬点名称嗅探）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-03`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p3-03-roll-centers
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-03-roll-centers/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `TODO`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #1 — `compute_vehicle_roll_centers` 改走新引擎并与改造前双叉臂对照
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on = p3-02`（`SUBTASKS.csv` 的 `p3-03`）；**引擎契约以 `tasks/p3-02-diffkinematics/raw/engine_contract.md` 为准**，本行不重定义它。跨阶段顺序为「阶段三在第一阶段完成后开工」（`EPIC.md` 行 211），且**前置 S1**——阶段一 Epic 01–07 全部 `DONE` 之前本行不得置 `IN_PROGRESS`（`EPIC.md` 前置一节：19 行实施行受此约束，四个只读冻结行不受限）
  - **与 p3-04 硬串行**（`EPIC.md` 行 228）：生产写范围不相交（本行写 `vehicle/roll_centers.py`，p3-04 写 `vehicle/static_loads.py`），但**测试写范围相交**——`tests/physics/test_vehicle_physics.py` **归本行**（它同时覆盖滚转中心与静平衡），p3-04 只写它**新建**的 `tests/physics/test_static_loads.py`。`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`，**不得并行**。两者都依赖 p3-02。
  - **零回归口径（`SUBTASKS.csv` 的 `p3-03` `notes`）**：本行**不改变任何 K/C 读数**（`kc_baseline` 逐位不变）。
  - **G3 判据（`EPIC.md` 行 83 与行 87）**：四种构型（双叉臂、5 连杆、麦弗逊、扭梁）**各有断言**；`grep -n "UPPER_INBOARD|LOWER_INBOARD|UCA_|_BODY_ALIASES"` 在新引擎路径无命中；**滚转中心高必须由侧倾反力虚功导数矩阵解算**（路线图 `docs/multibody_architecture_evolution.md:136-139` 的指定口径），**不得只求几何连线交点**，且有一条独立数值判据（`EPIC.md` 行 257(e)）。
  - **F7（`EPIC.md` 行 126）**：`compute_vehicle_roll_centers` 在生产代码里没有任何调用者——本行不能靠「未被调用」推断零影响，必须主动实跑证明 K/C 读数逐位未变。
- **Known issues**:
  - **改造前基准的可用性**：p3-02 已改 `roll_centers.py` 的调用段；若 p3-02 提前删了硬点别名表，本行无法复现改造前结果 → 按 `SPEC.md` 风险节登记为阻断项并回退给主代理。
  - **四种构型今天能否构造需实测**（`EPIC.md` 行 314 与行 34）：结论在 p3-01 的 `raw/config_refusals.md`，该文件尚不存在（p3-01 未开工）。本行三种构型的断言依赖它；开工前必须先读。
  - **`vehicle → subsystems` 依赖方向的连锁**（`EPIC.md` 行 315）：删除 `subsystems.geometry`（`side_hardpoints`）使用点会减轻依赖，但新引擎输入面可能重新引入；须跑 `tests/architecture/test_import_boundaries.py` 与 `check_module_layering.py --strict --final`。
  - **虚功导数口径与瞬心法的对照尚未做**（`EPIC.md` 行 257(e)）：滚转中心高今天实质是瞬心连线求交的派生量，本行必须改为由侧倾反力虚功导数矩阵解算，并给出与瞬心法的对照（一致或在已解释容差内一致）与一条独立数值判据；三项缺一即未达成，结论落 `raw/roll_center_virtual_work.md`。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py` 的现状（`:59-67` 硬点别名表、`:81-99 _instant_center`、`:96 _line_intersection`、`:42-47` 与 `:117-129` 整车层二维交点）、`tests/physics/test_vehicle_physics.py`（本行独占）与路线图 `docs/multibody_architecture_evolution.md:136-139`（虚功导数口径）；再读 `tasks/p3-02-diffkinematics/raw/engine_contract.md`（引擎接口）与 `tasks/p3-01-freeze/raw/config_refusals.md`（四构型构造能力）；先做改造前的双叉臂取值并留证，再改 `compute_vehicle_roll_centers` 走新引擎，最后补侧倾反力虚功导数矩阵与独立数值判据（`raw/roll_center_virtual_work.md`）。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记**已执行**的结果，不存虚构结果；口径见 `EPIC.md` 行 355）。
