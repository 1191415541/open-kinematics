# PROGRESS：p2-03 多体层力矩元接入

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-03`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p2-03-assembly-torque
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-03-assembly-torque/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 在 `modeling/primitives/` 声明力矩元素
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 依赖 p2-02（`SUBTASKS.csv` `p2-03` `depends_on`）：内核元素类型与 ABI 编组必须先落地，本行才有可编进内核输入的目标类型。
  - 前置 `S1`：阶段一 Epic 01–07 全部 `DONE`；**阶段一未完成之前本 Epic 任何一行不得置 `IN_PROGRESS`**（`EPIC.md` 行 67–75）。
  - **`subsystems/element_build.py` 三段串行**（`EPIC.md` 行 216）：本行占**构造分派段**，p4-04 占**轮胎段**，p3-02 只读。三段不得并行。
  - **判据 (b) 的核心约束**：施力体与反力体必须由 `Port` / `Connection` 配对决定，**不得硬编码 `upright`/`chassis` 字符串**（`EPIC.md` 行 233 与行 241 (b)；G2 判据行 81）。
  - **分层方向**：本行在 `modeling/primitives/` 新增声明，`modeling/` 不得反向导入作者层或 `subsystems/`（`EPIC.md` 行 226）。
- **Known issues**:
  - **编译层的具体落点 `EPIC.md` 未给锚点**：F1（行 111）只给消费点 `cases/vehicle_dynamic.py:579/582` 的契约表；本行需要的编译层入口路径属**开工时复核项**，不得凭印象补。
  - **新增测试的目录 `SUBTASKS.csv` 未列**：`notes` 只写写范围；候选是 `tests/modeling/` 与 `tests/subsystems/`（见本行 `validation_command`），开工时按项目现有布局确认。
  - 契约 schema 若需新增元素类型字段，必须与阶段一 03 的放置段改动**串行**（`EPIC.md` 行 218）。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py:60-71`（构造分派段，`EPIC.md` 行 241 (a) 给的锚点）与 `modeling/primitives/elements.py:567`/`:593` 两个既有元素类的形态，再确认 p2-02 的内核元素类型是否已落地，然后在分派段加分支并跑 `tests/modeling` + `tests/subsystems`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
