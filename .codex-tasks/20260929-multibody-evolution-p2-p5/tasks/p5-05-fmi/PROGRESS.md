# PROGRESS：p5-05 FMI 联合仿真导出

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-05`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p5-05-fmi
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-05-fmi/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（4 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: `p5-04`（`SUBTASKS.csv` 第 23 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）约束

## Context Recovery Block

- **Current milestone**: #1 — D4 范围与 D6 新依赖的裁决引用与落地口径
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 全仓**零** FMI/FMU/co-simulation 实现（`EPIC.md` F22，`grep -iE "fmi|fmu|cosim|co-simulation|co_simulation"` 只命中路线图文档）。
  - D4 建议（`EPIC.md` 行 63）：**FMU 2.0 Co-Simulation**，只导出「模型 + 输入/输出变量」，**不含 Python 侧求值**；不做实时/硬件在环承诺。
  - D6（`EPIC.md` 行 65）：FMI 库属**新依赖**，须**先提出再确认**；**除 p5-05 的 FMI 库外不新增依赖**。
  - 输入/输出变量的来源是 p5-03 的信号总线与 p5-02 的 `simulate` 入口。
- **Known issues**:
  - **D4 与 D6 未裁决前本行不得置 `IN_PROGRESS`**。
  - 若 D6 不允许引入 FMI 库：按 `EPIC.md` 行 319 降级为「导出为可联合仿真的接口契约 + 独立验证脚本」，并**登记为未闭合项**——不得把降级交付当作完整交付上报。
  - 独立验证脚本**不得**放进 `packages/**` 或 `tests/`（会被仓库测试收集，失去「仓库外独立」的含义）。
  - 「变量清单与输入/输出方向正确」不接受弱证据（仅「文件存在」）；须逐条列名称与 `causality` 并给理由。
- **Next action**: 先读 D4/D6 的裁决结论（`EPIC.md` 行 63/65）与 `SUBTASKS.csv` 第 23 行 `notes`，确认依赖口径；再按 `TODO.csv` 第 1 步起展开。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
