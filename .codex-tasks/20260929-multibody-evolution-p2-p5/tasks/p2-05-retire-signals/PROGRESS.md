# PROGRESS：p2-05 废除离线预采样力矩与 front_brake_bias

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-05`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p2-05-retire-signals
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-05-retire-signals/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 删除 `_build_wheel_torque_signals` 并复验 grep 零命中
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 依赖 p2-04（`SUBTASKS.csv` `p2-05` `depends_on`）：`front_brake_bias` 改为「模板属性槽驱动」需要 p2-04 的槽先存在。
  - **`preparation/vehicle_dynamic.py` 三段串行**（`EPIC.md` 行 217）：本行 = 力矩段；**p2-06 在本行之后**（转向段）；**p4-04 在 p2-06 之后**（轮胎拒绝段）。误改转向段即与 p2-06 冲突。
  - **本行改的是在用的动态路径**（`EPIC.md` 行 312）：`_build_wheel_torque_signals` 的产物今天进 `dynamic_hash_baseline`。对照基准是 p2-01 的 `tasks/p2-01-freeze/raw/torque_path_snapshot.json`——**这是本行的唯一基准**。
  - **D5 禁止重录**（`EPIC.md` 行 64 与行 228）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；若确需变化，逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」，**质量与质心相同不足以证明等价**。
  - 前置 `S1`：阶段一 Epic 01–07 全部 `DONE`；**阶段一未完成之前本 Epic 任何一行不得置 `IN_PROGRESS`**（`EPIC.md` 行 67–75）。
- **Known issues**:
  - `_WHEEL_NAMES`（`preparation/vehicle_dynamic.py:71`，`EPIC.md` F1 行 111）在力矩路径删除后是否失去用途，属本行开工时的判断项——`SPEC.md` 已把它列进力矩段锚点，但**是否删除须实测其残余消费点后再定**。
  - `preparation/vehicle_dynamic.py` 的同一文件内还有 p2-06 与 p4-04 的段落；本行须用 `git diff` 分段自证只出力矩段的 hunk（记入 `raw/retirement_grep.md`）。
  - `cases/vehicle_kc.py:128-136` 的准备期删除段归 **p2-06**（`EPIC.md` F5 行 120），本行**不得触碰**。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py` 的 `:1496`（函数定义）、`:249`（调用点）、`:291-302`（结果写入）、`:1531/1533`（`front_brake_bias`）与 `cases/vehicle_dynamic.py:579/582`，并对照 p2-01 的 `raw/torque_path_snapshot.json`，再按 TODO 第 1 步删除函数并复验 grep。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
