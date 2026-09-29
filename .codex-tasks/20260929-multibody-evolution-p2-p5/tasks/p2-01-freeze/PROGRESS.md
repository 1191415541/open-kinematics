# PROGRESS：p2-01 冻结现状事实与判据（阶段二）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-01`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p2-01-freeze
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-01-freeze/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 落盘制驱动力矩从工况文档到内核的完整路径快照
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 本行**不写生产代码**：写范围仅本目录 `raw/`、脚本与会话 scratch（`SUBTASKS.csv` `p2-01` `notes`）。
  - **本行是纯只读冻结行**：`depends_on` **为空**，本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，**可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。写范围只有本行 `raw/` 与会话 scratch，不写生产代码；冻结的是**阶段二的现状**（力矩与转向），不读前一阶段任何会被本 Epic 改写的产物。
  - **阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**（p2-02 ~ p2-06 与终局验收行），不影响本行。实测起点（2026-09-29，仅作背景）：阶段一 01/02 `DONE`、03 `IN_PROGRESS`、04–07 `TODO`（`EPIC.md` 行 79 与 F25 行 176）。
  - **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 237）——所有事实以本行实跑为准。
  - **基线不得重录**（D5，`EPIC.md` 行 64 与行 228）：本行只**读**基线并记起点值；`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门。
  - `kc_parity_check.py` 不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），**不构成证据**（`EPIC.md` 行 231）——若跑它必须注明是否带 `--actual-dir`。
- **Known issues**:
  - `SUBTASKS.csv` 的 `validation_command` 要求 `raw/torque_path_snapshot.json` 与 `raw/rear_steer_refusal.md` 非空；本行 `SPEC.md` 另加了 `raw/gates_baseline.md`、`raw/anchors_F1_F5.md`、`raw/baseline_notes.md` 三个落点，`validation_command` 未覆盖后三者——判据以 `SPEC.md` 的 Done-When 为准。
  - 路线图的两处已知过期说法必须复核并记「已过期」：`preparation/vehicle_dynamic.py:1506`（F1 行 111 明说已过期）与「轴 15；整车 30」的 ABI 说法（F3 行 116 明说已过期）。
  - 「13 个轴侧动态用例」的口径来自 `EPIC.md` 行 239 (e) 与行 311；本行只需记起点结论，不需解释口径来源。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py` 的 `:1496 _build_wheel_torque_signals` 定义与 `:249` 调用点、以及 `cases/vehicle_dynamic.py:579/582` 的 `wheel_torque`/`brake_torque` 契约表段，构造一次真实工况取路径快照，落到 `raw/torque_path_snapshot.json`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
