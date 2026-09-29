# PROGRESS：07 终局独立验收

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `07`

## Session Start

- **Date**: （未开工）
- **Task name**: 20260929-07-acceptance
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-07-acceptance/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（8 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 端到端 (a) 3 轴整车总成装配并跑通
- **Current status**: NOT_STARTED
- **Last completed**: 无
- **Current artifact**: `SPEC.md`、`TODO.csv`
- **Key context**: 本子任务尚未开工，`SPEC.md` / `TODO.csv` 为规划产物，**未执行任何步骤、未产生任何证据**。本行是**独立终局验收**，不得采信 02–06 的自报结论；前置是 02–06 全部 `DONE` 且 01 的 `#1` 快照落盘。产物是否变化的判据是 `tasks/20260929-01-freeze/snapshot.py --check`。
- **Known issues**: 01 的 7 组合快照尚未生成（`tasks/20260929-01-freeze/TODO.csv` 的 `#1` 仍为 `TODO`）；K/C 对标不得用不带 `--actual-dir` 的 `kc_parity_check`（自比较恒过，不构成证据）。
- **Next action**: 确认 02–06 的 `SUBTASKS.csv` 状态与 01 快照落盘情况；按 `TODO.csv` #1 实跑 Done-When (a)，逐条记录退出码与产物差异。

---

## Final Summary（未开工）

本子任务尚未开工，`SPEC.md` 与 `TODO.csv` 为规划产物；未执行任何步骤、未产生任何证据（`raw/` 为空）。`TODO.csv` 8 行状态全为 `TODO`。
