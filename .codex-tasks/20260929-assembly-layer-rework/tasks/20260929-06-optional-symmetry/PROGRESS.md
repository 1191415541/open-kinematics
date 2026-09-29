# PROGRESS：06 可选对称（镜像成为声明）

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `06`

## Session Start

- **Date**: （未开工）
- **Task name**: 20260929-06-optional-symmetry
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-06-optional-symmetry/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 单侧文件的坐标系/镜像方向/命名契约落文档并有测试
- **Current status**: NOT_STARTED
- **Last completed**: 无
- **Current artifact**: `SPEC.md`、`TODO.csv`
- **Key context**: 本子任务尚未开工，`SPEC.md` / `TODO.csv` 为规划产物，**未执行任何步骤、未产生任何证据**。依赖 `05`（与 `04` 写范围相交，不得并行，D5）。落地前后的「产物是否变化」判据是 `tasks/20260929-01-freeze/snapshot.py --check`（待 01 的 `#1` 完成后跑）。
- **Known issues**: 01 的 7 组合快照尚未生成（`tasks/20260929-01-freeze/TODO.csv` 的 `#1` 仍为 `TODO`），故 #6 的比对在快照落盘前不可执行。
- **Next action**: 读 `EPIC.md` 的 F6/G5/D5 与 01 的 `raw/painpoint_anchors.md`，确认 `_SIDES` 与镜像相关锚点（`subsystems/suspension.py`、`subsystems/si_assembly.py`、`subsystems/wheel.py:114-116`、`subsystems/element_build.py:205-210`），再按 `TODO.csv` #1 落文档契约。

---

## Final Summary（未开工）

本子任务尚未开工，`SPEC.md` 与 `TODO.csv` 为规划产物；未执行任何步骤、未产生任何证据（`raw/` 为空）。`TODO.csv` 6 行状态全为 `TODO`。
