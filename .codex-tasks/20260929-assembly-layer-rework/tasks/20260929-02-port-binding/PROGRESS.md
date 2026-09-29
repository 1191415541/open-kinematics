# PROGRESS：02 显式接口配对落地

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `02`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: 20260929-02-port-binding
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-02-port-binding/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 配对段可选字段写进契约 schema 与文档读取
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**: 本行依赖 01 的 `snapshot.py --check` 作"产物是否变化"判据（01 的 `#1` 未完成，脚本尚未生成）；与 03 共享 `assembly.schema.json`——本行只改配对段字段，03 再改放置与轮数段，**不得并行**。
- **Known issues**:
  - 本行验收中"`chassis`/`ground` 字符串规则在装配路径全部消除"有三处落在 03/04 的写范围内（`subsystems/vehicle_assembly.py:212-228`、`preparation/vehicle_dynamic.py:373`、`subsystems/vehicle_parts.py:172-173` 与 `:414`），本行只能收口自有路径；归属见 `SPEC.md` 的 Constraints。
  - `EPIC.md` F4 的待决项（`modeling/ports.py` / `templates/ports.py` 无外部消费者）需在本行给出启用或删除的结论。
- **Next action**: 读 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json` 与 `packages/suspension_multibody/src/suspension_multibody/authoring/documents.py` 的文档读取路径，确定配对段字段名与层级（条目层 `additionalProperties: false` 需同步放开），先在契约包里放开字段并跑 `packages/suspension_contracts/tests`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
