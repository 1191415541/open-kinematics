# PROGRESS：03 通用装配引擎（条目清单驱动）

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `03`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: 20260929-03-universal-assembler
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-03-universal-assembler/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 契约 schema 放开 middle 放置与非四轮声明
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 本行**依赖 02 定下的配对契约**（跨子系统插接只经 `connections/matcher.py:102 match_requirements`，配对段字段由 02 写进 schema）；02 未落地前不得开工。
  - 与 02 **共享** `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`：02 改**配对段字段**、本行改**放置与轮数段**，**不得并行**（`EPIC.md` 行 135）。
  - 与 04 串行：两者都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，04 的 `depends_on=03`（`EPIC.md` 行 134）。
  - **严禁原地改 `VehicleModel` schema**（D1）：走"新增文档驱动装配器 + `VehicleModel` 适配器"，硬门是适配器路径产物与 01 快照逐项一致。
- **Known issues**:
  - 本行的"装配路径 `grep` 无命中"只能覆盖自有写范围：`preparation/` 属 04，`EPIC.md` F4 的 `preparation/vehicle_dynamic.py:373` 同样归 04 → G2 的终局 grep 判定在 04 之后复验。
  - 01 的 `snapshot.py --check` 尚未生成（01 的 `#1` 为 TODO）；本行落地时若脚本仍不存在，硬门项登记为 BLOCKED，不得记通过。
- **Next action**: 先读 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`（`:23` 的 `placement_role` 枚举、`:50` 的 `vehicle.wheels` 四个限制）与 `packages/suspension_multibody/src/suspension_multibody/connections/policy.py` 的 `full_vehicle` 规则（`:202-217`），确认 02 的配对段改动已落地且未与本行段落冲突，然后在**放置与轮数段**放开 `middle` 与非四轮声明，并跑 `packages/suspension_contracts/tests`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
