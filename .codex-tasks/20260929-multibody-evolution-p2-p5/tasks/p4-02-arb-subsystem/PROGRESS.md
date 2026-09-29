# PROGRESS：p4-02 anti_roll_bar 独立子系统

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-02`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p4-02-arb-subsystem
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-02-arb-subsystem/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 同步 `templates/roles.py` 与三份契约 schema 的 `functional_role` enum
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - `depends_on = p4-01`；p4-01 交付的 F11/F12/F13 实测清单是本行**唯一**改动依据。
  - **`templates/roles.py` 与 `templates/builtin.py` 是共享注册文件，不得与 p2-04 并行（p2-04 在前）**（`EPIC.md:215`）；三份契约 schema 的 `functional_role` enum 与角色集合及硬断言同属本行，p2 的任何行不得增删角色名（`EPIC.md:227`）。
  - **F12 全清单漏一处即失败**（`EPIC.md:139`、`EPIC.md:316`）：`roles.py:158-162` 的集合硬断言与三份 schema enum 是前置墙，漏改即在 import 期抛 `RoleSpecError` 或契约 schema 拒绝。
  - **两套 ARB 物理不得混淆**（`EPIC.md:137`、`EPIC.md:317`）：Python `AntiRollBarElement` 是 z 位移差力偶力元，native 扭杆 ABI 是另一套物理；选型必须给理由。
  - 全局前置 `S1`：阶段一 01–07 全部 `DONE`；**未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:75`）。
- **Known issues**:
  - 尚未开工，故 F12 清单各项锚点为**待复核**；任何锚点过期只在本行 `raw/` 与本节记录，不改父文件。
  - 阶段二 p2-04 若尚未 `DONE`，本行不得开工（共享注册文件冲突）。
  - `suspension.py:682-695` 的硬编码跨接是今天防倾杆唯一入口；删除前必须先有端口通道（依赖 p4-03 的语义化端口）。
- **Next action**: 先读 `templates/roles.py:70` 与 `:158-162`、三份契约 schema 的 `functional_role` enum（`template.schema.json:11`、`subsystem.schema.json:8`、`assembly.schema.json:22`），确认 p2-04 已落地、共享文件无并发写入，然后按 `raw/role_table_sync.md`（p4-01 交付）逐项同步角色表并跑 `tests/templates` 与 `packages/suspension_contracts/tests`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
