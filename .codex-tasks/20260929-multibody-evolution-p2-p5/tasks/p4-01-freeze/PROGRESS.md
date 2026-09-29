# PROGRESS：p4-01 冻结现状事实与判据（阶段四）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-01`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p4-01-freeze
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-01-freeze/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — ARB 现状锚点复核与两套 ARB 物理对照
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on` **为空**：本行是**纯只读冻结行**（写范围只有本行 `raw/` 与会话 scratch，冻结的是**阶段四的现状**——ARB/wheel/角色表/端口），故**本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。`EPIC.md` 行 219 的阶段四顺序（阶段四在阶段三完成后开工）**只对实施行成立**。
  - **阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**（p4-02 ~ p4-05 与终局验收行），不影响本行长；本行**不写生产代码**（父行 `notes`：写范围仅本行 `raw/` 与脚本与会话 scratch）。
  - 本行是**只读冻结**行，写范围仅本行 `raw/` 与脚本与会话 scratch（父行 `notes`）。
  - 结论是 p4-02 / p4-03 / p4-04 的唯一事实依据：F11（ARB 硬编码跨接）、F12（角色表 10+ 同步点）、F13（端口不存在）、F14（wheel 文件链）、F15（`VerticalTireElement` 引用点）、F17（ARB 冻结产物）。
- **Known issues**:
  - 尚未开工，故所有锚点均为**待复核**：父 Epic 的 F 编号锚点若在本行开工时已过期，只在本行 `raw/` 与本节记录「锚点过期 + 实测新锚点 + 受影响的行」，**不改父文件**。
  - F12 的同步点数量以本行实测为准（`EPIC.md:139` 称 10+ 处）；清单缺项即本行未完成。
- **Next action**: 先按 `EPIC.md` F11（`EPIC.md:136`）逐条复核 `subsystems/suspension.py:682-695`、`modeling/primitives/elements.py:593`、`schema/elements.py:224`、`element_build.py:211/66-67` 的原文，并读对照用例 `tests/axle_dynamics/test_api.py:315`，把两套 ARB 物理的四列对照写进 `raw/arb_two_physics.md`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
