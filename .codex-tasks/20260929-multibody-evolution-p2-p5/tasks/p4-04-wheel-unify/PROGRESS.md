# PROGRESS：p4-04 单轴与整车 wheel 文件统一并消除轮胎补丁

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-04`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p4-04-wheel-unify
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-04-wheel-unify/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest
- **Status**: **DONE**

## Context Recovery Block

- **Current milestone**: #6 — 终局与验收（全 6 步完成）
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - **边界（2026-09-29 审核修订）**：本行**以阶段一 04 的实际交付为界，不得重做**（`EPIC.md:75` 前置边界、`EPIC.md:93` G6、`EPIC.md:271` 验证协议）。阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 `04` 行（`tasks/20260929-04-wheel-lifecycle`，其 `acceptance_criteria` 见该 Epic `SUBTASKS.csv` 第 5 行）已要求：单轴与整车读入同一份 wheel 子系统文件、文件读取链打通、装配阶段不再出现 `VerticalTireElement` 类型过滤、单轴侧凝结、K 台与 C 台受力激活各断言。本行只做：**(a) 独立复验其交付**（路径与内容指纹比对，**不看自报**；`grep VerticalTireElement` 在装配路径无命中）、**(b) 剩余项清单（可为空）**、**(c) 阶段一 04 未覆盖部分的收口**。
  - **开工第一步（硬性）**：必须先读阶段一 04 的 `PROGRESS.md` 与 `raw/`，把实测结论与（若需调整的）判据文本写入 `raw/stage1_04_intake.md` 并在本行记录，**同时提请父 Epic 修订 `SUBTASKS.csv`**——**本行不得直接改父表 `SUBTASKS.csv`（真源）**（`EPIC.md:271`，2026-09-29 复审修订：原文「按其实际交付重写本行 `acceptance_criteria`」已废）。
  - `depends_on = p4-03`；阶段四串行主线 p4-01 → p4-02 → p4-03 → p4-04 → p4-05（`EPIC.md:211`）。
  - **F14 现状**（`EPIC.md:147`）：`subsystems/wheel.py:44-59 _instance` 只读内置模板；`authoring/solver.py:596-598 _FILE_ROLE_TEMPLATES = ("steering","chassis")` 排除 wheel；`AssemblyRequest` 无 wheel 模板字段（`subsystems/types.py:151-160`，映射 `:249`）。**仓库无任何 `*.subsystem.json` 实体文件**（命名约定 `{name}.tpl.json` / `{name}.sub.json` / `{name}.asy.json`，`authoring/security.py:168/242/292`）——路线图说的 `wheel.subsystem.json` 是**尚不存在的目标物**。
  - **F15 现状**（`EPIC.md:149`）：按类型过滤删除在 `subsystems/assembler.py:231`，是全仓**唯一**的 `isinstance` 过滤；native 侧拒绝在 `preparation/vehicle_dynamic.py:55/900`。**这两处属阶段一 04 的动作，本行只复验。**
  - **共用文件串行**：`element_build.py` 的构造分派段归 p2-03、轮胎段归本行（`EPIC.md:220`）；`preparation/vehicle_dynamic.py` 其余段归 p2-05/p2-06，本行只改轮胎拒绝段（`EPIC.md:221`）。
  - 全局前置 `S1`：阶段一 01–07 全部 `DONE`（本行直接依赖其 04 轮端生命周期与 05 试验台非侵入，`EPIC.md:72`）；**未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:69`、`EPIC.md:79`）。
- **Known issues**:
  - **重做风险（本行最大风险）**：把阶段一 04 的动作（打通文件读取链、删 `assembler.py:231` 的 `isinstance` 过滤、单轴凝结、K/C 激活断言）当成自己的交付重做一遍。判定标准 = 本行 diff 是否只落在「阶段一 04 未覆盖项」上；发现未兑现只**登记**，不就地改代码（`EPIC.md:75`）。
  - 尚未开工，故 F14/F15 各锚点为**待复核**；任何锚点过期只在本行 `raw/` 与本节记录，不改父文件。
  - **父行 `notes` 的「两件事」已按审核修订重述**：阶段一 04 定**求解拓扑的刚性凝结**，本行定**独立复验 + 未覆盖项收口**；旧表述「本行定文件来源与类型判别」只在剩余项上成立。
  - `subsystems/rig_link.py` 的 `_reown_tires` 属**阶段一 05**，本 Epic 不碰（`EPIC.md:149`）。
- **Next action**: 先读阶段一 04（`.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-04-wheel-lifecycle/`）的 `PROGRESS.md` 与 `raw/` 全部证据，逐条对照其 `acceptance_criteria` 得出「已兑现 / 未兑现 / 未覆盖」三分清单并落盘 `raw/stage1_04_intake.md`，**若实测显示本行判据需调整，把调整后的判据一并写入该文件并在本节记录，同时提请父 Epic 修订 `SUBTASKS.csv`（本行不得直接改父表）**；再实测两侧消费的 wheel 文件路径与 sha256、跑 `grep -rn VerticalTireElement`，然后才决定是否改 `authoring/solver.py:596-598` 与 `AssemblyRequest` 模板字段。

---

## Final Summary

p4-04 **DONE**。

**第 1 步（硬性）**：阶段一 04（含 04b）的 `PROGRESS.md` 与 `raw/` 已读，
实测结论落 `raw/stage1_04_intake.md`——**剩余项清单为空**：阶段一 04 已把这四项全部兑现
（`wheel_template` 字段、`_ROLE_TEMPLATE_FIELD` 表项、`_FILE_ROLE_TEMPLATES` 放开 wheel、
类型过滤移除）。故本行按 SPEC 收缩为**独立复验**，未重做阶段一 04 的任何动作，未改父表。

**独立复验**（`raw/stage1_04_reverification.md`，不采信阶段一自报）：
单轴与整车的轮端**只有一条产出路径**——`subsystems/wheel.py`（模块指纹
`6313b9c8…1729`），模块内只有 **1** 处 `instantiate(`，由 `template_instance` 调用，
无覆盖时落到内置 `WHEEL`（`wheel_on_hub`）。父行的
`! grep -rn VerticalTireElement …/assembler.py` **exit=0**；
`grep -c isinstance …/assembler.py` = **0**（过滤机制本身不存在）。

**本行的唯一生产改动**：`assembler.py` 的两处**注释措辞**（`:36` / `:94`）。
首跑那条 grep 时退出码是 1——注释里写出了被移除的类名，使这条「按名字 grep」的机械判据
**恒假报警、失去判别力**。改写后 exit=0，注释仍完整解释「什么被移除、为什么」。
逻辑零改动。

**未做**：未跑 `tests/architecture` 整目录、`tests/adams`、`tests/cases`、
`case_parity_check.py`、`kc_perf_gate.py`（归 p4-05 与 Epic 收尾）。未重录任何基线。
