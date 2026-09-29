# PROGRESS：01 冻结现状事实与判据

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `01`

## Session Start

- **Date**: 2026-09-29
- **Task name**: 20260929-01-freeze
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 7 组合装配产物快照落盘
- **Current status**: IN_PROGRESS
- **Last completed**: #4 — 门禁与数值门实测值
- **Current artifact**: `TODO.csv`
- **Key context**: 02–07 的"产物是否变化"判据依赖 #1 的快照；其余三份 raw 证据（锚点、负例、门禁值）已落盘。
- **Known issues**: 无
- **Next action**: 写 `snapshot.py`（枚举 7 个 (总成, 试验台) 组合 → 导出 bodies/points/constraints/ideal_constraints/elements/connections 的集合与名字 → 落 `raw/assembly_snapshot.json`），并让 `--check` 逐字节比对。

---

## Milestone 2: 5 条痛点锚点复核

- **Status**: DONE
- **What was done**:
  - 派两个只读 explorer 分别核 R1–R2、R3–R5 的锚点；主代理亲自复核 6 个承重锚点（`authoring/solver.py:632 file_axles_from`、`connections/policy.py:202-217` 与 `:252-296`、`connections/matcher.py:102` 及其调用点 `composition.py:258/347`）。
  - 产出 `raw/painpoint_anchors.md`：逐条给 file:line、结论与复核者。
- **Key decisions**:
  - Decision: 5 条痛点全部属实，但 3 处措辞需修正（R2 的"缺少机制"、R3 的"整车再挂轮胎"、R4 的"镜像写死"）。
  - Reasoning: 机制/事实与用户陈述不完全一致，计划必须按代码现状写（已落 `EPIC.md` F4/F5/F6）。
- **Validation**: `test -f raw/painpoint_anchors.md` → exit 0
- **Next step**: Milestone 3 — 3 轴负例

## Milestone 3: 3 轴与不对称总成的负例原文

- **Status**: DONE
- **What was done**:
  - 实测两层拒绝：`check_assembly_shape("full_vehicle", …)` 对 3 悬挂报 `requires exactly two suspension subsystem(s), found 3`；对"左右各一个悬架文件"报 `requires one front suspension`。
  - 文件路线：`placement_role="middle"` 在 `AssemblyDocument.load` 阶段就被契约 schema 拒绝（`'middle' is not one of ['any','front','rear',…]`），**早于**形状规则。
  - 产出 `raw/triaxle_refusal.md`（报错原文 + 复现方式）。
- **Key decisions**:
  - Decision: 03 的目标不只是改装配函数，必须同时放开文档形状规则与契约 schema。
- **Validation**: `test -f raw/triaxle_refusal.md` → exit 0
- **Next step**: Milestone 4 — 门禁与数值门实测值

## Milestone 4: 门禁与数值门实测值

- **Status**: DONE
- **What was done**:
  - 逐条实跑：ruff / ty / 三个架构门 / 快速集（1015 passed 1 xfailed）/ `tests/cases`（100）/ `tests/architecture`（147）/ `tests/adams`（160 + 47 环境 skip）/ kernel+contracts（60）；数值门三项（dynamic_hash_sentinel 26 artifacts 逐字节、case_parity_check 8 families、kc_perf_gate 在预算内）。
  - 记录 2026-09-29 经用户授权重录的两项基线的前后值（车辆动态快照、K/C 性能基线）。
  - 产出 `raw/baseline_commands.md`。
- **Validation**: `test -f raw/baseline_commands.md` → exit 0
- **Next step**: Milestone 1 — 7 组合快照（见恢复块）

---

## Final Summary

（未完成；#1 与 #5 仍为 TODO）
