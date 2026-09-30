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

- **Current milestone**: 全部完成（#1–#5）
- **Current status**: DONE
- **Last completed**: #5 — 快照判据接入 02–07 的 SPEC（7 个 SPEC 均引用且口径统一）
- **Current artifact**: `snapshot.py`、`raw/assembly_snapshot.json`（539,636 字节）、`raw/approved_deltas.json`（`[]`）、`raw/snapshot_notes.md`
- **Key context**: 产物键按试验台（`axle_K@<试验台>`、`axle_C@<试验台>` + 整车 `vehicle`）；`--check` 的登记项四项精确匹配（无前缀覆盖）且 `registered_by` 必须写 05（未命中登记退出 1、登记不合法退出 4）；整车侧 5 个 rig 的试验台绑定不在快照覆盖内（`_meta.coverage_boundary`），那 5 个 rig 由 05 自己的运行时对照负责。
- **Known issues**: 本行的 `validation_command` 里含全量 pytest 腿——**用户于本轮明确指示「不要跑测试」，故本轮不执行**（曾尝试后按指示停止，未采用其结果）；`AGENTS.md` 也不把全量列为日常检查（只在提交前或跨子系统改动时跑）。本行只改 `.codex-tasks/` 下的证据与脚本，未触及 `packages/**`；已跑的检查是 `just check-fast`（快速集 1015 passed / 1 xfailed、kernel+contracts 60、ruff/ty/三个架构门全绿）与 `ruff check .codex-tasks/...`。全量留到 07 收尾时按用户指示决定。
- **Next action**: 无（本行完成）；02 起按 `SUBTASKS.csv` 顺序推进。

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

## Milestone 1: 7 组合装配产物快照落盘（含 #5）

- **Status**: DONE
- **What was done**:
  - 新增 `snapshot.py`：从 `suspension_multibody.rigs.rig.RIGS`（恰 7 项）枚举组合；供轮的两个试验台**各自成键**（`axle_K@kc_quasi_static`、`axle_C@kc_quasi_static`、`axle_K@axle_dynamic`、`axle_C@axle_dynamic`，经 `preparation.kc_quasi_static.assembly_for(model, mode, rig=<试验台>)`，产物含夹具刚体 `wheel_carrier_L/R`），整车侧 5 个 rig 共用 `vehicle`（`compose_vehicle_runtime`，模式由 `_select_assembly_mode(model, "auto")` 定）；每个 rig 记录 `family`/`route`/`study`/`supplies_wheels`/drives/outputs 与 `bench_bound`。
  - 落 `raw/assembly_snapshot.json`；`--check` 采「未变化部分逐项相等 + 已登记差异」口径，登记处 `raw/approved_deltas.json`，登记项四项精确匹配且 `registered_by` 必须为 05。
  - #5：7 个子任务 SPEC 均引用该判据且口径统一（`02/03/04/06` 不得产生差异、05 唯一可登记、07 逐条审计）。
- **Key decisions**:
  - Decision: 快照载荷不含生成时间等易变字段，时间写进 `raw/snapshot_notes.md`。
    - Reasoning: 01 的 Done-When 同时要求「注明时间」与「两次运行逐字节一致」，两者只能这样并存。
  - Decision: 非空判据按「受力列整体」判定，不要求 K 读数有 `elements`。
    - Reasoning: K 的受力列是 `ideal_constraints`、力元只在 C 出现（F9 分层）；原判据把它判成失败。
  - Decision: 集合（`frozenset`）序列化为排序列表。
    - Reasoning: 实测发现集合迭代顺序随进程哈希种子变化，导致两次运行 sha256 不同（`720d8437…` vs `d8f0ccb4…`），不修则快照不能当判据。
  - Decision（第二轮审核后）: 产物键按试验台拆开、登记改为四项精确匹配。
    - Reasoning: 审核指出「7 个 rig 只映射 3 个产物」撑不起 05 的逐组合登记门；且原先「登记一条 `/elements` 即可放行其下任意变化」过宽。
- **Validation**（均本会话实跑）:
  - 两次生成 sha256 同为 `b58d35acd93560fd52046228856307ccc0cd97655b37b69dcaad86e852b24566`。
  - `--check` 五条路径：未登记差异 → 1；精确登记 → 0；`registered_by` 非 05 → 4；`before` 不符 → 1；登记缺字段 → 4；还原后 → 0。
  - `just check-fast` → 退出 0；`git diff --check` 干净。
- **Next step**: 无（本行完成）

---

## Final Summary

5 步全部 DONE。交付：`snapshot.py`、`raw/assembly_snapshot.json`（539,636 字节，sha256 `b58d35ac…`）、`raw/approved_deltas.json`（`[]`）、`raw/snapshot_notes.md`、`raw/painpoint_anchors.md`、`raw/triaxle_refusal.md`、`raw/baseline_commands.md`。唯一未在本行执行的项：`validation_command` 里的**全量 pytest 腿**——用户本轮指示「不要跑测试」，故未执行（本行未改 `packages/**`；`just check-fast` 全绿）。该偏差已登记在本行 `TODO.csv` 第 1 行 notes 与 `SUBTASKS.csv` 的 01 行 notes。
