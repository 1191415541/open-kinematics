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

## 2026-09-29 计划修订（复核后）

- **改了 `SPEC.md`（Goal 2 的 (g) 与新增 (b2)、Goals 新增第 7 条 `approved_deltas.json` 审计、Constraints 两条产物判据、Risk 的拖挂条目、Done-When 第 5 条、Final Validation 的附加判据）与 `TODO.csv` 第 2、7、8 行**：拖挂铰接 (g) 改为「由 03 交付但**本行独立复跑**；仅当实测证明现有副类型不可行时才登记并提请用户裁决」；(b2) 要求本行独立复跑 06 交付的单轮/三轮最小用例。
- **为什么**：原措辞允许以「登记」代替运行证据，等于把必验用例软化；`raw/approved_deltas.json` 是 05 唯一允许改变产物的登记处，07 必须逐条审计其物理等价判据，否则「已登记差异」会变成免检通道。
- **产物差异口径**：所有 `snapshot.py --check` 位置统一为「未变化部分逐项相等 + 已登记差异」——**未登记的差异必须让 `--check` 非零退出**，即验收失败。
- **影响的行 id**：07 的 #2（补单轮/三轮独立复跑）、#7（补独立复跑与删软化措辞）、#8（补 `approved_deltas.json` 审计与口径）；行数仍为 8、status 全为 `TODO`。

## 2026-09-29 计划修订（第二轮复核后）

- **改动**：Goal 7 与 Done-When、Final Validation 说明补三条——登记必须与 **05 的 `PROGRESS.md` 逐步记录**逐条对得上、**脚本已无前缀覆盖语义**（登记不合法或缺字段/归属不是 05 时**直接退出 4**）、**整车侧 5 个 rig 的覆盖边界由本行确认 05 用了自己的运行时对照**（这 5 个 rig 的试验台绑定不在 01 快照覆盖内）；Risk 的「01 快照尚未生成（`#1` 仍为 `TODO`）」整条删除，改为「登记审计的两条硬口径」；Constraints 的产物判据与 Demo Flow 第 1 步把「待 01 的 `#1` 完成后跑；01 的 7 组合快照尚未生成」改为「**01 已交付**（`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘），直接跑」，并写明退出码语义（未登记差异退出 1、登记不合法退出 4）。`TODO.csv` 第 8 行 notes 同步（01 已交付）。
- **为什么**：登记口径已由「产品名 + 指针前缀」改为「四项精确匹配 + 归属校验」，没有前缀覆盖规则；本行若只数「有没有登记项」，会把退出 4 的非法登记当成通过，也会漏掉整车侧 5 个 rig 的非侵入证据（快照不含它们的试验台绑定）。
- **影响的行 id**：07 的 #8（`SPEC.md` 的审计口径与 `TODO.csv` notes）、`SPEC.md` 的 Goals 7、Constraints、Risk、Done-When、Final Validation、Demo Flow；不可判定项里的「01 快照尚未生成」一条随之移除。行数仍为 8、status 全为 `TODO`。**本节取代本文件 Key context 与 Known issues 中「01 的 `#1` 快照落盘」「01 的 7 组合快照尚未生成」的旧表述（原文按约定保留）：01 已交付，`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘。**

---

## 2026-09-29 终局验收（命令全跑；Done-When (b) NOT MET，Epic 不结项）

逐条命令与退出码见 `raw/final_validation.md`。摘要：全量 1526 passed / 1 skipped / 1 xfailed（26:39）；`tests/architecture` 147 passed（10:37）；contracts+kernel 65 passed；ruff/ty 全过；三个架构门 OK；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致（未重录）；K/C probe + `kc_parity_check --check --actual-dir` OK；`case_parity_check` 8 families；`kc_perf_gate --check` 在预算内；`snapshot.py --check` 零差异；`tests/data` 未被写；`approved_deltas.json` 保持 `[]`（全程零产物差异，无「未登记而通过」）。

Done-When 逐条：(a) DONE（03 用例 + 本行复跑）、(b) **NOT MET**（需 06 未交付的文档级对称声明）、(c) DONE、(d) DONE、(e) DONE、(f) DONE、(g) DONE（03 的拖挂用例本行复跑）。

**结论**：Epic 不结项。G5 未闭环，07 不给出「G1-G6 全部确认」的结论；`SUBTASKS.csv` 的 06 置 `IN_PROGRESS`。

---

## 2026-09-29 终局验收（刷新：Epic 关闭）

06 的两项阻断项闭环后（文档级对称声明、三轮整车用例），本行在最终代码上重跑全部终局命令，逐条退出码与数字见 `raw/final_validation.md` 的「刷新」一节：全量 1531 passed / 1 skipped / 1 xfailed、architecture 147、contracts+kernel 65、ruff/ty 全过、三个架构门 OK、sentinel 26 artifact 逐字节一致、K/C probe + `--actual-dir` parity OK、case_parity 8 families、kc_perf 在预算内、snapshot 零差异、`tests/data` 未被写、`approved_deltas.json` 全程 `[]`。

**Done-When：(a)–(g) 全部 MET**（(b) 附已登记的点 label 拼写残余差异；(g) 由 03 交付、本行独立复跑）。(f) 的零回归与零基线重录有本文件与 `SUBTASKS.csv` 的逐条记录支撑。**G1–G6 全部确认，Epic 关闭**。
