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

## 2026-09-29 计划修订（复核后）

- **改了 `SPEC.md`（Goals 新增第 7 条、Constraints 的写范围与串行约束、Risk 的 `_SIDES` 条目、Deliverables 与 Done-When 的用例清单、所有 `snapshot.py --check` 位置、Demo Flow 第 4 步）与 `TODO.csv` 第 4、5、6 行**：单轮/三轮最小用例改由本行交付（EPIC 裁决：单侧装配机制在 06 的写范围）。
- **为什么**：`subsystems/si_assembly.py:56 _SIDES`、`subsystems/wheel.py:114-116 sides()`、`subsystems/element_build.py:206/:213` 的固定双侧展开都在本行范围内；写范围因此新增 `subsystems/wheel.py`（仅单侧展开段 `sides()`）与 `subsystems/element_build.py`（仅单侧展开段），与 04 的轮端实体段**分段串行（04 在前）**，原「若必须触及则改归 04」的措辞已删除。
- **产物差异口径**：`snapshot.py --check` 统一为「未变化部分逐项相等 + 已登记差异」（登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`），**06 不得新增差异**——除 05 已登记项外的差异即失败。
- **影响的行 id**：06 的 #4（task 与 acceptance 扩为不对称硬点 + 单轮/三轮）、#5（notes 的写范围归属）、#6（比对口径与「不得新增差异」）；不改行数、不改 status。

## 2026-09-29 计划修订（第二轮复核后）

- **改动**：`SPEC.md` 的 Constraints 默认零变化判据、Risk、Final Validation 说明、Demo Flow 第 4 步与 Done-When 的比对条，统一把「待 01 的 `#1` 完成后跑；01 的 7 组合快照尚未生成，`SUBTASKS.csv` 的 `01` 行仍为 `TODO`」改为「**01 已交付**（`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘，`SUBTASKS.csv` 的 `01` 行是 `DONE`），落地前后直接跑」；并把默认路径判据补成「**02/03/04/06 不得产生差异，差异非空即失败**——登记只能由 05 写」。
- **为什么**：01 第 1 行已 `DONE`，快照与 `--check` 脚本已落盘，原措辞（含 Risk 里的「01 快照尚未生成」条）会让人以为本行的零变化判据无法执行；同时「06 不得新增差异」需要与 05 独占登记的口径对齐，写清不存在「先记差异再补登记」的通道。
- **影响的行 id**：06 的 #6（`TODO.csv` 第 6 行 notes 由「待 01 的 #1 完成后跑（01 快照尚未生成）」改为「01 已交付、直接跑；登记只能由 05 写，本行不得产生差异」）；`SPEC.md` 的 Constraints、Risk、Done-When、Final Validation、Demo Flow。行数仍为 6、status 全为 `TODO`。**本节取代本文件 Key context 与 Known issues 中「待 01 的 `#1` 完成后跑」「01 的 7 组合快照尚未生成」的旧表述（原文按约定保留）：01 已交付，`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘。**

---

## 2026-09-29 部分落地（IN_PROGRESS，两项阻断项交回执行）

**已交付**：`AssemblyRequest.sides`（侧成为声明，缺省＝对称对，即 D5 的「镜像仍为默认」）；`si_assembly._sides()` 统一取值点，`element_build.element_rows`、`wheel.sides(context)`、`suspension.side_body_order` 全部按声明取值（单侧总成不再出现「body_order 点名没有贡献产出的体」）；`geometry.side_hardpoints` 改为「镜像为默认 + 显式 `name__R` 覆盖」（`__R` 键不出现在结果里，键集两侧一致）。新增 `tests/subsystems/test_optional_symmetry.py`（4 条）：默认仍成对、单侧是拓扑、**单轮（单侧悬架）装配并跑通一次 K/C study**、不对称硬点可装配且左侧不受影响。

**未交付（阻断项）**：(1) 模板/子系统文档级的 `sides`/`mirror` 声明与 `runtime_template_from` 的「不再镜像」路径 —— 这是 Done-When (b)「镜像写法与左右独立文件写法逐项一致」与 Goal 5「单侧文件契约」的兑现行；(2) 三轮整车（两悬架 + 一个单侧）需要 `assembler.AxleEntry` 携带每条目的侧（06 写范围之外的小扩展）。另：`SIDES` 收口的 grep 判据仍有 2 处命中（`si_assembly._SIDES` 的定义与缺省回退），消费点已全部改走 `_sides(request)`。

**回归**：全量 1526 passed / 1 skipped / 1 xfailed；`snapshot.py --check` 零差异（默认路径不变）；三个架构门与数值门三项全绿。

---

## 2026-09-29 收尾：两项阻断项闭环，本行 DONE

**(1) 文档级对称声明落地**：`template.schema.json` 新增可选 `sides`/`mirror`（`additionalProperties: false`，故必须显式声明）；`documents.TemplateDocument` 新增 `_check_sides`（非空、只认 left/right、不重复；`mirror=false` 必须写两侧）与 `declared_sides`/`mirrors`；`bridge._mirrored_parts`、`solver._mirrored_connections` 支持 `mirror=False`（写两侧的文件不再被二次镜像）；`solver.runtime_template_from` 对 `mirror=False` 追加 `_per_side_roles`（把 `_R` 从 role/label 摘掉，**role 保持侧无关**，与 `geometry.side_hardpoints` 读模型 `name__R` 的约定同一套）；`solver._sides_from_file` 把条目声明的侧并成 `AssemblyRequest.sides`；`_connection_name` 的兜底改为「点所在的侧」而非固定 `L`（否则写两侧的文件会把右侧点命名为 `..._R_L`）。

**(2) 三轮整车用例落地**：`assembler.AxleEntry.sides` + `compose_entries_runtime` 的转发（`sides=entry.sides or resolved.sides`）；`test_a_three_wheeled_vehicle_is_two_suspensions_and_one_corner` 断言三轮（front 双侧 + rear 单侧）装配出 3 个轮端、rear 侧无任何 `_L` 体，且**右手**单侧角装配并跑通一次 K/C study。过程中修掉 `steering.py` 的侧硬编码（`part_placement(..., "L")` → `context.request.sides[0]`；否则右侧单侧角无法携带转向）——这是 Goal 6 要求的「SIDES 消费点全部按总成声明取值」的最后一处。

**(3) Done-When (b) 实测**：新增 `tests/authoring/test_symmetry_declaration.py`（4 条）。同一份总成文件两种写法（一侧+镜像 / 两侧各自写出）实测一致：模板层 `parts` 与 `connections` **逐字段**相等；运行层体集合相等、**逐体点坐标**相等、约束名与类型相等。**登记的残余差异**：写两侧的文件其右侧点 label 仍带文件自己的 token（`lower_outer_R` vs 镜像的 `lower_outer`）——坐标与约束一致，仅点名拼写不同；测试里以注释与判据形式写明，未用弱化断言掩盖。

**(4) 验证（最终代码）**：全量 1531 passed / 1 skipped / 1 xfailed；architecture 147 passed；contracts+kernel 65 passed；ruff/ty 全过；三个架构门 OK；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致（未重录）；K/C probe（9/66 states）+ `--actual-dir` parity OK；`case_parity_check` 8 families；`kc_perf_gate --check` 在预算内；`snapshot.py --check` 零差异；`tests/data` 未被写。

**超出 06 声明写范围的两处**（为了闭环 Done-When，已在 SUBTASKS 的 06 notes 登记）：`subsystems/assembler.py`（`AxleEntry.sides` + 转发，3 行）、`subsystems/steering.py`（侧硬编码 1 处）。
