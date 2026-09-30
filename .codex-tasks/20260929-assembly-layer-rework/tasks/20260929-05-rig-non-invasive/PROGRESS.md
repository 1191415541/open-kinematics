# PROGRESS：05 试验台非侵入

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `05`

## Session Start

- **Date**: （未开工）
- **Task name**: 20260929-05-rig-non-invasive
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-05-rig-non-invasive/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 被测总成不可变性断言
- **Current status**: NOT_STARTED
- **Last completed**: 无
- **Current artifact**: `SPEC.md`
- **Key context**: 本子任务**尚未开工**——`SPEC.md` / `TODO.csv` 是规划产物，本文件是它的恢复块，未执行任何步骤、未产生任何证据。开工前置：`04` 完成（`04` 与 `05` 都触及 `subsystems/si_assembly.py`，必须串行）；`01` 的 `#1` 完成（`snapshot.py --check` 才可用作"产物是否变化"的判据）。本行**独占** `subsystems/rig_link.py` 与 `rigs/`。
- **Known issues**: 本行**允许改变产物**（试验台不再改写被测实体），必须先给出独立于结果字节的物理等价判据并逐项登记，再决定 `supplies_wheels` 的新语义；不得用 `-k`/`--deselect` 豁免失败用例；`kc_baseline` 逐位不变仍适用（禁止重录）。
- **Next action**: 按 `TODO.csv` 从 `#1` 开始——先建立"接入前后被测 runtime 逐项一致"的断言，再做试验台夹具与轮胎两条力路径的对照。

## 待开工登记（规划产物，非实施记录）

- 本子任务尚未开工；`SPEC.md` 与 `TODO.csv` 为规划产物，所有 `TODO.csv` 行 `status` 均为 `TODO`、`completed_at` 为空。
- 本文件只记录开工所需的恢复信息与已确定的口径，**不得**用规划文本冒充实施记录；`raw/` 只存**已执行**的证据。

## 已确定的口径（来自父 Epic 裁决，不待实施确认）

- **D3（裁决，确认反转）**：删除 `subsystems/rig_link.py` 的所有权篡改（`_reown_tires`，`:315-348`；`merge_rig_link` `:243-268` 用它替换原元素），断言改为验证**被测总成不可变（immutability）**。依据：断言内部实现细节是反模式；试验台是外部激励与夹持，只应通过外加约束或接触力与被测物交互；作用线与力路径不变时数学物理严格等价。
- **判据口径（只比名字集合不够）**：接入试验台前后，被测 runtime 的 `bodies` / `points` / `constraints` / `elements` **逐项比较所有权、参数与几何值**（承载体名、`wheel_center_local`、约束端点与类型、点坐标）后一致——`_reown_tires` 正是"名字不变、所有权变"，只比实体名集合抓不到它。
- **既有测试契约的反转点**：`tests/subsystems/test_rig_link.py:172-199`（`test_the_tire_moves_to_the_bench_wheel`）今天断言 "the tire must be re-owned, not duplicated" 且 `all(body.startswith("wheel_carrier_"))`（F7）；本行必须反转这条契约。其余用例的失败要逐条判定是"契约变化"还是"真实回归"。
- **试验台只外加约束与载荷**：carrier 仍由 `WeldJoint` 焊到"声明 `wheel_center` 的那个 body"（`subsystems/rig_link.py:211-218`），约束只增不改。
- **`RigSpec.supplies_wheels` 改义**：`rigs/rig.py:73-110`（`kc_quasi_static` 为 True）的 `supplies_wheels` 语义改写为"**提供轮体**"，不再等于"夺走被测轮胎"；既有 rig 声明在新语义下**如何解释**必须有明确规则并登记。实体注入开关 `SUSPENSION_MULTIBODY_RIG_ENTITIES` 在 `subsystems/si_assembly.py:493-518`（默认开），应用点 `:467-482`。
- **不在本行范围的实体改写开关**（F7 登记）：`SUSPENSION_MULTIBODY_CONDENSE_WELDS` 与 `SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES`（`vehicle_parts.py`）、以及文档层的 `cases/vehicle_kc.py:132-137`（删 `steering_actuator`、追加驱动关节）。
- **与 D2 的关系**：本行不动单轴 K/C 的凝结，`kc_baseline` 保持逐位不变（04 的硬门）；本行只把试验台从"篡改零件"改为"外加约束/载荷"。

---

## Final Summary（未开工）

（`TODO.csv` 的 6 步全部为 `TODO`；本子任务尚未开工，未执行任何步骤、未产生任何证据）

## 2026-09-29 计划修订（复核后）

- **改了 `SPEC.md` 的 Goals 4 与 Constraints（产物判据）、Deliverables（`raw/rig_product_diff.md`）、Risk 两条、Done-When 两条、Final Validation 说明、Demo Flow 第 5 步，以及 `TODO.csv` 第 5 行**：`snapshot.py --check` 统一改为「未变化部分逐项相等 + 已登记差异」口径，登记处是 01 交付的 `tasks/20260929-01-freeze/raw/approved_deltas.json`，**未登记的差异必须让 `--check` 非零退出**。
- **为什么**：01 的交付口径已确定为「已登记差异不算变化」，若 05 只写「差异已登记」而不落 `approved_deltas.json`，`--check` 的退出码与证据就会脱节；同时必须写明 **05 是唯一被允许改变既有产物的行**，否则 04/06 的「零变化」与 05 的「允许变化」边界不清。
- **不影响其它行 id**：改动只落在 05；开关锚点由 `si_assembly.py:493-518` 修正为 `:521`（`RIG_ENTITIES_SWITCH` 实测定义在此），应用点 `:467-482` 不变。

## 2026-09-29 计划修订（第二轮复核后）

- **改动**：凡提到 `approved_deltas.json` 的位置（Goals 4、Constraints 的产物判据与「允许改变产物」条、Risk 两条、Deliverables 的 `raw/rig_product_diff.md`、Done-When 两条、Final Validation Command 说明、Demo Flow 第 5 步）统一补上完整登记口径——**四项精确匹配**（`product`/`pointer`/`before`/`after`，`product` 形如 `axle_K@kc_quasi_static`、`vehicle` 或 `rig:<名字>`，**没有前缀覆盖规则**）、登记项须带 `reason`/`registered_by`/`evidence` 且 `registered_by` 必须以 **05** 开头、**缺字段或归属不符退出 4、差异未命中登记退出 1**；并补**覆盖边界**：整车侧 5 个 rig 的试验台绑定不在 01 快照覆盖内，必须由本行自己的「接入前后运行时逐项对照」证明非侵入。所有「待 01 的 `#1` 完成后跑」「01 快照尚未生成」改为「**01 已交付**（`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘），直接跑」。
- **为什么**：第二轮复核指出登记口径写得太粗——只写「差异已登记」在**没有前缀覆盖规则**的脚本下会被判成未命中（退出 1），而缺字段或 `registered_by` 不是 05 会直接退出 4；同时 01 的快照对整车侧 5 个 rig 只记录与试验台无关的装配体，本行若只看快照会漏掉这 5 个 rig 的非侵入证据。
- **影响的行 id**：05 的 #5（`TODO.csv` 第 5 行的 acceptance 补四项精确匹配与 `registered_by` 以 05 开头，notes 补退出码语义、整车侧 5 个 rig 覆盖边界，并把「待 01 的 #1 完成后跑」改为「01 已交付、直接跑」；`SPEC.md` 的产物判据与登记口径、覆盖边界随本节生效）；`SPEC.md` 的 Goals 4、Constraints、Risk、Deliverables、Done-When、Final Validation Command、Demo Flow。行数仍为 6、status 全为 `TODO`。

---

## 2026-09-29 落地（DONE）

`_reown_tires`、`_is_replaced_tire` 与其调用点删除，`merge_rig_link` 改为纯追加（被测方任何实体都不替换、不重挂）。`test_rig_link.py` 的契约按 D3 反转为 `test_the_tire_stays_on_the_assembly_it_belongs_to`。

非侵入由本行自证：新增 `tests/subsystems/test_the_rig_is_not_invasive.py`（10 条）——两个供轮台逐项指纹对照（既有实体的质量/惯量/质心/位姿/点几何/约束/力元逐字相等，新增恰好是 `wheel_carrier_L/R` 的两个体、四点、两焊缝）、焊缝即轮心夹具（`point_a == point_b`）、7 个 rig 全覆盖（含 01 快照未覆盖的 5 个整车台）。

产物差异**为空**：冻结夹具不声明 `tires`，`_reown_tires` 本就空转；`snapshot.py --check` 零差异 → `raw/approved_deltas.json` 保持 `[]`，无需登记项。`RigSpec.supplies_wheels` 未改写，故「既有 rig 声明的解释规则登记」不适用。`dynamic_hash_sentinel --check` 26 artifact 逐字节一致。证据：`raw/rig_immutability.md`。