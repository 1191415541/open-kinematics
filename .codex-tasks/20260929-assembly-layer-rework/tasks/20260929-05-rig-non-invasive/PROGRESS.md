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
