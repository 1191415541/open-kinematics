# PROGRESS：p3-06 终局独立验收（阶段三）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-06`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p3-06-acceptance
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-06-acceptance/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `TODO`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #1 — G3 逐条实跑（四种构型的滚转中心与零硬点名称嗅探）
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on = p3-03;p3-05`（`SUBTASKS.csv` 的 `p3-06`）；二者全部落地之前不得开工。跨阶段顺序为「阶段三在第一阶段完成后开工」（`EPIC.md` 行 211）；**前置 S1**——阶段一 Epic 01–07 全部 `DONE` 之前本行不得置 `IN_PROGRESS`（`EPIC.md` 前置一节：19 行实施行受此约束，四个只读冻结行不受限）。
  - **本行是合并之后的集成验收**：p3-03 与 p3-04 生产写范围不相交但**测试写范围相交**（`tests/physics/test_vehicle_physics.py` 归 p3-03，`EPIC.md` 行 228），**硬串行**（`p3-04.depends_on = p3-03`）；p3-05 接在 p3-04 之后；「每个写者各自测过」与「合起来能跑」是两件事。
  - **独立于子任务自证**（`SUBTASKS.csv` 的 `p3-06` `notes`）：不得引用 p3-03/p3-05 的 `raw/` 结论作为通过依据，必须自己重跑。
  - **G3 判据（`EPIC.md` 行 83 与行 87）**：硬点名称嗅探零命中 + 双叉臂/5 连杆/麦弗逊/扭梁四种构型各有断言 + **滚转中心高由侧倾反力虚功导数矩阵解算并有独立数值判据**（路线图 `docs/multibody_architecture_evolution.md:136-139`）；**G4 判据（`EPIC.md` 行 85 与行 89）**：3 轴（6 点）与单轮**两例**（可平衡输入必须可解 + 不可平衡输入报错点名，**不得一律写成报错**，路线图 `:188`）+ `_WHEELS` 在 `vehicle/` 与 `report/` 零命中。
  - **零回归**（`EPIC.md` 行 228 与 D5 行 64）：`kc_baseline` 与 `dynamic_hash_baseline` 逐字节不变是硬门，**禁止重录**（重录会让本行判据自毁）。
  - **本行 `validation_command` 里的 `case_parity_check.py`（无参数）不是 K/C 对标**：**实测 `packages/suspension_multibody/scripts/case_parity_check.py:1142-1161` 只接受 `--family` / `--allow-partial` / `--record`，不存在 `--check`**，写成 `--check` 必然失败（`EPIC.md` 行 234）。K/C 对标须先跑 `kc_native_probe.py` + `kc_native_c_probe.py` 再带 `--actual-dir artifacts/kc-native-probe`（`EPIC.md` 行 231）；不带 `--actual-dir` 的 `kc_parity_check` 恒过、不构成证据。
- **Known issues**:
  - **起点对照尚未落地**：`tasks/p3-01-freeze/raw/start_state.md`（skip/xfail 起点与既有失败清单）尚不存在（p3-01 未开工），本行无法区分「本次回归」与「开工前已脏」。开工前必须先读它。
  - **集成缝隙未验**：`roll_centers` 与 `static_loads` 同时被报表消费、`outputs/builtin.py` 的声明与 p3-05 字段名的一致性，只在合并后暴露（p3-05 的 `raw/` 未产生）。
  - **Done-When (a)–(j) 尚未逐条实跑**（`EPIC.md` 行 289–314）：本行必须逐条给出命令与退出码，不能只跑一遍测试就把那一个退出码当成十条的证据；阶段三直接相关的是 (c)(d)(i)。
  - **本行不改生产代码**：发现缺口只能登记阻断项并指明退回的父行，不得就地修（`SUBTASKS.csv` 的 `p3-06` `notes`）。
- **Next action**: 先按 `validation_command` 的逐字命令跑一遍合并后的全量口径（`pytest packages/suspension_multibody/tests -q` → `tests/architecture` → `dynamic_hash_sentinel.py --check` → **`case_parity_check.py`（无参数）**），逐条记退出码到 `raw/`；再分别跑 G3（`EPIC.md` 行 83/87）与 G4（`EPIC.md` 行 85/89）的 grep 与断言，全部命令与输出原文落 `raw/g3_evidence.md`、`raw/g4_evidence.md`；最后把 `EPIC.md` 行 289–314 的 Done-When (a)–(j) **逐条**实跑并逐条记退出码，落 `raw/done_when_a_to_j.md`（**不得只记录测试退出码**）。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记**已执行**的结果，不存虚构结果；口径见 `EPIC.md` 行 355）。
