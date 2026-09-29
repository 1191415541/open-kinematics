# SPEC：p2-05 废除离线预采样力矩与 front_brake_bias

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-05`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p2-05` 的 `acceptance_criteria` 拆成下面 5 条，逐条可判定：

1. **`_build_wheel_torque_signals` 删除且全仓无命中**：该函数定义在 `preparation/vehicle_dynamic.py:1496`（`EPIC.md` F1 行 111），调用点 `:249`；删除后 `grep` 在源码内零命中（`EPIC.md` 行 245 (a)、行 289）。
2. **`front_brake_bias` 硬编码删除，改为模板属性槽驱动**：锚点 `preparation/vehicle_dynamic.py:1531/1533`（`EPIC.md` F1 行 111：「`front_brake_bias` 参与分配在 `:1531/1533`」）；改造后由模板 `property_slots` 驱动（与 p2-04 的属性槽标准化对接）。
3. **力矩时程一致性（或已登记的物理等价判据）**：同一工况下，**驾驶员开度/制动压力输入相同**的前提下，力矩时程在容差内一致；若必然变化，必须有**独立于结果字节的物理等价判据**（`EPIC.md` 行 245 (b)、行 288、行 312）。
4. **契约表按新机制生成且契约测试同步**：`cases/vehicle_dynamic.py` 的 `wheel_torque` / `brake_torque` 契约表（`EPIC.md` F1 行 111 与行 245 (c) 给的锚点 `:579/582`）按新机制生成；契约测试同步（`tests/cases/test_vehicle_dynamic_contract.py:185`，F6 行 122）。
5. **静止/倒车振荡失真有一条断言**：路线图 2.1 节要求的「杜绝静止倒车振荡与反向加速失真」（`EPIC.md` 行 24）必须有一条断言（`EPIC.md` 行 245 (e)）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p2-05` `notes`：「写范围 `preparation/vehicle_dynamic.py` 的力矩段与 `cases/vehicle_dynamic.py`；该文件被 p2-06 与 p4-04 也触及，故三段串行」。

- `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py` 的**力矩段**——按 `EPIC.md` 行 217 的分段：力矩段归本行、**转向段归 p2-06**、**轮胎拒绝段归 p4-04**（F15 行 145 的 `:55/:900` 拒绝逻辑与 `:900-903`）。本行段落锚点（`EPIC.md` F1 行 111）：`:71` 的 `_WHEEL_NAMES`（如因力矩路径删除而失去用途）、`:249` 调用点、`:291-302` 结果写入 `AxleDynamicsCase.wheel_torque_n_m`、`:1496` 函数定义、`:1531/1533` 的 `front_brake_bias`。
- `packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py` 的契约表段（`:579/582`，F1 行 111 与 F6 行 122）。
- 对应测试：`packages/suspension_multibody/tests/cases/`（含 `test_vehicle_dynamic_contract.py:185`）、`packages/suspension_multibody/tests/vehicle/`。
- 本目录 `raw/`。

## 禁止触碰

- **`preparation/vehicle_dynamic.py` 的转向段**：归 **p2-06**（`EPIC.md` 行 217：「`p2-06`（转向）… `p2-06` 在 p2-05 后」；行 247）。`F4` 行 118 的 `:205-211 _validate_steering_topology`（调用点 `:222`）**不得**在本行改。
- **`preparation/vehicle_dynamic.py` 的轮胎拒绝段**：归 **p4-04**（`EPIC.md` 行 217；F15 行 145 的 `:55/:900`；行 267 (c)）。本行不得触碰。
- **`templates/roles.py` 的角色表**（`:70 ROLES` 与 `:158-162` 硬断言）与 `templates/builtin.py`、`subsystems/brake.py`、`subsystems/drive.py`：归 p2-04（`EPIC.md` 行 215、行 219）。本行**不得**在 `preparation/` 里重定义属性槽——属性槽的真源在 p2-04 的模板文件。
- **`subsystems/element_build.py`**：构造分派段归 p2-03（行 216）。
- **`cases/vehicle_kc.py:128-136` 的准备期删除段**：归 **p2-06**（`EPIC.md` F5 行 120、行 247 (d)）。本行不得触碰。
- **内核侧**（元素类型与 ABI）归 p2-02；**版本常量只有 p2-02 可改**（行 225）。
- **不动轮胎力律本构**（行 97）：阶段二只改「力矩由谁产生、按什么求值」。
- **不得重录基线**（行 228、D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间**不再动**。
- **不得新增 skip/xfail**（行 229）。
- 其它子任务目录（p2-01 ~ p2-04、p2-06、p3/p4/p5 各行）、`EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`——不得改。
- `raw/` 中不得放未执行的内容（行 355）。

## 依赖与时机

- `depends_on = p2-04`（`SUBTASKS.csv` `p2-05`）。上游 S1（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 67–75）；**阶段一未完成之前不得置 `IN_PROGRESS`**（行 75）。
- 串行链：`p2-01 → p2-02 → p2-03 → p2-04 → p2-05 → p2-06`（行 203–204）。本行必须在 p2-04 的模板属性槽标准化与力矩元产出之后——`front_brake_bias` 改为「模板属性槽驱动」需要 p2-04 的槽先存在（行 245 (a)）。
- **`preparation/vehicle_dynamic.py` 三段串行**（`EPIC.md` 行 217）：本行 = 力矩段；**p2-06 在本行之后**（转向段）；**p4-04 在 p2-06 之后**（轮胎拒绝段）。三段不得并行。
- 与 p2-01 的关系：本行的判据 3（力矩时程一致性）与判据 4（契约表）必须对照 p2-01 的 `tasks/p2-01-freeze/raw/torque_path_snapshot.json`（`_build_wheel_torque_signals` 的输入/输出形状与样本数、契约表表头与行数）——**这是本行的唯一对照基准**。
- 与 p2-06 的接口：本行必须给 p2-06 留出转向段未被触碰的干净基线（`git diff` 分段自证）。

## 判据与证据落点

逐条对应 `EPIC.md` 行 245 的 (a)(b)(c)(d)(e)。

1. **(a) 删除 + grep 零命中** → `raw/retirement_grep.md`
   - 跑什么：`bash -c '! grep -rn _build_wheel_torque_signals packages/suspension_multibody/src'`（本行 `validation_command` 的第二段，逐字）；另跑 `grep -rn front_brake_bias packages/suspension_multibody/`。
   - 看什么：两条 grep 的**退出码与输出原文**；改造前全量命中清单（`file:line`）与改造后零命中的对照；被删除的调用点与结果写入点（`:249`、`:291-302`）的处置说明。
   - 落点：`raw/retirement_grep.md`。
2. **(b) 力矩时程一致性 / 物理等价判据** → `raw/torque_history_parity.md`
   - 跑什么：同一工况、**驾驶员开度与制动压力输入相同**，对比改造前后的力矩时程（改造前基准取自 p2-01 的 `raw/torque_path_snapshot.json`）。
   - 看什么：容差口径与实测差值（**必须给出数值**，不是「一致」二字）；若超容差，逐项给出**独立于结果字节的物理等价判据**（`EPIC.md` 行 228 列举的口径：自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径；**质量与质心相同不足以证明等价**）。
   - 落点：`raw/torque_history_parity.md`。
3. **(c) 契约表按新机制生成 + 契约测试同步** → `raw/contract_tables.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/cases -q`。
   - 看什么：`cases/vehicle_dynamic.py` 中 `role="wheel_torque"` / `role="brake_torque"` 契约表（`:579/582`）的**改前/改后表头与行数对照**；`tests/cases/test_vehicle_dynamic_contract.py:185`（F6 行 122）的改动「改前原文 → 改后原文 → 理由」。
   - 落点：`raw/contract_tables.md`。
4. **(d) 动态基线逐项登记（若变化）** → `raw/dynamic_hash_registration.md`
   - 跑什么：`uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check`。
   - 看什么：与 p2-01 起点值对照的逐字节结论。**若变化**：按 D5（行 64）逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」；**禁止重录基线**（行 60 的 `EPIC.md` D5 与行 228 冻结约束、行 245 (d)、行 312）。**若未变化**：记「逐字节一致」，同样落盘。
   - 落点：`raw/dynamic_hash_registration.md`。
5. **(e) 静止/倒车振荡失真断言** → `raw/stall_reverse_assertion.md`
   - 跑什么：本行新增的该条断言用例。
   - 看什么：路线图 2.1 节原文口径（`EPIC.md` 行 24：「核内力元在每次时间步进时按实时旋转角速度 ω 与打滑状态求力矩……杜绝静止倒车振荡与反向加速失真」）；断言的**测试文件:行 + 断言表达式原文 + 通过输出**；断言必须能判定「静止不倒车振荡 / 反向加速不失真」，**不得**写成「函数被调用」这类弱证据。
   - 落点：`raw/stall_reverse_assertion.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **基线不得重录**（行 228、D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；其余基线若确需变化，逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径），**质量与质心相同不足以证明等价**。`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间**不再动**。
- **`kc_parity_check.py` 的口径**（行 231）：不带 `--actual-dir` 时恒过，不构成证据。本行若用它，必须带 `--actual-dir` 并先跑生产者。
- **分层方向不可逆**（行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。本行在 `preparation/` 与 `cases/` 内改动，必须尊重该方向。
- **不得把「简化/专用」的分支写进 role 接口**（行 232）。
- **`model_dump(mode="json")` 的产物不得被改变**（行 227）：`api.py:116`/`:284` 用它算 `model_hash`。
- **不得新增 skip/xfail**（行 229）。
- **每步落地后必须重跑**（行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。**本行改的是在用的动态路径（行 312），故 `just gate-numeric` 三项必须全绿**：`dynamic_hash_sentinel.py --check`、`case_parity_check.py`、`kc_perf_gate.py`。
- **不动轮胎力律本构**（行 97）。

## 风险与回退

- **本行改的是在用的动态路径**（`EPIC.md` 行 312）：`_build_wheel_torque_signals` 的产物今天进 `dynamic_hash_baseline`。缓解：**p2-01 已先冻结路径快照**；改造后以「输入相同的力矩时程对照」为主要判据（判据 3）；若必然变化，按 D5 逐项登记并给出独立于结果字节的物理等价判据。**回退点**：`raw/torque_history_parity.md` 的差值超出容差且无物理等价判据时，退回该步，不得改口径蒙过。
- **`front_brake_bias` 删除会改变前/后制动力分配**（F1 行 111 的 `:1531/1533`）：缓解 = 分配改由 p2-04 的模板属性槽驱动，**默认值必须使既有工况的分配与改造前一致或差异被登记**；`raw/contract_tables.md` 用契约表行/值与改造前对照证明。
- **`preparation/vehicle_dynamic.py` 三段串行**（行 217）：误改转向段会与 p2-06 冲突。缓解：改动前用 `git diff` 分段自证（只出力矩段的 hunk），并把 hunk 清单记入 `raw/retirement_grep.md`。
- **契约 schema 无关**：本行不新增元素类型，**不应**触碰 `assembly.schema.json`（若确需，须遵行 218 的串行约束并在 PROGRESS 登记）。
- **既有失败**（行 320）：起点以 p2-01 `raw/baseline_notes.md` 为准；任何新增失败阻断完成，不相关既有失败独立列明。

## Done-When

- [ ] `_build_wheel_torque_signals` 已删除，`bash -c '! grep -rn _build_wheel_torque_signals packages/suspension_multibody/src'` 退出码 0；`front_brake_bias` 在力矩段无硬编码（`raw/retirement_grep.md` 含改前命中清单与改后零命中输出）。
- [ ] `front_brake_bias` 的替代路径是**模板属性槽驱动**（与 p2-04 的槽对接），分配口径由 `raw/torque_history_parity.md` 或 `raw/contract_tables.md` 给出对照。
- [ ] 同一工况、驾驶员开度与制动压力输入相同的前提下，力矩时程在容差内一致（**给出数值差**）；若不一致，已给出独立于结果字节的物理等价判据。
- [ ] `cases/vehicle_dynamic.py:579/582` 的 `wheel_torque` / `brake_torque` 契约表按新机制生成（改前/改后表头与行数对照）；`tests/cases/test_vehicle_dynamic_contract.py:185` 等受影响的契约测试已同步且理由登记。
- [ ] 路线图 2.1 的静止/倒车振荡失真有一条断言（`raw/stall_reverse_assertion.md` 含断言表达式原文与通过输出）；`EPIC.md` Done-When (a) 行 289 的三态断言中与本行相关的部分可指证据。
- [ ] `dynamic_hash_sentinel.py --check` 的结论已落盘：逐字节一致，或按 D5 逐项登记（`raw/dynamic_hash_registration.md`）；**未重录任何基线**。
- [ ] `just check-fast` 与 `just gate-numeric` 三项全绿；`tests/architecture` 绿；无新增 skip/xfail。
- [ ] 未触碰 `preparation/vehicle_dynamic.py` 的转向段（p2-06）与轮胎拒绝段（p4-04）、`cases/vehicle_kc.py`、`templates/roles.py` 的角色表、内核版本常量（`git diff` 分段自证）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/cases packages/suspension_multibody/tests/vehicle -q && bash -c '! grep -rn _build_wheel_torque_signals packages/suspension_multibody/src' && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
