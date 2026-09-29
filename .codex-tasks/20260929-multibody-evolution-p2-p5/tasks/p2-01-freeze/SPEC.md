# SPEC：p2-01 冻结现状事实与判据（阶段二）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-01`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p2-01` 的 `acceptance_criteria` 拆成下面 5 条，逐条可判定：

1. **力矩路径快照落盘**：制/驱动力矩从工况文档到内核的**完整路径快照**——`_build_wheel_torque_signals`（`preparation/vehicle_dynamic.py:1496`，调用点 `:249`）的输入/输出形状与样本数；契约表 `role="wheel_torque"` / `role="brake_torque"`（`cases/vehicle_dynamic.py:579/582`）的表头与行数。这是「力矩内建后同一工况数值一致」的对照基准。
2. **后轮转向限制的负例实测**：构造一份后齿条不固定的文档，记录 `preparation/vehicle_dynamic.py:205-211 _validate_steering_topology`（调用点 `:222`）与 `authoring/vehicle.py:112-119`（其中 `:119` 为强制写死点）**两层**的拒绝原文。
3. **门禁与数值门起点值**：三条架构门脚本与数值门三项的实测值与退出码，逐条落盘（命令与口径见 `EPIC.md` 行 230）。
4. **F1–F5 锚点复核清单**：`EPIC.md` 的 F1/F2/F3/F4/F5 逐条复核，给出 `file:line` + 原文；**锚点过期即记「已过期」，不得沿用过期锚点**。
5. **起点状态登记**：`dynamic_hash_sentinel.py --check` 的起点结论（13 个轴侧动态用例口径，`EPIC.md` 行 239 与行 311）与既有失败清单（`EPIC.md` 行 320：本 Epic 起点须先实测，任何新增失败阻断完成，不相关既有失败独立列明）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p2-01` `notes`：**写范围仅本 Epic 目录 `raw/` 与脚本与会话 scratch；不写生产代码**。

- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-01-freeze/raw/`（证据，**只存已执行的结果**）
- 本目录下的临时脚本（若需要脚本，落本目录或会话 scratch）
- 会话 scratch：`$PI_SCRATCH_DIR`（临时中间产物）

## 禁止触碰

- `packages/**` 下任何文件——本行是冻结行，一行不改（`SUBTASKS.csv` `p2-01` `notes`：「不写生产代码」）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录（p2-02 ~ p2-06，及 p3/p4/p5 各行）——**不得代写、不得预填**。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5，行 64）——只读，**禁止重录**。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355：`raw/` 只存**已执行**的证据，不存虚构结果）。

## 依赖与时机

- 父行 `depends_on` **为空**：本行是**纯只读冻结行**（写范围只有本行 `raw/` 与会话 scratch，冻结的是**阶段二的现状**——力矩与转向，**不读前一阶段任何会被本 Epic 改写的产物**），故**本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。
- **写范围只有本行 `raw/` 与会话 scratch，不写生产代码**：本行一行不改 `packages/**`（`SUBTASKS.csv` `p2-01` `notes`）。
- **阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**（即 p2-02 ~ p2-06 与终局验收行），不影响本行长；实测起点（2026-09-29）：阶段一 01/02 `DONE`、03 `IN_PROGRESS`、04–07 `TODO`（`EPIC.md` 行 79 与 F25 行 176，仅作背景登记）。
- 本行是阶段二串行主线的**第一行**：`p2-01 → p2-02 → p2-03 → p2-04 → p2-05 → p2-06`（`EPIC.md` 行 203–204）。p2-02 依赖本行的路径快照与锚点复核清单作为起点证据。
- 与其它阶段的时机：阶段三在阶段一完成后开工、阶段四在阶段三后、阶段五在阶段四后（`EPIC.md` 行 219）——本行不受影响，但**不得提前触及其它阶段的写范围**（如 `vehicle/roll_centers.py`、`templates/roles.py`）。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 237，父行 `notes`）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 237 的 (a)(b)(c)(d)；每条都要写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **(a) 力矩路径快照** → `raw/torque_path_snapshot.json`
   - 跑什么：在 `preparation/vehicle_dynamic.py` 的 `_build_wheel_torque_signals` 调用路径（调用点 `:249`）上取一次真实工况，记录该函数的输入（驾驶员开度/制动压力信号、时间栅格）与输出（数组形状与样本数）；再取 `cases/vehicle_dynamic.py:579/582` 生成的 `wheel_torque` / `brake_torque` 契约表的表头与行数。
   - 看什么：样本数与数组形状的实测值；契约表表头字符串与行数。锚点来源：`EPIC.md` F1（`preparation/vehicle_dynamic.py:71` 的 `_WHEEL_NAMES`、`:1496` 定义、`:249` 调用、`:291-302` 写入 `AxleDynamicsCase.wheel_torque_n_m`）。
   - 落点：`raw/torque_path_snapshot.json`（本行 `validation_command` 的第二段 `test -s` 断言它非空）。
2. **(b) 后轮转向限制负例** → `raw/rear_steer_refusal.md`
   - 跑什么：构造一份后齿条不固定的文档，分别走准备层与文档导出层；行号锚点 `preparation/vehicle_dynamic.py:205-211`（调用点 `:222`）与 `authoring/vehicle.py:112-119`（`EPIC.md` F4）。
   - 看什么：两层的**拒绝原文**逐字记录（异常类型 + 消息全文），并注明哪一层先触发。
   - 落点：`raw/rear_steer_refusal.md`（`validation_command` 第三段 `test -s` 断言它非空）。
3. **(c) 门禁与数值门起点** → `raw/gates_baseline.md`
   - 跑什么（逐条记命令与退出码）：`legacy_surface_gate.py --check`、`check_module_layering.py --strict --final`、`check_composable_release.py --skip-isolation`（`EPIC.md` 行 226 与行 230 列出的**三条秒级架构门脚本**）；数值门三项 `dynamic_hash_sentinel.py --check`、`case_parity_check.py`、`kc_perf_gate.py --check`（`EPIC.md` 行 230）。
   - 看什么：每条命令的退出码与关键输出；**`kc_parity_check.py` 不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），不构成证据**（`EPIC.md` 行 231）——若跑它，必须注明是否带 `--actual-dir`。
   - 落点：`raw/gates_baseline.md`。
4. **(d) F1–F5 锚点复核** → `raw/anchors_F1_F5.md`
   - 跑什么：逐条核 `EPIC.md` F1（行 111–112）、F2（行 114）、F3（行 116）、F4（行 118）、F5（行 120）给出的 `file:line`，取原文。
   - 看什么：每条给「锚点 = 文件:行 + 该行原文摘要 + 是否过期」；已知的过期项是路线图的 `:1506`（F1 明说已过期）与「轴 15；整车 30」的 ABI 说法（F3 明说已过期）——**这两条必须复核并记「已过期」**。
   - 落点：`raw/anchors_F1_F5.md`。
5. **起点状态** → 并入 `raw/gates_baseline.md`（数值门起点结论）与 `raw/baseline_notes.md`（既有失败与 skip/xfail 清单）。
   - 看什么：13 个轴侧动态用例口径下 `dynamic_hash_sentinel.py --check` 的起点结论；既有失败逐条列明并标注是否与本 Epic 相关（`EPIC.md` 行 320）。
   - 落点：`raw/baseline_notes.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **基线不得重录**（行 228，D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间**不再动**。本行只**读**基线并记起点值。
- **不得新增 skip/xfail**（行 229）：`tests/adams` 的环境 skip 是既有的，不得增长。本行记录起点 skip/xfail 计数，作为后续各行的对照。
- **每步落地后必须重跑**（行 230）：本行落地的是文档与证据，改动后重跑三条秒级架构门即可；不要求跑全量。
- **`kc_parity_check.py` 的口径**（行 231）：不带 `--actual-dir` 时恒过，不构成证据。
- **分层方向不可逆**（行 226）：本行**不改任何生产代码**，故只做只读核对；若核对中发现了跨层反向导入，记为事实，**不在本行修**。

## 风险与回退

- **ABI 变更风险（`EPIC.md` 行 311）**：p2-02 是全 Epic 最大风险，其判据依赖「13 个轴侧动态用例逐字节一致」。回退/缓解：**本行必须先把起点值钉死**（`raw/gates_baseline.md` + `raw/baseline_notes.md`）；起点值缺失时 p2-02 无对照，后续不得开工。
- **p2-05 改的是在用的动态路径（行 312）**：`_build_wheel_torque_signals` 的产物今天进 `dynamic_hash_baseline`。缓解：本行的路径快照（Goal 1）就是 p2-05 的「输入相同则力矩时程对照」的基准。
- **既有失败（行 320）**：本 Epic 起点须先实测，各子任务的 p*-01 负责记录各自的起点值；任何新增失败阻断完成，不相关既有失败独立列明。
- **过期说法风险**：F1 已点名路线图 `:1506` 过期、F3 已点名「轴 15；整车 30」过期。若本行复核时发现 `EPIC.md` 的某个 F 锚点也过期，**记为事实并写明当前真实行号**，不得静默沿用（`EPIC.md` 行 107：「带『修正』的条目是路线图文档陈述与代码现状不一致之处，计划按代码现状写」）。

## Done-When

- [ ] `raw/torque_path_snapshot.json` 非空，含 `_build_wheel_torque_signals` 的输入/输出形状与样本数、`wheel_torque`/`brake_torque` 契约表的表头与行数。
- [ ] `raw/rear_steer_refusal.md` 非空，含 `preparation/vehicle_dynamic.py:205-211` 与 `authoring/vehicle.py:112-119` **两层**的拒绝原文（异常类型 + 消息）与触发顺序。
- [ ] `raw/gates_baseline.md` 非空，含三条架构门与数值门三项的**命令 + 退出码 + 关键输出**，并注明 `kc_parity_check.py` 是否带 `--actual-dir`。
- [ ] `raw/anchors_F1_F5.md` 非空，F1–F5 逐条给 `file:line` + 原文 + 是否过期。
- [ ] `raw/baseline_notes.md` 非空，含 skip/xfail 起点计数与既有失败清单（标注是否与本 Epic 相关）。
- [ ] 未修改 `packages/**` 任何文件、未修改 `EPIC.md`/`SUBTASKS.csv`/父 `PROGRESS.md`、未重录任何基线（用 `git status` / `git diff --stat` 自证并记入 `raw/baseline_notes.md`）。
- [ ] 阶段二后续行（p2-02 ~ p2-06）的起点证据全部来自本行产物，无一处依据旧任务 DONE 结论（`EPIC.md` 行 237）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py -q && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-01-freeze/raw/torque_path_snapshot.json && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-01-freeze/raw/rear_steer_refusal.md
```
