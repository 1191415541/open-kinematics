# SPEC：p4-05 终局独立验收（阶段四）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-05`（`task_dir = tasks/p4-05-acceptance`，`depends_on = p4-04`）。
> 父 Epic 内对应段落：Goal G5（`EPIC.md:87`）、Goal G6（`EPIC.md:89`）、p4-05 验证协议（`EPIC.md:269`）、Done-When 的端到端独立验收口径（`EPIC.md:285-307`）、冻结约束（`EPIC.md:228-231`）、风险与回退的终局口径（`EPIC.md:320`）。

## Goal

把父行 `acceptance_criteria` 拆成编号分条，逐条可判定。本行是**独立于子任务自证**的终局验收行（父行 `notes`：`独立于子任务自证。记录退出码与产物差异`），只跑与外层比对，不改生产代码。

1. **G5 逐条实跑通过（防倾杆独立化）**。按 `EPIC.md:87` 的 G5 判据实跑：(i) `grep -n "\"upright_L\"\|\"upright_R\""` 在防倾杆构造路径**无命中**；(ii) **同一份防倾杆子系统文件**分别插到**双叉臂下臂**与**麦弗逊减振筒外筒**，装配产物符合**配对段声明**。判据 = 两条判据各自的命令、退出码与产物摘要。
2. **G6 逐条实跑通过（轮端统一）**。按 `EPIC.md:89` 的 G6 判据实跑：(i) `grep -rn "VerticalTireElement"` 在 `subsystems/` 与 `preparation/` 的**装配路径无命中**；(ii) `assembler.py:231` 的 `isinstance` 过滤**已不存在**；(iii) **单轴 K/C 与整车各跑通一次并引用同一份** wheel 子系统文件。判据 = 三条判据各自的命令、退出码与产物摘要。
3. **`kc_baseline` 与 `dynamic_hash_baseline` 逐位未变**。判据 = 与它们比对的结果（`dynamic_hash_sentinel.py --check` 通过；`kc_baseline/` 三个文件与工作区外/入库快照逐字节一致），逐字节判定，**不得以「测试通过」替代逐位比对**。
4. **快速集、`tests/architecture`、数值门三项全绿**。快速集 = 排除 `adams/`、`architecture/`、`cases/` 后的全量（`AGENTS.md` 第 1 节）；`tests/architecture` 整目录；数值门三项 = `dynamic_hash_sentinel.py --check`、**`case_parity_check.py`（无参数）**、`kc_perf_gate.py --check`。判据 = 三条命令各自的退出码与计数原文。**命令逐字用父表 `validation_command` 的三段**（SPEC 末尾 Final Validation Command 与父表逐字一致）：`case_parity_check.py` **不带任何参数**——实测依据 `scripts/case_parity_check.py:1142-1161` 只接受 `--family` / `--allow-partial` / `--record`，**不存在 `--check`**（`EPIC.md:234` 的实测提醒）；写成 `case_parity_check.py --check` 必然失败。
5. **无新增 skip/xfail**。判据 = 快速集与 architecture 的输出里 skip/xfail/xpassed 计数与**本行开工前实测的起点值**比对，增量必须为 0；`tests/adams` 的环境 skip 是既有的，不得增长（`EPIC.md:229`）。

## 写范围（允许改的路径）

照抄父行 `notes`：**独立于子任务自证。记录退出码与产物差异。**

- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-05-acceptance/raw/`（证据落点，开工后写入；规划轮必须为空）
- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-05-acceptance/PROGRESS.md` 的实施记录段
- 验收用临时脚本写会话 scratch（`$PI_SCRATCH_DIR`），**不落进仓库**

## 禁止触碰

- `packages/**` 下任何文件（本行**只跑不改**；发现失败不自行修，退回对应行 p4-01 ~ p4-04 或前置行）
- `.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`
- 其它行次的目录与 `raw/`（`tasks/p4-01-freeze/`、`tasks/p4-02-arb-subsystem/`、`tasks/p4-03-arb-ports/`、`tasks/p4-04-wheel-unify/`，以及全部 p2-*/p3-*/p5-*）
- 任何基线文件——本行只**读**（`EPIC.md:228`）；**任何情况下不得重录** `kc_baseline/`、`dynamic_hash_baseline.json`、`vehicle_dynamics_baseline/sha256.json`、`kc_perf_baseline_native.json`
- `raw/` 内不得放入任何虚构结果；规划轮 `raw/` 必须为空

## 依赖与时机

- `depends_on = p4-04`。阶段四串行主线 p4-01 → p4-02 → p4-03 → p4-04 → p4-05（`EPIC.md:207`）；阶段四整段在**阶段三完成后**开工（`EPIC.md:211`）。
- 全局前置 `S1`：阶段一 Epic 的 01–07 全部 `DONE`；**阶段一未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:75`）。本行的 G5/G6 判据同时依赖阶段一 03（通用装配引擎）与 04/05（轮端生命周期、试验台非侵入）（`EPIC.md:71-73`）。
- 本行是阶段四的**收尾**行；父 Epic 的完成还要看 p5-06（`EPIC.md:281`）。本行的结论只覆盖阶段四的范围。
- **本行不得与任何行并行写入**：它读取 p4-01 ~ p4-04 的全部产物与基线，验证期间工作区必须冻结（`EPIC.md:230` 的收尾口径同源）。

## 判据与证据落点

逐条对应 `EPIC.md:269`「p4-05（终局验收 · 阶段四）」：

- **G5/G6 逐条实跑** → `raw/g5_g6_rerun.md`：逐条列出 G5 的两条（`EPIC.md:87`）与 G6 的三条（`EPIC.md:89`）判据的**命令、退出码、输出原文、产物摘要**；每条标注「独立于 p4-02/p4-03/p4-04 的自证」——即命令本身不调用这几行新增的测试辅助，直接读产物/跑既有路径。
- **`kc_baseline` 与 `dynamic_hash_baseline` 逐位未变** → `raw/baseline_bitwise.md`：`dynamic_hash_sentinel.py --check` 的输出与退出码，以及 `tests/data/kc_baseline/{k_states.json,c_states.json,manifest.json}` 的逐字节判定结果（哈希或 diff 原文）。
- **快速集 + `tests/architecture` + 数值门三项全绿** → `raw/gates.md`：三类命令的退出码与计数原文（含 skip/xfail/xpassed 计数）。数值门按 `AGENTS.md` 第 2 节与 `EPIC.md:234`：`dynamic_hash_sentinel.py --check`、**`case_parity_check.py`（无参数）**、`kc_perf_gate.py --check`。**三项一个都不能漏**（复审发现原稿只跑了两项却声称三项全绿，已补第二项）；`case_parity_check.py` 的参数面实测为 `--family` / `--allow-partial` / `--record`（`case_parity_check.py:1142-1161`），**没有 `--check`**。**`kc_parity_check.py` 不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），不构成证据**（`EPIC.md:231`）——若报 K/C 对标，须先跑 `kc_native_probe.py` + `kc_native_c_probe.py` 再带 `--actual-dir artifacts/kc-native-probe`。
- **无新增 skip/xfail** → 起点值与本行实测值的对照表写入 `raw/skip_xfail.md`；增量非 0 即失败。
- **既有失败独立列明**（`EPIC.md:320`）：任何不相关的既有失败单列，不并入本行结论；任何新增失败阻断完成。

## Constraints（冻结约束）

- **独立于子任务自证**（父行 `notes` 逐字）：本行**不得**只跑 p4-02/p4-03/p4-04 自己新增的用例并宣告通过；G5/G6 的每条判据都要有独立命令直接读产物或跑既有路径（`EPIC.md:285` 的「端到端独立验收，不依赖子任务自证」同口径）。
- **基线不得重录**（`EPIC.md:228`，D5 见 `EPIC.md:64`）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；任何产物变化按 D5 逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」；质量与质心相同**不足以**证明等价。
- **F17 的「无 ARB 基线」不得用来放松**（`EPIC.md:150`）：ARB 冻结产物记录的是空字节 sha256（无 ARB），只说明当前基线不含 ARB 产物。
- **数值门三项每个阶段收尾必须全绿**（`EPIC.md:230` 与 `EPIC.md:234`）：`dynamic_hash_sentinel.py --check`、**`case_parity_check.py`（无参数）**、`kc_perf_gate.py --check`——三项缺一即不算全绿；`kc_parity_check.py` 不带 `--actual-dir` 恒过，不构成证据（`EPIC.md:231`）。
- **两套 ARB 物理不得混淆**（`EPIC.md:137`）：G5 的判据只关于**端口与插接**（装配产物符合配对段），不要求两套物理数值等价；验收报告里不得用一侧数值证明另一侧。
- **分层方向不可逆**（`EPIC.md:226`）：本行要确认阶段四的改动未破坏 `modeling -> templates -> subsystems -> preparation/studies -> cases` 方向，未被 `report/` 反向 import。
- **不得新增 skip/xfail**（`EPIC.md:229`）；`tests/adams` 的环境 skip 是既有的，不得增长。
- **未重录任何数值/性能基线**（`AGENTS.md` 提交前清单）：本轮**不得**以任何理由重录基线。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据。
- **不得用 `-k`、`--deselect` 长期豁免失败用例**（`AGENTS.md` 第 7 节）：失败要么退回对应行修，要么按既有登记说明原因。

## 风险与回退

- **子任务自证与终局验收混淆（父行 `notes` 的核心风险）**：p4-02 ~ p4-04 各自的测试通过≠目标达成。缓解 = 本行逐条独立实跑 `EPIC.md:87` / `EPIC.md:89` 的判据，命令不依赖这几行新增的测试辅助。
- **进度门与基线门容易被「测试通过」替代**：`kc_baseline` 与 `dynamic_hash_baseline` 要求**逐字节**判定。缓解 = `raw/baseline_bitwise.md` 记录哈希/diff 原文，不接受「相关测试通过」作为证据。
- **共用文件曾被多行触及（`EPIC.md:216-217`）**：`element_build.py` 与 `preparation/vehicle_dynamic.py` 的段落归属若被越界改动，会在本行暴露（例如 `grep` 命中残留、架构门失败）。缓解 = 发现越界即**退回对应行**，本行不自行修改。
- **既有失败混入结论**（`EPIC.md:320`）：缓解 = 起点值在本行开工第一步实测并记录；不相关的既有失败独立列明，任何新增失败阻断完成。
- **回退**：本行不改代码，无回退动作；若某项判据失败，把结论与失败原文交给对应行（p4-02 / p4-03 / p4-04）退回修复后重跑本行。**不得**以重录基线或改弱断言的方式让本行「通过」。

## Done-When

- [ ] G5 的两条判据（防倾杆构造路径无 `"upright_L"`/`"upright_R"`；同一份防倾杆子系统文件分别插双叉臂下臂与麦弗逊减振筒外筒并符合配对段声明）已独立实跑，命令、退出码与产物摘要落盘。
- [ ] G6 的三条判据（`VerticalTireElement` 在装配路径 grep 无命中；`assembler.py:231` 的 `isinstance` 过滤消失；单轴 K/C 与整车引用同一份 wheel 文件并各跑通一次）已独立实跑，命令、退出码与产物摘要落盘。
- [ ] `kc_baseline` 与 `dynamic_hash_baseline` 逐字节未变（逐位比对原文落盘）。
- [ ] 快速集、`tests/architecture`、数值门三项全绿（退出码与计数原文落盘）；数值门三项**逐字按父表 `validation_command`**：`dynamic_hash_sentinel.py --check`、**`case_parity_check.py`（无参数）**、`kc_perf_gate.py --check`。
- [ ] 无新增 skip/xfail；不相关的既有失败独立列明；任何新增失败已阻断并退回对应行。
- [ ] 本行未修改 `packages/**` 下任何文件与任何基线。
- [ ] 父行 `validation_command` 退出码为 0。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests -q && uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py && uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
```
