# SPEC：p3-06 终局独立验收（阶段三）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-06`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p3-06` 的 `acceptance_criteria` 拆成下面 6 条，逐条可判定：

1. **G3 逐条实跑通过（四种构型的滚转中心）**：`EPIC.md` 行 83 与行 87 的 G3 判据逐条实跑——(i) `grep -n "UPPER_INBOARD\|LOWER_INBOARD\|UCA_\|_BODY_ALIASES"` 在新引擎路径无命中；(ii) 同一套算法对双叉臂、5 连杆、麦弗逊、扭梁四种构型**各给出有限且可对照的滚转中心**（四种构型各有断言）；(iii) **滚转中心高由侧倾反力虚功导数矩阵解算**（路线图 `docs/multibody_architecture_evolution.md:136-139` 的指定口径），且有一条独立数值判据，**不是**只求几何连线交点。判据可判定：命令 + 退出码 + 断言清单；**用独立于子任务自证的证据**（见第 4 条）。
2. **G4 逐条实跑通过（广义静平衡与动态通道）**：`EPIC.md` 行 85 与行 89 的 G4 判据逐条实跑——(i) 3 轴（6 点接触）实跑静平衡且力矩平衡残差在容差内；**单轮（N = 1）跑两个例子**：质心投影与接触点重合这类**可平衡输入必须可解**，平衡条件不成立的输入**报错点名**——**不得把单轮一律写成报错**（路线图 `:188`「支持 N 点接触面」的字面要求）；(ii) `grep -n "_WHEELS = (\"front_left\""` 在 `vehicle/` 与 `report/` 无命中。判据可判定：命令 + 退出码 + 数值断言输出。
3. **零回归**：`kc_baseline` **逐字节未变**且 `dynamic_hash_baseline` **未变**（`EPIC.md` 行 228 与 D5 行 64）；快速集与 `tests/architecture` 与数值门三项全绿；**无新增 skip/xfail**（`EPIC.md` 行 229）。判据可判定：基线文件的哈希/Git 状态输出 + 各门退出码 + skip/xfail 计数与起点（`tasks/p3-01-freeze/raw/start_state.md`）的比较。
4. **独立于子任务自证**：`SUBTASKS.csv` 的 `p3-06` `notes` 明写「独立于子任务自证 不得采信子任务自报」。本行的每一条结论都必须来自本行**自己实跑**的命令与产物，不得引用 p3-03/p3-05 的 `raw/` 结论作为通过依据（可作为对照线索，但必须自己重跑）。判据可判定：`raw/` 中每条结论都带本行执行的命令 + 退出码 + 输出。
5. **K/C 对标不得采信自比较**：`SUBTASKS.csv` 的 `p3-06` `notes` 明写「K/C 对标不得用不带 `--actual-dir` 的 `kc_parity_check`（自比较恒过）」（同 `EPIC.md` 行 231）。本行若做 K/C 对标，必须先跑 `kc_native_probe.py` + `kc_native_c_probe.py` 再带 `--actual-dir artifacts/kc-native-probe` 判定（`EPIC.md` 行 231 的命令序列）；若不做，须在 `raw/` 中记「未做 K/C 对标」与原因，**不得**用恒过的自比较充当证据。逐条记录退出码（`SUBTASKS.csv` 的 `p3-06` `notes`）。
6. **逐条实跑 `EPIC.md` Done-When 的 (a)–(j) 并记录退出码**：验收**不能只记录测试的退出码**——`EPIC.md` 行 289–314 的 Done-When 列了 (a)–(j) 十条终局验收项，本行必须**逐条实跑**并把**每一条自己的命令与退出码**记下来（不是只跑一遍测试就把那一遍的退出码当成全部十条的证据）。判据可判定：`raw/done_when_a_to_j.md` 里 (a)–(j) 十条逐条有命令 + 退出码 + 输出原文或产物路径，缺任何一条即未达成。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p3-06` `notes`：**独立于子任务自证；不得采信子任务自报。K/C 对标不得用不带 `--actual-dir` 的 `kc_parity_check`（自比较恒过）。逐条记录退出码**。

- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-06-acceptance/raw/`（证据，**只存已执行的结果**）
- 本目录下的临时脚本（若需要脚本，落本目录或会话 scratch）
- 会话 scratch：`$PI_SCRATCH_DIR`（临时中间产物；K/C 对标的探针产物若落在 `artifacts/kc-native-probe` 按 `EPIC.md` 行 231 的命令口径）

`EPIC.md`「并行与写范围约束」（行 213–221）中与本行相关的条目：

- **p3-03 与 p3-04 生产写范围不相交，但测试写范围相交**（`EPIC.md` 行 228；**硬串行，`p3-04.depends_on = p3-03`**），**p3-05 在 p3-04 之后**（行 220）。本行**在 p3-03 与 p3-05 都落地之后**才开工（`SUBTASKS.csv` 的 `p3-06.depends_on=p3-03;p3-05`），因此本行是**合并之后的集成验收**——「每个写者各自测过」与「合起来能跑」是两件事，本行的意义就在这里。
- 本行**不改任何生产代码**（`p3-06` 的 `notes` 只授权 `raw/` + 脚本 + scratch）；发现缺口时登记为阻断项并退回上一行，**不得就地修**。

## 禁止触碰

- `packages/**` 下任何文件——本行是验收行，一行不改（`SUBTASKS.csv` 的 `p3-06` `notes`）。
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及 `tasks/` 下其它子任务目录（含 p3-01 ~ p3-05 的 `raw/`）——**不得代写、不得预填、不得改写别人已落盘的证据**。
- 任何冻结基线文件：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md` 行 228 与 D5 行 64）——只读，**禁止重录**。本行的零回归判据就是「这些文件未变」，重录会让判据自毁。
- 用 `-k` / `--deselect` 长期豁免失败用例（`AGENTS.md` 第 7 节）。
- `raw/` 中不得放未执行的内容（`EPIC.md` 行 355）。

## 依赖与时机

- 父行 `depends_on = p3-03;p3-05`（`SUBTASKS.csv` 的 `p3-06`）。二者全部落地之前不得开工。
- **前置 S1**：本行以阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 01–07 全部 `DONE` 为前置（`EPIC.md` 前置一节：**19 行实施行与终局验收行**受此约束；四个只读冻结行 p2-01/p3-01/p4-01/p5-01 不受限、可立即开工）；阶段一未完成之前本行不得置 `IN_PROGRESS`。
- **本行是阶段三的终局行**（`EPIC.md` 行 206 与行 259）：其结果决定阶段三能否收口、阶段四能否开工（`EPIC.md` 行 211 的跨阶段顺序）。
- **节奏与门禁**（`EPIC.md` 行 230 与父 `PROGRESS.md` 的门禁节）：本行须跑快速集 + `tests/architecture` + 数值门三项；这是「每阶段收尾」的口径。`kc_parity_check.py` 的用法受限（见 Goal 5）。
- **禁止依据旧任务 DONE 结论**（`EPIC.md` 行 320 的口径）——所有事实以本行实跑为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 259 的验证协议（本行整段：逐条实跑 G3/G4 判据，记录退出码与产物差异；跑快速集、`tests/architecture`、数值门三项；确认 `kc_baseline` 逐位未变）。每条写清「跑什么命令、看什么输出、证据落到哪个文件」。

1. **G3 逐条实跑** → `raw/g3_evidence.md`
   - 跑什么：`EPIC.md` 行 83 的两条判据——(i) 硬点名称嗅探的 `grep`（含 `_BODY_ALIASES`）；(ii) 四种构型各自的滚转中心断言（pytest 命令 + 用例名）。命令、退出码、输出原文全部记录。
   - 看什么：零命中；四种构型**逐种**有断言且通过。逐条记退出码（`SUBTASKS.csv` 的 `p3-06` `notes`）。
   - 落点：`raw/g3_evidence.md`。
2. **G4 逐条实跑** → `raw/g4_evidence.md`
   - 跑什么：`EPIC.md` 行 85 与行 89 的 G4 判据——(i) 3 轴（6 点接触）实跑静平衡且残差在容差内，**单轮（N = 1）跑两个例子**（质心投影与接触点重合这类可平衡输入**必须可解**；平衡条件不成立的输入报错点名）——**不得把单轮一律写成报错**；`EPIC.md` 行 87 的 G3 判据——滚转中心高**由侧倾反力虚功导数矩阵解算**并有独立数值判据（路线图 `docs/multibody_architecture_evolution.md:136-139`）；(ii) `_WHEELS` 的 `grep` 在 `vehicle/` 与 `report/` 无命中。
   - 看什么：残差数值与容差；grep 零命中；单轮**可解例的数值结果**与**不可解例的报错消息点名**各一条；虚功导数判据的数值与容差。
   - 落点：`raw/g4_evidence.md`。
3. **零回归：基线逐位未变** → `raw/zero_regression.md`
   - 跑什么：`dynamic_hash_sentinel.py --check`（`EPIC.md` 行 230 的数值门之一）；`kc_baseline` 三文件与 `dynamic_hash_baseline.json` 的哈希与 `git status` / `git diff --stat` 状态；`case_parity_check.py`（**无参数**——**实测 `scripts/case_parity_check.py:1142-1161` 只接受 `--family` / `--allow-partial` / `--record`，不存在 `--check`，写成 `--check` 必然失败**，`EPIC.md` 行 234）；`kc_perf_gate.py --check`。
   - 看什么：数值门三项退出码全 0；基线文件哈希与基线记录一致（或 `git diff` 对它们为空），**未重录**。**注意**：与本次改动无直接关系的文件若已在开工前就是脏的，须按 `tasks/p3-01-freeze/raw/start_state.md` 的起点对照区分，不得混算。
   - 落点：`raw/zero_regression.md`。
4. **Done-When (a)–(j) 逐条实跑并逐条记退出码** → `raw/done_when_a_to_j.md`
   - 跑什么：`EPIC.md` 行 289–314 的 Done-When 清单 (a)–(j) **逐条**实跑——每条自己跑命令、自己记退出码与输出（**不得**用某一次全量测试的退出码充当十条的证据，`EPIC.md` 行 289 的「端到端独立验收（不依赖子任务自证）逐条实跑」）。阶段三直接相关的是 (c)（四种构型 + 滚转中心高由侧倾反力虚功导数矩阵解算）、(d)（3 轴 6 点 + 单轮可解例与不可解报错例）、(i)（零回归），其余各条也要给出「本阶段是否触及 + 证据或不适用的理由」。
   - 看什么：十条**逐条**的行号引述 + 命令 + 退出码 + 输出原文或产物路径；缺任何一条即未达成。
   - 落点：`raw/done_when_a_to_j.md`。
5. **快速集 + `tests/architecture` + skip/xfail 计数** → `raw/fast_and_arch.md`
   - 跑什么：快速集（除 `adams/`、`cases/`、`architecture/` 的命令口径见 `AGENTS.md` 第 1 节，或按本行 `validation_command` 的全量口径）、`tests/architecture`、`kernel` 与 `contracts` 两包。
   - 看什么：退出码全 0；skip/xfail 计数与 `tasks/p3-01-freeze/raw/start_state.md` 的起点比较**未增长**（`EPIC.md` 行 229）。
   - 落点：`raw/fast_and_arch.md`。
6. **独立性与 K/C 对标口径自证** → `raw/independence.md`
   - 跑什么：记录本行每条结论所执行的命令（证明没有直接引用 p3-03/p3-05 的 `raw/` 结论）；若做 K/C 对标，记 `kc_native_probe.py` → `kc_native_c_probe.py` → 带 `--actual-dir artifacts/kc-native-probe` 的 `kc_parity_check.py` 三步；若不做，记「未做 K/C 对标」与原因。
   - 看什么：独立性可核（每条结论带本行命令）；**不得**出现不带 `--actual-dir` 的 `kc_parity_check` 作为证据（自比较恒过，`EPIC.md` 行 231）。
   - 落点：`raw/independence.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **不改动任何 K/C 读数**：`tests/data/kc_baseline/` 与 `dynamic_hash_baseline.json` **逐字节不变是硬门**（`EPIC.md` 行 228 与 D5 行 64）；本行**只读**这两个基线并断言未变，**禁止重录**。`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间不动。
- **分层方向不可逆**（`EPIC.md` 行 226）：验证时同时确认 `report/` 未 import native/kernel/solver、未自求力律——`legacy_surface_gate.py --check` 必须退出码 0（`report_native_import` / `report_constitutive_call` 规则）。
- **ABI 未变**：阶段三不改 ABI（D3，`EPIC.md` 行 62）；本行须确认 `tests/architecture/test_kernel_abi_version_single_source.py` 仍绿（`EPIC.md` 行 225 的单一真源门）。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）；`tests/adams` 的环境 skip 是既有的，不得增长。计数与 p3-01 的起点对照。
- **基线不得重录，也不得用 `-k` / `--deselect` 豁免**（`AGENTS.md` 第 7 节与 `EPIC.md` 行 228）。
- **`kc_parity_check.py` 的口径**（`EPIC.md` 行 231）：不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），**不构成证据**。
- **本行不改生产代码**：发现缺口 → 登记阻断项 → 退回 p3-03/p3-05 或提请主代理裁决，**不得就地修**（`SUBTASKS.csv` 的 `p3-06` `notes`）。

## 风险与回退

- **「全绿」可能只是各写者自测通过**（`EPIC.md` 行 228 的写范围约束与 `SUBTASKS.csv` 的 `p3-06` `notes`）：p3-03 与 p3-04 生产写范围不相交但测试写范围相交（硬串行，`p3-04.depends_on = p3-03`）、p3-05 接在 p3-04 后；**集成缝隙**（`roll_centers` 与 `static_loads` 同时被报表消费、`outputs/builtin.py` 的声明与 p3-05 字段名的一致性）只在合并后暴露。回退/缓解：本行必须按 `validation_command` 跑**合并后**的全量口径，并把「本行自跑」与「子任务自报」分开记录；发现不一致时登记阻断项并退回对应行。
- **4 轮情形向后兼容逐位一致（p3-04 的硬门，本行复验）**：`vehicle/service.py:40` 在生产用（F8 `EPIC.md` 行 128）。回退/缓解：本行独立复跑 4 轮路径并与 p3-01 起点对照；差异非空即阻断，**不得**用「差异很小」放行。
- **报表字段集合变化未登记**（D5，`EPIC.md` 行 64）：若 p3-05 改了字段集合而没有逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」，本行判为**阻断项**。
- **`case_parity_check.py` 与 K/C 对标的混淆**（`EPIC.md` 行 231 与行 234）：本行 `validation_command` 里的是 **`case_parity_check.py`（无参数）**（8 个族的族间等价），**不是** K/C 对标；**实测 `scripts/case_parity_check.py:1142-1161` 只接受 `--family` / `--allow-partial` / `--record`，不存在 `--check`**，写成 `--check` 必然失败。K/C 对标需要先跑两个探针再带 `--actual-dir`。两者不得混为一谈。
- **既有失败**（`EPIC.md` 行 320）：起点值取自 `tasks/p3-01-freeze/raw/start_state.md`；任何新增失败阻断完成，不相关的既有失败独立列明。
- **基线在开工前已脏**：本行的「逐位未变」判据必须相对**起点**而非绝对干净的工作区；否则会把与本次改动无关的既有差异算成回归。起点对照见 `tasks/p3-01-freeze/raw/start_state.md`。

## Done-When

- [ ] `raw/g3_evidence.md` 非空：G3 判据逐条实跑，含命令、退出码、输出原文；四种构型**逐种**有断言且通过；硬点名称嗅探零命中；**滚转中心高由侧倾反力虚功导数矩阵解算**且有独立数值判据（`EPIC.md` 行 87、路线图 `:136-139`）。
- [ ] `raw/g4_evidence.md` 非空：G4 判据逐条实跑；3 轴（6 点）有证据；单轮（N = 1）**可解数值例与不可解报错例各一**（**不得一律写成报错**，`EPIC.md` 行 89、路线图 `:188`）；`_WHEELS` 在 `vehicle/` 与 `report/` 零命中。
- [ ] `raw/zero_regression.md` 非空：数值门三项退出码全 0；`kc_baseline` 与 `dynamic_hash_baseline` 相对起点**逐字节未变**；未重录任何基线。
- [ ] `raw/fast_and_arch.md` 非空：快速集、`tests/architecture`、`kernel` + `contracts` 两包退出码全 0；skip/xfail 计数**未超过** p3-01 起点。
- [ ] `raw/independence.md` 非空：每条结论带本行执行的命令；K/C 对标口径合规（要么三步带 `--actual-dir`，要么明确记「未做」）。
- [ ] `raw/done_when_a_to_j.md` 非空：`EPIC.md` 行 289–314 的 Done-When (a)–(j) **逐条**有行号引述 + 命令 + 退出码 + 输出原文或产物路径，**不是**只记录测试退出码（`EPIC.md` 行 289）。
- [ ] 未修改 `packages/**` 任何文件、未修改 `EPIC.md` / `SUBTASKS.csv` / 父 `PROGRESS.md`（`git status` / `git diff --stat` 自证）。
- [ ] 缺口（若有）已登记为阻断项并指明退回的父行；**本行未就地修**。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests -q && uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
```
