# SPEC：p5-06 终局独立验收（阶段五）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-06`

## Goal

拆自 `SUBTASKS.csv` 第 24 行 `acceptance_criteria`，逐条可判定：

1. **G7 与 G8 逐条实跑通过**：`EPIC.md:95`（G7：`simulate(assembly_document, case_document)` 成为公共入口；`FrontAxleModel` 降级为向下兼容适配器**不得删除**；`legacy_surface_gate.py` 与 `check_composable_release.py` 全绿）与 `EPIC.md:97`（G8：信号总线暴露传感器测点与执行器输入；**ABS 或 ESC 之一的实际反馈闭环**；FMI 导出产物存在且可独立校验）。判据：逐条给出命令、退出码与产物。
2. **全量 `pytest packages/suspension_multibody/tests -q` 通过，且 skip 与 xfail 未增长**（`EPIC.md:285` 与冻结约束 `EPIC.md:233`；`AGENTS.md` 第 3 节的基线 **1297 passed, 1 skipped, 1 xfailed**）。判据：全量结果原文 + skip/xfail 计数与基线的对照。
3. **`tests/architecture`、两个架构包测试（`suspension_contracts` + `suspension_kernel`）、`ruff`、`ty`、三个架构门脚本、数值门三项全绿**（`EPIC.md:285`）。判据：每条命令的退出码与输出原文。
   - **两次 pytest 调用必须分开**：把 `suspension_kernel/tests` 与 `suspension_contracts/tests` 并进 multibody 目录会改 `rootdir`，使 `tests.benchmark_fixture` 解析失败（`AGENTS.md` 第 1 节；`SUBTASKS.csv` 第 24 行 `notes`）。
   - **`kc_parity_check` 不得用不带 `--actual-dir` 的自比较**（自比较恒过，不构成证据；`EPIC.md:235` 与 `SUBTASKS.csv` 第 24 行 `notes`）。
   - **`case_parity_check.py` 不带参数调用、没有 `--check`**（`EPIC.md:234` 审核阻断项 5）：实测 `packages/suspension_multibody/scripts/case_parity_check.py:1142-1161` 只接受 `--family` / `--allow-partial` / `--record`；写成 `--check` 的命令必然失败。
4. **`kc_baseline` 与 `dynamic_hash_baseline` 逐位未变**（`EPIC.md:310(i)` 与冻结约束 `EPIC.md:232`）。判据：两个冻结文件的逐字节对照（与 p5-01 冻结的起点值比对）。
5. **`simulate` 端到端、总线双向读写、ABS/ESC 实际反馈闭环、FMU 独立校验各有证据**（`EPIC.md:285`）。判据：四类证据各自带命令与产物；其中闭环一类必须含**同一次运行内的「状态 → 控制 → 执行器 → 状态」三段数值记录**（`EPIC.md:308(h)`、`EPIC.md:97` G8）。
6. **独立于子任务自证，不得采信子任务自报**（`SUBTASKS.csv` 第 24 行 `notes` 与 `EPIC.md:289`「端到端独立验收（不依赖子任务自证）」）。判据：本行执行的命令与输出落盘；引用子任务 `raw/` 时须**自己重跑**关键命令。
7. **Done-When (a)–(j) 逐条实跑并逐条记录退出码**（`EPIC.md:291-314`）：**不是只记录测试退出码**——每一条 (a)…(j) 都要有自己的实测命令与退出码（或按条给出「无独立命令时依据的实测产物 + 退出码」），并逐条标注依据的 `EPIC.md` 行号。其中：
   - **(f) 轮端统一**按「**独立复验阶段一 04 的实际交付**」判（`EPIC.md:75`、`EPIC.md:93` G6、`EPIC.md:271`）：实测单轴侧与整车侧消费的是**同一份** wheel 子系统文件（比对**文件路径与内容指纹**，**不看阶段一自报**），且 `grep VerticalTireElement` 在装配路径无命中。
   - **(h) 总线与闭环**按「**ABS 或 ESC 的实际反馈闭环 + 同次运行三段数值记录**」判（`EPIC.md:97` G8、`EPIC.md:308`）：三段（状态 → 控制 → 执行器 → 状态）都有可读数值；**开环回放不算**；ABI 若需第二次变更，必须已按 D2 裁决**新增专属子任务**，不得只登记缺口。

**本行是 Epic 收口**，Done-When (a)-(j) 逐条实跑（`SUBTASKS.csv` 第 24 行 `notes` 与 `EPIC.md:287-314`）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 第 24 行 `notes` 原文：

> 独立于子任务自证 不得采信子任务自报。两次 pytest 调用必须分开（kernel 与 contracts 并进 multibody 目录会改 rootdir 使 tests.benchmark_fixture 解析失败 见 AGENTS.md 第 1 节）。kc_parity_check 不得用不带 --actual-dir 的自比较。本行是 Epic 收口：Done-When (a)-(j) 逐条实跑

展开为：

- `tasks/p5-06-acceptance/raw/**`（本行全部证据）
- `tasks/p5-06-acceptance/{SPEC.md,TODO.csv,PROGRESS.md}`
- 只读核查与验收用脚本、临时产物与会话 scratch（`$PI_SCRATCH_DIR`）

**本行是只读验收轮**：不改生产代码、不改测试、不改契约 schema、不改基线。若发现失败，退回对应子任务修，不在本行就地改代码。

## 禁止触碰

- `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}`——父真值文件，禁止修改。
- `packages/**` 下任何文件：本行只读验收。
- **任何冻结基线**：`tests/data/kc_baseline/`、`tests/data/dynamic_hash_baseline.json`、`tests/data/vehicle_dynamics_baseline/sha256.json`、`tests/data/kc_perf_baseline_native.json`（`EPIC.md:232` 冻结约束）——**禁止重录**。
- **不得改门禁脚本与 allowlist**（`EPIC.md:170` F24）：门变红只能靠改实现或退回子任务，不得加白名单。
- **不得用 `-k` / `--deselect` 豁免失败**（`AGENTS.md` 第 7 节）；**不得新增 skip/xfail**（`EPIC.md:233`）。
- **不得用不带 `--actual-dir` 的 `kc_parity_check.py` 当证据**（`EPIC.md:235`）。
- **不得采信子任务自报结论**（`EPIC.md:289`）：关键判据必须自己重跑。

## 依赖与时机

- `depends_on = p5-05`（`SUBTASKS.csv` 第 24 行）：p5-02 ~ p5-05 全部落地后本行才有可验收的对象。
- 全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md:69`、`EPIC.md:75`）。
- **本行是 Epic 的终局验收**：`EPIC.md:287-314` 的 Done-When（含 `EPIC.md:289` 的「端到端独立验收」与 (a)-(j)）在本行**逐条实跑并逐条记录退出码**。
- 阶段五在阶段四完成后开工（`EPIC.md:215`）；本行之后 Epic 进入结项复核。
- 本行不引入新裁决依赖；D1–D6 的相关结论由前序子任务引用，本行只核对结果。

## 判据与证据落点

逐条对应 `EPIC.md:285`（阶段五验证协议）与 `EPIC.md:287-314`（Done-When (a)-(j)）：

| 父判据 | 做什么 | 看什么 | 证据文件 |
|---|---|---|---|
| G7/G8 逐条实跑 | 按 `EPIC.md:95`（G7）与 `EPIC.md:97`（G8）的判据逐条执行 | 每条的命令、退出码与产物；G8 的闭环一类含同次运行三段数值记录 | `raw/g7_g8.md` |
| 全量回归 | `uv run --no-sync pytest packages/suspension_multibody/tests -q` | 结果原文；skip/xfail 计数与 `AGENTS.md` 第 3 节基线（1297 passed, 1 skipped, 1 xfailed）对照 | `raw/full_regression.txt` |
| `tests/architecture` | `uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q` | 退出码与结果 | `raw/architecture.txt` |
| 两个架构包 | `uv run --no-sync pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q`（**必须与上一条分开调用**） | 退出码与结果；`rootdir` 未漂移 | `raw/packages.txt` |
| `ruff` / `ty` | `uv run --no-sync ruff check .`、`uv run --no-sync ty check .` | 两条退出码 0 | `raw/static_checks.txt` |
| 三个架构门脚本 | `legacy_surface_gate.py --check`、`check_module_layering.py --strict --final`、`check_composable_release.py --skip-isolation` | 三条退出码 0（`AGENTS.md` 第 1 节第 2 步） | `raw/arch_gates.txt` |
| 数值门三项 | `dynamic_hash_sentinel.py --check`、**`case_parity_check.py`（无参数）**、`kc_perf_gate.py --check` | 三条退出码 0；逐位/预算结果原文（`AGENTS.md` 第 2 节与 `EPIC.md:234`） | `raw/numeric_gates.txt` |
| 基线逐位未变 | 与 p5-01 冻结的起点值逐字节对照 `kc_baseline` 与 `dynamic_hash_baseline.json` | 逐字节相同；任何变化按 D5 逐项登记 | `raw/baselines.txt` |
| 四类交付证据 | `simulate` 端到端、总线双向读写、**ABS/ESC 实际反馈闭环（同次运行三段数值记录）**、FMU 独立校验各重跑一次 | 四条命令与产物（**不复用子任务 `raw/` 的结论**） | `raw/deliverable_evidence.md` |
| Done-When (a)–(j) 逐条实跑 | 逐条执行 (a)…(j) 的判据（`EPIC.md:291-314`），每条记录**自己的退出码**与依据行号；(f) 按独立复验阶段一 04 判、(h) 按 ABS/ESC 实际闭环 + 三段数值记录判 | 十条各有命令/产物与退出码，**不是只记录测试退出码** | `raw/done_when_a_to_j.md` |
| 审计口径 | 逐条标注「本人重跑」还是「引用子任务 `raw/`（已复核）」 | 无任何一条仅凭子任务自报 | `raw/independence_audit.md` |

## Constraints（冻结约束）

- **内核提交唯一归属仍只有 `simulation/backend.py`**（`EPIC.md:170` F24 与冻结约束 `EPIC.md:230`）。
- **三个公共 API 门禁全绿**（`EPIC.md` F24）：allowlist `mode = "strict"` 且零条目。
- **ABI 变更需单独裁决**（`EPIC.md:229` 冻结约束）：本 Epic 最多两次 ABI 变更（p2-02 的必要一次 + 仅当 D2 裁决要求内核单步接口时的第二次）。**第二次必须由「先提请裁决并新增的一行专属子任务」承担**（含 `mb_config/version.hpp` 与 `kernel/native.py` 的常量归属、对 `p2-02` 的前置依赖、自身验收），**不得由 p5-04 改版本常量，也不得只登记缺口就放行**。本行核对全部变更是否已登记，并确认 `test_kernel_abi_version_single_source.py` 绿（`EPIC.md:312(j)`）。
- **`model_dump(mode="json")` 产物不得被改变**（`EPIC.md:231` 冻结约束）。
- **基线不得重录**（`EPIC.md:232` 冻结约束；D5 见 `EPIC.md:64`）。
- **不得新增 skip/xfail**；`tests/adams` 的既有环境 skip 不得增长（`EPIC.md:233`）。
- **每条落地后重跑门禁**：`just check-fast`、`tests/architecture`、`just gate-numeric`（`EPIC.md:234`）；本行一次性全跑并记录退出码。数值门三项的 `case_parity_check.py` 以**无参数**调用（`EPIC.md:234`）。
- **`raw/` 只存已执行证据**；临时脚本与中间日志写会话 scratch。

## 风险与回退

- **风险：采信子任务自证**（`EPIC.md:289` 明令端到端验收不依赖子任务自证）。回退：四类交付证据与 G7/G8 判据一律**自己重跑**；引用子任务 `raw/` 时逐条标注并抽检原文。
- **风险：两次 pytest 调用合并导致 `rootdir` 漂移**（`SUBTASKS.csv` 第 24 行 `notes` 与 `AGENTS.md` 第 1 节：kernel/contracts 并进 multibody 目录会使 `tests.benchmark_fixture` 解析失败）。回退：严格分开调用，并在 `raw/packages.txt` 里记录两次调用的 `rootdir`。
- **风险：把「恒过」的命令当证据**（`EPIC.md:235`：`kc_parity_check.py` 不带 `--actual-dir` 是自比较）。回退：需要 K/C 对标时先跑 `kc_native_probe.py` 与 `kc_native_c_probe.py`，再带 `--actual-dir artifacts/kc-native-probe` 判定。
- **风险：数值门命令写错导致「未跑先失败」**（`EPIC.md:234` 审核阻断项 5）：`case_parity_check.py` **只接受 `--family` / `--allow-partial` / `--record`**（`scripts/case_parity_check.py:1142-1161`），**没有 `--check`**。回退：一律用**无参数**调用；若命令本身报「unrecognized arguments」即说明写错，按本条更正后重跑。
- **风险：为让门变绿而重录基线或加白名单**（`EPIC.md:232` 冻结约束与 `EPIC.md:170` F24）。回退：发现失败即**退回对应子任务**修复，本行不改代码、不改基线、不改门禁。
- **风险：全量回归耗时长（约 33 分钟）且中途可能超时**。回退：分块跑并保留原始输出；`adams/`（约 15 分钟）与 `cases/`（约 5 分钟）如需分跑，须说明并保证最终覆盖全部目录（`AGENTS.md` 第 3 节给出基线 1297 passed / 1 skipped / 1 xfailed）。
- **风险：skip/xfail 增长被漏看**。回退：`raw/full_regression.txt` 中显式给出 skip 与 xfail 的计数与名称，与基线逐条对照。

## Done-When

- [ ] G7 与 G8 的每条判据实跑通过（`EPIC.md:95` 与 `EPIC.md:97`），命令与产物落盘；G8 的闭环一类含**同一次运行内的三段数值记录**。
- [ ] 全量 `pytest packages/suspension_multibody/tests -q` 通过，且 **skip 与 xfail 未增长**（对照 `AGENTS.md` 第 3 节基线）。
- [ ] `tests/architecture`、`suspension_contracts/tests` 与 `suspension_kernel/tests`（**分开调用**）、`ruff check .`、`ty check .`、三个架构门脚本、数值门三项全绿；其中 `case_parity_check.py` 以**无参数**调用。
- [ ] `kc_baseline` 与 `dynamic_hash_baseline.json` **逐字节未变**。
- [ ] `simulate` 端到端、总线双向读写、ABS/ESC 实际反馈闭环、FMU 独立校验各有**本人重跑**的证据。
- [ ] Epic Done-When (a)-(j)（`EPIC.md:287-314`）**逐条实跑并逐条记录退出码**（不是只记录测试退出码）；其中 (f) 按独立复验阶段一 04 交付判、(h) 按 ABS/ESC 实际反馈闭环 + 同次运行三段数值记录判；ABI 变更全部已登记且 `test_kernel_abi_version_single_source` 绿。
- [ ] 未重录任何基线、未改门禁、未新增 skip/xfail、未修改 `packages/**` 任何文件。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests -q && uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q && uv run --no-sync pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q && uv run --no-sync ruff check . && uv run --no-sync ty check . && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py && uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
```
