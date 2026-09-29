# SPEC：p5-01 冻结现状事实与判据（阶段五）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-01`

## Goal

拆自 `SUBTASKS.csv` 第 19 行 `acceptance_criteria`，逐条可判定：

1. **公共面清单落盘**：`packages/suspension_multibody/src/suspension_multibody/__init__.py:43-80` 的 `_PUBLIC_NAMES` 与 `__all__` 的 **17 个名字**逐个列出并附原文（含 `run_case`/`run_dynamic_case`/`FrontAxleModel`，锚点见 `EPIC.md` F18、F24）。判据：清单恰好 17 条，且每条都能在 `__init__.py` 原文里逐字对上。
2. **调用者全量盘点**：按 `file:line` 列出 `FrontAxleModel` / `run_case` / `run_dynamic_case` 的全部调用者，生产约 20 处、测试 **48 个文件**（`EPIC.md` F18 的实测值，本行须重新实测并列出对照）。判据：生产与测试分别计数并逐条给出路径与行号，条数与 `grep` 输出一致；数量与 F18 不一致时写明差异来源。
3. **三个公共 API 门禁的规则原文复核**：`tests/architecture/legacy_surface_gate.py`（规则 `legacy_module_import` / `report_native_import` / `report_constitutive_call` / `legacy_forwarding_shell`，两模式 `MODE_MIGRATION` / `MODE_FINAL`，两模式定义在 `:45`）、`scripts/check_composable_release.py`（4 项检查）、`tests/architecture/test_public_api_boundary_gate.py:13`（规则 `direct_kernel_run_contract` / `direct_raw_decoder` / `axle_native_facade_import`，归属判定 `_is_owner` 在 `:74`）的**规则原文**落盘；并确认 allowlist `.codex-tasks/20260919-public-api-simulation-cutover/tasks/01-boundary-inventory/LEGACY_ALLOWLIST.toml` 为 `mode = "strict"` 且**无条目**、`legacy_surface_registry.json` 的 `entry` 为 `[]`（锚点见 `EPIC.md` F24）。判据：给出规则原文摘录 + 两个空清单的原文，**不得**只写「已确认」。
4. **`FrontAxleModel` 形状与哈希口径**：`schema/model.py:157` 的字段形状快照 + `model_dump(mode="json")` 的哈希口径记录（`api.py:116` 与 `:284` 用它算 `model_hash`，锚点见 `EPIC.md` 冻结约束行 227）。判据：实测一次哈希值并写明输入形状与算法口径，作为 p5-02「逐位不变」的对照基准。
5. **六类「不存在」事实的锚点复核**：F19（信号总线/测点：`outputs/builtin.py:154 ASSEMBLY_OUTPUTS`、`:220 RIG_OUTPUTS`、`outputs/declarations.py:1`、`results/channels.py:22 ChannelRegistry`、`adams/axle_channels.yaml:20`）、F20（执行器输入：`authoring/documents.py:948 _check_actuators`、`axle_dynamics/schema.py:982-984`）、F21（闭环控制：`cases/handling.py:8`、`tests/cases/test_handling.py:242`）、F22（FMI 全仓零实现）、F23（内核批式 ABI：`cpp/axle_dynamics/core_abi.hpp:135-136`、`kernel/__init__.py:186 run_contract`、`:174 _read_blocks`、`:14-16` 与 `:32`）。判据：每条给出 `grep` 原文与「零命中 / 存在」的明确结论（`EPIC.md` 验证协议行 271(d)）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 第 19 行 `notes` 原文：

> 写范围仅本行 raw/ 与脚本与会话 scratch。F23：内核是批式 ABI（无 step/state 入口）故实时闭环在当前 ABI 下无法真正实现 这是 D2 的硬约束

展开为：

- `tasks/p5-01-freeze/raw/**`（只存**已执行**的证据）
- `tasks/p5-01-freeze/{SPEC.md,TODO.csv,PROGRESS.md}`
- 只读核查用的临时脚本与会话 scratch（`$PI_SCRATCH_DIR`）

本行**不改任何生产代码与测试**（`EPIC.md` 行 355 的规划轮口径「`raw/` 只存**已执行**的证据，不存虚构结果」：p5-01 是只读冻结轮）。

## 禁止触碰

- `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}`——父真值文件，禁止修改。
- `packages/**` 下任何文件：本行是**只读冻结**，不写生产代码、不改测试、不改契约 schema。
- 任何冻结基线：`tests/data/kc_baseline/`、`tests/data/dynamic_hash_baseline.json`、`tests/data/vehicle_dynamics_baseline/sha256.json`、`tests/data/kc_perf_baseline_native.json`（`EPIC.md` 冻结约束行 228）。
- **禁止依据旧任务的 `DONE` 结论**（`EPIC.md` 行 237 对 p2-01 的同口径要求）：全部事实必须本行重新实测。
- 不得新增 skip/xfail；不得运行任何会改变仓库状态（写产物到仓库内）的命令。

## 依赖与时机

- 父行 `depends_on` **为空**（`SUBTASKS.csv` 的 `p5-01`）：本行是**纯只读冻结行**（写范围只有本行 `raw/` 与会话 scratch，冻结的是**阶段五的现状**——`api.py` 公共面、`FrontAxleModel` 形状与哈希口径、三个公共 API 门禁、六类「不存在」事实，**不读前一阶段任何会被本 Epic 改写的产物**），故**本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。
- **阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**（p5-02 ~ p5-06 与终局验收行），不影响本行长；本行**不写生产代码**（`SUBTASKS.csv` 的 `p5-01` `notes`：写范围仅本行 `raw/` 与脚本与会话 scratch）。
- 跨阶段顺序：阶段五在阶段四完成后开工（`EPIC.md` 行 219）——**该顺序只对实施行成立**；本行可提前开工，但只做只读核查，不得提前触及任何其它行的写范围。
- 无 D 编号裁决是本行的开工条件；但本行必须把 **D2 的硬约束（批式 ABI 无 step/state 入口，`EPIC.md` 行 61 + F23）** 与 **D4/D6（FMI 版本与新依赖，`EPIC.md` 行 63/65）** 的现状事实固化下来，供 p5-04/p5-05 开工时直接引用。
- 本行是 p5-02 ~ p5-06 的**唯一现状真源**：后续行的「不得改」硬门（`model_hash`、公共门禁、`FrontAxleModel` 调用者）以本行落盘的对照基准为准。

## 判据与证据落点

逐条对应 `EPIC.md` 行 271 的 (a)(b)(c)(d)：

| 父判据 | 做什么 | 看什么 | 证据文件 |
|---|---|---|---|
| (a) 公共面清单与调用者盘点 | `grep -rn` 三个公开名 + 读 `__init__.py:43-80` 原文 | 17 个名字逐条对上；生产/测试调用者计数与逐条 `file:line` | `raw/public_surface.md` |
| (b) 三公共 API 门禁规则原文复核 | 读三个门禁的规则定义与 allowlist / registry 原文 | 规则原文摘录；`mode = "strict"` 且无条目；`entry` 为 `[]` | `raw/api_gates.txt` |
| (c) `FrontAxleModel` 字段形状与哈希口径 | 读 `schema/model.py:157` 字段表；实跑一次 `model_dump(mode="json")` 并算哈希 | 字段形状全文 + 哈希实测值 + 口径说明 | `raw/public_surface.md`（哈希段） |
| (d) F19/F20/F21/F22/F23 六类不存在事实 | 按 Goal 5 逐条 `grep` | 每条 `grep` 原文 + 「零命中 / 存在」结论 | `raw/absent_features.md` |
| 起点门禁实测 | 跑本行 `validation_command` | 退出码与输出原文（起点值，供 p5-06 对照） | `raw/baseline_run.md` |

`raw/` 硬性非空要求（`validation_command` 的两条 `test -s`）：`raw/public_surface.md`、`raw/absent_features.md`。

## Constraints（冻结约束）

- **内核提交唯一归属仍只有 `simulation/backend.py`**（`EPIC.md` 行 100 与 F24 的 `direct_kernel_run_contract` 规则，`_is_owner` 在 `test_public_api_boundary_gate.py:74`）：本行的调用者盘点必须把「谁直接调 `run_contract`」单列一类。
- **三个公共 API 门禁必须全绿，且本行不得为了让它变绿而改门禁脚本或 allowlist**：allowlist 为 `mode = "strict"` 且**零条目**（`EPIC.md` F24）。
- **ABI 变更需单独裁决**：`EPIC.md` 冻结约束行 225——本 Epic 最多两次 ABI 变更，且只有 p2-02 与（视 D2 裁决的）p5-04 可以改 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量。本行只记录现状（`EPIC.md` F3 的 16/31/1 与 `kernel/native.py:33-35` 单一真源），不得改动。
- **分层方向不可逆**（`EPIC.md` 行 226）：`report/` 不得 import/调用 native/kernel/solver，也不得自求力律（`legacy_surface_gate.py` 的 `report_native_import`/`report_constitutive_call`）。
- **基线不得重录**（`EPIC.md` 行 228）：本行只记录现状哈希，不得重录任何基线。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）。
- **`kc_parity_check.py` 不带 `--actual-dir` 时是自比较（恒过），不构成证据**（`EPIC.md` 行 231）——本行若引用 K/C 对标必须按该条口径。
- **`raw/` 只存已执行证据**：不预填、不写虚构结果；临时脚本与中间日志写会话 scratch。

## 风险与回退

- **风险：把 F 编号的实测值当既有结论照抄**（`EPIC.md` 行 320「本 Epic 起点须先实测」）。回退：全部锚点本行重新 grep/读原文；与 F 编号不一致时以本行实测为准并写明差异，**不得**回头改 F 编号文字（父文件禁止触碰）。
- **风险：48 个测试文件与约 20 处生产调用者会随 p5-02 变化**（`EPIC.md` F18）。回退：本行落盘的计数是**改造前基线**，p5-02 只需证明「历史调用者全部可用」，不要求计数不变。
- **风险：反转既有契约（`cases/handling.py:8` 只表达开环）由 p5-04 承担**（`EPIC.md` 行 318）。本行只冻结拒绝原文与测试锚点，**不得**在本行尝试反转。
- **风险：FMI 是新依赖**（`EPIC.md` 行 319 与 D6）。本行只固化 F22 的「全仓零实现」事实，依赖裁决留给 p5-05。
- **本行本身只读，不产生物理数值变化**：`just gate-numeric` 不是本行判据；本行需要记录的是**起点值**（`EPIC.md` 行 320）。

## Done-When

- [ ] `raw/public_surface.md` 含 17 个公开名逐个原文、生产/测试调用者逐条 `file:line` 台账、`FrontAxleModel` 字段形状与 `model_dump(mode="json")` 哈希实测值。
- [ ] `raw/absent_features.md` 含 F19/F20/F21/F22/F23 六类事实的 `grep` 原文与「零命中 / 存在」结论，逐条带 `file:line`。
- [ ] 三个公共 API 门禁的规则原文与「allowlist `mode = "strict"` 且无条目 / `legacy_surface_registry.json` 的 `entry` 为 `[]`」原文已落盘（`raw/api_gates.txt`）。
- [ ] `raw/baseline_run.md` 记录了本行 `validation_command` 的退出码与输出原文（起点值）。
- [ ] 未修改任何生产代码、测试、契约 schema 与冻结基线；未在 `packages/**` 下产生任何 diff。
- [ ] 未新增 skip/xfail；未将其他任何子任务置为 `DONE` 或 `IN_PROGRESS`。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/architecture -q && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-01-freeze/raw/public_surface.md && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-01-freeze/raw/absent_features.md
```
