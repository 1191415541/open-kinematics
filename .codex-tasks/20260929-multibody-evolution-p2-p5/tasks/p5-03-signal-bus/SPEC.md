# SPEC：p5-03 信号总线（测点与执行器）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-03`

## Goal

拆自 `SUBTASKS.csv` 第 21 行 `acceptance_criteria`，逐条可判定：

1. **总线能读测点**：轮速、车身加速度**至少各一个**（`EPIC.md` G8 行 93 与行 275(a)）。判据：两个测点各有读取用例与断言值。
2. **总线能写执行器输入**：可变阻尼、电机力矩**至少各一个**（`EPIC.md` G8 行 93 与行 275(a)；`EPIC.md` F20 实测今天无可变阻尼，`grep -i "damper_ratio|variable_damp"` 零命中）。判据：两个执行器输入各有写入用例与断言。
3. **测点值可与结果文档中的同一物理量逐项对照，不是新算一遍**（`EPIC.md` 行 275(b)）。判据：对照记录给出「总线读数」与「结果文档同名字段」的两侧原文，逐项相等；若需换算，换算关系写明。
4. **既有 `outputs/` 静态声明与总线的关系有定义（谁是真源）**，未使用的声明要么接入要么登记（`EPIC.md` 行 275(c)）。今天的现状是两套互不相通：`outputs/builtin.py:154 ASSEMBLY_OUTPUTS`、`:220 RIG_OUTPUTS`、`outputs/declarations.py:1`（注释「What a run declares it produces, before anything computes it」）**未被 api.py 或 rigs 生产路径消费**（仅测试引用）；另一套是 `results/channels.py:22 ChannelRegistry`（只读注册表，读 `adams/axle_channels.yaml:20`，是冻结的 Adams 输出通道表，`EPIC.md` F19）。判据：写明真源归属，并列出每一条未使用声明的处置（接入 / 登记）。
5. **双向读写各有断言**（`EPIC.md` 行 275(d)）。判据：至少 2 条读 + 2 条写断言，且不是同一条测试里互相抵消的写法。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 第 21 行 `notes` 原文：

> 写范围新增信号总线模块与 outputs/builtin.py 的派生输出声明（与 p3-05 串行 同一文件）与 api.py 的暴露段。F19：今天只有 outputs 静态声明与 Adams ChannelRegistry（冻结通道表）两者互不相通

展开为：

- **新增信号总线模块**（本行自定落点与命名；须符合 `EPIC.md` 冻结约束行 226 的分层方向）
- `packages/suspension_multibody/src/suspension_multibody/outputs/builtin.py`（**派生输出声明段**）
- `packages/suspension_multibody/src/suspension_multibody/api.py`（**暴露段**，承接 p5-02 定稳的公开出口结构）
- 对应测试：`packages/suspension_multibody/tests/api/`、`tests/outputs/`、`tests/results/`
- `tasks/p5-03-signal-bus/raw/**` 与临时脚本与会话 scratch

### 共享写范围警示（必须串行）

- **本行与 p3-05（阶段三·动态通道注册）共享 `outputs/builtin.py`，故必须串行**：`EPIC.md` 行 221 明确「`outputs/builtin.py` 的派生输出声明归 p3-05；p5-03 若需新增测点声明，必须与 p3-05 串行（同一文件）」。本行不得与 p3-05 并行改该文件；p3-05 在前（`p5-01` 依赖 `p4-05`、`p4-01` 依赖 `p3-06`，阶段五整体在阶段三之后，`EPIC.md` 行 211）。
- `EPIC.md` 行 213~221 的通用约束同样适用：`templates/roles.py`、`templates/builtin.py`、`subsystems/element_build.py`、`preparation/vehicle_dynamic.py`、三份契约 schema 均为**共享写范围，不得并行**。

## 禁止触碰

- `simulation/backend.py` 之外不得直接调 `run_contract`：内核提交唯一归属仍只有 `simulation/backend.py`（`EPIC.md` 行 100 与冻结约束行 226；`_is_owner` 在 `test_public_api_boundary_gate.py:74`）。
- **不得改 `FrontAxleModel` 字段形状与 `model_dump(mode="json")` 的产物**（`EPIC.md` 冻结约束行 227）：`api.py:116` 与 `:284` 用它算 `model_hash`。
- **不得删除既有公开入口**（绞杀者模式，`EPIC.md` Non-Goals 行 100）：`run_case` / `run_dynamic_case` / `FrontAxleModel` 保持可用。
- **不改 ABI**：不得触碰 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量（`EPIC.md` 冻结约束行 225；本 Epic 只有 p2-02 与视 D2 裁决的 p5-04 可以）。
- **不得改 `results/channels.py:22 ChannelRegistry` 与 `adams/axle_channels.yaml:20`**：那是**冻结的 Adams 输出通道表**，不是运行时信号总线（`EPIC.md` F19）。本行只定义它与总线的关系，不改它。
- **不得改门禁脚本与 allowlist**（`EPIC.md` F24）。
- **分层方向不可逆**（`EPIC.md` 行 226）：`report/` 不得 import/调用 native/kernel/solver，也不得自求力律；新增总线模块的依赖方向必须落在这条链的正确一侧，并经 `test_import_boundaries.py` 检查。
- **不得新增 skip/xfail**；**不得重录任何基线**（`EPIC.md` 冻结约束行 228/229）。
- `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}` 禁止修改。

## 依赖与时机

- `depends_on = p5-02`（`SUBTASKS.csv` 第 21 行）：本行要在 `api.py` 的**暴露段**接线，故 p5-02 必须先定稳公开出口结构。
- **与 p3-05 串行**（同一文件 `outputs/builtin.py`，`EPIC.md` 行 221）。
- 全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）；阶段五在阶段四完成后开工（`EPIC.md` 行 211）。
- **D2 的裁决影响本行的验收深度**：`EPIC.md` 行 61（D2）——内核是批式 ABI（F23），真正的实时闭环需要内核单步接口。建议是本行**只做 Python 侧总线与开环**，并把「实测是否需要内核单步接口」的结论交给 p5-04 登记。
- 无其它 D 编号阻挡本行。

## 判据与证据落点

逐条对应 `EPIC.md` 行 275 的 (a)(b)(c)(d)：

| 父判据 | 做什么 | 看什么 | 证据文件 |
|---|---|---|---|
| (a) 读测点 + 写执行器 | 新增总线模块；轮速、车身加速度各 ≥1 个读；可变阻尼、电机力矩各 ≥1 个写 | 4 条断言原文（2 读 + 2 写）与实测值 | `raw/bus_io.md` |
| (b) 测点与结果文档逐项对照 | 同一物理量取「总线读数」与「结果文档字段」两侧 | 两侧原文逐项相等；不存在「总线自己重算一遍」的实现路径（`grep` 证明总线只读既有结果文档） | `raw/point_parity.md` |
| (c) 与 `outputs/` 静态声明的关系 | 读 `outputs/builtin.py:154`/`:220`、`outputs/declarations.py:1`、`results/channels.py:22` | 真源归属写明；未使用声明逐条处置（接入 / 登记）；`ChannelRegistry` 与 `axle_channels.yaml:20` 未被改 | `raw/outputs_relationship.md` |
| (d) 双向读写断言 | 跑 `tests/api`、`tests/outputs`、`tests/results` | 退出码 0 + 断言原文 | `raw/bus_io.md`（断言段） |
| 结构门 | 跑 `legacy_surface_gate.py --check` | 退出码 0；`report_native_import`/`report_constitutive_call` 未受影响 | `raw/gates.txt` |

父行 `validation_command` 不含 `test -s`，但四个证据文件仍须落盘（只记已执行结果）。

## Constraints（冻结约束）

- **内核提交唯一归属仍只有 `simulation/backend.py`**（`EPIC.md` 行 100 与冻结约束行 226）。
- **三个公共 API 门禁全绿**：`legacy_surface_gate.py --check`、`check_composable_release.py --skip-isolation`、`test_public_api_boundary_gate.py`（`EPIC.md` F24）。allowlist 为 `mode = "strict"` 且零条目，**不得加白名单**。
- **ABI 变更需单独裁决**（`EPIC.md` 冻结约束行 225）：本行不得改 ABI；若实测发现总线需要内核侧的按步状态读取，**登记为内核范围并提请单独裁决**，不得静默降级为「登记即收口」（`EPIC.md` 行 61 的 D2 口径）。
- **`model_dump(mode="json")` 产物不得被改变**（`EPIC.md` 冻结约束行 227）。
- **基线不得重录**（`EPIC.md` 冻结约束行 228；D5）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变。**新增测点会改变输出集合**——若确需变化，按 D5 逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」（`EPIC.md` 行 64 已点名阶段五新增测点这一情形）。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）。
- **每步落地后重跑 `just check-fast`**；改结构后加跑 `tests/architecture`（`EPIC.md` 行 230）。
- **`raw/` 只存已执行证据**；临时脚本与中间日志写会话 scratch。

## 风险与回退

- **风险：与 p3-05 并行改 `outputs/builtin.py`，后者覆盖前者**（`EPIC.md` 行 221 明列同一文件）。回退：**先确认 p3-05 已落地并记录其改动**再动该文件；若必须同时进行，本行只做新增、不做重排，并把冲突点登记。
- **风险：总线自己重算一遍测点值**（违反 `EPIC.md` 行 275(b)）。回退：测点只能从**既有结果文档**读取；对照证据必须是「两侧原文 + 相等」而不是「两侧都由总线算」。
- **风险：把 `ChannelRegistry` 当运行时总线用**（`EPIC.md` F19 明确它是冻结的 Adams 输出通道表）。回退：只读引用，不改、不扩展。
- **风险：新增测点改变输出集合，扰动 `dynamic_hash_baseline.json`**（`EPIC.md` 行 64 与行 279(c) 的 D5 口径）。回退：先跑 `dynamic_hash_sentinel.py --check` 取改动前后对照；变化必须按 D5 逐项登记并给独立于结果字节的物理等价判据，**禁止重录**。
- **风险：反了分层方向**（新增总线模块被 `report/` 或低层反向导入）。回退：先跑 `test_import_boundaries.py`；`EPIC.md` 行 315 已提醒该测试在子进程逐入口检查，必须先跑通再落地。
- **风险：为闭环预留实现而静默把内核单步需求吞掉**（`EPIC.md` 行 61 的 D2 明令禁止）。回退：本行只交付总线与开环可验证部分，把内核范围结论显式写进 `raw/` 并交 p5-04 提请裁决。

## Done-When

- [ ] 总线能读**至少一个轮速 + 一个车身加速度**测点，能写**至少一个可变阻尼 + 一个电机力矩**执行器输入，四个方向各有断言。
- [ ] 测点值与结果文档中的同一物理量**逐项对照相等**，且总线不重算（有实现层面的证据）。
- [ ] `outputs/` 静态声明与总线的真源关系已写明；未使用的声明逐条「接入或登记」（`raw/outputs_relationship.md`）。
- [ ] `results/channels.py:22 ChannelRegistry` 与 `adams/axle_channels.yaml:20` 未被改动。
- [ ] `legacy_surface_gate.py --check` 退出码 0；三个公共 API 门禁全绿；内核提交唯一归属仍只有 `simulation/backend.py`。
- [ ] `model_dump(mode="json")` 形状与 `model_hash` 未变；未重录任何基线；未新增 skip/xfail。
- [ ] 若实测证明总线需要内核按步状态读取，已**登记为内核范围并提请单独裁决**（不是「登记即收口」）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/outputs packages/suspension_multibody/tests/results -q && uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
```
