# SPEC：p5-05 FMI 联合仿真导出

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-05`

## Goal

拆自 `SUBTASKS.csv` 第 23 行 `acceptance_criteria`，逐条可判定：

1. **按 D4 的版本与范围导出 FMU，产物存在且可被独立校验**（`EPIC.md` 行 279(a)）：
   - **变量清单与输入/输出方向正确**：逐条列出变量名与 `causality`（input/output）并判定正确；
   - **能在仓库外被一个独立脚本加载并步进**：脚本不 import 本仓库任何模块，在仓库外的目录运行并完成步进。
   D4 建议（`EPIC.md` 行 63）：**FMU 2.0 Co-Simulation**，只导出「模型 + 输入/输出变量」，**不含 Python 侧求值**；不做实时/硬件在环承诺。判据：D4 裁决原文 + 变量清单（含方向）+ 独立脚本的路径、命令与输出原文。
2. **新依赖先提出再确认（D6）**（`EPIC.md` 行 279(b) 与 D6 行 65）：FMI 库属**新依赖**，**须先确认**。判据：依赖提出与裁决记录的原文落盘；**未确认前不得引入**。
3. **导出不影响既有运行路径**（`EPIC.md` 行 279(c)）。判据：既有 `tests/api` 全通过；`dynamic_hash_sentinel.py --check` 未受影响（若本行触及求解路径）；`kc_baseline` 与 `dynamic_hash_baseline` 逐字节未变。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 第 23 行 `notes` 原文：

> D4 建议 FMU 2.0 Co-Simulation 只导出模型与输入输出变量 不含 Python 侧求值 不做实时或硬件在环承诺。D6：FMI 库属新依赖须先确认 若不允许则降级为接口契约加独立验证脚本并登记为未闭合项。F22：全仓零 FMI 实现

展开为：

- **新增 FMI 导出模块或脚本**（本行自定落点与命名；须符合 `EPIC.md` 冻结约束行 226 的分层方向）
- **新增 FMI 导出用例**：`packages/suspension_multibody/tests/api/`（父行 `validation_command` 只跑该目录）
- **仓库外的独立验证脚本**：放在会话 scratch 或 `raw/` 同级的**独立位置**（**不得**放在 `packages/**` 或任何会被仓库测试收集的路径，否则就不是「仓库外独立脚本」）；脚本内容须落盘进 `raw/`
- `tasks/p5-05-fmi/raw/**` 与临时脚本与会话 scratch

## 禁止触碰

- **不得在任何路径下引入未获确认的新依赖**（D6，`EPIC.md` 行 65）：FMI 库须**先提出再确认**。若不允许，按 `EPIC.md` 行 319 降级为「导出为可联合仿真的接口契约 + 独立验证脚本」，并**登记为未闭合项**。
- **不得删除既有公开入口**（绞杀者模式，`EPIC.md` Non-Goals 行 100）：`run_case` / `run_dynamic_case` / `FrontAxleModel` 保持可用。
- **不得改 `FrontAxleModel` 字段形状与 `model_dump(mode="json")` 的产物**（`EPIC.md` 冻结约束行 227）。
- `simulation/backend.py` 之外不得直接调 `run_contract`（`EPIC.md` 行 100 与冻结约束行 226）。
- **不改 ABI**：不得触碰 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量（`EPIC.md` 冻结约束行 225）。
- **不做实时/硬件在环承诺**（D4，`EPIC.md` 行 63 与 Non-Goals 行 101）：FMU 只做**离线**联合仿真导出。
- **不得为了让 FMU 能跑而改既有运行路径**：导出必须是非侵入的旁路（`EPIC.md` 行 279(c)）。
- **不得改门禁脚本与 allowlist**（`EPIC.md` F24）；**不得改 `results/channels.py:22 ChannelRegistry` 与 `adams/axle_channels.yaml:20`**（冻结的 Adams 输出通道表，`EPIC.md` F19）。
- **不得重录任何基线**（`EPIC.md` 冻结约束行 228；D5）；**不得新增 skip/xfail**（`EPIC.md` 行 229）。
- `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}` 禁止修改。
- **不在 `packages/**` 下放独立验证脚本**：那样它会被仓库测试收集，失去「仓库外独立」的含义。

## 依赖与时机

- `depends_on = p5-04`（`SUBTASKS.csv` 第 23 行）。
- 全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）；阶段五在阶段四完成后开工（`EPIC.md` 行 211）。
- **D4 与 D6 必须先裁决**（`EPIC.md` 行 63 与行 65）：D4 决定 FMU 版本与范围（建议 FMU 2.0 Co-Simulation），D6 决定是否允许引入 FMI 库。未裁决前本行不得置 `IN_PROGRESS`。
- 现状：全仓**零** FMI/FMU/co-simulation 实现（`EPIC.md` F22，`grep -iE "fmi|fmu|cosim|co-simulation|co_simulation"` 只命中路线图文档）——本行是从零新增。
- 上游：本行导出的「输入/输出变量」来自 p5-03 的信号总线（测点与执行器输入）与 p5-02 的 `simulate` 入口。

## 判据与证据落点

逐条对应 `EPIC.md` 行 279 的 (a)(b)(c)：

| 父判据 | 做什么 | 看什么 | 证据文件 |
|---|---|---|---|
| (a) 按 D4 导出 FMU，产物存在且可被独立校验 | 导出 FMU；列变量清单与 `causality`；写一个**仓库外**的独立脚本加载并步进 | D4 裁决原文；变量清单（名称 + 方向）逐条判定；独立脚本路径、命令与输出原文；脚本不 import 本仓库模块 | `raw/fmu_validation.md`（`validation_command` 要求非空） |
| (b) 新依赖先提出再确认（D6） | 记录依赖提出与裁决；记录是否实际引入 | 提出与裁决原文；若不允许：降级方案为「接口契约 + 独立验证脚本」且已登记为**未闭合项** | `raw/fmu_validation.md`（D6 段）+ `raw/dependency_decision.md` |
| (c) 导出不影响既有运行路径 | 跑 `tests/api`；必要时跑 `dynamic_hash_sentinel.py --check` | `tests/api` 退出码 0；`--check` 逐字节未变；`kc_baseline`/`dynamic_hash_baseline` 逐字节未变 | `raw/no_regression.md` |

`raw/` 硬性非空要求（`validation_command` 的 `test -s`）：`raw/fmu_validation.md`。

## Constraints（冻结约束）

- **内核提交唯一归属仍只有 `simulation/backend.py`**（`EPIC.md` 行 100 与冻结约束行 226）。
- **三个公共 API 门禁全绿**（`EPIC.md` F24）：allowlist `mode = "strict"` 且零条目，不得加白名单。
- **ABI 变更需单独裁决**（`EPIC.md` 冻结约束行 225）：本行不改 ABI。
- **`model_dump(mode="json")` 产物不得被改变**（`EPIC.md` 冻结约束行 227）。
- **基线不得重录**（`EPIC.md` 冻结约束行 228；D5）。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）。
- **每步落地后重跑 `just check-fast`**；改结构后加跑 `tests/architecture`（`EPIC.md` 行 230）。
- **`raw/` 只存已执行证据**；临时脚本与中间日志写会话 scratch。

## 风险与回退

- **新依赖风险（`EPIC.md` 行 319 原文）**：「阶段五 FMI 是新依赖：D6 必须先确认；**若不允许，p5-05 降级为「导出为可联合仿真的接口契约 + 独立验证脚本」，并登记为未闭合项**。」本行必须按此执行：D6 未确认前不引入任何依赖；若不获允许，走降级路径并在 `PROGRESS.md` 与 `raw/` 中显式登记为**未闭合项**——不得把降级交付当作完整交付上报。
- **风险：D4 范围被悄悄放大到实时/硬件在环**（Non-Goals 行 101 与 `EPIC.md` 行 63 的 D4 建议都明令不做）。回退：FMU 只导出「模型 + 输入/输出变量」，不含 Python 侧求值；若发现需要实时保证，停下并提请裁决。
- **风险：独立验证脚本放进 `packages/**` 或 `tests/`，被仓库测试收集后就不再是「仓库外独立」**。回退：脚本放在会话 scratch 或仓库外目录，内容落盘进 `raw/fmu_validation.md`；命令与输出一并记录。
- **风险：为让 FMU 可加载而改既有运行路径或输出形状**（违反 `EPIC.md` 行 279(c)）。回退：导出走旁路；改动前后跑 `tests/api` 与 `dynamic_hash_sentinel.py --check` 对照。
- **风险：把「变量清单与方向正确」用弱证据糊过去**（只写「文件存在」）。回退：逐条列出变量名与 `causality`，并说明每条方向为何正确（输入由外部给定、输出由模型产生）。

## Done-When

- [ ] 按 D4 的版本与范围导出 FMU；产物存在；**变量清单（名称 + 输入/输出方向）逐条列出并判定正确**。
- [ ] 一个**仓库外**的独立脚本能加载该 FMU 并完成步进；脚本不 import 本仓库模块；脚本内容、运行命令与输出原文落盘。
- [ ] D6 的依赖提出与裁决已记录；若不允许，已按降级路径交付「接口契约 + 独立验证脚本」并**登记为未闭合项**。
- [ ] 导出不影响既有运行路径：`tests/api` 通过；`kc_baseline` 与 `dynamic_hash_baseline` 逐字节未变。
- [ ] 未改 ABI；内核提交唯一归属仍只有 `simulation/backend.py`；未重录任何基线；未新增 skip/xfail。
- [ ] 未做任何实时/硬件在环承诺（D4 范围）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/api -q && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-05-fmi/raw/fmu_validation.md
```
