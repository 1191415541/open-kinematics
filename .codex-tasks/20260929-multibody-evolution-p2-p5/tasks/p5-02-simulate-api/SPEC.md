# SPEC：p5-02 simulate 公共入口与 FrontAxleModel 降级

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-02`

## Goal

拆自 `SUBTASKS.csv` 第 20 行 `acceptance_criteria`，逐条可判定：

1. **`simulate(assembly_document, case_document)` 成为公共入口**：有明确签名、有文档、在 `__init__.py:43-80` 的 `_PUBLIC_NAMES` / `__all__` 中登记（`EPIC.md` F24 与 F18：今天 `def simulate` 全仓零命中）。判据：`from suspension_multibody import simulate` 可用；`simulate` 在 `__all__` 中；**端到端用例一条**（一份总成文档 + 一份工况文档跑通并拿到结果）。
2. **`FrontAxleModel` 不删除、`run_case` / `run_dynamic_case` 保持可用**：降级为**向下兼容适配器**，内部改走 `simulate`（`EPIC.md` G7 行 91 与 Non-Goals 行 100）。判据：p5-01 盘点的**全部历史调用者**（生产约 20 处 + 48 个测试文件，`EPIC.md` F18）仍可用；`FrontAxleModel` 仍在 `__init__.py` 的 `__all__` 中。
3. **`model_dump(mode="json")` 形状与 `model_hash` 逐位不变**：`api.py:116` 与 `:284` 用它算 `model_hash`（`EPIC.md` 冻结约束行 227、F18）。判据：与 p5-01 落盘的形状与哈希实测值**逐位相同**。
4. **三个公共 API 门禁全绿**：`legacy_surface_gate.py --check`、`check_composable_release.py --skip-isolation`（含 `composable_extension_examples.md` 的 `runnable` 代码块，`EPIC.md` F24）、`test_public_api_boundary_gate.py`。判据：三条命令退出码 0，且**不改门禁脚本、不改 allowlist**（allowlist 今天 `mode = "strict"` 且零条目）。
5. **内核提交唯一归属仍只有 `simulation/backend.py`**：`direct_kernel_run_contract` 规则的归属判定 `_is_owner` 在 `test_public_api_boundary_gate.py:74`（`EPIC.md` F24）。判据：`test_public_api_boundary_gate.py` 绿，且 `grep` 显示只有该文件直接 `run_contract`。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 第 20 行 `notes` 原文：

> 绞杀者模式：严禁删除 FrontAxleModel 与 VehicleModel 与既有公开入口。model_dump 形状不得改（api.py:116 与 :284 算 model_hash）。写范围 api.py 与 __init__.py 与 __all__ 与 examples 文档

展开为：

- `packages/suspension_multibody/src/suspension_multibody/api.py`
- `packages/suspension_multibody/src/suspension_multibody/__init__.py`（含 `_PUBLIC_NAMES` 与 `__all__`，`:43-80`）
- `examples` / 文档侧的示例与说明段（含 `composable_extension_examples.md` 的 `runnable` 代码块若需同步）
- `simulate` 新增实现所需的落点：文档驱动入口已存在于 `simulation/runner.py:78 run_request` 与 `kernel/__init__.py:186 run_contract`（`EPIC.md` F18）——**`simulate` 只是阶段一「文档驱动装配器」的对外出口**（`EPIC.md` 行 73），**不得**另起第二条装配路径
- 对应测试：`packages/suspension_multibody/tests/api/`
- `tasks/p5-02-simulate-api/raw/**` 与临时脚本与会话 scratch

## 禁止触碰

- **严禁删除 `FrontAxleModel` / `VehicleModel` / 既有公开入口**（绞杀者模式，`EPIC.md` Non-Goals 行 100 与冻结约束行 226）：`FrontAxleModel` 定义在 `schema/model.py:157`，仍被 `schema/vehicle.py:252-253` 使用，仍在 `__init__.py:46` 的 `__all__`（17 个名字之一）。
- **`model_dump(mode="json")` 的形状不得改**：`api.py:116` 与 `:284` 用它算 `model_hash`（`EPIC.md` 冻结约束行 227）。`FrontAxleModel` 的**字段形状**不得改。
- **内核提交唯一归属**：除 `simulation/backend.py` 之外任何模块不得直接调 `run_contract`（`EPIC.md` 行 100 与 F24）。
- **不改 ABI**：本行不得动 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量；本 Epic 只有 p2-02 与视 D2 裁决的 p5-04 可以（`EPIC.md` 冻结约束行 225）。
- **不改门禁脚本与 allowlist**：`tests/architecture/legacy_surface_gate.py`、`scripts/check_composable_release.py`、`tests/architecture/test_public_api_boundary_gate.py`、`.codex-tasks/20260919-public-api-simulation-cutover/.../LEGACY_ALLOWLIST.toml`、`legacy_surface_registry.json` 均只读（`EPIC.md` F24）。
- **不重录任何基线**：`tests/data/kc_baseline/`、`tests/data/dynamic_hash_baseline.json`（`EPIC.md` 冻结约束行 228）。
- **不得把「简化/专用」分支写进 role 接口**（`EPIC.md` 行 232）。
- **不新增 skip/xfail**（`EPIC.md` 行 229）。
- `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}` 禁止修改。

## 依赖与时机

- `depends_on = p5-01`（`SUBTASKS.csv` 第 20 行）：本行需要 p5-01 冻结的 17 个公开名清单、全部调用者台账、字段形状与 `model_hash` 基线、三个门禁规则原文。
- 全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）。
- 阶段五在阶段四完成后开工（`EPIC.md` 行 211）。
- **依赖阶段一 03 的文档驱动装配器**（`EPIC.md` 行 73）：`simulate(assembly_document, case_document)` 只是该装配器的对外出口，**不再自造 document**。若阶段一 03 的文档驱动入口与本行预期形态不一致，按实测调整并登记。
- 与 p5-03 的串行关系：p5-03 要在 `api.py` 的**暴露段**新增信号总线出口（`SUBTASKS.csv` 第 21 行 `notes`），故 p5-03 在本行之后；本行应把 `api.py` 的公开出口结构定稳，避免 p5-03 再动同一段的形状。
- 无 D 编号裁决阻挡本行；但 `FrontAxleModel` 降级深度受 `EPIC.md` Non-Goals 行 100 与 G7 行 91 约束。

## 判据与证据落点

逐条对应 `EPIC.md` 行 273 的 (a)(b)(c)(d)(e)：

| 父判据 | 做什么 | 看什么 | 证据文件 |
|---|---|---|---|
| (a) `simulate` 成为公共入口 | 新增 `simulate(assembly_document, case_document)`，登记进 `_PUBLIC_NAMES`/`__all__`，写端到端用例 | 签名/文档原文；导入成功；端到端用例通过及产物摘要 | `raw/simulate_entrypoint.md` |
| (b) `FrontAxleModel` 不删除、两入口可用 | `run_case`/`run_dynamic_case` 改为内部走 `simulate` | `FrontAxleModel` 仍在 `__all__`；p5-01 台账里的调用者逐个仍可用（测试集全绿作为回归网） | `raw/legacy_adapter.md` |
| (c) `model_dump` 形状与 `model_hash` 逐位不变 | 与 p5-01 的形状快照与哈希实测值对照 | 逐位相同（差异即失败） | `raw/legacy_adapter.md`（哈希段） |
| (d) 三个公共 API 门禁全绿 | 跑三条门禁命令 | 三条退出码 0 + 输出原文；`composable_extension_examples.md` 的 runnable 代码块被执行且通过 | `raw/gates.txt` |
| (e) 内核提交唯一归属 | 跑 `test_public_api_boundary_gate.py`；`grep` 直接 `run_contract` 的模块 | 只有 `simulation/backend.py` 命中；门绿 | `raw/gates.txt` |

`raw/` 证据文件：`simulate_entrypoint.md`、`legacy_adapter.md`（含哈希逐位对照）、`gates.txt`。父行 `validation_command` 不含 `test -s` 检查，但证据仍须落盘（`EPIC.md` 行 355 的「`raw/` 只存已执行的证据」总口径）。

## Constraints（冻结约束）

- **内核提交唯一归属仍只有 `simulation/backend.py`**（`EPIC.md` 行 100 与冻结约束行 226）。
- **三个公共 API 门禁全绿且不得放宽**：allowlist `mode = "strict"` 且零条目（`EPIC.md` F24）——门禁变红只能靠改实现，不能靠加白名单。
- **ABI 变更需单独裁决**：本行不得触碰 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量（`EPIC.md` 冻结约束行 225）。
- **`model_dump(mode="json")` 产物不得被改变**（`EPIC.md` 冻结约束行 227）。
- **分层方向不可逆**（`EPIC.md` 行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。
- **基线不得重录**（`EPIC.md` 行 228；D5）。
- **不得新增 skip/xfail**（`EPIC.md` 行 229）。
- **每步落地后重跑 `just check-fast`**；改结构后加跑 `tests/architecture`（`EPIC.md` 行 230 与 `AGENTS.md` 第 1 节）。
- **`raw/` 只存已执行证据**；临时脚本与中间日志写会话 scratch。

## 风险与回退

- **风险：重写 `api.py` 时 `FrontAxleModel` 字段形状跟着漂移**（`EPIC.md` 冻结约束行 227 明列为硬门）。回退：先用 p5-01 的形状快照与 `model_hash` 做对照；任何一位变化即退回该步，先给独立于结果字节的等价说明再继续。
- **风险：`simulate` 另起装配路径，和阶段一 03 的文档驱动装配器形成两条路**（`EPIC.md` 行 73/92 明确它只是对外出口）。回退：`simulate` 只做「文档 → 既有装配器 → 既有运行入口」的薄封装；若发现需要复制装配逻辑，先登记并停下确认。
- **风险：门禁变红后顺手改门禁脚本或 allowlist**（`EPIC.md` F24 零容忍）。回退：改动实现侧；`legacy_surface_gate.py` 的 `MODE_MIGRATION`/`MODE_FINAL` 两模式口径按 `:45` 的原文核对。
- **风险：`composable_extension_examples.md` 的 `runnable` 代码块被 `check_composable_release.py` 执行**（`EPIC.md` F24）——示例文档改动会直接进门禁。回退：示例只增不改既有的 runnable 块语义；改动后立即跑 `check_composable_release.py --skip-isolation`。
- **本行不触及求解路径本身**（只改调用面），故 `just gate-numeric` 不是本行判据；若实现中动了 `simulation/backend.py` 或求解入口，按 `EPIC.md` 行 230 加跑数值门。

## Done-When

- [ ] `simulate(assembly_document, case_document)` 有签名、有文档、在 `__init__.py` 的 `_PUBLIC_NAMES` 与 `__all__` 中登记；一条端到端用例通过。
- [ ] `FrontAxleModel` 仍在 `__all__` 且未删除；`run_case` / `run_dynamic_case` 保持可用（内部改走 `simulate`）；p5-01 台账中的历史调用者全部可用。
- [ ] `model_dump(mode="json")` 的形状与 `model_hash` 与 p5-01 基准**逐位相同**。
- [ ] `legacy_surface_gate.py --check`、`check_composable_release.py --skip-isolation`（含 `runnable` 代码块）、`test_public_api_boundary_gate.py` 全绿；门禁脚本与 allowlist 未改。
- [ ] 内核提交唯一归属仍只有 `simulation/backend.py`。
- [ ] 未重录任何基线；未新增 skip/xfail；未触碰 ABI 版本常量。

## Final Validation Command

```bash
uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check && uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation && uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/architecture -q
```
