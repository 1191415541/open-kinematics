# PROGRESS：p2-03 多体层力矩元接入

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-03`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p2-03-assembly-torque
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-03-assembly-torque/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 DONE）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: 完成（6/6）
- **Current status**: DONE
- **Last completed**: 第 6 步——既有元素类型装配产物对照与数值门
- **Current artifact**: `raw/element_declaration.md`、`raw/port_pairing.md`、`raw/min_assembly_torque.md`、`raw/legacy_elements_parity.md`、`raw/run_log.md`
- **Key context**:
  - 依赖 p2-02（内核元素类型与 ABI 编组）已落地：`ELEMENT_ROTATIONAL_TORQUE = 7`、参数槽 128/129/130/133/137、`kElementBlockSize = 216`、ABI 17/32/1。本行只消费，未改内核任何文件、未改版本常量。
  - **编译层落点实测复核**：SPEC 列出的三个候选（`cases/kc_quasi_static/contract.py:436-446`、`cases/axle_dynamic.py:188`、`studies/bridge.py:119-126`）都是**合同文档**发射点，而内核的文档读取器按类型名拒绝本族（实测原文：`model document: element "probe" has unsupported type "rotational_torque"`；同一批的对照条 bushing 因「缺 stiffness/damping」被拒，说明拒绝是本族专属而非文档格式问题）。内核能消费本族的唯一输入是 `mb_core_run` 的 `ElementBlock`。故落点取新模块 `compilation/element_blocks.py`。
  - `subsystems/element_build.py` 三段串行：本行占构造分派段（`:74-75`），轮胎段 `_tire`（`:140`）与分支（`:70-71`）在 `git diff` 中均为 context 行。
- **Known issues**:
  - **`subsystems/types.py::ELEMENT_KINDS`（`:102-109`）未登记本族，是已知缺口**（该文件不在本行写范围）。当前无生产代码校验 `row.kind` 于该元组，故不影响运行；**移交 p2-04**。
  - **内核的合同文档读取器不认识本族**：`cpp/src/cases/contract_model.cpp:830` 的 fall-through 与 `cpp/src/contract/contract_registry.cpp:36-40` 的 `kElements` 名单都缺本族。属内核范围（p2-02），本行只登记，未改。
  - **`just check-fast` 整体退出码为 1，原因在 `lint`**：`uv run --all-packages ruff check .` 报 17 个错误，**全部落在 `.codex-tasks/` 下其它行遗留的探针脚本**（p2-01 的 `raw/rear_steer_probe.py`、p3-01 的 `probe_*.py`），非本行文件。本行文件的 ruff 退出码 0；`ruff check --exclude .codex-tasks .` 与 `git ls-files -z -- '*.py' | xargs -0 ruff check` 均 `All checks passed!`。`check-fast` 的其余部件（type-check、三条架构门、`test-fast` 1130 passed/1 xfailed、kernel 41 passed、contracts 32 passed）全部退出码 0。
  - `dynamic_hash_sentinel.py --check` 内部打印的 `acceptance exit : 1` 与 9 个 `FAILED` 用例名是冻结基线的既有状态（与 p2-01 记录、p2-02 `raw/kernel_torque_evidence.md:119-135` 完全一致），脚本自身判定为 `OK: ... byte-for-byte`、退出码 0、combined sha256 逐字符一致。

## 交付

| 项 | 位置 |
|---|---|
| 元素声明 | `modeling/primitives/elements.py:633 RotationalTorqueParameters`、`:707 RotationalTorqueElement` |
| 构造分支 | `subsystems/element_build.py:74-75`（`if row.kind == "rotational_torque": return _rotational_torque(row)`），构造器 `:195` |
| 编译层 | 新模块 `compilation/element_blocks.py`（`:65` kind、`:71-81` 参数槽、`:133 pair_torque_bodies`、`:211 torque_element_row`、`:231 rotational_torque_block`） |
| 配对决定点 | `compilation/element_blocks.py:200-208`（`reaction_body=port.owner.local`，与 `connections/links.py:203` 同一规则；未改 `connections/`） |
| 新增测试 | `tests/modeling/test_rotational_torque_element.py`、`tests/subsystems/test_rotational_torque_element.py` |

四改三新，185 增 0 删；`git diff --numstat` 与 `git status --short` 全文见 `raw/run_log.md`。

## 四条验收命令的真实退出码

1. `pytest tests/modeling tests/subsystems -q -p no:cacheprovider` → **0**（224 passed）
2. `pytest tests/architecture -q -p no:cacheprovider` → **0**（147 passed，590.05 s）
3. `check_module_layering.py --strict --final` → **0**（`OK: layering matches the recorded baseline`）
4. `dynamic_hash_sentinel.py --check` → **0**，combined sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`；`git status --short -- tests/data/` 为空

数值门另两项：`case_parity_check.py`（无参数）→ 0，`kc_perf_gate.py --check` → 0。

## Next action

无（本行完成）。移交项见上面 `Known issues` 的两条：`ELEMENT_KINDS` 缺口归被派发的后续行，内核文档读取器缺本族归 p2-02 的后续处置。
