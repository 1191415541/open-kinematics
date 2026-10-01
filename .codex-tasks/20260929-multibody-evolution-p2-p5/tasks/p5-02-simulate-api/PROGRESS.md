# PROGRESS：p5-02 simulate 公共入口与 FrontAxleModel 降级

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-02`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p5-02-simulate-api
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-02-simulate-api/`
- **Spec**: `SPEC.md`
- **Plan**: `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `E:\杂件\open-kinematics`
- **Depends on**: `p5-01`（`SUBTASKS.csv` 第 20 行）

## Context Recovery Block

- **Current milestone**: 全部 5 步完成
- **Current status**: DONE（2026-10-01）
- **Last completed**: 步骤 5（门禁复验 + 证据落盘）
- **Current artifact**: `api.py` 的 `simulate` + 私有辅助、`__init__.py` 的登记、
  `tests/api/test_simulate.py`、`raw/{simulate_entrypoint.md,legacy_adapter.md,gates.txt,run_log.md}`
- **Key context**:
  - `simulate(assembly_document, case_document) -> SimulationRun`（`api.py:130`），
    是「文档 → 既有装配器 → 既有运行入口」的薄封装，未新增装配路径。
  - `__all__` **17 → 18**，`_PUBLIC_NAMES` **16 → 17**（各 +1 条 `simulate`）；
    `FrontAxleModel` 等既有名字一位未动。
  - `model_hash` 与 `model_dump(mode="json")` 形状**逐位不变**（两处基准见
    `raw/legacy_adapter.md` §3）。
  - 内核提交唯一归属仍是 `simulation/backend.py:24`。
- **Known issues（本行边界，非红）**:
  - `run_case` / `run_dynamic_case` **未字面**改为调用 `simulate`（理由与等价替代见
    `raw/legacy_adapter.md` §2）；它们与本入口共享提交环境与提交后端。
  - 只支持 `assembly_kind == "suspension_axle"` + `family == "kc_quasi_static"`，
    其余按名字拒绝（整车文档侧尚无文档驱动的 family preparation）。
  - 未改 `composable_extension_examples.md`（其 `runnable` 块被门禁执行，加自包含例子成本过高）；
    门禁原样输出 `[PASS] documentation examples: 3 example(s) executed: E-1, E-2, E-3`。
- **Next action**: 交回主管做 EPIC 层收尾（p5-03 要在 `api.py` 暴露段新增信号总线出口，
  本行的公开出口结构已定稳）。

---

## Final Summary

### 交付

| 文件 | 改动 |
|---|---|
| `src/suspension_multibody/api.py` | 新增公共 `simulate` + 私有 `_documented_case` / `_documented_text` / `_optional_text` / `_documented_reading` / `_documented_assembly` / `_element_wrench_facts`；`_compile_plan_run` 的元素通道改走同一上下文；`run_case` docstring 修正 |
| `src/suspension_multibody/__init__.py` | `TYPE_CHECKING` 导入、`_PUBLIC_NAMES`、`__all__` 各登记 `simulate` |
| `tests/api/test_simulate.py`（新增） | 18 个用例：公开面 3、端到端 8、拒绝 8 |
| `raw/simulate_entrypoint.md` | 签名原文、docstring、`__all__` 登记、端到端产物摘要、边界清单 |
| `raw/legacy_adapter.md` | 绞杀者现状、与 SPEC「内部改走 simulate」的偏差登记、`model_hash` 逐位对照 |
| `raw/gates.txt` | 三条门禁 + 快速集 + `run_contract` 归属 + 公共面计数的原始输出 |
| `raw/run_log.md` | 命令、退出码、关键输出、临时脚本清单 |

### 五条验收（全部实跑，退出码 0）

| # | 命令 | 退出码 | 关键行 |
|---|---|---|---|
| 1 | `legacy_surface_gate.py --check` | 0 | `findings : 0` / `OK: no unregistered Python boundary violation` |
| 2 | `check_composable_release.py --skip-isolation` | 0 | `[PASS] documentation examples: 3 example(s) executed: E-1, E-2, E-3` / `OK: 3 release checks passed` |
| 3 | `pytest tests/api tests/architecture -q` | 0 | `193 passed in 621.82s` |
| 4 | `dynamic_hash_sentinel.py --check` | 0 | `combined sha256 : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` / `OK: dynamic output matches the frozen baseline byte-for-byte` |
| 5 | `pytest tests -q --ignore=adams --ignore=cases --ignore=architecture` | 0 | `1108 passed, 1 xfailed in 38.45s` |

另跑：`pytest tests/architecture/test_public_api_boundary_gate.py` → `5 passed`；
`check_module_layering.py --strict --final` → 0 环、`OK`；
`pytest suspension_kernel/tests suspension_contracts/tests` → `73 passed`。

### `model_hash` 硬门

| 夹具 | 改动前 | 改动后 | 逐位 |
|---|---|---|---|
| `tests/data/benchmark_axle.json` 的 `"model"`（19 键 / 1007 B） | `72bd9f30…60b` | `72bd9f30…60b` | ✅ |
| `tests/conftest.py::full_vehicle_model`（12 键 / 11835 B） | `75cb96f5…fd9` | `75cb96f5…fd9` | ✅ |

### 未做（见 `raw/simulate_entrypoint.md` §5 与 `raw/legacy_adapter.md` §2）

1. `run_case` / `run_dynamic_case` 未字面调用 `simulate`（SPEC Done-When 第二条按等价共享登记，非静默降级）；
2. 整车（`full_vehicle`）文档与 `kc_quasi_static` 之外的 family 按名字拒绝；
3. 未改 `composable_extension_examples.md`。

## 主管独立复核与裁决（2026-10-01，本行收口）

### 独立复核的三条门禁（主管实跑，非采信子任务自报）

| 命令 | 退出码 | 关键输出 |
|---|---|---|
| `legacy_surface_gate.py --check` | **0** | `findings : 0` / `OK: no unregistered Python boundary violation` |
| `check_composable_release.py --skip-isolation` | **0** | `[PASS] documentation examples: 3 example(s) executed: E-1, E-2, E-3` / `OK: 3 release checks passed` |
| `pytest tests/api tests/architecture -q -p no:cacheprovider` | **0** | `193 passed in 637.67s` |
| `dynamic_hash_sentinel.py --check` | **0** | `combined sha256 : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` |
| `pytest tests/api -q`（复验） | **0** | `46 passed in 2.64s` |

另：`from suspension_multibody import simulate` 可用；`len(__all__)` = **18**（原 17）；
实测签名 `(assembly_document: 'str | Path | AssemblyDocument | SimulationAssembly', case_document: 'Mapping[str, Any]') -> 'SimulationRun'`。

### 裁决：SPEC 判据 2 改为**验收口径**，不要求两条旧入口字面调用 `simulate`

由 `code-reviewer`（只读裁决，2026-10-01）出具，结论与依据：

1. **字面调用在当前输入契约下不能实现**：`simulate` 收的是可加载的总成文档 + K/C 工况文档，
   而 `run_case`/`run_dynamic_case` 收的是 `FrontAxleModel`（`api.py:323-357`）；总成与 rig 必须从
   文件引用加载（`authoring/documents.py`），模型对象不能直接充当它们。要让旧入口「走 simulate」
   就得从 model 反造一份总成文档，那会另立装配来源、并越过 `authoring/solver.py` 要求的文档角色/模板/配对来源。
2. **零回归硬门优先**：`EPIC.md` 冻结约束与 `SPEC.md` 的 Constraints 都禁止改动被
   `kc_baseline`/`dynamic_hash_baseline` 冻结的产物。**不为满足调用关系而牺牲零回归。**
   （裁决同时纠正了原理由的一处表述：`kc_baseline/` 冻结的是 K/C 状态与清单，不是「输入文档字节」；
   反对的理由应表述为「改变装配来源会带来结果变化风险」，不能断言必然改变每份输入文档。）
3. **替代验收口径**（本行据此判定 Done-When 第二条为**达成**）：
   - 旧入口的**签名未改**、仍可用（快速集 **1108 passed / 1 xfailed** 作为历史调用者回归网）；
   - `FrontAxleModel` 未删除、仍在 `__all__` 与 `_PUBLIC_NAMES` 中；
   - `model_dump(mode="json")` 形状与 `model_hash` **逐位不变**（19 键 benchmark_axle 与 12 键 full_vehicle_model 两条均一致）；
   - 两条路**共享可观测的提交后端**：同一 `_element_wrench_facts()` 提交环境（两条路的结果都带 `element_wrench` 块）、
     同一唯一内核提交点 `simulation/backend.py:24`。
   - **不采信**新增测试里「名字可导入」这类弱断言单独承担该验收。
4. **不是判据弱化**：逐个 hunk 检查了 `api.py` 与 `__init__.py` 的 diff，删除行只有 import 改写与
   `_compile_plan_run` 里 try/finally 的**原位抽取**（抽成 `with _element_wrench_facts():`，行为等价）；
   没有任何既有断言被删除或放宽。`tests/api/` 除新增 `test_simulate.py` 外无改动。
5. **未另起第二条物理装配路径**：`simulate` 调用的是既有 `authoring.solver.assembly_request_for` /
   `front_axle_model_for` / `preparation.kc_quasi_static.assembly_for` 与 `simulation.runner.run_request`，
   未见第二套物理装配器。

### 裁决要求的两处 docstring 修正（主管已亲自改，2026-10-01）

裁决指出实施者的两处 docstring 陈述不准确，主管已修正：

1. `simulate` 的 docstring（`api.py:137-142` 原文）称「two documents go to the assembler … the contract
   documents they imply are compiled by the family compiler」——**不准确**（所述与实际调用链不符）。
   已改为准确表述：总成文档由既有的文件装配器读取，该装配自身产出的 model document 由装配层发射，
   model/case 对经既有的 document runner 送进唯一内核提交点；本入口**不**自行写文档、**不**挑选被验对象、
   **不**拥有第二套物理装配实现。
2. `run_case` 的 docstring（`api.py:334-338` 原文）称两入口经「the same compiler and runner」、
   是「the same run rather than two paths that agree」——**不准确**：旧入口走
   `compile_plan` + `run_compiled`，新入口走 `run_request` + `DocumentPairCompiler`。
   已改为准确表述：两条入口**共享提交环境与唯一内核提交点**，而非共享调用路径。

### 其它已登记的未做项（不是缺口，是边界）

- 整车（`full_vehicle`）文档与 `kc_quasi_static` 之外的 family **按名字拒绝**（整车家族尚无文档驱动准备层，`preparation/**` 不在本行写范围）。
- 未改 `composable_extension_examples.md`（新增一个自包含的 `simulate` 示例需在 gate 执行的 `runnable` 块里内联约 60 行项目 JSON，超出本行最小交付）。
