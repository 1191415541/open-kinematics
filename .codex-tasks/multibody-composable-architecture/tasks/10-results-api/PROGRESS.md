# 10 输出、公共 API 与兼容结果

## Recovery

- 任务：`10 收敛输出解码并薄化公共API`。形态：single-full。依赖 07（DONE）;09（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 11 行；输入规格 `../../TASKS.md` 第 10 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/outputs packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/architecture -q`，补充 product 全量。

## 实测现状（进入本任务时）

- `api.py` 963 行，承担三类不属于编排的工作：解码原生行（`_rigid_state` 读 `entry[:3]`/`entry[3:7]`）、读原生块列（`_tire_compression` 写死列 2）、**在 Python 里算本构**（`_collect_element_results` 调 `evaluate_generalized_forces` 并算 `-K d + preload` 与应变能）。
- 列布局知识有**两个家**：`api.py:87-93` 声明 `_DIAGNOSTIC_POSITION_RESIDUAL = 7` 等三个常量，而 `results/raw.py:242` 用同样的字面量 7/9；api 那份**无人读**。
- `outputs/` 包（1790 行）在生产代码中**不可达**：`src/` 内 grep 无引用，只有测试导入。
- **GAP-2 实测仍在，且比 01 记录的更靠前**：
  - `api._kc_assembly` 恒以 `DEFAULT_AXLE_SUBSYSTEMS` 构建，**无转向运行根本无法通过公共入口提出**——所以此前的测试只能验私有 `_k_grid`。
  - `model_document` 的 `_driven_coordinates` 无条件发 rack 驱动坐标并调 `assembly.point("rack","center")`，无 rack 时抛 `KeyError: ('rack','center')`。
  - 01 记录的 `ValueError: missing required front-axle hardpoint for rack_center`（`subsystems/geometry.py:102`）是**另一个触发点**：steering 存在而硬点被删时才发生。

## 做了什么

### 解码归位 `results/kc_state.py`

新增模块，三件事各只在一处：**米转毫米**、**行切片到状态对象**、**块列到下标的命名量**。`api._rigid_state` 与 `api._tire_compression` 保留原名（调用点不变）但改为调用它；`api` 不再自己切片。

行方向说明：契约的 body_state 行是每体 19 列（含速度等），原先的 `entry[:3]`/`entry[3:7]` 取的是**每体头 7 列**。新实现按体索引而非固定 reshape，所以更宽的行也能读，同时把"行数必须等于体数、每行至少 7 列"作为显式拒绝条件。

### 清除 `api` 自有的列布局常量

三个 `_DIAGNOSTIC_*` 常量删除。原先 api 与 `results.raw` 各持一份 7/9，而只有 results 那份被读——两处一个数字、无一处权威，正是本任务要消除的形态。

### GAP-2 闭合：无转向 rack 通道真正消失

三层依次收缩，缺一层这个缺口就不闭合：

1. **case 能提出请求**：`CaseSpec` 增 `subsystems: frozenset[str] | None = None`（默认 `None` = 默认单轴集，既有调用方不变）；`api._kc_assembly` 收到非 `None` 时构造 `AssemblyRequest` 传入 `assembly_for`。
2. **文档不再要求不存在的体**：`contract.has_rack(assembly)` 按「有 rack 体 **且** 有 rack center 点」判定；`_driven_coordinates` 只在为真时发 rack 驱动坐标；`case_document` 的 `axis_map["rack"]` 同样按它出现。
3. **结果通道省略而非置零**：`api._k_drives` 仅在 `has_rack` 为真时写入 `rack_displacement`。

内核侧一处最小放宽：`kc_quasi_static.cpp` 的 shorthand 原要求 `rack_values_mm` 与 `axis_map.rack` 必须存在。现改为可选；缺省时轴取 `[0.0]`（该维不扫）。**模型确有 rack 坐标而轴为空仍拒绝**——否则等于用“扫过”的名字把 rack 悄悄固定在装配值上。这与契约 schema 一致（`rack_values_mm`/`axis_map.rack` 本就非 required）。

## 关键实测

| 判据 | 实测值 |
|---|---|
| 无转向运行 | `run_case(无 rack_center 模型, CaseSpec(mode=K, subsystems=DEFAULT−steering))` → 3 states，全部收敛，残差 < 1e-6 |
| 无转向结果通道 | `drives` 键 = `{wheel_travel_left, wheel_travel_right}`，**无** `rack_displacement` |
| 有转向结果通道 | `drives` 含 `rack_displacement`（负对照） |
| 无轮胎的压缩通道 | `tire_compression == {}`（缺席而非 0.0） |
| 报告值与内核值 | `tire_compression_from_run(run,0)[side] == run.block("tire_output")[0,i,2]` 逐位相等 |
| K/C 冻结快照 | kc_parity 容差内，无漂移 |
| 8 个 family | case_parity 全 PASS |

## 门禁设计（17 项新增）

`tests/api/test_no_steering_shrinks_rack.py`（9 项）：`has_rack` 问装配而非角色名 / 无 rack 时文档不发 rack 坐标且无 rack 体 / 有 rack 时仍发（负对照）/ 无转向运行收敛 / **rack 通道缺席** / 有转向仍含该通道 / 非单轴角色点名拒绝 / `DynamicCaseSpec` 无该字段 / `CaseSpec` 默认 `None`。

`tests/api/test_decoding_belongs_to_results.py`（8 项，含 2 参数化）：`api` 不再声明自有列下标 / `api` 经 `results` 解码 / 解码函数定义在 `results` / element-wrench 生产开关状态被记录为已知未接线 / 报告值与内核块逐位相等 / 无轮胎时无键 / K 与 C 两模式都走同一解码路径。

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/results tests/outputs tests/metrics tests/architecture -q` | 0 | 主验收 **233 passed** |
| `pytest tests/api -q` | 0 | 20 passed（新增 17 项） |
| `python scripts/kc_parity_check.py --check` | 0 | 冻结快照容差内，无漂移 |
| `python scripts/case_parity_check.py` | 0 | 8 families 全 PASS |
| `pytest tests/rigs -q` | 0 | 56 passed（GAP-2 相关断言已更新为断言收缩） |
| `ruff check .` / `ty check .` | 0 | 全通过 |
| `pytest tests -q`（全量） | 0 | **1277 passed, 1 skipped, 1 xfailed**（10 前实测 1259/1/1；skip 与 xfail 未增长，无原本通过者转失败） |

## 未覆盖与保留（明确登记，不虚称已解决）

- **`api._collect_element_results` 仍在 Python 里算本构**（`-K d + preload` 与应变能）。原生 element-wrench 通道存在且由 `results.element_wrench` 解码，但它在**环境变量开关**之后，生产未开启；接线等于改变生产行为与输出契约，属独立事项，已在 `test_decoding_belongs_to_results.py` 中作为已知未接线状态固定下来，不由本任务悄悄声称完成。
- **列布局仍有不止一个家**：`axle_dynamics/result.py`（九个块列表）、`results/element_wrench.py`（元素行）、`io/artifacts.py`（manifest layouts）、`report/metrics/*`（硬编码下标）、`outputs/builtin.py`（第三次硬编码）。本任务收口的是 `api` 这一处（子任务写范围内的违规），其余需要跨 `axle_dynamics`/`report`/`outputs` 的协同改动，未在本任务一并做。
- **`outputs/` 生产不可达**：声明与派生层已建好且有测试，但 `SimulationRequest.outputs`/`SolvePlan.outputs` 无人从 `DeclarationSet` 填充。把它接入是「输出声明驱动运行」的更大改动。
- 新增行为：`CaseSpec.subsystems` 是公开输入的新字段（默认 `None`，不改变既有调用）。
