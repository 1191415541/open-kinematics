# P0 退出验证证据

本文件记录 P0（统一定义契约）的实际行为验收结果。**证据是行为判定，不是文件存在或退出码 0。**
每条结论都给出触发方式与观测到的输出；未执行或无法执行的项显式标为未覆盖。

环境：AMD Ryzen 9 5950X / Python 3.12.13 / numpy 2.2.0 / numba 0.67.0 / Windows 11 10.0.26200。

---

## 1. 请求身份、单位、版本、边界可往返（SPEC §58）

正向穿透：在 solve-plan 上写入 `characteristics` 后，经
`plan_from_case -> ResolvedSolvePlan -> compile_resolved -> case 重建 -> native reader`
全链，请求仍然存在且字段未被默认化、未被重排。

```
plan carries characteristics: True
compiled; case carries request: True
```

case 重建处的关键点：`compile_resolved` **重建** case 文档，若不显式搬运，
请求会在这一步静默丢失——调用方拿到旧通道却以为请求已被采纳。
已在 `compilation/resolved.py` 中显式搬运，并由测试 `test_request_survives_case_rebuild` 钉住。

## 2. 非法或未支持请求被逐层拒绝（SPEC §58、§65）

拒绝分两层实现，二者是同一契约的**独立**实现：Python 契约校验器与 native reader。
只测 Python 一侧无法发现「内核静默忽略未知字段」这一失效模式。

### 2.1 Python 侧（10 类负例，全部拒绝）

| 负例 | 观测到的拒绝信息 |
|---|---|
| 未知字段（`unit` 拼错为单数） | `$/characteristics/measurements[0]: unexpected fields ['unit']` |
| 未知 domain | `'nonsense' is not one of ['kinematic', 'equilibrium', 'trajectory', 'dynamic']` |
| 未知单位 | `measurement declares unsupported unit 'furlong'` |
| 未知 response_kind | `measurement declares unknown response_kind 'made_up'` |
| `samples: "all"` | `$/characteristics/samples: expected array, got str` |
| 重复样本 | `duplicate sample index 0` |
| 零版本 | `$/characteristics/boundary/version: must be >= 1` |
| 缺 `frozen_conditions` | `characteristics.frozen_conditions must be an object` |
| 缺 `response_version` | `$/characteristics/measurements[0]: missing required field 'response_version'` |
| dynamic kind 用于非 dynamic 域 | `definition kind 'continuous_descriptor' is dynamic-only` |

### 2.2 native 侧（绕过 Python schema 直喂 ABI，8 类全部拒绝）

方法：编译出合法 bundle 后**篡改已编译的 case 文档**并重新打包，直接调用
`suspension_kernel_run`。此时 Python schema 已不可能拦截，内核是唯一能拒绝的一方。

```
native accepts valid request -> code 0 err:
  REJECTED: unknown field in measurement :: characteristics measurement[0] has unknown field(s) "unit"
  REJECTED: unknown domain            :: characteristics request has unknown domain "nonsense"
  REJECTED: unknown unit              :: characteristics measurement[0] declares unsupported unit "furlong"
  REJECTED: unknown response kind     :: characteristics measurement[0] declares unknown response_kind "made_up"
  REJECTED: duplicate sample          :: characteristics request has duplicate sample index 0
  REJECTED: frozen without protocol   :: definition "frozen_force_tangent" requires frozen_conditions
  REJECTED: unknown definition kind   :: dynamic definition kind "vibes" is not a dynamic definition kind
  REJECTED: zero version              :: boundary requires a positive integer version
```

对照组：同一路径下**合法**请求被接受（`code 0`），因此上述拒绝来自请求内容而非打包方式。
另有一条测试确认「篡改请求 + 原始 payload」仍拒绝，证明拒绝不是 payload 损坏的副作用。

## 3. 未支持能力 fail-closed（SPEC §67）

本构建**读取**请求但不**计算**任何响应。这一点必须显式声明，否则会出现
「接受请求 → 返回原通道 → 调用方以为已计算」的空壳能力。

- `capabilities` 新增两字段：`characteristics_request_versions: [1]`（读取能力）与
  `characteristics_response_kinds: []`（应答能力，本构建为空）。
- 编译期 `require_characteristics_answerable(kind)` 对未支持 kind 直接报错：

```
ValueError: loaded kernel does not compute characteristics response kind 'frozen_force_tangent'
```

读取能力与应答能力**分开声明**是刻意的：把它们合成一个字段就无法表达
「能读不能算」这个真实状态。

## 4. 无请求时旧通道逐位不变（SPEC §67）

同一算例分别在「无请求」与「带请求」下运行，比较全部 native 数值块：

```
block set unchanged: True
ALL blocks bit-identical (NaN-aware): True
sample_count: 18 == 18
```

说明：`diagnostics` 块含结构性 NaN 填充（两例 NaN 掩码完全一致，有限值最大差 **0.0**），
因此比较用 NaN-aware 语义；直接 `array_equal` 会因 NaN != NaN 误报差异。
独立佐证：`dynamic_hash_sentinel.py --check` 报告
`OK: dynamic output matches the frozen baseline byte-for-byte`（26 个 artifact 冻结哈希不变）。

## 5. 准静态 all_accepted 与伪时间 trajectory（SPEC §67）

`definition.kind` 四个值 `frozen_force_tangent` / `frozen_load_compliance` /
`continuous_descriptor` 均为 **dynamic-only**：它们分别命名、分别接受，且在
`kinematic` / `equilibrium` / `trajectory` 域上被拒绝（见 §2.1 末行）。
因此准静态与伪时间域的请求不会静默套用冻结态定义。

`frozen_*` 两值额外要求 `frozen_conditions.protocol`，缺失即拒绝；`continuous_descriptor`
不要求该字段。这一区分由 `test_dynamic_definitions_are_named_and_separated` 覆盖。

## 6. 未覆盖与阻断项（诚实登记）

- **Adams Car 2024.1 已安装（2026-10-13 修订）**：原版此处断言「未安装、仅有
  `G:\MSC.Software\Adams\2025_1_1`」，**该断言已被实测证伪**。实测证据：
  `C:\Program Files\MSC.Software\Adams\2024_1\` 存在（含 `bin\adams2024_1.bat`、
  `acar\shared_car_database.cdb\subsystems.tbl\TR_Front_Suspension.sub`）；
  `suspension_multibody.adams.probe_profile('adams-car-2024.1')` 返回
  `available=True` / `version="2024.1"` / `license_probe="passed"`；
  `validate-adams --profile adams-car-2024.1 --smoke --require-installed` **exit 0**。
  原版失败的根因是**路径核对范围不足**——只查了 `G:\...` 与 `C:\MSC.Software\...`，
  漏掉真实的 `C:\Program Files\MSC.Software\Adams\2024_1`。
  六组规则据此在 2024.1 本地 help 上重建：G6 由 `pending_definition` 升为 `stated`，
  G3 由 `pending_definition` 改为 `partial`（`adams_solver/gtcmat.html` 明确给出
  C++ Solver 动态柔度算法，「动态段无完整算法」的旧结论对 Solver 手册不成立），
  G1/G2/G5 为 `partial`，G4 为 `not_stated`。**判定口径未放宽**：Adams 兼容的完整动态冻结
  等价声称仍被阻断，未知项不得默认使用本项目规则。
  本节引用的版本一致性缺陷（原版版本列写 2024.1、路径列却是 2025_1_1）已随
  `raw/adams-rule-audit.csv` / `.md` 的重写一并修正；原版保留在
  `backups/20261013-p0-adams-2024-1-audit/`。
  按 EPIC §36 与 P0 SPEC §41–42，定义缺口**不阻断 P0 DONE**，
  但**严格阻断 P5 与 P6 的 Adams 兼容发布**。
- **独立参考数据缺失**：Adams 对标所需独立数据保持 `pending_reference`。
- **native 其余路径仍宽松**：`contract_read_characteristics` 已严格拒绝未知字段；
  case 文档的其余部分仍走旧的宽松 `JsonValue::find` 路径，未知字段被忽略。
  这是 P0 step 5 的既定范围（仅 characteristics 请求），不是遗漏。
- **指标计算未实现**：本阶段只落契约与读取链；37 项指标的数值实现属 P1–P4。
  `metric-catalog.csv` 中除 `roll_center_legacy=existing` 外全部 `planned`。

## 7. 门禁结果（仓库根执行，2026-10-10/12 历史记录）

> 下表是步骤 5/6 首次收口时的记录。2026-10-13 因重开步骤 5/6（native 请求读取与
> manifest 回显）后的**重验结果见 §9**；下表的数字不代表当前状态。

| 门 | 结果 |
|---|---|
| `ruff check .` | `All checks passed!` (exit 0) |
| `ty check .` | `All checks passed!` (exit 0) |
| `legacy_surface_gate.py --check` | `OK: no unregistered Python boundary violation` (exit 0) |
| `check_module_layering.py --strict --final` | `OK: layering matches the recorded baseline`（108 source edges / 0 cycles / 0 mutual） (exit 0) |
| `check_composable_release.py --skip-isolation` | `OK: 3 release checks passed` (exit 0) |
| 快速测试集 | **1581 passed, 1 xfailed**（skip 0；xfail 未增长） |
| `packages/suspension_kernel/tests` + `packages/suspension_contracts/tests` | **176 passed** |
| `dynamic_hash_sentinel.py --check` | `OK: dynamic output matches the frozen baseline byte-for-byte` (exit 0) |

新增测试：`packages/suspension_multibody/tests/characteristics/test_request_contract.py`（28 例）、
`packages/suspension_kernel/tests/test_characteristics_request.py`（15 例）。共 43 例。

未重录任何数值/性能/架构基线。唯一改动的基线是
`packages/suspension_kernel/layering_baseline.json`——新增 `mb_cases -> mb_contract`
的 `source_edges` 记录与 `source_edge_evidence`（来源 `cpp/src/cases/case_characteristics.cpp`，
kind `symbol_reference`，symbol `ContractDocument`），并附人工复核判决；
该边方向与既有 `abi -> mb_contract` 一致，未引入新依赖方向，门禁复跑 0 cycles / 0 mutual。

## 8. 过程中发现并修复的真实缺陷

1. **请求在 case 重建处丢失**：`compile_resolved` 重建 case 文档时未搬运
   `characteristics`，导致请求静默消失。已修，并由测试钉住。
2. **循环导入**：`characteristics.py` 以 `from suspension_contracts import ContractError`
   引入自身包，且 `__init__.py` 把它排在 `multibody` 之前，形成部分初始化包导入失败。
   已改为 `from .multibody import ContractError` 并把导入排到 `multibody` 之后。
   该缺陷由 `check_composable_release.py` 的文档示例执行暴露。
3. **变量遮蔽导致 `AttributeError`**：门禁循环变量 `request` 遮蔽了
   `compile_resolved` 的 `request` 关键字参数。已改名 `declared`。
4. **测试进程崩溃（Windows access violation）**：在 `packages/suspension_kernel/tests/`
   中于进程内调用 `migrate_v1_kc_case`，会在本会话已通过 ctypes 载入内核共享库之后
   触发 numba JIT 崩溃（崩在 `spatial.py::_quaternion_to_matrix_unit`）。
   该崩溃仅在 `test_binding.py` 之后出现，单跑不出现。
   已改为**子进程**编译 bundle 并落盘复用，套件不再触发崩溃（kernel 套件 112 passed）。

## 9. 重开步骤 5/6 后的重验（2026-10-13）

步骤 5/6 于 2026-10-12 被重开，原因：native 只读取 `profile`/`version`，
`held`/`released` 与 `wheel_end_response` 未被完整消费。本轮补齐并重验。

### 9.1 补齐的实现（可执行证据）

- `cpp/include/mb_cases/functions.hpp`：`ContractCharacteristics` 新增
  `boundary_released` / `boundary_held`（`std::vector<std::string>`）与
  `has_wheel_end_response` / `wheel_end_response`（`JsonValue`）。
- `cpp/src/cases/case_characteristics.cpp`：新增 `read_boundary_names(...)`，
  逐项拒绝非字符串、空串与列表内重复；并**拒绝同时出现在 `held` 与 `released`
  的名字**（错误信息含 `in both held and released`）；`wheel_end_response` 由
  「只校验 `is_object`」改为**原样存回**。
- `cpp/src/abi/kernel_contract_run.cpp`：请求存在时，把解码后的请求**回显到结果
  manifest 的 `characteristics` 字段**（兑现 `functions.hpp` 原有承诺）。
- 测试 `packages/suspension_kernel/tests/test_characteristics_request.py`：
  **15 → 25 passed**，新增 10 条（含 5 个参数化负例与 2 个回显正例）。

**反证（判别力验证）**：把 held/released 冲突检查改为 `if (false)` → 重建 →
测试 **FAILED（缺陷被逮住）**；还原后 25 passed。说明该测试能区分「接线被断开」
与「正常实现」，不是只断言部署。

### 9.2 关键语义（权威来源，避免二次猜测）

- `solve_plan.schema.json:53-54`：`released`/`held` 是
  `{"type":"array","items":{"type":"string"}}`（字符串数组＝运动边界名）；
  `:81`：`wheel_end_response` 是 `{"type":"object"}`。
- `characteristics.py:102`：`_BOUNDARY_FIELDS = {"profile","version","released","held"}`。
- 设计 `unified_characteristics_design.md:132`：`wheel_end_response` 记录左右轮心
  frame/测量 ID、轴及版本。

### 9.3 本轮门禁（实际执行，命令 / 结果 / 退出码）

| 门 | 命令 | 结果 |
|---|---|---|
| build-kernel | `python packages/suspension_kernel/scripts/build_suspension_kernel.py` | exit 0 |
| build-axle-native | `python packages/suspension_multibody/scripts/build_axle_native.py` | exit 0 |
| P0 native 契约测试 | `pytest packages/suspension_kernel/tests/test_characteristics_request.py` | **25 passed** |
| kernel 套件 | `pytest packages/suspension_kernel/tests` | **129 passed** |
| contracts 套件 | `pytest packages/suspension_contracts/tests` | **78 passed** |
| no_legacy 架构测试 | `pytest .../architecture/test_no_legacy_vehicle_models.py` | **5 passed** |
| 快速集 | `pytest packages/suspension_multibody/tests --ignore=…adams --ignore=…architecture --ignore=…cases` | **2051 passed, 1 xfailed**（skip 0，xfail 未增长） |
| characteristics 定向 | `pytest packages/suspension_multibody/tests/characteristics` | **489 passed** |
| `ruff check .` | — | `All checks passed!` (exit 0) |
| `ty check .` | — | `All checks passed!` (exit 0) |
| `legacy_surface_gate.py --check` | — | `OK: no unregistered Python boundary violation`（findings 0, exit 0） |
| `check_module_layering.py --strict --final` | — | `OK: layering matches the recorded baseline`（0 cycles, exit 0） |
| `check_composable_release.py --skip-isolation` | — | `OK: 3 release checks passed` (exit 0) |
| `dynamic_hash_sentinel.py --check` | — | `OK: dynamic output matches the frozen baseline byte-for-byte` (exit 0) |
| `case_parity_check.py` | — | `OK: 8 families accepted` (exit 0) |
| `kc_perf_gate.py` | — | **FAIL**（见 9.4） |
| `git diff --check` | — | exit 0（仅 Git 的 LF→CRLF 提示） |

未重录任何数值/性能/架构基线；`layering_baseline.json` 未再改动。

### 9.4 `kc_perf_gate.py` 的 FAIL 判定（登记口径）

本轮两次执行均 FAIL，但**判定为「高负载下的负载敏感抖动 + 该路径不经本次改动」**，
依据两条独立证据：

1. **比值自身在抖动**（基线固定不变）：k-100 ×1.288 → ×1.467，c-66 ×1.275 → ×1.427；
   门禁脚本自述「同一负载在安静机器实测 ×0.92、繁忙桌面 ×1.5」，并因此默认取 5 次
   最优值（`kc_perf_gate.py:39-51`）。一个真实回归不会让同一二进制在同一基线上
   两次读数相差 0.18。
2. **本次改动不在该路径上**：实测 perf 的两个用例（k-100 / c-66）经 `validate()`
   后**不含 `characteristics` 键**（打印 `k-100 contains characteristics key: False`
   / `c-66 … False`），即 P0 新增的 `characteristics` 解码与 manifest 回显分支
   **根本不执行**。

因此**不重录基线、不放宽 `BUDGET_FACTOR`**，如实记为
`BASELINE_RED` 性质的环境抖动，而非本次回归；也不把它写成 `PASS`。

### 9.5 本轮仍未覆盖

- `raw/reference-manifest.json` 仍为 `pending_reference`（P5 逐点对标所需独立参考未生成）。
- `contract_read_characteristics` 之外的 case 文档字段仍走旧宽松路径（P0 既定范围，非遗漏）。
- 指标数值实现属 P1–P4，不在 P0 范围。
