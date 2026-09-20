# 子任务 02 进度：迁移六个非 vehicle_dynamic family 的装配前处理

- **任务**：迁移六个非 vehicle_dynamic family 的装配前处理
- **形态**：single-full（Epic 子任务）
- **进度**：4/4 步骤 DONE
- **状态**：主线程验收完成；独立审查缺口已修复并复测
- **文件**：`.codex-tasks/20260920-unified-preparation-cutover/tasks/02-family-preparation/`

## 交付内容

### 新增 preparation 包（每个 family 一个模块，导出 `prepare_request(request) -> PreparedSimulation`）

| registry key | 模块 | 准备类型 | 产出 context keys |
|---|---|---|---|
| `("axle","axle_dynamic")` | `preparation/axle_dynamic.py` | `AxleDynamicPrepared`(L31) | `prepared_simulation`, `axle_dynamic_prepared` |
| `("axle","kc_quasi_static")` | `preparation/kc_quasi_static.py` | `KcQuasiStaticPrepared`(L56) | `prepared_simulation`, `kc_assembly`, `model_document`, `case_document` |
| `("vehicle","vehicle_kc")` | `preparation/vehicle_kc.py` | `VehicleKcPrepared`(L49) | `prepared_simulation`, `model_document_pair`, `vehicle_assembly`, `wheels`, `case_document` |
| `("vehicle","handling")` | `preparation/handling.py` | `HandlingPrepared`(L47) | `prepared_simulation`, `model_document`, `model_payload`, `case_document` |
| `("vehicle","ride_four_post")` | `preparation/ride_four_post.py` | `RideFourPostPrepared`(L42) | 同 handling |
| `("vehicle","ride_random_road")` | `preparation/ride_random_road.py` | `RideRandomRoadPrepared`(L45) | 同 handling |

- `preparation/__init__.py`：只放包说明，不 import 任何 family 模块，保持中央延迟 import 表语义。
- domain 输入载体：`KcQuasiStaticCase`(kc L38)、`VehicleKcSweep`(vehicle_kc L32)、`HandlingManoeuvre`(handling L37)、`FourPostRide`(four_post L32)、`RandomRoadRide`(random_road L34)；axle_dynamic 直接使用既有 `AxleDynamicsModel`/`AxleDynamicsCase`。
- 四个 vehicle family 的准备模块在 `prepare_request` 内调用 `cases.vehicle_dynamic.prepare_vehicle_run` 完成整车装配，再调用既有 `cases.*` 文档 builder 生成 model/case document 与 payload；`vehicle_kc` 额外产出编译器所需的 `vehicle_assembly`/`wheels`。
- `VehicleKcSweep.times_s`（vehicle_kc L44）：K/C 扫掠窗口属于 sweep 自身（ramp 窗口是验收的一部分），未给定时退回整车运行输出网格。

### 修改

- `simulation/compiler.py`：新增 `_axle_dynamic_prepared`(L63)；`AxleDynamicCompiler`(L334) 改为消费 `axle_dynamic_prepared` 并把已授权的 document 请求交给 `DocumentPairCompiler`（document bypass 仍做 request identity 校验）。其余五个 compiler section 未改：它们本来就只从 context 读 `model_document`/`case_document`/`model_document_pair`/`vehicle_assembly`/`wheels`，现在这些键由 preparation 产出。
- `tests/cases/test_axle_dynamic_contract.py`：+2 测试（默认 registry 准备与 compiler 消费；document 请求 bypass 且仍校验 identity）。
- `tests/cases/kc_quasi_static/test_contract_boundary.py`：+2 测试（同上，含真实 K 原生运行）。
- `tests/cases/test_vehicle_kc.py`：+2 测试（domain 输入经 registry 端到端运行；document bypass 与 identity 校验）。
- `tests/cases/test_handling.py`、`tests/cases/test_ride_four_post.py`、`tests/cases/test_ride_random_road.py`：family 自身运行改为真实 domain 输入经默认 registry；显式参考改为 document 请求；各 +2 测试（准备类型/context/compiler 消费；bypass 与 identity 校验）。
- `tests/simulation/test_compilers.py`：`test_axle_dynamic_compiler_owns_document_and_blob_authoring` 改为 `test_axle_dynamic_compiler_frames_the_prepared_documents`，并新增 `test_axle_dynamic_compiler_carries_authored_documents_through`（该文件是这六个 compiler section 的既有契约测试）。

## 与规格的差异与承接事项（需 03/04 处理）

1. **三个扩展 family 测试的参考腿无法在 02 通过**：原测试用 `context={"prepared": ...}` 触发 `vehicle_dynamic` 的 legacy adapter，而该 adapter 位于 `preparation/vehicle_dynamic.py`（子任务 03）。02 无法写出该文件，故把参考腿改为同一物理量的 document 请求（`cases.vehicle_dynamic` 的 model/case document + `DocumentPairCompiler`，走 document bypass）。物理比对与容差未变，但 legacy `prepared` 键在这三个测试中不再被覆盖；该键的真实集成覆盖应由 03 负责。
2. **`prepare_vehicle_run` 依赖路径**：四个 vehicle family 的准备模块从 `cases.vehicle_dynamic` 延迟导入 `prepare_vehicle_run`（当前是旧模块的活体 re-export）。03 把该函数迁到 `preparation/vehicle_dynamic.py` 时，必须保留该 re-export 或同步改这四个模块的导入。
3. **`VehicleKcPrepared.assembly` 类型为 `object`**：上游 `_PreparedVehicleRun.assembly` 声明就是 `object`（vehicle_dynamics.py L155），为避免引入 cast，此处按上游类型标注；03 迁移后可收紧为 `VehicleAssembly`。
4. **vehicle_kc 的 driven joint 授权仍在 compiler**：document 请求会 bypass preparation，既有调用方传入的是未加 driven 坐标的源 model document pair，因此派生必须留在 compiler（矩阵也把 `vehicle_assembly`/`wheels` 列为 preparation 产出的 context keys）。compiler 不再自行装配整车，装配由 preparation 完成。
5. **rigid 整车上的 vehicle_kc 会被核条件拒绝**（rank deficient，既有测试注释已说明），新测试沿用 compliant 整车 fixture。
6. 未修改 `simulation/preparation.py`、`simulation/runner.py`、`simulation/__init__.py`、`tests/simulation/test_preparation.py`、`preparation/vehicle_dynamic.py`（不存在）、`results/*`、旧 `vehicle_dynamics.py` 及整车 service；未新增依赖。

## 验证记录（供主线程抄入父级 PROGRESS）

### 验证记录：子任务 02 / 六个 family 回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/cases/kc_quasi_static packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/cases/test_handling.py packages/suspension_multibody/tests/cases/test_ride_four_post.py packages/suspension_multibody/tests/cases/test_ride_random_road.py packages/suspension_multibody/tests/axle_dynamics/test_api.py -q`
- **退出码**：0
- **摘要**：63 passed；六个 family 的 registry key、准备类型、context 与 compiler 消费均有断言，含真实原生运行。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-family-tests.log

### 验证记录：子任务 02 / architecture
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q`
- **退出码**：0
- **摘要**：44 passed；依赖边界、统一入口和 ABI 门禁未回退。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-architecture.log

### 验证记录：子任务 02 / registry key 枚举
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody python -c "from suspension_multibody.simulation.preparation import default_preparation_registry; required={('axle','axle_dynamic'),('axle','kc_quasi_static'),('vehicle','vehicle_kc'),('vehicle','handling'),('vehicle','ride_four_post'),('vehicle','ride_random_road')}; keys=set(default_preparation_registry().keys()); assert required <= keys, (required-keys, keys)"`
- **退出码**：0
- **摘要**：六个非整车 family 的规范化 key 均在默认 registry 中。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-registry.log

### 验证记录：子任务 02 / axle-KC-vehicleKC 适配回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/cases/kc_quasi_static packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/axle_dynamics/test_api.py -q`
- **退出码**：0
- **摘要**：47 passed；axle_dynamic/kc_quasi_static/vehicle_kc 经默认 registry 编译且契约输出不变。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-row2-pytest.log

### 验证记录：子任务 02 / handling-four-post-random-road 适配回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_handling.py packages/suspension_multibody/tests/cases/test_ride_four_post.py packages/suspension_multibody/tests/cases/test_ride_random_road.py -q`
- **退出码**：0
- **摘要**：16 passed；三个扩展 family 经默认 registry 与显式参考一致（max diff < 1e-12）。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-row3-pytest.log

### 验证记录：子任务 02 / 步骤 4 命令链（含未完成的记录审计）
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/cases/kc_quasi_static packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/cases/test_handling.py packages/suspension_multibody/tests/cases/test_ride_four_post.py packages/suspension_multibody/tests/cases/test_ride_random_road.py packages/suspension_multibody/tests/axle_dynamics/test_api.py -q && uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q && python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --non-vehicle-matrix && python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --progress-records 02`
- **退出码**：1
- **摘要**：前三步通过（63 passed / 44 passed / non-vehicle-matrix pass）；末步 `--progress-records 02` 失败于 `AssertionError: 2`，因为父级 PROGRESS 尚无 `子任务 02` 记录（父级文件不在本子任务写范围）。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-row4-chain.log

### 验证记录：子任务 02 / non-vehicle-matrix
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02
- **命令**：`python .codex-tasks/20260920-unified-preparation-cutover/tasks/05-final-validation/planning_contract_scan.py --non-vehicle-matrix`
- **退出码**：0
- **摘要**：六个非整车 family 的矩阵行、字段与 legacy adapter 章节通过。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-non-vehicle-matrix.log

### 验证记录：子任务 02 / scoped ruff
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02 / scoped ruff
- **命令**：`uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests packages/suspension_multibody/scripts`
- **退出码**：0
- **摘要**：All checks passed（含新增 preparation 包与六个 family 测试）。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-ruff-full.log

### 验证记录：子任务 02 / scoped ty
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02 / scoped ty
- **命令**：`uv run --all-packages ty check packages/suspension_multibody/src/suspension_multibody/preparation packages/suspension_multibody/src/suspension_multibody/simulation/compiler.py`
- **退出码**：0
- **摘要**：All checks passed；同时 `uv run --all-packages ty check .` 退出码 0（证据 02-ty-full.log）。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-ty-scoped.log

### 验证记录：子任务 02 / simulation 与 compiler 契约回归
- **时间**：2026-09-20
- **子任务/门禁**：子任务 02 / simulation 契约回归
- **命令**：`uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation -q`
- **退出码**：0
- **摘要**：54 passed；含子任务 01 的协议/生命周期测试与 compiler 契约测试。
- **证据文件**：C:\Users\zzy11\.pi-desktop\scratch\5d217c5d-208c-4ebd-a11c-39e7f374b599\02-simulation.log

## 剩余问题

- 步骤 4 的父级 PROGRESS 记录（`子任务 02` 标签，需覆盖 4 条 pytest 命令）未写入，属主线程写范围；写入后 `--progress-records 02` 应通过。
- 本子任务未运行 `git diff --check`、compileall 与终局全量 pytest（属 04/05 门禁）；未提交 git。
- 若 03 移除 `cases.vehicle_dynamic.prepare_vehicle_run` 的 re-export，四个 vehicle family 准备模块需同步改导入（见承接事项 2）。

## 主线程验收

- 只读审查 aa1e1a09-c557-465c-b589-cd52d37ff9c9 确认六family迁移成立；发现axle document分支未启用contract校验。
- 主线程复现错误family被接受后，在AxleDynamicCompiler启用validate_contract_family=True，新增7种contract/version/kind/family错误回归；受影响axle+simulation+architecture合计117 passed，ruff与全仓ty通过。
- 父级PROGRESS证据已补齐；--progress-records 02退出码0，日志02-parent-records.log。原父级缺记录失败已关闭。
- 后续03须迁移四个vehicle preparation对cases.vehicle_dynamic.prepare_vehicle_run的依赖及四组family测试旧模块导入；04仍负责其余调用方和最终删除。
- 可选审查建议未扩范围：K/C模式与默认时间栅格沿用本次实现，真实legacy prepared与整车staged/facade单次准备由03验证。
