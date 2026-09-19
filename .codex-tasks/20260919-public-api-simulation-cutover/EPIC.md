# Epic：Public API / CLI 全链路切换到统一仿真架构

- **任务编号**：20260919-public-api-simulation-cutover
- **创建日期**：2026-09-19
- **状态**：IN_PROGRESS（任务 01–07 已完成，任务 08 待执行）
- **范围**：`packages/suspension_multibody` 及其直接 CLI、Adams、脚本和测试入口
- **形态**：Epic
- **前置条件**：`simulation/`、`results/`、`metrics/` 基础设施已落地；`native_kc/` 已迁移并删除；现有 contract parity、K/C probe、动态/整车基础回归可运行。

## 目标

把所有对外仿真入口完全收敛到以下唯一生产链路。`ContractRun` 是 backend 与结果层之间的内部 kernel transport，不是公共入口层的新分支：

```text
Public API / CLI
        ↓
SimulationRequest
        ↓
cases/<family> compiler
        ↓
simulation.run_request()
        ↓
NativeContractBackend
        ↓
ContractRun                 (internal transport)
        ↓
RawContractResult
        ↓
results.decoder
        ↓
AxleResult / VehicleResult
        ↓
metrics
        ↓
统一 artifact IO
```

时域/回放工况在同一链路上按时间样本重复执行或消费 contract 输出；其聚合结果使用 `results.timeseries.TimeSeriesResult`，而不是重新创建旧的 `schema.DynamicResultBundle`：

```text
每个时间样本：RawContractResult → results.decoder → AxleResult / VehicleResult
聚合结果：TimeSeriesResult(times_s, samples, diagnostics, metrics, provenance)
```

完成后：

1. Public API、CLI、Adams 和外部脚本只构造 `SimulationRequest` 并调用统一运行服务。
2. 每个工况族保留独立 compiler 和独立物理语义，但 compiler 不提交 native、不解码结果、不计算指标。
3. `kernel.run_contract()` 的生产唯一调用点是 `NativeContractBackend`。
4. 所有正式 contract 输出先经过 `RawContractResult`，再由 `results.decoder` 组装为 `AxleResult` 或 `VehicleResult`。
5. 时域/回放结果由 `TimeSeriesResult` 聚合，旧 `DynamicResultBundle` 只保留历史读取兼容。
6. 所有派生指标只从正式结果对象计算，并通过 `metrics/` 暴露。
7. 成功、partial、failed 结果使用同一套可追溯 artifact IO 协议。
8. 旧入口、重复解码、重复 writer 和兼容 facade 只有在全部删除门槛通过后才移除。

## 硬性架构不变量

### 入口层

Public API、CLI、Adams 和脚本只允许负责：

- 读取、校验和规范化用户输入；
- 构造 `SimulationRequest`；
- 选择工况族和输出选项；
- 调用统一 service；
- 展示 `AxleResult`、`VehicleResult`、`TimeSeriesResult`、指标和诊断；
- 请求统一 artifact 写出。

入口层禁止直接调用：

- `kernel.run_contract()`；
- `cases.*.run_*_contract()`；
- `results.raw.decode_contract_run()`；
- `axle_dynamics.native`；
- `vehicle_dynamics.py` 或 `axle_dynamics/io.py` 内部 writer；
- 任何 native block 的手工解码函数。

### 工况 compiler 层

每个 `cases/<family>` compiler 只负责：

- 将领域模型和工况输入编译为 model/case contract document；
- 生成 payload、layout、metadata 和工况专用配置；
- 保留 K&C、操稳、平顺、轴动态、整车动态等独立物理语义；
- 返回统一 `CompiledSimulation` 所需的编译结果。

compiler 禁止：

- 加载 DLL 或调用 native ABI；
- 直接调用 `kernel.run_contract()`；
- 将 native 输出解码为领域结果；
- 计算指标或写出 artifact。

### simulation 层

`simulation/` 只负责：

- 定义和校验 `SimulationRequest`；
- 注册、查找和调用 compiler；
- 生成 `CompiledSimulation`；
- 通过 `NativeContractBackend` 提交 native contract；
- 将 backend 的 `ContractRun` 交给结果层生成 `RawContractResult`；
- 提供 `run_request()`、`run_compiled()` 等统一服务。

不得新增第二套 native solver，不得理解工况专用物理语义。

### results 层

`results/` 只负责：

- 从 `ContractRun` 生成 `RawContractResult`；
- 解码公共时间网格、body、tire、constraint、element、diagnostics、performance；
- 按 assembly 组装 `AxleResult` / `VehicleResult`；
- 用 `TimeSeriesResult` 聚合时域/回放样本；
- 明确表达 unavailable、not-applicable、partial 和 failed 状态；
- 保留现有通道名、顺序、单位和 manifest hash。

`DynamicResultBundle` 不再是新生产结果类型：`schema.loader.load_dynamic_result()` 的历史读取兼容可以暂留，新的公共 API、Adams 和 IO 不得创建它。

### metrics 层

`metrics/` 只负责从正式结果对象计算派生量：

- `metrics.common`：跨工况通用指标；
- `metrics.axle`：整轴指标；
- `metrics.vehicle`：整车指标；
- `metrics.case_specific`：工况专用指标注册和分派。

K&C 指标从 `analysis/metrics.py::compute_k_metrics` 迁移到 `metrics/` 的注册实现；`analysis/vehicle_physics.py` 中的物理求解保留，但其派生 wheel-load 指标迁入 `metrics.vehicle`。每个工况族必须注册明确的 case-specific implementation；当前没有专用派生量的工况使用显式 `not_applicable` 实现，不得因未注册而静默跳过。

指标层不得复制 native block、事实时间序列或替代结果对象。

### 统一 artifact IO 层

统一 writer 的唯一 owner 是 `suspension_multibody.io.artifacts`，由任务 06 在切换入口前先建立并接通：

- `write_artifact(...)`：接收正式结果对象、metrics、request/provenance 和 status；
- `read_artifact(...)`：读取统一 manifest、arrays、diagnostics、performance 和 failure evidence；
- manifest、模型/工况 document 和 hash；
- channel registry、layout、数组和时间网格；
- diagnostics、status、failure evidence 和 partial evidence；
- native build metadata、solver settings、performance；
- `AxleResult` / `VehicleResult` / `TimeSeriesResult` 和 metrics 的序列化；
- 成功、partial、failed 三类结果的同构协议。

旧 `io/results.py`、`axle_dynamics/io.py`、`vehicle_dynamics.py` writer 在迁移期只能保留历史读取或兼容转发，不得继续产生新的非统一格式。

## 迁移范围

### 必须迁移的入口

- `suspension_multibody.api.run_case()`；
- `suspension_multibody.api.run_dynamic_case()`；
- `suspension_multibody.api._run_axle_quasi_static()` 及 K/C replay 胶水；
- `suspension_multibody.vehicle_dynamics.run_vehicle_dynamics()`；
- `suspension_multibody.axle_dynamics.contract_run.run_axle_dynamics()`；
- 顶层 `__init__.py` 导出；
- `cli.py` 的整轴、整车命令和输出路径；
- Adams 全部入口，包括 `time_domain_gate.py`、`vehicle_kc_time_domain.py`、`axle_equivalence.py`、`reference.py`、`strict_k.py`、`strict_c.py`；
- probes、性能门、parity 脚本和以下已知脚本调用方：
  - `scripts/run_native_tire_rig.py`；
  - `scripts/measure_native_fiala_solver_time.py`；
  - `scripts/diagnose_native_initial_pac_moments.py`；
  - `scripts/run_dynamic_kc_correlation.py`；
  - `scripts/run_real_adams_car_benchmark.py`；
  - `scripts/run_full_native_three_model_comparison.py`；
  - `scripts/case_parity_check.py`。

### 必须接入生产链路的现有模块

- `simulation/request.py`；
- `simulation/compiler.py`；
- `simulation/runner.py`；
- `simulation/backend.py`；
- `simulation/dispatch.py`；
- `results/raw.py`；
- `results/decoder.py`；
- `results/timeseries.py`（新增）；
- `results/axle.py`；
- `results/vehicle.py`；
- `metrics/common.py`；
- `metrics/axle.py`；
- `metrics/vehicle.py`；
- `metrics/case_specific.py`；
- `io/artifacts.py`（新增，统一 writer owner）。

### 工况族

保持以下工况族的独立 compiler 和物理语义，不合并为单一 compiler：

- `kc_quasi_static`；
- `axle_dynamic`；
- `vehicle_kc`；
- `vehicle_dynamic`；
- `handling`；
- `ride_four_post`；
- `ride_random_road`。

## 分阶段实施

### 阶段 1：边界盘点与架构门禁

建立生产调用图、符号矩阵、入口矩阵、删除候选清单和已知违规基线；新增审计式架构门禁。任务 01 先允许当前已知旧路径进入精确 allowlist，但必须禁止新增旁路；后续每个任务都要运行该门禁并删除已迁移条目。allowlist 以可提交的 `tasks/01-boundary-inventory/LEGACY_ALLOWLIST.toml` 为唯一真源，最终严格模式只允许 `NativeContractBackend` 的 native 调用和明确的历史读取兼容。

产物固定为：

- `tasks/01-boundary-inventory/BOUNDARY.md`；
- `tasks/01-boundary-inventory/SYMBOL_MATRIX.csv`；
- `tasks/01-boundary-inventory/LEGACY_ALLOWLIST.toml`。
迁移记录固定为：

- `MIGRATION_NOTES.md`：记录 `run_dynamic_case()` 返回类型变化、`DynamicResultBundle` 历史读取边界、公开异常/属性兼容和版本策略。
- `tasks/01-boundary-inventory/LEGACY_ALLOWLIST.toml`：架构门禁唯一可提交真源；每条例外必须是文件+符号级、绑定 owner、删除任务、删除条件和验证证据。

### 阶段 2：compiler / runner 全面切换

将所有工况族统一接入 `SimulationRequest → compiler → simulation.run_request()`。保留的 `run_*_contract()` 只能是薄兼容 wrapper，内部不得自行提交 native。

### 阶段 3：results decoder 与时域结果接线

把所有 native 输出先转为 `RawContractResult`，再由 `results.decoder` 分派到 `AxleResult` / `VehicleResult`；任务 03 同时落地 `TimeSeriesResult`、时域/replay 聚合协议，并固定 `run_dynamic_case()` 的正式返回契约。删除入口内部的重复 block 读取、时间网格拼接和领域结果装配。

### 阶段 4：整轴入口迁移

只拆分整轴内部 service：`axle_dynamics/contract_run.py` 负责兼容输入适配和 service 转发，执行进入 simulation，事实结果进入 results，派生量进入 metrics，错误和诊断统一由结果层保留。Public API、CLI、Adams 调用方统一在后续入口阶段切换。

### 阶段 5：整车入口迁移

拆分 `vehicle_dynamics.py`：保留必要的领域 preparation 和兼容公共对象，迁移 compiler、decoder、metrics 和 artifact 调用；最终不再由该文件承载唯一执行、解码或写出逻辑。

### 阶段 6：统一 artifact owner 与 Public API / CLI / Adams / IO 全面切换

先建立 `io/artifacts.py` 和其 manifest/status 协议，再切换顶层 API、CLI、Adams、脚本和所有 IO 调用方。`run_dynamic_case()` 的正式返回契约由任务 03 固定为 `TimeSeriesResult`；任务 06 只迁移调用方、展示、序列化和历史 bundle 读取适配。Adams 新路径消费 `history_from_result_series()`，`history_from_dynamic_bundle()` 只作为历史兼容适配器。

### 阶段 7：metrics 与 artifact IO 收口

让 common、axle、vehicle、case-specific metrics 在成功和 partial 生产路径中实际执行；迁移 K&C、vehicle physics 派生指标和各工况族注册；停止产生旧 bundle 和重复 artifact，补齐统一 writer 的重读回归。

### 阶段 8：兼容代码删除与最终门禁

在全部生产、测试、脚本、CLI、Adams 和文档引用切换后，删除兼容 facade、旧入口、重复解码和重复 writer；重写仍断言 `axle_dynamics.native` 存在的架构测试，运行全套回归与全仓残留扫描。

## 子任务依赖

```text
01 boundary inventory + baseline allowlist
        ↓
02 compiler cutover
        ↓
03 results decoder + TimeSeriesResult
       ↙ ↘
04 axle service cutover   05 vehicle service cutover
       ↘                   ↙
06 artifact owner + API/CLI/Adams/IO cutover
        ↓
07 metrics/artifact cleanup
        ↓
08 final deletion gates
```

## 删除门槛

任何兼容代码、旧入口或重复实现必须同时满足以下条件后才允许删除：

1. `kernel.run_contract()` 的生产唯一调用点是 `NativeContractBackend`。
2. 生产代码不再直接调用 `cases.*.run_*_contract()`。
3. Public API、CLI、Adams 和脚本只调用统一 service。
4. `results.decoder`、`results.axle`、`results.vehicle` 和 `results.timeseries` 均被生产路径实际使用。
5. `metrics.common`、`metrics.axle`、`metrics.vehicle` 和 `metrics.case_specific` 均有生产调用证据。
6. `axle_dynamics.native` 无生产、测试和脚本引用；保留的历史读取兼容不得导入它。
7. `vehicle_dynamics.py` 不再承载唯一 preparation、decode 或 artifact 逻辑。
8. `schema.DynamicResultBundle` 不再被新生产路径创建；若保留读取兼容，必须明确标注为历史格式。
9. `io/results.py`、`axle_dynamics/io.py`、`vehicle_dynamics.py` 不再产生非统一 artifact；新写出唯一 owner 是 `io.artifacts`。
10. 成功、partial、failed 结果均能写出并重读 manifest、数组、diagnostics、performance 和 failure evidence。
11. CLI、Adams 门、probe、性能门、contract parity、结果快照、时域/动态和整车回归全部通过。
12. 任务 01 的架构门禁切换为 strict mode；allowlist 只剩允许的 backend 调用和明确的历史读取兼容。
13. scoped `ruff check`、`uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests scripts`、`uv run --all-packages ty check .`、`git diff --check` 全部通过。
14. 全仓搜索无旧入口导入、无死兼容符号、无重复解码、无重复 writer、无失效文档命令。
15. 对外 API 返回类型变化、DynamicResultBundle 历史读取边界和兼容异常变更已在迁移文档和版本策略中记录。

## 明确不在范围内

- 不修改 C++ native 物理内核、ABI、契约版本或物理方程；
- 不新增第二套 native 求解器；
- 不把不同工况的物理语义合并为一个 compiler；
- 不把 K&C、操稳、平顺专用指标塞入 common metrics；
- 不把显式多体整车模型替换成传统 14/15-DOF 降阶模型；
- 不修改 `axle_channels.yaml` 的通道名、顺序、单位和 manifest hash；
- 不在验证完成前删除公开入口或兼容异常类型；
- 不为了迁移顺手重命名无关模块、重写 native 物理或扩展产品功能。

## 全局验收

- 所有子任务状态为 `DONE`；
- contract parity、结果快照、动态/时域工况、整车工况、K/C probe、Adams 门和性能门通过；
- 成功、partial、failed artifact 回归和重读通过；
- `uv run --all-packages ruff check packages/suspension_multibody/src packages/suspension_multibody/tests` 通过；
- `uv run --all-packages python -m compileall -q packages/suspension_multibody/src packages/suspension_multibody/tests scripts` 覆盖本次修改涉及的 Python 路径；
- `uv run --all-packages ty check .` 通过；
- `git diff --check` 通过；
- 全仓搜索确认旧入口、兼容 facade、重复解码、重复 writer 和失效文档命令均已清理或被明确列入历史读取兼容；
- 所有迁移结果、删除理由、allowlist 变更和验证命令记录在对应 `PROGRESS.md`。

## 恢复信息

- **任务**：将 Public API / CLI / Adams / IO 全面切换到统一 simulation/results/metrics/artifact 架构
- **形态**：epic
- **进度**：7/8 子任务完成
- **当前**：任务 01–07 已完成；任务 08 依赖已满足，尚未开始
- **文件**：`.codex-tasks/20260919-public-api-simulation-cutover/tasks/08-final-deletion-gates/`
- **下一步**：执行兼容代码删除、全仓残留扫描和最终 strict 门禁
