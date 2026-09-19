# 统一仿真架构迁移记录

## 迁移目标

所有 Public API、CLI、Adams 和脚本统一进入：

```text
Public API / CLI / Adams / scripts
        ↓
SimulationRequest
        ↓
cases/<family> compiler
        ↓
simulation.run_request()
        ↓
NativeContractBackend
        ↓
ContractRun（内部 kernel transport）
        ↓
RawContractResult
        ↓
results.decoder
        ↓
AxleResult / VehicleResult
        ↓
metrics
        ↓
io.artifacts
```

时域与 replay 使用 `TimeSeriesResult` 聚合单样本正式结果；不再由新生产路径创建 `schema.DynamicResultBundle`。

## 已固定的责任边界

- `cases/<family>` 只编译 model/case document、payload、layout 和 metadata，不提交 native、不解码、不计算指标、不写 artifact。
- `simulation` 只负责 request 校验、compiler 注册、统一 runner 和 `NativeContractBackend` 提交。
- `results` 负责 `ContractRun → RawContractResult → AxleResult / VehicleResult`，以及 `TimeSeriesResult` 聚合。
- `metrics` 只从正式结果对象计算派生量，不复制 native block、事实时间序列或对象维度事实数据。
- `io.artifacts` 是唯一新 artifact writer/reader owner，覆盖成功、partial、failed 三种状态。
- `kernel.run_contract()` 的生产唯一调用点是 `NativeContractBackend`。

## 动态返回类型切换

- 任务 03 负责落地 `TimeSeriesResult`、时域/replay 样本协议，并固定 `api.run_dynamic_case()` 的正式返回类型为 `TimeSeriesResult`。
- 任务 03 完成后，新生产路径不得创建 `DynamicResultBundle`；仅允许历史 loader/adapter 读取旧格式。
- 任务 06 不重新定义返回类型，只迁移 API、CLI、Adams、脚本和 IO 调用方，补齐展示、序列化、文档和版本策略。
- Adams 新路径使用 `history_from_result_series()`；`history_from_dynamic_bundle()` 只处理历史输入。

- 任务 08 已删除 `axle_dynamics.native` 兼容 facade、旧 writer 和旧 K&C contract runner；公开的 `NativeKernelUnavailableError` / `native_build_metadata` 仍由 `kernel.native` 转发。
- 保留 `schema.DynamicResultBundle`、`schema.load_dynamic_result()` 和 `adams.time_domain.history_from_dynamic_bundle()` 仅用于历史 artifact 读取；删除条件是历史 artifact 读取验证、发布兼容窗口和所有外部调用方迁移均完成后，再单独移除该适配器。
- 保留旧 artifact 表格读取 helper（`read_table`、`canonical_hash`、`META_KEY`）作为读取/哈希兼容，不承担新格式写出。
- 不修改 C++ native、ABI、contract version、物理方程、通道顺序、单位或 manifest hash。

## 版本与发布记录

`run_dynamic_case()` 返回类型从历史 `DynamicResultBundle` 切换为 `TimeSeriesResult`，属于对外契约变化。实施时必须在发布说明、类型导出和调用方迁移文档中明确记录；旧 bundle 只作为历史读取格式，不作为新 API 的返回兼容层。

## 验收证据

每个子任务完成后，将以下内容写入对应 `PROGRESS.md`：

1. 变更文件和责任边界；
2. 实际调用图或静态扫描证据；
3. 专项测试、回归测试和门禁命令；
4. allowlist 删除条目及其删除依据；
5. 失败、partial、diagnostics、performance 和 artifact 重读证据。
