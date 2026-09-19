# 子任务 03：RawContractResult、results.decoder 与 TimeSeriesResult 接线

## 目标
让所有正式 native 输出先进入 `RawContractResult`，再由统一 decoder 组装为 `AxleResult` 或 `VehicleResult`；为 replay/time-domain 增加 `TimeSeriesResult`，停止新生产路径创建旧 `DynamicResultBundle`。

## 范围
- `results/raw.py`
- `results/decoder.py`
- `results/axle.py`
- `results/vehicle.py`
- 新增 `results/timeseries.py`
- `results/channels.py`
- `analysis/vehicle_kc_time_domain.py` 的结果构造协议
- `schema/dynamic.py`、`schema/loader.py` 的历史读取兼容边界
- 结果快照、动态/时域、失败和 partial fixture

## 必须完成
1. `ContractRun` 到 `RawContractResult` 的转换保留 document、named blocks、time grid、body/tire metadata、diagnostics 和 performance。
2. decoder 根据 assembly type 组装 `AxleResult` / `VehicleResult`。
3. 新增不可变 `TimeSeriesResult`，至少保留 `times_s`、按样本的正式结果对象、聚合 diagnostics、metrics、provenance、status 和 failure/partial evidence。
4. `analysis/vehicle_kc_time_domain.py` 和 quasi-static replay 的聚合输出改为 `TimeSeriesResult` 所需的样本协议，并在本任务完成时固定 `api.run_dynamic_case()` 的正式返回契约为 `TimeSeriesResult`；任务 06 只迁移调用方、展示、序列化和历史读取适配。
5. `DynamicResultBundle` 仅保留历史 loader/adapter 读取兼容；任务 03 完成后不得新增生产创建点，也不得通过兼容适配器继续产生新 bundle。
6. 所有事实数据保留在 results 层；指标不复制 native block。
7. 保留 `AxleDynamicsResult`、`VehicleDynamicsResult` 的兼容包装、属性转发和 `steering_output` 语义。
8. 保留 unavailable、not-applicable、partial、failed 的明确状态和错误证据。
9. 保持 `axle_channels.yaml` 的通道名、顺序、单位和 manifest hash 不变。
10. 对成功、partial、failed、诊断、performance、动态样本和 TimeSeriesResult 建立结果快照回归。

## 约束
- 不在 Public API、CLI、Adams 或工况 compiler 中读取 native block。
- 不引入第二套结果 schema；`TimeSeriesResult` 是 results 层对多样本正式结果的聚合，不替代单样本 contract schema。
- 不把缺失数据静默填成零或空数组。

## 责任边界
- 任务 03 负责结果类型、解码器、时域聚合协议，并固定 `run_dynamic_case()` 的正式返回类型和新生产禁用 `DynamicResultBundle` 的边界。
- 任务 04/05 只负责整轴/整车 service 接入这些结果类型，不重复实现 decoder。
- 任务 06 只负责公共 API、Adams、CLI、脚本和 IO 调用方切换；不重新定义 `run_dynamic_case()` 返回类型。

## 验收
- 生产路径实际调用 `results.decoder`、`results.axle`、`results.vehicle`、`results.timeseries`。
- 结果对象能覆盖整轴和显式多体整车，不误称为 14/15-DOF 降阶结果。
- `DynamicResultBundle` 新建路径清零，历史读取兼容边界有测试。
- 兼容异常和公开属性回归通过。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/contract packages/suspension_multibody/tests/analysis/test_axle_dynamic.py packages/suspension_multibody/tests/analysis/test_vehicle_dynamic.py -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`
