# 子任务 06：统一 artifact owner 与 Public API / CLI / Adams / IO 全面切换

## 目标
先建立 `suspension_multibody.io.artifacts` 作为唯一新 artifact writer/reader，再将所有对外入口统一切换到 `SimulationRequest → simulation.run_request()`、正式结果对象和统一 artifact IO。

## 范围
- 新增 `io/artifacts.py` 及 `io/__init__.py` 导出
- 顶层 `suspension_multibody/__init__.py`
- `api.py`，包括 `run_case()`、`run_dynamic_case()`、quasi-static replay 胶水和 K/C replay 路径
- `cli.py` 的整轴/整车命令和输出路径
- Adams 全部相关入口：`time_domain_gate.py`、`vehicle_kc_time_domain.py`、`axle_equivalence.py`、`reference.py`、`strict_k.py`、`strict_c.py`
- `adams/time_domain.py` 的 `history_from_result_series()` 新适配及旧 `history_from_dynamic_bundle()` 历史兼容
- probes、性能门、parity 脚本和已知脚本调用方：`run_native_tire_rig.py`、`measure_native_fiala_solver_time.py`、`diagnose_native_initial_pac_moments.py`、`run_dynamic_kc_correlation.py`、`run_real_adams_car_benchmark.py`、`run_full_native_three_model_comparison.py`、`case_parity_check.py`
- `io/results.py`、`axle_dynamics/io.py`、`vehicle_dynamics.py` 的旧 writer 调用方
- API、CLI、Adams、IO 回归测试和文档命令

## 必须完成
1. `io/artifacts.py` 提供唯一新写出/读取 API：`write_artifact(...)`、`read_artifact(...)`，覆盖 manifest、status、model/case hash、channel/layout、arrays、time grid、diagnostics、performance、metrics 和 failure/partial evidence。
2. `run_case()`、`run_dynamic_case()` 和 replay 胶水只负责输入校验、构造 request、调用统一 service、选择结果展示和输出；`run_dynamic_case()` 在任务 03 完成后已经返回 `TimeSeriesResult`，本任务不得重新定义该契约。
3. CLI 只调用正式公共 service 和 `io.artifacts`，不直接调用 kernel、case contract runner、native facade、底层 decoder 或旧 writer。
4. Adams 新路径消费 `AxleResult`、`VehicleResult` 或 `TimeSeriesResult`；`history_from_dynamic_bundle()` 只作为历史读取/兼容适配器。
5. `adams/axle_equivalence.py`、所有已知脚本和 parity/probe/performance 入口完成切换，保留各自物理语义和参数。
6. 顶层导出切换到正式入口和结果对象；兼容导出在删除门槛前保持可用但不得产生第二条 native 链路。
7. 旧 `write_bundle()`、`write_dynamic_bundle()`、`write_axle_dynamics_artifact()`、`write_vehicle_dynamics_artifact()` 不再被新生产入口调用；如保留，只能显式转发统一 writer 或作为历史读取兼容。
8. 更新测试 fixture、导入路径、文档命令和架构门禁。

## 约束
- Public API / CLI / Adams 不得直接调用 `kernel.run_contract()`。
- 不改变现有公开参数含义、异常类型和 K/C、动态、整车输入语义；`run_dynamic_case()` 返回类型变化由任务 03 固定，并由本任务补齐迁移文档、版本策略和所有调用方。
- 不把 Adams 或 CLI 特有展示逻辑塞入 compiler、results 或 metrics。

## 验收
- 全部对外入口的调用图符合统一生产链路。
- API、CLI、Adams、probe、性能门和脚本无旧 native 旁路。
- `io.artifacts` 是唯一新 artifact writer/reader owner，且入口实际调用。
- 公开导出、命令、历史 bundle 适配器和文档示例与实际路径一致。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/cli packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/io -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`
