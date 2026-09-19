# 子任务 02：compiler 与统一 runner 全面切换

## 目标
让所有工况族通过 `SimulationRequest → cases/<family> compiler → simulation.run_request()` 执行；native 提交唯一收敛到 `NativeContractBackend`。

## 范围
- `simulation/request.py`、`compiler.py`、`runner.py`、`backend.py`、`dispatch.py`
- `cases/kc_quasi_static/`
- `cases/axle_dynamic.py`
- `cases/vehicle_kc.py`
- `cases/vehicle_dynamic.py`
- `cases/handling.py`
- `cases/ride_four_post.py`
- `cases/ride_random_road.py`
- 仍被调用的 `run_*_contract()` 兼容 wrapper

## 必须完成
1. 为每个工况族定义明确的 request kind、payload schema、compiler 和 assembly 类型。
2. compiler 只生成 model/case document、payload、layout、metadata 和 `CompiledSimulation`。
3. `simulation.run_request()` 统一完成 compiler 分派、backend 提交和 `RawContractResult` 包装。
4. `NativeContractBackend` 成为生产唯一 `kernel.run_contract()` 调用点。
5. 旧 `run_*_contract()` 若暂留，必须只构造 request 或转发统一 service，不得直接提交 native。
6. 统一错误、失败状态、诊断和 performance 的传播，不丢失原有异常语义。

## 约束
- 不新增第二套 native solver。
- 不在 compiler 中解码结果、算指标或写 artifact。
- 不合并 K&C、操稳、平顺、轴动态和整车动态的物理语义。
- 保持 contract document、payload、layout、单位和时间约定不变。

## 验收
- 所有工况族均能由 `SimulationRequest` 进入统一 runner。
- 生产代码中 `kernel.run_contract()` 仅出现在 `NativeContractBackend`。
- compiler 单测覆盖 document、payload、layout、metadata 和错误输入。
- 兼容 wrapper 的行为与旧公共入口一致。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/cases -q`
