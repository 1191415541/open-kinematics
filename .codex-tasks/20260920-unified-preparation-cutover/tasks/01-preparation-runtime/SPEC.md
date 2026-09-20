# 子任务 01：统一 preparation 协议与 runner 生命周期

## 目标

在不改变现有 contract/compiler/backend/results 行为的前提下，建立统一的 preparation 协议和 registry，并固定显式阶段 `prepare_request() → family compiler → run_request(compiled)`；保留 `run_request(SimulationRequest)` 作为一次性便捷 facade。

## 交付范围

- `simulation/preparation.py`：定义 `PreparedSimulation`、准备器协议、`PreparationRegistry`、默认七 family 的延迟 import 组装、legacy adapter 调用和 `prepare_request()`。
- `simulation/runner.py`：支持 `run_request(SimulationRequest)` 的 prepare→compile→submit，以及 `run_request(CompiledSimulation)` 的 submission stage；保留 `run_compiled()` 兼容入口。
- `simulation/__init__.py`：导出必要的统一入口；不承载 family 物理逻辑。
- `tests/simulation/test_preparation.py`：用可注入的 `PreparationRegistry`/`Preparation` 替身和计数 backend 覆盖 registry key 规范化、七个默认 key 的延迟注册枚举、上下文注入、document bypass、document + legacy `prepared` bypass 优先、legacy adapter 包装、匹配 `prepared_simulation` 单次复用、不匹配/stale prepared 重新 preparation、compiled submission 和错误传播；测试不得依赖子任务 02/03 尚未实现的真实 family 模块。

## 冻结行为

- registry key 使用 `str(value).strip().casefold()` 规范化；空的 assembly/family 抛出 `ValueError`，未注册键抛出包含规范化 key 的 `KeyError`。
- document request 不查 preparation registry，也不调用 legacy adapter，只执行 compiler contract 校验；document bypass 优先于其它 preparation 路由。
- preparation 返回的 context 与原 request context 合并，preparation context 覆盖同名键。
- vehicle dynamic 的旧 `prepared` 仅由 `preparation.vehicle_dynamic.adapt_legacy_prepared_request()` 包装为 `prepared_simulation` 后进入统一匹配，不得由 simulation 之外的 compiler 直接消费。
- `prepared_simulation` 只有在值为 `PreparedSimulation`、规范化 assembly/family 相同，且 `prepared.request.model is request.model`、`prepared.request.case is request.case` 时才可复用；其它情况重新 preparation。
- 默认 registry 通过固定的延迟 import 表按七个规范化 key 登记 family：key 枚举、document bypass 和 `prepared_simulation` 复用检查不得 import family 模块，只有对某个 key 执行 family preparation 时才 import 对应模块。
- 未注册 key、未实现的 family 模块或 adapter 缺失必须原样抛出可诊断的 `KeyError`/`ImportError`/`ModuleNotFoundError`；不得捕获并改写、吞掉或以占位实现回退，也不得在生产代码中新增占位 family 模块。

## 约束

- 不把 family-specific 物理语义写进 simulation 层。
- 不调用 native、不解码结果、不计算 metrics。
- 不改变 `SimulationRun`、`CompiledSimulation` 和现有 document 请求行为。
- preparation 结果必须可被 compiler 消费，并能在测试中独立检查。

## 验收标准

1. 领域 `SimulationRequest` 可以经 `prepare_request()` 产生显式准备上下文。
2. 显式 staged path 和 `run_request(SimulationRequest)` facade 都只执行一次 preparation；`run_request(CompiledSimulation)` 不重复 preparation 或 compile。
3. 已有 document 请求不执行 family preparation 或 legacy adapter；未注册 family、请求身份错误和 preparation 失败均保留可诊断异常。
4. legacy `prepared` 的 adapter、document bypass 优先级、包装后的匹配复用和 stale identity 重新 preparation 均由测试直接断言。
5. 现有 simulation/compiler/architecture 测试通过，新增测试能证明 preparation、compiler、submission 和 `results.decoder` 边界。
6. 本子任务的全部验证命令在六个非整车 family 模块和 `preparation/vehicle_dynamic.py` 尚不存在时即可独立通过：默认 registry 能枚举起七个规范化 key，协议、包装、bypass 优先级、匹配与 stale 行为由可注入替身断言，不依赖真实 family 模块。
7. 测试与生产代码不捕获或改写 `ImportError`/`ModuleNotFoundError`，不引入占位 family；真实 family 与 legacy adapter 的集成验证由子任务 02/03 提供，终局门禁不得以替身覆盖替代它们。

## 验证

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run --package suspension-multibody python -c "from suspension_multibody.simulation.preparation import default_preparation_registry; required={('axle','axle_dynamic'),('axle','kc_quasi_static'),('vehicle','vehicle_kc'),('vehicle','handling'),('vehicle','ride_four_post'),('vehicle','ride_random_road'),('vehicle','vehicle_dynamic')}; keys=set(default_preparation_registry().keys()); assert keys==required, keys^required"
```

## 阶段验证策略

- 本子任务完成时 `preparation/` 包中的六个非整车 family 模块和 `preparation/vehicle_dynamic.py` 尚不存在，因此本任务的阶段门禁是 `tests/simulation` + `tests/architecture`，用替身证明协议与生命周期。
- 现有 `tests/cases`、`tests/results` 中以领域对象调用 `run_request(SimulationRequest)` 的用例（如 axle dynamic、K/C quasi-static）需要对应 family 模块，属于子任务 02/03 的门禁范围；本任务不得为了让它们通过而添加占位 family、吞掉 import 错误或跳过用例。
- 子任务 02/03 必须用真实 family preparation 与 legacy adapter 复跑相关领域路径；终局 04/05 门禁必须在真实模块存在时运行全量专项测试，替身覆盖不得成为任何 family 集成的唯一证据。
