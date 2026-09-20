# 子任务 01：统一 preparation 协议与 runner 生命周期

## 目标

在不改变现有 contract/compiler/backend/results 行为的前提下，建立统一的 preparation 协议和 registry，并固定显式阶段 `prepare_request() → family compiler → run_request(compiled)`；保留 `run_request(SimulationRequest)` 作为一次性便捷 facade。

## 交付范围

- `simulation/preparation.py`：定义 `PreparedSimulation`、准备器协议、`PreparationRegistry`、默认七 family 的延迟 import 组装、legacy adapter 调用和 `prepare_request()`。
- `simulation/runner.py`：支持 `run_request(SimulationRequest)` 的 prepare→compile→submit，以及 `run_request(CompiledSimulation)` 的 submission stage；保留 `run_compiled()` 兼容入口。
- `simulation/__init__.py`：导出必要的统一入口；不承载 family 物理逻辑。
- `tests/simulation/test_preparation.py`：覆盖 registry key 规范化、七个默认 key、上下文注入、document bypass、document + legacy `prepared` bypass 优先、legacy adapter 包装、匹配 `prepared_simulation` 单次复用、不匹配/stale prepared 重新 preparation、compiled submission 和错误传播。

## 冻结行为

- registry key 使用 `str(value).strip().casefold()` 规范化；空的 assembly/family 抛出 `ValueError`，未注册键抛出包含规范化 key 的 `KeyError`。
- document request 不查 preparation registry，也不调用 legacy adapter，只执行 compiler contract 校验；document bypass 优先于其它 preparation 路由。
- preparation 返回的 context 与原 request context 合并，preparation context 覆盖同名键。
- vehicle dynamic 的旧 `prepared` 仅由 `preparation.vehicle_dynamic.adapt_legacy_prepared_request()` 包装为 `prepared_simulation` 后进入统一匹配，不得由 simulation 之外的 compiler 直接消费。
- `prepared_simulation` 只有在值为 `PreparedSimulation`、规范化 assembly/family 相同，且 `prepared.request.model is request.model`、`prepared.request.case is request.case` 时才可复用；其它情况重新 preparation。
- family modules 只提供 preparation 实现和其显式迁移 adapter，不在 import 时修改共享 registry；默认 registry 的唯一组装点是 `simulation/preparation.py`。

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

## 验证

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/simulation -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
```
