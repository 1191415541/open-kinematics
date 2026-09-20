# 子任务 03：拆分 vehicle_dynamics 职责

## 目标

将 `vehicle_dynamics.py` 中的整车 preparation、结果类型/映射和高层运行包装拆到明确模块，同时把 `("vehicle", "vehicle_dynamic")` 接入统一 preparation registry，保持现有整车结果、失败证据、metrics、artifact 和公开兼容语义。

## 交付范围

- `preparation/vehicle_dynamic.py`：迁移整车 preparation 数据结构、装配前处理、单位/输入/solver 映射及 compiler 所需上下文，导出 `PreparedVehicleRun`/`prepare_vehicle_run`，并提供唯一的 `adapt_legacy_prepared_request`，供中央 registry/lifecycle 延迟加载。
- `results/vehicle.py`：迁移 `VehicleDynamicsResult`、整车结果映射和兼容属性；提供 `decode_vehicle_result()` family adapter，不重新定义统一 `decode_result()`。
- `results/decoder.py`：把整车分派接线到 `results.vehicle.decode_vehicle_result()`，清理旧 `vehicle_dynamics` 导入；统一 `decode_result()` 定义保持唯一并仍只留在 `results/decoder.py`。
- `vehicle/service.py`：迁移 `run_vehicle_dynamics()` 的统一 request 调用、partial/failed 处理、metrics 和静态轮荷补充。
- `cases/vehicle_dynamic.py`、`simulation/compiler.py` 的 `VehicleDynamicCompiler` section：改为依赖新 preparation 归属，不再导入旧模块。
- `tests/vehicle/test_service_contract.py` 及矩阵列出的 vehicle dynamic/compiler/results/service 测试：证明显式 staged path、便捷 facade、匹配/不匹配 preparation、legacy `prepared` adapter 类型/identity 校验、typed result→metrics/error evidence 边界；新增 `test_vehicle_legacy_definition_map` 逐项断言 `PREPARATION_MATRIX.md` 中全部旧顶层符号落入 preparation/result/service 新模块，新增 `test_vehicle_service_metrics_and_artifact` 端到端断言统一 runner 结果经 metrics 和 artifact sink；并以计数 backend 证明 staged/facade 各一次 native backend submission；同时断言 `results.decoder` 的整车分派经 `results.vehicle.decode_vehicle_result()`，且 `results.decoder`、`results.vehicle` 和新 service 均不导入旧 `vehicle_dynamics`。
- 整车 preparation 同时导出 `prepare_request(request: SimulationRequest) -> PreparedSimulation`，中央延迟注册表按此固定函数名加载。
- `simulation/runner.py` 仅迁移传给 decoder 的上下文读取：从旧 `prepared` 改为 `vehicle_dynamic_prepared`；旧键只允许迁移 adapter 消费。真实整车 staged/facade 测试必须断言 typed result 非空，覆盖这处接线。
- `PreparedVehicleRun` 保留全部既有物理字段，增加来源 model/case 的对象引用用于 legacy adapter 身份校验；引用不参与 dataclass 相等性或 repr，不序列化进 contract。`prepare_vehicle_run` 的唯一构造点负责填写来源。
- 03 允许将旧 `vehicle_dynamics.py` 改为临时转发薄壳：只从新 preparation/results/service 重导出既有符号（含旧私有准备类型名映射），不保留第二份结果类或执行实现，不删除文件；顶层导出迁移与薄壳最终删除仍由04负责。
- 同步迁移 `tests/results/test_adapters.py` 的 monkeypatch 到 `results.vehicle._vehicle_axle_result`；既有 vehicle/results 测试直接使用新归属。新增结构断言同时禁止新 preparation/results/service 导入旧模块，不能只检查 decoder。

## 约束

- 不在 `simulation/runner.py` 写整车物理逻辑。
- 不改变 `VehicleDynamicsResult` 对外可见属性、异常类型、失败诊断、通道名、单位和时间语义。
- `results.decoder.decode_result()` 是唯一统一 raw-to-typed dispatcher；整车分派必须接线到 `results.vehicle.decode_vehicle_result()` 并清理旧模块导入；`results.vehicle` 和结果层不重新执行 preparation 或 native。
- 只提供 preparation 实现给中央 registry，不直接修改共享 registry 组装点。
- 只有在所有内部引用迁移并通过删除前门禁后，才进入子任务 04 删除文件。

## 验收标准

1. 新 preparation 模块能提供整车 compiler 所需的完整准备上下文，并通过中央 registry 覆盖 `("vehicle", "vehicle_dynamic")`；旧模块清单中的全部 preparation 定义均由 `test_vehicle_legacy_definition_map` 归属到该模块，旧 `prepared` 只能经 adapter 包装。
2. `results.vehicle` 成为 `VehicleDynamicsResult`、`_vehicle_axle_result`（保持同一私有名）等整车结果映射的唯一生产归属，`results.decoder` 的整车分派接线到 `results.vehicle.decode_vehicle_result()` 并清理旧模块导入；统一解码仍从 `results.decoder` 进入，全仓只有一个统一 `decode_result` 定义。
3. 高层 vehicle service 只构造 `SimulationRequest`、调用统一 runner、补充 metrics/错误证据并将结果交给 artifact sink；`test_vehicle_service_metrics_and_artifact` 和计数 backend 证明 staged/facade 各只提交一次。
4. `vehicle_dynamics.py` 不再承载唯一执行、解码、artifact 或 compiler 逻辑，且新代码不再依赖其路径；旧模块完整职责清单中的 service/result/preparation 符号均有可指的新归属。

## 验证

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/vehicle/test_native_vehicle.py packages/suspension_multibody/tests/vehicle/test_service_contract.py packages/suspension_multibody/tests/results packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/io/test_artifacts_unified.py -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run --package suspension-multibody python .codex-tasks/20260920-unified-preparation-cutover/tasks/04-delete-legacy/architecture_contract_scan.py
uv run --package suspension-multibody python -c "import importlib, inspect; from suspension_multibody.simulation.preparation import default_preparation_registry; prep=importlib.import_module('suspension_multibody.preparation.vehicle_dynamic'); result=importlib.import_module('suspension_multibody.results.vehicle'); service=importlib.import_module('suspension_multibody.vehicle.service'); decoder=importlib.import_module('suspension_multibody.results.decoder'); assert ('vehicle','vehicle_dynamic') in default_preparation_registry().keys(); assert all(hasattr(prep,name) for name in ('PreparedVehicleRun','prepare_vehicle_run','adapt_legacy_prepared_request')); assert all(hasattr(result,name) for name in ('VehicleDynamicsResult','_vehicle_axle_result','_contract_constraint_names','decode_vehicle_result')); assert hasattr(service,'run_vehicle_dynamics'); assert 'vehicle_dynamics' not in inspect.getsource(decoder)"
```
