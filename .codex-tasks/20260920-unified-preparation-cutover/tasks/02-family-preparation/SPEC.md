# 子任务 02：迁移六个非 vehicle_dynamic family 的装配前处理

## 目标

将除 `vehicle_dynamic` 外六个工况族的装配、单位归一化、输入信号准备和 compiler 前置上下文接入统一 preparation registry；保持 family 独立语义，不建立巨型共享准备器。`vehicle_dynamic` 的完整 preparation 迁移由子任务 03 独占收口。

## 覆盖 family

- `axle_dynamic`
- `kc_quasi_static`
- `vehicle_kc`
- `handling`
- `ride_four_post`
- `ride_random_road`

本子任务不得修改 `simulation/preparation.py`、`tests/simulation/test_preparation.py`、`preparation/vehicle_dynamic.py`、`results/vehicle.py`、`results/decoder.py` 或整车 service；默认 registry 的注册表结构由子任务 01 提供。

## 交付范围

- 新增 `preparation/` 包中六个按 family 划分的准备模块；必要时保留 cases 中纯 document builder。
- 将现有 `build_front_axle`、车辆装配、输入信号转换、solver 配置等前置工作包装为矩阵指定的显式准备对象。
- 仅修改 `simulation/compiler.py` 中六个指定 compiler section，使其消费准备上下文；不得改动 `VehicleDynamicCompiler` section。
- 修改六个 family 自有 cases 和 contract 测试；每个 family 测试至少断言默认 registry key、准备对象类型和矩阵声明的 compiler context。
- 每个 family 模块必须导出 `prepare_request(request: SimulationRequest) -> PreparedSimulation`；中央延迟注册表按此固定函数名加载。

## 约束

- 不合并各 family 的 contract document 和物理语义。
- 不迁移 native 提交、结果解码、metrics 或 artifact 写出到 preparation。
- 不直接注册或变更共享 preparation registry。
- 不改变现有模型/工况文件、通道、单位和输出契约。

## 验收标准

1. 六个指定 family 的规范化 key 均存在于默认 preparation registry，并能返回矩阵指定准备类型和 compiler context。
2. 六个 compiler section 只消费准备结果和领域输入并输出 `CompiledSimulation`，不再隐式调用散落的装配前处理入口。
3. 已有直接 document 请求仍可编译，且不会重复构建领域装配；对应回归必须证明 bypass 仍执行 compiler contract identity 校验。
4. 矩阵列出的六组 family tests 和 architecture tests 通过；整车 dynamic 专属测试留给子任务 03。

## 验证

```bash
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/cases/test_axle_dynamic_contract.py packages/suspension_multibody/tests/cases/kc_quasi_static packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/cases/test_handling.py packages/suspension_multibody/tests/cases/test_ride_four_post.py packages/suspension_multibody/tests/cases/test_ride_random_road.py packages/suspension_multibody/tests/axle_dynamics/test_api.py -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q
uv run --package suspension-multibody python -c "from suspension_multibody.simulation.preparation import default_preparation_registry; required={('axle','axle_dynamic'),('axle','kc_quasi_static'),('vehicle','vehicle_kc'),('vehicle','handling'),('vehicle','ride_four_post'),('vehicle','ride_random_road')}; keys=set(default_preparation_registry().keys()); assert required <= keys, (required-keys, keys)"
```
