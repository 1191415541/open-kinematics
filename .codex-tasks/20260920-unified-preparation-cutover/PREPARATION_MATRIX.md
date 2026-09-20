# Preparation 迁移矩阵

本文件是 `20260920-unified-preparation-cutover` 的 family 级执行矩阵。所有子任务以本矩阵为准；修改矩阵必须同步 `EPIC.md` 和 `SUBTASKS.csv`。

| registry key | preparation module | preparation type | domain inputs | context keys produced | compiler consumer | document owner | registry registration | primary tests |
|---|---|---|---|---|---|---|---|---|
| `("axle", "axle_dynamic")` | `suspension_multibody.preparation.axle_dynamic` | `AxleDynamicPrepared` | `AxleDynamicsModel`, `AxleDynamicsCase` | `prepared_simulation`, `axle_dynamic_prepared` | `AxleDynamicCompiler` | `cases/axle_dynamic.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/test_axle_dynamic_contract.py` |
| `("axle", "kc_quasi_static")` | `suspension_multibody.preparation.kc_quasi_static` | `KcQuasiStaticPrepared` | `FrontAxleModel`/`FrontAxleAssembly`, K/C case documents or API case | `prepared_simulation`, `kc_assembly`, `model_document`, `case_document` when documents already exist | `KcQuasiStaticCompiler` | `cases/kc_quasi_static/contract.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/kc_quasi_static/*`, `tests/axle_dynamics/test_api.py` |
| `("vehicle", "vehicle_kc")` | `suspension_multibody.preparation.vehicle_kc` | `VehicleKcPrepared` | vehicle assembly, wheel corners and K/C inputs | `prepared_simulation`, `model_document_pair`, `vehicle_assembly`, `wheels`, `case_document` | `VehicleKcCompiler` | `cases/vehicle_kc.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/test_vehicle_kc.py` |
| `("vehicle", "handling")` | `suspension_multibody.preparation.handling` | `HandlingPrepared` | vehicle model or model document pair, steering shapes, solver settings | `prepared_simulation`, `model_document`, `model_payload`, `case_document` | `HandlingCompiler` | `cases/handling.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/test_handling.py` |
| `("vehicle", "ride_four_post")` | `suspension_multibody.preparation.ride_four_post` | `RideFourPostPrepared` | vehicle model or model document pair, four-post corners/signals, solver settings | `prepared_simulation`, `model_document`, `model_payload`, `case_document` | `RideFourPostCompiler` | `cases/ride_four_post.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/test_ride_four_post.py` |
| `("vehicle", "ride_random_road")` | `suspension_multibody.preparation.ride_random_road` | `RideRandomRoadPrepared` | vehicle model or model document pair, random-road wheels/components, solver settings | `prepared_simulation`, `model_document`, `model_payload`, `case_document` | `RideRandomRoadCompiler` | `cases/ride_random_road.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/test_ride_random_road.py` |
| `("vehicle", "vehicle_dynamic")` | `suspension_multibody.preparation.vehicle_dynamic` | `PreparedVehicleRun` | `VehicleModel`, `VehicleDynamicCase`; legacy `prepared` only through `adapt_legacy_prepared_request` | `prepared_simulation`, `prepared` only as migration input, final `vehicle_dynamic_prepared` | `VehicleDynamicCompiler` | `cases/vehicle_dynamic.py` | `simulation/preparation.py::default_preparation_registry` | `tests/cases/test_vehicle_dynamic_contract.py`, `tests/vehicle/test_native_vehicle.py`, `tests/vehicle/test_service_contract.py`, `tests/simulation/test_preparation.py` |

## Registration and staged execution

- `simulation/preparation.py::default_preparation_registry()` 是唯一默认 registry 组装点；family modules export preparation implementations but do not mutate this registry.
- The explicit staged path is `SimulationRequest → prepare_request() → compile_request(prepared.request) → run_request(compiled) → NativeContractBackend`.
- `run_request(SimulationRequest)` is only a convenience facade over the same staged path and must call preparation once.
- `results.decoder.decode_result()` is the only unified raw-to-typed dispatcher. `results.vehicle` owns the typed vehicle result and its family adapter, not the unified dispatcher.

## Document request bypass

以下任一条件成立时，`prepare_request()` 必须返回直通 `PreparedSimulation`，且不得调用 family preparation：

- `request.context` 含 `model_document` 或 `case_document`；
- `request.context` 含 `model_document_pair`、`model_payload` 或 `case_payload`；
- `request.model` 或 `request.case` 是 contract document / `(document, payload)` pair；
- `request.context` 含匹配当前 request 的 `prepared_simulation`；匹配定义为值是 `PreparedSimulation`，规范化 assembly/family 相同，且 `prepared.request.model is request.model`、`prepared.request.case is request.case`。

直通路径仍由 compiler 做 contract identity 校验；bypass 不是跳过 compiler。registry key 规范化统一使用 `str(value).strip().casefold()`，空键报 `ValueError`，不定义别名。
## Legacy `prepared` 适配

- `preparation.vehicle_dynamic.adapt_legacy_prepared_request()` 是旧 vehicle dynamic `prepared` 输入的唯一适配器；它校验规范化 assembly/family、`PreparedVehicleRun` 类型以及 model/case identity，并产生带 `prepared_simulation` 和 `vehicle_dynamic_prepared` context 的新 request。
- `simulation.prepare_request()` 在 document bypass 之后调用该适配器；当 request 同时含 document context 和旧 `prepared` 时，document bypass 优先，既不查 adapter 也不查 family preparation，但仍由 compiler 做 contract identity 校验。
- 适配后的 `prepared_simulation` 仍遵守统一匹配/stale 规则；其它 family、compiler 和 service 不得直接读取旧 `prepared` 键。
- 必测组合为：旧 `prepared` 包装、document + 旧 `prepared` bypass 优先、包装后匹配复用、model/case stale 时重新 preparation，以及错误类型/身份的可诊断异常。



## 旧整车模块符号新归属

| 旧符号 | 新归属 | 删除前要求 |
|---|---|---|
| `prepare_vehicle_run` | `preparation.vehicle_dynamic.prepare_vehicle_run` | tests、cases、compiler、scripts 改为新导入 |
| `_PreparedVehicleRun` | `preparation.vehicle_dynamic.PreparedVehicleRun` | 不再从旧模块引用私有名 |
| `VehicleDynamicsResult` | `results.vehicle.VehicleDynamicsResult` | 顶层导出、IO、测试和 CLI 改为 results/service 路径 |
| `_vehicle_axle_result` | `results.vehicle.vehicle_axle_result` 或私有等价实现 | `results.vehicle` 内部自持，不反向导入旧模块 |
| `run_vehicle_dynamics` | `vehicle.service.run_vehicle_dynamics`，顶层公开导出可转发 | CLI、scripts、Adams、tests 改为 service 或顶层公开路径 |
| `decode_result` | `results.decoder.decode_result` | 统一 raw-to-typed 分派不得留在旧模块或 `results.vehicle` |


## `vehicle_dynamics.py` 顶层定义迁移清单

子任务 03 和 04 必须逐项核对此清单；路径扫描只能证明旧引用清零，不能替代职责迁移验收。

| 旧定义组 | 完整符号 | 新归属 |
|---|---|---|
| 常量 | `_WHEEL_NAMES`, `_ROAD_KIND`, `_PRESCRIBED_STEERING_TYPES` | `suspension_multibody.preparation.vehicle_dynamic` |
| preparation 数据对象 | `_VehicleSteeringBuffers`, `_VehicleRoadBuffers`, `_NativeVehicleModel`, `_PreparedVehicleRun`, `_BodyFrame` | `suspension_multibody.preparation.vehicle_dynamic`；`_PreparedVehicleRun` 迁移为 `PreparedVehicleRun` |
| preparation 入口与校验 | `_select_assembly_mode`, `_validate_steering_topology`, `prepare_vehicle_run`, `_length_scale`, `_validate_units` | `suspension_multibody.preparation.vehicle_dynamic` |
| preparation 构建辅助 | `_build_static_rotation_gauges`, `_uses_horizontal_static_gauge`, `_output_times`, `_initial_body_state`, `_resolve_vehicle_body`, `_rotation_from_quaternion`, `_tuple3`, `_tuple4`, `_matrix3`, `_shift_point`, `_build_aerodynamic_drags`, `_build_joints`, `_build_coordinate_couplers`, `_build_elements`, `_damper_curve`, `_length_force_curve`, `_bushing_force_curves`, `_spring_force_curve`, `_tuple6`, `_wheel_forward_local`, `_build_tires`, `_build_steering`, `_resolve_named_body`, `_resolve_steering_rack`, `_steering_target_value`, `_steering_target_rate`, `_build_road`, `_build_wheel_torque_signals`, `_native_solver_settings` | `suspension_multibody.preparation.vehicle_dynamic` |
| typed result 与映射 | `VehicleDynamicsResult`, `_contract_constraint_names`, `_vehicle_axle_result` | `suspension_multibody.results.vehicle` |
| 高层执行 service | `run_vehicle_dynamics` | `suspension_multibody.vehicle.service`；顶层公开导出可转发 |

## 必测行为

1. registry key 按 `str(value).strip().casefold()` 规范化，空键和未注册键错误可诊断；
2. document request bypass 不调用 family preparation，但仍执行 compiler contract identity 校验；
3. 匹配当前 request 的 `prepared_simulation` 只复用一次 preparation；不匹配或 stale prepared context 必须重新 preparation；
4. 七个 family 都能从默认 registry 获得 compiler 所需上下文；
5. 显式 staged path 与 `run_request(SimulationRequest)` facade 均只产生一次 native submission，`run_request(CompiledSimulation)` 不查 preparation registry；
6. `results.decoder.decode_result()` 是全仓唯一统一 raw-to-typed dispatcher，`results.vehicle` 只提供 vehicle family adapter；
7. `vehicle_dynamics.py` 完整顶层定义清单均迁入指定新归属，删除后交付目录无旧模块 import 或文件路径引用；
8. 独立 `test_dynamic_result_compat.py` 覆盖历史 `DynamicResultBundle` 读取，历史边界不回退。
