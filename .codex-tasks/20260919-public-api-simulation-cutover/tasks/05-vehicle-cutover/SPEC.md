# 子任务 05：整车 service 入口迁移

## 目标
拆分 `vehicle_dynamics.py` 的整车 preparation、统一执行和结果解码职责，使整车内部 service 依赖 `SimulationRequest`、`VehicleResult` 和 vehicle metrics；公共 API 与 artifact 写出在任务 06 切换。

## 范围
- `vehicle_dynamics.py`
- `cases/vehicle_dynamic.py`
- `cases/vehicle_kc.py`
- `results/vehicle.py` 的调用接口配合
- `metrics/vehicle.py`
- `metrics/common.py`
- `VehicleDynamicsResult`、属性转发和 `steering_output`
- 整车 service、结果快照、Adams/vehicle 回归

## 必须完成
1. 整车 preparation 只负责把显式多体整车模型和工况输入规范化为 request payload 或 compiler 输入。
2. 整车执行统一经 `simulation.run_request()`，不得在 `vehicle_dynamics.py` 直接提交 native。
3. 正式输出经 `results.decoder` 组装为 `VehicleResult`，再按兼容需要包装为 `VehicleDynamicsResult`。
4. `metrics.vehicle` 和明确注册的 case-specific metrics 可从正式结果对象计算；不在本任务内重复实现 decoder。
5. 统一保存刚体、约束、元件、轮胎、诊断、性能和时间序列；不把模型误称为 14/15-DOF 降阶模型。
6. `write_vehicle_dynamics_artifact()` 不在本任务产生新的非统一 artifact；统一 writer 由任务 06 建立。
7. 保留现有公开属性、`axle` 访问、属性转发、`steering_output` 和兼容异常行为。

## 约束
- 不修改显式多体模型方程和 native contract。
- 不将整车与整轴 compiler 合并。
- 不丢失 steering、road、wheel torque 等整车工况输入语义。
- 不把 `api.py`、CLI、Adams 外部调用方切换提前纳入本任务。

## 验收
- 整车内部 service 完整经过统一 runner、RawContractResult、VehicleResult 和 metrics。
- `vehicle_dynamics.py` 不再承载唯一执行或解码逻辑；artifact 写出仍等待任务 06。
- 整车成功、失败、partial、属性兼容、结果快照回归通过。

## 验证
`uv run --all-packages pytest packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py packages/suspension_multibody/tests/cases/test_vehicle_kc.py packages/suspension_multibody/tests/analysis/test_vehicle_dynamic.py packages/suspension_multibody/tests/adams/test_full_vehicle_model.py -q && uv run --all-packages pytest packages/suspension_multibody/tests/architecture -q`
