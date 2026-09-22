# 07 第 1 项：report 边界与逐符号归属登记

本文件是子任务 07 第 1 项（「冻结 report 边界与逐符号归属」）的**逐符号登记**。口径按 SPEC 目标 1-3 与 EPIC「Python 迁移矩阵」：纯诊断迁 `report/`、求解归 native、不以「无内部调用」擅自删除公开能力；`report` 只消费结果与只读说明数据。

判定规则（负例门禁 `tests/architecture/legacy_surface_gate.py` 实际执行）：

- `report/**` 不得 import 或调用 native/kernel/solver/axle_dynamics（`NATIVE_TOKENS`）；
- 不得调用 `run_contract`/`solve`/`solve_static`/`solve_dynamic`/`run_solver`（`SOLVE_NAMES`）；
- 不得 import 或调用 preparation/authoring（新增 `report_preparation_import`/`report_preparation_call`）；
- 不得求值元件力律（新增 `report_constitutive_call`）。

实测 `report` 的 import 面只有：`__future__`、`collections.abc`、`dataclasses`、`typing`、`numpy`，以及 `suspension_multibody.report.*` 自身子模块；无 native/kernel/solver/axle_dynamics、无 preparation、无 results。

## A. `metrics/**` → `report/metrics/**`（逐字搬运，统计定义与阈值不变）

| 旧符号 | 旧位置 | 新位置 | 生产调用者 | 判定 |
|---|---|---|---|---|
| `compute_axle_metrics` | `metrics/axle.py:26` | `report/metrics/axle.py` | `api.py:296`（`_result_metrics`） | 迁 report；签名与返回键不变 |
| `axle_metrics` | `metrics/axle.py:68` | 同上 | 无生产调用者 | 保留（公开别名），不删 |
| `_tire_column` / `_TIRE_*` | `metrics/axle.py:11-23` | 同上 | 内部 | 随实现迁移 |
| `compute_common_metrics` | `metrics/common.py:97` | `report/metrics/common.py` | `api.py:287` | 迁 report |
| `peak` / `rms` / `time_metrics` / `convergence_metrics` / `contact_metrics` | `metrics/common.py:20,26,38,49,76` | 同上 | 经 `compute_common_metrics`/`compute_axle_metrics` | 迁 report |
| `residual_norm` | `metrics/common.py:32` | 同上 | **无生产调用者** | 保留公开能力（SPEC 目标 3 口径），不删 |
| `_finite_array` | `metrics/common.py:11` | 同上 | 内部 | 随实现迁移 |
| `_CASE_METRICS`（注册表） | `metrics/case_specific.py:10` | `report/metrics/case_specific.py:18` | 模块级单例 | 保持原样：模块级 dict、唯一注册入口 |
| `register_case_metric` | `metrics/case_specific.py:13` | 同上 | 仅 `_register_defaults` | 迁 report |
| `compute_case_metrics` | `metrics/case_specific.py:26` | `report/metrics/case_specific.py:34` | `api.py:179,249,305,456,516,531`；`axle_dynamics/contract_run.py:129,145`；`vehicle/service.py:72,95` | 迁 report；签名与 KeyError 文案不变 |
| `case_metric_for` | `metrics/case_specific.py:37` | 同上 | **无生产调用者** | 保留公开能力，不删 |
| `registered_case_metric_families` | `metrics/case_specific.py:46` | 同上 | **无生产调用者** | 保留公开能力，不删 |
| `not_applicable_metrics` | `metrics/case_specific.py:51` | 同上 | 经注册表 5 个 `not_applicable` family | 迁 report，占位语义逐项不变 |
| `wheel_metrics` | `metrics/case_specific.py:67` | 同上 | `analysis/metrics.py:15`（转发面，已改指 report）；`analysis/__init__` 公开导出 | 迁 report，并在 `report/metrics/__init__` 导出（承接 `analysis.wheel_metrics` 公开能力） |
| `compute_k_metrics` | `metrics/case_specific.py:96` | 同上 | `analysis/metrics.py:20`（转发面，已改指 report） | 同上 |
| `_summarize_series_values` / `_kc_quasi_static_metrics` / `_axle_dynamic_metrics` / `_vehicle_dynamic_metrics` | `metrics/case_specific.py:117,128,164,170` | 同上 | 经注册表 | 迁 report |
| `_register_defaults`（唯一注册入口） | `metrics/case_specific.py:176`，模块末尾 `:195` 调用 | `report/metrics/case_specific.py:184`，`:203` 调用 | 模块级 | 保持原样；默认 family 与占位逐项不变（真实实现：`kc_quasi_static`/`axle_dynamic`/`vehicle_dynamic`；`not_applicable`：`vehicle_kc`/`vehicle_kc_dynamic`/`handling`/`ride_four_post`/`ride_random_road`） |
| `wheel_load_metrics` | `metrics/vehicle.py:16` | `report/metrics/vehicle.py` | `analysis/vehicle_physics.py:83`（兼容转发面，已改指 report） | 迁 report；签名与 4 角校验不变 |
| `compute_vehicle_metrics` | `metrics/vehicle.py:54` | 同上 | `report/metrics/case_specific.py` 的 `_vehicle_dynamic_metrics` | 迁 report |
| `vehicle_metrics` | `metrics/vehicle.py:78` | 同上 | **无生产调用者** | 保留公开别名，不删 |
| `_embedded_wheel_loads` | `metrics/vehicle.py:41` | 同上 | 内部 | 随实现迁移 |

## B. `analysis` 的轮几何、柔度与统计 → `report/geometry.py`、`report/compliance.py`

| 旧符号 | 旧位置 | 新位置 | 生产调用者 | 判定 |
|---|---|---|---|---|
| `_wheel_geometry` | `analysis/_geometry.py:19` | `report/geometry.py` | `adams/reference.py:125`、`cases/kc_quasi_static/workflow.py:46`（两处 import 已改指 report） | 迁 report；符号/角度约定逐字保留，调用方传入已有位姿，不执行 preparation |
| `_WheelGeometry` | `analysis/_geometry.py:11` | 同上 | 上述两处的返回值 | 迁 report |
| `secant_compliance` | `analysis/compliance.py:28` | `report/compliance.py` | `api.py:558,561` | 迁 report；数值口径不变（有测试钉住） |
| `tangent_compliance` | `analysis/compliance.py:20` | 同上 | **无生产调用者** | 保留公开能力，不删 |
| `validate_compliance` | `analysis/compliance.py:8` | 同上 | **无生产调用者** | 保留公开能力，不删 |
| `wheel_metrics` / `compute_k_metrics`（转发面） | `analysis/metrics.py:15,20` | 转发目标改为 `report/metrics/case_specific.py` | `analysis/__init__:16` | 只改转发目标，不删（08 删除目录本体） |

## C. `analysis/time_domain_physics.py` 等「仅导出功能」逐符号判定

`analysis/time_domain_physics.py` 全模块（`DynamicLoadTransferSample:21`、`DynamicLoadTransferResult:36`、`diagnose_dynamic_load_transfer:52`、`_contact_normal_loads:136`、`_center_of_mass:146`、`_momentum:160`、`_external_wrench:182`）在生产、脚本、测试中**均无调用者**，且其输入 `FullVehicleDynamicRun`（`:16`）是**已退役的 Python 车辆积分器**的 run 对象，没有任何现存生产者。

判定：**纯诊断，迁 `report/time_domain_physics.py`**，四个私有辅助函数一并迁移，语义逐字保留（含 `gravity` 默认值与 `ignore_before` 校验）。唯一改动：原 `:10` 的 `from ..model import VehicleAssembly` 不再引入——该类型只出现在三个私有辅助函数的注解上，`report` 引用已退役的 `model` 包会构成 report→legacy 边；注解改为结构化（与同文件 `FullVehicleDynamicRun = Any` 同口径）。运行期行为不变（`from __future__ import annotations` 下注解不求值）。

该模块依赖 `WheelLoadSummary`/`summarize_wheel_loads`（旧 `analysis/vehicle_physics.py:18,57`），故这两个纯聚合符号一并迁入 `report/wheel_loads.py`（`analysis` 侧副本保留给 `analysis/time_domain_physics.py` 与 `StaticWheelLoadResult.summary`，08 随目录删除）。

**保留公开能力并说明**（不迁 report，理由是与「只消费结果与只读说明数据」冲突）：

| 符号 | 位置 | 不迁理由 |
|---|---|---|
| `compute_static_wheel_loads` | `analysis/vehicle_physics.py:88` | 调用 `build_vehicle(vehicle, mode="K")`，即**执行 preparation**；EPIC A2 已裁定保留原样并登记（native 无静力求解 ABI 入口、ABI 导出面冻结） |
| `compute_vehicle_roll_centers` | `analysis/vehicle_physics.py:145` | 同样调用 `build_vehicle(...)`（`:151`）并从作者侧硬点算瞬心，属作者层几何，迁 report 会引入 report→preparation 边。**无生产调用者**，按 SPEC 不删除，但**无 A2 式登记**——见第 7 项遗留 |
| `RollCenterResult` / `StaticWheelLoadResult` / `WheelLoadSummary`（analysis 副本） | `analysis/vehicle_physics.py:48,32,18` | 随各自生产者保留；`WheelLoadSummary` 已在 `report/wheel_loads.py` 另立一份 |
| `analysis/__init__` 的全部再导出 | `analysis/__init__.py:14-31` | 08 删除目录本体；本任务只切生产调用方 |

## D. `analysis/vehicle_kc_time_domain.py` → `simulation/replay.py` + `results/timeseries.py`

| 旧符号 | 旧位置 | 新位置 | 调用者 | 判定 |
|---|---|---|---|---|
| `VehicleKCTimeDomainSolver` | `analysis/vehicle_kc_time_domain.py:23` | `simulation/replay.py:38` | 唯一生产调用者 `api.py:163`（`run_dynamic_case`） | 编排迁 `simulation/replay.py`；`api.py` 的 import 改指新入口 |
| `VehicleKCTimeDomainSolver.run` | `:26` | `simulation/replay.py:41` | 同上 | 保留「无积分」语义：不引入积分器、时间网格与样本内容不变 |
| `_vehicle_metrics` | `:64` | `simulation/replay.py:77` | `run` | 逐字迁移（`degrees_of_freedom`/`steering_input`/`rack_displacement`/`roll_angle`/`body_roll`/`body_pitch`/`body_yaw`/`body_heave` 键不变） |
| `_motion` | `:82` | `simulation/replay.py:95` | `run`/`_vehicle_metrics` | 逐字迁移（别名表不变） |
| `_pose_from_angles` | `:99` | `simulation/replay.py:112` | `run` | 逐字迁移；`rotation_vector_to_quaternion` 改从 `preparation/geometry.py` 取（不再经 `core`） |
| 结果聚合 | 旧 `TimeSeriesResult.from_samples(...)` 调用点 `:55-61` | `results/timeseries.py` 新增 `aggregate_replay_samples`（编排只传样本与 provenance） | `simulation/replay.py` | 聚合协议固定在 `results`：时间轴 = 样本时间按序、默认指标 `{"sample_count": n}`、`diagnostics=()`；口径不变 |
| `analysis/time_signals` 依赖 | `:20` `from .time_signals import ...` | `from ..preparation.signals import ...`（06 产物） | `run` | 残留调用面改指 preparation；`analysis/time_signals.py` 本体保留（08 删） |

## E. `analysis/benchmarks.py` → `tests/data` 声明式夹具

| 旧符号 | 旧位置 | 新位置 | 调用者 | 判定 |
|---|---|---|---|---|
| `benchmark_model` | `analysis/benchmarks.py:27` | `tests/data/benchmark_axle.json` 的 `model` 字段（数据） | 脚本：`case_parity_check.py:295,316`、`kc_native_probe.py:98`、`kc_perf_gate.py:72,75`（全部改读显式夹具路径）；测试：7 个文件 | 声明式数据 + 显式路径读取；脚本不 import 测试包 |
| `benchmark_grid` | `analysis/benchmarks.py:23` | 同上的 `grid` 字段 | `kc_perf_gate.py:75` | 同上 |
| `_WHEEL_VALUES_MM` / `_RACK_VALUES_MM` | `analysis/benchmarks.py:19,20` | 同上（`grid.wheel_values_mm` / `grid.rack_values_mm`，10×10 原值） | — | 数据化；数值逐位一致（实测 `model_dump(mode="json")` 相等、网格元组相等） |

测试侧：新增 `tests/benchmark_fixture.py`（按显式路径读同一 JSON，不写模块级状态）；6 个测试模块改用它；`tests/cases/kc_quasi_static/kc_fixtures.py` **自持**读取（该模块被 3 个脚本按显式路径加载，不能依赖 `tests` 可导入），夹具覆盖未减少。`analysis/benchmarks.py` 本体保留（08 删除）。

## F. 边界与 08 交接

- `report` 只消费结果与只读说明数据：无 native 调用、无 preparation 执行、无本构复算（三类负例门禁已补，见 `raw/step7_verification.md`）。
- `io.artifacts` 仍是唯一读写归属：本任务未新增任何 artifact 写路径。
- 未删除任何目录或文件本体：`analysis/**`、`metrics/**`、`core/**`、`model/**` 全部保留（08 负责删除）。
- 08 接手时需要处理的遗留：`analysis/vehicle_physics.py` 的 `compute_vehicle_roll_centers`（无 A2 登记、无生产调用者、因调用 preparation 不能迁 report）。
