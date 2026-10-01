# p2-05 证据 (a)：旧路径的调用与数据流隔离

> 实测日期 2026-10-01。裁决 `ddc3f952` 采纳「保留 none 兜底、判据改为隔离」（见 §0）。

## 0. 为什么判据从「删除 + grep 零命中」改为「隔离」（实测依据）

把 `wheel_demand_wheels` 临时改成「`none` 按 `brake` 处理」（即默认路径也走力矩元）后：

```
$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py --family vehicle_dynamic
suspension_multibody.axle_dynamics.errors.NativeAxleError:
case native-vehicle failed with status 5: time integration failed at t=0.000000 s: Newton solve did not converge
```

即 **默认路径走力矩元与 8 个冻结基线逐位一致不可同时成立**：那 8 个用例的模型都是同一个
整车夹具，它给初速即不收敛（与 `p2-11` 的独立实测一致，见
`tasks/p2-11-torque-response/raw/non_zero_response.md`；既有 `xfail`
`tests/vehicle/test_native_vehicle.py:1698-1721` 记录同一结构性问题）。实验已回滚。

故采纳「保留旧路径为 `none` 的兜底」，判据改为**可判定的隔离**（下面的实测即是）。

## 1. 调用隔离（AST 支配关系 + 运行时计数）

生产代码里 `_build_wheel_torque_signals` 的调用点：

```
# AST 扫描 suspension_multibody 生产代码
call sites of _build_wheel_torque_signals: 1
call lines in prepare_vehicle_run: [308]        # 唯一调用，位于 declared_demand == "none" 分支内
```

`packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py:298-315`
改后原文（要点）：

```
declared_demand = getattr(model.driveline, "torque_demand", "none")
if declared_demand == "none":
    wheel_torque, brake_torque = _build_wheel_torque_signals(
        model, case, times, length_scale
    )
    wheel_demand = {}
    brake_demand = {}
else:
    wheel_torque = {}
    brake_torque = {}
    wheel_demand, brake_demand = _build_demand_signals(model, case, times)
```

四种声明下的运行时实测（同一模型、`brake=0.5`）：

| `torque_demand` | `rotational_torque` 元素数 | 旧表行数（wheel+brake） | 新需求表行数（wheel+brake） |
|---|---|---|---|
| `none` | **0** | **4 + 4** | 0 + 0 |
| `brake` | 4 | **0 + 0** | 0 + 4 |
| `drive` | 2 | **0 + 0** | 2 + 0 |
| `both` | 6 | **0 + 0** | 2 + 4 |

即三种 opt-in 取值下旧 helper 的调用次数为 **0**（旧表为空），`none` 下它是唯一来源。

## 2. `front_brake_bias` 的处置：保留字段，隔离用途

- **字段保留**：`DrivelineSpec.front_brake_bias` 是**非 exclude** 字段
  （`schema/vehicle.py:222-224`），删除会改变 `model_dump(mode="json")` 与 `api.py` 算出的
  `model_hash`，进而动到冻结基线。故**不删、不改成 exclude**。
- **用途隔离**：opt-in 路径**不读**它。AST 实测：

```
# torque_elements 模块内 front_brake_bias 的属性读取次数
front_brake_bias attribute reads in torque_elements: 0
```

  分配口径改由该角色自己的常量 `BRAKE_FRONT_SHARE = 0.6`（`torque_elements.py`）驱动，
  前/后轴各按其制轮数均分（复刻退役 builder 的均分口径）。实测：`brake` 声明下前轮增益
  `8.700000000000001 N·m`（= 29000 N·mm × (0.6/2) × 1e-3），后轮 `5.8 N·m`。

## 3. 未越界

- 未改 `templates/roles.py` / `templates/builtin.py`（归 p2-04）与内核（归 p2-02）。
- 未改转向段（归 p2-06）与轮胎拒绝段（归 p4-04）。
- 未改 ABI 版本常量；未重录基线；未新增 skip/xfail。
