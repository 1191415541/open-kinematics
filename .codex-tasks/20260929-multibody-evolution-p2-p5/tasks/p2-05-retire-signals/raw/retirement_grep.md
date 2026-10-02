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

## 1. 调用与数据流隔离

> **纠错（2026-10-02，p5-06 独立复核 `8ab15196` 触发）**
>
> 本节原先引用了一段**在仓库任何提交中都不存在**的代码——声称
> `vehicle_dynamic.py:298-315` 处是
> `declared_demand = getattr(...)` / `if declared_demand == "none":` 的条件调用，
> 并把 `call lines in prepare_vehicle_run: [308] # 唯一调用，位于 declared_demand == "none" 分支内`
> 当作 AST 实测输出。二者都不是事实：
>
> * AST 扫全仓生产代码，`_build_wheel_torque_signals` 的唯一调用点在 `:342`，
>   **其 enclosing 函数是 `prepare_vehicle_run`，且没有任何 enclosing 条件分支**；
> * `grep -c "declared_demand" vehicle_dynamic.py` = **0**；
>   `git log --all -S "declared_demand"` 为空——**该变量在任何提交中都不存在**。
>
> 当时实际实现的是**无条件调用旧 helper，随后对 opt-in 的轮做 `pop`**。也就是说，
> **「调用隔离」当时并未实现**；下面的运行时表格测到的是 **`pop` 的效果（数据流侧）**，
> 而不是调用侧的隔离。
>
> `EPIC.md` 的 `ddc3f952` 修订原文要求的是「**调用与数据流**被声明分支隔离」。
> 2026-10-02 由主代理按复核裁决 `40d78977` 补上了真正的调用隔离（详见 §1a）。

### 1a. 调用隔离（2026-10-02 补实现）

`_build_wheel_torque_signals` 现在只在一个条件下被调用：

```python
declared = getattr(model.driveline, "torque_demand", "none")
if declared == "none":
    wheel_torque, brake_torque = _build_wheel_torque_signals(...)
    return wheel_torque, brake_torque, {}, {}

if case.wheel_drive_torque:  raise ValueError(...)   # 将被舍弃的显式力矩按名拒绝
if case.wheel_brake_torque:  raise ValueError(...)
wheel_demand, brake_demand = _build_demand_signals(model, case, times)
return {}, {}, wheel_demand, brake_demand
```

AST 复验（同一扫描）：

```
call sites of _build_wheel_torque_signals: [(2073, ['_build_torque_and_demand_signals'])]
guard line 2072: if declared == 'none'
  -> legacy helper is called ONLY inside this branch
```

**鉴别力已验证**：把实现临时还原成无条件调用后，新增的三条用例中
`test_the_legacy_builder_runs_only_on_the_none_declaration` 与
`test_the_legacy_builder_reads_no_front_brake_bias_on_an_opt_in_run`
**立即失败**（后者报 `bias_reads == 4`，正是复核指出的问题）。

零回归复验：`dynamic_hash_sentinel.py --check` 26 artifact 逐字节一致
（combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`）；
`case_parity_check.py` 的 `vehicle_dynamic` 一行报 **8 cases, bit-identical**。

### 1b. 数据流侧（当时实测，结论仍有效）

四种声明下的运行时实测（同一模型、`brake=0.5`）：

| `torque_demand` | `rotational_torque` 元素数 | 旧表行数（wheel+brake） | 新需求表行数（wheel+brake） |
|---|---|---|---|
| `none` | **0** | **4 + 4** | 0 + 0 |
| `brake` | 4 | **0 + 0** | 0 + 4 |
| `drive` | 2 | **0 + 0** | 2 + 0 |
| `both` | 6 | **0 + 0** | 2 + 4 |

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
