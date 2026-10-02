# p5-04 证据：结果块 `controller_output`（定义、列序、开关、默认关闭）

> 实测日期 2026-10-02。D2 裁决口径：控制段没有现成承载位置，须新增结果块
> `controller_output`，且这是**结果契约扩展，不是 ABI 变更**。

## 1. 为什么必须新增一个块

`element_wrench` 的语义被冻结为「元素实际施加的 wrench」——它的两行是同一力偶的两端，
不能改解释成控制量；元素的整数槽又属**静态输入**（`demand_source`/`demand_tire`/`controller_enabled`
都是模型文档里写死的）。因此控制段（测到的滑移、目标、算出的需求、驾驶员信号）没有承载位置。

## 2. 定义（逐文件）

| 文件 | 内容 |
|---|---|
| `cpp/include/mb_config/constants.hpp` | `inline constexpr int kControllerOutputWidth = 4;`，并注明四列列序 |
| `cpp/include/mb_config/controller_output.hpp`（新增） | `class ControllerSink`、`ControllerSink* controller_sink()`、`bool controller_sink_active()`、`void record_controller_sample(...)`、`void set_controller_driver_demand(...)` |
| `cpp/include/mb_config/env.hpp` + `cpp/src/config/kernel_config.cpp` | `bool controller_output_enabled()`，环境变量 **`SUSPENSION_KERNEL_CONTROLLER_OUTPUT`** |
| `cpp/src/abi/kernel_contract_run.cpp` | 分配 `controller_block`、case 循环内 `configure`、`push_block("controller_output", ...)`、`append_doubles(...)`，以及一个 `ControllerGuard` |
| `cpp/src/output/kernel_output.cpp` | observer 每样本 `begin_sample`/`end_sample` |
| `cpp/src/element/anti_roll.cpp` + `element/directional.cpp` | 控制律记录一步 |
| `cpp/include/mb_model/types.hpp` | `RotationalTorque` 增 `target_slip`、`controller_gain` |
| `cpp/include/mb_input/types.hpp` | 整数槽 `ELEMENT_INT_CONTROLLER_ENABLED = 8`；参数槽 138/139 |
| `cpp/src/assembly/element_reader.cpp` | 读三槽并校验（非法 flag / 启用但缺 target/gain 按名拒绝） |
| `cpp/src/cases/contract_model.cpp` | 解析文档 `controller_enabled` / `target_slip` / `controller_gain`（**缺省即关闭**） |

## 3. 列序（冻结，测试照此断言）

| 列 | 名称 | 含义 |
|---|---|---|
| 0 | `measured_slip` | 该拍读到的轮胎纵向滑移（`State::tire_sx`，无量纲滑移率） |
| 1 | `target_slip` | 该律的目标滑移；**未跑律时为 −1** |
| 2 | `control_demand` | 该律算出的归一化需求（0..1） |
| 3 | `driver_demand` | 该样本携带的驾驶员需求（**律把它替换之前**的值） |

未记录的样本行保持 NaN——「无记录」与「记录了一个 0」是两件事。

## 4. 开关语义（默认关闭）

`controller_output_enabled()` 读环境变量，默认（未设或首字符为 `0`/空）为 **false**。
关闭时：

- `controller_block` 是**空 vector** ⇒ `push_block` 直接 return ⇒ **不产生 descriptor**；
- `append_doubles` 传 0 长度 ⇒ **blob 不增加字节**；
- `controller_sink()` 返回 `nullptr` ⇒ 元素律那一行 `controller_sink_active()` 恒 false，**不写任何行**。

### 实测

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases/test_abs_closed_loop.py -q
......                                                                   [100%]
6 passed in 1.06s
```

其中 `test_the_control_ledger_is_absent_unless_it_is_asked_for` 断言：
未设 `SUSPENSION_KERNEL_CONTROLLER_OUTPUT` 时 `"controller_output" not in raw.blocks`；
设了之后块存在且形状 `(201, 4)`。

## 5. 不是 ABI 变更（实测依据）

- `mb_config/version.hpp` 三个常量仍是 **17 / 32 / 1**（`kernel/native.py` 成对同步）；
- `AxleInput`/`AxleOutput`/`VehicleInput`/`VehicleOutput`/`ElementBlock` 的形状**一字未改**；
  新块走的是 `kernel_contract_run.cpp` 既有的「动态分配 + descriptor + blob append」机制，
  与该文件里 `energy`/`steering_output`/`element_wrench` 等块**完全同一套**。
- 参数槽 138/139 取自 137（`..._MAX_TORQUE`）与 144（`ELEMENT_ANTI_ROLL_STIFFNESS`）之间的
  空隙，`static_assert` 已钉住不越界、不与其他族重叠；整数槽 8 取自 `kElementIntBlockSize = 16`
  的既有空位（0..7 已用）。
