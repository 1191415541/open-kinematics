# p2-06 证据 (e)：两通道总成装配成功并跑通一次 study

> 判据来源：`EPIC.md` G2 行 81 与 Done-When (b) 行 290；父行 p2-06 (e)。
> 夹具：`tests/authoring/fixtures.py::write_vehicle_project`（现有整车三文件项目：`vehicle_assembly.json` + 前后悬架子系统文件 + wheels）。
> 新增测试：`packages/suspension_multibody/tests/preparation/test_steering_channels.py`（8 用例，新增）。

## 1. 通道声明片段原文

在夹具项目的 `vehicle` 段追加一条（唯一改动，其它内容原样）：

```json
{"channel_name": "rear_rack", "placement": "rear", "rack_body": "rack", "ratio": 1.0}
```

读入结果（实跑）：

```
model.steering.channel_name = front_rack  placement = front
model.steering_channels = [('rear_rack', 'rear', 'rack', 1.0)]
axle rack_fixed_to_chassis as the files state it: front = False  rear = False
```

`ratio = 1.0` 是声明「输入按位移读」：`_case()` 用 `steering_input = TimeSignal(times=(0.0, 0.004), values=(0.0, 0.02))`（毫米），准备层把它按 `/1000` 缩放到文档的米制单位，所以请求的 0.02 mm 就是 `2e-05 m`。

**后通道能自由的前提**：`subsystems/steering.py:199`（`WeldJoint` 分支）与 `:239`（`PrismaticJoint` 分支）按 `model.rack_fixed_to_chassis` 二选一。夹具的 rear 子系统文件与 front 是**同一份**（`fixtures.py` 只改 `name` 与 `placement_role`），`rack_fixed_to_chassis` 为默认 `False`，所以解除 `authoring/vehicle.py` 的覆写后 rear 的 rack 保持**自由**，落入 `PrismaticJoint`（guide）分支 —— 后通道因此有可被 prescribed 的平动自由度。若 rear 文件声明 bolted，`_rack_guide_joint` 会按名拒绝而不是给焊死的齿条编一个轴。

实跑确认（前后轴取值直接来自各自子系统文件）：

```
axle rack_fixed_to_chassis from files: front = False  rear = False
```

## 2. 装配产物：逐通道实体清单

`compose_vehicle(model, "K", assembly_request_for(document))` 的产物（实跑）：

```
bodies containing 'rack': ['front_rack', 'rear_rack']
constraints containing 'rack': ['front_rack_tie_joint_L', 'front_rack_tie_joint_R',
                                'front_rack_guide', 'rear_rack_tie_joint_L',
                                'rear_rack_tie_joint_R', 'rear_rack_guide']
```

| 通道（声明序） | 齿条体 | 导向约束 | 转向横拉杆约束 | 转向执行器元素（契约面） |
|---|---|---|---|---|
| `front_rack`（主通道，`placement="front"`） | `front_rack` | `front_rack_guide`（`PrismaticJoint`） | `front_rack_tie_joint_L` / `_R`（来自**悬挂**模板，requirement 1） | `steering_front_rack` |
| `rear_rack`（附加通道，`placement="rear"`） | `rear_rack` | `rear_rack_guide`（`PrismaticJoint`） | `rear_rack_tie_joint_L` / `_R` | `steering_rear_rack` |

两个通道的齿条是**两个不同的刚体**，各自由自己的 `placement` 解析得到（`_resolve_steering_rack(..., placement=)`）—— 这正是「按声明而非按顺序判定身份」的落点：没有 `placement` 参数时 `rack_body="rack"` 会两次解析到同一根前齿条。逐层断言：

```python
# tests/preparation/test_steering_channels.py:137
assert prepared.steering.body[0] != prepared.steering.body[1]
```

实跑读数：`prepared.steering.body = [11 25]`，`prepared.steering.names = ('front_rack', 'rear_rack')`。

准备层数组堆叠形状（`tests/preparation/test_steering_channels.py:122-139` 有对应断言）：

```
prepared.steering.point_local.shape            = (2, 3)
prepared.steering.reaction_point_local.shape   = (2, 3)
prepared.steering.axis_local.shape             = (2, 3)
prepared.steering.reference_quaternion.shape   = (2, 4)
prepared.steering.output.shape                 = (5, 2, 4)
prepared.steering.target.shape                 = (10,)
prepared.steering.target_rate.shape            = (10,)
```

`target`/`target_rate` 的 10 = 5 采样 × 2 通道，排布为内核索引表达式 `actuator.target_angle[k * count + j]` 要求的**采样主序、通道次序**（`np.column_stack([...]).reshape(-1)`；通道主序会解析通过但驱动错样本，故测试断言了布局而不假设）。

## 3. study 实跑：命令、退出码、逐通道转角读数

驱动脚本（临时脚本，落在 session scratch；工作目录 = 仓库根）：

```
$ uv run --no-sync python <scratch>/final_two_channel.py
EXIT=0
```

`run_request(SimulationRequest(assembly="vehicle", family="vehicle_dynamic", model=model, case=case, context={"prepared": prepared}))` 的实跑输出原文：

```
declared row: {"channel_name": "rear_rack", "placement": "rear", "rack_body": "rack", "ratio": 1.0}
axle rack_fixed_to_chassis as the files state it: front = False  rear = False
bodies containing 'rack': ['front_rack', 'rear_rack']
constraints containing 'rack': ['front_rack_tie_joint_L', 'front_rack_tie_joint_R', 'front_rack_guide', 'rear_rack_tie_joint_L', 'rear_rack_tie_joint_R', 'rear_rack_guide']
--- law=direct
    channels        : ('front_rack', 'rear_rack')
    body indices    : [11, 25]
    allocation      : []
    target rows (m) : [[0.0, 0.0], [5e-06, 5e-06], [1e-05, 1e-05], [1.5e-05, 1.5e-05], [2e-05, 2e-05]]
    run.status      : success  steering_output.shape: (5, 2, 4)
    rack travel per channel (m), last sample: [2e-05, 2e-05]
    contract elements: ['steering_front_rack', 'steering_rear_rack']
--- law=four_wheel_steer
    channels        : ('front_rack', 'rear_rack')
    body indices    : [11, 25]
    allocation      : [('front_rack', [0.0, 0.005, 0.01, 0.015, 0.02]), ('rear_rack', [-0.0, -0.00125, -0.0025, -0.00375, -0.005])]
    target rows (m) : [[0.0, -0.0], [5e-06, -1.25e-06], [1e-05, -2.5e-06], [1.5e-05, -3.75e-06], [2e-05, -5e-06]]
    run.status      : success  steering_output.shape: (5, 2, 4)
    rack travel per channel (m), last sample: [2e-05, -5e-06]
    contract elements: ['steering_front_rack', 'steering_rear_rack']
```

逐通道转角读数汇总：

| 分配律 | 声明车速 | `prepared.steering_allocation`（弧度，逐采样） | `run.status` | `steering_output` 形状 | 末样本各通道齿条位移（m） |
|---|---|---|---|---|---|
| `direct` | 0.0 | `[]`（direct 不计算分配） | `success` | `(5, 2, 4)` | `[2e-05, 2e-05]`（两通道等值跟随同一信号） |
| `four_wheel_steer` | 0.0（< 5.0 阈值） | `front_rack` = `[0.0, 0.005, 0.01, 0.015, 0.02]`；`rear_rack` = `[-0.0, -0.00125, -0.0025, -0.00375, -0.005]`（= `-0.25 ×` 前通道） | `success` | `(5, 2, 4)` | `[2e-05, -5e-06]`（后通道**对向**，位移为负） |

读数的两点说明：

* `direct` 律下两个通道位移完全相等 —— **后者不是复制出来的**，而是同一信号分别经各自通道的 `input`/`ratio` 换算（此夹具两通道 `ratio` 都是 1.0），并且内核记录的是两根不同齿条各自的位移（`body indices [11, 25]`）。
* `four_wheel_steer` 律下记录的弧度与实测位移比值一致：`-0.005 / 0.02 = -0.25`，与 `prepared.steering.target` 的 `[-5e-06] / [2e-05]` 同样一致 —— 分配的角确实传到了内核并被积分出来，而不是算完就丢。

求解设置（最小可收敛配置，参考既有跑法 `tests/authoring/test_vehicle_assembly_documents.py:290-345`）：`end_time=0.004`、`step_size=0.001`、`internal_step_size=0.001`、`min_internal_step_size=0.001`、`adaptive_substepping=False`、`integrator="generalized_alpha"`、`gravity=Vec3(0, 0, -9806.65)`（重力打开，给总成一条落地的载荷路径；夹具自己的工况同样这么做）。

同一条 `four_wheel_steer` 路径另有参数化用例在**低速与高速两端**各跑一次（`tests/preparation/test_steering_channels.py::test_four_wheel_steer_reaches_the_channels_it_drives`，`:184`），断言低速后通道为 `-0.25 ×` 前通道、高速为 `+0.15 ×` 前通道，并直接断言符号：

```python
    assert table[-1, 0] == 0.02 * scale
    assert table[-1, 1] == expected_gain * 0.02 * scale
    assert [name for name, _series in prepared.steering_allocation] == [
        "front_rack",
        "rear_rack",
    ]
    front, rear = (series[-1] for _name, series in prepared.steering_allocation)
    assert front == 0.02
    assert rear == expected_gain * 0.02
    same_sign = (rear > 0.0) == (front > 0.0)
    assert same_sign == (expected_gain > 0.0)
```

（上段为便于阅读把断言按原文摘出，逐字原文见 `allocator_assertions.md` §2.2 与测试文件 `:207-219`。）

**高速端只验准备层、不跑求解器**：`initial_forward_speed_mps > 0` 时内核报 `status 7: initial velocity violates velocity constraints`（实跑原文见第 5 节），这是既有的整车初速限制、与本行无关，本行不去绕它。低速与直接律两端都真跑了求解器（`run.status == "success"`）。

## 4. 契约面元素清单

```python
# tests/preparation/test_steering_channels.py:279
assert [entry["name"] for entry in steering] == ["steering_front_rack", "steering_rear_rack"]
```

实跑：`contract elements: ['steering_front_rack', 'steering_rear_rack']`；`tests/preparation/test_steering_channels.py:283/286` 另断言 `{entry["target"]} == {"front_rack", "rear_rack"}` 与 `len(bodies) == 2`（两个执行器指向两根不同的体）。

## 5. 未达成 / 边界（如实登记）

1. **高速 4WS 只验到准备层，未跑求解器**。原因不是本行引入的：给整车非零初速时内核返回 `status 7: initial velocity violates velocity constraints`，原文：

```
suspension_multibody.kernel.KernelContractError: case two-channel failed with status 7: initial velocity violates velocity constraints
```

**该限制与转向通道无关，已用单通道车辆实测确认**：同一夹具的**单通道**模型（`steering_channels == ()`）在 `speed = 0.0` 时 `status=success`，在 `speed = 0.5` 与 `speed = 2.0` 时同样报 `status 7: initial velocity violates velocity constraints`（退出码 0 的脚本内逐条打印）：

```
one-channel model, steering_channels = ()
speed=0.0: status=success
speed=0.5: KernelContractError: case speed failed with status 7: initial velocity violates velocity constraints
speed=2.0: KernelContractError: case speed failed with status 7: initial velocity violates velocity constraints
```

拒绝发生在内核侧（`suspension_kernel/cpp/src/abi/kernel_abi.cpp:276`），是既有的整车初速限制，本行不去绕。判据「4WS 高速同向」由准备层的 per-channel target 表 + `steering_allocation` 记录背书（`tests/preparation/test_steering_channels.py:184-230`），与低速端走同一条代码路径，低速端确实跑通到 `run.status == "success"`。

2. **`ackermann` / `multi_axle_follow` 未在本夹具整车路径上落地**。夹具前后轴的 `WHEEL_CENTER.x` 实测相同（均为 `0.0`），故 `wheelbase_mm = 0.0`、`distance_to_reference_mm = 0.0`：`ackermann` 先按名拒绝（`ValueError: ackermann needs a positive wheelbase on 'front_rack'; found 0.0`，实跑原文），`multi_axle_follow` 亦会因 `distance_to_reference_mm = 0` 拒绝。这两条律的数值断言由 `tests/preparation/test_steering_allocator.py` 用**显式几何量**覆盖（`L = 2800 mm`、`t = 1600 mm`、`Li = 4200/5600 mm`），那正是「几何由调用方给、本模块不猜」的设计目的。**未**声称这两条律已在一台两通道整车上跑通。

3. 夹具上 `prepared.steering.reaction_body` 的两项取值未在测试里断言具体体名（只断言形状 `(2,)`）；两通道的 `actuator_body`/`actuator_reaction_body` 均未在声明里给，走主通道的默认派生路径。此项为设计允许（spec 未要求该断言），如实登记以防被读成已覆盖。
