# p2-06 证据 (c)：转向分配器的四种分配律与逐条断言

> 模块：`packages/suspension_multibody/src/suspension_multibody/preparation/steering_allocator.py`（322 行，新增）
> 测试：`packages/suspension_multibody/tests/preparation/test_steering_allocator.py`（311 行 / 24 用例，新增）
> 分层：落在 `preparation/`（`modeling -> templates -> subsystems -> preparation/studies -> cases`）。模块只 import `math` / `dataclasses` / `typing`，**不 import 任何 multibody 内部模块**；几何量（wheelbase / track / distance_to_reference）全部由调用方给。

## 1. 公开接口与四种分配律的公式声明

```python
AllocationLaw = Literal["direct", "ackermann", "four_wheel_steer", "multi_axle_follow"]

@dataclass(frozen=True)
class AllocationChannel:
    name: str
    placement: str
    wheelbase_mm: float
    track_mm: float
    distance_to_reference_mm: float

@dataclass(frozen=True)
class SteeringAllocation:
    names: tuple[str, ...]
    angles_rad: tuple[float, ...]
    law: str
    speed_mps: float
    steer_input_rad: float

def allocate(law, channels, *, steer_input_rad, speed_mps,
             low_speed_threshold_mps=DEFAULT_LOW_SPEED_THRESHOLD_MPS,
             rear_gain_low=FOUR_WHEEL_STEER_LOW_SPEED_GAIN,
             rear_gain_high=FOUR_WHEEL_STEER_HIGH_SPEED_GAIN) -> SteeringAllocation
```

模块级常量（`steering_allocator.py:68/73/78`）：`FOUR_WHEEL_STEER_LOW_SPEED_GAIN = -0.25`、`FOUR_WHEEL_STEER_HIGH_SPEED_GAIN = 0.15`、`DEFAULT_LOW_SPEED_THRESHOLD_MPS = 5.0`。

| 分配律 | 公式（模块 docstring 与 `_direct`/`_ackermann`/`_four_wheel_steer`/`_multi_axle_follow` 的实现一致） |
|---|---|
| `direct` | 每个通道转角 = `steer_input_rad`（恒等，不做几何变换） |
| `ackermann` | 参考通道 `channels[0]` 取 `δc = steer_input_rad`；`L = channels[0].wheelbase_mm`、`t = channels[0].track_mm`，`R = L / tan(|δc|)`；`|δin| = atan(L / (R − t/2))`、`|δout| = atan(L / (R + t/2))`，符号随 `sign(δc)` 恢复。`|δc| < 1e-12` → 全零；`R − t/2 <= 0` → 按名 `ValueError`（"geometrically unreachable"） |
| `four_wheel_steer` | 参考通道 `δf = steer_input_rad`；其余通道 `δr = gain · δf`，`gain = rear_gain_low if speed_mps < low_speed_threshold_mps else rear_gain_high` |
| `multi_axle_follow` | 参考通道 `δref = steer_input_rad`；其余通道 `δi = atan((Li / Lref) · tan(δref))`，`Lref = channels[0].wheelbase_mm`、`Li = channels[i].distance_to_reference_mm` |

`allocate` 的边界行为：`channels` 为空 → 空 `SteeringAllocation`；未知 `law` → `ValueError`（消息含具名 law）；非有限输入（`steer_input_rad` / `speed_mps` / `low_speed_threshold_mps` / 两个 gain / 每个通道的三项几何量）→ `ValueError`。输出顺序与输入通道顺序一致。

## 2. 逐条断言（测试文件:行 + 断言表达式原文）

### 2.1 `tests/preparation/test_steering_allocator.py`

| # | 用例（行） | 断言表达式原文 |
|---|---|---|
| 1 | `test_direct_passes_the_input_through_on_one_channel`（`:47`） | `:51` `assert result.names == ("front_rack",)`；`:52` `assert result.angles_rad == (0.2,)`；`:53` `assert result.law == "direct"`；`:54` `assert result.steer_input_rad == 0.2` |
| 2 | `test_direct_passes_the_input_through_on_every_channel`（`:57`） | `:61` `assert result.names == ("front_rack", "rear_rack")`；`:62` `assert result.angles_rad == (-0.17, -0.17)` |
| 3 | `test_ackermann_splits_the_reference_angle_about_the_turn_centre`（`:65`） | `:98` `assert result.angles_rad[0] == dc`；`:99` `assert result.angles_rad[1] == pytest.approx(expected_inner, rel=1e-12)`；`:100` `assert result.angles_rad[2] == pytest.approx(expected_outer, rel=1e-12)`；`:103` `assert abs(result.angles_rad[1]) > abs(result.angles_rad[0]) > abs(result.angles_rad[2])`；期望值在 `:90-92` 用 `math.atan` 独立重写：`radius = length / math.tan(abs(dc))`、`expected_inner = math.atan(length / (radius - track / 2.0))`、`expected_outer = math.atan(length / (radius + track / 2.0))` |
| 4 | `test_ackermann_at_zero_input_is_zero_rather_than_a_division`（`:106`） | `:112` `assert result.angles_rad == (0.0, 0.0)`（输入 `1e-15`） |
| 5 | `test_ackermann_rejects_a_geometrically_unreachable_pair`（`:115`） | `:117` `with pytest.raises(ValueError, match="geometrically unreachable"):` |
| 6 | `test_ackermann_with_one_channel_returns_the_reference_angle`（`:121`） | `:125` `assert result.angles_rad == (0.2,)` |
| 7 | `test_four_wheel_steer_turns_the_rear_wheels_against_the_front_at_low_speed`（`:128`） | `:139` `assert FOUR_WHEEL_STEER_LOW_SPEED_GAIN == -0.25`；`:140` `assert result.angles_rad[0] == 0.1`；`:141` `assert result.angles_rad[1] == pytest.approx(-0.25 * 0.1, rel=1e-12)`；`:142` `assert math.copysign(1.0, result.angles_rad[1]) != math.copysign(1.0, result.angles_rad[0])` |
| 8 | `test_four_wheel_steer_turns_the_rear_wheels_with_the_front_at_high_speed`（`:147`） | `:153` `assert FOUR_WHEEL_STEER_HIGH_SPEED_GAIN == 0.15`；`:154` `assert result.angles_rad[0] == 0.1`；`:155` `assert result.angles_rad[1] == pytest.approx(0.15 * 0.1, rel=1e-12)`；`:156` `assert math.copysign(1.0, result.angles_rad[1]) == math.copysign(1.0, result.angles_rad[0])` |
| 9 | `test_the_four_wheel_steer_crossover_is_the_caller_s_threshold`（`:161`） | `:178` `assert low.angles_rad[1] == pytest.approx(-0.025, rel=1e-12)`（4.9 m/s）；`:179` `assert high.angles_rad[1] == pytest.approx(0.015, rel=1e-12)`（5.0 m/s） |
| 10 | `test_four_wheel_steer_refuses_two_gains_of_the_same_sign`（`:182`） | `:184` `with pytest.raises(ValueError, match="rear_gain_low < 0 < rear_gain_high"):`（`rear_gain_low=0.2`） |
| 11 | `test_multi_axle_follow_places_every_axle_on_the_reference_circle`（`:195`） | `:229` `assert result.names == ("front_rack", "middle_rack", "third_rack")`；`:230` `assert result.angles_rad[0] == dref`；`:231` `assert result.angles_rad[1] == pytest.approx(math.atan(4200.0 / 2800.0 * math.tan(dref)), rel=1e-12)`；`:234` `assert result.angles_rad[2] == pytest.approx(math.atan(5600.0 / 2800.0 * math.tan(dref)), rel=1e-12)` |
| 12 | `test_multi_axle_follow_refuses_an_axle_that_is_not_behind_the_reference`（`:239`） | `:241` `with pytest.raises(ValueError, match="middle_rack"):`（`Li = 0.0`） |
| 13 | `test_multi_axle_follow_refuses_a_negative_distance_by_name`（`:253`） | `:255` `with pytest.raises(ValueError, match="middle_rack"):`（`Li = -100.0`） |
| 14 | `test_no_channels_allocate_nothing`（`:267`） | `:271` `assert result.names == ()`；`:272` `assert result.angles_rad == ()`；`:273` `assert result.law == "ackermann"`；`:274` `assert result.speed_mps == 7.0`；`:275` `assert result.steer_input_rad == 0.3` |
| 15 | `test_an_unknown_law_is_refused_by_name`（`:278`） | `:280` `with pytest.raises(ValueError, match="crab_steer"):` |
| 16 | `test_a_non_finite_input_is_refused`（`:284`，`law` 参数化 4 例） | `:288` `with pytest.raises(ValueError, match="steer_input_rad must be finite"):`（`math.inf` / `-math.inf` / `math.nan`） |
| 17 | `test_a_non_finite_speed_is_refused`（`:292`，`law` 参数化 4 例） | `:295` `with pytest.raises(ValueError, match="speed_mps must be finite"):` |
| 18 | `test_a_non_finite_channel_geometry_is_refused`（`:299`） | `:301` `with pytest.raises(ValueError, match="front_rack"):`（`wheelbase_mm=math.inf`） |

`--collect-only` 实测：`packages/suspension_multibody/tests/preparation/test_steering_allocator.py : 24 tests collected in 0.05s`（退出 0）。

### 2.2 `tests/preparation/test_steering_channels.py`（分配律落到准备层与求解器）

| 用例（行） | 断言表达式原文 |
|---|---|
| `test_the_preparation_builds_one_actuator_per_channel`（`:106`） | `:122` `assert prepared.steering.names == ("front_rack", "rear_rack")`；`:132` `assert prepared.steering.output.shape == (len(prepared.times), 2, 4)`；`:133` `assert prepared.steering.target.shape == (len(prepared.times) * 2,)`；`:134` `assert prepared.steering.target_rate.shape == (len(prepared.times) * 2,)`；`:137` `assert prepared.steering.body[0] != prepared.steering.body[1]`（两通道是两根不同的齿条）；`:139` `assert np.allclose(table[:, 0], table[:, 1])` |
| `test_direct_two_channel_run_drives_both_racks`（`:142`） | `:165` `assert run.status == "success", dict(run.failure_evidence)`；`:167` `assert output.shape[1] == 2`；`:173` `assert np.allclose(table[-1], expected_m)`；`:176` `assert abs(final - expected_m) < 1e-12, f"{name} reached {final} m"`（每通道各一条） |
| `test_four_wheel_steer_reaches_the_channels_it_drives`（`:184`，参数化低速/高速） | `:207` `assert table[-1, 0] == 0.02 * scale`；`:208` `assert table[-1, 1] == expected_gain * 0.02 * scale`；`:210` `assert [name for name, _series in prepared.steering_allocation] == ["front_rack", "rear_rack"]`；`:215` `assert front == 0.02`；`:216` `assert rear == expected_gain * 0.02`；`:219` `assert same_sign == (expected_gain > 0.0)` |
| `test_the_direct_law_is_what_a_one_channel_vehicle_gets`（`:222`） | `:242` `assert full_vehicle_model.allocation_law == "direct"`；`:243` `assert len(_steering_channel_specs(full_vehicle_model)) == 1`；`:255` `assert prepared.steering.names == ("front_rack",)`；`:263` `assert np.allclose(column, expected)`；`:264` `assert column[-1] == 0.016 / 1000.0`；`:265` `assert prepared.steering_allocation == ()`（direct 下不计算分配） |
| `test_the_two_channel_model_document_names_both_steering_elements`（`:268`） | `:279` `assert [entry["name"] for entry in steering] == ["steering_front_rack", "steering_rear_rack"]`；`:283` `assert {entry["target"] for entry in steering} == {"front_rack", "rear_rack"}`；`:286` `assert len(bodies) == 2` |

`--collect-only` 实测：`test_steering_channels.py : 8 tests collected in 0.53s`（退出 0）。

### 2.3 `tests/schema/test_vehicle_steering_channels.py`（第 7、8 项 schema 断言）

| 用例（行） | 断言表达式原文 |
|---|---|
| `test_the_model_still_has_twelve_dumped_keys`（`:41`） | `:45` `assert len(payload) == 12`；`:46-59` `assert set(payload) == {...12 键...}`；`:60` `assert "steering_channels" not in payload`；`:61` `assert "allocation_law" not in payload` |
| `test_the_declared_channel_adds_no_key_to_the_dump`（`:64`） | `:86` `assert len(payload) == 12`；`:87` `assert "steering_channels" not in payload`；`:88` `assert canonical_hash(payload) == FROZEN_ONE_CHANNEL_HASH`；`:89-91` `assert canonical_hash(declared.model_dump(mode="json")) == canonical_hash(full_vehicle_model.model_dump(mode="json"))` |
| `test_the_one_channel_hash_is_the_recorded_value`（`:94`） | `:98-101` `assert canonical_hash(full_vehicle_model.model_dump(mode="json")) == FROZEN_ONE_CHANNEL_HASH`（`FROZEN_ONE_CHANNEL_HASH` 常量在 `:36-38`，值 `75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9`） |
| `test_steering_remains_a_required_singleton`（`:104`） | `:106` `assert VehicleModel.model_fields["steering"].is_required()`；`:107` `assert "steering" in VehicleModel.model_fields`；`:108` `assert "steering_channels" in VehicleModel.model_fields`；`:110` `assert not VehicleModel.model_fields["steering_channels"].is_required()` |
| 第 8 项拒绝条件 | `:128`/`:130` `with pytest.raises(ValidationError):`（附加通道缺 `channel_name` / 缺 `placement`）；`:158` `pytest.raises(ValidationError, match="rear_rack")`（重复 `channel_name`）；`:174` `pytest.raises(ValidationError, match="front_rack")`（与主通道重名）；`:192` `pytest.raises(ValidationError, match="rear")`（重复 `placement`）；`:211` `pytest.raises(ValidationError, match="channel_name")`（空主 `channel_name`）；`:222` `pytest.raises(ValidationError, match="placement")`（空主 `placement`）；`:236` `pytest.raises(ValidationError, match="against itself")`（`actuator_body == actuator_reaction_body`）；`:250` `pytest.raises(ValidationError, match="rack_body")`（空 body 名） |
| `test_the_spec_is_a_public_schema_name`（`:259`） | `:263` `assert "SteeringChannelSpec" in schema.__all__`；`:264` `assert schema.SteeringChannelSpec is SteeringChannelSpec` |
| `test_channel_identity_is_not_read_off_a_placement_name`（`:267`） | `:290` `assert declared.steering_channels[0].placement == "middle"`（模型不按 `"front"/"rear"` 字面量猜身份） |

拒绝断言一律经 `VehicleModel.model_validate`（辅助函数 `_with_channels`，`:134-151`），**不用 `model_copy`**：`model_copy` 是浅拷贝且不跑校验器，用它做拒绝断言等于什么都没断言（该差别已写进辅助函数 docstring）。

`--collect-only` 实测：`tests/schema/test_vehicle_steering_channels.py : 14 tests collected in 0.12s`（退出 0）。

## 3. 相同方向盘输入下的数值对照表

实跑脚本（`uv run --no-sync python`，工作目录 = 仓库根，退出 0）：

* `direct`（`steer_input_rad = 0.2`）：单通道 `(0.2,)`；两通道 `(0.2, 0.2)`
* `ackermann`（`δc = 0.2`，`L = 2800 mm`，`t = 1600 mm`）：`R = 13812.833651643301`；三通道输出 `(0.2, 0.2119407426314249, 0.18931769732080145)`；独立重算的期望内/外侧 `0.2119407426314249` / `0.18931769732080145`；单通道 `(0.2,)`；`δc = 1e-15` → `(0.0, 0.0)`；顺序判据 `|in| > |c| > |out|` = `True`
* `four_wheel_steer`（`steer_input_rad = 0.1`，增益 `-0.25` / `0.15`）：`speed = 2.0` → `(0.1, -0.025)`；`speed = 20.0` → `(0.1, 0.015)`；阈值 5.0 的两侧 `4.9` → `(0.1, -0.025)`、`5.0` → `(0.1, 0.015)`
* `multi_axle_follow`（`δref = 0.1`，`Lref = 2800`，`Li = 4200 / 5600`）：`('front_rack','middle_rack','third_rack')` → `(0.1, 0.14938087291079097, 0.19803907709180682)`；独立重算 `0.14938087291079097` / `0.19803907709180682`
* 空通道（`law="ackermann"`，无通道）：`SteeringAllocation(names=(), angles_rad=(), law='ackermann', speed_mps=7.0, steer_input_rad=0.3)`

原样输出：

```
== laws, steer_input_rad = 0.2 ==
direct      1ch: (0.2,)
direct      2ch: (0.2, 0.2)
ackermann   3ch: (0.2, 0.2119407426314249, 0.18931769732080145)
   R = 13812.833651643301
   expected inner = 0.2119407426314249
   expected outer = 0.18931769732080145
   ordering |in|>|c|>|out|: True
ackermann   1ch: (0.2,)
ackermann  dc=1e-15: (0.0, 0.0)
== four_wheel_steer, steer_input_rad = 0.1 ==
gains: low = -0.25  high = 0.15
speed 2.0 : (0.1, -0.025)  rear = -0.025
speed 20.0: (0.1, 0.015)  rear = 0.015
crossover 4.9: (0.1, -0.025)
crossover 5.0: (0.1, 0.015)
== multi_axle_follow, dref = 0.1 ==
names : ('front_rack', 'middle_rack', 'third_rack')
angles: (0.1, 0.14938087291079097, 0.19803907709180682)
   expected mid   = 0.14938087291079097
   expected third = 0.19803907709180682
empty: SteeringAllocation(names=(), angles_rad=(), law='ackermann', speed_mps=7.0, steer_input_rad=0.3)
```

## 4. 分配律在整车路径上的落地点（含几何来源声明）

`preparation/vehicle_dynamic.py` 的新增件：

| 符号（行） | 职责 |
|---|---|
| `_steering_channel_specs(model)`（`:277`） | 有序逻辑通道列表 = 主通道（`enabled` 时）+ 声明序的附加通道中 `enabled=True` 的；单通道模型返回长度 1 |
| `_steering_allocation_angles(model, case, specs, steering_input, times)`（`:1444`） | `allocation_law == "direct"` 或 `specs` 为空 → 空（不计算分配）；否则逐采样点对参考通道调用 `allocate(...)`，车速取 `case.initial_forward_speed_mps`（本层唯一可得的车速，瞬时车速会让被积的 prescribed target 依赖正在积分的状态） |
| `_steering_channel_signals(specs, steering_input, times, allocation)`（`:1487`） | 无分配时每个通道都用 `case.steering_input`（单通道逐位兼容的来源）；有分配时每通道用自己那条角度序列，仍经该通道的 `input`/`ratio` 换算 |
| `_allocation_channels(model, specs)`（`:1509`） | 从模型取几何：`track_mm` = 该 placement 的轮对横向间距；`wheelbase_mm` = 该通道自身轴与另一个被放置轴的中心距；`distance_to_reference_mm` = 与参考通道轴的中心距（纵向读数表示跟车轴、横向读数表示车辙端点）。通道声明的 placement 若不被该模型放置，按名 `ValueError` 并列出实际放置 |

`distance_to_reference_mm` 的**符号含义重载**是设计的一部分，已在两处写出：`ackermann` 用它区分内侧/外侧（负=内侧、正=外侧、零拒绝），`multi_axle_follow` 用它表示跟车轴的距离（必须为正）。测试夹具 `write_vehicle_project` 的前后轴 `WHEEL_CENTER.x` 相同（实测前后均为 `x = 0.0`），故该夹具上 `distance_to_reference_mm == 0.0`，`ackermann` 对它按名拒绝（`ValueError: ackermann needs a positive wheelbase on 'front_rack'; found 0.0` 是 `L = 0` 的先行拒绝）；该夹具上能落地的律是 `direct` 与 `four_wheel_steer`（后者不使用 `distance_to_reference_mm`），测试即按此选择。`ackermann` / `multi_axle_follow` 的多几何断言由 `test_steering_allocator.py` 用显式几何量覆盖（那正是「几何由调用方给」的设计目的）。

## 5. 测试执行结果

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/preparation packages/suspension_multibody/tests/schema/test_vehicle_steering_channels.py -q -p no:cacheprovider
46 passed in 2.07s
EXIT=0
```

（24 + 8 + 14 = 46；`packages/suspension_multibody/tests/preparation/` 目录为本行新建。）
