# p2-06 证据 (b)：转向通道列表与单通道向后兼容

> 硬门：`api.py:116`/`:284` 把 `model.model_dump(mode="json")` 哈希成 `Provenance.model_hash`；`io/artifacts.py` 再哈希一次进 artifact manifest。所以单通道声明的 `model_dump` 必须**键集合 / 字节数 / canonical_hash 三项逐项一致**（裁决 `1331b13d`）。

## schema 改前 / 改后字段形状对照

### `SteeringSystemSpec`（`schema/vehicle.py:148`）

| | 改前（`git show HEAD`，`:148-167`） | 改后（现 `:148-190`） |
|---|---|---|
| 序列化字段 | 13 个：`rack_body` `ratio` `max_rack_displacement` `input` `rack_displacement_per_steering_wheel_angle` `rack_stiffness` `rack_damping` `max_steering_angle` `actuator_mode` `actuator_body` `actuator_reaction_body` `actuator_axis_local` `actuator_reference_rotation` | **一个不改、一个不增、顺序不变**（同 13 个） |
| 非序列化运行元数据 | 无 | 追加 3 个 `exclude=True`：`channel_name`（默认 `"front_rack"`）、`placement`（默认 `"front"`）、`enabled`（默认 `True`） |
| `model_dump(mode="json")` 键集合 | 13 | **13（同上，逐字相同）** |

### 新增 `SteeringChannelSpec(SteeringSystemSpec)`（现 `:192-198`）

只覆盖两个字段为**必填**（去掉默认值）：

```python
class SteeringChannelSpec(SteeringSystemSpec):
    """One additional steering channel, beyond the compatibility primary."""

    channel_name: str = Field(min_length=1, exclude=True)
    placement: str = Field(min_length=1, exclude=True)
```

一个附加通道必须自报名字与放置：匿名通道无法被寻址，未声明放置的通道只能靠猜 rack —— 两者都在 `VehicleModel._topology` 的新校验里被拒。

### `VehicleModel`（现 `:295` 起）

12 个原字段**一个不增不减不改顺序/类型**。在 `steering` 之后插入两个：

| 新增字段 | 类型 | 默认 | exclude |
|---|---|---|---|
| `steering_channels` | `tuple[SteeringChannelSpec, ...]` | `()` | `True` |
| `allocation_law` | `Literal["direct","ackermann","four_wheel_steer","multi_axle_follow"]` | `"direct"` | `True` |

原 12 字段：`schema_version` `name` `units` `coordinate_system` `chassis` `front_axle` `rear_axle` `wheels` `steering` `driveline` `coordinate_couplers` `aerodynamic_drag`。

关键设计：**`steering` 仍是那个必填的单例**（`VehicleModel.model_fields["steering"].is_required() is True`），`steering_channels` 只承载「附加」通道。这样单通道声明的 `model_dump` 形状不可能漂移 —— 没有任何字段被改成列表。

### 新增校验（`VehicleModel._check_steering_channels`，`_topology` 的 `return self` 之前）

四条，全部只查**客观条件**，不按 `"front"`/`"rear"` 字面量猜身份：

1. 主通道的 `channel_name` 与 `placement` 去空白后非空；
2. 所有通道（主 + 附加）的 `channel_name` 互不重复 → 报错消息含重复的那个名字；
3. 所有通道的 `placement` 互不重复 → 报错消息含重复的那个名字；
4. 每个通道的 `rack_body` / `actuator_body` / `actuator_reaction_body`（非 `None` 时）去空白后非空，且 `actuator_body` 与 `actuator_reaction_body` 同时非 `None` 且相等时拒绝（「actuates X against itself」）。

**没有**加的规则：任何一条依据 placement 的字面值判断「谁是驱动轴」的规则 —— 那就是刚解掉的那条限制换了个层。

## 单通道 `model_dump` 的逐项实测对照表

实测脚本（`uv run --no-sync python`，模型 = `tests/conftest.py::full_vehicle_model` 的等价构造，即 12 字段默认值 + `SteeringSystemSpec(ratio=16.0)`）：

| 声明 | `len(model_dump(mode="json"))` | 顶层键集合 == 12 个原键 | `steering` 内部键数 | canonical JSON 字节数 | `canonical_hash` |
|---|---|---|---|---|---|
| 单通道（默认，改造前的等价声明） | 12 | 是 | 13 | 11835 | `75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9` |
| 单通道 + `allocation_law="ackermann"` | 12 | 是 | 13 | 11835 | 同上（逐字节相同） |
| 单通道 + 声明 1 个附加通道 | 12 | 是 | 13 | 11835 | 同上（逐字节相同） |

原样输出：

```
single channel (default): keys=12 bytes=11835 hash=75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
   steering inner keys = ['actuator_axis_local', 'actuator_body', 'actuator_mode', 'actuator_reaction_body', 'actuator_reference_rotation', 'input', 'max_rack_displacement', 'max_steering_angle', 'rack_body', 'rack_damping', 'rack_displacement_per_steering_wheel_angle', 'rack_stiffness', 'ratio']
single channel + law=ackermann: keys=12 bytes=11835 hash=75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
   steering inner keys = ['actuator_axis_local', 'actuator_body', 'actuator_mode', 'actuator_reaction_body', 'actuator_reference_rotation', 'input', 'max_rack_displacement', 'max_steering_angle', 'rack_body', 'rack_damping', 'rack_displacement_per_steering_wheel_angle', 'rack_stiffness', 'ratio']
two channels declared: keys=12 bytes=11835 hash=75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
   steering inner keys = ['actuator_axis_local', 'actuator_body', 'actuator_mode', 'actuator_reaction_body', 'actuator_reference_rotation', 'input', 'max_rack_displacement', 'max_steering_angle', 'rack_body', 'rack_damping', 'rack_displacement_per_steering_wheel_angle', 'rack_stiffness', 'ratio']

bytes identical single vs +law   : True
bytes identical single vs 2chan  : True
steering_channels present in dump: False
```

顶层 12 键（逐字）：`aerodynamic_drag` `chassis` `coordinate_couplers` `coordinate_system` `driveline` `front_axle` `name` `rear_axle` `schema_version` `steering` `units` `wheels`。

`75cb96f5…03fd9` 这个值在**开工前**（HEAD 状态）就已实测为同一值，故它是「改造前」的实测值而非事后回填：改造前的 `model_dump` 与我改动后逐个对照过键集合与字节数，都是 12 / 11835 / 同哈希。

> **`_stated` 的多出键问题（规格第 3 条要求确认）**：`authoring/vehicle.py::_stated` 会把 `exclude=True` 的字段也导出，所以 `vehicle_document_from(model)["steering"]` 现在**多了 `channel_name`/`placement`/`enabled` 三个键**。这是对的（文件是模型描述而非 hash 描述），而且**既有往返测试不受影响**：
>
> - `tests/authoring/test_vehicle_assembly_documents.py:244` 断言的是 **section 的顶层键**（`set(section) == {"chassis","wheels","steering","driveline"}`），不是 `steering` 内部；实测该用例通过。
> - 往返本身（`:236-244`：`again = vehicle_model_from(...)`、`assert again == model`、`assert vehicle_document_from(again) == section`）也通过 —— 导出的三个键被 `vehicle_model_from` 原样读回，固定点成立。
> - `vehicle_document_from` 只在 `model.steering_channels` 非空时追加 `"steering_channels"` 键，所以单通道导出**不多出顶层键**。
>
> 因此无需把三个字段的默认值改成「与文件缺省一致的读法」，`PROGRESS.md` 登记该结论。

## `model_hash` 的三层实测

| 层 | 检查 | 实测 |
|---|---|---|
| 键集合 | `len(model_dump(mode="json")) == 12`，且不含 `steering_channels` / `allocation_law` | 通过（新增测试 `tests/schema/test_vehicle_steering_channels.py::test_the_model_still_has_twelve_dumped_keys`） |
| 字节数 | canonical JSON = 11835 字节，声明附加通道后不变 | 通过（`...::test_the_declared_channel_adds_no_key_to_the_dump`） |
| canonical_hash | `== 75cb96f5…03fd9` | 通过（`...::test_the_one_channel_hash_is_the_recorded_value`，常量 `FROZEN_ONE_CHANNEL_HASH` 写死） |

第 8 项的每条拒绝断言也各有实测（`tests/schema/test_vehicle_steering_channels.py`）：

| 拒绝条件 | 用例 | 断言表达式 | 行 |
|---|---|---|---|
| 重复 `channel_name`（附加通道之间） | `test_two_channels_with_one_name_are_refused_by_name` | `pytest.raises(ValidationError, match="rear_rack")` | `:145` |
| 重复 `channel_name`（附加 vs 主） | 同上 | `pytest.raises(ValidationError, match="front_rack")` | `:161` |
| 重复 `placement` | `test_two_channels_at_one_placement_are_refused_by_name` | `pytest.raises(ValidationError, match="rear")` | `:184` |
| 空主 `channel_name` | `test_a_blank_primary_name_is_refused` | `pytest.raises(ValidationError, match="channel_name")` | `:203` |
| 空主 `placement` | `test_a_blank_primary_placement_is_refused` | `pytest.raises(ValidationError, match="placement")` | `:214` |
| `actuator_body == actuator_reaction_body` | `test_an_actuator_against_itself_is_refused` | `pytest.raises(ValidationError, match="against itself")` | `:229` |
| 空 `rack_body` | `test_an_empty_body_name_is_refused` | `pytest.raises(ValidationError, match="rack_body")` | `:253` |
| 附加通道缺 `channel_name` / 缺 `placement` | `test_a_channel_name_is_stated_by_the_additional_channel` | 两个 `pytest.raises(ValidationError)` | `:130`/`:132` |

`SteeringChannelSpec` 已加入 `schema/__init__.py` 的 `from .vehicle import (...)`（`:74`）与 `__all__`（`:132`），实测 `"SteeringChannelSpec" in suspension_multibody.schema.__all__` 为真（`test_the_spec_is_a_public_schema_name`）。

## 单通道产物的逐项一致（准备层）

规格要求「单通道时产出的 `_VehicleSteeringBuffers` 与改造前逐项相同」。实测（文件夹具车辆 + `steering_input=TimeSignal(times=(0.0,0.002), values=(0.0,3.0))`）：

| 数组 | 改造前 | 改造后 | 判据 |
|---|---|---|---|
| `names` | `("front_rack",)` | `("front_rack",)` | 硬编码改为 `steering_spec.channel_name`，主通道默认 `"front_rack"`，故相同 |
| `actuator_type` | `(1,) int32` | `(1,) int32` | 相同 |
| `body` / `reaction_body` | `(1,) int32` | `(1,) int32` | 相同 |
| `point_local` / `reaction_point_local` / `axis_local` | `(1,3) float64` | `(1,3) float64` | 相同 |
| `reference_quaternion` | `(1,4) float64` | `(1,4) float64` | 相同 |
| `target` / `target_rate` | `(n,) float64` | `(n,) float64` | 相同 |
| `stiffness` / `damping` | `(1,) float64` | `(1,) float64` | 相同 |
| `output` | `(n,1,4) float64` | `(n,1,4) float64` | 相同 |

`len(prepared.steering.names) == 1`，`prepared.steering_allocation == ()`（`direct` 法则下不计算分配）。

**数值不变的最强证据**：`dynamic_hash_sentinel.py --check` 的 26 个 artifact **逐字节一致**（退出 0，原文见 `run_log.md`），其中含整车侧 8 例；`case_parity_check.py`（无参数）的 `vehicle_dynamic` 一行报 **`8 cases, bit-identical to the frozen snapshot`**；`vehicle_kc` 一行报 `grid matches an independent expansion; 10 mm reaches every wheel drive`。

## 既有 `__all__` 变化

`tests/vehicle/test_service_contract.py` 的 `PREPARATION_SYMBOLS` 与 `__all__` 的关系：**只增名，未删名**。新增 5 个：`_steering_channel_specs`、`_steering_allocation_angles`、`_steering_channel_signals`、`_allocation_channels`、`_build_steering_channel`。该用例 `test_vehicle_legacy_definition_map`（断言 46 项计数）与全套 `tests/vehicle/` 通过（462 passed / 1 xfailed）。
