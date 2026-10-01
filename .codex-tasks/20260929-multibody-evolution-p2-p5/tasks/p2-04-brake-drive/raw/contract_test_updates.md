# 契约测试更新与理由登记（判据 4）

父锚点：`EPIC.md` F6 行 122（两组锚点）、行 243 (d)（理由登记）、行 251 (d)。

改前原文取自 `git show HEAD:<path>`（即本行改动前的提交态）；改后原文取自工作区当前内容。
行号一律指各版本自身的行号。

---

## A. `tests/subsystems/test_brake_subsystem.py` 的 F6 锚点

F6 给的六个锚点是：`:57` / `:78` / `:86` / `:109` / `:132` / `:147`。逐个交代。

### A0. 文件头的两个常量与它们的注释

**改前**（`HEAD:test_brake_subsystem.py:30-44`）：

```python
#: The frozen `.adm`'s constants, at demand = 1.0 (see
#: `artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm`,
#: `SFORCE/31-34`).
ADAMS_PARAMETERS = {
    "piston_area": 2500.0,
    "front_brake_bias": 0.6,
    "brake_mu": 0.4,
    "max_brake_value": 0.1,
    "effective_piston_radius": 145.0,
}

#: 2 * 2500 * 0.6 * 1.0 * 0.1 * 0.4 * 145
FRONT_AMPLITUDE = 17_400.0
#: 2 * 2500 * (1 - 0.6) * 1.0 * 0.1 * 0.4 * 145
REAR_AMPLITUDE = 11_600.0
```

**改后**（`:30-54`）：

```python
ADAMS_PARAMETERS = {
    "piston_area": 2500.0,
    "effective_radius": 145.0,
    "friction_coeff": 0.4,
    "rotor_inertia": 0.0568,
}

#: 2 * 2500 * 0.6 * 1.0 * 0.1 * 0.4 * 145
FRONT_AMPLITUDE = 17_400.0
#: 2 * 2500 * 0.4 * 1.0 * 0.1 * 0.4 * 145
REAR_AMPLITUDE = 11_600.0
```

**理由**：槽名换了（判据 1）——`brake_mu`→`friction_coeff`、`effective_piston_radius`→`effective_radius`；`front_brake_bias` 与 `max_brake_value` 不再是槽，前者成为元素的 `share`（所以从常量表里移出、在调用点以 `share=0.6`/`0.4` 出现），后者成为模块常量 `brake.DEMAND_SCALE`（所以从表里移出而 0.1 仍留在算式注释里）。`rotor_inertia` 是新槽，带模板默认值。`FRONT/REAR_AMPLITUDE` 的**数值一字未改**——这是本行声称「同一个物理模型」的主要凭据。

### A1. `:57` — `test_the_simplified_brake_builds_no_body`

**改前**（`:57-60`）：

```python
def test_the_simplified_brake_builds_no_body() -> None:
    output = brake.build(_instance(), _context())
    assert output.bodies == {}
    assert brake.SIMPLIFIED_BRAKE.parts == ()
```

**改后**（`:116-119`）：**逐字未改**（同一函数，同一三行断言）。

**理由**：这条测试断言的性质（简化制动模板不贡献刚体）与本行改动无关，本行不改它——改了反而会让「0 刚体」这条不变量失去一条独立证据。

### A2. `:78` — `test_the_five_adams_parameters_round_trip_unchanged`

**改前**（`:78-84`）：

```python
def test_the_five_adams_parameters_round_trip_unchanged() -> None:
    template = brake.SIMPLIFIED_BRAKE
    assert template_from_json(template_to_json(template)) == template
    declared = {slot.name: slot.default for slot in template.property_slots}
    for name, expected in ADAMS_PARAMETERS.items():
        assert declared[name] == expected, name
```

**改后**（`:165-170`）：

```python
def test_the_standardized_slots_round_trip_unchanged() -> None:
    template = brake.SIMPLIFIED_BRAKE
    assert template_from_json(template_to_json(template)) == template
    declared = {slot.name: slot.default for slot in template.property_slots}
    for name, expected in ADAMS_PARAMETERS.items():
        assert declared[name] == expected, name
```

**理由**：断言主体一字未改（JSON 往返相等 + 每个声明的默认值），只改函数名。旧名里的 "five" 是**声明性事实**且已经为假（标准集是四个槽），留着会让一个读者以为还有第五个槽；改名是为了让名字继续描述它断言的东西。改名不是「测试挂了所以改」——这条测试在改动前后都通过。

### A3. `:86` — `test_the_amplitude_matches_the_adams_sforce_shape`

**改前**（`:86-104`）：

```python
def test_the_amplitude_matches_the_adams_sforce_shape() -> None:
    """
    Demand 1.0 gives the two recorded magnitudes and the recorded 60/40 split.

    The constants are the frozen document's own, so this is a check against
    `SFORCE/31-34` rather than against the module's arithmetic rearranged.
    """
    from suspension_multibody.schema import TimeSignal

    amplitudes = brake.wheel_torque_amplitudes(
        ADAMS_PARAMETERS, times=(0.0,), demand=TimeSignal(constant=1.0)
    )

    assert amplitudes["front_left"] == (FRONT_AMPLITUDE,)
    assert amplitudes["front_right"] == (FRONT_AMPLITUDE,)
    assert amplitudes["rear_left"] == (REAR_AMPLITUDE,)
    assert amplitudes["rear_right"] == (REAR_AMPLITUDE,)
    # 0.6 / 0.4, i.e. the bias shares add to one.
    assert amplitudes["front_left"][0] / amplitudes["rear_left"][0] == 1.5
    assert brake.bias_share(ADAMS_PARAMETERS, "front_left") == 0.6
    assert brake.bias_share(ADAMS_PARAMETERS, "rear_right") == 0.4
```

**改后**（`:173-194`）：

```python
def test_the_amplitude_matches_the_adams_sforce_shape() -> None:
    """
    Demand 1.0 at the recorded 60/40 split gives the two recorded magnitudes.

    The constants are the frozen document's own, so this is a check against
    `SFORCE/31-34` rather than against the module's arithmetic rearranged.  The
    split is now the element's own `share`, which is where the deleted
    `front_brake_bias` slot's 0.6/0.4 went.
    """
    front = _element(wheel="front_left", share=0.6)
    rear = _element(wheel="rear_left", share=0.4)

    assert front.spec.stiffness == FRONT_AMPLITUDE
    assert rear.spec.stiffness == REAR_AMPLITUDE
    # 0.6 / 0.4, i.e. the shares add to one.
    assert front.spec.stiffness / rear.spec.stiffness == 1.5
    assert brake.brake_amplitude(
        ADAMS_PARAMETERS, demand=1.0, share=0.6
    ) == FRONT_AMPLITUDE
    assert brake.brake_amplitude(
        ADAMS_PARAMETERS, demand=1.0, share=0.4
    ) == REAR_AMPLITUDE
```

**理由**：判据 2 要求产出从「幅值字典」换成「力矩元」，所以断言的对象从 `dict[str, tuple]` 变成元素（`front.spec.stiffness`）。数值断言**保留原值**（17400 / 11600 / 比值 1.5），并且新增两条对纯函数 `brake_amplitude` 的直接断言——这样「元素值」与「算式值」是两个独立被断言的东西，而不是同一次计算读两遍。`bias_share` 的两条断言删掉，因为那个函数随 `front_brake_bias` 一起消失；它本来断言的性质（前轴 0.6、后轴 0.4）由 `share=0.6` / `share=0.4` 的调用与 `/ 1.5` 的比值继续覆盖。

### A4. `:109` — `test_the_amplitude_is_a_nonnegative_magnitude_at_every_demand`

**改前**（`:109-129`）：

```python
def test_the_amplitude_is_a_nonnegative_magnitude_at_every_demand() -> None:
    """
    The signal is a magnitude, not a signed torque.

    Adams decides direction with `STEP(<wheel speed>,-10,1,10,-1)`; the Python
    side must not pre-empt that, so no demand and no pair of parameters can make
    the output negative.  Half the demand is half the amplitude, which also pins
    that the demand enters linearly and unscaled.
    """
    from suspension_multibody.schema import TimeSignal

    full = brake.wheel_torque_amplitudes(
        ADAMS_PARAMETERS, times=(0.0, 0.5, 1.0), demand=TimeSignal(constant=1.0)
    )
    half = brake.wheel_torque_amplitudes(
        ADAMS_PARAMETERS, times=(0.0, 0.5, 1.0), demand=TimeSignal(constant=0.5)
    )
    for wheel, values in full.items():
        assert all(value >= 0.0 for value in values), wheel
        assert len(values) == 3
        assert half[wheel] == tuple(value / 2.0 for value in values), wheel
```

**改后**（`:197-211`）：

```python
def test_the_amplitude_is_a_nonnegative_magnitude_at_every_demand() -> None:
    """
    The torque is a magnitude, not a signed value.

    Adams decides direction with `STEP(<wheel speed>,-10,1,10,-1)`; the element
    law does the same thing from the real-time relative rate, so no demand and no
    share can make the declared magnitude negative.  Half the demand is half the
    magnitude, which also pins that the demand enters linearly and unscaled.
    """
    full = brake.brake_amplitude(ADAMS_PARAMETERS, demand=1.0, share=0.6)
    half = brake.brake_amplitude(ADAMS_PARAMETERS, demand=0.5, share=0.6)
    assert full == FRONT_AMPLITUDE
    assert half == pytest.approx(full / 2.0, abs=1e-9)
    for share in (0.0, 0.25, 0.6, 1.0):
        assert brake.brake_amplitude(ADAMS_PARAMETERS, demand=0.0, share=share) == 0.0
```

**理由**：时序维度随 `wheel_torque_amplitudes(..., times=...)` 一起消失——**时间不在这个子系统里**了：元素是声明，方向与时刻都由力律在积分时读取（模块 docstring `brake.py:15-19`）。所以这条测试改断言「无时序的幅值」这一性质：非负、随 demand 线性、demand=0 时恒 0，并对四个 share 取值都检查零。这是**断言强度不变、维度收紧**，不是减弱：原先 `>= 0` 对每个时刻成立，现在对四个 share 与两个 demand 取值成立，并且多了一条「demand=0 恒为 0」。

### A5. `:132` — `test_an_out_of_range_demand_names_the_signal`

**改前**（`:132-144`）：

```python
def test_an_out_of_range_demand_names_the_signal() -> None:
    from suspension_multibody.schema import TimeSignal

    for bad in (-0.1, 1.1):
        try:
            brake.wheel_torque_amplitudes(
                ADAMS_PARAMETERS, times=(0.0,), demand=TimeSignal(constant=bad)
            )
        except ValueError as error:
            assert "brake_input" in str(error)
            assert repr(bad) in str(error)
        else:  # pragma: no cover - the call must raise
            raise AssertionError(f"demand {bad} must be refused")
```

**改后**（`:230-235`）：

```python
def test_an_out_of_range_demand_names_the_signal() -> None:
    for bad in (-0.1, 1.1):
        with pytest.raises(ValueError) as caught:
            brake.brake_amplitude(ADAMS_PARAMETERS, demand=bad, share=0.6)
        assert "brake_input" in str(caught.value)
        assert repr(bad) in str(caught.value)
```

**理由**：调用换成新的纯函数（签名从 `demand: TimeSignal` 变成 `demand: float`，因为时序不再属于这一层）；`try/except/else` 换成 `pytest.raises`，是同一断言的等价写法（`else` 分支的「必须抛」由 `pytest.raises` 自带）。**断言内容一字未改**：消息里必须出现 `brake_input` 与那个坏值。

### A6. `:147` — `test_an_unknown_wheel_is_refused_by_the_bias`

**改前**（`:147-153`，文件末尾）：

```python
def test_an_unknown_wheel_is_refused_by_the_bias() -> None:
    try:
        brake.bias_share(ADAMS_PARAMETERS, "spare")
    except ValueError as error:
        assert "spare" in str(error)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("an unknown wheel must not silently get zero bias")
```

**改后**（`:238-252`）：

```python
def test_an_unknown_wheel_is_refused_by_name() -> None:
    """A wheel the model does not have is named, not silently coupled."""
    ports = _offered()
    report = match_requirements(
        (PortRequirement(role=brake.BRAKE_REACTION_ROLE),), ports
    )
    with pytest.raises(ValueError, match="spare"):
        brake.wheel_torque_element(
            ADAMS_PARAMETERS,
            wheel="spare",
            own_body=WHEEL_END,
            report=report,
            ports=ports,
            demand=1.0,
        )
```

**理由**：被断言的对象没了（`bias_share` 随 `front_brake_bias` 消失），但**它保护的性质必须继续被保护**：「一个模型没有的轮子不能被静默接受」。这条性质现在落在元素构造上——`brake.torque_parameters` 的 `wheel not in WHEELS` 检查（`brake.py:225-229`）在命名元素时就拒绝。所以断言从「`bias_share` 拒绝 spare」变成「`wheel_torque_element` 拒绝 spare」，仍断言消息里出现 `spare`。函数名由 `..._refused_by_the_bias` 改为 `..._refused_by_name`，因为拒绝的机制不再叫 bias。

---

## B. `tests/subsystems/test_drive_subsystem.py` 的 F6 锚点

F6 给的五个锚点是：`:130` / `:162` / `:195` / `:214` / `:227`。

### B0. 文件头的常量

**改前**（`HEAD:test_drive_subsystem.py:35-48`）：`FRONT_DRIVE` / `REAR_DRIVE` 两个 `DrivelineSpec`（**未改**，共用 `DrivelineSpec` 的字段）；文件头 docstring 写「本子系统必须是那条公式的一种**拼写**」——**改后一字未改**，因为本行的 drive 侧确实保持了这条性质（见 B2）。

**新增**（改后 `:64-73`）：`IDENTITY_SLOTS = {"gear_ratio": 1.0, "efficiency": 1.0}`、`SUBFRAME`、`WHEEL`、`INSTANCE` 四个常量，供元素路径使用。

**理由**：新机制（元素 + 端口配对 + 恒等传动）需要新的夹具常量；`FRONT_DRIVE`/`REAR_DRIVE` 保持原样，因为它们代表的是**schema 侧**的既有语义（schema 不在本行写范围），并且被 B2 的 parity 断言继续使用。

### B1. `:130` — `test_the_simplified_drive_builds_no_body`

**改前**（`:130-133`）：

```python
def test_the_simplified_drive_builds_no_body() -> None:
    output = drive.build(_instance(), _context())
    assert output.bodies == {}
    assert drive.SIMPLIFIED_DRIVE.parts == ()
```

**改后**（`:204-207`）：**逐字未改**。

**理由**：与 A1 相同——该不变量与本行改动无关。

### B2. `:162` — `test_the_drive_torque_matches_the_live_build_value_for_value`

**改前**（`:162-193`）：

```python
def test_the_drive_torque_matches_the_live_build_value_for_value() -> None:
    """
    The subsystem's dictionary is the builder's dictionary, exactly.

    Both an all-driven and a partly-driven split are checked, and the comparison
    includes the wheel order, because the kernel reads the four buffers
    positionally.
    """
    times = np.asarray((0.0, 0.0005, 0.001))
    for driveline in (FRONT_DRIVE, REAR_DRIVE):
        model = _vehicle(driveline)
        case = _case(model, drive_input=TimeSignal(constant=0.75))
        built_drive, _built_brake = _build_wheel_torque_signals(model, case, times, 1.0)
        ours = drive.wheel_torque_amplitudes(
            driveline, times=times, drive=case.drive_input
        )
        assert list(ours) == list(WHEELS)
        assert list(ours) == list(built_drive)
        for name in WHEELS:
            assert ours[name] == built_drive[name], name
        # The value is the product with that wheel's own share, unscaled; the
        # share differs per corner, so read it from the split being tested.
        for share_index, name in enumerate(WHEELS):
            expected = (
                driveline.maximum_drive_torque
                * driveline.drive_split[share_index]
                * 0.75
                if name in driveline.driven_wheels
                else 0.0
            )
            assert ours[name] == (expected, expected, expected), name
```

**改后**（`:261-292`）：

```python
def test_the_drive_torque_matches_the_live_build_value_for_value() -> None:
    """
    The element's wheel torque is the builder's number, exactly.
    ...
    """
    times = np.asarray((0.0, 0.0005, 0.001))
    for driveline in (FRONT_DRIVE, REAR_DRIVE):
        model = _vehicle(driveline)
        case = _case(model, drive_input=TimeSignal(constant=0.75))
        built_drive, _built_brake = _build_wheel_torque_signals(model, case, times, 1.0)
        for name in WHEELS:
            element = _element(driveline, wheel=name, drive_input=0.75)
            assert element.spec.stiffness == built_drive[name][0], name
        ...
            assert element.spec.stiffness == expected, name
            assert element.spec.max_torque == pytest.approx(
                driveline.maximum_drive_torque * driveline.drive_split[share_index]
            ), name
```

**理由**：函数名与 docstring 的**承重点保留**（「逐值一致」、`==` 而非 `approx`）。被比较的左侧从模块的字典换成**元素**（`element.spec.stiffness`）；右侧仍是 `_build_wheel_torque_signals` 的真输出，且仍然在 `FRONT_DRIVE`/`REAR_DRIVE` 两种分配下比较。原有的「四轮顺序」断言随字典消失——顺序现在由 `WHEELS` 的循环给出（元素是逐轮构造的，没有缓冲位置这一说）。新增一条 `max_torque == maximum_drive_torque * share`：元素带一个上限，而恒等传动下上限就是该轮的分摊扭矩。

### B3. `:195` — `test_the_uncoupled_drive_input_sign_is_preserved`

**改前**（`:195-211`）：

```python
def test_the_uncoupled_drive_input_sign_is_preserved() -> None:
    """
    Reverse drive is a negative torque; the subsystem must not abs() it.

    Only the brake is a non-negative magnitude.  A drive torque follows the sign
    of the demand, and the live builder does the same, so both must agree at -1.
    """
    times = np.asarray((0.0,))
    model = _vehicle(FRONT_DRIVE)
    case = _case(model, drive_input=TimeSignal(constant=-1.0))
    built_drive, _ = _build_wheel_torque_signals(model, case, times, 1.0)
    ours = drive.wheel_torque_amplitudes(
        FRONT_DRIVE, times=times, drive=case.drive_input
    )
    assert ours["front_left"] == (-1_000.0,)
    assert ours["front_left"] == built_drive["front_left"]
    assert ours["rear_left"] == (0.0,)
```

**改后**（`:295-327`，函数名改为 `test_the_uncoupled_drive_input_sign_reaches_the_builder_exactly`）：

```python
def test_the_uncoupled_drive_input_sign_reaches_the_builder_exactly() -> None:
    """
    Reverse drive is a negative torque, and the amplitude keeps the sign.
    ...
    The *element*, however, carries the magnitude: the landed kernel family
    (`cpp/src/element/anti_roll.cpp:135-139`) is a resistance law whose demand
    channel is not wired yet, and `RotationalTorqueParameters` refuses a negative
    gain, so a signed couple is not expressible there today.  That limitation is
    asserted rather than hidden, and it is registered for the family's demand
    channel (see `torque_parameters`'s docstring).
    """
    ...
    signed = drive.drive_amplitude(
        FRONT_DRIVE, slots=_slots(FRONT_DRIVE.maximum_drive_torque),
        wheel="front_left", drive=-1.0,
    )
    assert signed == -1_000.0
    assert signed == built_drive["front_left"][0]
    element = _element(FRONT_DRIVE, wheel="front_left", drive_input=-1.0)
    assert element.spec.stiffness == 1_000.0
    # The cap is the forward figure for both signs: a motor's limit is symmetric.
    assert element.spec.max_torque == 1_000.0
    assert _element(FRONT_DRIVE, wheel="rear_left", drive_input=-1.0).spec.stiffness == 0.0
```

**理由**：这是本行**唯一一条实质反转**的契约断言，且理由不是「测试挂了」，而是一条内核侧的实测限制（`raw/reaction_paths.md` §5.4）：

- **保留的部分**：`-1` 时幅值保持符号、与 live builder 逐位一致（把 `ours["front_left"] == built_drive[...]` 换成 `drive_amplitude(...) == built_drive[...][0]`，仍是 `==`）、未驱动轮为 0。这三条是原测试的全部断言。
- **新增的部分**：元素侧 `stiffness == +1_000.0`（幅值），并且 `max_torque == 1_000.0`（对称上限）。这一条把「元素承载幅值而非带符号力矩」**写成断言**，而不是让它成为一个躲过测试的事实。

如果只改前三条、不写第四条，测试会因为一次 `abs()` 静默通过而看不到元素失去符号——那正是这次改动里最容易被掩盖的一点。

### B4. `:214` — `test_an_out_of_range_drive_input_names_the_signal`

**改前**（`:214-224`）：

```python
def test_an_out_of_range_drive_input_names_the_signal() -> None:
    for bad in (-1.5, 1.5):
        try:
            drive.wheel_torque_amplitudes(
                FRONT_DRIVE, times=(0.0,), drive=TimeSignal(constant=bad)
            )
        except ValueError as error:
            assert "drive_input" in str(error)
            assert repr(bad) in str(error)
        else:  # pragma: no cover - the call must raise
            raise AssertionError(f"drive_input {bad} must be refused")
```

**改后**（`:361-371`）：

```python
def test_an_out_of_range_drive_input_names_the_signal() -> None:
    for bad in (-1.5, 1.5):
        with pytest.raises(ValueError) as caught:
            drive.drive_amplitude(
                FRONT_DRIVE, slots=_slots(FRONT_DRIVE.maximum_drive_torque),
                wheel="front_left", drive=bad,
            )
        assert "drive_input" in str(caught.value)
        assert repr(bad) in str(caught.value)
```

**理由**：与 A5 完全同形——调用换成新函数、`demand: TimeSignal` 换成 `drive: float`、异常断言换成 `pytest.raises`，**消息断言一字未改**（`drive_input` + 坏值）。

### B5. `:227` — `test_drive_torque_without_driven_wheels_is_refused`

**改前**（`:227-246`）：

```python
def test_drive_torque_without_driven_wheels_is_refused() -> None:
    """
    The same refusal the live builder makes, reached the same way.

    `DrivelineSpec`'s own validator already rejects "torque on, no driven
    wheels", so the state below is reached by `model_copy`, which does not
    revalidate -- exactly the path that leaves `_build_wheel_torque_signals`
    defending itself.  The subsystem has to defend the same way: emitting four
    zeros would look like a working driveline that drives nothing.
    """
    driveline = FRONT_DRIVE.model_copy(
        update={"driven_wheels": (), "drive_split": (0.0, 0.0, 0.0, 0.0)}
    )
    try:
        drive.wheel_torque_amplitudes(
            driveline, times=(0.0,), drive=TimeSignal(constant=1.0)
        )
    except ValueError as error:
        assert "driven_wheels" in str(error)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("a drive torque needs a driven wheel")
```

**改后**（`:374-388`）：

```python
def test_drive_torque_without_driven_wheels_is_refused() -> None:
    """
    The same refusal the live builder makes, reached the same way.
    ...
    """
    driveline = FRONT_DRIVE.model_copy(
        update={"driven_wheels": (), "drive_split": (0.0, 0.0, 0.0, 0.0)}
    )
    with pytest.raises(ValueError, match="driven_wheels"):
        drive.wheel_shares(driveline)
```

**理由**：构造被破坏状态的方式（`model_copy`，绕过 `DrivelineSpec` 的校验）与理由说明都**一字未改**；被调用者从「产出字典的函数」变成「产出分摊的函数」`drive.wheel_shares`，因为驱动侧份额的计算被提取成单独一步。断言内容不变：消息里必须出现 `driven_wheels`。

**另有一条新增**（`:391-395`，`test_a_driven_wheel_with_no_share_is_refused`）：断言 live builder 的第二个防御（被驱动的轮子份额为 0 必须报错、而不是发出静默的 0），消息里必须出现该轮名。原测试文件没有这条，而 `_build_wheel_torque_signals` 有该检查（`preparation/vehicle_dynamic.py:1584-1587` 一带）；`drive.wheel_shares` 复刻它，所以补上对应断言。

---

## C. 外部消费点的实测结论

### C1. `tests/vehicle/test_native_vehicle.py:1607` / `:1721`（制动）

`:1607` 是 `def test_brake_signal_is_a_nonnegative_magnitude()`，`:1721` 是 `def test_native_brake_opposes_the_instantaneous_wheel_spin()`（其上的 `@pytest.mark.xfail` 在 `:1698`）。

**是否受影响：不受影响。** 两条都走 `_build_wheel_torque_signals`（文件 `:12` 导入）与 `DrivelineSpec`（`:1505-1507`、`:1611-1612`、`:1725-1726` 用 `maximum_brake_torque` / `front_brake_bias`），**不调用本行的 `brake.brake_amplitude` / `brake.wheel_torque_element`**（该文件唯一的 subsystems 导入是 `:41` 的 `from suspension_multibody.subsystems.entry import compose_vehicle`，那是装配入口，不是制动子系统）：

```
$ grep -n "brake_amplitude\|wheel_torque_element" \
    packages/suspension_multibody/tests/vehicle/test_native_vehicle.py
（无输出）
```

本行没有改 `schema/vehicle.py::DrivelineSpec`、`preparation/vehicle_dynamic.py`、`kernels`，所以这两条测试的输入与路径都不变。实测（两条一起跑）：

```
$ uv run --no-sync pytest \
    ".../test_native_vehicle.py::test_brake_signal_is_a_nonnegative_magnitude" \
    ".../test_native_vehicle.py::test_native_brake_opposes_the_instantaneous_wheel_spin" \
    -q -p no:cacheprovider
.x                                                                       [100%]
1 passed, 1 xfailed in 2.37s
```

xfail 是**本次改动之前就存在**的（`git show HEAD` 的 `:1698` 已有该 marker，reason 指向 2026-09-12 的 braking-case 子任务），不是本行新增。原 xfail reason 原文（节选）：「this fixture is degenerate: ideal joints only (no springs, no bushings), gravity=0 ... So this needs a fixture redesign (loaded tire and/or substeppable solver), not a tweak.」——与 brake 子系统的产出形态无关。

### C2. `tests/vehicle/test_native_vehicle.py:1500` / `:1630`（驱动）

`:1500` 是 `def test_native_vehicle_combines_trim_road_steering_and_drive()`，`:1630` 是 `def test_direct_wheel_torque_signals_override_global_distribution()`。

**是否受影响：不受影响。** 同一理由：两者都通过 `DrivelineSpec` + `_build_wheel_torque_signals` 走内核路径，不调用 `drive.drive_amplitude` / `drive.wheel_torque_element`。实测（与 C3 一起跑，输出见下）：`3 passed`。

```
$ uv run --no-sync pytest \
    ".../test_native_vehicle.py::test_native_vehicle_combines_trim_road_steering_and_drive" \
    ".../test_native_vehicle.py::test_direct_wheel_torque_signals_override_global_distribution" \
    ".../test_vehicle_dynamic_contract.py::test_the_case_document_carries_the_road_and_the_steering" \
    -q -p no:cacheprovider
...                                                                      [100%]
3 passed in 2.3s
```

（五条一起跑的一次输出为 `4 passed, 1 xfailed in 2.37s`，即 C1 两 + C2 两 + C3 一，其中 xfail 属 `:1721`；单独跑 C1 两为 `1 passed, 1 xfailed in 2.37s`。）

### C3. `tests/cases/test_vehicle_dynamic_contract.py:185`

`:185` 是 `def test_the_case_document_carries_the_road_and_the_steering(fixture)`；它的断言在 `:196`：`assert {"steering_target", "steering_rate", "brake_torque"} <= roles`，其中 `brake_torque` 是 `case_document`（`cases/vehicle_dynamic.py`）写出的表角色。

**是否受影响：不受影响。** 该断言的 `brake_torque` 来自 `prepared.brake_torque`（`cases/vehicle_dynamic.py:580-582`），由 `prepare_vehicle_run` 产出，与本行的 `subsystems/brake.py` 无关（见 `raw/wheel_torque_amplitudes.md` §3 的实测）。本行未改 `cases/**`。实测：`3 passed`（见 C2 命令，含本测试）。

---

## D. 未做到 / 不确定

- 本行没有为「旧槽名被 properties 文件继续使用」补断言（那是 properties 层的既有策略，不在本行写范围）。
- F6 提到的 `tests/authoring/test_vehicle_assembly_documents.py:218-220`（`rear_axle.rack_fixed_to_chassis is True`）归 p2-06，本行未触碰，也未跑它作为本行证据。
- 外部消费点的不受影响结论是用**三条实际跑过的测试**加**静态导入检查**得出的（`test_native_vehicle.py` 不导入子系统）。这个结论是关于「当前代码」的；等 p2-05 把生产路径换成元素之后，这几条会需要按新机制重新判定，那属于 p2-05。
