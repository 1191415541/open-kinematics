# 反力路径断言与零命中证据（判据 3）

父锚点：`EPIC.md` 行 24（路线图 2.1）、行 233（禁止按名字猜身份）、行 243 (c)、行 251 (c)。

## 1. 机制：反力体由端口配对决定（复用 p2-03，未另造）

`subsystems/brake.py` 与 `drive.py` 都不含任何体名规则；两者都只声明一个**需求角色**常量，把 `report`（`match_requirements` 的返回值）与 `ports`（被匹配的候选端口表）交给 p2-03 的配对函数：

- `subsystems/brake.py:270-278`
  ```python
  from ..compilation.element_blocks import pair_torque_bodies, torque_element_row

  pairing = pair_torque_bodies(
      name=f"brake_{wheel}",
      role=BRAKE_REACTION_ROLE,
      own_body=own_body,
      report=report,
      ports=ports,
  )
  ```
  需求角色常量：`subsystems/brake.py:118` `BRAKE_REACTION_ROLE = "brake_reaction"`。
- `subsystems/drive.py:279-287` 同形，角色常量在 `subsystems/drive.py:99` `DRIVE_REACTION_ROLE = "drive_reaction"`。

配对函数本体在 `compilation/element_blocks.py:133-205`（`pair_torque_bodies`）：反力体取自 `port.owner.local`（`element_blocks.py:154-158` 的 docstring 明说），行构造在 `compilation/element_blocks.py:211-228`（`torque_element_row`），`body_a = pairing.reaction_body`、`body_b = pairing.driven_body`（`element_blocks.py:226-227`）。

两个需求角色都是**角色名，不是体名**——这一点很重要：`brake_reaction` / `drive_reaction` 说「谁提供这个接口」，不说「谁叫这个名字」，所以卡钳支架、转向节、副车架、车身都能满足它。

## 2. 制动反力：断言原文与通过输出

测试文件：`packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py`。

### 2.1 反力体读自被匹配的端口（`:272-286`）

```python
def test_the_reaction_body_comes_from_the_matched_port() -> None:
    """
    The reaction body is whatever the matched port says its owner is.

    Two different owners are run through the *same* call, one of them the name
    the roadmap uses for a brake's reaction (`upright`) and one a caliper
    carrier: nothing here can be reading a name, because both work and neither
    name appears in the production path.
    """
    caliper = _element(owner=CALIPER)
    upright = _element(owner="upright_L")
    assert caliper.body_a == CALIPER
    assert upright.body_a == "upright_L"
    # The driven end is the requiring side's own body both times.
    assert caliper.body_b == upright.body_b == WHEEL_END
```

`CALIPER = "caliper_carrier_L"`（`:63`）、`WHEEL_END = "wheel_carrier"`（`:64`）——**都是包内任何表里都没有的名字**，所以断言成立只能是因为端口拥有者被读出，不能是因为查表。

**被读出的量**：`element.body_a`（行上的反力端）与 `element.body_b`（行上的受力端）。`body_a` 每次都等于「提供端口那个实体」的名字，`body_b` 每次都等于调用方传进来的 `own_body`。

### 2.2 无端口时按角色名拒绝（`:289-309`）

```python
def test_a_missing_reaction_port_is_refused_by_role() -> None:
    """
    A wheel the assembly never bound a reaction port for fails by role.
    ...
    """
    ports = _offered()
    report = match_requirements(
        (PortRequirement(role="some_other_need", required=False),), ports
    )
    assert report.bindings == ()
    assert report.binding_for(brake.BRAKE_REACTION_ROLE) is None
    with pytest.raises(ValueError, match=brake.BRAKE_REACTION_ROLE):
        brake.wheel_torque_element(...)
```

`pair_torque_bodies` 的拒绝点：`compilation/element_blocks.py:172-177`（`BindingError`，`ValueError` 子类），消息点名需求角色，不退回「调用方恰好传进来的那个体」。

### 2.3 构造路径零体名规则（`:316-327`）

```python
def test_the_brake_path_names_no_body_and_no_reacting_part() -> None:
    ...
    source_root = Path(__file__).parents[2] / "src" / "suspension_multibody"
    for relative in ("subsystems/brake.py", "subsystems/drive.py"):
        text = (source_root / relative).read_text(encoding="utf-8").lower()
        for name in ("upright", "chassis"):
            assert name not in text, f"{relative} names {name!r}"
```

这条测试读**生产源码文本**本身——「按名字猜身份」这种缺陷读者找得出来、调用测不出来，所以检查对象是文件而不是返回值。

通过输出：

```
$ uv run --no-sync pytest \
    "packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py::test_the_reaction_body_comes_from_the_matched_port" \
    "packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py::test_a_missing_reaction_port_is_refused_by_role" \
    "packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py::test_the_brake_path_names_no_body_and_no_reacting_part" \
    "packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py::test_the_reaction_body_comes_from_the_matched_port" \
    "packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py::test_the_drive_path_names_no_body_and_no_reacting_part" \
    -v -p no:cacheprovider
packages\suspension_multibody\tests\subsystems\test_brake_subsystem.py::test_the_reaction_body_comes_from_the_matched_port PASSED [ 20%]
packages\suspension_multibody\tests\subsystems\test_brake_subsystem.py::test_a_missing_reaction_port_is_refused_by_role PASSED [ 40%]
packages\suspension_multibody\tests\subsystems\test_brake_subsystem.py::test_the_brake_path_names_no_body_and_no_reacting_part PASSED [ 60%]
packages\suspension_multibody\tests\subsystems\test_drive_subsystem.py::test_the_reaction_body_comes_from_the_matched_port PASSED [ 80%]
packages\suspension_multibody\tests\subsystems\test_drive_subsystem.py::test_the_drive_path_names_no_body_and_no_reacting_part PASSED [100%]

============================== 5 passed in 1.44s ==============================
```

## 3. 驱动反力：断言原文

测试文件：`packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py:409-423`。

```python
def test_the_reaction_body_comes_from_the_matched_port() -> None:
    """
    The reaction body is whatever the matched port says its owner is.

    Two owners are run through the *same* call -- the roadmap's subframe and a
    vehicle body -- and neither name appears in the production path, so the
    answer cannot be a name rule.
    """
    subframe = _element(FRONT_DRIVE, wheel="front_left", drive_input=1.0)
    body = _element(
        FRONT_DRIVE, wheel="front_left", drive_input=1.0, owner="vehicle_body"
    )
    assert subframe.body_a == SUBFRAME
    assert body.body_a == "vehicle_body"
    assert subframe.body_b == body.body_b == WHEEL
```

`SUBFRAME = "subframe"`（`:71`）、`WHEEL = "wheel_front_left"`（`:72`）。驱动侧的「副车架或车身」两个落点在同一次调用形态下都被走到，断言的就是被读出的 `body_a`。

零体名规则断言：`tests/subsystems/test_drive_subsystem.py:426-437`，读 `subsystems/drive.py` 的文件文本，对 `upright` / `chassis` 各断言一次不出现。

## 4. `grep -ic upright` / `grep -ic chassis` 命中数：0

```
$ for f in .../subsystems/brake.py .../subsystems/drive.py; do
    for pat in upright chassis; do echo "$f grep -ic $pat -> $(grep -ic $pat "$f")"; done
  done
brake.py  grep -ic upright -> 0
brake.py  grep -ic chassis -> 0
drive.py  grep -ic upright -> 0
drive.py  grep -ic chassis -> 0
```

本行改动段（`git diff -U0` 的**新增行**）里的命中数：

```
brake.py                     -> 0
drive.py                     -> 0
templates/builtin.py         -> 0
templates/roles.py           -> 0
subsystems/types.py          -> 0
```

`templates/builtin.py` 改动的 `_BRAKE_MOUNTS`/`BRAKE`/`_DRIVE_MOUNTS`/`DRIVE` 段（`:596-662`）内：`upright` 0、`chassis` 0。`templates/roles.py` 内 `chassis` 只出现在 `_check_roles`（`:158-166`）的六角色集合里，那是**角色名**且归 p4-02，不在本行改动范围；本行对 `roles.py` 的改动段（两个 `RoleSpec` 条目）命中 0。

## 5. 登记：未接线与内核缺口（不绕过）

### 5.1 组合层还没有调用点

本行交付的是「子系统侧能产出力矩元」，而 `subsystems/composition.py`（归 p4-02）与 `preparation/vehicle_dynamic.py`（归 p2-05）**尚未**调用 `brake.wheel_torque_element` / `drive.wheel_torque_element`：

```
$ grep -rn "wheel_torque_element\|BRAKE_REACTION_ROLE\|DRIVE_REACTION_ROLE" \
    packages/suspension_multibody/ --include=*.py | grep -v tests/
（只有 brake.py / drive.py 自身的定义与导出）
```

因此「制动反力传到 upright / 卡钳支架」「驱动反力传到副车架 / 车身」目前是**子系统层可证的性质**（端口拥有者被读出、两个不同拥有者都成立），还**不是**一条生产装配路径上跑出来的结果。把元素接到装配上、并在装配层声明 `needs` 的那一步不在本行写范围内。

### 5.2 模板上的 `needs` 会被导出/读回丢掉（实测）

上面的「装配层还需要一个 `needs` 声明」并不是顺手就能加的。`Template` 有 `needs` 字段（`templates/model.py:312-313`），但文件格式的导出函数**不写它**：

```
$ uv run --no-sync python -c "
from suspension_multibody.authoring.solver import template_document_from
from suspension_multibody.templates.builtin import BRAKE, DRIVE
for t in (BRAKE, DRIVE):
    doc = template_document_from(t)
    print(t.name, 'has needs key:', 'needs' in doc)
    print('  slots:', [(s['name'], s.get('default'), s['required']) for s in doc['property_slots']])
"
brake_4wdisk_simplified has needs key: False
  slots: [('piston_area', 2500.0, True), ('effective_radius', 145.0, True), ('friction_coeff', 0.4, True), ('rotor_inertia', 0.0568, True)]
powertrain_simplified has needs key: False
  slots: [('gear_ratio', 1.0, True), ('efficiency', 1.0, True), ('max_torque', 0.0, True)]
EXIT=0
```

`template_document_from`（`authoring/solver.py:831-963`）写 `hardpoints`/`joints`/`elements`/`property_slots`/`outputs`/`ports`/`suspension_kind`/`description`，**没有** `needs`；而运行时模板侧是有这个字段的（`templates/model.py:509-519` 的序列化、`:611-621` 的反序列化）。所以在内置模板上声明 `needs` 会被导出/读回这一圈丢掉，读回来的是一个**看起来一样、行为不一样**的模板。本行**没有**加 `needs`，理由是上面的实测，不是遗漏。

（注：导出的 `hardpoints` 只有 `wheel_center` 与 `spin_axis`——即 `_BRAKE_MOUNTS`/`_DRIVE_MOUNTS` 声明的两个 role，每个 role 写一次；这与 `solver.py:851-860` 的注释一致。）

### 5.3 内核文档路由不接受 `rotational_torque`（内核侧缺口，本行不改）

`packages/suspension_kernel/cpp/src/cases/contract_model.cpp:831` 的元素类型分派没有 `rotational_torque` 分支：

```
return fail(error, "element " + quote(*element_name) + " has unsupported type " + ...
```

主管实测的报错原文：`model document: element "..." has unsupported type "rotational_torque"`。
```
$ grep -rn "rotational_torque" \
    packages/suspension_kernel/cpp/src/cases/contract_model.cpp \
    packages/suspension_kernel/cpp/src/contract/contract_registry.cpp
（无输出）
```

后果：一旦组合层把本行的元素写进 `model_document`，内核会在读文档时拒绝它。**这不是本行的修复对象**（`packages/suspension_kernel/**` 不在写范围），只是登记。因为 5.1 的组合层还没接线，本行任何已执行的验收命令都没有触发它。

### 5.4 驱动元素承载幅值而非带符号力矩（内核力律限制，本行不改）

`cpp/src/element/anti_roll.cpp:132-139` 的 `demand` 至今硬编码 `1.0`，力偶方向由实时相对角速度决定（`rate > kEps` 给 `-magnitude`，`rate < -kEps` 给 `+magnitude`）——那是一条**阻力**律。所以一个基于该族的元素今天只能把「幅值」施加在两侧相对运动的反方向上：

- 制动语义正确（制动本来就是阻力）；
- 驱动语义在**方向**上是错的：驱动力矩应与相对角速度同向（加速），而该族会减速。反向驱动更是与正向驱动不可区分。

`RotationalTorqueParameters.__post_init__`（`modeling/primitives/elements.py:661-670`）对
`stiffness` / `max_torque` / `damping` 逐个要求「有限且非负」，负的 `stiffness` 直接抛
`ElementError`：

```python
for label, value in (("stiffness", self.stiffness), ("max_torque", self.max_torque),
                     ("damping", self.damping)):
    if not np.isfinite(value) or value < 0.0:
        raise ElementError(f"rotational torque {label} must be finite and non-negative")
```

本行的处置是**断言这条限制而不是掩盖它**：

- `drive.drive_amplitude` 保留 live builder 的带符号值，与 `preparation/vehicle_dynamic.py::_build_wheel_torque_signals` **逐位一致**（`tests/subsystems/test_drive_subsystem.py:295-327`，`==` 而非 `approx`）；
- `drive.torque_parameters` 承载绝对值，并在 `subsystems/drive.py:219-233` 把上述限制与出处写清；
- 该限制的归属是**内核力律的 demand 通道**（p2-02 的家族接线），登记在此，不在多体层绕过（例如不在 `drive.py` 里手动翻符号——那会让元素在任何速率下都朝同一个方向使劲，比「减速」更错）。

### 5.5 `ELEMENT_KINDS` 补登

`subsystems/types.py::ELEMENT_KINDS` 原本没有 `"rotational_torque"`（p2-03 的写范围不含该文件）。本行补齐并附注册说明（`subsystems/types.py:100-119`），使「一个行类型属于 `ELEMENT_KINDS`」对制动/驱动元素成立。这是**补充登记**，没有改动已有的种类顺序或语义。

## 6. 未做到 / 不确定

- 组合层未接线 ⇒ 上面 §2/§3 的断言证明的是**子系统构造路径**的反力来自端口，而非一条端到端装配跑出来的反力。这条差距需要 p4-02（`composition.py`）与 p2-05（`preparation/vehicle_dynamic.py`）接上后才会闭合。
- §5.2 的 `needs` 缺口意味着「接口声明落在模板上」这件事今天做不到；它需要有行去扩 `template_document_from` 的导出（含 `needs`）并配套测试，那不在本行写范围内。
- §5.3 与 §5.4 是内核侧缺口，本行只登记。
