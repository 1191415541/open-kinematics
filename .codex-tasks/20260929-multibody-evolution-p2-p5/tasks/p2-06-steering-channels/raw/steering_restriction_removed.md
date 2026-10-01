# p2-06 证据 (a)：两层后轮转向限制的解除

> 冻结基线：`packages/suspension_multibody/tests/data/` 全程未写（`git status --short -- packages/suspension_multibody/tests/data/` 为空，退出 0）。
> 改造前基准：`tasks/p2-01-freeze/raw/rear_steer_refusal.md` 记录的两层拒绝原文。

## 第一层：准备层的 `_validate_steering_topology`

### 改前原文（`git show HEAD:...preparation/vehicle_dynamic.py`，行 236–255）

```python
def _validate_steering_topology(facts: VehicleFacts) -> None:
    """
    Reject a second steerable rack instead of freezing it silently.

    One steering system steers one axle: the first placement drives the rack and
    every other axle's rack is bolted, which is what the native model can
    represent.  Which placements are which is a fact about the assembly, so this
    names the offending placements instead of reading an axle field.
    """
    steered, *trailing = facts.axles or ("",)
    unbolted = sorted(
        placement
        for placement in trailing
        if not facts.rack_fixed_to_chassis.get(placement, False)
    )
    if unbolted:
        raise ValueError(
            f"native vehicle dynamics actuates the rack of {steered!r} only; "
            f"rack_fixed_to_chassis must be true on {', '.join(unbolted)}"
        )
```

关键点：`steered, *trailing = facts.axles` —— **按装配顺序把第一个 placement 当作被驱动的那个**，其余 placement 一律要求 `rack_fixed_to_chassis` 为真。这是按「位置」而非按「声明」判定身份，正是 `EPIC.md` 行 233「装配层不得出现按名字猜身份的规则」的同族问题。

### 改后原文（现 `preparation/vehicle_dynamic.py:236-278`）

```python
def _validate_steering_topology(facts: VehicleFacts) -> None:
    """
    Judge the assembly's steering placements from the facts alone.

    This check used to refuse a second free rack.  It read the *first* axle in
    assembly order as the steered one and demanded that every other placement
    have its rack bolted to the chassis -- a `rack_fixed_to_chassis` requirement
    stated as a check *in the preparation layer* -- which enforced "one steering
    system steers one axle" by *position* rather than by declaration.  Subtask
    p2-06 removes that: a model may declare several steering channels, each
    naming the placement it steers, so which rack is driven is the model's own
    declaration and nothing a preparation may infer from ordering.  Keeping the
    refusal would refuse exactly the four-wheel-steered vehicle that channels
    exist for.

    What is left is what the assembly facts can actually decide: a placement may
    only appear once.  Two axles at one placement would make the placement an
    ambiguous name for a channel -- which is the *reason* the check exists, so it
    is stated as the condition rather than as a consequence of it.
    """
    placements = list(facts.axles)
    repeated = next(
        (name for index, name in enumerate(placements) if name in placements[:index]),
        None,
    )
    if repeated is not None:
        raise ValueError(
            f"the assembly places more than one axle at {repeated!r}; a placement "
            "names one axle, and a steering channel is addressed by it"
        )
```

函数名与位置保留（`tests/vehicle/test_service_contract.py:62` 断言它在 `__all__` 里，`:1965` 仍在）。新职责只做「装配事实上可判定」的检查：同一 placement 出现两次即按名报错。

去掉的原文断言：**「谁排第一谁被驱动」** —— 不再据此决定谁必须 bolted。新增的拒绝条件是「重复 placement」（`/platform 事务` 上的客观条件，与「谁是驱动轴」无关）。

### 调用点

`prepare_vehicle_run` 的 `_validate_steering_topology(facts)`（改前后都在 `:271` 附近）未改；解除的是被调函数体内的拒绝条款。函数名保留的原因是 `tests/vehicle/test_service_contract.py` 的 `PREPARATION_SYMBOLS` 元组（`:62`）断言它存在 —— 只允许增名，未删任何名。

## 第二层：文档导出层的强制写死

### 改前原文（`git show HEAD:...authoring/vehicle.py:112-119`）

```python
    axles = file_axles_from(assembly)
    # A vehicle states one steering system, and a steering system steers one axle:
    # the rear axle's rack is therefore bolted to the chassis rather than driven,
    # which is what `rack_fixed_to_chassis` records.  Deriving it here is what
    # makes "the file declares one steering system" mean one *steered* axle --
    # leaving the rear rack free would describe a four-wheel-steered car that the
    # document does not.
    axles["rear"] = axles["rear"].model_copy(update={"rack_fixed_to_chassis": True})
```

### 改后原文（现 `authoring/vehicle.py:120-132`）

```python
    axles = file_axles_from(assembly)
    # The axles come back exactly as their own subsystem files describe them,
    # including whether each rack is bolted to the chassis.  A vehicle may declare
    # several steering channels (subtask p2-06), so "the file states one steering
    # system" no longer means one *steered* axle, and bolting the rear rack down
    # here would silently refuse the second channel a document asked for.  Whether
    # a rack is bolted is a fact about the axle its subsystem file describes, and
    # that file is the one place that decides it.
```

**整段（含注释）删除**，`axles["front"]` / `axles["rear"]` 现原样返回：rear 的 rack 是否 bolted 由该轴自己的子系统文件决定（`subsystems/steering.py:199/239` 据此在 `WeldJoint` / `PrismaticJoint` 之间二选一）。

## 两条 grep 的退出码与输出原文

命令（工作目录 = 仓库根，bash）：

```
$ grep -rn "must be true" packages/suspension_multibody/src --include=*.py
$ echo "PY_EXIT=$?"
PY_EXIT=1
```

**退出码 1，输出为空**（零命中）。准备层已无「必须为真」的校验。

> 口径说明：未加 `--include=*.py` 时会命中 `__pycache__/*.pyc` 的历史字节码（二进制匹配），那不是源码命中；本行以源码为准，并已实测确认 `--include=*.py` 下零命中。G2 判据行 81 要求的是「准备层无『必须为真』的校验」，`preparation/vehicle_dynamic.py` 与 `authoring/vehicle.py` 均无。

```
$ grep -rn "rack_fixed_to_chassis" packages/suspension_multibody/src --include=*.py
packages/suspension_multibody/src/suspension_multibody/adams/full_vehicle_model.py:1670:        rack_fixed_to_chassis=rear_rack_fixed,
packages/suspension_multibody/src/suspension_multibody/adams/full_vehicle_model.py:2811:        rack_fixed_to_chassis=rear,
packages/suspension_multibody/src/suspension_multibody/schema/model.py:177:    rack_fixed_to_chassis: bool = False
packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py:191:    is the subset whose model declares compliance; ``rack_fixed_to_chassis`` maps a
packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py:206:    rack_fixed_to_chassis: Mapping[str, bool] = field(default_factory=dict)
packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py:228:        rack_fixed_to_chassis={
packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py:229:            entry.placement: bool(entry.axle.rack_fixed_to_chassis)
packages/suspension_multibody/src/suspension_multibody/subsystems/explicit.py:138:        if model.rack_fixed_to_chassis:
packages/suspension_multibody/src/suspension_multibody/subsystems/explicit.py:144:                name="rack_fixed_to_chassis",
packages/suspension_multibody/src/suspension_multibody/subsystems/steering.py:21:`model.rack_fixed_to_chassis`, which is a model field, and a template has no
packages/suspension_multibody/src/suspension_multibody/subsystems/steering.py:165:    the model's call rather than the template's -- `model.rack_fixed_to_chassis` --
packages/suspension_multibody/src/suspension_multibody/subsystems/steering.py:199:        if model.rack_fixed_to_chassis:
packages/suspension_multibody/src/suspension_multibody/subsystems/steering.py:239:        if model.rack_fixed_to_chassis:
packages/suspension_multibody/src/suspension_multibody/subsystems/steering.py:276:RACK_BOLTED_NAME = "rack_fixed_to_chassis"
packages/suspension_multibody/src/suspension_multibody/subsystems/types.py:239:        (`rack_fixed_to_chassis`), and a request that guessed would be choosing a
packages/suspension_multibody/src/suspension_multibody/templates/builtin.py:448:# `model.rack_fixed_to_chassis`, which is a *model* field, and a template has no
$ echo "EXIT=$?"
EXIT=0
```

**退出码 0**，命中 16 处源码，全部是「消费这个字段」而不是「要求它为真」：

| 命中位置 | 性质 |
|---|---|
| `schema/model.py:177` | 字段定义（`FrontAxleModel.rack_fixed_to_chassis`，归 p2-01 之后的现状，本行未碰） |
| `subsystems/assembler.py:191/206/228/229` | 把该字段读成装配事实 `VehicleFacts.rack_fixed_to_chassis` |
| `subsystems/steering.py:21/165/199/239/276` | `WeldJoint`/`PrismaticJoint` 二选一（角色相关段归 p4-02，本行未碰） |
| `subsystems/explicit.py:138/144` | 显式轴路径的同族二选一（本行未碰） |
| `subsystems/types.py:239` | 注释 |
| `templates/builtin.py:448` | 注释（该文件归 p4-02，本行未碰） |
| `adams/full_vehicle_model.py:1670/2811` | Adams 源模型导入时的赋值（本行未碰） |

**准备层与文档导出层已无任何「要求它为真」的命中。**

## 解除后的既有断言反转（改前 → 改后 → 理由）

### 1. `tests/authoring/test_vehicle_assembly_documents.py:218-220`

**改前**

```python
    # One steering system is one steered axle: the rear rack is bolted down.
    assert model.front_axle.rack_fixed_to_chassis is False
    assert model.rear_axle.rack_fixed_to_chassis is True
```

**改后**

```python
    # A rack is bolted because its own subsystem file says so, not because a
    # vehicle declares one steering system.  Both of this fixture's suspension
    # files leave it free, and the reader returns what they state.
    assert model.front_axle.rack_fixed_to_chassis is False
    assert model.rear_axle.rack_fixed_to_chassis is (
        file_axles_from(document)["rear"].rack_fixed_to_chassis
    )
```

**理由**：`is True` 断言的是 `authoring/vehicle.py:119` 的覆写结果。覆写删除后，rear 的取值等于它的子系统文件自己的声明 —— 而 `write_vehicle_project` 写出的 rear 子系统文件与 front 是同一份（`fixtures.py:440`），`rack_fixed_to_chassis` 默认 `False`。新断言检验的正是解除后的新契约：**等于文件自己的声明**（此处实测为 `False`），而不是一个固定字面量。`tests/authoring/test_vehicle_assembly_documents.py` 的 `vehicle_document_from(model) == section` 与 `again == model` 往返断言（`:241-244`）仍通过。

### 2. `test_a_bolted_rack_keeps_the_built_in_fixed_template`

**改前原文**

```python
def test_a_bolted_rack_keeps_the_built_in_fixed_template(tmp_path: Path) -> None:
    """
    The model decides *whether* a rack is steered; the file decides *how*.

    `vehicle_model_from` bolts the rear axle's rack down, because the document
    declares one steering system: that rack reaches the assembly as a weld to the
    chassis, while the front one is guided in the support the document's own steering
    template declares.  The tie rods are the *suspension* template's (requirement 1),
    so both axles build them and neither steering template names one.
    """
    ...
    assert "front_rack_guide" in names
    assert "rear_rack_fixed_to_chassis" in names
    assert "front_rack_tie_joint_L" in names and "rear_rack_tie_joint_L" in names
    assert not [name for name in names if name.endswith("rack_tie")]
```

**改后原文**（重命名为 `test_the_rack_branch_follows_the_subsystem_files`）

```python
def test_the_rack_branch_follows_the_subsystem_files(tmp_path: Path) -> None:
    """
    The file decides *whether* a rack is bolted; the model only reads it.

    This test used to be `test_a_bolted_rack_keeps_the_built_in_fixed_template`
    and asserted `"rear_rack_fixed_to_chassis" in names`, because
    `vehicle_model_from` bolted the rear axle's rack down on the ground that a
    document declaring one steering system declares one *steered* axle.  Subtask
    p2-06 removed that override: a vehicle declares its steering channels
    explicitly, so bolting a rack the file left free would silently refuse the
    rear channel a document asked for.  Measured on this fixture -- whose rear
    suspension file leaves `rack_fixed_to_chassis` free, as `write_vehicle_project`
    writes it -- both axles now build the guide branch, which is what the files
    state.  The other branch is still built, and still covered, by
    `tests/subsystems/test_explicit_in_composition.py::112` and
    `tests/subsystems/test_assembly_matches_snapshot.py`, which drive it from the
    model field directly.

    What the templates decide is unchanged: the *guide* comes from each axle's
    own steering template (the front through the support that template declares,
    the rear through the same one placed at `rear`), the tie rods are the
    *suspension* template's (requirement 1), and neither steering template names
    one.
    """
    ...
    assert "front_rack_guide" in names
    assert "rear_rack_guide" in names
    assert "front_rack_tie_joint_L" in names and "rear_rack_tie_joint_L" in names
    assert not [name for name in names if name.endswith("rack_tie")]
    # The branch is the *file's*: nothing bolts a rack the file left free.
    assert not [name for name in names if name == "rear_rack_fixed_to_chassis"]
```

**实测决定**（规格要求「先实测再定」）：`write_vehicle_project` 写出的 rear 子系统文件与 front 是同一份（`fixtures.py:440-442` 只改 `name` 与 `placement_role`），其 `rack_fixed_to_chassis` 为默认 `False`。因此解除覆写后 rear 的 rack **不 bolted**，`WeldJoint` 不再产生，`rear_rack_fixed_to_chassis` **不在** names 里，取而代之的是 `rear_rack_guide`。故按规格给出的第二分支改写：断言 `"rear_rack_guide" in names`。

实测原文（`compose_vehicle(model, "K", assembly_request_for(document)).constraints` 中名字含 `rack` 的全部条目）：

```
['front_rack_guide', 'front_rack_tie_joint_L', 'front_rack_tie_joint_R', 'rear_rack_guide', 'rear_rack_tie_joint_L', 'rear_rack_tie_joint_R']
```

**理由**：该用例原来的主张（「模板决定 how、模型决定 whether」）在解除后只剩「文件决定 whether」。welded 分支并未失去保护 —— `tests/subsystems/test_explicit_in_composition.py:112`（`assert [weld.name for weld in welds] == ["rack_fixed_to_chassis"]`）与 `tests/subsystems/test_assembly_matches_snapshot.py:40` 都直接从 model 字段驱动该分支，两者全绿。

## 既有转向用例的继续通过

| 用例 | 结果 |
|---|---|
| `tests/subsystems/test_steering_can_be_absent.py` | 通过（未改） |
| `tests/api/test_no_steering_shrinks_rack.py` | 通过（未改） |
| `tests/subsystems/test_assembly_matches_snapshot.py:40` | 通过（未改） |
| `tests/authoring/test_vehicle_assembly_documents.py:218-220` | 已按上表改写并通过 |
| `tests/vehicle/test_service_contract.py`（`__all__` 名字表） | 通过（只增名：`_steering_channel_specs`、`_steering_allocation_angles`、`_steering_channel_signals`、`_allocation_channels`、`_build_steering_channel`） |

## 未触碰项的自证

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空，退出 0）
```

`templates/roles.py`、`templates/builtin.py`、`subsystems/rig_link.py`、`rigs/**`、`mb_config/version.hpp`、`kernel/native.py` 的 ABI 常量均未出现在改动清单中（见 `run_log.md` 的 `git status --short` 原文）。
