# 属性槽标准化（判据 1）

任务：`.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/`
父锚点：`EPIC.md` G1、行 24（路线图 2.1 槽清单）、F1 行 112、行 227（角色条目可改范围）。

## 1. 改前 → 改后 字段对照

### 1.1 BRAKE

落点：`templates/builtin.py` `BRAKE.property_slots`（改前 `:599-609`，改后 `:603-628`）；
`templates/roles.py` `ROLES["brake"].required_slots`（改前 `:126-132`，改后 `:125-130`）。

| 改前槽名 | 单位 | 默认值 | 改后 | 理由 |
|---|---|---|---|---|
| `brake_mu` | `-` | 0.4 | **删除**，改为 `friction_coeff` | 纯改名：同一个物理量（摩擦片对盘面的摩擦系数），0.4 原值不动。旧名是 Adams `.adm` 里 `MU` 的转写习惯，新名是路线图 2.1 的标准化名。 |
| `effective_piston_radius` | `mm` | 145.0 | **删除**，改为 `effective_radius` | 纯改名，145.0 原值不动；`effective_radius` 是路线图 2.1 的标准化名，语义（卡钳等效作用半径）未变。 |
| `piston_area` | `mm^2` | 2500.0 | 保留（名与值都不动） | 路线图 2.1 已经用它，旧名就是标准名，没有改的理由。 |
| `max_brake_value` | `-` | 0.1 | **删除**，降级为模块常量 `brake.DEMAND_SCALE = 0.1` | 它**不是制动器的物理属性**，而是源文档 `SFORCE/31-34` 对驱动输入的归一化系数（同一个 0.1 出现在制动力与驱动力两侧）。留在槽集合里会让「一个模板声明自己的物理参数」这件事失真；直接删掉又会让所有记录幅值整体 ×10（物理变化而非改名），所以它必须继续存在，只是换个归属：`subsystems/brake.py:120-125`，并在模块 docstring `:41-46` 记录它的出处**未能核实**（`.sub` 源文件不在本仓库）。 |
| `front_brake_bias` | `-` | 0.6 | **删除**，改为元素的 `share` 调用参数 | 一个轮子一个力矩元之后，前/后分配是**同一份 demand 如何在四个角之间切分**的问题，属需求分配（p2-05 的领地），不属于制动器这个角色自身的参数；p2-05 改不了模板文件（写范围限制），所以切分必须能在调用点表达。`brake.brake_amplitude(..., share=)` / `brake.torque_parameters(..., share=)` / `brake.wheel_torque_element(..., share=)` 提供该参数。记录值可完全复现：`share=0.6` → 17400 N*mm，`share=0.4` → 11600 N*mm（`tests/subsystems/test_brake_subsystem.py:182-194`）。 |
| `rotor_inertia` | `kg*m^2` | 0.0568 | **新增** | 路线图 2.1 的第四个槽。当前落地的内核力律**不读惯量**（力偶直接加在模型本来就有惯量的车轮体上），所以它是**声明性**槽位：默认值由几何导出并写明推导——实心钢盘，外径 0.28 m，厚 12 mm，7850 kg/m³ → 5.80 kg，`0.5*m*r^2` = 0.0568 kg*m^2（`subsystems/brake.py:59-65`、`templates/builtin.py:606-613`）。 |

标准集结果（`ROLES["brake"].required_slots`，改后）：

```
('piston_area', 'effective_radius', 'friction_coeff', 'rotor_inertia')
```

### 1.2 DRIVE

落点：`templates/builtin.py` `DRIVE.property_slots`（改前 `:622-629`，改后 `:644-653`）；
`templates/roles.py` `ROLES["drive"].required_slots`（改前 `:145`，改后 `:144`）。

| 改前槽名 | 单位 | 默认值 | 改后 | 理由 |
|---|---|---|---|---|
| `maximum_drive_torque` | `N*mm` | 0.0 | **删除**，改为 `max_torque` | 改名 + 语义澄清：把传动显式写出来之后（`T = max_torque * gear_ratio * efficiency * share * drive_input`），`max_torque` 是**电机端**扭矩，`maximum_drive_torque` 读起来像**轮端**扭矩，两者在非 1:1 传动下不是一个数，旧名会误导。旧名在 `DrivelineSpec` 上仍在（schema 不在本行写范围），本次只改模板槽。 |
| `driven_wheels` | `-` | 0.0 | **删除** | 一个轮子一个力矩元之后，「驱动轮集合」就是「拥有力矩元的轮的集合」，模板里再放一份弱表示只会与元素本身打架。`DrivelineSpec.driven_wheels` 仍保留并继续校验（schema 不在写范围）。 |
| `drive_split` | `-` | 0.0 | **删除** | 同上：切分是元素自己的 `share`（`drive.wheel_shares(driveline)` 从 `DrivelineSpec.drive_split` 读出，按内核缓存用的轮序），模板不再重复声明。 |
| `gear_ratio` | `-` | 1.0 | **新增** | 路线图 2.1 的传动槽。默认 1.0 是**有意**的恒等传动，不是占位符：恒等传动下这一个元素逐位复现 live builder（`preparation/vehicle_dynamic.py::_build_wheel_torque_signals` 的驱动分支）算出的轮端扭矩，所以「元素 = 该公式的另一种写法」是可测的（`tests/subsystems/test_drive_subsystem.py:254-292` 用精确 `==`）。轮毂电机的减速比由调用方在槽里写自己的数，此时 `max_torque` 读作电机扭矩（`test_a_reduction_scales_the_wheel_torque`）。 |
| `efficiency` | `-` | 1.0 | **新增** | 同上，默认 1.0 是恒等传动的一半。 |
| `max_torque` | `N*mm` | 0.0 | **新增** | 见 `maximum_drive_torque` 一行。默认 0.0 保留旧默认：现有所有模型都处于「不驱动」状态，要驱动就由调用方给数。 |

标准集结果（`ROLES["drive"].required_slots`，改后）：

```
('gear_ratio', 'efficiency', 'max_torque')
```

### 1.3 `templates/roles.py` 条目内的其他字段

`required_mounts`、`outputs`、`has_torque_channel` 两个角色都**未改**：

- `required_mounts=("wheel_center", "spin_axis")` 原样保留（改前改后同）；
- `outputs=("brake_torque",)` / `("drive_torque",)` 原样保留；
- `has_torque_channel=True` 原样保留。

只改了 `note` 文本，因为原文与改后的机制不再一致（原 brake note 写 "emits a per-wheel torque"，原 drive note 写 "declares a distribution"）——理由与机制改动一致，不是顺手改格式。

角色名集合与 `roles.py:158-162`（现为 `:162-...` 的 `_check_roles`）**未触碰**：`git diff` 对 `roles.py` 只落在 `"brake"`/`"drive"` 两个条目内。

## 2. 改后的槽集合自证（已执行）

```
$ uv run --no-sync python -c "from suspension_multibody.templates import ROLES; print(tuple(ROLES['brake'].required_slots)); print(tuple(ROLES['drive'].required_slots))"
('piston_area', 'effective_radius', 'friction_coeff', 'rotor_inertia')
('gear_ratio', 'efficiency', 'max_torque')
```

模板声明与角色要求一致，由 `tests/subsystems/test_brake_subsystem.py:122-134`（`declared_slots == set(spec.required_slots)`）与 `tests/subsystems/test_drive_subsystem.py:218-230` 断言；四槽/三槽的**精确集合**断言分别在 `test_brake_subsystem.py:137-162`、`test_drive_subsystem.py:233-260`（后者同时断言旧名不再出现）。

## 3. 缺槽负例（异常类型 + 消息全文 + 发生层 file:line）

**已有机制，未另造**：`Template.check_role_contract()`，`templates/model.py:342-367`。
缺槽分支的 raise 点在 `templates/model.py:362-366`（消息在 `:363-366`）。
该函数被两处调用，即槽校验真正生效的两个层次：

- `templates/registry.py:39`（注册期）
- `templates/instantiate.py:221`（实例化期）
- 另有第三条相关但不同层次的门：`templates/instantiate.py:405-415`（properties 文件层，走 `check_filled`，`model.py:369-390`，消息是 `has unfilled required property slot(s)`）

负例构造：取**已注册的内建模板**，用 `dataclasses.replace` 只摘掉一个槽，再调 `check_role_contract()`（脚本：会话 scratch 的 `negative_slot.py`，输出如下原文）。

```
$ uv run --no-sync python "$PI_SCRATCH_DIR/negative_slot.py"
--- remove 'piston_area'; suspension_multibody.templates.model.TemplateError
    message: template 'brake_4wdisk_simplified' does not satisfy role 'brake': missing property slot(s) ['piston_area']
    layer:   model.py:363 (check_role_contract)
--- remove 'effective_radius'; suspension_multibody.templates.model.TemplateError
    message: template 'brake_4wdisk_simplified' does not satisfy role 'brake': missing property slot(s) ['effective_radius']
    layer:   model.py:363 (check_role_contract)
--- remove 'friction_coeff'; suspension_multibody.templates.model.TemplateError
    message: template 'brake_4wdisk_simplified' does not satisfy role 'brake': missing property slot(s) ['friction_coeff']
    layer:   model.py:363 (check_role_contract)
--- remove 'rotor_inertia'; suspension_multibody.templates.model.TemplateError
    message: template 'brake_4wdisk_simplified' does not satisfy role 'brake': missing property slot(s) ['rotor_inertia']
    layer:   model.py:363 (check_role_contract)
--- remove 'gear_ratio'; suspension_multibody.templates.model.TemplateError
    message: template 'powertrain_simplified' does not satisfy role 'drive': missing property slot(s) ['gear_ratio']
    layer:   model.py:363 (check_role_contract)
--- remove 'efficiency'; suspension_multibody.templates.model.TemplateError
    message: template 'powertrain_simplified' does not satisfy role 'drive': missing property slot(s) ['efficiency']
    layer:   model.py:363 (check_role_contract)
--- remove 'max_torque'; suspension_multibody.templates.model.TemplateError
    message: template 'powertrain_simplified' does not satisfy role 'drive': missing property slot(s) ['max_torque']
    layer:   model.py:363 (check_role_contract)
EXIT=0
```

七个槽逐个摘除都点名缺的那个槽（`assert missing in str(error) and "missing property slot" in str(error)` 全部通过）。异常类型是 `suspension_multibody.templates.model.TemplateError`（`ValueError` 子类，`templates/model.py:62`）。

消息的完整模板（`templates/model.py:364-365`）：

```
template {self.name!r} does not satisfy role {self.role!r}: missing property slot(s) {missing_slots}
```

同一机制对两侧角色都生效，验证命令：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py packages/suspension_multibody/tests/templates -q -p no:cacheprovider
82 passed in 1.63s
EXIT=0
```

**模板槽**里的旧名（`brake_mu` / `effective_piston_radius` / `max_brake_value` / `front_brake_bias` / `maximum_drive_torque`）现在都不存在了；但 `DrivelineSpec`（`schema/vehicle.py:203-232`，**不在本行写范围**）仍然带着这五个字段并继续校验——本次只改模板与角色槽，未动 schema。`templates/model.py` 也没有针对旧槽名的拒绝逻辑：某个外部 properties 文件若仍写 `brake_mu`，会被当作**多余**槽（`instantiate.py:405-415` 只查缺不查多，本行未改该策略）。

## 4. 未做到 / 不确定

- 旧槽名现在既不在模板也不在角色里，只是不再存在（见上一段的实测）。
- `rotor_inertia` 的默认值 0.0568 是**推导值**，不是从冻结文档读到的数；文档里没有这个量，已在两处代码注释与本节写明。
