# p3-01 / 子任务 2（Goal 2）：四种构型最小几何模型「今天能否构造」实测

本文件只存**实跑**结果。两套入口都跑了，因为「构造」在本仓库有两个含义，结论不同：

- **入口 A — 滚转中心链路**：`compute_vehicle_roll_centers(vehicle)`（`vehicle/roll_centers.py:35`）。这是 G3（`EPIC.md:87`）要求四种构型各给出滚转中心的**同一条**算法入口。
- **入口 B — 装配链路**：`compose_axle(model, "K")`（`subsystems/entry.py:28`）与 `compose_vehicle_runtime(vehicle, mode="K")`（`subsystems/vehicle_assembly.py:124`）。这是「能不能装出一台车」的入口，走 `subsystems/suspension.py` 的 `side_content`，读的仍是模板的硬点 role。

## 实跑命令

```bash
# 入口 A + B，四种构型的自定义硬点集（双叉臂沿用 tests/conftest.py 的 full_vehicle_model 几何）
uv run --no-sync python tasks/p3-01-freeze/probe_configs.py      # 退出码 0

# 显式拓扑（topology="explicit"）：5 连杆 / 麦弗逊 / 扭梁 用自定义关节自行声明
uv run --no-sync python tasks/p3-01-freeze/probe_three.py        # 退出码 0

# 显式拓扑装进整车运行时
uv run --no-sync python tasks/p3-01-freeze/probe_vehicle_route.py # 退出码 0

# 现成夹具（5 硬点、explicit 的扭梁式单轴）走两条入口
uv run --no-sync python tasks/p3-01-freeze/probe_axle_rollcenter.py
uv run --no-sync python tasks/p3-01-freeze/probe_trailingarm_vehicle.py
```

脚本落在本行目录（`probe_*.py`），入口 A/B 四构型完整输出落 `raw/out_configs.txt`。

---

## 结论表（逐种一行）

| 构型 | 入口 A：`compute_vehicle_roll_centers` | 入口 B：装配（`compose_axle` / `compose_vehicle_runtime`） |
|---|---|---|
| **双叉臂** | **可构造**（有限数值，见 §1） | **可构造**（13 body / 16 约束） |
| **5 连杆** | **不可构造** —— `ValueError: missing hardpoint for roll-center role upper_front`（`roll_centers.py:78`） | 显式拓扑**可装配**；模板路径（`topology=symmetric_proxy`）不可 —— `ValueError: missing required front-axle hardpoint for upper_front`（`geometry.py:146`） |
| **麦弗逊** | **不可构造** —— 同 5 连杆，同一条拒绝（`roll_centers.py:78`） | 显式拓扑**可装配**（6 约束）；模板路径同 5 连杆被拒（`geometry.py:146`） |
| **扭梁** | **不可构造** —— 同一条拒绝（`roll_centers.py:78`） | 显式拓扑**可装配**（3 约束 / 整车 8 约束）；模板路径被拒（`geometry.py:146`） |

**对 G3 的关键事实**：`compute_vehicle_roll_centers` 是四种构型唯一的滚转中心入口，它对**除双叉臂以外的三种一律在第一步就拒绝**，根因是 `roll_centers.py:82-89` 只认 `upper_front`/`upper_rear`/`upper_outer`/`lower_front`/`lower_rear`/`lower_outer` 六个 role，而三种构型一个都不具备。这是 F7（`EPIC.md:134`）预期的结论，**已实测确认**。

---

## §1 双叉臂：可构造（成功输出原文）

命令：`uv run --no-sync python tasks/p3-01-freeze/probe_configs.py`，退出码 **0**。

```
===== [double_wishbone] entry A: compute_vehicle_roll_centers =====
CONSTRUCTED: front center=array([ 1.13686838e-13, -1.80000000e+02]) left_ic=array([-1166.66666667,   100.        ]) right_ic=array([1166.66666667,  100.        ])
CONSTRUCTED: rear center=array([ 1.13686838e-13, -1.80000000e+02]) left_ic=array([-1166.66666667,   100.        ]) right_ic=array([1166.66666667,  100.        ])
----- [double_wishbone] entry B: compose_axle(mode='K') -----
ASSEMBLED: bodies=['ground', 'lower_arm_L', 'lower_arm_R', 'rack', 'rack_housing', 'tie_rod_L', 'tie_rod_R', 'upper_arm_L', 'upper_arm_R', 'upright_L', 'upright_R', 'wheel_hub_L', 'wheel_hub_R'] constraints=16
```

- 入口 A 产物形状：`dict[str, RollCenterResult]`，键 `{"front", "rear"}`；每条 `RollCenterResult(axle, center, left_instant_center, right_instant_center)`，都是 2 元 `[y, z]` 数组。此处 `center[0]` = `1.14e-13`（数值零），`center[1]` = `-180.0`（滚转中心高 = 路面下方 180 mm，纯几何构造的裸结果）。
- 入口 B 用 `tests/conftest.py:26 full_vehicle_model` 的硬点几何（`UPPER_INBOARD_FRONT` / `UPPER_INBOARD_REAR` / `UPPER_OUTBOARD` / `LOWER_INBOARD_FRONT` / `LOWER_INBOARD_REAR` / `LOWER_OUTBOARD` / `TIE_ROD_INBOARD` / `TIE_ROD_OUTBOARD` / `WHEEL_CENTER` / `RACK_CENTER`）。
- 同一条链路也是 `tests/physics/test_vehicle_physics.py:56`（唯一调用者）的路径，见 `tests_physics_assertions.md`。

## §2 5 连杆：不可构造（拒绝原文）

命令：`uv run --no-sync python tasks/p3-01-freeze/probe_configs.py`，退出码 **0**（脚本内部捕获并打印；异常由被测代码抛出）。

### 入口 A（滚转中心）拒绝原文

```
===== [five_link] entry A: compute_vehicle_roll_centers =====
CONSTRUCT REFUSED: ValueError: missing hardpoint for roll-center role upper_front
    at .../vehicle/roll_centers.py:43 in compute_vehicle_roll_centers: left_ic = _instant_center(axle, "L")
    at .../vehicle/roll_centers.py:83 in _instant_center: _hardpoint(axle, "upper_front", side) + _hardpoint(axle, "upper_rear", side)
    at .../vehicle/roll_centers.py:78 in _hardpoint: raise ValueError(f"missing hardpoint for roll-center role {role}")
```

- 异常类型：`ValueError`
- 消息全文：`missing hardpoint for roll-center role upper_front`
- 触发位置：`packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:78`（`raise`）；调用栈 `:43` → `:83` → `:78`。
- **关键**：这条拒绝与「5 连杆是否被支持」无关——模型声明（`FrontAxleModel` + `VehicleModel`）**通过了**，拒绝发生在滚转中心求值的第一步。即便把硬点命名为 `LINK1_INNER`/`LINK1_OUTER`…，`roll_centers.py` 也不会看它们。

### 入口 B（模板路径，`topology=symmetric_proxy`）拒绝原文

```
----- [five_link] entry B: compose_axle(mode='K') -----
ASSEMBLY REFUSED: ValueError: missing required front-axle hardpoint for upper_front
    at .../subsystems/types.py:407 in lookup: return lookup_hardpoint(self.side_schema[side], role)
    at .../subsystems/geometry.py:146 in lookup_hardpoint: raise ValueError(f"missing required front-axle hardpoint for {role}")
```

- 异常类型：`ValueError`；消息全文：`missing required front-axle hardpoint for upper_front`
- 触发位置：`packages/suspension_multibody/src/suspension_multibody/subsystems/geometry.py:146`。
- 完整调用栈（`probe_tb.py` 实跑原文）：

```
  File ".../subsystems/entry.py", line 52, in compose_axle
    return si_assembly_for_axle(model, request=request).assembly.physical
  File ".../subsystems/si_assembly.py", line 468, in si_assembly_for_axle
  File ".../subsystems/si_assembly.py", line 242, in axle_contributions_and_order
    suspension_outputs.append(suspension_subsystem.side_content(context, side))
  File ".../subsystems/suspension.py", line 397, in side_content
    points[(body, label)] = context.local(body, context.point(side, hardpoint_role))
  File ".../subsystems/types.py", line 411, in point
  File ".../subsystems/types.py", line 407, in lookup
  File ".../subsystems/geometry.py", line 146, in lookup_hardpoint
ValueError: missing required front-axle hardpoint for upper_front
```

- 即：**默认双叉臂子系统在装配时硬读 `upper_front` 等 role**，这正是 p3-01 要冻结的「零硬编码」缺口。

### 入口 B（显式拓扑）：可装配

命令：`uv run --no-sync python tasks/p3-01-freeze/probe_three.py`，退出码 **0**。

```
--- 5-link (topology=explicit, bodies=['upright_L', 'upright_R']) ---
  ASSEMBLED: bodies=['chassis', 'upright_L', 'upright_R'] constraints=10
```

装进整车运行时（`probe_vehicle_route.py`，退出码 **0**）：

```
--- 5-link: VehicleModel accepted ---
  compose_vehicle_runtime OK: bodies=9 constraints=24 total_mass=1380.0
```

**结论**：5 连杆**可以构造**，但只有走 `topology="explicit"` 自己声明关节这条路；**滚转中心链路不支持它**（每侧 5 根球面副，无臂线可求交）。对 p3-03 而言，5 连杆断言**需要新增最小几何模型声明**（显式拓扑已够），且 `compute_vehicle_roll_centers` 必须先换成新引擎才有值可断言。

## §3 麦弗逊：不可构造（拒绝原文）

命令：`uv run --no-sync python tasks/p3-01-freeze/probe_configs.py`，退出码 **0**。

### 入口 A 拒绝原文

```
===== [macpherson] entry A: compute_vehicle_roll_centers =====
CONSTRUCT REFUSED: ValueError: missing hardpoint for roll-center role upper_front
    at .../vehicle/roll_centers.py:43 in compute_vehicle_roll_centers: left_ic = _instant_center(axle, "L")
    at .../vehicle/roll_centers.py:83 in _instant_center: _hardpoint(axle, "upper_front", side) + _hardpoint(axle, "upper_rear", side)
    at .../vehicle/roll_centers.py:78 in _hardpoint: raise ValueError(f"missing hardpoint for roll-center role {role}")
```

- 异常类型 `ValueError`，消息全文 **`missing hardpoint for roll-center role upper_front`**，触发位置 `roll_centers.py:78`。
- 麦弗逊是**下臂 + 滑柱**，没有上臂；`upper_front`/`upper_rear`/`upper_outer` 三个 role 全缺。用 `STRUT_TOP`/`STRUT_LOWER` 代替上臂线也无效——`:88-89` 只查 `upper_outer`/`lower_outer`，别名表（`:59-67`）里没有 `STRUT_*`。

### 入口 B（模板路径）拒绝原文

```
----- [macpherson] entry B: compose_axle(mode='K') -----
ASSEMBLY REFUSED: ValueError: missing required front-axle hardpoint for upper_front
    at .../subsystems/types.py:407 in lookup: return lookup_hardpoint(self.side_schema[side], role)
    at .../subsystems/geometry.py:146 in lookup_hardpoint: raise ValueError(f"missing required front-axle hardpoint for {role}")
```

同上：`geometry.py:146`，消息 `missing required front-axle hardpoint for upper_front`。

### 入口 B（显式拓扑）：可装配

```
--- MacPherson (topology=explicit, bodies=['upright_L', 'upright_R']) ---
  ASSEMBLED: bodies=['chassis', 'upright_L', 'upright_R'] constraints=6
```

（每侧：下臂前端 revolute + 后端 spherical + 滑柱顶 spherical = 3 约束/侧。）

## §4 扭梁：不可构造（拒绝原文）

命令：`uv run --no-sync python tasks/p3-01-freeze/probe_configs.py`，退出码 **0**。

### 入口 A 拒绝原文

```
===== [twist_beam] entry A: compute_vehicle_roll_centers =====
CONSTRUCT REFUSED: ValueError: missing hardpoint for roll-center role upper_front
    at .../vehicle/roll_centers.py:43 in compute_vehicle_roll_centers: left_ic = _instant_center(axle, "L")
    at .../vehicle/roll_centers.py:83 in _instant_center: _hardpoint(axle, "upper_front", side) + _hardpoint(axle, "upper_rear", side)
    at .../vehicle/roll_centers.py:78 in _hardpoint: raise ValueError(f"missing hardpoint for roll-center role {role}")
```

- 异常类型 `ValueError`，消息全文 **`missing hardpoint for roll-center role upper_front`**，触发位置 `roll_centers.py:78`。
- 本行用的最小声明是 `TRAILING_ARM_PIVOT` / `WHEEL_CENTER` / `BEAM_PIVOT` / `SPRING_SEAT`。

**现成夹具复现了同一条拒绝。** `packages/suspension_multibody/tests/data/composable/synthetic_trailing_arm_axle.json`（5 个硬点 `ARM_PIVOT` / `WHEEL_CENTER` / `SPRING_CHASSIS` / `SPRING_ARM` / `WHEEL_CENTER__R`，`topology="explicit"`），命令 `uv run --no-sync python tasks/p3-01-freeze/probe_trailingarm_vehicle.py`：

```
explicit trailing-arm axle topology = explicit bodies = ['upright_L', 'upright_R']
[as-is (topology=explicit, per the fixture)] VEHICLE DECLARED
[as-is (topology=explicit, per the fixture)] REFUSED: ValueError: missing hardpoint for roll-center role upper_front
    at .../vehicle/roll_centers.py:83 in _instant_center: _hardpoint(axle, "upper_front", side) + _hardpoint(axle, "upper_rear", side)
    at .../vehicle/roll_centers.py:78 in _hardpoint: raise ValueError(f"missing hardpoint for roll-center role {role}")
```

这个夹具**在装配链路上是通的**（`probe_axle_rollcenter.py` 输出，退出码 0）：

```
trailing-arm explicit model declared OK; topology = explicit
trailing-arm assembled: bodies = ['chassis', 'upright_L', 'upright_R'] constraints = 2
```

→ **扭梁今天「能装配、不能出滚转中心」**；两条链路的差别就在这里，不是模型声明的问题。

- 另：**默认拓扑**（`symmetric_proxy`）下的扭梁声明被 `schema` 直接拒绝。实测原文（`probe_trailingarm_vehicle.py` 的第一版，退出码 1）：

```
pydantic_core._pydantic_core.ValidationError: 1 validation error for VehicleModel
  Value error, front axle requires positive mass specs for: lower_arm_L, lower_arm_R, rack, tie_rod_L, tie_rod_R, upper_arm_L, upper_arm_R, upright_L, upright_R [type=value_error, input_value={'chassis': RigidBodySpec..., x=0.0, y=0.0, z=0.0))}, input_type=dict]
```

来源：`schema/vehicle.py` 的 `_topology` 校验器，实测锚点 `:291-296`（条件 `:291` `axle.topology == "symmetric_proxy"`、`:292-293` 判缺名、`:294-296` 抛错）。

### 入口 B（显式拓扑）：可装配

```
--- twist-beam (topology=explicit, bodies=['upright_L', 'upright_R']) ---
  ASSEMBLED: bodies=['chassis', 'upright_L', 'upright_R'] constraints=3
```

整车运行时：

```
--- twist-beam: VehicleModel accepted ---
  compose_vehicle_runtime OK: bodies=9 constraints=8 total_mass=1380.0
```

---

## §5 四种构型走**模板**路径的附加拒绝（模板注册期）

除装配期拒绝外，模板注册期还有一道拒绝。实测脚本 `probe_axle_rollcenter.py`（`uv run --no-sync python`，退出码 0）：

```
five-link template REFUSED: TemplateError: template 'five_link_probe' does not satisfy role 'suspension': missing mount(s) ['lower_outer', 'tie_inner', 'tie_outer', 'upper_front', 'upper_outer', 'upper_rear', 'wheel_center']
```

- 异常类型 `TemplateError`（`templates/model.py:64`，继承 `ValueError`），触发位置 `templates/model.py:356-359`（`check_role_contract`）。
- 原因：`templates/roles.py:71-90` 的 suspension role 硬要求 9 个 mount。实测输出：

```
suspension.required_mounts = ('upper_front', 'upper_rear', 'upper_outer', 'lower_front', 'lower_rear', 'lower_outer', 'wheel_center', 'tie_inner', 'tie_outer')
suspension.required_slots  = ('spring', 'damper', 'bushing')
suspension.outputs         = ('wheel_travel', 'camber', 'toe', 'track_change')
```

- **后果（对 p3-03 的直接影响）**：任何非双叉臂构型，只要走**模板/子系统**这条路，都被要求声明 `upper_front`/`upper_rear`/`upper_outer` 三个名存实亡的上臂 mount。`tests/templates/test_template_drives_the_subsystem.py:58-100` 的 `_single_arm()` 就是被迫这么做的，注释原文在 `:84`：

```
        # The upper mounts are declared, and locate geometry, but are inert.
```

- 结论：**模板路径上「四种构型算法统一」今天做不到**，除非改 `templates/roles.py:73-83` 的 mount 契约或绕开模板走 explicit。这是 `EPIC.md:34` 的配套要求与现状之间的真实缺口，记为本行事实，留给 p3-02/p3-03。

## §6 已注册模板清单（实测）

```
registered templates: ('brake_4wdisk_simplified', 'double_wishbone', 'powertrain_simplified', 'steering', 'vehicle_body', 'wheel_on_hub')
double_wishbone.suspension_kind = double_wishbone
```

**内置模板里只有一个悬架模板**（`DOUBLE_WISHBONE`，`templates/builtin.py:397`，`:405` `suspension_kind="double_wishbone"`）。全仓 grep `five.?link|macpherson|mcpherson|twist.?beam|multilink` 在 `packages/**` 的源码与测试中**零命中**（唯一命中在两份 `.codex-tasks` 旧文档里，不是产品代码）——即**没有任何内置的三种构型模板**，p3-02/p3-03 若要断言必须**新增最小模型声明**。

## §7 未完成项（明确不伪造）

- 「四种构型各自跑通一次 study/solve」**未在本行验证**：本行判据只要求构造性盘点，且真实求解会触及 `cases/`，超出本行写范围。
- 麦弗逊走 explicit 时**滑柱用 spherical 近似**（不是 prismatic/cylindrical 的真实滑柱约束）；这是可跑的声明，**不是工程等价模型**，本行不据此断言任何物理。
- 5 连杆「显式每侧 5 个球面副」的过约束判定与数值可解性**未验证**——只验证了装配产物生成成功（`constraints=10`），没有跑求解。
- 「扭梁扭杆」的物理（左右臂通过扭杆耦合）在本行只用一个 revolute 代替，**未验证**它是否表达扭梁语义；仅是能通过装配入口的最小声明。
