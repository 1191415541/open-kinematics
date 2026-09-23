# 8 种副的现状落点与截断点

任务：20260922-01-baseline
实测时间：本轮（2026-09-22 会话）

## 三处落点

### 1. 内核注册表（行数表）

`packages/suspension_kernel/cpp/src/contract/contract_registry.cpp:23-30`：

```cpp
const JointEntry kJoints[] = {
    {"spherical", 3},      {"revolute", 5},          {"fixed", 6},
    {"prismatic", 5},      {"universal", 4},         {"cylindrical", 4},
    {"inplane", 1},        {"convel", 4},            {"driven_translation", 1},
    {"driven_rotation", 1},
};
```

**共 10 项** = 8 种真实副 + 2 种 driven 坐标。注释声明「Row counts mirror `mb_joint/types.hpp`; the self-test asserts they agree with the constraint registry.」

同文件另有：
- `kElements`（:33-36）：`spring_damper`/`bushing`/`anti_roll_bar`/`bump_stop`/`aerodynamic_drag`/`steering_actuator`/`wheel_torque`/`point_wrench`/`gravity`
- `kTires`（:38）：`vertical_linear`/`fiala`/`pac2002`/`native_brush`
- `kCaseFamilies`（:40+）：7 个 family + `comparison`

**结论：内核侧无需改动即可支持任意总成使用任意副。**

### 2. 装配层（副类）

`packages/suspension_multibody/src/suspension_multibody/preparation/assembly/types.py`：

| 行号 | 类名 |
|---|---|
| :51 | `BallJoint`（继承 `PointCoincidence`） |
| :58 | `WeldJoint` |
| :81 | `RevoluteJoint` |
| :94 | `UniversalJoint` |
| :107 | `ConstantVelocityJoint` |
| :123 | `CylindricalJoint` |
| :136 | `InPlaneJoint` |
| :148 | `PrismaticJoint` |

**8 种副类齐备。**

### 3. schema（作者声明）

`packages/suspension_multibody/src/suspension_multibody/schema/model.py:100-109` 的 `IdealJointSpec.kind`：

```python
kind: Literal[
    "spherical", "revolute", "prismatic", "fixed",
    "universal", "constant_velocity", "cylindrical", "inplane",
]
```

**注意命名不一致**：schema 用 `constant_velocity`，内核注册表用 `convel`。

## 截断点（唯一的截断处）

`packages/suspension_multibody/src/suspension_multibody/cases/kc_quasi_static/contract.py:44-48`：

```python
_JOINT_KINDS = {
    "BallJoint": "spherical",
    "RevoluteJoint": "revolute",
    "PrismaticJoint": "prismatic",
}
```

**只映射 3 种副**，其余抛 `NativeKcError`（同文件 :101-102：`raise NativeKcError(f"unsupported joint {type(constraint).__name__}")`）。

**这是 kc 作者层的唯一截断点。** 装配层已有 8 种副类、schema 已允许 8 种，故 02 的实质是**拆掉这处截断**，不是新建底座。

## `constant_velocity → convel` 改名在两处各写一遍

| 文件 | 行号 | 代码 |
|---|---|---|
| `cases/axle_dynamic.py` | :292 | `"type": "convel" if joint.kind == "constant_velocity" else joint.kind,` |
| `cases/vehicle_dynamic.py` | :79 | `"type": "convel" if joint.kind == "constant_velocity" else joint.kind,` |

另有 `cases/vehicle_dynamic.py:87-92` 处理 `constant_velocity` 的附加字段 `convel_angle_target`。

**应收口到一处**（02 的目标之一）。

## `explicit` 拓扑已支持全部 8 种副

`preparation/assembly/front_axle.py:271-315` 的 `_explicit_constraint` 已支持全部 8 种副——即 `explicit` 路径无截断，只有 `symmetric_proxy` 经 kc 作者层时被截断。

## 实测确认

- 命令 2（`check_module_layering --strict --final`）退出码 0，未触及注册表
- 命令 3（kernel 15 passed）退出码 0，注册表自检通过
