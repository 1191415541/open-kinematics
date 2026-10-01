# 新引擎模块路径上的硬点名称嗅探检查

本文件回答 SPEC Done-When 第 4 条（对应父 Epic 事实 F16 的修正项与 G3 的「不得按名字猜身份」）：
**引擎模块路径上 `grep UPPER_|LOWER_|UCA_|_BODY_ALIASES` 退出码 1（零命中）。**

---

## 1. 引擎模块的实际落点

```
packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py
```

（SPEC 允许「模块名由你定并登记」，此处登记为 `screw_kinematics`。）

## 2. 实测命令与输出

命令（SPEC 给的形状，路径按本行实际落点写死）：

```bash
bash -c '! grep -rn -e UPPER_ -e LOWER_ -e UCA_ -e _BODY_ALIASES packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py'
```

实测：

```
$ bash -c '! grep -rn -e UPPER_ -e LOWER_ -e UCA_ -e _BODY_ALIASES packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py'
$ echo $?
0
```

取反前先看原始 grep 的退出码，避免「取反后为 0」被误读成「有命中」：

```
$ grep -rn -e UPPER_ -e LOWER_ -e UCA_ -e _BODY_ALIASES packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py
$ echo $?
1
```

**原始 grep 退出码 `1`（零命中），取反后 `0`** —— 零命中成立。

## 3. 同一检查在测试侧的口径

新增测试文件里也有一条独立的文本检查（`test_no_engine_source_names_a_hardpoint_role`，
`tests/vehicle/test_screw_kinematics.py:467`），它读的是**模块源码文本**，在 pytest 内断言：

```
packages\suspension_multibody\tests\vehicle\test_screw_kinematics.py::test_no_engine_source_names_a_hardpoint_role PASSED [100%]
```

## 4. `_BODY_ALIASES` 的取用与否

- `_BODY_ALIASES` 定义在 `packages/suspension_multibody/src/suspension_multibody/subsystems/geometry.py:257`
  （SPEC 记的 `:226` 与当前行号不一致，实测在 257；本文件以实测为准），
  消费点在 `:289`（`_BODY_ALIASES.get(normalized, normalized)`）与 `:296`（按 stem + 侧别拼名）。
- **引擎模块一个字都没用**：`screw_kinematics.py` 不 import `subsystems`，所以谈不上取用。

## 5. 依赖方向：引擎没有 import `subsystems`

`grep -c subsystems` 在引擎模块上零命中；引擎的全部 import 只有：

```python
import numpy as np
from ..modeling.primitives import (...)
```

即 `vehicle → modeling`，**没有新增 `vehicle → subsystems` 边**。
SPEC 第 92 行的回退条件（「不通过则把引擎的输入改为调用方注入」）因此是按**主动选择**落实的，
不是被架构门逼出来的：引擎从设计上就只吃「调用方注入的约束集合 + 点表 + 装配状态」。
`roll_centers.py` 的调用段通过 **keyword-only 的 `instant_center_engine` 回调** 路由，
默认 `None` 时仍是原逻辑，因此 `roll_centers` 自己也不 import 引擎。

## 6. 仍会在 `vehicle/` 目录里命中的东西（如实登记）

SPEC 的 Done-When 只要求**引擎模块路径**零命中；整个 `vehicle/` 目录的 grep **当前不是零命中**：

```
$ grep -rn -e UPPER_ -e LOWER_ -e UCA_ -e _BODY_ALIASES packages/suspension_multibody/src/suspension_multibody/vehicle/
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:72:    "upper_front": ("UPPER_INBOARD_FRONT", "UPPER_INNER_FRONT", "UCA_FRONT"),
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:73:    "upper_rear": ("UPPER_INBOARD_REAR", "UPPER_INNER_REAR", "UCA_REAR"),
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:74:    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:75:    "lower_front": ("LOWER_INBOARD_FRONT", "LOWER_INNER_FRONT", "LCA_FRONT"),
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:76:    "lower_rear": ("LOWER_INBOARD_REAR", "LOWER_INNER_REAR", "LCA_REAR"),
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:77:    "lower_outer": ("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER"),
（另有一条 __pycache__/roll_centers.cpython-312.pyc 的二进制命中）
```

**全部命中都在 `roll_centers.py:72-77` 的 `_POINT_ALIASES` 表**，也就是 SPEC 明确归 p3-03 删除的那张表。
本行**不得**删它（删了 p3-03 就失去「改造前可对照」基准），所以这个目录级 grep 在 p3-02 结束时**必然非零**，
且**不是本行的缺陷**。本行只改 `roll_centers.py` 的调用段（第 36–52 行加 keyword-only 回调），别名表一个字未动。

## 7. 未做到

- `vehicle/` **目录级**的零命中本行做不到，原因见第 6 节（归属 p3-03）。
- 引擎模块路径上另有 `numpy`、`modeling.primitives` 两类来源的名称（如 `RevoluteJoint`、`BallJoint`），
  这些是**约束类型名**，不是硬点名，不在 SPEC 的四条关键词里。它们被引擎用作「残余行登记表」的键，
  是 SPEC 认可的做法的直接后果（把内核 joint registry 的残余行转写过来）。
