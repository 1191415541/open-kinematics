# 单旋转副瞬轴 = 该副轴线的解析解断言

本文件回答 SPEC Done-When 第 2 条：**存在断言：单旋转副的瞬轴等于其轴线（方向与轴上一点均带容差判定），pytest 通过。**
（对应父 Epic 判据 Done-When (c) 的解析解部分。）全部内容为**实跑输出**，命令与退出码在 `raw/run_log.md` 登记。

---

## 1. 断言原文

断言文件：`packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py`。

- 参数化用例：`test_a_single_revolute_joint_has_the_joint_axis_as_its_instant_axis[axis0-point0-4]` /
  `[axis1-point1-3]` / `[axis2-point2-5]`（第 278 行起）；断言正文在 296–324 行。
- 三个用例的入参（第 279–283 行）：

  | 用例 | 轴线方向 | 轴线上一点 | 驱动分量 |
  |---|---|---|---|
  | `axis0-point0-4` | `(0, 1, 0)` | `(100, 20, 30)` | 4（绕局部 y） |
  | `axis1-point1-3` | `(1, 0, 0)` | `(10, -220, 45)` | 3（绕局部 x） |
  | `axis2-point2-5` | `(0.3, 0.4, 0.5)`（非单位，斜轴） | `(-5, 60, 5)` | 5（绕局部 z） |

- 判定两件事，各带容差（原文见第 300–306 行）：
  1. **方向**：报告方向相对单位化轴线的**垂直分量**
     `perpendicular = d - declared * (d @ declared)`，判 `|perpendicular| <= AXIS_DIRECTION_TOLERANCE`。
  2. **轴上一点**：`offset = screw.point - declared_point`，判
     `|offset - declared * (offset @ declared)| <= AXIS_POINT_TOLERANCE_MM`。

  轴上哪一点都算对，所以只能判垂直距离，不能逐分量比坐标。
- 同一用例还判（第 311–324 行）：`rank == 6`、`nullity == 0`、`constraint_residual < 1e-9`、`drive_residual < 1e-9`、
  旋转块本身落在轴线上（`np.allclose(rotation_block, declared * (rotation_block @ declared), atol=AXIS_DIRECTION_TOLERANCE)`）、
  `|pitch| <= AXIS_PITCH_TOLERANCE`。

## 2. 容差取值与理由

| 常量 | 取值 | 理由 |
|---|---|---|
| `AXIS_DIRECTION_TOLERANCE`（`test_screw_kinematics.py:58`） | `1e-9` | 判的是垂直分量（`sin θ`），不是 `arccos`。实测三种用例的垂直分量是 `2.292725e-13` / `4.472343e-14` / `3.336300e-11`。同一实测下 `arccos` 形式在斜轴用例上给出 `2.107342e-08` rad —— 比真实偏差大三个数量级，纯粹是 `arccos(1-δ)` 在 `δ→0` 时的 `sqrt` 塌缩。`1e-9` 比最差的垂直分量（`3.3e-11`）宽 30 倍，又能在 1° 的 `1e-6` 量级上抓住真实的轴对齐错误。 |
| `AXIS_POINT_TOLERANCE_MM`（`:66`） | `1e-4` | 实测轴上点垂直距离 `1.2e-7` / `5.7e-7` / `1.1e-8` mm，量级由数值微分的步长决定（`h=1e-6`，`eps·|f|/h ≈ 2.2e-7`），不是建模误差。`1e-4` mm（0.1 µm）比最差实测宽约 175 倍，仍比任何可分辨的机构长度小四个数量级。 |
| `AXIS_PITCH_TOLERANCE`（`:73`） | `1e-6` | 单旋转副两体共轴共点，相对运动是纯旋转，螺距真值为 0。实测 `-1.1e-13` / `4.9e-13` / `8.2e-9`；`1e-6` 比最差实测宽约 120 倍，而双叉臂整车上真实的轴螺距是 `37.8`（`raw/engine_contract.md`），高 7 个数量级，不会被这条容差漏过。 |

窄容差不是「为了好看」：每条都留了 ≥2 位余量，且落在「数值微分舍入下限」与「下一个物理量级」之间。

## 3. 实跑输出（原文）

命令：`uv run --no-sync pytest packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py -v -p no:cacheprovider`

```
packages\suspension_multibody\tests\vehicle\test_screw_kinematics.py::test_a_single_revolute_joint_has_the_joint_axis_as_its_instant_axis[axis0-point0-4] PASSED [ 41%]
packages\suspension_multibody\tests\vehicle\test_screw_kinematics.py::test_a_single_revolute_joint_has_the_joint_axis_as_its_instant_axis[axis1-point1-3] PASSED [ 50%]
packages\suspension_multibody\tests\vehicle\test_screw_kinematics.py::test_a_single_revolute_joint_has_the_joint_axis_as_its_instant_axis[axis2-point2-5] PASSED [ 58%]
============================= 12 passed in 1.96s ==============================
```

退出码 `0`。

## 4. 断言背后被判的实际数值

`raw/probe_engine.py` 的 `analytic case` 段落（该脚本退出码 `0`）原文：

```
===== analytic case: a single revolute joint's instant axis IS its axis =====
axis +y, spin about local y:
    declared axis  : (0.0, 1.0, 0.0) through (100.0, 20.0, 30.0)
    screw.direction: array([-7.49400542e-15,  1.00000000e+00, -2.29150032e-13])  angle_to_axis=np.float64(0.0) deg
    screw.point    : array([9.99999999e+01, 7.62390153e-12, 3.00000001e+01])
    distance_of_screw_point_to_axis=1.2369872106012186e-07 mm
    pitch=-1.1156004300558765e-13  |omega|=1.0000000000000728
    constraint_residual=2.291500322825942e-13 drive_residual=2.291500322825942e-13
    rank/nullity=6/0
axis +x, spin about local x:
    declared axis  : (1.0, 0.0, 0.0) through (10.0, -220.0, 45.0)
    screw.direction: array([1.00000000e+00, 2.74363865e-14, 3.53189700e-14])  angle_to_axis=np.float64(0.0) deg
    screw.point    : array([ 4.44665139e-12, -2.20000001e+02,  4.50000001e+01])
    distance_of_screw_point_to_axis=5.65763496691509e-07 mm
    pitch=4.916463513770655e-13  |omega|=0.9999999999999997
    constraint_residual=4.916463350761029e-13 drive_residual=4.916463350761029e-13
    rank/nullity=6/0
skew axis, spin about local z:
    declared axis  : (0.3, 0.4, 0.5) through (-5.0, 60.0, 5.0)
    screw.direction: array([0.42426407, 0.56568542, 0.70710678])  angle_to_axis=np.float64(1.2074182697257333e-06) deg
    screw.point    : array([-20.00000001,  40.        , -19.99999999])
    distance_of_screw_point_to_axis=1.1411696636928061e-08 mm
    pitch=8.242352578785938e-09  |omega|=1.4142135623259182
    constraint_residual=1.2079226507921703e-13 drive_residual=1.2079226507921703e-13
    rank/nullity=6/0
```

两点需要说明，避免误读：

- `screw.point` 落在轴线上**哪一点**都可以（`axis +x` 用例报的是 `(4.4e-12, -220, 45)`，轴线上的另一点）。判的是垂直距离 `5.7e-7`，不是坐标本身 —— 这正是断言只能取垂直距离的原因。
- `angle_to_axis` 那一列是 `arccos` 形式，斜轴用例显示 `1.2e-06` deg：那是舍入噪声（对应 `2.1e-08` rad），不是真实偏差。断言用的是垂直分量，实测 `3.3e-11`。

## 5. 未做到的

- 本行的解析解覆盖**单旋转副**（SPEC Done-When 第 2 条的字面要求）与**平面四连杆**（`raw/numerics_and_comparison.md` 第 2 节，那里是「今日构造 = 精确解」的等价检查）。
- SPEC「风险与回退」提到的「四种构型」中另两种的解析解验证**未在本行做**：按 p3-01 的 `raw/config_refusals.md`，本行的解析解先落在可构造的构型上。此处如实登记，不静默删除。
