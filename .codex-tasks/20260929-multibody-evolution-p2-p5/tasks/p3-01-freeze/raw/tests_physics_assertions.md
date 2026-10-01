# p3-01 / 子任务 3（Goal 3）：`tests/physics/test_vehicle_physics.py` 断言原文全表

文件：`packages/suspension_multibody/tests/physics/test_vehicle_physics.py`，**88 行**，**5 个测试函数、共 17 条 `assert` 语句**。（SPEC 与 `EPIC.md:253(c)` 说的「5 条断言」实测为「5 个测试函数」；逐条 `assert` 是 17 条。以下逐条列出，兼做两种口径。）

`compute_vehicle_roll_centers` 的调用位置实测（F7 起点锚点 `:5/55`）：
- 导入：`:5` —— 精确命中。
- 调用：**`:56`** —— F7 写 `:55`，`:55` 是 `def` 行，**偏移 1 行，已记**。
- 该调用位于 `def test_front_and_rear_roll_centers_are_finite_and_symmetric`（`:55`）内，是整个仓库唯一一处调用。

`compute_static_wheel_loads` 的调用位置：`:10`、`:31`、`:32-35`（`acceleration=[1000,0,0]`）、`:44`、`:45-48`（`acceleration=[0,1000,0]`）、`:78`。

---

## 测试 1 `test_static_wheel_loads_balance_weight_and_moments`（`:9-27`）

```python
 9:def test_static_wheel_loads_balance_weight_and_moments(full_vehicle_model) -> None:
10:    result = compute_static_wheel_loads(full_vehicle_model)
11:
12:    assert result.rank == 3
13:    # A roundoff-scale residual rather than a physical one: the loads are ~1e4 N, so
14:    # this bound is 1e-10 of the quantity being balanced.  It is sensitive to the mass
15:    # distribution the composed runtime carries -- 方式 A's hub and the steering
16:    # housing are bodies of their own -- so the bound is stated at the scale of the
17:    # arithmetic rather than at the one a particular fixture happened to hit.
18:    assert result.residual < 1e-6
19:    assert all(value > 0.0 for value in result.wheel_loads.values())
20:    assert np.isclose(
21:        result.summary.total,
22:        result.total_mass * 9810.0,
23:        rtol=0.0,
24:        atol=1e-8,
25:    )
26:    assert np.isclose(result.summary.left_side, result.summary.right_side)
27:    assert np.isclose(result.summary.front_axle, result.summary.rear_axle)
```

逐条 `assert`（6 条）：

| # | 行 | 断言的量 | 判定方式 |
|---|---|---|---|
| 1 | `:12` | `result.rank` | **相等**（`== 3`）—— 这正是 p3-04 要改的秩口径 |
| 2 | `:18` | `result.residual` | **范围**（`< 1e-6`）—— `EPIC.md:249(a)` 点名「仅记录、不参与抛错」的那个残差，此处是**唯一**使用点 |
| 3 | `:19` | `result.wheel_loads.values()` 全为正 | **逐值范围**（`> 0.0`） |
| 4 | `:20-25` | `summary.total` vs `total_mass * 9810.0` | **相等**（`np.isclose`，`rtol=0.0`、`atol=1e-8`）—— 硬编码重力 9810.0 |
| 5 | `:26` | `summary.left_side` vs `summary.right_side` | **对称**（`np.isclose` 默认容差） |
| 6 | `:27` | `summary.front_axle` vs `summary.rear_axle` | **对称**（`np.isclose` 默认容差） |

**对 p3-04 的约束**：`:12` 与 `:18` 同时存在 —— 一条把 `rank == 3` 写成硬门，一条把 `residual < 1e-6` 写成硬门。p3-04 把判据从秩改成载荷相容性时，`:12` 是必须处理的那条。

## 测试 2 `test_longitudinal_acceleration_transfers_load_rearward`（`:30-38`）

```python
30:def test_longitudinal_acceleration_transfers_load_rearward(full_vehicle_model) -> None:
31:    static = compute_static_wheel_loads(full_vehicle_model)
32:    accelerated = compute_static_wheel_loads(
33:        full_vehicle_model,
34:        acceleration=np.array([1_000.0, 0.0, 0.0]),
35:    )
36:
37:    assert accelerated.summary.front_axle < static.summary.front_axle
38:    assert accelerated.summary.rear_axle > static.summary.rear_axle
```

逐条 `assert`（2 条）：

| # | 行 | 断言的量 | 判定方式 |
|---|---|---|---|
| 1 | `:37` | 前轴载荷随纵向加速度 `+1000` 减小 | **不等式（方向）** |
| 2 | `:38` | 后轴载荷随纵向加速度 `+1000` 增大 | **不等式（方向）** |

## 测试 3 `test_positive_lateral_acceleration_transfers_load_to_negative_y_side`（`:41-52`）

```python
41:def test_positive_lateral_acceleration_transfers_load_to_negative_y_side(
42:    full_vehicle_model,
43:) -> None:
44:    static = compute_static_wheel_loads(full_vehicle_model)
45:    accelerated = compute_static_wheel_loads(
46:        full_vehicle_model,
47:        acceleration=np.array([0.0, 1_000.0, 0.0]),
48:    )
49:
50:    assert accelerated.summary.left_side > static.summary.left_side
51:    assert accelerated.summary.right_side < static.summary.right_side
52:    assert accelerated.summary.right_left_delta < 0.0
```

逐条 `assert`（3 条）：

| # | 行 | 断言的量 | 判定方式 |
|---|---|---|---|
| 1 | `:50` | `left_side` 随横向加速度 `+1000` 增大 | **不等式（方向）** |
| 2 | `:51` | `right_side` 随之减小 | **不等式（方向）** |
| 3 | `:52` | `right_left_delta` 为负 | **不等式（符号）** |

## 测试 4 `test_front_and_rear_roll_centers_are_finite_and_symmetric`（`:55-64`）— 唯一调用滚转中心

```python
55:def test_front_and_rear_roll_centers_are_finite_and_symmetric(full_vehicle_model) -> None:
56:    centers = compute_vehicle_roll_centers(full_vehicle_model)
57:
58:    assert set(centers) == {"front", "rear"}
59:    for result in centers.values():
60:        assert np.all(np.isfinite(result.center))
61:        assert np.isclose(result.center[0], 0.0, atol=1e-8)
62:        assert np.isclose(
63:            result.left_instant_center[0], -result.right_instant_center[0]
64:        )
```

逐条 `assert`（4 条；`:60-63` 在 `for` 循环内，对每个轴各判一次）：

| # | 行 | 断言的量 | 判定方式 |
|---|---|---|---|
| 1 | `:58` | 返回字典的键集合 | **相等**（`== {"front", "rear"}`）—— 定死两个轴名 |
| 2 | `:60` | `result.center` 有限 | **有限性**（`np.all(np.isfinite(...))`） |
| 3 | `:61` | `result.center[0]`（前视 y 坐标） | **对称 / 相等**（`np.isclose(..., 0.0, atol=1e-8)`）—— 断言车辆中心线 |
| 4 | `:62-64` | `left_instant_center[0]` vs `-right_instant_center[0]` | **对称**（`np.isclose`，默认容差） |

**对 p3-03 的约束（重要）**：这 4 条是**阶段三改造前唯一的回归网**，且**只测有限性与左右对称**——
- **没有任何一条断言滚转中心的数值**（`:61` 只断言 y 分量为 0，`center[1]` 即滚转中心高**未被断言、未被检查范围、未被对过任何参考值**）。
- 也没有断言 `center[1]` 与 `road_z` 的关系、没有上下界。
- 因此「双叉臂结果与改造前可对照」（`EPIC.md:261(a)` 的 p3-03 判据）**没有现成的数值基准**；p3-03 必须先录下改造前的 `center[1]` 值才有对照。本行实测该值为 **`-180.0`**（用 `tests/conftest.py:26` 的 `full_vehicle_model` 几何，见 `config_refusals.md` §1 的实跑输出），可作为起点参考，但它**不是既有断言的一部分**。

## 测试 5 `test_static_wheel_loads_are_the_minimum_norm_solution`（`:67-88`）

```python
67:def test_static_wheel_loads_are_the_minimum_norm_solution(full_vehicle_model) -> None:
68:    """
69:    The retained Python solver must still return the *minimum norm* load split.
70:
71:    The four vertical reactions are underdetermined (three balance equations),
72:    so the algorithm -- not the physics alone -- picks one of a family of valid
73:    answers.  An equal-front-rear, equal-left-right layout has a symmetric
74:    minimum norm split; a solver that returned any other balanced solution (or
75:    an unconstrained least-squares fit) would fail this.  This pins the
76:    algorithmic choice that 2026-09-22 decision A2 keeps in Python.
77:    """
78:    result = compute_static_wheel_loads(full_vehicle_model)
79:
80:    assert result.rank == 3
81:    loads = result.wheel_loads
82:    quarter = result.summary.total / 4.0
83:    for name, value in loads.items():
84:        assert np.isclose(value, quarter, rtol=1e-9, atol=1e-8), name
85:    # The minimum-norm member of the balanced family is the uniform split, so
86:    # equality with the quarter load is the algorithm's fingerprint: a solver
87:    # returning any other balanced solution would satisfy the balance checks
88:    # above but fail here.
```

逐条 `assert`（2 条；`:84` 在循环内对四轮各判一次）：

| # | 行 | 断言的量 | 判定方式 |
|---|---|---|---|
| 1 | `:80` | `result.rank` | **相等**（`== 3`）—— 与 `:12` 重复 |
| 2 | `:84` | 每个轮的载荷 `value` vs `quarter = total / 4.0` | **相等**（`np.isclose`，`rtol=1e-9`、`atol=1e-8`）；`msg=name` 点名失败轮 |

**对 p3-04 的约束（硬门）**：`EPIC.md:263(d)` 要求「4 轮情形的结果与改造前逐位一致」。这 2 条断言正是「四轮均分」的指纹；其中 `:80` 的 `rank == 3` 是**四轮特有的结论**（3×4 满行秩），p3-04 改成 `rank(A) == N` 后 `3 != 4`，该断言**必须改写**，且改写前后**载荷值须逐位不变**。

---

## 实跑记录

命令：

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics -q -p no:cacheprovider
```

真实输出（末尾统计行）：

```
.....                                                                    [100%]
5 passed in 2.02s
```

退出码：**0**（`PYTEST_EXIT=0`；第二次复跑同命令为 `5 passed in 1.56s`，退出码 0）。

`git status` 确认 `packages/**` 无改动（见 `run_log.md` 与 `baseline_impact.md` 的自证小节）。
