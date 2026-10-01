# p3-04 证据 (b)(d)：3 轴（6 点）与单轮（N = 1）的断言与残差容差

判据对应 `EPIC.md` 行 255 的 **(b)** 与 **(d)**。

## 0. 跑的是什么

新增断言全部落在**新建**文件
`packages/suspension_multibody/tests/physics/test_static_loads.py`（9 个用例）。
**未改** `packages/suspension_multibody/tests/physics/test_vehicle_physics.py`（p3-03 归属）。

数值证据由 `raw/n_point_cases_probe.py` 输出（本文所有数字均由它实跑给出）：

```bash
uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/raw/n_point_cases_probe.py
```

退出码 `0`。

## 1. (b) 3 轴：6 个接触点，解存在、残差在容差内

接触点清单（6 个，实测自 `result.support_points`）：

```
front_left, front_right, middle_left, middle_right, rear_left, rear_right
```

静态求解结果：

```
N = 6
rank(A) = 3
unique  = False           # rank(3) < N(6)：最小范数解，解不唯一
residual = 1.955777406692505e-08
tolerance = 0.00400853318553198
total_mass = 408.617042358
weight     = 4008533.18553198
六个载荷都是 668088.8642553301
载荷之和   = 4008533.1855319804
```

`residual` 比容差小 **5 个数量级**（1.96e-8 vs 4.01e-3），即「载荷相容、解存在」。

纵向加速度 1000 mm/s² 时（载荷转移方向可由断言检查）：

```
front_left = 660770.0941290087   （减小）
middle_left = 668088.8642553302  （基本不变）
rear_left  = 675407.6343816515   （增大）
residual = 9.569339454174042e-08   tolerance = 0.00400853318553198
```

对应的断言原文（`tests/physics/test_static_loads.py`）：

```python
def test_three_axle_six_contact_points_balance() -> None:
    ...
    assert len(result.support_points) == 6
    assert set(result.support_points) == {
        "front_left", "front_right",
        "middle_left", "middle_right",
        "rear_left", "rear_right",
    }
    assert set(result.wheel_loads) == set(result.support_points)
    assert result.rank == 3
    # Three independent equations against six unknowns: a family, not a point.
    assert result.unique is False
    assert result.residual <= result.residual_tolerance
    # Force and moment balance, rebuilt from the published numbers.
    for residual in _equilibrium_rows(result):
        assert residual <= result.residual_tolerance
    assert np.isclose(
        sum(result.wheel_loads.values()),
        result.total_mass * 9810.0,
        rtol=0.0,
        atol=result.residual_tolerance,
    )
    # A minimum-norm member of a symmetric six-point layout splits the load evenly;
    # any other balanced solution would satisfy the balance checks and fail here.
    quarter = result.total_mass * 9810.0 / 6.0
    for name, value in result.wheel_loads.items():
        assert np.isclose(value, quarter, rtol=1e-9, atol=1e-6), name
```

以及纵向载荷转移断言：

```python
def test_three_axle_six_contact_points_transfer_load_longitudinally() -> None:
    ...
    assert accelerated.rank == 3
    assert accelerated.residual <= accelerated.residual_tolerance
    assert accelerated.wheel_loads["front_left"] < static.wheel_loads["front_left"]
    assert accelerated.wheel_loads["rear_left"] > static.wheel_loads["rear_left"]
```

## 2. (b) 单轮可解例：接触点在质心正下方、无侧向/纵向加速度

构造：一条 entry、`sides=("L",)`、轮端名 `single_left`，轴的全部硬点与部件都在车辆中心线上
（`side_y=0`），因此整车质心的 x、y 都落在接触点正上方。

```
N = 1
contact = [0.0, 0.0, 0.0]（路面上的接触点）
center_of_mass = [0.0, 0.0, 97.61085039578978]
|com.x - contact.x| = 0.0
|com.y - contact.y| = 0.0
rank(A) = 1
unique  = True            # rank(A) = 1 = N，解唯一 —— 不得标成「解不唯一」
residual = 0.0
tolerance = 0.00073675886425533
total_mass = 75.102840393
weight     = 736758.86425533
wheel_loads = {"single_left": 736758.86425533}
```

即载荷恰等于整车重量，`rank(A) = 1 = N`，**解唯一**。
（改造前这条路径被 `rank < 3` 无条件拒绝，抛
`ValueError: four wheel support points do not span force/moment balance`。）

断言原文：

```python
def test_single_contact_point_under_the_centre_of_mass_solves_and_is_unique() -> None:
    ...
    assert len(result.support_points) == 1
    contact = result.support_points["single_left"]
    assert abs(result.center_of_mass[0] - contact[0]) <= 1e-9
    assert abs(result.center_of_mass[1] - contact[1]) <= 1e-9

    assert result.rank == 1
    assert result.rank == len(result.support_points)
    assert result.unique is True
    assert result.residual <= result.residual_tolerance
    assert result.residual == 0.0
    assert np.isclose(
        result.wheel_loads["single_left"],
        result.total_mass * 9810.0,
        rtol=0.0,
        atol=result.residual_tolerance,
    )
    for residual in _equilibrium_rows(result):
        assert residual <= result.residual_tolerance
```

## 3. (b) 单轮不可解报错例：载荷与接触点几何不相容

两个例子，几何与矩阵秩都与可解例相同或同阶（**秩恒为 1**，不随载荷改变），
差别只在载荷相容性：

### 例 1：几何与可解例完全一致，只把载荷换成侧向加速度 1000 mm/s²

```
A = [[1.0], [0.0], [0.0]]    rank = 1
raised: IncompatibleStaticLoadsError
  residual       = 7330852.1179
  tolerance      = 0.00073675886425533
  contact_points = 1
消息全文：
the static loads are incompatible with the 1 contact point(s) of this vehicle: the residual of the three equilibrium equations is 7330852.1179 and the tolerance is 0.00073675886425533; no set of vertical reactions at those points balances the force and the moments
```

`residual / tolerance ≈ 1.0e10`。

### 例 2：接触点横向偏离质心 700 mm，纵向 -3000 mm/s²、侧向 2000 mm/s²

```
A = [[1.0], [0.0], [-503.3098588841543]]    rank = 1
raised: IncompatibleStaticLoadsError
  residual       = 21992556.3537
  tolerance      = 0.00073675886425533
  contact_points = 1
消息全文：
the static loads are incompatible with the 1 contact point(s) of this vehicle: the residual of the three equilibrium equations is 21992556.3537 and the tolerance is 0.00073675886425533; no set of vertical reactions at those points balances the force and the moments
```

**消息含残差实测值、容差与接触点数 N，不含秩。**

断言原文（参数化两例）：

```python
@pytest.mark.parametrize(
    ("label", "case"),
    _single_point_incompatible_cases(),
    ids=[label for label, _ in _single_point_incompatible_cases()],
)
def test_single_contact_point_reports_incompatible_loads_by_name(
    label: str, case: dict
) -> None:
    ...
    assembly = _corner_assembly(side_y=case["side_y"])
    with pytest.raises(IncompatibleStaticLoadsError) as raised:
        compute_static_wheel_loads_for_assembly(assembly, acceleration=case["acceleration"])

    error = raised.value
    text = str(error)
    assert error.contact_points == 1
    assert error.residual > error.tolerance
    assert repr(error.residual) in text
    assert repr(error.tolerance) in text
    assert "1 contact point(s)" in text
    assert "rank" not in text
    assert isinstance(error, ValueError)
```

以及「秩无法区分两例」的显式断言：

```python
def test_the_rank_of_a_single_contact_point_matrix_does_not_change_with_the_loads() -> None:
    ...
    assembly = _corner_assembly()
    solved = compute_static_wheel_loads_for_assembly(assembly)
    with pytest.raises(IncompatibleStaticLoadsError):
        compute_static_wheel_loads_for_assembly(
            assembly, acceleration=np.array([0.0, 1_000.0, 0.0])
        )

    contact = solved.support_points["single_left"]
    matrix = np.array(
        [
            [1.0],
            [contact[0] - solved.center_of_mass[0]],
            [contact[1] - solved.center_of_mass[1]],
        ],
        dtype=float,
    )
    assert int(np.linalg.matrix_rank(matrix)) == solved.rank == 1
```

## 4. (d) 力矩平衡残差容差：取值与理由

**容差**（`vehicle/static_loads.py:65`/`:69`/`:243`）：

```
_RESIDUAL_RELATIVE_TOLERANCE = 1e-9
_MINIMUM_LOAD_SCALE = 1.0            # N
tolerance = 1e-9 * max(|total_mass * (gravity + accel[2])|, 1.0)
```

判据：`residual <= tolerance` 即判「载荷相容、解存在」；`residual > tolerance` 才报错。

**理由（三项都独立于任何一次结果的具体字节）：**

1. **量纲对齐**。三条平衡方程都是力（第一条是力、后两条是力矩，而力矩由「力×力臂」给出，
   故整个 `A x − b` 的分量都是力）。容差也必须是力，不能是「数」——
   所以取 `total_mass * 竖直加速度` 这个**方程正在平衡的竖直载荷**为尺度，
   而不是取解向量范数或某个元素的最大值。同一辆车、同一加速度，
   无论 N 是 4 还是 6，容差不变（实测 4 轮 0.01444435545702132、3 轴 0.00400853318553198，
   差别只来自整车质量，不来自点数）。
2. **上界来自舍入、下界来自不可解，中间有八个数量级空档**。
   载荷量级 O(1e6~1e7) N，双精度相对误差 2.2e-16，矩阵向量乘累加几次 →
   残差舍入底噪约 1e-9~1e-8 N（实测四轮 9.54e-07、三轴 1.96e-08、单轮 0.0）。
   1e-9 的相对因子在 1e7 N 量级上给出 1e-2 N，比舍入底噪高约 6 个数量级；
   而不可解例的残差是载荷本身（实测 7.3e6、2.2e7 N），比容差高 8~10 个数量级。
   两个区间之间没有可争的边界，所以这个「1e-9」不需要精调，
   也不会因为某台机器的浮点差异而翻转判定。
3. **`_MINIMUM_LOAD_SCALE` 兜底**。`total_mass * 竖直加速度 = 0`（无重力的模型）时容差会退化为 0，
   那样自身舍入就会被判成不相容。取 1 N 下限，使空载/无重力情形仍以 1e-9 N 判定。
   （该下限只影响 `total_mass * (gravity + accel[2]) == 0` 的退化输入。）

**断言覆盖**（`EPIC.md` 行 255(d) 要求 3 轴与 4 轮两种情形都跑）：
`_equilibrium_rows(result)` 从结果自己公布的数字（`wheel_loads`、`support_points`、
`center_of_mass`、`total_mass`）**重建** 3×N 矩阵与右端项，再算
`A x − b` 的三行绝对值，逐行断言 `<= result.residual_tolerance`。
这不是读回结果里的 `residual`，而是独立重算一遍。
该断言用在：3 轴两例、单轮可解例、4 轮一例，共 4 处。

## 5. (b) pytest 输出与退出码

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics/test_static_loads.py -q -p no:cacheprovider
```

```
.........                                                                [100%]
9 passed in 1.52s
EXIT=0
```

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider
```

```
102 passed, 1 xfailed in 9.46s
EXIT=0
```

`1 xfailed` 是既有登记项（`tests/vehicle/test_pac2002_contact_mass.py` 一侧），
本行未新增任何 skip/xfail。

## 6. 9 个用例清单

| # | 用例 | 断言要点 |
|---|---|---|
| 1 | `test_three_axle_six_contact_points_balance` | 6 点、解存在、残差在容差内、力/力矩重建残差、`unique is False`、最小范数均分 |
| 2 | `test_three_axle_six_contact_points_transfer_load_longitudinally` | 6 点下纵向载荷转移方向 |
| 3 | `test_single_contact_point_under_the_centre_of_mass_solves_and_is_unique` | N=1 可解、`rank = 1 = N`、`unique is True`、残差 0、载荷 = 重量 |
| 4–5 | `test_single_contact_point_reports_incompatible_loads_by_name[...]`（2 例） | `pytest.raises` + 消息含残差与容差、含 N、不含秩 |
| 6 | `test_the_rank_of_a_single_contact_point_matrix_does_not_change_with_the_loads` | 两例矩阵同秩（恒为 1），秩无法作判据 |
| 7 | `test_four_wheel_vehicle_keeps_the_minimum_norm_split` | 4 轮最小范数解与 `unique is False` 不变（零回归门） |
| 8 | `test_the_contact_points_are_the_assembly_wheel_table` | 接触点集合 == `assembly.wheel_centers`，坐标逐位取自装配 |
| 9 | `test_a_two_axle_model_still_reports_through_the_model_door` | `VehicleModel` 入口仍给出 4 轮结果 |
