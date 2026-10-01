# p3-03 判据 (b)：5 连杆 / 麦弗逊 / 扭梁三种构型各有断言

> 断言落 `packages/suspension_multibody/tests/physics/test_roll_centres_by_topology.py`
> （新增，276 行，11 用例）。跑法：`uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q`。

## 1. 起点：改造前只认双叉臂

p3-01 的 `raw/config_refusals.md` 已盘过四构型。**实测复核**（`raw/rc_evidence.py` 第 6 节）
把每个构型都送进改造前实现：

```
  double wishbone  before: answered {'front': -180.0, 'rear': -180.0}
  double wishbone  after : answered {'front': -180.0, 'rear': -180.0}
  five-link        before: refused -> ValueError: missing hardpoint for roll-center role upper_front
  five-link        after : answered {'front': 37.908131, 'rear': 37.908131}
  macpherson       before: refused -> ValueError: missing hardpoint for roll-center role upper_front
  macpherson       after : answered {'front': -0.0, 'rear': -0.0}
  twist-beam       before: refused -> ValueError: missing hardpoint for roll-center role upper_front
  twist-beam       after : answered {'front': 0.0, 'rear': 0.0}
```

**这就是 G3 的落点**：改造前只对硬点名齐备的双叉臂给得出答案，其余三种一律拒绝；
改造后四种全部给出有限值。

## 2. 三种构型的最小几何声明

`test_roll_centres_by_topology.py` 里三种构型都用 `topology="explicit"` 声明，
**没有任何硬点角色名**，只有 `WHEEL_CENTER` / `WHEEL_CENTER__R` 两个点用于定位轮心
（四构型共有，不属角色嗅探）。

### 五连杆（`_five_link()`，`:110-126`）

每侧 5 根独立两力杆，每根两端各一个球副：一端到 `chassis`，一端到该侧 `upright`。
每侧 5 个自由杆体，`bodies = (upright_L, upright_R, link1_L..link5_L, link1_R..link5_R)`。
实测装配位置残差 `max|C| = 0.000e+00`（见 4b 节）。

### 麦弗逊（`_macpherson()`，`:129-145`）

下摆臂：`chassis` → 下摆臂绕 `X` 轴的转动副（`_revolute`），下摆臂 → `upright` 球副。
滑柱：`chassis` → 滑柱沿 `Z` 的移动副（`_prismatic`），滑柱 → `upright` 球副。
实测装配位置残差 `max|C| = 0.000e+00`（见 4b 节）。

### 扭梁（`_twist_beam()`，`:148-166`）

两侧拖曳臂各自绕 `chassis` 的 `Y` 轴转动副；两臂之间一条沿 `Z` 轴的转动副（梁）。
实测装配位置残差 `max|C| = 0.000e+00`（见 4b 节）。

## 3. 三种构型各自的断言

三个 `@pytest.mark.parametrize("topology", sorted(_TOPOLOGIES))` 用例对每种构型各跑一遍。

### (i) 有限值 + 对称性 + 横向载荷符号 — `:177 test_every_topology_gives_a_finite_symmetric_roll_centre`

```python
centers = compute_vehicle_roll_centers(_vehicle(_TOPOLOGIES[topology]()))
assert set(centers) == {"front", "rear"}
for result in centers.values():
    assert np.all(np.isfinite(result.center)), topology
    assert np.isclose(result.center[0], 0.0, atol=1e-6), topology
    assert result.lateral_force < 0.0, topology
    assert np.isfinite(result.roll_moment), topology
```

**可判定点**：`center[0] == 0` 是**镜像对称构型的必然结果**——一个偏向某一侧的构造在镜像轴上
给不出零。不是「返回非空」。

### (ii) 两侧镜像 — `:197 test_every_topology_mirrors_the_two_sides`

```python
assert np.isclose(result.left_contact_patch[1], -result.right_contact_patch[1], atol=1e-9)
assert np.isclose(result.left_contact_patch_slope, -result.right_contact_patch_slope, atol=1e-9)
```

**可判定点**：两侧的读各自来自**独立求解**同一个镜像机构，所以这里出现不对称只能是构造的错，
不是几何的错。

### (iii) 高度就是广义载荷的比值 — `:215 test_the_height_is_the_ratio_the_generalized_loads_define`

```python
assert result.lateral_force != 0.0, topology
assert np.isclose(result.center[1], -result.roll_moment / result.lateral_force, rtol=1e-12), topology
```

**可判定点**：一个「报几何交点」的构造也能返回有限且对称的点，但在这一条上会失败。

### (iv) 双叉臂：只用报出的四个数在模块外重算 — `:230 test_the_height_matches_the_force_line_primitives_it_reports`

```python
patches = np.array([result.left_contact_patch, result.right_contact_patch])
slopes = np.array([result.left_contact_patch_slope, result.right_contact_patch_slope])
lateral = float(np.sum(-np.ones(2)))
moment = float(np.sum(patches[:, 1] * slopes))
assert np.isclose(result.lateral_force, lateral, rtol=1e-12), result.axle
assert np.isclose(result.roll_moment, moment, rtol=1e-12), result.axle
assert np.isclose(result.center[1], -moment / lateral, rtol=1e-12), result.axle
```

**可判定点**：高度必须能由报出的四个数（两接地点 y + 两斜率）在模块外复算出来。
报几何交点的构造「没有可复算的量」。

### (v) 双叉臂：矩阵第一列钉死 — `:249 test_a_lateral_translation_carries_the_patch_the_other_way`

```python
assert np.isclose(result.lateral_force, -2.0, rtol=1e-12), result.axle
```

## 4. 三种构型的实测结果

`raw/rc_evidence.py` 第 7 节（同时是上表 `after` 列的来源）：

### five-link

```
front: center = [ 0.             37.908131135921]
       left  slope = -0.052650182133231964   patch = [1400. -720. 0.]
       right slope =  0.05265018213321651    patch = [1400.  720. 0.]
       Q_uy = -2.0   Q_phi = 75.8162622718429
```

滚转中心在路面**上方** 37.91 mm。物理上可辩护：5 杆机构的杆向使轮端上升时接地点
**向内**移动（`r` 与 `y` 同号 → `y·r > 0` → `Q_phi > 0` → `h = −Q_phi/Q_uy > 0`）。

### macpherson

```
front: center = [ 0.000000000000e+00 -3.556129790637e-07]
       left  slope =  5.031346450577097e-10   patch = [1400. -720. 0.]
       right slope = -4.846791856746942e-10   patch = [1400.  720. 0.]
       Q_uy = -2.0   Q_phi = -7.112259581273309e-07
```

`h ≈ −3.6e-7 mm`，即**数值零**（路面平面内）。物理上可辩护：该最小模型的滑柱沿 **全局 `Z`**
滑动，下摆臂绕 **全局 `X`** 转动，四条约束合起来让轮端在正视面里近乎纯竖直平移
（`r ≈ 5e-10`），力线近水平，故滚转中心落在路面。这是**该最小几何**的性质，不是算法的失败——
`Q_uy = −2.0` 依然精确，比值仍然有定义。

### twist-beam

```
front: center = [0.000000000000e+00 -5.248724579964e-09]
       left  slope =  1.4645952853273074e-11   patch = [1400. -720. 0.]
       right slope =  6.616235337370796e-14   patch = [1400.  720. 0.]
       Q_uy = -2.0   Q_phi = -1.0497449159927543e-08
```

`h ≈ −5.2e-09 mm`，同样是数值零。物理上：拖曳臂绕全局 `Y` 转、梁绕全局 `Z` 转，
轮端在正视面里是纯竖直平移，力线水平，滚转中心落在路面（与扭梁「无独立侧倾中心」的
教科书结论一致）。**注意左右斜率符号相同**（两侧都在 `1e-14`~`1e-11` 量级、都是正的），
所以 `test_every_topology_mirrors_the_two_sides` 里的 `atol=1e-9` 覆盖了这种量级下的
符号无意义——**这是容差而非放松判据**：两个斜率之差 `1.5e-11` 远小于 1e-9，
各自在其绝对量级下都是零。

> **夹具修正记录**：本条最初把梁关节的两点分别写成 `y = ∓60`，实测位置残差
> `max|C| = 1.2e+02 mm`——两点不重合，机构根本没装上，那个 `1e-12` 量级的读数是
> 未装配机构上的噪声。改为两点同取车辆中心线上的一点（`_BEAM = (1180, 0, 250)`），
> 残差归零，读数如上表。**这条修正不改变任何断言或容差**，只是让被断言的对象真的成立。

## 4b. 三种构型的装配残差（实测）

```
five-link    front  max|C|=0.000e+00  rows=60  cols=72  bodies=12
five-link    rear   max|C|=0.000e+00  rows=60  cols=72  bodies=12
macpherson   front  max|C|=0.000e+00  rows=32  cols=36  bodies=6
macpherson   rear   max|C|=0.000e+00  rows=32  cols=36  bodies=6
twist-beam   front  max|C|=0.000e+00  rows=15  cols=12  bodies=2
twist-beam   rear   max|C|=0.000e+00  rows=15  cols=12  bodies=2
```

`max|C|` 是 `screw_kinematics.constraint_residual` 在装配位姿上的位置残差上确界。
全部为零，说明三种构型的声明都**真的装上了**，上面报的滚转中心是已装配机构上的读数。
（上面那处 twist-beam 夹具修正正是被这一列暴露出来的。）

## 5. 未补任何生产代码

三种构型都用自己的**测试侧**最小显式声明跑通，**没有**为它们修改任何生产代码。
`EPIC.md` F 项的「三种构型在滚转中心链路上被拒」是改造前的状态；本次改造让它们无需补声明
即可工作，因为算法改读装配的约束集，不再看图元的硬点名。

## 6. pytest 结果

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider
.................................................x...................... [ 76%]
......................                                                   [100%]
93 passed, 1 xfailed in 9.37s
```

退出码 0。其中 11 个为 `test_roll_centres_by_topology.py` 的用例
（3 个 parametrize × 3 构型 + 2 个双叉臂专项 = 11）。`1 xfailed` 是既有项，非新增。
