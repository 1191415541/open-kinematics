# p3-03 实跑日志

> 本文件只记**已执行**的命令与其原文输出。所有数字都可由 `raw/` 下的脚本复现。

## 0. 本行交付的文件

| 文件 | 性质 |
|---|---|
| `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py` | 重写（267 → 273 行）；`_POINT_ALIASES` / `_hardpoint` / `_instant_center` / `_line_intersection` / `side_hardpoints` 全部删除，改走 p3-02 引擎 |
| `packages/suspension_multibody/tests/physics/test_roll_centres_by_topology.py` | 新增，11 用例（三构型 parametrize + 双叉臂专项） |
| `packages/suspension_multibody/tests/physics/test_vehicle_physics.py` | 滚转中心断言更新为 `center[1] ≈ -180.0` |
| `packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py` | 路由钩子用例改名并改用新字段名 |

原始材料（scratch，不入库）：`rc_evidence.py`（一次跑出全部证据）、`rc_cmp.py`（夹具）、
`roll_centers_before.py`（`git show HEAD:` 原文）、`rc_ic_check.py`、`resid_check.py`。

## 1. 起点：改造前实现的复现

```
$ git show HEAD:packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py \
    > $PI_SCRATCH_DIR/p303/roll_centers_before.py
$ wc -l $PI_SCRATCH_DIR/p303/roll_centers_before.py
132
```

两处相对导入（`:21` `from ..schema import ...`、`:22` `from ..subsystems.geometry import side_hardpoints`）
改为绝对导入后原样运行，**没有改逻辑**。

## 2. 主证据跑（`raw/rc_evidence.py`）

```
$ uv run --no-sync python $PI_SCRATCH_DIR/p303/rc_evidence.py
exit=0
```

八个 section 的原文见本目录其它五份证据文件（各自引用了对应 section）。
`rc_evidence.py` 的完整 stdout 已落 `raw/rc_evidence_output.txt`。

## 3. 主路径公式与实测数值

`roll_centers.py` 的核心三行：

```python
matrix = np.column_stack((-np.ones(2), patches[:, 1] * slopes))
generalized = matrix.T @ increments          # increments = np.ones(2)
lateral_force = float(generalized[0])        # Q_uy
roll_moment = float(generalized[1])          # Q_phi
height = -roll_moment / lateral_force        # h
```

夹具实测：

```
Q_uy  = -2.0
Q_phi = -359.99999946237244
h     = -179.99999973118622 mm
```

## 4. 本行自查发现并修正的两处缺陷

**这一节是诚实记账，两处都是本行实现过程中的真实缺陷，均由实跑证据暴露。**

### 4.1 字段名与实际内容不符

`RollCenterResult.force_line_slope` 存的是**接地点自身运动轨迹的 `d y / d z`**（即 `r = v_y / v_z`），
而不是**力线的斜率**。力线垂直于该轨迹，其 `d z / d y` 等于 `-r`。名字描述的与存的是两回事。

暴露方式：写判据 (a) 的并列对照表时，按 `dz/dy = 1/r` 换算出 `y=0` 交点高度 **3125.0 mm**
（与瞬心法的 −180.0 差 3305 mm），明显不通。回查 `raw/rc_ic_check.py` 打印的接地点速度与
零速度点，正面算出力线的 `dz/dy ≈ -0.24 = -r`，确认是名字与换算口径都错了。

修正：字段改名 `force_line_slope` → `contact_patch_slope`（左右各一），
模块 docstring 与两处局部 docstring 一并改正；三个测试文件同步改名。
**断言与容差一律未动**（`0.24` 这个期望值本来就是 `r`，改的只是名字）。

### 4.2 扭梁夹具的梁关节两点不重合

`test_roll_centres_by_topology.py::_twist_beam` 的梁关节写成
`upright_L` 上的 `y = -60` 对 `upright_R` 上的 `y = +60`。转动副的两点必须重合，
所以这个机构**根本没装上**：

```
$ uv run --no-sync python $PI_SCRATCH_DIR/p303/resid_check.py
twist-beam   front  max|C|=1.200e+02  rows=15  cols=12  bodies=2
twist-beam   rear   max|C|=1.200e+02  rows=15  cols=12  bodies=2
```

120 mm 的位置残差。此前那条 `h ≈ 2.4e-10` 的读数是在**未装配机构**上取的噪声——
它碰巧是有限值、也碰巧对称，所以三个 parametrize 用例全都过了，**没有一条断言能发现它**。

修正：梁关节两点同取车辆中心线上的一点（模块常量 `_BEAM = (1180, 0, 250)`，两侧共用）。

```
$ uv run --no-sync python $PI_SCRATCH_DIR/p303/resid_check.py
five-link    front  max|C|=0.000e+00  rows=60  cols=72  bodies=12
five-link    rear   max|C|=0.000e+00  rows=60  cols=72  bodies=12
macpherson   front  max|C|=0.000e+00  rows=32  cols=36  bodies=6
macpherson   rear   max|C|=0.000e+00  rows=32  cols=36  bodies=6
twist-beam   front  max|C|=0.000e+00  rows=15  cols=12  bodies=2
twist-beam   rear   max|C|=0.000e+00  rows=15  cols=12  bodies=2
```

修正后扭梁读数变为 `h = -5.248724579964e-09`（仍是数值零，结论不变），
但**这次是已装配机构上的读数**。断言与容差同样未动。

**这两处都说明「有限值 + 对称」不足以证明机构成立**；装配残差是必要的独立证据，
已作为 4b 节写进 `raw/three_topologies.md`。

## 5. 测试实跑

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider
.................................................x...................... [ 76%]
......................                                                   [100%]
93 passed, 1 xfailed in 9.43s
exit 0
```

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/physics/test_roll_centres_by_topology.py \
    -q -p no:cacheprovider --collect-only
11 tests collected in 0.65s
exit 0
```

`1 xfailed` 是既有项（`EPIC.md` 行 229 的基线），非本行新增。

## 6. 架构门实跑

```
$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
kernel root                : E:\杂件\open-kinematics\packages\suspension_kernel
mode                       : final
headers                    : 56
cpp translation units      : 75
modules present            : 23
header edges               : 118
source edges               : 107
source edge evidence       : 229 records (tu_include 213, symbol_reference 175, both 159; kind mentions 388)
target modules missing     : 0
legacy modules present     : 0
legacy modules unregistered: 0 []
mutual (reverse) edges     : 0
self-including headers     : 0
cross-aggregate includes   : 0
module cycles (SCC size>1) : 0

OK: layering matches the recorded baseline
exit 0
```

```
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
exit 0
```

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q -p no:cacheprovider
56 passed in 574.46s (0:09:34)
exit 0
```

## 7. SPEC 的 Final Validation Command

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q && \
  bash -c '! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ packages/suspension_multibody/src/suspension_multibody/vehicle'
```

两段均退出 0（前者的输出见第 5 节；后者 `grep-exit=0`，见 `raw/name_sniffing_removed.md`）。

## 8. 未做（明确记账）

- **未跑 `just gate-numeric` 三项**：本行尚未收敛到收尾点；`kc_baseline` 与
  `dynamic_hash_baseline` 的逐字节不变由 p3-06（阶段三收尾）与 Epic 收尾统一实跑并记录。
  本行的零回归证据止于上面的架构门与测试集，**不声称数值门已跑**。
- **未重录任何基线**：`git status --short -- packages/suspension_multibody/tests/data/` 的输出见
  阶段收尾记录。
