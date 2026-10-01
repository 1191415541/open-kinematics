# p3-06 判据 1–2：G3 / G4 逐条实跑（本行自己执行，不采信子任务自报）

本行的每条结论都来自**本行自己跑的命令**，p3-03 / p3-05 的 `raw/` 只作对照线索。

## G3（`EPIC.md:83` 与 `:87`）：四种构型的滚转中心

### (i) 新引擎路径无硬点名称嗅探

```
$ bash -c '! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ -e _BODY_ALIASES packages/suspension_multibody/src/suspension_multibody/vehicle'
exit=0        # 0 = 零命中
```

**注意 `_BODY_ALIASES` 也在判据里**：p4-03 把 `geometry.py` 的别名表整表删除，
`grep` 在 `vehicle/` 下现在连别名表的名字也找不到。

### (ii) 四种构型各有断言

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider
102 passed, 1 xfailed in 9.78s
exit=0
```

其中 `tests/physics/test_roll_centres_by_topology.py` 对 **5 连杆 / 麦弗逊 / 扭梁**
三种构型各跑 3 条 parametrize 断言（有限值 + 中心线上 + 镜像 + 高度等于广义载荷比值），
双叉臂由 `test_vehicle_physics.py` 与同文件的两条专项断言承接。四种构型**都有断言**。

### (iii) 滚转中心高由侧倾反力虚功导数矩阵解算

复跑 p3-03 的对照探针（独立执行，不是读它的结论）：

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-03-roll-centers/raw/rc_evidence.py
exit=0
```

关键读数（本行实跑复现，与 p3-03 落盘值一致）：
`Q_uy = -2.0`、`Q_phi = -359.99999946237244`、`h = -179.99999973118622`，
独立有限位移判据与主路径差 `8.984e-05 mm`。
**不是**几何连线交点：`_line_intersection` / `_instant_center` 在源码中零命中。

## G4（`EPIC.md:85` 与 `:89`）：广义静平衡与动态通道

### (i) 三轴（6 点）与单轮两例

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/raw/n_point_cases_probe.py
exit=0
```

复跑要点（本行实跑）：

- **单轮可解例**：接触点在质心正下方 → `load single_left = 736758.86425533`，
  等于整车重量的全部；`rank = 1 = N`，`unique = True`（**没有被标成「解不唯一」**）。
- **单轮不可解例 1**：质心正下方 + 侧向 1000 m/s² → 抛 `IncompatibleStaticLoadsError`，
  消息含残差 `7330852.1179`、容差 `0.00073675886425533`、接触点数 `1`。
- **单轮不可解例 2**：接触点偏离质心 700 mm + 纵向 -3000 / 侧向 2000 → 残差 `21992556.3537`，
  同样含容差与点数。
- **两个不可解例的矩阵秩都是 1**（与可解例相同）→ 秩**无法**区分可解与不可解；
  区分它们的是**载荷与几何的相容性**（残差）。这正是 `EPIC.md:89` 的要求。
- 消息里**只有残差、容差、点数，没有秩**。

### (ii) `_WHEELS` 在 `vehicle/` 与 `report/` 无命中

```
$ bash -c '! grep -rn "_WHEELS = (\"front_left\"" packages/suspension_multibody/src/suspension_multibody/vehicle packages/suspension_multibody/src/suspension_multibody/report'
exit=0

$ grep -rn "_WHEELS" packages/suspension_multibody/src/suspension_multibody/vehicle packages/suspension_multibody/src/suspension_multibody/report
（零命中）
```

第二个命令是加强检查：连**任何** `_WHEELS` 出现都没有（不只是那个字面量赋值）。
p3-04 删了 `vehicle/static_loads.py` 的，p3-05 删了 `report/metrics/vehicle.py` 的。
