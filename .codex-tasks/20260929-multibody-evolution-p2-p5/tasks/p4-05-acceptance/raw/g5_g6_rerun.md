# p4-05 第 2–3 步：G5 / G6 独立实跑

> 独立口径：下面的结论来自**本行自己跑的命令**，不引用 p4-02 / p4-03 / p4-04 的 `raw/` 结论。

## G5（`EPIC.md:87`）：防倾杆独立化

### (i) 防倾杆构造路径无硬编码 `upright_L` / `upright_R`

```
$ bash -c '! grep -rn "\"upright_L\"\|\"upright_R\"" packages/suspension_multibody/src/suspension_multibody/subsystems'
exit=0        # 零命中
```

`subsystems/suspension.py` 里的两个体名字面量已删（改读模板声明）；
`subsystems/anti_roll_bar.py` 与 `templates/builtin.py` 的端口声明里也没有它们。

### (ii) 子系统与插接

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/vehicle_assembly -q -p no:cacheprovider
200 passed in 5.17s
exit=0
```

其中 `tests/subsystems/test_anti_roll_bar_subsystem.py`（6 用例）证明：
- `anti_roll_bar` 角色与模板 `anti_roll_bar_simplified` 已声明，模板含
  `torsion_bar_L/R` + `droplink_L/R` 四个部件；
- 四个端口 `chassis_mount_L/R`、`droplink_mount_L/R` 逐字声明、各带左右标签，
  去掉一个即无法注册（`TemplateError` 点名缺失的 mount）。

`tests/subsystems/test_arb_mount_ports.py`（6 用例）证明：
- 悬架声明 `arb_mount_L/R`，`owner` 为下臂，且**到达装配产物**；
- 双叉臂 → 落在 `lower_arm_L`；麦弗逊（测试内最小模板）→ 落在 `strut_L`
  （**同一 role、两个拓扑给出不同落点**）；
- 无候选时**点名拒绝**；owner 不在本届装配的声明**不报出去**。

**本行如实标注的边界**：防倾杆子系统尚未接入 `si_assembly.py` 的贡献派发链，
所以装配产物里还没有防倾杆自己的体；本行验的是**端口与配对**（G5 判据的文字范围）。

## G6（`EPIC.md:89`）：轮端统一

### (i) 父行的逐字命令

```
$ bash -c '! grep -rn VerticalTireElement packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py'
exit=0        # 零命中
```

### (ii) 过滤机制不存在

```
$ grep -c isinstance packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py
0
```

### (iii) 本行的加强检查：把范围放宽到 `subsystems/` + `preparation/`（如实记录）

```
$ bash -c '! grep -rn VerticalTireElement packages/suspension_multibody/src/suspension_multibody/subsystems packages/suspension_multibody/src/suspension_multibody/preparation'
exit=1        # 有命中
```

命中清单与**逐条判定**（用 `git show HEAD:` 比对，确认是不是本 Epic 引入的补丁）：

| 命中 | 性质 | 判定 |
|---|---|---|
| `subsystems/element_build.py:33/140/142` | 该元素类的 **import 与构造器**（`_tire` 每次轮胎行调用它） | **合法生产者**，不是补丁。删了就没有轮胎力元 |
| `subsystems/rig_link.py:297` | `_unloaded_radius` 读轮胎元素的 `unloaded_radius` | **读**，不是过滤。`HEAD` 处即存在 |
| `preparation/vehicle_dynamic.py:55/940` | 明确**拒绝**该元素：`"vertical tire element ... must be represented by the native tire ABI"` | **刻意的拒绝**，不是绕过。`HEAD` 处即存在 |

**三处都在 `HEAD`（阶段一提交 `8d8c5c0`）就已存在**（`git show HEAD:` 逐条比对，见下表），
即**本 Epic 没有引入任何新的类型过滤或补丁**。这正是阶段一 04 的判据要消除的那个东西
（「装配阶段按类型过滤掉轮胎」），而本行放宽范围后找到的三处都不是它。

```
$ git show HEAD:packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py | grep -n VerticalTireElement
55:    VerticalTireElement,
940:        elif isinstance(element, VerticalTireElement):

$ git show HEAD:packages/suspension_multibody/src/suspension_multibody/subsystems/rig_link.py | grep -n VerticalTireElement
297:        if type(element).__name__ != "VerticalTireElement":
```

### (iv) 单轴与整车跑通、且同一份 wheel 子系统

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/vehicle_assembly packages/suspension_multibody/tests/subsystems -q -p no:cacheprovider
200 passed in 5.17s
exit=0
```

单轴与整车的轮端唯一生产者是 `subsystems/wheel.py`（本行复跑 p4-04 的探针确认：
模块指纹 `6313b9c8…1729`，模块内仅 1 处 `instantiate(`）。
