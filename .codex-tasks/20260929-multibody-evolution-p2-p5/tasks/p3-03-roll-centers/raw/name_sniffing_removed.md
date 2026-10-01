# p3-03 判据 (c)：硬点别名表与二维交点路径删除

> 本行 `validation_command` 的第二段。命令与退出码由 `raw/rc_evidence.py` 第 8 节实跑记录。

## 1. 命令与输出原文

```
$ grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ packages/suspension_multibody/src/suspension_multibody/vehicle
  exit code = 1   (1 == no match)
  stdout    = ''
  stderr    = ''
```

**退出码 1 = 零命中**，即 `bash -c '! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ …'`
退出 0。

复核（手工）：

```
$ bash -c '! grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ packages/suspension_multibody/src/suspension_multibody/vehicle'
grep-exit=0
```

## 2. 被删路径逐条核对

```
$ grep -n -e _line_intersection -e _instant_center -e _POINT_ALIASES -e _hardpoint -e side_hardpoints \
    packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py
  exit code = 1   (1 == no match)
  stdout    = ''
```

源码逐 token 复核：

```
roll_centers.py: 273 lines
  'side_hardpoints' in source: False
  'UPPER_' in source: False
  'LOWER_' in source: False
  'UCA_' in source: False
  '_line_intersection' in source: False
```

| 改造前的名字 | 改造前位置（`git show HEAD`） | 现状 |
|---|---|---|
| `_POINT_ALIASES` | `:59-67`（六条角色 → 别名元组） | **已删**，文件内零命中 |
| `_hardpoint(axle, role, side)` | `:70-79` | **已删** |
| `_instant_center(axle, side)` | `:81-99` | **已删** |
| `_line_intersection(a, b, c, d)` | `:96`（二维直线求交） | **已删** |
| `from ..subsystems.geometry import side_hardpoints` | `:27` | **已删**，该模块不再 import |

**替代关系**：`_instant_center` / `_line_intersection` **没有同名概念留在 `vehicle/`**。
接地点轨迹斜率由 p3-02 的引擎（`vehicle/screw_kinematics.py`）解出；力线的表达改成了
「轨迹斜率 `r`」而不是「连线端点」（`raw/roll_center_virtual_work.md` §1）。
`force_line_slope` 这个旧字段名一并改为 `contact_patch_slope`，因为原名字描述的
是力线、存的却是轨迹斜率（`raw/run_log.md` §4 记录了这次改名）。

## 3. 依赖方向的连带效果

`side_hardpoints` 的唯一使用点被删除，`vehicle/` 对 `subsystems.geometry` 的依赖**减轻**
（`EPIC.md` 行 315 的预期方向）。但 p3-02 的引擎输入面重新引入了 `vehicle → subsystems`
边（`_assembly_for` → `subsystems.entry.compose_vehicle`），该边是既有的、非本行新增。
实测：

```
$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
module cycles (SCC size>1) : 0
OK: layering matches the recorded baseline
exit 0
```

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q
56 passed in 574.46s (0:09:34)
exit 0
```

## 4. 仍然存在的硬点名（不在本行写范围内）

`side_hardpoints` 本身还在 `subsystems/geometry.py:102`，仍被三处生产代码使用：
`adams/strict_c.py:37/946`、`authoring/solver.py:362`（注释引用）、`subsystems/si_assembly.py:195`。
那是它们各自的链路，**不属 p3-03 的写范围**，本行只确保 `vehicle/` 的滚转中心链路不再用它。
