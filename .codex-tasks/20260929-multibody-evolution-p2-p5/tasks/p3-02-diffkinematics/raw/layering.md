# 分层与导入边界

本文件回答 SPEC Done-When 第 5 条：
**`check_module_layering.py --strict --final` 退出码 0（0 环）且 `tests/architecture/test_import_boundaries.py` 通过。**

---

## 1. 关键事实先说

新引擎模块 `vehicle/screw_kinematics.py` **只 import 两样东西**（实测 `grep`）：

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from ..modeling.primitives import (...)
```

即依赖方向只有 `vehicle → modeling`（加标准库与 numpy）。
`grep -c subsystems` 在引擎模块上返回 `0`（退出码 1），
因此**本行没有新增 `vehicle → subsystems` 边**，SPEC 第 92 行的回退条件不触发，
也不需要靠「先跑 `test_import_boundaries` 再落地」来兜。
`roll_centers.py` 的调用段用的是 keyword-only 回调 `instant_center_engine`，自己也不 import 引擎 —— 路由依赖是零。

## 2. `check_module_layering.py --strict --final`

命令与实测输出（退出码 `0`）：

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
$ echo $?
0
```

关键行：`module cycles (SCC size>1) : 0`、`target modules missing : 0`、`legacy modules unregistered: 0 []`。

## 3. `tests/architecture/test_import_boundaries.py`

命令与实测输出（退出码 `0`）：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q -p no:cacheprovider
........................................................                 [100%]
56 passed in 594.31s (0:09:54)
$ echo $?
0
```

这条门按 SPEC 的要求跑了**完整**的一遍（含那个为 23×22 个顺序组合各开子解释器的用例，耗时约 10 分钟）。
它验的是「从任一入口进入、按任一顺序 import 产品模块，都不会出现越层或环」。

## 4. `legacy_surface_gate.py --check`（SPEC Final Validation Command 的第二段）

命令与实测输出（退出码 `0`）：

```
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
$ echo $?
0
```

`findings: 0` 表示 `report_native_import` / `report_constitutive_call` 等退役面规则均未命中：
本行没有把引擎接到 `report/` 侧，也没有引入力律计算。

## 5. 与 `roll_centers.py` 改动的关系

改了调用段（第 17 行加 `from collections.abc import Callable`；第 36–52 行加 keyword-only 参数与一行
`engine = instant_center_engine or _instant_center`；第 55–56 行把两处直接调用换成 `engine(...)`）。
**没有动** `_POINT_ALIASES`（71–79 行）、`_hardpoint`（82–90）、`_instant_center`（93–111）、
`_contact_front_view`（114–126）、`_line_intersection`（129–141）。默认路径逐位不变（见 `raw/run_log.md`
里的 `-1166.6666666666665` 对照）。

## 6. 未做到

- 无。三条分层/边界门全部实测通过，且引擎本身没有新增跨层边。
