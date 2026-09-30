# 03a 证据：引擎 + 适配器 + 适配器路径产物零差异

> 口径：只记录**已执行**的命令与原文输出。所有命令在仓库根 `c:/杂件/open-kinematics`、`git rev-parse --short HEAD` = `db22f9e`（工作区含 02 的未提交改动）。
> 差异登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`：**本行未新增任何登记，`--check` 差异为空**。

## 1. 适配器路径产物 vs 01 快照（硬门）

```
$ uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
OK: the seven combinations assemble exactly what the snapshot froze
EXIT=0
```

原文一行，无 warning、无差异明细。这是 D1 与 `EPIC.md` 行 196 的零回归硬门：换成条目清单引擎后，
`VehicleModel` 适配器路径的七种组合仍逐项等同于 01 冻结快照。

## 2. 改动目录 pytest

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/vehicle \
    packages/suspension_multibody/tests/vehicle_assembly \
    packages/suspension_multibody/tests/subsystems \
    packages/suspension_multibody/tests/model \
    packages/suspension_multibody/tests/authoring -q
.....................................x.................................. [ 25%]
........................................................................ [ 50%]
........................................................................ [ 76%]
...................................................................      [100%]
282 passed, 1 xfailed in 18.91s
EXIT=0
```

其中新增 `tests/subsystems/test_vehicle_assembler.py`（3 个用例，含 2 个参数化 = 5 项）：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_vehicle_assembler.py -q
...                                                                      [100%]
3 passed in 1.47s
EXIT=0
```

`1 xfailed` 是既有的（未新增 skip/xfail）。

## 3. 静态门与 `just check-fast`

```
$ uv run --no-sync ruff check .
All checks passed!
EXIT=0

$ uv run --no-sync ty check .
All checks passed!
EXIT=0

$ just check-fast
...
1033 passed, 1 xfailed in 40.26s     # 快速集
33 passed in 10.98s                  # suspension_kernel/tests
32 passed in 0.11s                   # suspension_contracts/tests
EXIT=0
```

快速集由 02 落地后的 1030 passed 增至 1033 passed，增量即本行新增的 3 个测试；无 skip/xfail 增长。

## 4. `git diff --stat -- packages/`

```
$ git diff --stat -- packages/
 .../contracts/assembly.schema.json                 |  15 ++
 .../suspension_multibody/authoring/documents.py    |  29 +++
 .../suspension_multibody/subsystems/composition.py |  49 +++-
 .../src/suspension_multibody/subsystems/types.py   |  10 +
 .../subsystems/vehicle_assembly.py                 | 247 ++++++---------------
 5 files changed, 166 insertions(+), 184 deletions(-)
```

前四项是 02 落地后的未提交改动（本行未碰）。本行对 `packages/` 的改动只有两项：

- `subsystems/vehicle_assembly.py`（`68 insertions(+)`, `179 deletions(-)`）：入口段由装配实现改为适配器。
- `subsystems/assembler.py`（未跟踪新文件，`git status` 显示 `??`）、`tests/subsystems/test_vehicle_assembler.py`（未跟踪新文件）。

```
$ git status --short -- packages/
 M packages/suspension_contracts/src/.../assembly.schema.json      （02，本行未碰）
 M packages/suspension_multibody/src/.../authoring/documents.py    （02，本行未碰）
 M packages/suspension_multibody/src/.../subsystems/composition.py （02，本行未碰）
 M packages/suspension_multibody/src/.../subsystems/types.py       （02，本行未碰）
 M packages/suspension_multibody/src/.../subsystems/vehicle_assembly.py   ← 本行
?? packages/suspension_multibody/src/.../subsystems/assembler.py          ← 本行新增
?? packages/suspension_multibody/tests/subsystems/test_vehicle_assembler.py ← 本行新增
```

## 5. 引擎不含 front/rear 轴字面量（本行自检，非终局 G1 判据）

```
$ grep -n "front_\|rear_\|front_axle\|rear_axle\|\"chassis\"\|\"ground\"" \
    packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py
29:it by comparing strings against ``"chassis"``/``"ground"``, which meant a document
```

唯一命中在模块 docstring 里，是**说明被删掉的那条旧规则**的文字，不是代码。
`assembler.py` 中没有任何 `front`/`rear` 前缀、轴字段或 `"chassis"`/`"ground"` 字符串规则。

`vehicle_assembly.py`（适配器）仍含 `model.front_axle` / `model.rear_axle` 与 `"front_"`/`"rear_"` 字面量——
按本轮口径第 1/3 条，这两处**必须**留在适配器里由本条显式写出；G1(a) 的终局 grep 无命中判据在 03c（`preparation/` 图谱化）之后复验。
