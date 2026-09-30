# 05 证据：试验台非侵入（被测总成不被改写）

本文件记录**已执行**的命令与退出码。工程目录：`E:\杂件\open-kinematics`。

## 1. 删掉了什么

| 位置 | 变化 |
|---|---|
| `subsystems/rig_link.py::_reown_tires`（原 `:315-348`） | **整函数删除**。它把轮胎 `replace(tire, wheel_body=carrier, wheel_center_local=zeros(3))`，即改写被测总成里的元素归属 |
| `rig_link.py::link_wheel_supplying_rig` 的调用点（原 `:229-232`） | **删除**。link 的 `elements` 现在恒为空 |
| `rig_link.py::_is_replaced_tire`（原 `:271-281`） | **删除**。它按名字把被测方的轮胎判为“被取代”，是改写所需的配套规则 |
| `rig_link.py::merge_rig_link` | 改为**纯追加**：`bodies/points/constraints/ideal_constraints/connections/elements` 一律 `runtime 的 + link 的`，不再过滤被测方任何实体 |
| `rig_link.py` 模块 docstring | 第二条由「tire moves to the carrier」改写为「the assembly is not rewritten」，并写明 D3 就是这条不可变性 |

`RigSpec` 与既有 rig 声明**未改动**：`supplies_wheels` 的含义仍是「这台试验台自带车轮」，
改的是它**不再改写被测物**。因此验收里「`RigSpec.supplies_wheels` 改写后既有 rig 声明的解释规则已登记」
一条**不适用**：没有改写，就没有需要重新登记的既有解释。

## 2. 逐项对照（本行自证，不采信子任务自报）

新增 `packages/suspension_multibody/tests/subsystems/test_the_rig_is_not_invasive.py`（10 条）：

* `test_a_wheel_bench_adds_only_its_own_entities[axle_dynamic|kc_quasi_static]`：
  对**两个供轮台**逐项比较被测 runtime 的 `bodies/points/constraints/ideal_constraints/elements`：
  每一条既有实体的指纹（`repr`，含质量/惯量/质心/位姿/固定性、点几何、约束与力元全部字段）
  与接入前**逐字相等**；新增集合**恰好**是 `wheel_carrier_L/R` 两个体、四个载点、两条焊缝；
* `test_the_bench_attaches_through_the_wheel_centre_fixture`：新增的两条约束都是 `WeldJoint`，
  两条 `point_a == point_b`（同一物理位置，世界坐标约定），且被测方自带的轮胎归属一字未变；
* `test_binding_any_bench_leaves_the_assembly_it_loads_unchanged[7 个 rig]`：**七个 rig 全覆盖**
  （含 01 快照未覆盖的 5 个整车台）。供轮台在装配层新增 `wheel_carrier_L/R`；整车台在装配层
  **不新增任何实体**（它们的执行器在 study 层绑定），两种情况都被测物逐项未变。
  这正是 `snapshot.py` 的 `_meta.coverage_boundary` 要求 05 自己提供的那一份对照。

```
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_the_rig_is_not_invasive.py -q
  -> 10 passed
```

反转的既有契约（D3）：

```
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_rig_link.py -q
  -> 通过
```

`test_the_tire_moves_to_the_bench_wheel` 已反转为 `test_the_tire_stays_on_the_assembly_it_belongs_to`：
断言改成「tire 的 `(name, wheel_body, wheel_center_local, stiffness, unloaded_radius)` 接入前后完全相等、
没有任何元素属于 `wheel_carrier_`、文档里的轮胎体就是承载轮心的那个体」。

## 3. 产物差异：为空，因此 `approved_deltas.json` 没有新增登记

```
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
  -> OK: the seven combinations assemble exactly what the snapshot froze      (exit 0)

cat .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/raw/approved_deltas.json
  -> []
```

原因（已核实，不是巧合）：冻结夹具 `tests/data/benchmark_axle.json` 与 C 夹具都不声明 `tires`
（`model.tires == ()`），所以 5 个产物里**没有任何 `VerticalTireElement`**，
`_reown_tires` 本来就遍历不到东西；删掉它，产物一位不动。也就是说本行的产物差异**恰好为空**，
按登记口径「未命中登记的差异即失败」，空差异无需登记（登记表保持 `[]`）。

轴侧动态硬门同样未动：

```
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
  -> artifacts hashed : 26
  -> combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
  -> OK: dynamic output matches the frozen baseline byte-for-byte              (exit 0)
```

`fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` 与 03/04 收尾时相同，未重录。

## 4. 快速集

```
uv run --no-sync pytest packages/suspension_multibody/tests \
  --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q -p no:cacheprovider
  -> 1069 passed, 1 xfailed
```

无新增 skip/xfail。
