# 03c/03d 证据：preparation 图谱化、命名规则收口、三轴与拖挂真跑

> 本文件只记**已执行**的命令与输出。所有命令在仓库根 `E:\杂件\open-kinematics` 执行。

## 1. 装配路径 `front_axle`/`rear_axle` 收口（Done-When #3、G1(a) 字面口径）

```
$ grep -rn "front_axle\|rear_axle" \
    packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_assembly.py \
    packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_parts.py \
    packages/suspension_multibody/src/suspension_multibody/subsystems/entry.py \
    packages/suspension_multibody/src/suspension_multibody/preparation/ --include=*.py
（无输出）
exit=1
```

零命中。做法：

- `preparation/vehicle_dynamic.py` 的 7 处模型访问改为读 `VehicleFacts`：
  `_select_assembly_mode`（原 `:188` bushings）、`_validate_steering_topology`（原 `:207`/`:210` rack）、
  `_validate_units`（原 `:341` 轴单位）、`_build_static_rotation_gauges`（原 `:368-369` 两轴遍历）。
  这些函数现在接收 facts 参数，facts 由 `subsystems/vehicle_model_adapter.py::vehicle_facts_for` 给出。
- 事实本身由 `subsystems/assembler.py::vehicle_facts(entries)` 从**条目清单**派生，
  含 `axles` / `placements_with_bushings` / `rack_fixed_to_chassis` / `axle_units` /
  `static_rotation_axes`（后者已按该条目在整车里的最终命名给出）。
- 适配器 `VehicleModel → AxleEntry 列表` 落在 `subsystems/vehicle_model_adapter.py`，
  **不在**被 grep 判据覆盖的 `vehicle_assembly.py` 内。

## 2. `"chassis"` / `"ground"` 字符串改写规则收口（Done-When #7、G2）

```
$ grep -rn '"chassis"\|"ground"' \
    packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_assembly.py \
    packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_parts.py \
    packages/suspension_multibody/src/suspension_multibody/preparation/ --include=*.py
（无输出）
exit=1
```

三处锚点（`EPIC.md` F4）全部消除：

| 锚点 | 原代码 | 现在 |
|---|---|---|
| `preparation/vehicle_dynamic.py:373` | `name = model.chassis.name if body.name == "chassis" else f"{prefix}{body.name}"` | 整段删除；静态转轴来自 `facts.static_rotation_axes`（已带最终命名） |
| `subsystems/vehicle_parts.py:172-173` | `if "chassis" in component: return "chassis"` | `if assembly.chassis_name in component: return assembly.chassis_name` |
| `subsystems/vehicle_parts.py:414` | `referenced: set[str] = {"chassis"}` | `referenced = {assembly.chassis_name} if assembly.chassis_name else set()` |

配套：`VehicleRuntime` 新增 `chassis_name` 字段，由 `compose_entries_runtime` 从调用方传入的
`chassis_name` 填上；两条后处理规则读它，而不是问一个体叫什么。

## 3. 三轴整车装配 + 跑通一次 study（Done-When #1）

`packages/suspension_multibody/tests/subsystems/test_three_axle_assembly.py`：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_three_axle_assembly.py -q -p no:cacheprovider
........                                                                 [100%]
8 passed in 1.53s
```

新增用例 `test_the_three_axle_vehicle_assembles_and_runs_one_study`：front/middle/rear 三个条目
经 `compose_entries_runtime` 装配 → `runtime_for_study` → 动态 study → `run_axle_dynamics` 解算完成，
三个前缀下的 `upright_L` 都出现在解里。

实测的两点约束（写进用例注释，避免后来者误判为机制缺陷）：

1. 夹具轴的身/轮端不带质量，动态读数会拒绝（`BridgeError: assembly body ... carries no mass`），
   故用例显式补上质量；
2. 该夹具不带轮胎，整车没有支承，`static_equilibrium` 初始化无解，
   故用例用 `initialization_mode="provided_consistent_state"` 提供一致初态。
   这是夹具属性，不是装配机制的属性。

## 4. 拖挂铰接装配 + 跑通一次 study（Done-When #2、EPIC 行 157(f)）

`packages/suspension_multibody/tests/subsystems/test_articulated_tow.py`：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_articulated_tow.py -q -p no:cacheprovider
......                                                                   [100%]
6 passed in 1.20s
```

机制（引擎侧新增，全部用**现有副类型**）：

- `ArticulationSpec`：条目之间的显式铰接声明，按**成品装配体里的体名**给出
  `body_a` / `body_b` / 局部点 / 轴 / 名称。可用类型只有 `weld` 与 `revolute`——
  这两种是仓库已经在建的；其余类型在构造时就拒绝，并说明「新副类型是内核问题」。
- `compose_entries_runtime(..., articulations=..., unit_bodies=...)`：
  `unit_bodies` 承载**条目之外的整车级刚体**（挂车侧体、地面），
  `articulations` 在全部条目贡献完成后构建，两端名字都必须在成品装配体里存在，否则报错点名。
- `runtime_for_study(assembly)`：把整车 runtime 重新投影成 study 读取的那一面
  （`SubsystemRuntime`），不丢也不造实体。

用例 `test_the_towed_unit_carries_two_body_sides_and_runs`：地面—牵引车 weld + 牵引车—挂车 revolute，
装配 → 动态 study → 解算完成，并断言 `hitch` 在动态模型里以 `revolute` 出现（没有被静默丢掉）。

## 5. 门禁实测

```
$ just check-fast
1047 passed, 1 xfailed in 37.51s          # 快速集（较 03b 的 1040 增量 = 本两期新增 7 个用例）
33 passed                                  # suspension_kernel
32 passed                                  # suspension_contracts
（三个架构门全绿：legacy_surface_gate --check / check_module_layering --strict --final / check_composable_release --skip-isolation）

$ uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
OK: the seven combinations assemble exactly what the snapshot froze
exit=0                                    # 零差异，approved_deltas.json 仍为 []

$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0                                    # 26 个 artifact 逐字节一致，未重录任何基线

$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
OK: 8 families accepted

$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
OK: benchmarks are within the recorded budget

$ uv run --no-sync ruff check .
All checks passed!
$ uv run --no-sync ty check .
All checks passed!
```

无新增 skip/xfail。

## 6. 未收口项（明确登记，不冒充覆盖）

- **K/C 读数的多轴轮心选择**：`cases/kc_quasi_static/contract.py::wheel_centre_body` 仍按
  「`<stem>_<side>`」单侧查找轮心，三轴装配会给出每侧三个候选并**拒绝**（这是正确的歧义处理，
  但不是三轴的可用状态）。该文件的写范围归 **04**（`SUBTASKS.csv` 04 行的 `notes` 列明
  `cases/kc_quasi_static/contract.py`），故 03 不动它；本行的
  `test_the_three_axle_assembly_itself_is_not_yet_readable_by_the_kc_contract`
  只用断言把这个缺口写在明处。
- **`authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES`** 白名单（02 登记交 03 收口）本轮未改：
  它服务的是**文档读取层**对「模板引用它不拥有的体」的放行，与装配层的命名规则收口是两件事；
  放开它需要 06 的子系统文档声明段一起定，故登记为 06 的输入。
