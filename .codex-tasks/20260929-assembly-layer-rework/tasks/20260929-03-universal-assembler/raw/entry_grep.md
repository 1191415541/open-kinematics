# 03 证据：装配路径名字规则收口（TODO 第 5 行的 `raw/entry_grep.md`）

> 只记**已执行**的命令与输出。仓库根：`E:\杂件\open-kinematics`。

## 1. `front_axle` / `rear_axle`（`EPIC.md` G1(a) 字面口径：装配层三文件 + 整个 `preparation/`）

```
$ grep -rn "front_axle\|rear_axle" <装配层三文件> <preparation/ 整目录> --include=*.py
（无输出）
exit=1
```

零命中。落地方式：

- `preparation/vehicle_dynamic.py` 的 7 处模型访问改为读 `VehicleFacts`
  （`_select_assembly_mode` 原 `:188`、`_validate_steering_topology` 原 `:207`/`:210`、
  `_validate_units` 原 `:341`、`_build_static_rotation_gauges` 原 `:368-369`）；
- facts 由 `subsystems/assembler.py::vehicle_facts(entries)` 从**条目清单**派生
  （`axles` / `placements_with_bushings` / `rack_fixed_to_chassis` / `axle_units` /
  `static_rotation_axes`，最后一项已按该条目在整车里的最终命名给出）；
- `VehicleModel → AxleEntry 列表` 的适配器落在
  `subsystems/vehicle_model_adapter.py`，**不在**被本判据覆盖的 `vehicle_assembly.py` 里；
- `vehicle_assembly.py::compose_vehicle_runtime` 名字与签名不变（`tests/architecture/legacy_surface_gate.py`
  登记在册的表面），只改为消费适配器给出的条目清单。

## 2. `"chassis"` / `"ground"` 字符串改写规则（`EPIC.md` F4 三处锚点，G2）

```
$ grep -rn '"chassis"\|"ground"' <同上三文件> <preparation/ 整目录> --include=*.py
（无输出）
exit=1
```

三处锚点全部消除：

| 锚点 | 原代码 | 现在 |
|---|---|---|
| `preparation/vehicle_dynamic.py:373` | `name = model.chassis.name if body.name == "chassis" else f"{prefix}{body.name}"` | 整段删除；静态转轴改读 `facts.static_rotation_axes` |
| `subsystems/vehicle_parts.py:172-173` | `if "chassis" in component: return "chassis"` | `if assembly.chassis_name in component: return assembly.chassis_name` |
| `subsystems/vehicle_parts.py:414` | `referenced: set[str] = {"chassis"}` | `referenced = {assembly.chassis_name} if assembly.chassis_name else set()` |

配套：`VehicleRuntime` 新增 `chassis_name` 字段，由 `compose_entries_runtime` 从调用方
传入的车身名填上；两条后处理规则读它，而不是问一个体叫什么。

## 3. 未收口项

- `cases/kc_quasi_static/contract.py::wheel_centre_body` 仍按 `<stem>_<side>` 单侧找轮心，
  三轴装配会被它按「歧义」拒绝（正确处理，但不是三轴可用状态）。该文件的写范围归 **04**，
  03 不动；缺口由 `tests/subsystems/test_three_axle_assembly.py` 的
  `test_the_three_axle_assembly_itself_is_not_yet_readable_by_the_kc_contract` 写在明处。
- `authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES`（02 登记交 03 收口）本轮未改：
  它服务的是**文档读取层**对「模板引用它不拥有的体」的放行，与装配层的命名规则收口是两件事，
  放开它需要与 06 的子系统文档声明段一起定，故登记为 06 的输入。
