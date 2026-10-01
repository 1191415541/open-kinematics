# p3-04 证据 (a)：`_WHEELS` 已从生产文件消失

## 1. 判据 grep（逐字原文，Final Validation Command 的第二段）

命令（在仓库根目录执行）：

```bash
bash -c '! grep -rn -e "_WHEELS = (\"front_left\"" packages/suspension_multibody/src/suspension_multibody/vehicle'
```

输出：**空**（grep 零命中，`!` 取反）

```
EXIT=0
```

## 2. 更强的 grep：`vehicle/` 下连 `_WHEELS` 这个名字都没有了

命令：

```bash
bash -c 'grep -rn -e "_WHEELS" packages/suspension_multibody/src/suspension_multibody/vehicle'
```

输出：**空**

```
EXIT=1        # grep 的「无命中」退出码；即零命中
```

即生产文件 `vehicle/static_loads.py` 里没有 `_WHEELS` 常量，也没有任何其它 `_WHEELS` 引用；
`vehicle/` 下另一个文件 `roll_centers.py`（p3-03 交付）也不含它。

对照（改造前）：`_WHEELS` 曾出现在 `vehicle/static_loads.py:28` 并用于 `:82`、`:83`、`:100`
（原始文件已存档为 `raw/static_loads_before.py`，md5 `2b989cfe078bacc5a5969dbb4ba19250`）。

## 3. 接触点从哪里来（实测）

**来源：装配后车辆自己的轮端表 `VehicleRuntime.wheel_centers`。**

`_support_points(assembly, road_z)`（`vehicle/static_loads.py:265`）逐个读
`assembly.wheel_centers`：每项的值是 `(承载轮心的 body, 轮心的 body-local 坐标)`，
取其在世界系中的位置，再把 z 落到路面。因此：

- 集合与顺序都是车辆自己的，不是本模块规定的四轮名字元组；
- 点数 N 由车辆声明了多少个轮端决定 —— 两轴车 4、三轴车 6、单轮台架 1。

实测（`raw/static_loads_before.py` 与改造后同一份装配）：

| 构型 | 装配来源 | `wheel_centers` 键 | N |
|---|---|---|---|
| 两轴四轮（`VehicleModel`） | `compose_vehicle_runtime(model, mode="K")` | `front_left, front_right, rear_left, rear_right` | 4 |
| 三轴（entry 列表） | `compose_entries_runtime(entries, ...)` | `front_left, front_right, middle_left, middle_right, rear_left, rear_right` | 6 |
| 单轮台架（entry，`sides=("L",)`） | 同上 | `single_left` | 1 |

改造后的模型入口多了一层：`compute_static_wheel_loads(vehicle)`（`:111`）只做装配，
真正的求解在 `compute_static_wheel_loads_for_assembly(assembly, ...)`（`:143`）。
模型入口仍按 `compose_vehicle_runtime(vehicle, mode="K")` 装配，与改造前同一行代码。

## 4. 输出字段形状

见 `raw/static_loads_contract.md`（冻结，供 p3-05 对接）。
