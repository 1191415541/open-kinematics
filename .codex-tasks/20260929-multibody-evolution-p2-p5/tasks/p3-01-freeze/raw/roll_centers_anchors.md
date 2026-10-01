# p3-01 / 子任务 1：`roll_centers.py` 与 `static_loads.py` 现状锚点全表复核

只读核查。所有锚点均按实测复核，F7/F8 给出的行号若不符即标注「已过期」并给出真实行号。

基线文件：
- `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py`（132 行）
- `packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py`（132 行）

---

## 1. `roll_centers.py` 硬点别名表（逐条目全表）

锚点：`roll_centers.py:59-67`，F7（`EPIC.md:134`）给出 `:59-67`，**未过期**。
原文（逐字）：

```python
59:_POINT_ALIASES: dict[str, tuple[str, ...]] = {
60:    "upper_front": ("UPPER_INBOARD_FRONT", "UPPER_INNER_FRONT", "UCA_FRONT"),
61:    "upper_rear": ("UPPER_INBOARD_REAR", "UPPER_INNER_REAR", "UCA_REAR"),
62:    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
63:    "lower_front": ("LOWER_INBOARD_FRONT", "LOWER_INNER_FRONT", "LCA_FRONT"),
64:    "lower_rear": ("LOWER_INBOARD_REAR", "LOWER_INNER_REAR", "LCA_REAR"),
65:    "lower_outer": ("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER"),
66:    "wheel_center": ("WHEEL_CENTER", "WHEEL_CENTRE", "WHEEL_CG"),
67:}
```

逐条目（role → 别名元组，一条不漏）：

| # | role | 别名元组（顺序即解析优先级） |
|---|---|---|
| 1 | `upper_front` | `("UPPER_INBOARD_FRONT", "UPPER_INNER_FRONT", "UCA_FRONT")` |
| 2 | `upper_rear` | `("UPPER_INBOARD_REAR", "UPPER_INNER_REAR", "UCA_REAR")` |
| 3 | `upper_outer` | `("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER")` |
| 4 | `lower_front` | `("LOWER_INBOARD_FRONT", "LOWER_INNER_FRONT", "LCA_FRONT")` |
| 5 | `lower_rear` | `("LOWER_INBOARD_REAR", "LOWER_INNER_REAR", "LCA_REAR")` |
| 6 | `lower_outer` | `("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER")` |
| 7 | `wheel_center` | `("WHEEL_CENTER", "WHEEL_CENTRE", "WHEEL_CG")` |

- 共 **7 条 role、21 个别名字符串**；`UPPER_*` 写法 3 条（`UPPER_INBOARD_FRONT` / `UPPER_INNER_FRONT` / `UPPER_OUTBOARD`……），`LOWER_*` 写法 3 条，`UCA_*` / `LCA_*` 各 3 条（`UCA_FRONT`/`UCA_REAR`/`UCA_OUTER`，`LCA_*` 同）。
- **注意**：这份表是 `roll_centers.py` **私有的第二份**别名表，与 `subsystems/geometry.py:37-80 HARDPOINT_ALIASES` 不是同一份。`geometry.py` 的同名 role 收录更多写法（例如 `upper_front` 另有 `UCA_INNER_FRONT`、`UPPER_FRONT`，见 `geometry.py:46-52`）。G3 判据 `grep -n "UPPER_INBOARD\|LOWER_INBOARD\|UCA_\|_BODY_ALIASES"` 在新引擎路径须零命中，即须删掉本表。
- 全仓 `_BODY_ALIASES` 命中 1 处：`subsystems/geometry.py:257`（与 roll centre 无关，属 body 名解析，G3 判据把它一并列出属「同名嗅探」的集合口径）。

## 2. `roll_centers.py::_hardpoint`（别名解析实现）

锚点：`roll_centers.py:70-78`。原文：

```python
70:def _hardpoint(axle: FrontAxleModel, role: str, side: str) -> np.ndarray:
71:    side_points = side_hardpoints(axle.hardpoints, side)  # type: ignore[arg-type]
72:    normalized = {
73:        key.upper().replace("-", "_"): value for key, value in side_points.items()
74:    }
75:    for alias in _POINT_ALIASES[role]:
76:        if alias in normalized:
77:            return normalized[alias].as_array()
78:    raise ValueError(f"missing hardpoint for roll-center role {role}")
```

**依赖方向（F10 / `EPIC.md:140` 点名的依赖，记为事实基线）**：
- `roll_centers.py:22` `from ..subsystems.geometry import side_hardpoints` —— 即 `vehicle → subsystems` 依赖边**今天已经存在**，不是新引入。这一条对 p3-02 的 `test_import_boundaries.py` 判定有直接影响。

## 3. `roll_centers.py::_instant_center`（`_line_intersection` 的调用者与 `[y,z]` 二维化）

锚点：`roll_centers.py:81-99`，F7（`EPIC.md:134`）给出 `:81-99`，**未过期**。原文：

```python
81:def _instant_center(axle: FrontAxleModel, side: str) -> np.ndarray:
82:    upper_inner = 0.5 * (
83:        _hardpoint(axle, "upper_front", side) + _hardpoint(axle, "upper_rear", side)
84:    )
85:    lower_inner = 0.5 * (
86:        _hardpoint(axle, "lower_front", side) + _hardpoint(axle, "lower_rear", side)
87:    )
88:    upper_outer = _hardpoint(axle, "upper_outer", side)
89:    lower_outer = _hardpoint(axle, "lower_outer", side)
90:    upper_line = np.array(
91:        [[upper_inner[1], upper_inner[2]], [upper_outer[1], upper_outer[2]]]
92:    )
93:    lower_line = np.array(
94:        [[lower_inner[1], lower_inner[2]], [lower_outer[1], lower_outer[2]]]
95:    )
96:    intersection = _line_intersection(*upper_line, *lower_line)
97:    if intersection is None:
98:        raise ValueError(f"{side} suspension arm lines are parallel")
99:    return intersection
```

- `:91` 与 `:94` 取的就是**分量 [1], [2]**，即 `[y, z]` 前视平面。上臂内点取 `upper_front`/`upper_rear` 的中点（`:82-84`），下臂同理（`:85-87`）：这是双叉臂「两条臂线」假设的硬编码所在。
- 这是**唯一**支持的构型假设：5 连杆每侧 5 根杆无「上臂线」，麦弗逊无上臂，扭梁无臂线（见 `config_refusals.md` 的实测）。

## 4. `roll_centers.py::_line_intersection`（退化判据）

锚点：`roll_centers.py:117-129`。F7（`EPIC.md:134`）写的 `_line_intersection`，`:96` 实测是**调用行**（见上 `_instant_center:96`），**定义在 `:117-129`**。**记为锚点补全**（不是过期，是 F7 未给定义行）。原文：

```python
117:def _line_intersection(
118:    point_a: np.ndarray,
119:    point_b: np.ndarray,
120:    point_c: np.ndarray,
121:    point_d: np.ndarray,
122:) -> np.ndarray | None:
123:    direction_a = np.asarray(point_b, dtype=float) - np.asarray(point_a, dtype=float)
124:    direction_b = np.asarray(point_d, dtype=float) - np.asarray(point_c, dtype=float)
125:    matrix = np.column_stack((direction_a, -direction_b))
126:    if abs(float(np.linalg.det(matrix))) <= 1e-12:
127:        return None
128:    parameters = np.linalg.solve(matrix, np.asarray(point_c) - np.asarray(point_a))
129:    return np.asarray(point_a, dtype=float) + parameters[0] * direction_a
```

**退化判据**：`:126` `abs(float(np.linalg.det(matrix))) <= 1e-12` → 返回 `None`。
- 判据是「两条方向向量构成的 2×2 矩阵的行列式绝对值 ≤ 1e-12」，即两线平行/共线。
- 无尺度归一：行列式量纲是「长度²」，1e-12 是**绝对阈值**而非相对阈值；毫米与米两种单位下同一几何的判定不同。
- 返回 `None` 的两条抛错链路：`_instant_center:97-98`（`"{side} suspension arm lines are parallel"`）与整车层 `:48-49`（`"{axle_name} roll-center lines are parallel"`）。

## 5. `roll_centers.py` 整车层二维交点

锚点：`roll_centers.py:42-47`（F7 给出 `:42-47`，**未过期**）；第二次 `_line_intersection` 调用在 `:47`。原文 `:41-56`：

```python
41:    results: dict[str, RollCenterResult] = {}
42:    for axle_name, axle in (("front", vehicle.front_axle), ("rear", vehicle.rear_axle)):
43:        left_ic = _instant_center(axle, "L")
44:        right_ic = _instant_center(axle, "R")
45:        left_contact = _contact_front_view(axle, "L", vehicle, axle_name, road_z)
46:        right_contact = _contact_front_view(axle, "R", vehicle, axle_name, road_z)
47:        center = _line_intersection(left_contact, left_ic, right_contact, right_ic)
48:        if center is None:
49:            raise ValueError(f"{axle_name} roll-center lines are parallel")
50:        results[axle_name] = RollCenterResult(
```

接地点：`_contact_front_view`（`:102-114`）关键行原文：

```python
109:    point = _hardpoint(axle, "wheel_center", side)
113:    del vehicle, axle_name
114:    return np.array([point[1], road_z], dtype=float)
```

`:114` 把轮心 **y** 与路面高度 `road_z` 组成 `[y, z]` 点——接地点前视投影。`vehicle` 与 `axle_name` 被 `:113` 显式丢弃（当前未使用）。

## 6. `roll_centers.py` 数据类与 `__all__`

- `:25-32` `@dataclass(frozen=True) class RollCenterResult`，字段 `axle: str / center: np.ndarray / left_instant_center: np.ndarray / right_instant_center: np.ndarray`。
- `:132` `__all__ = ["RollCenterResult", "compute_vehicle_roll_centers"]`。
- `:4-12` 模块 docstring 明说这是「a vehicle-level geometric construction, not a solve」，且说它放在 `vehicle/` 是因为 `vehicle/` 是允许 import `preparation` 的层；`report/` 不得有 `report -> preparation` 边。

## 7. `compute_vehicle_roll_centers` 调用者盘点（grep 全仓）

命令（仓库根执行，排除 `.codex-tasks/` 下的任务书本身）：

```bash
grep -rn "compute_vehicle_roll_centers" --include=*.py --include=*.md . | grep -v "^./.codex-tasks"
```

真实输出（5 行；2 行定义/导出，1 行文档，2 行唯一测试调用者）：

```
./packages/suspension_multibody/README.md:71:`vehicle/static_loads.py`，`compute_vehicle_roll_centers` 迁至
./packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:35:def compute_vehicle_roll_centers(
./packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:132:__all__ = ["RollCenterResult", "compute_vehicle_roll_centers"]
./packages/suspension_multibody/tests/physics/test_vehicle_physics.py:5:from suspension_multibody.vehicle.roll_centers import compute_vehicle_roll_centers
./packages/suspension_multibody/tests/physics/test_vehicle_physics.py:56:    centers = compute_vehicle_roll_centers(full_vehicle_model)
```

结论：
- **生产调用者：0 个**（`def` 行与 `__all__` 行不是调用；`README.md:71` 是迁移记录文字）。
- **测试调用者：1 个** —— `tests/physics/test_vehicle_physics.py:56`（导入在 `:5`；测试函数 `def` 在 `:55`）。
- F7（`EPIC.md:134`）给出的起点锚点 `:5/55` 实测：导入 `:5` 精确；**调用行是 `:56` 不是 `:55`**（`:55` 是 `def test_...` 行）。**记为偏移 1 行**。
- 对照：`compute_static_wheel_loads` **有** 1 个生产调用者 —— `vehicle/service.py:40`（`static_wheel_loads = compute_static_wheel_loads(model).wheel_loads`），且包在 `:39-42` 的 `try/except (ValueError, np.linalg.LinAlgError)` 内，失败置 `None`。这是 p3-04「4 轮逐位一致」硬门的依据。

---

## 8. `static_loads.py::_WHEELS` 全表（逐元素）

锚点：`static_loads.py:28`，F8（`EPIC.md:136`）给出 `:28`，**未过期**。原文：

```python
28:_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
```

逐元素：`[0] = "front_left"`、`[1] = "front_right"`、`[2] = "rear_left"`、`[3] = "rear_right"`。共 4 个字符串，写死四轮。
G4 判据要求 `grep -n "_WHEELS = \(\"front_left\""` 在 `vehicle/` 与 `report/` 无命中。

同一份四轮字段在 report 层**重复定义**（F8 点名；本行未逐行重读 report 层，按 F8 记录，不冒充实测）：`report/wheel_loads.py:17 _WHEELS`、`:26-31` 的 `front_axle`/`rear_axle`/`front_rear_delta`/`left_side`/`right_side`/`right_left_delta`、`:36-38` 强制恰好四角；`report/metrics/vehicle.py:21-43` 重复定义 `normal_load_front_axle`、`load_transfer_front_minus_rear`。

## 9. `static_loads.py` 3×4 平衡矩阵与 `np.linalg.lstsq`

锚点：`static_loads.py:79-95`，F8 给出 `:79-95`，**未过期**。原文：

```python
79:    matrix = np.array(
80:        [
81:            np.ones(4),
82:            [support_points[name][0] - center_of_mass[0] for name in _WHEELS],
83:            [support_points[name][1] - center_of_mass[1] for name in _WHEELS],
84:        ],
85:        dtype=float,
86:    )
87:    rhs = np.array(
88:        [
89:            total_mass * (gravity + accel[2]),
90:            -total_mass * height * accel[0],
91:            -total_mass * height * accel[1],
92:        ],
93:        dtype=float,
94:    )
95:    loads, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=1e-12)
```

- 矩阵第 0 行 `np.ones(4)`（竖力）、第 1 行 x 偏移（俯仰矩）、第 2 行 y 偏移（侧倾矩）→ **A 是 3×4**。
- 未知量 4 个（四轮）、方程 3 条 → 欠定，取最小范数解（`rcond=1e-12`）。`loads` 长度 4。
- `_support_points`（`:118-129`）遍历 `vehicle.wheels`，用 `assembly.wheel_centers[wheel.name]` 取轮心世界点并把 z 拉到 `road_z`（`:125-128`）。
- `_center_of_mass`（`:109-115`）按 `assembly.bodies` 的质量加权。

## 10. `static_loads.py` 的「秩 / 残差」两处现状（p3-04 要改的口径）

### 10.1 字段声明（`:39-40`）

实测原文：

```python
31:@dataclass(frozen=True)
32:class StaticWheelLoadResult:
33:    """Quasi-static vertical support reactions for the four contact points."""
34:
35:    wheel_loads: dict[str, float]
36:    total_mass: float
37:    center_of_mass: np.ndarray
38:    support_points: dict[str, np.ndarray]
39:    rank: int
40:    residual: float
```

锚点 `:39-40` = `rank: int` 与 `residual: float`，**未过期**。`:42-44` 另有 `summary` 属性（转发 `summarize_wheel_loads`）。

### 10.2 `residual` 的计算口径（`:96`）

实测原文：

```python
95:    loads, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=1e-12)
96:    residual = float(np.max(np.abs(matrix @ loads - rhs)))
```

SPEC/TODO 引用的 `float(np.max(np.abs(matrix @ loads - rhs)))` 与 `:96` **逐字一致，锚点未过期**。
口径：这不是 `lstsq` 返回的第 4 个值（那个被 `:95` 的 `_` 丢弃），而是**重新算的无穷范数残差** `‖A x − b‖∞`。

### 10.3 `rank < 3` 抛错段（`:97-98`）

实测原文：

```python
97:    if rank < 3:
98:        raise ValueError("four wheel support points do not span force/moment balance")
```

锚点 `:97-98` **未过期**（F8 写 `:98`：`:98` 是 `raise` 行，`:97` 是条件行）。

### 10.4 两处的用途现状（p3-04 的纠正对象）

- **`residual` 今天只记录、不参与抛错**：`:96` 算出后只写进返回值 `:105`（`residual=residual`），**全文无 `if residual > ...`**。核对方式：`static_loads.py` 132 行内 `residual` 仅出现 3 次 —— `:40` 声明、`:96` 赋值、`:105` 传参，无比较。
- **抛错由 `rank < 3` 决定**，与 `residual` 无关。这正是 `EPIC.md:89-93`（G4 判据修订）与 `:263(b)`（p3-04 判据）要反转之处：**存在性 ⇒ 载荷相容性（残差在容差内）**；**唯一性 ⇒ `rank(A) == N`**；而 `rank < 3`「只表示三条平衡方程不独立，与存在性、唯一性无关、不得据此报错」。
- 后果（单轮 N=1）：今天 `rank ≤ 1 < 3` 必然抛 `ValueError: four wheel support points do not span force/moment balance`；按修订口径，单轮方程相容时 `rank = 1 = N`、解唯一，必须给出可解数值例。**今天这条路径被无条件拒绝。**

### 10.5 `static_loads.py` 其余锚点

- `:47-53` `compute_static_wheel_loads(vehicle, *, acceleration=None, gravity=9810.0, road_z=0.0)`。
- `:64-65` `gravity <= 0.0 or not np.isfinite(gravity)` → `ValueError("gravity must be finite and positive")`。
- `:67-68` `accel.shape != (3,) or not np.all(np.isfinite(accel))` → `ValueError("acceleration must contain three finite values")`。
- `:74` `assembly = compose_vehicle_runtime(vehicle, mode="K")`；`:76-77` `assembly.total_mass` / `_center_of_mass(assembly, total_mass)`；`:78` `height = center_of_mass[2] - road_z`。
- `:99-106` 返回值构造；`:100` `wheel_loads={name: float(loads[index]) for index, name in enumerate(_WHEELS)}`（索引顺序即 `_WHEELS` 顺序）。
- `:132` `__all__ = ["StaticWheelLoadResult", "compute_static_wheel_loads"]`。
- `:24-26` 依赖：`..report.wheel_loads`、`..schema`、`..subsystems.vehicle_assembly` → `vehicle → report` 与 `vehicle → subsystems` 两条边今天都存在。

---

## 11. 锚点过期/偏移汇总

| 来源 | 引用锚点 | 实测 | 判定 |
|---|---|---|---|
| F7 | `roll_centers.py:59-67` 别名表 | `:59-67` | 未过期 |
| F7 | `roll_centers.py:81-99 _instant_center` | `:81-99` | 未过期 |
| F7 | `roll_centers.py:96 _line_intersection` | `:96` 是**调用行**；定义在 `:117-129` | 补全（F7 只给调用点） |
| F7 | `roll_centers.py:42-47 / :117-129` 整车层交点 | `:42-47`，第二次调用在 `:47` | 未过期 |
| F7 | 调用者 `tests/physics/test_vehicle_physics.py:5/55` | 导入 `:5` 精确；调用在 `:56`（`:55` 是 `def`） | **偏移 1 行** |
| F8 | `static_loads.py:28 _WHEELS` | `:28` | 未过期 |
| F8 | `static_loads.py:79-95` 3×4 矩阵 + `lstsq` | `:79-95` | 未过期 |
| F8 | `static_loads.py:98` 抛错 | 条件 `:97`、`raise` `:98` | 未过期（`:97-98` 段） |
| `EPIC.md:249(a)` / TODO | `residual` 口径 `:96` | `:96` 逐字一致 | 未过期 |
| `EPIC.md:249(a)` / TODO | 字段声明 `:39-40` | `:39-40` | 未过期 |

按 `EPIC.md:107` 的口径，本行唯一偏移项是 F7 的测试调用行 `:55` → **`:56`**，已记。

## 12. p3-03 / p3-04 调度关系（对齐父表，不写「可并行」）

- 生产写范围：p3-03 → `vehicle/roll_centers.py`；p3-04 → `vehicle/static_loads.py`，**文件级不相交**。
- 测试写范围**相交**：`tests/physics/test_vehicle_physics.py` 同时覆盖滚转中心（`:55-64`，`compute_vehicle_roll_centers`）与静平衡（`:9-27`、`:30-38`、`:41-52`、`:67-88`，`compute_static_wheel_loads`），**归 p3-03**。
- 结论（照 `EPIC.md:228`、`:253(e)` 与 `SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`）：**p3-03 → p3-04 硬串行，不得并行。文件级切分不是并行的理由。**
