# p3-04 冻结输出契约（供 p3-05 对接）

本文冻结 `vehicle/static_loads.py` 改造后的**输出字段形状、含义与单位约定**。
p3-05 消费本行的广义静平衡字段时以本文为准。

## 1. 模块与公开入口

文件：`packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py`
`__all__`（`:287`）：

```python
__all__ = [
    "IncompatibleStaticLoadsError",
    "StaticWheelLoadResult",
    "compute_static_wheel_loads",
    "compute_static_wheel_loads_for_assembly",
]
```

| 入口 | 位置 | 签名 / 构造 | 用途 |
|---|---|---|---|
| `compute_static_wheel_loads` | `:125` | `(vehicle: VehicleModel, *, acceleration: np.ndarray \| None = None, gravity: float = 9810.0, road_z: float = 0.0) -> StaticWheelLoadResult` | 旧入口，签名逐字未变。内部 `compose_vehicle_runtime(vehicle, mode="K")` 后转调下一个。生产调用点 `vehicle/service.py:40` 走这里。 |
| `compute_static_wheel_loads_for_assembly` | `:160` | `(assembly: VehicleRuntime, *, acceleration=None, gravity=9810.0, road_z=0.0) -> StaticWheelLoadResult` | 新增。任意接触点数（N）的求解在这里。三轴等多于两轴的车辆只能从装配值进（`VehicleModel` 形状固定两轴）。 |
| `StaticWheelLoadResult` | `:99` | `@dataclass(frozen=True)`，字段见第 2 节 | 返回类型。 |
| `IncompatibleStaticLoadsError` | `:72` | `ValueError` 子类，构造 `(*, residual: float, tolerance: float, contact_points: int)` | 载荷不相容时的报错。**取代**旧的 `ValueError("four wheel support points do not span force/moment balance")`。 |

**单位与坐标系**：长度 mm、力 N、质量 kg（沿用模块既有约定：`gravity` 默认 `9810.0`，
与 `schema/dynamic.py:152` 的 `-9810.0` 同量级，即 mm/s²）。
坐标系为车辆系 `(+x 前, +y 指向正 y 一侧车轮, +z 上)`。
`acceleration` 是 3 分量车辆系加速度，`None` 等价于 `[0, 0, 0]`。

## 2. `StaticWheelLoadResult`（`vehicle/static_loads.py:99`）

| 字段 | 类型 | 含义 | 单位 |
|---|---|---|---|
| `wheel_loads` | `dict[str, float]` | 每个轮端一个竖直反力。键是**该轮端自己的名字**，来自装配（两轴车 `front_left/front_right/rear_left/rear_right`；三轴车另有 `middle_*`；单轮台架 `single_left`）。值与 `support_points` **逐键一致、顺序一致** | N |
| `total_mass` | `float` | 装配体全部刚体质量之和（`assembly.total_mass`） | kg |
| `center_of_mass` | `np.ndarray` 形如 `(3,)` | 质量加权的整车质心，世界系 | mm |
| `support_points` | `dict[str, np.ndarray]` | 每个轮端的接触点，每项 `(3,)` 世界系，z 为 `road_z` | mm |
| `rank` | `int` | 3×N 平衡矩阵 `A` 的秩（`np.linalg.lstsq` 返回的秩） | — |
| `residual` | `float` | 解代回后的残差 `max(abs(A x − b))` | N |
| `unique` | `bool` | **本行新增**。`rank(A) == N` 为 `True`（解唯一）；`rank(A) < N` 为 `False`（最小范数解，解不唯一）。4 轮恒为 `False`；单轮在接触点位于质心正下方且无侧向/纵向加速度时为 `True` | — |
| `residual_tolerance` | `float` | **本行新增**。`residual` 的判定上界；`residual <= residual_tolerance` 即判「载荷相容、解存在」 | N |

属性：`summary -> WheelLoadSummary`（`@property`，`vehicle/static_loads.py:120`）。
实现为 `report.wheel_loads.summarize_wheel_loads(self.wheel_loads)` —— 未改动，
仍要求**恰好四角**（`report/wheel_loads.py:36-38`）。因此：

> **给 p3-05 的注意**：`result.summary` 只在四轮情形可用。三轴/单轮的 `wheel_loads`
> 直接按名字取用即可；`summary` 的通用化按 `EPIC.md` 归 p3-05（`report/wheel_loads.py`
> 与 `report/metrics/vehicle.py` 由 p3-05 改为按安装角色动态生成，
> `SUBTASKS.csv` 的 p3-05 行 (a)）。本行不改这两个文件。

### 4 轮实例（`full_vehicle_model` 同形 fixture）

```
N=4  rank=3  unique=False
residual=9.5367431640625e-07   residual_tolerance=0.01444435545702132
wheel_loads:
  front_left  3611088.8642553305
  front_right 3611088.86425533
  rear_left   3611088.86425533
  rear_right  3611088.86425533
```

### 3 轴实例（6 点接触）

```
N=6  rank=3  unique=False
residual=1.955777406692505e-08   residual_tolerance=0.00400853318553198
wheel_loads: 六个都是 668088.8642553301
  front_left / front_right / middle_left / middle_right / rear_left / rear_right
```

### 单轮实例（N=1，接触点在质心正下方）

```
N=1  rank=1  unique=True
residual=0.0   residual_tolerance=0.00073675886425533
wheel_loads: {"single_left": 736758.86425533}
```

## 3. 两个判定，互不相干

- **解是否存在** ⟺ 载荷相容 ⟺ `residual <= residual_tolerance`。
  容差 = `1e-9 * max(|total_mass * (gravity + accel[2])|, 1.0)`
  （`_RESIDUAL_RELATIVE_TOLERANCE` `:65`、`_MINIMUM_LOAD_SCALE` `:69`、
  `_residual_tolerance` `:243`）。取值理由见 `raw/n_point_cases.md` 第 4 节。
- **解是否唯一** ⟺ `rank(A) == N`，即 `result.unique`。
- **`rank(A) < 3` 不参与任何判定**，不报错。它只表示三条平衡方程不独立。

## 4. `IncompatibleStaticLoadsError`（`vehicle/static_loads.py:72`）

载荷不相容（`residual > residual_tolerance`）时抛出。属性：

| 属性 | 类型 | 含义 |
|---|---|---|
| `residual` | `float` | 残差实测值 |
| `tolerance` | `float` | 判定所用的容差 |
| `contact_points` | `int` | 实际接触点数 N |

消息模板见 `:90`；实测原文（单轮，N=1，侧向 1000 mm/s²）：

```
the static loads are incompatible with the 1 contact point(s) of this vehicle: the residual of the three equilibrium equations is 7330852.1179 and the tolerance is 0.00073675886425533; no set of vertical reactions at those points balances the force and the moments
```

**消息含残差实测值、容差与接触点数 N；不含秩。** 它是 `ValueError` 子类，
`vehicle/service.py:41` 的 `except (ValueError, np.linalg.LinAlgError)` 继续命中，
生产路径行为不变（该处本就不改）。

## 5. 向后兼容

- `compute_static_wheel_loads` 的签名、返回类型、`wheel_loads` 的键与值
  **4 轮逐位一致**（证据见 `raw/four_wheel_bitwise.md`）。
- 新增字段 `unique`、`residual_tolerance` 无默认值且用关键字传入；全仓
  `StaticWheelLoadResult(` 构造点只有本模块自己一处（`vehicle/static_loads.py:217`），
  已核实无其它调用点。
- `wheel_loads` 的键顺序由装配的轮端表给出；四轮情形与旧 `_WHEELS` 相同
  （`front_left, front_right, rear_left, rear_right`）。
