# p3-02 引擎契约：微分运动学引擎的公开面（p3-03 / p3-04 的唯一对接面）

> 本文件是 p3-02 交付的**冻结接口**。p3-03 与 p3-04 各自对着这份签名写，不需要彼此协商。
> 所有内容都是**实跑**结果；重跑脚本见 `raw/probe_engine.py`。

## 1. 引擎落点

```
packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py
```

模块名 `screw_kinematics`。**它不 import `subsystems`、不 import `schema`、不 import `preparation`**——
只依赖 `modeling.primitives`（`Constraint` 声明类、`RigidBodyState`）与 numpy。
因此本行**没有新增 `vehicle → subsystems` 依赖边**（详见 `raw/layering.md`）。

## 2. 输入面

| 输入 | 类型 | 含义 |
|---|---|---|
| `constraints` | `Sequence[Constraint]` | 本次装配的约束集合。装配运行时取 `runtime.constraints` 或 `runtime.ideal_constraints` |
| `state` | `RigidBodyState` | 位姿。装配运行时取 `runtime.state` |
| `drives` | `Sequence[TangentDrive \| PointDrive]` | 施加的驱动（可空：不给驱动时求的是整个自由运动族的最小范数代表） |
| `points` | `Mapping[tuple[str, str], np.ndarray] \| None` | 装配的点表，键 `(body, label)`、值是该 body 的**局部坐标**。装配运行时取 `runtime.points`。`PointDrive` 与 `pivot_point` 需要它 |
| `pivot_body` | `str \| None` | 读哪个 body 的速度旋量。不给且只有一个 body 被驱动时取该 body |
| `pivot_point` | `tuple[str, str] \| None` | 轴上一点报成最接近这个点的那个。键必须属于 `pivot_body` |
| `reference` | `str \| None` | 相对谁。默认取装配里唯一的固定 body（双叉臂轴是 `"ground"`）。若装配无固定 body，则相对世界 |
| `bodies` | `Sequence[str] \| None` | 覆盖列顺序。默认 `free_bodies(...)`（按名字排序） |
| `step` | `float` | 数值微分步长，默认 `NUMERICAL_STEP = 1e-6` |

**引擎不按名字字符串反查几何**：它只读调用方给的 `points` 表与约束声明自带的点/轴。
引擎模块源码里 `UPPER_` / `LOWER_` / `UCA_` / `_BODY_ALIASES` 零命中（`raw/no_name_sniffing.md`）。

### 驱动类型（两个）

```python
TangentDrive(body: str, component: int, rate: float = 1.0)
# component 0..5 = 局部切空间增量 (dx, dy, dz, dtheta_x, dtheta_y, dtheta_z)
# 语义：该 body 的第 component 个局部切坐标速率 == rate

PointDrive(body: str, label: str, direction: np.ndarray, rate: float = 1.0)
# (body, label) 必须是 points 表的键；direction 是世界向量（内部归一化）
# 语义：该点在 direction 方向上的速度分量 == rate
```

## 3. 输出面

`solve_rigid_motion(...) -> RigidMotion`。`RigidMotion` 的字段：

| 字段 | 形状 / 类型 | 含义 |
|---|---|---|
| `bodies` | `tuple[str, ...]` | **列顺序**。`jacobian` 与 `velocity` 都按它编列 |
| `jacobian` | `(rows, 6 * len(bodies))` | 约束行对局部切坐标的雅可比，由中心差分得到 |
| `velocity` | `(6 * len(bodies),)` | 广义速度 `q_dot`，按 `bodies` 编列 |
| `null_space` | `(6 * len(bodies), nullity)` | 驱动之后仍然自由的运动方向（列正交归一） |
| `twists` | `dict[str, Twist]` | 每个 body 的绝对速度旋量 |
| `reference` | `str \| None` | 实际使用的参照 body |
| `pivot_body` | `str` | 实际使用的 pivot body |
| `pivot_point` | `(3,)` | pivot 点的**世界坐标**（锚点） |
| **`omega_rel`** | `(3,)` | `pivot_body` 相对 `reference` 的角速度（世界坐标，rad/单位时间） |
| **`v_rel`** | `(3,)` | `pivot_point` 处、同一相对意义下的线速度（世界坐标） |
| **`screw`** | `Screw` | 空间瞬轴（见下） |
| `constraint_residual` | `float` | `‖J q_dot‖∞`，判断返回的运动是不是机构运动 |
| `drive_residual` | `float` | `‖A q_dot − b‖∞`，判断驱动是否被满足 |
| `rank` | `int` | 堆叠系统 `[J; A]` 的秩 |
| `nullity` | `int` | `6 * len(bodies) − rank`，驱动之后仍自由的维数 |

辅助方法 `RigidMotion.twist_of(body) -> Twist`（未知 body 抛 `KinematicError`）。

### 瞬轴表示（`Screw`，四元最小表示）

```python
Screw(point: np.ndarray, direction: np.ndarray, pitch: float, angular_speed: float)
```

| 字段 | 形状 | 含义 |
|---|---|---|
| `point` | `(3,)` | 轴线上**一点**的世界坐标；具体是「离锚点最近的那个轴点」 |
| `direction` | `(3,)` | **单位**转轴方向（`omega / \|omega\|`） |
| `pitch` | `float` | 沿轴每弧度前进量（螺距），`omega·v / \|omega\|²` |
| `angular_speed` | `float` | `\|omega_rel\|` |

**重构恒等式**（`pivot_body` 的绝对旋量与 `point` 处）：

```
v(point) = omega x (point - point) + pitch * omega = pitch * omega
```

一般世界点 `x`：`v(x) = omega x (x - point) + pitch * omega`。
`point` 在轴上可任意滑动（沿 `direction` 平移不改变螺钉），所以引擎报的是离锚点最近的那个点。

**`omega_rel == 0` 时没有瞬轴**：纯平移（例如单个滑动副）**抛 `KinematicError`**，
消息 `"this motion has no angular velocity, so it has no instantaneous axis: a pure translation is not a screw"`。
不返回零方向——那会让调用方误以为存在一根轴。

### 独立函数

```python
constraint_residual(constraints, state) -> np.ndarray          # 堆叠的位置残差 C(q)
constraint_rows(constraint, state) -> np.ndarray               # 单条约束的残差行
constraint_jacobian(constraints, state, *, bodies=None, step=1e-6) -> (J, bodies)
free_bodies(constraints, state) -> tuple[str, ...]             # 可动 body，按名字排序
screw_from_twist(twist, *, anchor=None) -> Screw
NUMERICAL_STEP = 1e-6
RANK_TOLERANCE = 1e-9
KinematicError(ValueError)                                     # 引擎的全部拒绝都是这个类型
```

## 4. 约束类型支持面（照内核 `cpp/src/joint/registry.cpp`）

有残差行：`PointCoincidence`/`BallJoint`（3 行）、`RevoluteJoint`（5 行）、`WeldJoint`（6 行）、
`PrismaticJoint`（5 行）、`CylindricalJoint`（4 行）、`UniversalJoint`（1 行）、
`InPlaneJoint`（1 行）、`ConstantVelocityJoint`（1 行）。

**没有**残差行、按名字拒绝：`DistanceConstraint`、`CoordinateDrive`，以及内核的 driven 行。
理由：内核 registry 没给它们绑残差，在 Python 侧自己补一个会让本引擎解的约束系统与内核解的不是同一个。

装配出来的双叉臂轴共 16 条声明、65 行。

## 5. 一次真实最小调用（实跑原文）

命令：

```bash
uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-02-diffkinematics/raw/probe_engine.py
```

调用的输入取自装配运行时（`compose_axle(double_wishbone_axle_model(), "K")`，
几何为 `tests/conftest.py::full_vehicle_model` 的前轴）：

```python
motion = screw_kinematics.solve_rigid_motion(
    runtime.constraints,                                   # 16 条声明
    runtime.state,
    [screw_kinematics.PointDrive("wheel_hub_L", "wheel_center", np.array([0.0, 0.0, 1.0]))],
    points=runtime.points,
    pivot_body="upright_L",
    pivot_point=("upright_L", "spindle"),
)
```

实跑输出（`x` 已转成十进制；原文见 `raw/run_log.md`）：

```
constraint declarations        : 16
residual rows                  : 65
movable bodies (columns)       : 12 ['lower_arm_L', 'lower_arm_R', 'rack', 'rack_housing',
                                     'tie_rod_L', 'tie_rod_R', 'upper_arm_L', 'upper_arm_R',
                                     'upright_L', 'upright_R', 'wheel_hub_L', 'wheel_hub_R']
jacobian shape                 : (65, 72)
rank / nullity                 : 66 / 6
constraint_residual |J q|_inf  : 1.802114013571554e-12
drive_residual                 : 1.802114013571554e-12
reference                      : 'ground'
pivot_body                     : 'upright_L'
pivot_point (world anchor)     : array([1400., -750.,  300.])
omega_rel                      : array([ 2.40000000e-03, -1.44068679e-12,  2.19557023e-04])
|omega_rel|                    : 0.002410021842284571
v_rel (at the anchor)          : array([ 1.81394284e-08, -4.80000000e-01,  1.00000001e+00])
screw.point                    : array([ 1418.14456694, -1163.20853781,   101.65990289])
screw.direction                : array([ 9.95841596e-01, -5.97789932e-10,  9.11016734e-02])
screw.pitch                    : 37.80118952118657
screw.angular_speed            : 0.002410021842284571
null_space shape               : (72, 6)
```

``null_space`` 的 6 列是整个自由运动族（只给了一条驱动，`nullity = 6`），见 §7。
`constraint_residual` 与 `drive_residual` 都在 `1e-12` 量级，即返回的运动确实是机构运动且驱动被满足。

### 换 `pivot_body = "wheel_hub_L"`（同一份运动，另一个 body 的旋量）

```
omega_rel   : array([ 0.0024    , -0.00193629,  0.00021956])
screw.point : array([ 1208.43132454, -1001.11349675,   179.46551265])
screw.dir   : array([ 0.77631977, -0.62632568,  0.07101936])
screw.pitch : 120.21817891882621
```

**读数差异来源已定位（不是数值误差）**：`wheel_hub_L` 与 `upright_L` 之间还有一个
`wheel_spin_joint_L`（绕轮轴的转动副），所以轮毂的旋量里多了一个**自由轮转**分量。
`upright` 才是悬架自己的运动；轮毂的 `pitch = 120.2` 是轮转的螺距，不是瞬时轴的偏移。

## 6. 瞬轴与驱动的选择无关（实跑，同一份装配）

| 驱动 | `\|omega\|` | `screw.point` | `screw.direction` | `screw.pitch` |
|---|---|---|---|---|
| 轮心 z 速度 = 1 | 2.4100218423e-03 | `[1418.144566943, -1163.208537807, 101.659902887]` | `[0.995841596388, -5.98e-10, 0.091101673438]` | 37.801189521 |
| 齿条 y 速度 = 1 | 6.6945050980e-03 | `[1418.144567504, -1163.208537689, 101.659902938]` | `[0.995841596258, 8e-11, 0.091101674857]` | 37.801190059 |
| `upright_L` 局部 x 转角速率 = 1 | 1.0041757683e+00 | `[1418.144567531, -1163.20853769, 101.659902939]` | `[0.995841596256, 1.36e-10, 0.091101674878]` | 37.801190041 |

三行的轴点在 `1e-6` mm 内一致、方向在 `1.5e-10` 内一致、螺距在 `1.4e-6` 内一致。
**瞬轴是机构构型的性质，不是驱动的性质**；差的那一点是 `1e-6` 步长数值微分的噪声量级。

## 7. `nullity` 与「解不唯一」的对接约定（**p3-03 / p3-04 必读**）

- `rank` / `nullity` 是**堆叠系统 `[J; A]`** 的，不是 `J` 单独的。
- 双叉臂轴在**只有一条驱动**时 `nullity = 6`（整车 K 工况里齿条与轮跳并不都锁死）；
  再多给驱动 `nullity` 才降。**因此单靠一条驱动读出的瞬轴不是唯一运动族的唯一成员**。
- 引擎返回的是最小欧氏范数解（引擎没有惯量去权衡别的解）。**`null_space` 给出整个自由族**，
  调用方要别的成员就沿 `null_space` 的列加。
- 实测该族对 `upright_L` 瞬轴的播散（沿每个自由方向 ±1e-3）：
  `[9.6598122e-05, 4.6909477e-05, 0.000865505927]` mm。即族内轴点变化 < 1e-3 mm。
- **与今天的 `_instant_center` 对照必须选对族成员**：今天的二维构造按定义是「车体绕 x 侧倾」，
  所以对照支是族里「角速度最接近绕 x」的那个成员，不是默认返回的最小范数支。
  选法（尺度无关，避免靠放大系数压残差）：未知量取族系数 `c` 与绕 x 角速度 `λ`，
  解 `min over (c, λ) | M c + b - λ e_x |`，其中 `b` 是最小范数支在 `pivot_body` 上的角速度、
  `M` 是 `null_space` 各列的作用矩阵；再报 `残差 / |λ|` 作为「离纯绕 x 多远」。
  实测（几何 A，全轴）：残差 `1.1e-16`，与今天 `_instant_center` 一致到 `3.06e-06` mm；
  几何 C 同样 `3.07e-06` mm。完整推导、逐条差异来源与三种几何的对照表见 `raw/numerics_and_comparison.md` §3。
  **不得**把默认的最小范数支当成今天的 `_instant_center` 对照——同一构型同一几何上二者差 `3.458` mm。
  几何 B（上臂转轴被转斜 3.81°）下该族没有精确的绕 x 成员（非绕 x 成分 `1.111e-01`，见 $3 与运行日志），
  对照值仍可给，但必须同时报出这个残差。
  几何 B（上臂转轴被转斜 3.81°）下该族没有精确的绕 x 成员（非绕 x 成分 `1.111e-01`，见 `raw/numerics_and_comparison.md` §3），
  对照值仍可给，但必须同时报出这个残差。
## 8. 错误面（全部是 `KinematicError`，继承 `ValueError`）

| 情形 | 消息片段 |
|---|---|
| 驱动指向不可动 body | `drive names body 'chassis', which is not movable in this assembly; the movable bodies are [...]` |
| `PointDrive` 没给 `points` | `... needs the assembly's point table; pass points= as well` |
| 点表没有该键 | `the point table has no entry ('upright_L', 'not_a_point')` |
| 多个 body 被驱动但没给 `pivot_body` | `... name pivot_body= explicitly` |
| `pivot_body` 不可动 | `pivot body 'x' is not movable in this assembly; ...` |
| `pivot_point` 不属于 `pivot_body` | `pivot point (...) belongs to 'a', not to the pivot body 'b'` |
| `reference` 是可动的 | `reference body 'x' is movable, so it has no motion for the others to be relative to; ...` |
| 纯平移（无角速度） | `this motion has no angular velocity, so it has no instantaneous axis: a pure translation is not a screw` |
| 约束类型无内核残差 | `constraint 'distance' of type DistanceConstraint has no row in this engine: ...` |
| `step` 非正/非有限 | `the differentiation step must be finite and positive, got ...` |

## 9. p3-03 的对接方式（把引擎接到 `roll_centers`）

`vehicle/roll_centers.py` 已经加了**调用段路由**（本行只改调用段，主体与硬点别名表按 SPEC 不动）：

```python
compute_vehicle_roll_centers(
    vehicle, *, road_z=0.0,
    instant_center_engine: Callable[[FrontAxleModel, str], np.ndarray] | None = None,
)
```

- 默认 `None` ⇒ 仍走今天的 `_instant_center`（`tests/physics/test_vehicle_physics.py` 与
  `raw/numerics_and_comparison.md` 的实测默认值 `(-1166.6666666666665, 100.0)` 逐位不变）。
- 传入函数 ⇒ 每个 side 调 `engine(axle_model, "L"/"R")`，返回 2 元 `[y, z]`。
- **依赖留在调用侧**：`roll_centers.py` 不 import 引擎，so 没有新增 `vehicle → subsystems` 边，
  也没有让 `roll_centers` 依赖 `screw_kinematics`。

p3-03 要做的：在调用侧把 `FrontAxleModel` 装配成运行时、跑引擎、
把 `screw` 沿 `null_space` 投到「绕 x 轴」的成员、再取前视穿刺点作为 `[y, z]`。
`raw/numerics_and_comparison.md` §3 有这条链的完整实测数值。

## 10. p3-04 的对接方式（N 点静平衡）

引擎提供的是**速度层**，p3-04 需要的是**静力层**。可复用的是：

- `constraint_jacobian(constraints, state) -> (J, bodies)`：与内核 registry 同源的约束雅可比，
  p3-04 若要把接触点几何放进约束系统，用同一份 `J` 而不是另写一套。
- `free_bodies`：可动 body 的确定性顺序。
- `null_space`：`J` 的零空间即「机构允许的运动」，也就是可以虚位移取的方向。

**引擎本身不产生接触点集合，也不做载荷求解**——那归 p3-04。二者接口不重叠。

## 11. 契约的边界（本行**没有**承诺的东西）

- **不承诺**引擎的**默认**输出等于今天的 `_instant_center`（SPEC 只要求「可对照 + 差异有解释」）。
  默认返回的是最小范数支，与今天的二维构造差 `3.458` mm（几何 A）；按 §7 选对照支后差 `3.06e-06` mm。
  逐条差异来源见 `raw/numerics_and_comparison.md` §3。
- **不承诺**支持 `DistanceConstraint` / `CoordinateDrive`。
- **不承诺** ABI 侧有雅可比 API：本行是**纯 Python 数值微分**，内核 ABI 一字未改（D3）。
- **不承诺**任意构型：引擎对任何装配产物都能算，但「四种构型各给滚转中心」是 p3-03 的判据。
