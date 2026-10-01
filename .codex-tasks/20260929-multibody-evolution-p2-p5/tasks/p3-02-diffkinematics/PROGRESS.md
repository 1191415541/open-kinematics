# PROGRESS：p3-02 微分运动学引擎（速度旋量与空间瞬轴）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-02`

## 状态

`DONE`（2026-10-01）。六条 `TODO.csv` 行全部完成，全部 `validation_command` 实跑通过。
四条验收门（`check_module_layering --strict --final`、快速 pytest 集、`legacy_surface_gate --check`、
`dynamic_hash_sentinel --check`）退出码均为 `0`。

## 交付物

| 判据 | 文件 | 内容 |
|---|---|---|
| 引擎本体 | `packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py`（647 行） | 注入式约束集合 + 点表 → `omega_rel` / `v_rel` / 空间瞬轴；只 import `modeling.primitives` + numpy |
| 调用段路由 | `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py` | 只改调用段：加 keyword-only `instant_center_engine` 回调；别名表与 `_instant_center` / `_line_intersection` 一字未动 |
| 引擎测试 | `packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py`（12 个用例） | 12 passed |
| (1) 契约 | `raw/engine_contract.md`（267 行） | 公开签名、输入/输出形状、瞬轴表示与容差约定、一次最小调用实跑输出、错误面、p3-03/p3-04 各自对接方式 |
| (2) 解析解断言 | `raw/axis_assertions.md` + 上述测试 | 单旋转副 3 个参数化用例（含斜轴）；容差 `1e-9` / `1e-4 mm` / `1e-6` 各有实测理由 |
| (3) 数值微分与对照 | `raw/numerics_and_comparison.md`（234 行） | 步长扫描实测、截断/舍入量级、平面四连杆精确退化、双叉臂并列对照表、差异逐条解释 |
| (4) 无名称嗅探 | `raw/no_name_sniffing.md` | 引擎模块 grep 退出码 `1`（零命中）；目录级非零命中归属说明 |
| (5) 分层 | `raw/layering.md` | 两门（+`legacy_surface_gate`）退出码与关键输出 |
| (6) 运行日志 | `raw/run_log.md`（180 行） | 全部门与脚本的命令 + 退出码 + 关键输出，含失败与既有状态 |
| 证据脚本 | `raw/fixtures.py`、`raw/probe_engine.py`、`raw/probe_numerics.py`、`raw/check_tilt.py` | 均退出码 `0`，本行报告里每个数字都可复跑 |

## 实测要点（供 p3-03 / p3-04 使用）

- **接口已冻结**：`solve_rigid_motion(constraints, state, drives, *, points, pivot_body, pivot_point, reference, bodies, step)`。
  输入是**调用方注入**的约束集合与点表，引擎不装配、不按名字查几何。`__all__` 共 14 项，见 `raw/engine_contract.md` §3。
- **依赖边为零**：引擎只 `vehicle → modeling`；`roll_centers` 通过回调路由，不 import 引擎。
  故 SPEC 第 92 行的「加重 `vehicle → subsystems`」风险**未发生**；`test_import_boundaries.py` 56 passed 是独立证据而非前置条件。
- **`nullity = 6` 而非常见误解的 0**：双叉臂轴只给一条驱动时，堆叠系统 `[J; A]` 的秩是 66 / 72 列。
  引擎如实暴露 `null_space`（72×6）。**默认返回的是最小范数支**，它与今日 `_instant_center` 差 `3.458` mm；
  与今日构造可比的支是「角速度最接近绕 x」的那一支，需调用方沿 `null_space` 自选。
  选法写成尺度无关的 `min |M c + b − λ e_x|`（`raw/numerics_and_comparison.md` §0）。
- **对照结果**（正视图平面 `x = 1400`）：
  - 几何 A（出厂）：今天 `(-1166.6666666666665, 100.0)`；引擎对照支 `(-1166.6666697267606, 100.00000022753248)`，Δ = `−3.06e-06` mm，非绕 x 成分 `1.1e-16`。
  - 几何 C（`LOWER_OUTBOARD` → `(1400,-780,90)`）：Δ = `−3.07e-06` mm，非绕 x 成分 `1.5e-14`。
  - 几何 B（`UPPER_INBOARD_REAR` → `(1700,-500,520)`）：今天 `(-1140.625, 100.0)`，引擎全轴对照支 `(-1166.545215952889, 100.01552156094023)`，Δy = `−25.9202` mm。
- **几何 B 的差异是今日构造的错误，不是引擎的，且已逐位手算复现**：`UPPER_INBOARD_REAR` 不是铰链，
  但上臂的**转轴方向**由它与 `UPPER_INBOARD_FRONT` 共同决定（`subsystems/suspension.py:332` → `_local_axes`），
  所以移动它把真转轴转了 `3.814°`。今日公式取两内侧硬点平均 ⇒ z 偏 10 ⇒ 正视图交点偏 `26.0417` mm。
  同理，出厂几何下两个内侧硬点的 `[y,z]` 恰好相同，所以这个错误**在出厂几何上完全不可见**。
- **`check_tilt.py` 的单调性**：后点从 z=500 抬到 800（倾角 0°→45°），`|引擎−今日|` 从 `3.1e-06` 单调放大到 `2.09e+02` mm，
  而 `|引擎−真轴构造|` 始终 `≤ 1.5` mm。归因因此钉死。
- **解析解精度地板**：单旋转副的轴上点垂直距离 `1.2e-7` / `5.7e-7` / `1.1e-8` mm，量级由 `h=1e-6` 的舍入项 `2.2e-7` 决定。
  步长扫描证明 `h=1e-6` 就是实测最优点（`1e-7`、`1e-8` 处误差按 `1/h` 放大到 `1.0e-6`、`1.1e-5`）。
- **`roll_centers` 的默认路径逐位不变**：`test_roll_centers_routes_the_instant_center_to_the_injected_construction`
  与 `tests/physics/test_vehicle_physics.py` 共 6 passed，默认值仍是 `(-1166.6666666666665, 100.0)`。
- **零 K/C 影响已主动自证**（不靠「没被调用」推断）：`dynamic_hash_sentinel --check` 退出码 `0`，
  combined sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（与 SPEC 冻结值逐位相同）；
  `git status --short -- packages/suspension_multibody/tests/data/` 空输出。

## 已知问题与遗留

- `raw/no_name_sniffing.md` §6 登记：整 `vehicle/` **目录级** grep 非零命中，全部来自 `roll_centers.py:72-77 _POINT_ALIASES`。
  本行按 SPEC 不得在 p3-03 之前删它，所以目录级零命中归 p3-03。这是设计约束，不是缺陷。
- 本行未为几何 B 那种「两臂转轴不平行于 x」的构型给出替代的二维对照读法（例如改到臂轴平面内做二维化）。
  该构型的非绕 x 成分是 `1.111e-01`（左四杆）/ `2.2e-05`（全轴），即正视图构造本身不再是精确瞬心 —— 这是发现，
  处置归 p3-03。
- 「四种构型」中另两种（5 连杆、麦弗逊、扭梁）的解析解验证**未在本行做**；按 p3-01 `raw/config_refusals.md`，
  本行解析解先落在可构造的构型（单旋转副 + 双叉臂）上。
- 过程记录：本行修掉自己两个缺陷（`_minimum_norm` 的 `null_space` 在宽矩阵上被 `full_matrices=False` 截断；
  docstring 的舍入量级把 `|f|` 误取成装配残差）。两者都写进 `raw/run_log.md` §G。

## 并发写入者观察（如实登记，不归本行）

会话中途 `packages/suspension_multibody/src/suspension_multibody/api.py` 曾被**另一写入者**留下 `SyntaxError`
（第 121 行 docstring 未闭合），导致门 2/3/4 一度无法运行（`pytest` 14 个 collection error、两个脚本 `ast.parse` 失败）。
对方修复后四门全部恢复通过。本行**未触碰** `api.py`（它在禁改清单上），也未触碰 `packages/suspension_kernel/**`、
`kernel/native.py`。工作树里这些文件的 modified 状态与本行无关。
