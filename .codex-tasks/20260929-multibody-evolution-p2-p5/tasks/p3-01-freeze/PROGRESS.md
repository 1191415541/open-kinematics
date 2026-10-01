# PROGRESS：p3-01 冻结现状事实与判据（阶段三）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-01`

## 状态

`DONE`（2026-10-01）。五条判据全部实测，证据落 `raw/`。本行是**纯只读冻结行**：未写任何生产代码，
未改 `packages/**`、`EPIC.md`、`SUBTASKS.csv`。

## 交付物

| 判据 | 文件 | 内容 |
|---|---|---|
| (1) 锚点全表 | `raw/roll_centers_anchors.md`（301 行） | 硬点别名表**全表**（7 条 role→别名）、`_instant_center`/`_line_intersection`/整车层二维交点的 `file:line` + 原文、`static_loads.py` 的 `_WHEELS`/3×4 矩阵/`lstsq`、`:96` residual 与 `:97-98` `rank < 3` 抛错段原文、调用者盘点、锚点过期表 |
| (2) 四构型盘点 | `raw/config_refusals.md`（261 行）+ `raw/out_configs.txt` | 双叉臂**可构造**；5 连杆/麦弗逊/扭梁在滚转中心链路与模板注册路径的**拒绝原文**；并证明三者走 `explicit` 拓扑都能装配 |
| (3) 断言原文 | `raw/tests_physics_assertions.md`（181 行） | 5 个测试函数、17 条 `assert` 逐条原文（行号 + 量名 + 判定方式） |
| (4) 基线影响面 | `raw/baseline_impact.md`（181 行） | roll center 字段 **0 命中**（附命令与退出码）+ 两处基线完整字段清单 + `roll_stiffness`/`track_change` 位置 + 四份 sha256 前后一致 |
| (5) 起点自证 | `raw/run_log.md`（138 行） | 44 条命令全表（命令 + 退出码 + 关键输出）、只读自证、未能验证清单、并发写入者观察 |

## 实测要点（供 p3-02 / p3-03 / p3-04 使用）

- **F7 锚点部分过期**：`compute_vehicle_roll_centers` 生产调用者 **0**（确认 F7 的关键事实）；唯一测试调用者在
  `tests/physics/test_vehicle_physics.py:5`（导入）与 **`:56`**（调用，F7 写 `:55` 偏移 1 行，`:55` 是 `def` 行）。
- **四构型的真实差别**：双叉臂可构造（实测中心 `[1.14e-13, -180.0]`）；5 连杆/麦弗逊/扭梁在**滚转中心链路**与
  **模板注册路径**一律拒于同一处 —— `ValueError: missing hardpoint for roll-center role upper_front`
  （`vehicle/roll_centers.py:78`，调用栈 `:43 → :83 → :78`）；模板路径的注册期原文是
  `TemplateError: template 'five_link_probe' does not satisfy role 'suspension': missing mount(s) [...]`。
  **但拒绝不在模型声明层**：三者走 `topology="explicit"` 都能装配（`compose_axle` 约束数 10 / 6 / 3），
  整车 `compose_vehicle_runtime` 也接受（5-link `constraints=24`；twist-beam `constraints=8`）。p3-03 的
  三构型断言必须建立在这个事实上，而不是「它们今天装不出来」。
- **p3-03 没有现成的数值对照基准**：既有 5 个测试只断言键集合、有限性、`center[0] == 0`、左右瞬心 x 对称，
  **`center[1]`（滚转中心高）从未被断言**。故「与改造前可对照」必须由 p3-02/p3-03 自己产生基准值
  （本行探测给的 `-180.0` 不是基线）。
- **p3-04 的改造输入已固化**：`static_loads.py:96` 的 `residual` 只写进返回值（`:105`），**全文无比较**、
  不参与抛错；抛错只由 `:97-98` 的 `rank < 3` 触发。既有测试里 `:12` 的 `assert result.rank == 3` 与
  `:18` 的 `assert result.residual < 1e-6` 是把旧口径写死的两条硬门——p3-04 改判据时必须同步处理 `:12`
  （按 `EPIC.md:249(a)` 与 G4 的两层口径：存在性由载荷相容性判、唯一性由 `rank(A) == N` 判，
  `rank < 3` 只表示方程不独立）。
- **调度关系实测确认**：`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`，两行**硬串行**。生产写范围
  （`roll_centers.py` vs `static_loads.py`）不相交，但**测试写范围相交**（`tests/physics/test_vehicle_physics.py`
  同时覆盖滚转中心与静平衡，归 p3-03）——文件级切分不是并行的理由。
- **基线**：`kc_baseline/` 与 `dynamic_hash_baseline.json` 的 roll center 类字段 **0 命中**（命令与退出码 1 已记录）；
  `roll_stiffness` 只有常量（`adams/vehicle_parameters.py:40`）、`track_change` 只有声明
  （`templates/builtin.py:393`、`templates/roles.py:85`）。四份基线 sha256 工作前后一致，**未重录**。
- **起点既有失败：无**（`pytest tests/physics` = `5 passed`，退出码 0）；`tests/physics` 目录无 skip/xfail 输出，
  起点 skip/xfail 计数未增长。

## 未做（本行边界，已记入 `raw/run_log.md` H 节）

- 四构型各自跑通一次 study/solve（超出本行判据，且会触及 `cases/`）。
- 麦弗逊滑柱真实约束形式、5 连杆 10 球面副的过约束可解性、扭梁扭杆耦合语义（本行只验「能否构造」）。
- `kc_parity_check.py` **未跑**（`EPIC.md:231` 判定其不带 `--actual-dir` 时恒过、不构成证据）。
- `tests/data/**/sha256.json` 与 `kc_perf_baseline_native.json`（非本行范围，未读）。

## 环境观察（必须随行传递，避免验收误读）

本行执行期间 `packages/suspension_kernel/cpp/**` 有 **7 个文件**被修改（`git diff --stat` = 238 insertions /
8 deletions）。**这不是本行所改**：本行全部写入只落在本目录 `raw/`、`probe_*.py` 与会话 scratch；
该集合的 mtime（02:31–02:37）与本行命令时间线（02:33–02:39）交叉，属**另一并发写入者**
（p2-02 内核旋转力矩元方向）。本行的只读探测只依赖 Python 侧 `suspension_multibody`，
`pytest tests/physics` 两次复跑结果一致，证据未受污染。
