# p3-02 运行日志（命令 + 退出码 + 关键输出，诚实记录）

所有命令在仓库根 `E:\杂件\open-kinematics` 下、经 Git Bash 执行。
时间戳为本次会话（2026-10-01）。**失败与既有失败均照实记录，不隐去。**

---

## A. 交付物

新增：

- `packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py`（引擎，647 行）
- `packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py`（12 个测试）

修改（只改调用段）：

- `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py`

证据脚本与文档（`.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-02-diffkinematics/raw/`）：
`fixtures.py`、`probe_engine.py`、`probe_numerics.py`、`check_tilt.py`、
`engine_contract.md`、`axis_assertions.md`、`numerics_and_comparison.md`、`no_name_sniffing.md`、`layering.md`、`run_log.md`。

## B. 四条验收门（全部实测）

| # | 命令 | 退出码 | 关键输出行 |
|---|---|---|---|
| 1 | `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | **0** | `module cycles (SCC size>1) : 0` / `target modules missing : 0` / `legacy modules unregistered: 0 []` / `OK: layering matches the recorded baseline` |
| 2 | `uv run --no-sync pytest packages/suspension_multibody/tests -q --ignore=.../adams --ignore=.../cases -p no:cacheprovider` | **0** | `1237 passed, 1 xfailed in 648.98s (0:10:48)` |
| 3 | `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | **0** | `mode      : migration` / `findings  : 0` / `OK: no unregistered Python boundary violation` |
| 4 | `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | **0** | `combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` / `OK: dynamic output matches the frozen baseline byte-for-byte` |

第 2 条跑了两次，两次都是 `1 xfailed`（**无新增 skip/xfail**）：
定稿后的最终一次为 `1255 passed, 1 xfailed in 673.08s (0:11:13)`（退出码 `0`）。
两次数值不同（1237 → 1255）是因为会话期间**其他写入者**往 `tests/` 里补了用例，不是本行的改动。
第 4 条的独立哨兵实测：

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
solver self-convergence: FAILED
adams accuracy: BLOCKED
BLOCKED: no real Adams evidence supplied: the 5 percent channel gates and the wall-time ratio gate cannot be evaluated
BLOCKED: the frozen median-of-N timing protocol was not run: pass --performance to collect it
  static_equilibrium: PASSED
  road_step_finite_rise: FAILED
  ... (省略: road_pulse / road_sine / single_wheel_road / in_phase_road / opposite_phase_road / combined_load /
       tire_liftoff_and_recontact / large_amplitude_high_frequency 共 9 条 FAILED)
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency', 'opposite_phase_road',
                    'road_pulse', 'road_sine', 'road_step_finite_rise', 'single_wheel_road',
                    'tire_liftoff_and_recontact']
OK: dynamic output matches the frozen baseline byte-for-byte
$ echo $?
0
```

**关于 `acceptance exit : 1` 与 9 条 FAILED**：这是**记录在案、与本次改动无关的既有状态**。
`dynamic_hash_sentinel.py` 自己的模块 docstring（第 12–15 行）写明：

> A non-zero acceptance exit code is *expected*: three road cases are documented to fail the
> ``time_convergence`` gate on ``fixture.force_z`` alone (``docs/axle_dynamics_results.md``),
> and the project records that honestly rather than loosening the cases.  The sentinel therefore
> records the exit code and the failing case list, and fails only on drift.

哨兵的**判定依据是 combined sha256 是否漂移**，不是 acceptance exit code。
实测 combined sha256 与 SPEC 要求的冻结值逐位相同，故门通过（退出码 0）。
`adams accuracy: BLOCKED` 同理：未提供真实 Adams 证据时该分项按设计记为 BLOCKED，不影响哈希判定。

**关于 `acceptance exit : 1` 与 9 条 FAILED**：这是**冻结基线自身就记录着的状态**，与本次改动无关。
直接读基线文件即可确认（`packages/suspension_multibody/tests/data/dynamic_hash_baseline.json`）：

```
acceptance_exit_code = 1
failed_cases = ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency', 'opposite_phase_road',
                'road_pulse', 'road_sine', 'road_step_finite_rise', 'single_wheel_road',
                'tire_liftoff_and_recontact']
combined_sha256 = fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
```

`dynamic_hash_sentinel.py` 的模块 docstring（第 12–15 行）写明：非零的 acceptance exit code 是
**预期**的，项目如实记录而不放宽用例；哨兵**只在哈希漂移时失败**。
本行实测的 9 条 FAILED、`acceptance exit 1`、以及 `solver self-convergence: FAILED` /
`adams accuracy: BLOCKED`（未提供真实 Adams 证据与冻结计时协议时的设计行为）都与基线逐项相同，
combined sha256 也与 SPEC 要求的冻结值逐位相同，故门通过（退出码 `0`）。**本行没有改任何 K/C 读数。**


另有 SPEC `raw/layering.md` 要求的结构门：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q -p no:cacheprovider
........................................................                 [100%]
56 passed in 594.31s (0:09:54)
$ echo $?
0
```
## C. 基线未动的实测证明

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空输出）
$ echo $?
0

$ git status --short -- packages/suspension_multibody/tests/data/kc_baseline \
      packages/suspension_multibody/tests/data/dynamic_hash_baseline.json \
      packages/suspension_multibody/tests/data/kc_perf_baseline_native.json
（空输出）

$ uv run --no-sync python -c "import hashlib,pathlib; p=pathlib.Path('packages/suspension_multibody/tests/data/dynamic_hash_baseline.json'); print(hashlib.sha256(p.read_bytes()).hexdigest(), p.stat().st_size)"
7f81076b011cb006a6f23fed9b0e75dc6ec6f34806725359214a3b483a49898b 9507
```

即：`tests/data/` 下**没有任何文件被改动**，也未重录任何基线。

## D. 引擎专属测试

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py -v -p no:cacheprovider
12 passed in 1.96s
$ echo $?
0
```

12 条用例名（原文见 `raw/axis_assertions.md` §3）：

```
test_residual_rows_follow_the_kernel_joint_registry
test_the_motion_is_a_screw_on_the_assembled_axle
test_the_drive_value_follows_the_named_point_velocity
test_a_drive_the_mechanism_cannot_meet_is_reported_and_the_drive_is_refused
test_a_single_revolute_joint_has_the_joint_axis_as_its_instant_axis[axis0-point0-4|axis1-point1-3|axis2-point2-5]
test_a_planar_four_bar_reproduces_the_front_view_construction
test_a_pure_translation_has_no_instantaneous_axis_and_is_refused
test_a_constraint_without_a_kernel_residual_is_refused_by_name
test_roll_centers_routes_the_instant_center_to_the_injected_construction
test_no_engine_source_names_a_hardpoint_role
```

## E. 证据脚本

```
$ uv run --no-sync python .codex-tasks/.../raw/probe_engine.py
$ echo $?
0        # 131 行输出，见 raw/engine_contract.md §5 与 raw/axis_assertions.md §4

$ uv run --no-sync python .codex-tasks/.../raw/probe_numerics.py
$ echo $?
0        # 100 行输出，见 raw/numerics_and_comparison.md §1 §2 §3 §4

$ uv run --no-sync python .codex-tasks/.../raw/check_tilt.py
$ echo $?
0        # 见 raw/numerics_and_comparison.md §3.4
```

（`...` 处的完整路径为 `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-02-diffkinematics`。）

## F. 静态检查

```
$ uv run --no-sync ruff check \
      packages/suspension_multibody/src/suspension_multibody/vehicle/screw_kinematics.py \
      packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py \
      packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py \
      .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-02-diffkinematics/raw/
All checks passed!
$ echo $?
0
```

## G. 过程中修掉的两个自身缺陷（如实记录，不隐去）

1. **`_minimum_norm` 的 `null_space` 被截断**：`np.linalg.svd(..., full_matrices=False)` 在宽矩阵上
   只给出 `min(rows,cols)` 个右奇异向量，于是自由方向一旦超过这个数就被静默丢掉。
   改为 `full_matrices=True`。实测影响：双叉臂轴 `nullity` 从错误的较小值变为正确的 `6`，
   整轴 `null_space` 形状 `(72, 6)`。测试 `test_the_motion_is_a_screw_on_the_assembled_axle` 覆盖。
2. **模块 docstring 的舍入误差量级算错**：原写 `eps·|f|/h` 约 `~2e-10`，用的 `|f|` 是装配残差（恒为 0）。
   正确的是 `|f|` 取残差构筑时被相减的坐标尺度（实测最大 1550 mm），量级 `2.2e-7 mm`。
   已按实测改正 docstring（`screw_kinematics.py:24-37`），并把步长扫描的实测值写进
   `raw/numerics_and_comparison.md` §1 —— 实测最优点确在 `h=1e-6`，与改正后的量级判断一致。

## H. 发现但**未修**的既有问题（不在本行范围，仅登记）

- **`roll_centers._instant_center` 把「两内侧硬点取平均」当成了臂的转轴**（`roll_centers.py:94-99`）。
  实测：出厂几何下两内侧硬点的 `[y,z]` 相同，平均值恰好等于前点，所以错误不可见；
  一旦某内侧硬点的 `[y,z]` 不同（几何 B），交点就偏 `26.04` mm（逐位可手算复现，见
  `raw/numerics_and_comparison.md` §3.3(1)）。
  **删除/重构该构造归 p3-03**（本行 SPEC 明确要求不得在此之前删别名表或改主体）。此处只登记。
- **工作树里本行范围外的既有改动**：`packages/suspension_kernel/cpp/**`、`kernel/native.py`、
  `api.py` 等在本次会话开始前已是 modified 状态（`api.py` 中途还一度是并发写入者留下的 `SyntaxError`，
  导致门 2/3/4 短暂无法运行；对方修好后即恢复）。本行**未触碰**这些文件，其内容也不由本行负责。
  `git status` 中这些条目不是本行的产出。
