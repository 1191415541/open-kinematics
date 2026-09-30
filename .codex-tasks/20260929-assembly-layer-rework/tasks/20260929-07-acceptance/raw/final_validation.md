# 07 终局验收：逐条命令与结论（2026-09-29）

## 1. 命令与退出码（全部 0）

| 命令 | 结果 |
|---|---|
| `pytest packages/suspension_multibody/tests -q` | **1526 passed, 1 skipped, 1 xfailed**（1599.77 s） |
| `pytest packages/suspension_multibody/tests/architecture -q` | 147 passed（637.66 s） |
| `pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q` | 65 passed |
| `ruff check .` | All checks passed |
| `ty check .` | All checks passed |
| 三个架构门脚本 | 全部 OK |
| `dynamic_hash_sentinel.py --check` | 26 artifact 逐字节一致，combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（未重录） |
| `kc_native_probe.py` | 9 K states（worst ratio 1.65548e-05） |
| `kc_native_c_probe.py` | 66 C states（worst ratio 0.000185873） |
| `kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` | OK |
| `case_parity_check.py` | OK: 8 families accepted |
| `kc_perf_gate.py --check` | OK（c-66 1.6990 s vs 1.4904 s，x1.140） |
| `snapshot.py --check` | OK: 七组合零差异 |
| `git status --short -- packages/suspension_multibody/tests/data/` | 空（两条基线未被写） |

skip/xfail 未增长。

## 2. 登记审计

`raw/approved_deltas.json` = `[]`。本 Epic 全程（02–06）零产物差异，故无登记项，也不存在「未登记而通过」；05 的允许变化在本轮恰好为空（冻结夹具不声明 `tires`），已在 05 的 `raw/rig_immutability.md` 说明。

## 3. Done-When 逐条

| 条 | 结论 | 证据 |
|---|---|---|
| (a) 3 轴整车装配并跑通；路径无 `front_axle/rear_axle` | **DONE** | 03 的 `test_three_axles_assemble_and_each_keeps_its_own_prefix`、`test_the_three_axle_vehicle_assembles_and_runs_one_study`；grep 零命中（03 的 `raw/entry_grep.md`） |
| (b) 「镜像」与「左右独立文件」两次装配逐项一致 | **NOT MET** | 需要 06 未交付的文档级 `sides`/`mirror` 声明（见 06 的 `raw/symmetry_declaration.md` 第 2 节） |
| (c) 同一 wheel 子系统文件；无删建轮胎；单轴侧凝结且 `kc_baseline` 逐位不变 | **DONE** | 04/04b：`test_wheel_lifecycle.py`、`test_wheel_file_chain.py`、`kc_parity_check --actual-dir`、`snapshot --check` |
| (d) 试验台非侵入 | **DONE** | 05：`test_the_rig_is_not_invasive.py`（10 条，7 rig 覆盖） |
| (e) 显式配对；subframe 改名后同一文件仍装配 | **DONE** | 02：`tests/connections/test_links.py`、`tests/authoring/test_assembly_pairings.py` |
| (f) 7 组合与数值门全绿；重录都有登记 | **DONE** | 本文件第 1 节（零重录、零差异） |
| (g) 拖挂铰接并跑通一次 | **DONE** | 03 的 `tests/subsystems/test_articulated_tow.py`（6 条），本行独立复跑 |

## 4. 结论

**Epic 不结项**：G1–G4、G6 有证据，**G5 未闭环**（06 两项阻断项），Done-When (b) 无证据。`SUBTASKS.csv` 的 `06` = `IN_PROGRESS`；06 的 `TODO.csv` 第 1/2/3/4 行、07 的 `TODO.csv` 第 2/8 行保持 `TODO`。

---

## 刷新（2026-09-29，06 闭环后的最终代码）

| 命令 | 结果 |
|---|---|
| `pytest packages/suspension_multibody/tests -q` | **1531 passed, 1 skipped, 1 xfailed**（1828.23 s） |
| `pytest packages/suspension_multibody/tests/architecture -q` | 147 passed（762.26 s） |
| `pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q` | 65 passed |
| `ruff check .` / `ty check .` | All checks passed |
| 三个架构门脚本 | 全部 OK |
| `dynamic_hash_sentinel.py --check` | 26 artifact 逐字节一致，combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（未重录） |
| `kc_native_probe.py` / `kc_native_c_probe.py` | 9 K states（1.65548e-05）/ 66 C states（0.000185873） |
| `kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` | OK |
| `case_parity_check.py` | OK: 8 families accepted |
| `kc_perf_gate.py --check` | OK |
| `snapshot.py --check` | OK: 七组合零差异 |
| `git status --short -- packages/suspension_multibody/tests/data/` | 空 |

**Done-When 最终判定：(a) DONE　(b) MET　(c) DONE　(d) DONE　(e) DONE　(f) DONE　(g) DONE。**
(b) 的证据：`tests/authoring/test_symmetry_declaration.py::test_the_two_ways_of_writing_one_file_land_on_the_same_template`（模板层逐字段 + 运行层逐体点坐标与约束名一致）。**已登记残余**：写两侧的文件其右侧点 label 带自身 token（坐标一致、仅点名拼写不同）。G1–G6 逐条有证据，**Epic 关闭**。

---

## 提交前复审与处置（2026-09-29）

独立只读复审（code-reviewer）报 2 blocker + 5 should-fix，**全部处置后重跑上表全部命令**（结果与上节相同，数字见 SUBTASKS 的 07 notes）：

| # | 级别 | 问题 | 处置 |
|---|---|---|---|
| 1 | blocker | `sides=["right"] + mirror=true` 通过校验但镜像只从左侧写出右侧，缺左侧实体 | `_check_sides` 拒绝该组合与 `both+mirror`；`_sides_from_file` 只认显式声明过的条目；补断言 |
| 2 | blocker | `merge_rig_link` 沿用旧 `state`，缺 `wheel_carrier_*` | 合并后重建 `RigidBodyState`；`_reorder_bodies` 同步重建 |
| 3 | should-fix | 凝结丢弃车轮体其余点、连接行仍指向已删除的体 | 点按原名跟到挂接体、连接行重指向；补断言 |
| 4 | should-fix | `sides=()` 产生自相矛盾的总成 | `AssemblyRequest.__post_init__` 拒绝空/未知侧 |
| 5 | should-fix | `_per_side_roles` 可能误截字样以 `_L`/`_R` 结尾的 role | 只在摘 token 后不与同体既有 role 撞名时才改 |
| 6 | optional | `_condense_wheel_end` 的 `bodies is None` 分支不可达 | 删除 |

复审确认无问题：单侧 `("L",)` 的顺序无矛盾；既有模板的连接命名未变；台架无第二处轮胎力消费；无新增 skip/xfail。
