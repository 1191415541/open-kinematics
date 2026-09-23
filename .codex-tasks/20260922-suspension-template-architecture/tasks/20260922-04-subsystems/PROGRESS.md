- 任务：从 build_front_axle 拆出六类子系统（左右悬架／转向／车轮／车身／制动／驱动）
- 形态：single-full（Epic 子任务）
- 进度：9/13 步骤 DONE，4/13 TODO（7-10 属后续修订，见文末登记）
- 当前：单轴侧六类子系统拆分已完成并逐位验证通过；`build_front_axle` 已改为子系统组合。制动/驱动的简化模板与可用性矩阵（TODO 7-10）未落地。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-04-subsystems/`
- 验证：全量套件 814 passed／47 skipped／1 xfailed；`tests/subsystems` 19 passed；四种组合逐位一致（独立脚本 + 测试双重证据）；六条门禁全绿；未重录任何基线。

## 恢复信息

**本步已开工（包骨架与能力契约已落），六类子系统本体与 `build_front_axle` 改造尚未实施。** 以下为开工前必须核验的约束，已全部满足：

- 03 已完成：`templates/` 包存在，`ConnectionDefinition` 支持 joint 与 bushing 双列，内置双叉臂模板与现役装配已建立逐点对照。
- 02 已完成：统一副表 `joints/` 可用（本步的副编码沿用其名称）。
- 父 `EPIC.md` 的 G2（三层架构、六类子系统粒度与可用性矩阵）、G2b（简化→复杂可扩展性）与「不得修改 `templates/**`」约束有效。
- 与 03 串行：两者都触及 `preparation/assembly/types.py`，不得同时进行。

## 用户裁决的落点（本步是其实现）

用户原话：「子系统是左右悬架、转向、轮胎、车身……」（需求 13）与「我想把制动系统和驱动系统以及轮胎也分别实现为子系统」（需求 16）、「悬架实验总成不需要制动、驱动子系统，整车总成必须要」（需求 17）、「当前制动和驱动子系统可以做成简化实现，但是一定要有能力在后续的模板中可以扩展成带刚体的复杂形式」（需求 20）——本步把六类子系统从 `build_front_axle` 拆出，并让制动/驱动以**可替换的简化模板**落地。

## 本任务的现状事实（制定计划时实测，实施时复核）

- `build_front_axle` 单函数同时产出四类内容：`preparation/assembly/front_axle.py:625-926`。
- 实测（`tests/data/benchmark_axle.json`，本 SPEC 制定时实跑）：`bodies=10`（顺序 `chassis, rack, upper_arm_L, lower_arm_L, upright_L, tie_rod_L, upper_arm_R, lower_arm_R, upright_R, tie_rod_R`）、`points=36`（K 与 C 键序与数值完全一致）、`connections=16`。
- 实测计数：K 模式 `constraints=13`／`ideal_constraints=13`／`bushings=0`／`elements=0`；C 模式 `constraints=9`／`ideal_constraints=17`／`bushings=8`／`elements=8`（8 条衬套名称序为 `uca_bushing_{S}_inner_front, uca_bushing_{S}_inner_rear, lca_bushing_{S}_inner_front, lca_bushing_{S}_inner_rear`，L 后 R）。
- 轮胎子系统落点 `:561-574`；转向落点 `:838-903`；车身落点 `:644`（轴侧）与 `vehicle.py:99`（整车侧）。
- `MassSpec`（`schema/model.py:28-60`，字段 `:165`）在轴侧装配中**从未被读取**；轴侧只读 `RigidBodySpec.mass`（`front_axle.py:174`、`:191`）。
- `anti_roll_bars`（`:591-601`）跨左右（`upright_L` / `upright_R`），只能归"左右悬架"这一含双侧的子系统。
- `legacy_surface_gate`（`tests/architecture/legacy_surface_gate.py`）在 MODE_MIGRATION 下只容忍注册表既有条目（`legacy_surface_registry.json` 的 `entry[1]`、`entry[2]` 为 `front_axle.py` / `vehicle.py` 导入 `elements`）；新包直接导入 `elements` 会失败。

## 逐项对照台账（2026-09-23 实测填写）

对照基线：`benchmark_axle.json` 在**改动前**现场抓取并留 `raw/assembly_snapshot.json` 的现役产物。四种组合（K/C × `rack_fixed_to_chassis` 真/假）全部实跑比对。

| 对照项 | 现役值 | 组合实现值 | 差异 | 理由 |
|---|---|---|---|---|
| `bodies` 集合与顺序 | `chassis, rack, upper_arm_L, lower_arm_L, upright_L, tie_rod_L, (R 同序)` | 完全相同 | **无** | — |
| `points` 键集与数值 | 36 条，K/C 一致 | 完全相同（逐键 `np.array_equal`） | **无** | 插入序未比对：快照用排序键序列化且无消费方依赖插入序（已核实） |
| `hardpoints` | 模型硬点 + `__L`/`__R` 副本 + `RACK_CENTER` | 完全相同（键集） | **无** | — |
| `connections` | 16 条，顺序见快照 | 完全相同 | **无** | `rack_guide` 不进连接表（与原实现一致） |
| `constraints`（K） | 13 条，`rack_guide` 收尾 | 完全相同（名称 + 类型，逐序） | **无** | — |
| `ideal_constraints`（C） | 17 条 | 完全相同（名称 + 类型，逐序） | **无** | — |
| `bushings` / `elements`（C） | 8 / 8 | 完全相同（名称序） | **无** | 占位衬套仍为零刚度，未改语义 |
| 驱动坐标定义 | 由 `points` 推导 | 点表一致即推导一致 | **无** | 无转向时 `capabilities.drive_coordinates` 不含 `rack_*` |
| `explicit` 拓扑产物 | `_build_explicit_axle` | 逐行保留（仅补 `capabilities`） | **无** | 路径未被拆分触及 |
| `MassSpec` 消费状态 | 轴侧未消费 | 仍未消费 | **无** | 该事实已写进 `subsystems/chassis.py` 模块文档 |

**逐位比对方式**：主代理亲写独立脚本（放会话 scratch，不采信实现方自证）读 `raw/assembly_snapshot.json` 与夹具，逐组合比对上述八个维度，输出「ALL FOUR COMBINATIONS MATCH」；同一判据落在 `tests/subsystems/test_assembly_matches_snapshot.py`（含硬点数值、连接行全字段、约束几何与 C 模式占位刚度）。

## 基线台账（2026-09-23 实测填写）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `kc_baseline/k_states.json` | **否** | — | 未变 | 未变 | `git status` 为空；`kc_parity_check --check` 退出 0 |
| `kc_baseline/c_states.json` | **否** | — | 未变 | 未变 | 同上；占位衬套仍为零刚度 |
| `kc_baseline/manifest.json` | **否** | — | 未变 | 未变 | 同上 |
| `kc_perf_baseline*.json` | **否** | — | 未变 | 未变 | 同上 |
| `dynamic_hash_baseline.json` | **否** | — | `e7407656…` | 同一值 | `dynamic_hash_sentinel --check` 退出 0，26/26 逐位一致 |
| `axle_dynamics_baseline/`、`vehicle_dynamics_baseline/` | **否** | — | 未变 | 未变 | `case_parity_check` 8 family accepted |
| `suspension_kernel/layering_baseline.json` | **否** | — | `0ba7571a…` | 同一值 | `check_module_layering --strict --final` 退出 0 |

## 本步的放行 gate（全部通过）

1. **逐位一致**：`build_front_axle` 组合路径与改动前的现役实现逐位一致（点表按键 `np.array_equal`、集合与列表逐顺序），四种组合全部无差异。
2. **未重录任何基线**：`git status --porcelain packages/suspension_multibody/tests/data packages/suspension_kernel/layering_baseline.json` 为空。
3. **`legacy_surface_gate.py --check` 绿且注册表条目数不变**：仍为 8 条已注册 `legacy_module_import`，`subsystems/` 未成为新的 legacy import 站点（该包不导入 `elements`/`core`/`model`/`analysis`/`metrics`）。

## 门禁实测（2026-09-23）

| 命令 | 结果 |
|---|---|
| 全量 `pytest packages/suspension_multibody/tests` | `814 passed, 47 skipped, 1 xfailed`（较 01 基线 737 多 77 = 02 的 27 + 03 的 31 + 04 的 19） |
| `tests/subsystems` | 19 passed |
| `kc_parity_check.py --check` | 退出 0 |
| `case_parity_check.py` | 退出 0，8 families accepted |
| `dynamic_hash_sentinel.py --check` | 退出 0，26/26 逐位一致 |
| `legacy_surface_gate.py --check` | 退出 0，8 findings 全为已注册条目 |
| `check_module_layering.py --strict --final` | 退出 0 |
| `ruff check .` / `ty check .` | All checks passed |

## 本步**未**完成、不得标为 DONE 的部分（如实登记）

SPEC 把六类子系统一次列全，但本步只拆到单轴侧实际存在的内容。以下四步仍为 TODO，属**后续修订**范围，不是本轮遗漏：

- TODO 7（简化制动子系统）、TODO 8（简化驱动子系统）：需 `schema/vehicle.py` 的制动参数子集与整车侧接线，本步按 SPEC 未触碰 `schema/**`。
- TODO 9（简化/复杂模板可替换性硬门）、TODO 10（可用性矩阵测试）：依赖 7、8 先存在。
- 因此 TODO.csv 状态为 9 DONE / 4 TODO，**不是 13/13**。`SUBTASKS.csv` 第 04 行仍为 `IN_PROGRESS`。

## 下一步

启动子任务 05（模板实例化与 K/C 列激活），或先补齐本步 TODO 7-10 所需的 `schema/vehicle.py` 制动参数子集。父 `SUBTASKS.csv` 第 04 行状态由主代理回填。
