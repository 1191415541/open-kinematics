- 任务：从 build_front_axle 拆出六类子系统（左右悬架／转向／车轮／车身／制动／驱动）
- 形态：single-full（Epic 子任务）
- 进度：1/13 步骤 IN_PROGRESS（包骨架与契约已落，六类子系统本体未实现）
- 当前：`subsystems/{__init__,types,capabilities}.py` 已落（`SubsystemOutput`／`merge_outputs`／`AssemblyCapabilities`／`capabilities_for`／六类枚举与坐标命名空间）；`raw/assembly_snapshot.json` 已抓现役 K/C × `rack_fixed_to_chassis` 四种组合产物。`build_front_axle` 尚未改造，`schema/model.py` 与 `schema/vehicle.py` 尚未新增字段。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-04-subsystems/`
- 验证：`raw/assembly_snapshot.json` 四种组合的计数与 SPEC 验收 2 的实测值一致（K 13/13/0/0、C 9/17/8/8，bodies 10、points 36、connections 16）；`tests/subsystems` 尚不存在，本步无验收证据。

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

## 逐项对照台账（实施时填写）

对照基线：`benchmark_axle.json` 在**改动前**现场抓取并留 `raw/` 的现役产物。

| 对照项 | 现役值 | 组合实现值 | 差异 | 理由（差异非零时必填） |
|---|---|---|---|---|
| `bodies` 集合与顺序 | 10 个，chassis 起、tie_rod_R 止 | 待填 | 待填 | 待填 |
| `points` 键序与数值 | 36 条；K/C 一致 | 待填 | 待填 | 待填 |
| `hardpoints` | 模型硬点 + `__L`/`__R` 副本 + `RACK_CENTER` | 待填 | 待填 | 待填 |
| `connections` | 16 条 | 待填 | 待填 | 待填 |
| `constraints`（K） | 13 条，顺序见 SPEC 验收 2 | 待填 | 待填 | 待填 |
| `ideal_constraints`（C） | 17 条，顺序见 SPEC 验收 2 | 待填 | 待填 | 待填 |
| `bushings` / `elements`（C） | 8 / 8，名称序见上 | 待填 | 待填 | 待填 |
| 驱动坐标定义 | 由 `points` 推导（`cases/kc_quasi_static/contract.py:177-222`） | 待填 | 待填 | 待填 |
| `explicit` 拓扑产物 | `_build_explicit_axle` `:387` | 待填 | 待填 | 待填 |
| `MassSpec` 消费状态 | 轴侧未消费 | 待填 | 待填 | 不得改为消费 |

## 基线台账（实施时填写）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `kc_baseline/k_states.json` | **应为否** | — | — | — | 本步是纯重构 |
| `kc_baseline/c_states.json` | **应为否** | — | — | — | 本步不改 K/C 语义（占位衬套仍为零刚度） |
| `kc_baseline/manifest.json` | **应为否** | — | — | — | 同上 |
| `dynamic_hash_baseline.json` | **应为否** | — | — | — | axle 侧产物逐位不变 |
| `axle_dynamics_baseline/`、`vehicle_dynamics_baseline/` | **应为否** | — | — | — | 同上 |
| `suspension_kernel/layering_baseline.json` | **应为否** | — | — | — | 本步不写 C++ |

## 本步的放行 gate（不得跳过）

1. **逐位一致**：`build_front_axle` 组合路径与改动前的现役实现逐位一致（点表 `np.array_equal`、集合逐顺序）。
2. **未重录任何基线**：`git diff` 在 `tests/data/**` 与 `layering_baseline.json` 上为空。任一基线变化即停止上报，不得顺手重录。
3. **`legacy_surface_gate.py --check` 绿且注册表条目数不变**：新 `subsystems/` 包不得成为新的 legacy import 站点。

## 下一步

等 03 完成后，从 `TODO.csv` 第 1 行开始；先抓现役产物快照入 `raw/`，再逐个拆子系统，每拆一类立即对照。父 `SUBTASKS.csv` 第 04 行状态由主代理回填。
