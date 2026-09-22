- 任务：从 build_front_axle 拆出四类子系统（左右悬架／转向／轮胎／车身）
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 03（模板与连接点双列数据模型）未完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-04-subsystems/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 03 已完成：`templates/` 包存在，`ConnectionDefinition` 支持 joint 与 bushing 双列，内置双叉臂模板与现役装配已建立逐点对照。
- 02 已完成：统一副表 `joints/` 可用（本步的副编码沿用其名称）。
- 父 `EPIC.md` 的 G2（三层架构、四类子系统粒度）与「不得修改 `templates/**`」约束有效。
- 与 03 串行：两者都触及 `preparation/assembly/types.py`，不得同时进行。

## 用户裁决的落点（本步是其实现）

用户原话：「子系统是左右悬架、转向、轮胎、车身，左右悬架加转向加轮胎加悬架实验台得到悬架实验总成，前后悬架加转向加车身加轮胎加整车 kc 实验台得到整车实验总成」——本步把四类子系统从 `build_front_axle` 中拆出，就是这条裁决的实现；「前后悬架加转向加车身加轮胎」说明整车侧是同一批子系统的复用，因此子系统必须是可独立实例化、可复用的单元。

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
