- 任务：试验台抽取与总成 × 试验台正交组合
- 形态：single-full（Epic 子任务）
- 进度：0/12 步骤 TODO，尚未实施
- 当前：未开工。前置 09（准静态/动态 study 合并）未完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-10-rigs/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 09 已完成：准静态/动态 study 概念落地，同一装配入口可用。
- 07 已完成：输出声明与衍生输出机制可用（本步把总成输出与试验台输出接进组合）。
- 04/05 已完成：六类子系统与模板实例化可用（总成的"可驱动坐标名义空间"从这里来）。
- 父 `EPIC.md` 的 G7（正交组合）与验证协议中 10 的验收口径有效。

## 本步对应的用户需求原文

- 「我设想的对于任意仿真应该是不同总成+试验台的组合，比如 kc_quasi_static 是单轴总成+悬架 KC 试验台、vehicle_kc 是整车总成（整车总成又相当于两个单轴总成+其他子总成）+悬架 KC 试验台、axle_dynamic 是单轴总成+悬架 KC 试验台」
- 「所有仿真的输出由总成+试验台定义，总成带有它所定义的输出，试验台也带有它独有的输出」

## 本任务的现状事实（制定计划时实测，实施时复核）

现有 7 个写死的组合，每个一个 compiler（`simulation/dispatch.py:20-33` 的 `default_registry`）：

| # | assembly | family | compiler |
|---|---|---|---|
| 1 | `axle` | `kc_quasi_static` | `KcQuasiStaticCompiler` |
| 2 | `axle` | `axle_dynamic` | `AxleDynamicCompiler` |
| 3 | `vehicle` | `vehicle_dynamic` | `VehicleDynamicCompiler` |
| 4 | `vehicle` | `vehicle_kc` | `VehicleKcCompiler` |
| 5 | `vehicle` | `handling` | `HandlingCompiler` |
| 6 | `vehicle` | `ride_four_post` | `RideFourPostCompiler` |
| 7 | `vehicle` | `ride_random_road` | `RideRandomRoadCompiler` |

内核侧 family 枚举在 `cpp/src/cases/case_dispatch.cpp:17-19`（7 个 + `comparison` 按设计 N/A，`case_parity_check` 亦标其为 N/A）。

**"试验台职责"尚未成型的证据**（模型侧与 case 侧的边界因 family 而异）：

- `axle × kc_quasi_static`：驱动坐标由**模型侧**声明（`cases/kc_quasi_static/contract.py:177-222` 的 `_driven_coordinates`），case 只给网格值（`api.py:403-411` 的 `_case_envelope`）。
- `vehicle × ride_four_post`／`ride_random_road`／`handling`：case 侧**只产出 `case_document`**（如 `cases/ride_four_post.py:39`、`cases/handling.py:82`），模型文档复用 `cases/vehicle_dynamic.py` 的 `model_document`。
- `vehicle × vehicle_kc`：模型文档由 `cases/vehicle_kc.py:101` 的 `model_document` 在整车文档基础上**追加驱动关节**，并移除转向执行器（`:118-122`）。

这三类边界互不相同，正是本步要统一的。

## 本步的边界

- **不改各 family 的物理语义**：驱动坐标含义、时间网格意义、求解器默认值都不动。
- **不删除现有 7 个组合**，它们必须继续可用且行为不变。
- **不做"总成组合"**（整车 = 两个轴总成之和）：那属于 04 的子系统与 05 的实例化；本步只做**总成 × 试验台**这一维。

## 基线重录台账（实施时填写；本步预期不应重录）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `case_parity_check.py` 8 family 快照 | **应为否** | — | — | — | 现有组合行为不变 |
| `vehicle_dynamics_baseline/sha256.json` | **应为否** | — | — | — | 同上；若变须单独裁决 |
| `dynamic_hash_baseline.json`（26 artifact） | **应为否** | — | — | — | axle 侧不受本步影响 |

**若现有 7 个组合中任何一个行为变化**：先判断是"重构引入的回归"还是"物理改变"。前者必须修掉；后者必须单独裁决并登记——**不得以"重构"为名顺手重录 vehicle 系列基线**。

## 全量套件基线（主代理实测）

`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` → `783 passed, 1 skipped, 1 xfailed`（退出 0）。

## 下一步

等 09 完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 10 行状态由主代理回填。
