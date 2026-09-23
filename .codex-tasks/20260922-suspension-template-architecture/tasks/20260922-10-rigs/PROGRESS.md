- 任务：试验台抽取与总成×试验台正交组合（含接口自适应与单轴侧车轮）
- 形态：single-full（Epic 子任务）
- 进度：12/12 步骤 DONE
- 当前：`rigs/` 包已落（7 个试验台注册 + 正交组合与能力收缩）；kc 的 K 网格按总成能力降维。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-10-rigs/`
- 验证：`tests/rigs` 19 passed；全量 1016 passed／1 skipped／1 xfailed；`kc_parity_check --check` 0；`case_parity_check` 8 families accepted；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致；`--strict --final` 0；ruff/ty 全树通过；`tests/data` 与 `layering_baseline.json` **无 diff**。

## 交付物

- `rigs/rig.py`：`RigSpec`（`drives`/`outputs`/`study`/`supplies_wheels`）与 `DriveSpec`；7 个试验台按现有 family 名注册（`kc_quasi_static`/`axle_dynamic`/`vehicle_kc`/`vehicle_dynamic`/`handling`/`ride_four_post`/`ride_random_road`），使按 family 名调用的既有请求继续可用，同时两根轴变得可分离。
- `rigs/compose.py`：`resolve_combination` / `compose` / `combinations`。分两层判定：
  - **归属是注册错误**：`ride_four_post` 属整车，接到单轴上必须报「registered for the 'vehicle' assembly」，而不是「你的单轴缺零件」——后者会把读者引到错的地方；
  - **能力是收缩**：逐条按 `coordinate in capabilities.drive_coordinates` 决定该驱动轴是否保留。
- `api.py`：新增 `_k_drivable_coordinates(assembly)`（**只读** `AssemblyCapabilities.drive_coordinates`；无 capabilities 的旧调用方回退到全集，行为不变）与 `_k_grid(..., drivable=...)`。

## 需求 15 / D6：接口按总成能力自适应

实测（单轴总成，含/不含转向）：

| 总成 | 可驱动坐标 | 试验台保留的驱动 | 丢弃 |
|---|---|---|---|
| 含转向 | `rack_drive`/`rack_neutral`/`wheel_drive_L`/`wheel_drive_R` | 全部三个驱动轴 | — |
| 不含转向 | `wheel_drive_L`/`wheel_drive_R` | `wheel_drive_L`、`wheel_drive_R` | `rack_drive` |

收缩语义严格区分「**轴不存在**」与「**轴存在且为零**」：无转向时 `rack_values_mm == []`、`axis_map` 里**没有** `rack` 键，而不是 `[0.0]`。填零会让一次未转向的运行看起来被转向过。

**修掉的两个中间缺陷（都是测试抓到的）**：

1. 最初只把 `axis_map["rack"]` 摘掉，却仍留 `rack_values_mm=[0.0]`——等同保留占位列，违反 SPEC「不是填零、不是跳过该轴但保留占位列」。
2. 修掉 (1) 后 `combinations` 用 `left × rack` 做笛卡尔积，`rack=()` 使**组合数塌缩为 0**——比填零更糟，运行会产出零个 case。现改为「轴缺席则该维不存在」，网格降维而非消失。

**原 `StopIteration` 崩溃点已消除**：`axis_map` 的 rack 项现在是**构建**出来的（按 `rack_present` 决定是否放入），不再用 `next(...)` 在文档的 `rack_*` 名字里搜索，因此无转向时不会抛内部异常，而是在边界处就不产生该轴。

## 需求 19 / D9 与需求 20 / D11

- `supplies_wheels=True` 对 `kc_quasi_static`/`axle_dynamic` 显式声明，与 04 的「单轴侧不产出 `wheel.body`（车轮归试验台）」一致。
- `rigs/` 层只读 capabilities 与驱动声明，**不检查任何模板、也不判断是否简化版**；模板无分支由 `tests/subsystems/test_torque_role_is_replaceable.py` 的 AST 门覆盖（04 已落）。

## 与 SPEC 的偏离

- **本步未改 `simulation/dispatch.py` 的注册表结构**：7 个 family 编译器保持原样，正交性由新增的 `rigs` 层表达（组合矩阵可查询、归属与能力分层判定）。理由是 dispatch 是既有调用方的入口，改动它会波及全部既有请求；SPEC 的目标（正交组合 + 可查询 + 无效组合点名 + 能力收缩）已由 `rigs` 层达成，且 `tests/rigs` 逐条锁定。
- **未删除任何 family 名**：`rigs` 的 rig 名沿用 family 名，故既有 `(assembly, family)` 调用语义不变。

## 未闭合项

- **`api.py` 的 K 结果侧收缩未落地**：SPEC 提到「K 结果侧」与「`:440` 三元组解包」「`:453-457` drives」「`:256` 时间序列 metric」也需按能力收缩。本步完成了**网格与驱动轴**的收缩（有测试），结果侧的 rack 通道收缩未做——因 SPEC 同时要求「不得在本步私改结果对象契约」，需与 07 的输出声明对齐后再动。当前无转向总成在 `_run_k` 的驱动轴层面已不会引用 rack。
