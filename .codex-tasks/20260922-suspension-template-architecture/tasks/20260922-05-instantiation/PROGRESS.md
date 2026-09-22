- 任务：模板实例化与 K/C 列激活
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 03（模板数据模型）与 04（四类子系统拆分）均未完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-05-instantiation/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 03 已完成：`templates/` 包存在，`ConnectionDefinition` 支持 joint 与 bushing 双列，内置双叉臂模板与现役装配已建立逐点对照。
- 04 已完成：四类子系统（左右悬架／转向／轮胎／车身）可独立实例化，且与现役 `build_front_axle` 产物逐项对照通过。
- 父 `EPIC.md` 的 G3 口径与 D4（基线重录授权）有效。

## 用户裁决的落点（本步是其实现）

用户原话：「模型装配好后也可以任意切换 K、C 模式」——本步的 `with_mode` 是这条裁决的实现。
用户原话：「零刚度占位衬套可以改为使用模板定义好的默认属性代替」——本步第 6 个步骤是这条裁决的实现。

## 本任务的现状事实（制定计划时实测，实施时复核）

- C 模式占位衬套现为零刚度：`preparation/assembly/front_axle.py:737` 与 `:802` 的 `stiffness=np.zeros((6,6))`；实测 8 条衬套的刚度范数（`abs(stiffness).sum()`）**全为 0.0**。
- 实测计数（`benchmark_axle.json`）：K 模式 `constraints=13`／`ideal_constraints=13`／`bushings=0`；C 模式 `constraints=9`／`ideal_constraints=17`／`bushings=8`。
- `FrontAxleAssembly`（`front_axle.py:82-95`）只带**单一模式**的产出：K 模式 `bushings=()`，C 模式内点的 `RevoluteJoint` 不存在。因此现状无法原地互切，本步要改变这一点。
- K 模式基准文件：`packages/suspension_multibody/tests/data/kc_baseline/k_states.json`；C 模式：`c_states.json`；另有 `manifest.json`。

## 本步的放行 gate（不得跳过）

1. **`with_mode("C")` 与直接 `instantiate(mode="C")` 逐位一致**。这是"切换不是第二套实现"的唯一证据。做不到就是没达成。
2. **K 模式逐字节不变**。K 模式是既有 `kc_baseline/k_states.json` 的基准；若 K 也变了，说明误改了 K 列语义，必须停止上报，而不是顺手重录。
3. **基线重录必须登记**。本步是 EPIC 中预期重录 `kc_baseline` C 部分的步骤；重录前必须先有"零刚度 vs 模板属性"的差异定量（证明是模型改变），重录后在 `PROGRESS.md` 记明：基线文件 + 导致重录的步骤 + 重录前后的值 + 判定依据。

## 基线重录台账（实施时填写）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `kc_baseline/c_states.json` | 待定 | 步骤 6-7 | 待填 | 待填 | 待填 |
| `kc_baseline/k_states.json` | **应为否** | — | — | — | K 模式不应受本步影响 |
| `kc_baseline/manifest.json` | 待定 | 步骤 7 | 待填 | 待填 | 待填 |
| `dynamic_hash_baseline.json`（26 artifact） | **应为否** | — | — | — | axle 侧不经过本步改动 |

## 下一步

等 03 与 04 完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 05 行状态由主代理回填。
