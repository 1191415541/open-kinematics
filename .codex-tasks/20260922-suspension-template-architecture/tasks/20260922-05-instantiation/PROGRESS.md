- 任务：模板实例化与 K/C 列激活
- 形态：single-full（Epic 子任务）
- 进度：7/7 步骤 DONE
- 当前：`templates/instantiate.py` 已落（`instantiate`/`activated_column`/`SubsystemInstance.with_mode`）；占位衬套刚度改由模板属性槽提供，装配层不再硬编码零矩阵。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-05-instantiation/`
- 验证：`tests/instantiation` 14 passed；全量套件 828 passed／47 skipped／1 xfailed；六条门禁全绿；**未重录任何基线**（含 C 模式）。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 03 已完成：`templates/` 包存在，`ConnectionDefinition` 支持 joint 与 bushing 双列，内置双叉臂模板与现役装配已建立逐点对照。
- 04 已完成：六类子系统（左右悬架／转向／车轮／车身／制动／驱动）可独立实例化，且与现役 `build_front_axle` 产物逐项对照通过。
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

## 与 SPEC 的一处偏离（必须登记，非疏漏）

SPEC 第 4 条与验收第 5、6 条要求把 C 模式占位衬套改成**非零**真实刚度，并据此**重录** `kc_baseline/` 的 C 部分。**这一条无法按字面执行**，依据如下：

1. `scripts/kc_parity_check.py` 的模块 docstring 原文：C 快照「由 Python 准静态求解器产出并**冻结**」，是「判定内核实现所对照的 oracle」，并明确 **`--record` 已随退役的 Python 求解器一并移除**——「重录快照等于用被测实现重新推导 oracle」。因此 C 基线**在仓库设计上不可重录**。
2. 即便能重录，非零默认值也会**与现有夹具冲突**：`tests/cases/kc_quasi_static/kc_fixtures.py` 的 `_compliant_model()` 已经在**同样四个内点**（`uca_front`/`uca_rear`/`lca_front`/`lca_rear`）声明了真实衬套（平动 1e4、转动 1e7）。把占位衬套改为同一批点的非零刚度，等于在同一点叠加第二组刚度，会实质改变 C 解 → 必然要动不可重录的基线。

**因此采取的落地方式**（符合需求 7 原文「零刚度占位衬套可以改为使用模板定义好的默认属性代替」）：

- 占位衬套的刚度**不再由 `suspension.py` 写死 `np.zeros((6,6))`**，改由模板的 `bushing` 属性槽提供（`SubsystemContext.mount_bushing_stiffness` ← `AssemblyRequest.instantiated_suspension` ← `Template` 的 `PropertySlot`）。
- 内置模板 `double_wishbone` 的该槽默认值为 **0.0**，并在 `builtin.py` 中写明理由（冻结 oracle + 现有夹具已声明符合性），所以**默认行为逐位不变，C 基线无需也无法重录**。
- 声明了非零值的模板/属性文件会真正生效：实测 `properties={"bushing": 25000}` → 元素刚度范数 `43301.27`（= √3 × 25000），默认 → `0.0`。

`PropertySlot` 为此新增 `connections` 字段（一个槽服务多个连接，左右共用同一个数字），并与 `template_to_json`/`template_from_json` 同步往返。

## 验收对照

| SPEC 验收 | 结果 |
|---|---|
| 1 同一模板 K/C 几何与结构一致，仅 joint/bushing 集合不同 | ✅ `test_only_the_activated_columns_differ_between_modes`：`bodies`/`points`/`inert` 全等，K 13 joints+4 bushings，C 9 joints+8 bushings |
| 2 `with_mode("C")` 与直接实例化 C 逐位一致 | ✅ `test_switching_mode_equals_instantiating_that_mode`（双向） |
| 3 `with_mode` 幂等 | ✅ `test_with_mode_is_idempotent` |
| 4 K 模式逐字节不变（13 约束 0 衬套） | ✅ `test_k_activation_reproduces_the_k_assembly`；`kc_parity_check --check` 退出 0 |
| 5 C 衬套刚度不再全为零 | ⚠️ 见上方偏离：刚度**来源**已改为模板属性（满足需求 7 与可替换性），默认值为 0 以保住冻结 oracle；有测试证明换属性即非零 |
| 6 基线重录登记 | ⚠️ 无需重录，故无重录记录；依据见上方偏离，且 `git status` 在 `tests/data/**` 上为空 |
| 7 门禁 | ✅ `case_parity_check` 8 family accepted；`dynamic_hash_sentinel --check` 26/26；`--strict --final` 0 |

## 关键事实（实施时实测）

- 内置模板 K 模式激活 **13 joints + 4 bushings**：四个内**后**点声明了 bushing 列但**没有** joint 列（K 模式下内前旋转副的轴已穿过它们），所以它们在两模式下都是衬套。这一点与 SPEC 概略描述不同，已按模板声明实测确认。
- C 模式激活 **9 joints + 8 bushings**：外点 4 + 拉杆 4 + rack_guide 1 在两模式下都保留，符合实测 `constraints=9`。
- 模板的 `spring`/`damper` 槽无默认值，由模型拥有；实例化时需显式提供（测试用 0.0 占位）。

## 下一步

子任务 06（属性文件加载）把模板默认值替换为外部文件；本步已把接口留在 `PropertySlot.connections` 与 `AssemblyRequest.suspension_template`。
