- 任务：弹性元件属性文件加载
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 05（模板实例化与 K/C 列激活）未完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-06-properties/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 05 已完成：`templates.instantiate(model, mode, properties=...)` 存在（05 SPEC 目标 1），C 模式衬套刚度已不再全为零，`kc_baseline` 的 C 部分重录已在 05 的 PROGRESS 登记。
- 03 已完成：`PropertySlot` 结构可用（本步消费其 `name` / `kind` / `required` / 默认值；若 03 另带 bounds 则以更严者为准）。
- 本步与 05 的写范围在 `templates/**` 上重叠，因此**必须串行**：05 完成后再开工。
- 父 `EPIC.md` 的 G4（属性文件）与「不得引入新依赖」约束有效。

## 用户裁决的落点（本步是其实现）

用户原话：「衬套、弹簧、减振器等弹性元件由外部属性文件控制属性，也就是说可以加载不同属性文件达到不同刚度、阻尼等等特性」——本步是这条裁决的实现。

配套裁决「零刚度占位衬套可以改为使用模板定义好的默认属性代替」由 05 落地；本步把"模板默认属性"扩展为"模板默认属性或属性文件提供的属性"，因此默认属性文件必须逐位等于 05 的模板默认值，否则 05 的基线立刻失效。

## 本任务的现状事实（制定计划时实测，实施时复核）

- 现役数值入口（本步不删、不改）：`schema/model.py:166-171` 的 `springs` / `dampers` / `bushings` / `tires`；对应类字段在 `schema/elements.py`——`LinearSpring` `:13-25`、`StaticDamper` `:62-76`、`Bushing6x6` `:111-146`、`VerticalTire` `:215-221`、`BumpStop` `:237-248`。
- 严格模型约定：`schema/common.py:12-19` 的 `StrictModel`（`extra="forbid"`、`frozen=True`）；版本化输入文档约定在 `schema/loader.py:20-47`（根对象 + `schema_version == 1`，否则抛 `ValueError`）。
- `tests/data/benchmark_axle.json` 是测试夹具，根键为 `name` / `description` / `model` / `grid`，**无根 `schema_version`**，由 `tests/benchmark_fixture.py:19-31` 直接 `json.loads` 后 `model_validate(payload["model"])`。属性文件不沿用该松散约定（见 SPEC 约束节）。
- C 模式占位衬套现由 05 改为模板默认属性；旧零刚度落点在 `preparation/assembly/front_axle.py:737`、`:802`（`stiffness=np.zeros((6,6))`），本步不得再回退到零刚度。
- `legacy_surface_gate`（`tests/architecture/legacy_surface_gate.py`）在 MODE_MIGRATION 下只容忍注册表既有条目；新包不得导入 `core` / `elements` / `model` / `analysis` / `metrics`。

## 属性文件格式（本 SPEC 冻结，实施时不得另立）

- 载体：JSON，根对象，根 `schema_version == 1`，`properties` 为对象，键为属性槽名。
- 条目字段名与 `schema/elements.py` 对应类**逐一对应**；`kind` ∈ `{spring, damper, bushing6x6, tire, bump_stop}`。
- 取值约束沿用 `schema/elements.py`（`stiffness > 0`、`damping >= 0`、`unloaded_radius > 0`、`clearance >= 0`、形状 6×6 / 6）。
- 夹具位置：`packages/suspension_multibody/tests/data/properties/**`；加载器接受任意路径，不假设目录。

## 三个负例（必须失败并点名）

| 负例 | 输入 | 期望 |
|---|---|---|
| 属性缺失 | `required=True` 且无模板默认值的槽不在文件里 | `ValueError`，含槽名与文件路径 |
| 类型错误 | `stiffness` 给字符串，或给 3×3 而非 6×6 | `ValueError`，含槽名与字段名 |
| 值越界 | `stiffness <= 0`、`damping < 0`、`unloaded_radius <= 0` | `ValueError`，含槽名、字段名与边界 |

有模板默认值的槽在文件中省略是**合法回退**，不算缺失。

## 基线台账（实施时填写）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `kc_baseline/c_states.json` | **应为否** | — | — | — | 默认属性文件逐位等于 05 的模板默认值 |
| `kc_baseline/k_states.json` | **应为否** | — | — | — | 本步只改属性来源，不改 K 列 |
| `kc_baseline/manifest.json` | **应为否** | — | — | — | 同上 |
| `dynamic_hash_baseline.json` | **应为否** | — | — | — | axle 侧产物逐位不变 |
| `axle_dynamics_baseline/`、`vehicle_dynamics_baseline/` | **应为否** | — | — | — | 同上 |
| `suspension_kernel/layering_baseline.json` | **应为否** | — | — | — | 本步不写 C++ |

若某行实际变为"是"，必须写清：哪个步骤导致、重录前后值、判定为模型改变（而非数值噪声）的依据；**禁止先改基线让门变绿**。

## 本步的放行 gate（不得跳过）

1. **三个负例失败且点名**：属性缺失、类型错误、值越界；只报"失败了"不算达成。
2. **换属性文件只改刚度/阻尼**：几何（`points` 键与值、`bodies`、`connections`、`hardpoints`）逐项不变，只有被替换元件的刚度/阻尼变化。
3. **未重录基线**：默认属性文件下 `kc_parity_check.py --check` 通过且 `tests/data/kc_baseline/**` 无 diff。若变化，停止上报而不是顺手重录。

## 下一步

等 05 完成后，从 `TODO.csv` 第 1 行开始：先落格式与加载器（含三个负例），再落 `templates/` 的解析接线，最后跑"可复现 / 换文件"两个对照与门禁。父 `SUBTASKS.csv` 第 06 行状态由主代理回填。
