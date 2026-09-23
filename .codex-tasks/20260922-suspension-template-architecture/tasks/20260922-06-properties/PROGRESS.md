- 任务：弹性元件属性文件加载
- 形态：single-full（Epic 子任务）
- 进度：9/9 步骤 DONE
- 当前：`properties/` 包已落（冻结格式 + `load_properties`）；`templates/` 新增 `resolve_properties`，按槽名把属性集绑定到模板；默认属性文件逐位复现 05 的模板默认值。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-06-properties/`
- 验证：`tests/properties` 18 passed；六条门禁全绿；`tests/data/kc_baseline/**` 无 diff（未重录）。

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

## 落地方式与 05 的接口差异（登记）

SPEC 目标 3 与约束假定注入路径是 `instantiate(model, mode, properties=...)`；05 实际落地的是 `instantiate(template, *, mode, properties)`，且 `properties` 是**槽名 -> 标量**的字典。本步未改 `preparation/`（SPEC 明令禁止），改为在 `templates/` 侧新增解析入口 `resolve_properties(template, property_set)`，把属性文件解析成 05 需要的那个字典。SPEC 允许这条路径：约束只禁止改 `preparation/`/`subsystems/`/`outputs/`，并允许写 `templates/**`。

## 格式与规则（R1-R3 实测）

- JSON 根对象 + `schema_version == 1` + `properties` 对象；未知根键拒绝。
- 条目必须有 `kind` ∈ `{spring, damper, bushing6x6, tire, bump_stop}`，其余字段与 `schema/elements.py` 同名同义；未知字段拒绝。
- **必填字段规则**：只强制 schema 类中**无默认值**的字段；有默认值的字段（如 `Bushing6x6.damping`）省略合法。放置类字段（`name`/`body_a`/`body_b`/`point_a`/`point_b`/`contact_point`）属性文件不必提供——数字归文件、位置归模型。
- 取值约束沿用 `schema/elements.py`（`stiffness > 0`、`damping >= 0`、`unloaded_radius > 0`、`clearance >= 0`、6×6 形状），不另立一套；这是 R3 的唯一来源（03 的 `PropertySlot` 未提供 bounds，已登记）。
- 报错抛 `PropertiesError(ValueError)`，消息含**文件路径 + 属性名 + 字段名 + 原因**。

## 标量槽与矩阵属性之间的桥

模板槽是标量（一个 N/m），而属性条目可以是完整 6×6。规则（两侧一致、非有损）：

- 6×6 平动对角**各向同性** → 取该对角值填入槽；
- 平动对角**各向异性** → 拒绝并点名槽与对角值（不静默取首项、不丢信息）。

`damper` 的标量字段是 `viscous_damping`（非 `stiffness`），映射表在两处保持一致。

## 验收对照

| SPEC 验收 | 结果 |
|---|---|
| 1 格式有 schema 级约束，R1-R3 均有测试，未知根键与条目键被拒 | 18 项含根键/条目键/kind/必填/越界/形状负例 |
| 2 同模板 + 同文件逐位一致 | `test_same_file_twice_is_reproducible` |
| 3 换文件只改刚度/阻尼，几何逐项不变 | `test_a_different_file_changes_stiffness_and_nothing_else`（bodies/points/connections/hardpoints/constraints 全等，仅刚度变） |
| 4 三个负例失败并点名 | 缺失（`resolve_properties` 点名槽 + 文件路径）、类型错（点名属性 + 字段 + 路径）、越界（点名属性 + 字段 + 路径） |
| 5 模板直接给数值的现役路径仍可用 | `test_the_default_file_reproduces_the_template_defaults_bit_for_bit`：默认文件与模板默认值实例化结果相等 |
| 6 门禁保持且未重录基线 | 六条门禁退出 0；`tests/data/kc_baseline/**` 无 diff |

## 交付物

- `src/suspension_multibody/properties/{__init__,load}.py`
- `templates/instantiate.py` 的 `resolve_properties`
- `tests/data/properties/{baseline_compliance,stiffer_compliance}.json`（夹具；前者逐位复现模板默认值，后者用于"换文件"对照）
- `tests/properties/test_properties.py`（18 项）

## 下一步

07（输出声明与衍生输出）与 09（study 合并）可启动；本步为 12 的判据 (c)「换一份属性文件重跑」提供了实现。
