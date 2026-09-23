# 子任务 06：弹性元件属性文件加载

## 目标

让「衬套、弹簧、减振器等弹性元件的属性由外部属性文件控制」成为事实，模板本身只声明引用。

用户原话：「衬套、弹簧、减振器等弹性元件由外部属性文件控制属性，也就是说可以加载不同属性文件达到不同刚度、阻尼等等特性」。

1. 新建 `suspension_multibody/properties/` 包，**在本 SPEC 中先冻结属性文件格式**，并实现加载器 `load_properties(path)`。

   **属性文件格式（冻结）**——JSON，根对象：

   ```json
   {
     "schema_version": 1,
     "name": "baseline_compliance",
     "units": "mm-N-N*mm-kg-deg",
     "properties": {
       "uca_inner_front_bushing": {
         "kind": "bushing6x6",
         "stiffness": [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], ["...6x6..."]],
         "damping": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
         "preload": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       },
       "front_spring": {
         "kind": "spring",
         "stiffness": 45.0,
         "free_length": 250.0,
         "reference_length": 240.0,
         "preload": 0.0
       },
       "front_damper": {
         "kind": "damper",
         "viscous_damping": 1.5,
         "gas_stiffness": 0.0,
         "friction": 0.0
       },
       "front_tire": {"kind": "tire", "stiffness": 250.0, "unloaded_radius": 320.0},
       "front_bump_stop": {
         "kind": "bump_stop",
         "clearance": 20.0,
         "stiffness": 120.0,
         "direction": "bump"
       }
     }
   }
   ```

   规则（可判定）：
   - **R1 文档级**：根必须是对象；根 `schema_version` 必须为 `1`；`properties` 必须是对象；未知键一律拒绝（沿用 `schema/common.py:12-19` 的 `StrictModel`：`extra="forbid"`、`frozen=True`）。
   - **R2 条目级**：每个条目必须有 `kind`，取值 ∈ `{spring, damper, bushing6x6, tire, bump_stop}`；其余字段名与 `schema/elements.py` 中对应类的字段名**逐一对应**——`LinearSpring` `:13-25`、`StaticDamper` `:62-76`、`Bushing6x6` `:111-146`、`VerticalTire` `:215-221`、`BumpStop` `:237-248`。因此一个属性条目可直接作为模板中同名直接数值的替代，字段语义不再另立一套。
   - **R3 取值级**：沿用 `schema/elements.py` 既有约束（`stiffness > 0`、`damping >= 0`、`unloaded_radius > 0`、`clearance >= 0`、矩阵形状 6×6 / 6）；若 03 的 `PropertySlot` 另带 bounds，则以更严者为准（03 未提供 bounds 时，本步以 `schema/elements.py` 为唯一来源并在 PROGRESS 登记该事实）。
   - **R4 报错**：校验失败抛 `ValueError`，消息含**文件路径 + 槽名 + 字段名 + 原因**（三者缺一不可），不得静默回退到默认值。

2. 模板只声明**引用**：模板中的弹性元件槽位可以是 03 定义的 `PropertySlot`（结构在 03 冻结）；`templates/` 侧新增解析入口 `resolve_properties(template, property_set)`，把属性集解析成实例化可直接消费的属性对象。
3. 同一模板 + 不同属性文件 → 不同刚度/阻尼特性，**模板与几何逐项不变**（有对照断言）。
4. 模板也可直接给数值（保持现役行为）：属性引用是**新增可选项**，不取代直接数值；现役 `schema/model.py:166-171` 的 `springs` / `dampers` / `bushings` / `tires` 数值入口保持可用。

## 非目标

- 不改 K/C 语义（05 已完成）。
- 不实现输出求值（07）。
- 不删除现役 `schema/model.py` 的 `springs` / `bushings` / `tires` 数值入口。
- 不改 `preparation/**`、`subsystems/**`、`outputs/**`。
- 不引入新依赖（不新增第三方格式或校验库；`json` + 既有 `pydantic` 足够）。
- 不做属性文件的写入/编辑工具、不做 GUI、不做属性扫描目录的自动发现。

## 约束

- **属性缺失/类型错/越界值必须报错而非静默用默认**。边界要说清：仅当槽 `required=True` 且模板**未给默认值**时，文件里缺失才报错；模板已给默认值的槽，文件中省略是**合法回退**——这正是"模板也可直接给数值"。
- **注入路径只能走 05 留下的 `instantiate(model, mode, properties=...)`**（05 SPEC 目标 1）。若 05 落地时该参数不存在或形状不同，本步**不得自行改 `preparation/`**，须停止并上报，由主代理裁决是否扩展 05 的接口。
- **不得改 `preparation/`、`subsystems/`、`outputs/`。**
- **不得新增 legacy import**：`tests/architecture/legacy_surface_gate.py` 的 MODE_MIGRATION 规则下，新包不得导入 `core` / `elements` / `model` / `analysis` / `metrics`；`legacy_surface_gate.py --check` 必须在**不新增注册条目**的前提下保持绿。
- **默认属性文件必须逐位复现 05 落地后的模板默认属性**，以保证 `kc_baseline` 不变；换另一份属性文件产生不同数值是**新算例**，不得借机重录基线。
- **与仓库既有 JSON 约定的关系**（明确写出，避免两套约定混用）：
  - `tests/data/*.json` 是**测试夹具**：`tests/data/benchmark_axle.json` 根键为 `name` / `description` / `model` / `grid`，**没有根 `schema_version`**，由 `tests/benchmark_fixture.py:19-31` 直接 `json.loads` 再 `FrontAxleModel.model_validate(payload["model"])`。属性文件**不沿用**这种松散夹具约定。
  - 属性文件沿用 `schema/loader.py:20-47` 的**版本化输入文档**约定：根对象 + `schema_version == 1` + 严格模型校验 + 错误信息带文件路径。
  - 属性文件夹具放 `packages/suspension_multibody/tests/data/properties/**`；加载器不假设固定目录（用户可传任意路径）。
- 不引入新依赖。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/properties/**`
  - `packages/suspension_multibody/src/suspension_multibody/templates/**`（属性引用解析；03 建包、05 接线的同一个包）
  - 新增测试 `packages/suspension_multibody/tests/properties/**`
  - 新增属性文件夹具 `packages/suspension_multibody/tests/data/properties/**`
  - `packages/suspension_multibody/src/suspension_multibody/__init__.py`（仅当需要导出新符号，且不得改变现有导出）
- 只读：`schema/elements.py`、`schema/model.py`、`schema/loader.py`（字段名与取值约束的来源）、`subsystems/**`、05 的实例化入口。
- 不写：`preparation/**`、`subsystems/**`、`outputs/**`、`report/**`、`tests/data/kc_baseline/**`（本步预期不需要重录；若确需重录必须先登记）、父级计划文件（归主代理）。

## 依赖

- 前置：05（实例化入口与 K/C 列激活；`instantiate(..., properties=...)` 参数与模板默认属性）。
- 后续：09（准静态/动态 study 合并——两个 study 共用同一属性来源）、12（终局验收中的"换一份属性文件重跑"一项）。

## 验收标准

1. 属性文件可加载与校验，格式有 schema 级约束：R1–R3 每条都有对应测试；未知根键与未知条目键均被拒绝。
2. **同模板 + 同属性文件 → 结果逐位一致（可复现）**：两次加载 + 两次实例化的 `bodies` / `points` / 约束集合 / 刚度与阻尼矩阵用 `np.array_equal` 与逐顺序比较全等。
3. **换属性文件 → 刚度/阻尼按属性变化，模板与几何逐项不变**：有对照断言——`points` 键与值、`bodies`、`connections`、`hardpoints` 逐项相等；被替换元件的刚度/阻尼等于新文件的值且不等于旧值。
4. **三个负例失败并点名**：
   - 属性缺失：`required` 且无模板默认值的槽缺失 → `ValueError`，消息含槽名与文件路径；
   - 类型错误：如 `stiffness` 给字符串、`stiffness` 给 3×3 而非 6×6 → `ValueError`，消息含槽名与字段名；
   - 值越界：如 `stiffness <= 0`、`damping < 0`、`unloaded_radius <= 0` → `ValueError`，消息含槽名、字段名与边界。
5. 模板直接给数值的现役路径仍可用：不经属性文件的实例化结果与 05 落地结果一致。
6. 门禁保持且**未重录基线**：默认属性文件下 `kc_parity_check.py --check`、`case_parity_check.py`、`dynamic_hash_sentinel.py --check`、`check_module_layering.py --strict --final`、`legacy_surface_gate.py --check`、三套 pytest、ruff、ty、`git diff --check` 全绿，且 `tests/data/kc_baseline/**` 无 diff。若本步确实导致 C 模式数值变化，必须在 `PROGRESS.md` 按 05 的格式登记基线重录台账（文件 + 步骤 + 重录前后值 + 判定依据）。

## 验证协议

1. 加载器落地：跑 `tests/properties` 的正例 + 三个负例（负例必须失败并点名，不接受"报错了就行"）。
2. 解析接线后：跑"同模板 + 同文件可复现"与"换文件只改刚度/阻尼、几何逐项不变"两个对照测试。
3. 默认属性文件落地后：跑 `kc_parity_check.py --check` + `case_parity_check.py`——这是"未重录基线"的证据。
4. 收尾：三套 pytest、`ruff check .`、`ty check .`、`check_module_layering.py --strict --final`、`legacy_surface_gate.py --check`、`git diff --check`，并确认 `tests/data/kc_baseline/**` 无 diff。

**若默认属性文件导致 C 模式基线变化，立即停止并上报**——那说明属性路径改变了 05 的既有数值，而不是"预期重录"。
