# SPEC：02 显式接口配对落地

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `02`。

## Task Shape

- **Shape**: `single-full`

## 本轮口径（2026-09-29，主代理冻结的机制契约）

- **机制落点**：新建 `connections/links.py`（`LinkSpec` + `build_links`），把 `match_requirements` 的 bindings 变成**实体**（`Connection` 行 + 与之对应的 `Constraint`）；`subsystems/composition.py::compose_simulation_assembly` 在匹配之后调用它，把生成的行并入合并后的 fragment。`match_requirements` 仍是唯一的匹配通道，本行不得另立第二条推断路径。
- **无配方即零变化**：贡献里没有 `LinkSpec` 时 `build_links` 不产出任何行 ⇒ 既有产物逐字节不变。这是「01 快照 `--check` 零差异」的实现保证，不是事后声明。
- **配方放在 contribution 一侧**：`LinkSpec(role, kind, body_a, point_a_local, ...)`；`PortRequirement` 保持纯匹配语义（角色/能力/标签/数量），不掺入几何。`kind` 取 `weld` / `revolute` / `bushing`，一律用仓库既有的 `Constraint` 类型构造，**不新造运动副类型**（新副类型属内核范围）。
- **文档配对段**：`assembly.schema.json` 新增可选 `pairings`（每项 `{"requirement_role": ..., "port": ...}`）；`AssemblyDocument` 读进 `pairings`，经 `AssemblyRequest` 新增的可选字段（`subsystems/types.py`，默认 `{}`，**不进任何 `model_dump` 哈希**）送到 composition 的 `explicit_bindings`。留空时按角色/能力唯一匹配，既有总成文件不受影响。
- **判据在机制层兑现**：(a)「显式配对的产物与推断配对产物逐项一致」用**合成的两个 contribution**（一个提供端口、一个声明需求 + 配方）在 `tests/connections/` 内直接对比；(b)「车身侧刚体改名后仍能装配」在 02 内以「配对端命名为 `subframe`（不在任何硬编码集合里）仍配对成功」的机制证据兑现。整车路径的端到端由 **03** 在消费本契约时复验，07 的终局 `grep` 收口。
- **本行自有路径的「名字猜身份」清点**：`connections/`、`modeling/ports.py`、`templates/ports.py`、`subsystems/composition.py` 今天**没有**字符串改写规则；`authoring/documents.py` 的 `_ASSEMBLY_SUPPLIED_BODIES`（`:76`）是**白名单校验**（供子系统文档引用"由总成提供的刚体"），不是改写规则——本行要给出它的处置结论（保留并说明它校验什么，或改为由角色/端口声明提供）并登记，不得默认放过。
## Goals

把跨子系统连接从"按名字猜身份"改为**显式接口配对**（父 Epic 的 G2 / R2 / P2 / D4）。逐条承接 `SUBTASKS.csv` 第 3 行的 `acceptance_criteria`：

1. **`match_requirements` 成为跨子系统插接的唯一通道**：子系统声明提供的 port 与需要的 requirement，装配层只经 `connections/matcher.py:102 match_requirements`（explicit > role / capabilities / labels）完成悬架↔车身等跨子系统插接；`subsystems/composition.py:258`（子系统贡献之间）与 `:347`（`_bind_rig`）之外的推断路径不再自行配对接。
2. **bindings 真正转成两点间的运动副/衬套**：今天 `composition.py:302-305` 只产出 bindings 记录、不据此建立物理插接；落地后配对结果必须落到实体（约束类型 + 两端体 + 两端点 + 点坐标），而不是只留一条记录。
3. **配对段可选并写进契约 schema**：配对段写进 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`（**仅配对段字段**）；配对段**可选**——留空时按角色/能力唯一匹配（即今天的推断行为），以保住既有总成文件；`packages/suspension_contracts/tests` 同步更新并通过。
4. **车身侧刚体改名后同一份总成文件仍能装配**：把车身侧刚体改名为 `subframe`（不在任何硬编码集合里）后，**同一份配对文件**仍装配成功（这是"配对是真源"的判据）。
5. **配对歧义与悬空配对报错点名**：候选不唯一时抛 `AmbiguousBindingError`（`matcher.py:102` 既有行为）；显式配对指向不存在的 port 时报错并点出该 port 与需求名。
6. **`"chassis"` / `"ground"` 字符串改写规则在装配路径全部消除**：本行自有路径（五个路径列全：`connections/`、`subsystems/composition.py`、`authoring/documents.py`、`modeling/ports.py`、`templates/ports.py`）的判据是 `grep` 无命中；三处跨行锚点（`subsystems/vehicle_assembly.py:212-228`、`subsystems/vehicle_parts.py:172-173` 与 `:414`、`preparation/vehicle_dynamic.py:373`）**全部归 03**，全装配路径的终局 `grep` 判据由 03 收口、07 复验；过渡期兼容开关必须删除并登记。
7. **产物变化判据**：用 `tasks/20260929-01-freeze/` 交付的 `snapshot.py --check`（01 的可重跑快照脚本），口径为「**未变化部分逐项相等 + 已登记差异**」，登记处是 `tasks/20260929-01-freeze/raw/approved_deltas.json`——**未登记的差异必须让 `--check` 非零退出**。本行落地前后各跑一次；**02 不得产生差异，差异非空即失败**（只有 05 被允许改变既有产物并登记）。

## Non-Goals

- **不做 03 的通用装配引擎**：本次不改 `subsystems/vehicle_assembly.py` 的入口形态、不放开 N 轴与 `placement_role` 枚举（属 03）。
- **不动试验台**：`subsystems/rig_link.py`、`rigs/` 一行不改（05 独占），`RigSpec.supplies_wheels` 的语义不在本行。
- **不改设备轮模型与轮端生命周期**：单轴/整车"删轮胎再建车轮"的消除属 04。
- **不改镜像与 `_SIDES`**（属 06）。
- **不改模板数据模型**（`templates/model.py` 的 parts/connections/属性槽）：配对用的是模板已声明的 `ports`。
- **不改 C++ 内核**：ABI 七符号与版本常量（15/30/1/1）不变。
- **不重录任何基线**。

## Constraints

- **写范围**（照 `SUBTASKS.csv` 第 3 行 `notes`，逐条列明）：
  - `packages/suspension_multibody/src/suspension_multibody/connections/`（`matcher.py`、`policy.py`、`geometry.py`、`__init__.py`）
  - `packages/suspension_multibody/src/suspension_multibody/modeling/ports.py`
  - `packages/suspension_multibody/src/suspension_multibody/templates/ports.py`
  - `packages/suspension_multibody/src/suspension_multibody/authoring/documents.py`
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/composition.py`
  - `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`（**仅配对段字段**）
  - 对应测试：`packages/suspension_multibody/tests/connections/`、`packages/suspension_multibody/tests/subsystems/`（本行涉及的部分）、`packages/suspension_multibody/tests/authoring/`（本行涉及的部分）、`packages/suspension_contracts/tests/`
- **不得动** `subsystems/rig_link.py` 与 `rigs/`（05 独占，见 `EPIC.md` 行 137）。
- **不得动** `subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`、`authoring/vehicle.py`、`authoring/solver.py`、`schema/vehicle.py`（03 的写范围，见 `SUBTASKS.csv` 第 4 行 `notes` 与 `EPIC.md` 行 136）。
- **与 03 共享契约 schema，不得并行**：`assembly.schema.json` 由 02 改配对段字段、03 改放置与轮数段（`EPIC.md` 行 135）。本行落地后 03 才开始改同一文件。
- **⚠️ 跨行归属（已由 Epic 复核确认）**：本行验收要求"`chassis`/`ground` 字符串规则在装配路径全部消除"（`SUBTASKS.csv` 第 3 行）与 `EPIC.md` 行 154(d) 点名的三处——`subsystems/vehicle_assembly.py:212-228`、`subsystems/vehicle_parts.py:172-173` 与 `:414`、`preparation/vehicle_dynamic.py:373`（锚点见 `EPIC.md` F4）——**全部归 03、都不在本行写范围内**。因此：
  - 本行职责 = 把机制与配对真源做出来（`connections/` + `composition.py`），并让本行自有路径（五个路径列全）`grep` 无命中；
  - 上述三处的删除随 03 落地，**"全装配路径 `grep` 无命中"的终局判定由 03 收口、07 复验**；
  - 本行不因此改 03 的文件；该归属若需调整，由 Epic 归属决定后写入对应行的 SPEC。
- **配对段可选**：留空时按角色/能力唯一匹配，既有总成文件不得失效；只有"候选不唯一"才报错（`EPIC.md` 行 200 的缓解口径）。
- **`EPIC.md` F4 的待决项**：`modeling/ports.py` / `templates/ports.py` 的 `ChannelPort` / `declaration_to_port` / `Assembly.all_ports()` **无外部消费者**（预备性代码）——本行必须给出**启用或删除**的结论并在 PROGRESS 登记理由。
- **不得新增 skip/xfail**；`tests/adams` 的 47 个环境 skip 是既有的，不得增长。
- **不得重录任何基线**；`model_dump(mode="json")` 的产物形状不得改变（D1）。
- **装配层不得出现按名字猜身份的规则**：`"chassis"` / `"ground"` / 前缀拼接这类字符串改写是本行的消除对象（`EPIC.md` 行 147）。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160 passed / 47 skipped；kernel+contracts 60

## Risk Assessment

- [ ] **把配对写成硬性要求会让所有既有总成文件失效**（`EPIC.md` 行 200 已登记）→ 配对段可选，留空时退回"按角色/能力唯一匹配"的推断行为；显式配对只是覆盖手段。
- [ ] **`assembly.schema.json` 的条目当前 `additionalProperties: false`**（`EPIC.md` F2 引用 `:17-18`）→ 新增配对段必须同时放开对应层级，否则合法文件在 `AssemblyDocument.load` 阶段就被拒（拒绝点早于装配逻辑，见 F2 与 `tasks/20260929-01-freeze/raw/triaxle_refusal.md` 的实测）。
- [ ] **产物漂移**：把 bindings 转成运动副/衬套会新增约束 → 以 01 的 `snapshot.py --check` 作判据（口径「未变化部分逐项相等 + 已登记差异」，登记处 `raw/approved_deltas.json`，未登记的差异即非零退出）；**02 不得产生差异，差异非空即失败**——默认路径（无显式配对）必须与 01 快照逐项一致。
- [ ] **2 处既有调用点被破坏**：`composition.py:258`、`:347` → 改 `match_requirements` 的返回值形状时同步改这两个调用点，并跑 `tests/subsystems` 全套。
- [ ] **`"chassis"` 规则跨 3 个文件但不是本行写范围**（见 Constraints 的跨行归属）→ 本行只收口自有路径（五个路径列全）；未收口项在 PROGRESS 的 Known issues 里点名，**三处锚点全部交 03（终局判定由 03 收口、07 复验）**，不允许静默放过。
- [ ] **`preparation/vehicle_dynamic.py:373` 与 `vehicle_parts.py:172-173/:414` 的删除会改变命名语义** → 均由 03 按其行内判据处理（终局判定由 03 收口、07 复验），本行仅在文档中给出替代物（端口配对）的定义。

## Deliverables

- **生产代码改动**（逐文件说明改了什么，段级别）：
  - `connections/matcher.py`：配对入口的契约（若需扩展 explicit 映射的输入形状）
  - `subsystems/composition.py`：把 bindings 转成两点间运动副/衬套；`:302-305` 的记录不再止步于记录
  - `authoring/documents.py`：配对段的文档读取（与 `EPIC.md` F4 的 `_ASSEMBLY_SUPPLIED_BODIES`（`:76`）相关的白名单按需收口）
  - `modeling/ports.py` / `templates/ports.py`：启用或删除的结论（F4 待决项）
  - `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`：配对段字段
- **测试**：`tests/connections/`、`tests/subsystems/`、`tests/authoring/`、`packages/suspension_contracts/tests/` 内的新增/更新用例（用例名实施时确定）
- **`raw/` 证据**（每一步落地后追加，不预填）：
  - `raw/snapshot_diff.md` — 各步前后 `snapshot.py --check` 的结果（口径「未变化部分逐项相等 + 已登记差异」，登记处 `raw/approved_deltas.json`）；**02 不得产生差异，差异非空即失败**
  - `raw/binding_evidence.md` — 显式配对装配的约束清单（类型 + 两端体 + 两端点 + 点坐标），与既有 `body_map` 结果的逐项对照
  - `raw/removal_register.md` — `"chassis"`/`"ground"` 规则与过渡开关的删除登记（文件 + 行 + 替代物）
- **PROGRESS 登记**：F4 待决项结论、跨行归属的未收口项

## Done-When

- [ ] 悬架↔车身跨子系统插接只经 `match_requirements`，且配对结果落成运动副/衬套（有约束类型与端点，不只记录）。
- [ ] 一份**显式配对**的悬架↔车身总成文件装配产物，与今天的 `body_map` 结果**逐项一致**（体/点/约束集合与名字，含约束类型与端点）。
- [ ] 车身侧刚体改名 `subframe` 后，**同一份配对文件**仍能装配。
- [ ] 配对缺失且候选不唯一 → `AmbiguousBindingError`；配对指向不存在的 port → 报错点名（port 名 + 需求名）。
- [ ] 本行自有路径（五个路径列全：`connections/`、`subsystems/composition.py`、`authoring/documents.py`、`modeling/ports.py`、`templates/ports.py`）`grep` 无 `"chassis"`/`"ground"` 字符串改写规则，过渡开关已删除并登记（三处锚点均归 03，终局判定由 03 收口、07 复验，见 Constraints）。
- [ ] 配对段写进 `assembly.schema.json` 且 `packages/suspension_contracts/tests` 通过；既有总成文件（不含配对段）仍全部可加载。
- [ ] `tasks/20260929-01-freeze/snapshot.py --check` 通过（口径「未变化部分逐项相等 + 已登记差异」，登记处 `raw/approved_deltas.json`；**02 不得产生差异，差异非空即失败**）（**01 已交付**：`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘，落地前后直接跑；差异只能由 05 登记，本行差异非空即失败）。
- [ ] 无新增 skip/xfail；未重录任何基线。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests/connections packages/suspension_multibody/tests/subsystems packages/suspension_contracts/tests -q && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```

（第 2 条为 01 交付的产物不变判据，口径「未变化部分逐项相等 + 已登记差异」，登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`；**02 不得产生差异，差异非空即失败**——登记只能由 05 写，02 出现差异即产物被改动。**01 已交付**：`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘，落地前后直接跑。）

## Demo Flow

1. 用显式配对写一份悬架↔车身总成文件 → 装配 → 导出 bodies / points / constraints / connections 清单。
2. 同一份文件把车身侧刚体名改成 `subframe` → 再装配 → 产物与第 1 步逐项一致（差异只应是名字本身）。
3. 去掉配对段 → 仍装配（按角色/能力唯一匹配），产物与 01 快照一致。
4. 写一条指向不存在 port 的配对、再写一条候选不唯一的配对 → 各报错一次，报错点出 port / 需求名。
5. 前后各跑一次 `snapshot.py --check`（口径「未变化部分逐项相等 + 已登记差异」，登记处 `raw/approved_deltas.json`），输出即"产物是否变化"的证据；**02 不得产生差异，差异非空即失败**。
