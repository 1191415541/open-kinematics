# SPEC：05 试验台非侵入

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `05`。

## Task Shape

- **Shape**: `single-full`

## Goals

把试验台（rig）从"篡改被测总成内部构件"改成"只外加约束与外加载荷"，并让断言验证**被测总成不可变**。判据逐条来自父行 `05` 的 `acceptance_criteria` 与 `validation_command`，并按 `EPIC.md` 的「验证协议 / **05（试验台非侵入）**」与 **D3 裁决**展开：

1. **接入前后被测 runtime 逐项一致**：`bodies` / `points` / `constraints` / `elements` **逐项比较所有权、参数与几何值**后一致（承载体名、`wheel_center_local`、约束端点与类型、点坐标），只允许新增试验台自己的实体。**只比实体名集合不够**——`_reown_tires`（`subsystems/rig_link.py:315-348`，F7）正是"名字不变、所有权变"。
2. **`_reown_tires` 消除且契约反转（D3）**：`_reown_tires` 被删除（`grep` 无命中）；`merge_rig_link`（`:243-268`）不再用它替换原元素；`tests/subsystems/test_rig_link.py:172-199`（`test_the_tire_moves_to_the_bench_wheel`，断言 "the tire must be re-owned, not duplicated" 且 `all(body.startswith("wheel_carrier_"))`）的契约**反转为"不改写被测实体"**并有断言。依据 D3：断言内部实现细节是反模式；试验台是外部激励与夹持，只应通过外加约束或接触力与被测物交互；作用线与力路径不变时数学物理严格等价。
3. **试验台只经轮心夹具或接触点建约束**：carrier 仍由 `WeldJoint` 焊到"声明 `wheel_center` 的那个 body"（`subsystems/rig_link.py:211-218`），约束只增不改；不得转让被测零件的所有权。
4. **差异逐项登记**：现有 7 个组合在 `SUSPENSION_MULTIBODY_RIG_ENTITIES=1`（`subsystems/si_assembly.py:521`，默认开）下产物与 01 快照的差异**逐项登记**（允许变化，但必须点名：文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据）并写入 01 交付的 `raw/approved_deltas.json`。**登记口径是四项精确匹配**——`product`（供轮试验台的产物键形如 `axle_K@kc_quasi_static`、整车为 `vehicle`、rig 表项形如 `rig:<名字>`）、`pointer`、`before`、`after` 四项全等，**没有前缀覆盖规则**；登记项还必须带 `reason`、`registered_by`、`evidence`，且 `registered_by` 必须**以 05 开头**（只有 05 被允许移动既有产物）。缺字段或归属不符 → `snapshot.py --check` 退出 **4**；差异未命中登记 → 退出 **1**；登记全部命中 → **0**。**05 是唯一被允许改变既有产物的行**。**覆盖边界**：整车侧 5 个 rig（`vehicle_kc`、`vehicle_dynamic`、`handling`、`ride_four_post`、`ride_random_road`）的试验台绑定不在 01 快照覆盖范围内，必须由本行自己的「接入前后运行时逐项对照」证明非侵入。
5. **`RigSpec.supplies_wheels` 改义与既有 rig 声明的解释规则**：`supplies_wheels`（`rigs/rig.py:73-110`；`kc_quasi_static` 为 True）语义改写为"**提供轮体**"，不再等于"夺走被测轮胎"；既有 rig 声明在新语义下**如何解释**必须有明确规则并登记。

## Non-Goals

- 不动 `subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/wheel.py`（04 的写范围）。
- 不反转父 Epic 的 D2（单轴 K/C 凝结与 `kc_baseline` 逐位不变）；**严禁重录 `kc_baseline`**。
- 不改轮胎本构与求解器数值路径。
- 不重构试验台输出声明（属 20260922 Epic 的范围）。
- 不做可选对称（06 的范围）。

## Constraints

- **写范围（逐条照 `SUBTASKS.csv` `05` 的 `notes`）**：
  - `subsystems/rig_link.py`
  - `rigs/`
  - `subsystems/si_assembly.py`（仅 **bench 接入段**）
- **本行独占 `subsystems/rig_link.py` 与 `rigs/`**：任何其它子任务不得改这两个路径。
- `subsystems/si_assembly.py` 在 04 与 05 的写范围里都出现：本行只动 bench 接入段（`SUSPENSION_MULTIBODY_RIG_ENTITIES` 的应用点 `:467-482` 与开关定义 `:521`，F7），04 不动该段；两行串行执行、不得并发写同一文件。
- 本行依赖 `04`；`06` 的 `depends_on` 为 `05`。
- **先做试验台夹具与轮胎两条力路径的对照**，再决定 `supplies_wheels` 的新语义（父 Epic 风险条目）。
- 除 `link_wheel_supplying_rig` 外的实体改写开关不在本行范围：`SUSPENSION_MULTIBODY_CONDENSE_WELDS` 与 `SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES`（`vehicle_parts.py`）、以及文档层的 `cases/vehicle_kc.py:132-137`（F7 登记），本行不动。
- 本行**允许改变产物**（试验台不再改写被测实体），但必须：(i) 登记变化（文件 + 步骤 + 前后值）并写入 01 交付的 `raw/approved_deltas.json`——每条登记与差异**四项精确匹配**（`product`/`pointer`/`before`/`after`，没有前缀覆盖规则）且带 `reason`/`registered_by`/`evidence`、`registered_by` 必须以 **05** 开头，**未登记的差异即失败（退出 1）、登记缺字段或归属不符退出 4**；(ii) 先给出**独立于结果字节的物理等价判据**（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径）并列出允许差异；(iii) 数值门与快速集重跑通过。
- **产物是否变化的判据**：用 `tasks/20260929-01-freeze/snapshot.py --check`（**01 已交付**：`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘），口径为「未变化部分逐项相等 + 已登记差异」——登记处是 `tasks/20260929-01-freeze/raw/approved_deltas.json`，**未登记的差异必须让 `--check` 非零退出**；登记口径是**四项精确匹配**（`product`/`pointer`/`before`/`after`，无前缀覆盖规则）且登记项须带 `reason`/`registered_by`/`evidence`、`registered_by` 必须以 **05** 开头（缺字段或归属不符退出 **4**，差异未命中登记退出 **1**）；**05 是唯一被允许改变既有产物的行**。**覆盖边界**：整车侧 5 个 rig 的试验台绑定不在 01 快照覆盖范围内（`_meta.coverage_boundary` 与各 rig 的 `bench_bound`），这 5 个 rig 必须由本行自己的「接入前后运行时逐项对照」证明非侵入，不能只看快照。
- 「不得把"简化/试验台专用"的分支写进 role 接口」（沿用 20260922 Epic 的 G2b 口径）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。
- 每步落地后必须重跑 `just check-fast` 与数值门三项；结构改动后必须重跑 `tests/architecture`；不得新增 skip/xfail。
- 只改本行写范围；`raw/` 只存**已执行**的证据，不存虚构结果。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160/47 skip；kernel+contracts 60

## Risk Assessment

- [ ] **只比名字集合会漏掉真实篡改** → 判据必须是"逐项比较所有权、参数与几何值"（承载体名、`wheel_center_local`、约束端点与类型、点坐标）。
- [ ] **C 垫板机的轮胎作用点与力路径会变** → 缓解：先做"试验台夹具 vs 轮胎"两条力路径的对照，再决定 `supplies_wheels` 的新语义；变化逐项登记并写入 `raw/approved_deltas.json`（父 Epic 风险条目）。每条登记必须与差异**四项精确匹配**（`product`/`pointer`/`before`/`after`，无前缀覆盖）且带 `reason`/`registered_by`/`evidence`、`registered_by` 写 05——否则 `--check` 退出 4。
- [ ] **`supplies_wheels=True` 的既有 rig 声明语义不清** → 改写后必须给出"既有 rig 声明如何解释"的明确规则并登记（供轮 = 提供轮体）。
- [ ] **反转既有测试契约会让 `test_rig_link.py` 大面积失败** → 反转范围必须点名（`tests/subsystems/test_rig_link.py:172-199`），其余用例的失败要逐条判定是"契约变化"还是"真实回归"，不得用 `-k`/`--deselect` 豁免。
- [ ] **本行改变了产物**（试验台不再改写被测实体）→ 变化必须逐项登记（文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据）并写入 `raw/approved_deltas.json`，每条与差异**四项精确匹配**（`product`/`pointer`/`before`/`after`，无前缀覆盖规则）且 `registered_by` 必须以 **05** 开头；未登记的差异让 `snapshot.py --check` 退出 1、登记缺字段或归属不符退出 **4**；**整车侧 5 个 rig 不在 01 快照覆盖内，必须由本行自己的运行时逐项对照证明非侵入**；禁止先改基线让门变绿。
- [ ] **`si_assembly.py` 与 04 冲突** → 只动 bench 接入段，串行执行。

## Deliverables

- 写范围内各路径的改造（`subsystems/rig_link.py`、`rigs/`、`subsystems/si_assembly.py` 的 bench 接入段）。
- 覆盖 Goals 1–5 的断言（含"接入前后被测 runtime 逐项一致（所有权/参数/几何值）""`_reown_tires` 无命中""反转后的 `test_rig_link` 契约""只经轮心夹具或接触点建约束""`supplies_wheels` 新语义"）。
- `raw/rig_immutability.md` — 接入前后被测 runtime 的逐项比较证据（比较维度：承载体名、`wheel_center_local`、约束端点与类型、点坐标；本行开工后生成）。
- `raw/rig_product_diff.md` — 现有 7 组合在 `SUSPENSION_MULTIBODY_RIG_ENTITIES=1` 下与 01 快照的差异逐项登记（文件 + 步骤 + 前后值 + 物理等价判据；本行开工后生成），同时把每个登记项写入 01 交付的 `raw/approved_deltas.json`（本行是唯一被允许改变既有产物的行）。登记项必须与差异**四项精确匹配**（`product`/`pointer`/`before`/`after`，`product` 取 `axle_K@kc_quasi_static` 这类产物键或 `vehicle` 或 `rig:<名字>`，**没有前缀覆盖规则**）并带 `reason`/`registered_by`/`evidence`，`registered_by` 以 **05** 开头；缺字段或归属不符即 `--check` 退出 **4**。**整车侧 5 个 rig 的试验台绑定不在 01 快照覆盖范围内**，本文件必须另记本行自己的「接入前后运行时逐项对照」结论。
- `raw/force_path_crosswalk.md` — "试验台夹具 vs 轮胎"两条力路径的对照与 `supplies_wheels` 新语义及其对既有 rig 声明的解释规则（本行开工后生成）。

## Done-When

- [ ] 接入试验台前后被测总成 runtime 的 `bodies` / `points` / `constraints` / `elements` **逐项比较所有权、参数与几何值**后一致；只允许新增试验台自己的实体。
- [ ] `grep -n "_reown_tires"` 在 `subsystems/rig_link.py` 无命中，`merge_rig_link` 不再用它替换原元素。
- [ ] `tests/subsystems/test_rig_link.py` 的契约反转为"不改写被测实体"且有断言（含 `test_the_tire_moves_to_the_bench_wheel` 的反转）。
- [ ] 试验台只通过轮心夹具 / 接触点建立约束（只增不改）。
- [ ] 现有 7 组合在 `SUSPENSION_MULTIBODY_RIG_ENTITIES=1` 下产物与 01 快照的差异逐项登记（文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据），并写入 `raw/approved_deltas.json`；每条登记与差异**四项精确匹配**（`product`/`pointer`/`before`/`after`）且 `registered_by` 以 **05** 开头；未登记的差异即失败（退出 1），登记缺字段或归属不符退出 **4**；**整车侧 5 个 rig 由本行自己的运行时逐项对照证明非侵入**（不在 01 快照覆盖内）。
- [ ] `RigSpec.supplies_wheels` 的新语义（供轮 = 提供轮体）与既有 rig 声明的解释规则已登记。
- [ ] `just check-fast` 与数值门三项全绿；`kc_baseline` 逐位未变；无新增 skip/xfail。
- [ ] `tasks/20260929-01-freeze/snapshot.py --check`（**01 已交付**，直接跑）通过「未变化部分逐项相等 + 已登记差异」口径；本行造成的每个变化都已按**四项精确匹配**登记并写入 `raw/approved_deltas.json`（`registered_by` 以 05 开头），未登记的差异会让 `--check` 退出 1、登记不合法退出 4。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_rig_link.py packages/suspension_multibody/tests/subsystems/test_bench_in_composition.py packages/suspension_multibody/tests/rigs -q && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```

其中最后一行（`snapshot.py --check`，作为"产物是否变化"的判据）**01 已交付、直接跑**，口径为「未变化部分逐项相等 + 已登记差异」（登记处 `raw/approved_deltas.json`；登记须与差异**四项精确匹配**、`registered_by` 以 05 开头；未登记的差异退出 1、登记缺字段或归属不符退出 4）。

## Demo Flow

1. 装配被测总成 → 记录 `bodies` / `points` / `constraints` / `elements` 的逐项快照（所有权、参数、几何值）。
2. 接入试验台 → 再记录一次 → 逐项比较：只允许新增试验台自己的实体；`wheel_center_local`、约束端点与类型、点坐标不变。
3. `test_rig_link.py` 反转后的契约跑通；`grep "_reown_tires"` 无命中。
4. `supplies_wheels` 新旧语义对照：打印既有 rig 声明在两套解释下的行为 → 登记新规则。
5. 落地前后各跑一次 `snapshot.py --check`（01 已交付，直接跑）→ 差异逐项登记并写入 `raw/approved_deltas.json`（每条与差异**四项精确匹配**、`registered_by` 写 05），并给出物理等价判据；未登记的差异让 `--check` 退出 1、登记不合法退出 4。整车侧 5 个 rig 不在快照覆盖内，另行用本行的接入前后运行时对照记录结论。
