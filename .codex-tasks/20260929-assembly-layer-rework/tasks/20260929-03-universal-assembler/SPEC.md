# SPEC：03 通用装配引擎（条目清单驱动）

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `03`。

## Task Shape

- **Shape**: `single-full`

## 本轮口径（2026-09-29，主代理冻结的契约与分期）

1. **入口不得改名**：`subsystems/vehicle_assembly.py::compose_vehicle_runtime` 是 `tests/architecture/legacy_surface_gate.py:87` 登记在册的既有表面，且被 `preparation/vehicle_dynamic.py:230`、`subsystems/entry.py:72`、`vehicle/static_loads.py:74`、`adams/full_vehicle_model.py:3167` 与多个测试调用。**保持名字与签名不变**；本行之后它是「`VehicleModel` → 引擎输入」的适配器。
2. **引擎**：新建 `subsystems/assembler.py`，含 `AxleEntry`（`placement`、`prefix`、`axle: FrontAxleModel`、`replace_bodies: Mapping[str, str]`、`wheels: tuple[WheelSpec, ...]`）与 `compose_entries_runtime(entries, *, chassis_name, mode, request)`：逐条调用 `si_assembly_for_axle`，按**条目声明的前缀**命名（不再出现 `"front_"`/`"rear_"` 字面量），`replace_bodies` 说明「该轴的哪些体由整车的哪个体顶替」。今天 `body_map` 的规则（`chassis`/`ground` → 车身名）由适配器**显式写出来**（`{"chassis": <车身名>, "ground": <车身名>}`），文档路径则由条目声明或 02 的配对给出。
3. **适配器**（必须落在 `preparation/` 之外，放 `authoring/vehicle.py` 或新增模块）：`VehicleModel → (AxleEntry 列表, chassis_name)`，两条轴两个条目（`placement` 为 `front`/`rear`，前缀 `front_`/`rear_`），`replace_bodies` 如第 2 条。**其产物必须与 01 快照逐项一致**（这是 D1 的硬门）。
4. **`preparation/` 只读图谱**：`preparation/vehicle_dynamic.py` 的 7 处 `model.front_axle`/`model.rear_axle`（`:188`、`:207`、`:210`、`:341`、`:368-369`）改为读「装配运行时 + 一个事实记录」（本次装配的轴清单与各自模式、是否带衬套、转向 rack 是否固定到车身），事实由适配器算出并随请求传入；`:373` 的 body 命名规则一并删除。硬门：`dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节不变。
5. **形状规则数据化**（`connections/policy.py`）：不再写死 `suspension: 2` 与 `front/rear` 必需放置；改为——同一 `(functional_role, placement_role)` 不得重复（重复即报错点名）；`full_vehicle` 至少一个 `suspension`；「轮端是否齐备」由总成声明的轮条目推导（3 轴即 6 个，单轮/三轮按声明）。正例（3 轴）与负例（缺放置、重复放置）各有断言。
6. **契约 schema 与文档段**：`placement_role` 加 `middle`、`vehicle.wheels` 不再限定恰好四个、整车 `required` 允许多个车身；`authoring/documents.py` 的放置与角色枚举段（`PLACEMENT_ROLES` 与 `:139`）同步放开。`_ASSEMBLY_SUPPLIED_BODIES`（`documents.py:76`）按 02 的登记由本条**收口**：改为由总成条目声明的角色/端口推导，或给出「保留」的明确理由并配测试。
7. **分期（按序交付，每期各自可验收；一期做完并报告后再进下一期）**：
   - **03a**：入口 + 引擎 + 适配器 + 01 快照逐项一致（**不动** `preparation/`、`policy.py`、schema）。
   - **03b**：policy 数据驱动 + schema/documents 放开 + 3 悬挂（front/middle/rear）装配并跑通一次 study。
   - **03c**：`preparation/` 图谱化 + `dynamic_hash_sentinel.py --check` 26 artifact 逐字节不变。
   - **03d**：拖挂铰接最小用例（两个车身侧体，现有副类型的铰接副）装配并跑通一次 study。
   每一步都跑 `just check-fast`；03c/03d 之后跑数值门三项。
## Goals

把装配执行层的驱动源从"硬编码两轴 `VehicleModel`"换成"总成文件声明的子系统条目清单"（父 Epic 的 G1 / R1 / P1 / D1）。逐条承接 `SUBTASKS.csv` 第 4 行的 `acceptance_criteria`：

1. **3 悬挂（front / middle / rear 放置）整车总成装配成功并跑通一次 study**：这是 `EPIC.md` Done-When (a) 的最小实体；装配产物与文件声明的子系统条目**一一对应**。
2. **拖挂铰接总成（牵引车 + 挂车两个车身侧体）装配并跑通一次**：同一总成内两个车身侧体经**现有副类型**的铰接副连接并跑通一次 study；这要求 `connections/policy.py` 的角色数规则与 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json:46`（整车 `required` 今天只列一个 `chassis`）支持多车身。只有实测证明现有副类型不可行时，才登记为内核范围并提请用户裁决（`EPIC.md` 行 157(f)）。**单轮（单侧悬架）与三轮（两悬架 + 一个单侧）的最小用例归 06**——单侧展开机制在 06 的写范围（`subsystems/si_assembly.py:56`、`subsystems/wheel.py:114-116 sides()`、`subsystems/element_build.py:206/213` 今天固定展开 L/R），07 复验。
3. **装配路径 `grep "front_axle\|rear_axle"` 无命中**：`EPIC.md` G1 判据 (a) 的字面口径 = 装配层三文件（`subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`）**加整个 `preparation/` 目录**。今天是 `subsystems/vehicle_assembly.py:184-187` 把两轴硬编码成 `("front", model.front_axle, "front_")` / `("rear", model.rear_axle, "rear_")`（锚点见 `EPIC.md` F1）；`preparation/vehicle_dynamic.py` 有 7 处模型访问（`:188` bushings、`:207` 与 `:210` rack_fixed_to_chassis、`:341` 与 `:368-369` 两轴遍历）。**适配器（`VehicleModel` → 装配图谱）必须落在 `preparation/` 之外**（`authoring/vehicle.py` 或本行新增模块），`preparation/` 内只读图谱。
4. **`VehicleModel` 适配器路径产物与 01 快照逐项一致**（**零回归硬门**，D1）：**严禁原地改 `VehicleModel` schema**——`schema/vehicle.py:244` 的 `front_axle` / `rear_axle` 字段（`:252-253`）与 `:262-265` 的四角校验保持不变；本行的做法是**新增文档驱动装配器**（直接消费 `AssemblyDocument`，原生支持任意轴数与子系统拓扑）+ **把 `VehicleModel` 降级为向下兼容适配器**（绞杀者模式）。硬门判据 = 适配器路径产物与 01 的 `snapshot.py --check` 逐项一致，尤其 `model_dump(mode="json")` 的形状不得漂移（D1 的连锁风险：`api.py:116`、`:284` 用它算 `model_hash`）。
5. **`check_assembly_shape` 的 N 轴规则改为数据驱动，正例与负例各有断言**：今天 `connections/policy.py:202-217` 的 `full_vehicle` 规则写死 `role_counts={"suspension": 2, ...}` 与 `required_placements={("suspension","front"),("suspension","rear")}`，由 `:252-296 check_assembly_shape` 逐条执行（锚点见 `EPIC.md` F2）。正例 = 3 轴；负例 = 缺放置、重复放置，各自成功 / 失败。负例基线原文见 `tasks/20260929-01-freeze/raw/triaxle_refusal.md`（实测：`requires exactly two suspension subsystem(s), found 3`、`requires one front suspension`）。
6. **`assembly.schema.json` 接受 `middle` 放置、非四轮声明与整车 `required` 的多车身**：契约 schema 本身是一道**前置墙**，在装配逻辑之前生效——`placement_role` 枚举不含 `middle`、整车轮数被限定为四、整车 `required`（`:46`）今天只列一个 `chassis`（拖挂的两车身总成在此被挡）（锚点见 `EPIC.md` F2）；**且只改 schema 不够**：`authoring/documents.py:139` 用 `PLACEMENT_ROLES`（`:62-64`）校验 `placement_role`，3 轴文件的 `middle` 会在**文档读取层先被拒**，故同一行必须同步放开 `authoring/documents.py` 的放置与角色枚举段；`EPIC.md` 行 156(e) 要求契约包测试同步通过。
7. **新入口不必与既有生产调用者兼容**：`authoring/vehicle.py:82 vehicle_model_from`（文件→`VehicleModel` 的唯一入口）在生产代码里**没有调用者**，只在 `tests/authoring/test_vehicle_assembly_documents.py` 闭环（锚点见 `EPIC.md` F3）——这是本行换入口的机会，也是 R1 的成因。
8. **产物变化判据**：用 `tasks/20260929-01-freeze/snapshot.py --check`（口径「未变化部分逐项相等 + 已登记差异」，登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`）；**硬门**是适配器路径产物与快照逐项一致，**03 不得产生差异，差异非空即失败**（只有 05 被允许改变既有产物并登记）。

## Non-Goals

- **不做 02 的配对机制**：本行**依赖 02 定下的配对契约**（`connections/matcher.py:102 match_requirements` 为跨子系统插接唯一通道、配对段字段已在 schema 中就位）。本行只**消费**该契约，不重定义它；配对段的字段与语义归 02。
- **不动试验台**：`subsystems/rig_link.py`、`rigs/` 一行不改（05 独占）。
- **不做轮端生命周期统一**：单轴/整车"删轮胎再建车轮"的消除与 K/C 凝结属 04；本行不改 `subsystems/wheel.py`、`subsystems/element_build.py`、`cases/kc_quasi_static/contract.py`。
- **但 `preparation/` 的图谱化属本行**（不是 Non-Goal，是必做项）：整个 `preparation/` 的模型访问（`vehicle_dynamic.py` 的 `:188`、`:207`、`:210`、`:341`、`:368-369`）与 `:373` 的 body 命名规则（今天的 `"chassis"` 字符串改写）改为消费装配图谱，适配器落在 `preparation/` 之外；**轮端内容段（`:900-903` 的 `VerticalTireElement` 拒收、`:1009-1025` 的轮胎构建）归 04**，本行不动、且必须在 03 之后。
- **不做镜像可配置与 `_SIDES` 收口**（属 06）；**单轮/三轮最小用例也归 06**。
- **不删除 `VehicleModel`**：降级为适配器，历史读取与既有调用者保持可用（`EPIC.md` Non-Goals）。
- **不改 C++ 内核**：ABI 七符号与版本常量（15/30/1/1）不变，`check_module_layering.py --strict --final` 保持绿。
- **不实现具体车型物理**：**3 轴与拖挂都要交付最小端到端用例**（拖挂见 Goal 2）；铰接副必须**先用现有副类型实现**，只有实测证明现有副类型不可行时才登记为内核范围并提请用户裁决，不得自行新增内核副类型。
- **不重录任何基线**；不新增 skip/xfail。

## Constraints

- **写范围**（照 `SUBTASKS.csv` 第 4 行 `notes`，逐条列明）：
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_assembly.py`（**入口段**）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_parts.py`（**前缀段 + 名字规则段**：`:172-173` 的焊合根命名与 `:414` 的 `referenced` 初值——父行 `SUBTASKS.csv` 第 4 行的 notes 已授权这两段，本行不得只改前缀段）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/entry.py`
  - `packages/suspension_multibody/src/suspension_multibody/authoring/vehicle.py`
  - `packages/suspension_multibody/src/suspension_multibody/authoring/solver.py`
  - `packages/suspension_multibody/src/suspension_multibody/connections/policy.py`
  - `packages/suspension_multibody/src/suspension_multibody/authoring/documents.py`（**放置与角色枚举段**：`PLACEMENT_ROLES` 与 `:139` 的 `placement_role` 校验；配对读取段归 02、子系统文档的可选对称声明段归 06）
  - `packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py`（**仅新增适配器读取路径**）
  - `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`（**放置段 + 轮数段 + 整车 `required` 多车身段**，见 Goal 2 与 Goal 6）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/`（**整目录**：`front_axle` / `rear_axle` 与两轴模型访问改为消费装配图谱，含 `vehicle_dynamic.py` 的 7 处与 `:373` 的 body 命名规则；**轮端内容段 `:900-903`、`:1009-1025` 归 04**）
  - 新增的装配图谱 / 适配器模块（**必须落在 `preparation/` 之外**，放 `authoring/vehicle.py` 或本行新增模块）
  - 对应测试：`packages/suspension_multibody/tests/authoring/`、`packages/suspension_multibody/tests/connections/`、`packages/suspension_multibody/tests/subsystems/`、`packages/suspension_multibody/tests/schema/`、`packages/suspension_multibody/tests/vehicle_assembly/`、`packages/suspension_multibody/tests/preparation/`（如存在）、`packages/suspension_contracts/tests/`
- **依赖 02 定下的配对契约**：`SUBTASKS.csv` 第 4 行 `depends_on=02`。跨子系统插接只经 02 交付的 `match_requirements` 通道；本行不得另立第二条推断路径，也不得改配对段字段的定义。
- **与 02 共享 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`，不得并行**：02 改**配对段字段**、03 改**放置段 + 轮数段 + 整车 `required` 多车身段**（`EPIC.md` 行 135）。本行必须在 02 落地之后才开始改该文件，且只动放置（`placement_role` 枚举）、轮数（`vehicle.wheels` 的 `minItems`/`maxItems`）与整车 `required`（`:46` 今天只列一个 `chassis`）三段；**只改 schema 会在 `authoring/documents.py:139` 被拒**（`PLACEMENT_ROLES` 在 `:62-64`），故本行同时放开 `authoring/documents.py` 的**放置与角色枚举段**。
- **与 04 分段串行（同一文件 `preparation/vehicle_dynamic.py` 上不许并行）**：03 改**模型访问段**（`:188`、`:207`、`:210`、`:341`、`:368-369`）与 `:373` 的命名规则段；04 只改**轮端内容段**（`:900-903`、`:1009-1025`），且**必须在 03 之后**。03 与 04 还都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，04 的 `depends_on=03`（`EPIC.md` 行 134），不得并行。
- **D1 硬约束**：严禁原地改 `VehicleModel` schema；`schema/vehicle.py` 只允许**新增适配器读取路径**，不得改 `front_axle` / `rear_axle` 字段形状（`EPIC.md` F1 锚点 `:244`、`:252-253`）与 `:262-265` 的四角校验。`model_dump(mode="json")` 的产物形状不得改变（`api.py:116`、`:284` 用它算 `model_hash`）。
- **过渡期兼容开关由本行删除并登记**（**不是「03 之前」，而是「03 完成之前」**）：`EPIC.md` 行 147——`"chassis"` / `"ground"` / 前缀拼接这类字符串改写是 R2 的消除对象；`"chassis"` 规则的三处锚点（`subsystems/vehicle_assembly.py:212-228`、`subsystems/vehicle_parts.py:172-173` 与 `:414`、`preparation/vehicle_dynamic.py:373`，见 `EPIC.md` F4）**全部属本行写范围**，均以本行落地为删除点（02 的 SPEC 明确不改这三个文件，故删除只能落在本行）。终局判据：`grep` 在装配路径（装配层三文件 + **整个 `preparation/` 目录**）无命中（`EPIC.md` 行 154(d) 与 G2），由本行收口、07 复验。
- **不得把"简化/试验台专用"的分支写进 role 接口**（`EPIC.md` 行 148）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。
- **`check_assembly_shape` 的规则必须数据驱动**：不得用"N == 3 特判"之类的新硬编码替换旧硬编码。
- **不得新增 skip/xfail**；`tests/adams` 的 47 个环境 skip 是既有的，不得增长。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据。

## Environment

- **Project root**: `c:/杂件/open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Package manager**: `uv`
- **Test framework**: `pytest`
- **Build command**: 无（纯 Python；内核 DLL 已构建，本行不动内核）
- **Existing test count**: 快速集 1015 passed / 1 xfailed；`tests/cases` 100；`tests/architecture` 147；`tests/adams` 160 passed / 47 skipped；kernel+contracts 60

## Risk Assessment

- [ ] **换入口会让现有产物漂移**（`EPIC.md` 行 196 已登记）→ 01 的字符化快照作硬门；**新入口与适配器路径并行**，先证明适配器路径产物与快照逐项一致再切换。差异逐项登记。
- [ ] **`VehicleModel` 适配器若要"顺手改 schema"就会连锁瘫痪**（D1）→ `schema/vehicle.py` 仅新增适配器读取路径；`model_dump(mode="json")` 形状由测试钉住；任何形状变化即视为越界。
- [ ] **契约 schema 是前置墙，改晚了会发现正例在更早一层被拒** → 先放开 `placement_role` 的 `middle`、`wheels` 的四轮限制**与整车 `required` 的多车身（`:46` 今天只列一个 `chassis`，拖挂的两车身总成在此被挡）**，**并同步放开 `authoring/documents.py` 的放置与角色枚举段（`PLACEMENT_ROLES` `:62-64` 与 `:139` 的校验）——只改契约 schema 不够，`middle` 会在文档读取层先被拒**，再改形状规则；实测拒绝点见 `EPIC.md` F2 与 `tasks/20260929-01-freeze/raw/triaxle_refusal.md`。
- [ ] **N 轴规则改成数据驱动会同时放开既有负例** → 保留 `suspension_axle` 与 `full_vehicle` 的原有约束断言，新增正例（3 轴）与负例（缺放置、重复放置）各一条，且负例报错**点名**。
- [ ] **拖挂与 `policy` / schema 的单车身限制耦合** → `connections/policy.py` 的角色数规则与 `assembly.schema.json:46`（整车 `required` 今天只列一个 `chassis`）都按数据驱动放开，可能同时放开既有负例：保留 `suspension_axle` 与 `full_vehicle` 的原有约束断言，并对多车身规则补正例与负例各一条。铰接副先用现有副类型，实测不可行才登记为内核范围并提请用户裁决。单轮/三轮归 06，本行不预设其声明形状。
- [ ] **`preparation/` 图谱化会让动态基线漂移** → 分步切换（先接适配器、再逐段改模型访问），**每步跑 `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check`**，26 个 artifact 必须逐字节一致；**本轮不得重录任何动态基线**（`dynamic_hash_baseline.json` 等冻结文件只读）。任何一位变化即退回该步，先给独立于结果字节的等价说明再继续。
- [ ] **`authoring/vehicle.py:82 vehicle_model_from` 无生产调用者**（F3）→ 换入口不影响既有生产调用者，但 `tests/authoring/test_vehicle_assembly_documents.py` 的闭环必须继续通过（适配器路径的回归网）。

## Deliverables

- **新增文档驱动装配器**：直接消费 `AssemblyDocument`，按条目清单逐条实例化子系统（按 `placement_role` 打前缀；镜像与否按条目声明），原生支持任意轴数与非 front/rear 的放置。
- **`VehicleModel` 适配器**：`VehicleModel` →（适配）→ 文档驱动装配器；**产物与 01 快照逐项一致**。`schema/vehicle.py` 只新增读取路径，字段形状不动。
- **形状规则数据化**：`connections/policy.py` 的 `full_vehicle` 规则改为数据驱动（`authoring/documents.py:62-64 PLACEMENT_ROLES` 与 schema 枚举同步放开，含整车 `required` 的多车身）。
- **契约 schema 改动**：`assembly.schema.json` 的放置段（接受 `middle`）、轮数段（接受非四轮声明）与整车 `required` 段（接受多车身，拖挂需要）；`authoring/documents.py` 的放置与角色枚举段（`PLACEMENT_ROLES` `:62-64`、`:139` 校验）同步放开——只改 schema 不行，`middle` 会先在文档读取层被拒。
- **测试**：
  - 3 悬挂整车总成装配 + 跑通一次 study 的用例
  - 拖挂铰接总成（牵引车 + 挂车两个车身侧体，经现有副类型的铰接副）装配并跑通一次 study 的用例
  - `check_assembly_shape` 的正例（3 轴）与负例（缺放置、重复放置）断言
  - 适配器路径产物对照 01 快照的用例
  - 契约包测试（接受 `middle` 与非四轮声明）
- **`raw/` 证据**（每步落地后追加，不预填）：
  - `raw/entry_grep.md` — 装配层三文件**与整个 `preparation/` 目录**的 `grep "front_axle\|rear_axle"`，以及 `"chassis"`/`"ground"` 字符串改写的实测输出与未收口项
  - `raw/adapter_parity.md` — 适配器路径产物与 01 快照的逐项对照与差异登记（口径「未变化部分逐项相等 + 已登记差异」，登记处 `raw/approved_deltas.json`；**03 不得产生差异，差异非空即失败**）
  - `raw/new_shape_cases.md` — 3 轴与**拖挂**用例的实跑命令、退出码与产物摘要；`dynamic_hash_sentinel.py --check` 每一步的逐字节比对结果
  - `raw/snapshot_diff.md` — 各步前后 `snapshot.py --check` 的结果（口径「未变化部分逐项相等 + 已登记差异」，登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`；**03 不得产生差异，差异非空即失败**）
- **PROGRESS 登记**：与 02 的契约边界、与 04 的未收口项、非四轮声明下 `VehicleModel` 适配器的取值来源。

## Done-When

- [ ] 一份 **3 悬挂（front / middle / rear 放置）**整车总成文件装配成功并跑通一次 study；实体清单与文件声明的子系统条目一一对应。
- [ ] **拖挂铰接总成**（牵引车 + 挂车两个车身侧体）经**现有副类型**的铰接副连接并跑通一次 study；形状规则与 `assembly.schema.json:46` 的整车 `required` 支持多车身。
- [ ] 装配路径（`subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`）**与整个 `preparation/` 目录**`grep "front_axle\|rear_axle"` 无命中（适配器落在 `preparation/` 之外）。
- [ ] **硬门**：`VehicleModel` 适配器路径产物与 01 快照**逐项一致**（体/点/约束/elements/connections 的集合与名字，含约束类型与端点）；`model_dump(mode="json")` 形状未变。
- [ ] `check_assembly_shape` 的 N 轴规则数据驱动，正例（3 轴）通过、负例（缺放置 / 重复放置）各自失败且报错点名。
- [ ] `assembly.schema.json` 接受 `middle` 放置、非四轮声明与整车 `required` 的多车身，`authoring/documents.py` 的放置与角色枚举段（`PLACEMENT_ROLES` `:62-64`、`:139` 校验）同步放开，`packages/suspension_contracts/tests` 通过。
- [ ] 本行写范围内的 `"chassis"` / `"ground"` 字符串改写规则（三处锚点均归本行）消除并登记；`grep` 在装配层三文件与整个 `preparation/` 目录无命中（G2 由本行收口、07 复验）。
- [ ] `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` 通过：26 个 artifact 逐字节一致，未重录任何动态基线。
- [ ] 无新增 skip/xfail；未重录任何基线。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests -q && \
uv run --no-sync pytest packages/suspension_contracts/tests -q && \
uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation && \
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```
（倒数第二条为动态基线硬门：26 个 artifact 逐字节一致；最后一条为 01 交付的产物不变判据，口径「未变化部分逐项相等 + 已登记差异」、登记处 `tasks/20260929-01-freeze/raw/approved_deltas.json`，**03 不得产生差异，差异非空即失败**——登记只能由 05 写。**01 已交付**：`snapshot.py` 与 `raw/assembly_snapshot.json` 已落盘，落地前后直接跑。）

## Demo Flow

1. 写一份 3 悬挂（front/middle/rear）整车总成文件 → 装配 → 跑一次 study → 导出实体清单，逐条对回文件条目。
2. 同一机制装配**拖挂铰接**（牵引车 + 挂车两个车身侧体，经现有副类型的铰接副）一次并跑通 study；记录 `check_assembly_shape` 的角色数规则、`assembly.schema.json:46` 的整车 `required` 多车身放开项，以及 `authoring/documents.py` 的放置与角色枚举段（`PLACEMENT_ROLES` `:62-64` 与 `:139` 校验）的同步放开项。
3. 走 `VehicleModel` 适配器路径装配既有组合 → 与 01 快照逐项比较 → 全等。
4. 对 `check_assembly_shape` 跑三条输入：3 轴（通过）、缺放置（失败）、重复放置（失败），记录报错原文。
5. `grep "front_axle\|rear_axle"`（装配层三文件 + **整个 `preparation/` 目录**）与 `grep "chassis"` 的输出，连同每一步 `dynamic_hash_sentinel.py --check` 的逐字节结果一并记入 `raw/`。
