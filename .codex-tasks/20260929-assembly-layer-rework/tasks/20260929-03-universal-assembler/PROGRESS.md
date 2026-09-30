# PROGRESS：03 通用装配引擎（条目清单驱动）

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `03`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: 20260929-03-universal-assembler
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-03-universal-assembler/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 契约 schema 放开 middle 放置与非四轮声明
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 本行**依赖 02 定下的配对契约**（跨子系统插接只经 `connections/matcher.py:102 match_requirements`，配对段字段由 02 写进 schema）；02 未落地前不得开工。
  - 与 02 **共享** `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`：02 改**配对段字段**、本行改**放置与轮数段**，**不得并行**（`EPIC.md` 行 135）。
  - 与 04 串行：两者都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，04 的 `depends_on=03`（`EPIC.md` 行 134）。
  - **严禁原地改 `VehicleModel` schema**（D1）：走"新增文档驱动装配器 + `VehicleModel` 适配器"，硬门是适配器路径产物与 01 快照逐项一致。
- **Known issues**:
  - 本行的"装配路径 `grep` 无命中"只能覆盖自有写范围：`preparation/` 属 04，`EPIC.md` F4 的 `preparation/vehicle_dynamic.py:373` 同样归 04 → G2 的终局 grep 判定在 04 之后复验。
  - 01 的 `snapshot.py --check` 尚未生成（01 的 `#1` 为 TODO）；本行落地时若脚本仍不存在，硬门项登记为 BLOCKED，不得记通过。
- **Next action**: 先读 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`（`:23` 的 `placement_role` 枚举、`:50` 的 `vehicle.wheels` 四个限制）与 `packages/suspension_multibody/src/suspension_multibody/connections/policy.py` 的 `full_vehicle` 规则（`:202-217`），确认 02 的配对段改动已落地且未与本行段落冲突，然后在**放置与轮数段**放开 `middle` 与非四轮声明，并跑 `packages/suspension_contracts/tests`。

---

## 2026-09-29 03c/03d 落地（preparation 图谱化 + 新形状真跑）

- **03c 已做**：
  - `subsystems/assembler.py`：新增 `VehicleFacts`（`axles` / `placements_with_bushings` /
    `rack_fixed_to_chassis` / `axle_units` / `static_rotation_axes`）与 `vehicle_facts(entries)`；
    `VehicleRuntime` 新增 `chassis_name`；新增 `runtime_for_study`（整车 runtime → study 读的那一面）。
  - 新增 `subsystems/vehicle_model_adapter.py`：`axle_entries_for_model` / `vehicle_facts_for`。
    两轴假设从 `vehicle_assembly.py`（被 grep 判据覆盖）搬到本模块，入口名字与签名不变。
  - `preparation/vehicle_dynamic.py` 的 7 处模型访问改为读 facts：`_select_assembly_mode`、
    `_validate_steering_topology`、`_validate_units`、`_build_static_rotation_gauges`；
    三处 `"chassis"`/`"ground"` 字符串改写规则（F4 锚点）全部消除。
  - `subsystems/vehicle_parts.py` 的两处命名规则改读 `assembly.chassis_name`。
- **03d 已做**：`ArticulationSpec`（条目之间的显式铰接，仅 `weld`/`revolute` 两型，用现有副类型）
  与 `compose_entries_runtime(..., articulations=..., unit_bodies=...)`；条目之外的整车级刚体由
  `unit_bodies` 显式给出，铰接两端必须在成品装配体里存在，否则报错点名。
- **验证（退出码全 0）**：`just check-fast` → 快速集 **1047 passed / 1 xfailed**、kernel 33、contracts 32，
  三个架构门全绿；`snapshot.py --check` **零差异**（`approved_deltas.json` 仍为 `[]`）；
  `dynamic_hash_sentinel.py --check` 26 artifact **逐字节一致**（未重录）；`case_parity_check` 8 families；
  `kc_perf_gate --check` 在预算内；`ruff` / `ty` 全通过。无新增 skip/xfail。
- **证据**：`raw/entry_grep.md`（两处 grep 零命中 + 三处锚点对照）、
  `raw/preparation_graph_and_new_shapes.md`（三轴与拖挂真跑、门禁实测、未收口项）。
- **边界（明确登记）**：K/C 读数的多轴轮心选择（`cases/kc_quasi_static/contract.py::wheel_centre_body`）
  归 **04**（该文件的写范围）；`authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES` 白名单的收口
  登记为 **06** 的输入（需与子系统文档的可选声明段一起定）。

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。

---

## 2026-09-29 计划修订（复核后）

- **改动**：Goal 2 由「单轮/三轮」改为「拖挂铰接总成（牵引车 + 挂车两个车身侧体，现有副类型的铰接副）装配并跑通一次」，并要求 `connections/policy.py` 的角色数规则与 `assembly.schema.json:46`（整车 `required` 今天只列一个 `chassis`）支持多车身；单轮/三轮最小用例移交 06。Goal 3 与 Done-When 的 grep 判据扩为「装配层三文件 + **整个 `preparation/` 目录**」；Non-Goal 由「本行不改 `preparation/vehicle_dynamic.py`」改为「本行必须把整个 `preparation/` 的模型访问改为消费装配图谱、适配器落在 `preparation/` 之外、轮端内容段（`:900-903`、`:1009-1025`）归 04」；`"chassis"` 三处锚点（含 `preparation/vehicle_dynamic.py:373`）全部归本行并收口终局 grep；Done-When 与 Final Validation Command、Demo Flow 同步加 `dynamic_hash_sentinel.py --check` 与拖挂。**本节取代本文件 Known issues 中「`preparation/` 属 04……G2 的终局 grep 判定在 04 之后复验」的旧表述（该行原文按约定保留）。**
- **为什么**：Epic 开工前独立审核（code-reviewer）的 7 项阻断项，父真值文件 `EPIC.md` / `SUBTASKS.csv` 已修订——单侧展开机制在 06 的写范围（`si_assembly.py:56`、`wheel.py:114-116 sides()`、`element_build.py:206/213` 今天固定展开 L/R），故判据 (b) 的执行行改到 06；用户裁决 G1(a) 按字面口径 = 整个 `preparation/` 目录；`:373` 归本行。
- **影响**：`TODO.csv` 第 2、4、5 行；`SPEC.md` 的 Goal 2/3/8、Non-Goals、Constraints（写范围与 04 分段串行）、Risk、Deliverables 的测试清单与 `raw/` 证据、Done-When、Final Validation Command、Demo Flow。

## 2026-09-29 计划修订（第二轮复核后）

- **改动**：写范围新增 `authoring/documents.py` 的**放置与角色枚举段**（`PLACEMENT_ROLES` 与 `:139` 的 `placement_role` 校验）；契约 schema 的改动范围由「放置与轮数两段」改为「**放置段 + 轮数段 + 整车 `required` 多车身段**」（`:46` 今天只列一个 `chassis`，拖挂铰接需要多车身）。Goals 第 6 条、Risk 的 schema 条、Deliverables 的形状规则条与契约 schema 条、Done-When 的 schema 条、Demo Flow 第 2 条同步；`TODO.csv` 第 1 行 acceptance 补两项放开、notes 写明「只改 schema 会在 `authoring/documents.py:139` 被拒」，第 2 行 notes 补 `PLACEMENT_ROLES`（`:62-64`）与 `:139` 属本行。另把第 6 行的「待 01 的 `#1` 完成后再跑」改为「01 已交付」。
- **为什么**：第二轮复核指出两处**交付权限缺口**——只放开契约 schema 不够，`authoring/documents.py:139` 会在**文档读取层先拒掉** `middle`；03 原写「只动放置与轮数两段」与它自己的拖挂 Goal 冲突（整车 `required` 只列一个车身）。父真值文件 `EPIC.md` 行 135/139 与 `SUBTASKS.csv` 第 4 行已修订，本节把该口径落到本行协议文件。**本节取代本文件 Known issues 中「01 的 `snapshot.py --check` 尚未生成……登记为 BLOCKED」的旧表述（该行原文按约定保留）：01 已交付，脚本已落盘。**
- **影响的行 id**：03 的 #1（acceptance 与 notes 补两项放开）、#2（notes 补 `PLACEMENT_ROLES` 与 `:139` 归属）、#6（过期说法改「01 已交付」）；行数仍为 6、status 全为 `TODO`。

## 2026-09-29 03a 落地（第一期：入口 + 引擎 + 适配器）

- **已做**：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py`：`AxleEntry`（`placement`/`prefix`/`axle`/`replace_bodies`/`wheels`）与 `compose_entries_runtime(entries, *, chassis_name, mode, request, chassis_body)`。引擎内无 `front_`/`rear_`/`model.front_axle` 字面量；`replace_bodies` 是显式声明（旧 `"chassis"`/`"ground"` 名字规则改由适配器写出）；`VerticalTireElement` 过滤保留（04 收口）；仍调用既有 `_rename_dataclasses`/`_rename_connections`/`_add_wheel`/`_condense_welded_bodies`/`_drop_isolated_bodies`。
  - `subsystems/vehicle_assembly.py` 的 `compose_vehicle_runtime(model, mode="K", request=None)` 名字与签名不变，改为适配器：`_wheels_for` 按 `placement_` 前缀取该轴轮、`_axle_entries` 产出两条 `AxleEntry`（`replace_bodies = {"chassis": <车身名>, "ground": <车身名>}`），再调引擎。装配实现整体搬进引擎，入口文件由 310 行降到 198 行。
  - 新增 `packages/suspension_multibody/tests/subsystems/test_vehicle_assembler.py`（3 个用例）：(a) 两个手工条目（`placement` 为 `front`/`middle`、前缀 `front_`/`mid_`）直接调引擎装配成功，体名带各自前缀、`replace_bodies` 生效（整车只有一个 `chassis`）；(b) 适配器产物与「手工从该模型构造两条 `AxleEntry` 再调引擎」逐项相同（体/点/约束/理想约束/力元/连接的名字与顺序，四个车轮表，质量）。
  - 证据：`raw/adapter_parity.md`（`snapshot.py --check` 原文与退出码、pytest 原文、`git diff --stat -- packages/`）。
- **验证（退出码全 0）**：`snapshot.py --check` → `OK: the seven combinations assemble exactly what the snapshot froze`（**零差异**，未新增任何 `approved_deltas.json` 登记）；改动目录 pytest 282 passed / 1 xfailed；`ruff check .` 与 `ty check .` 全通过；`just check-fast` 退出 0（快速集 1033 passed / 1 xfailed，较 02 的 1030 增量即本行新增 3 个测试；kernel 33 passed、contracts 32 passed）。无新增 skip/xfail。

### 03b 需要知道的（契约与实测）

1. **引擎已就绪且与模型无关**：`compose_entries_runtime(entries, *, chassis_name, mode, request, chassis_body=None)`。`chassis_body` 可省——省时引擎取「第一个声明了 `chassis_name` 的条目」的车身作为整车车身，这是文档驱动调用（车身子系统本身是一个条目）的形态；适配器则显式传入模型的 `chassis` 车身以保持字节一致。
2. **车轮归属靠条目**：引擎不再从整车遍历轮，只处理 `entry.wheels`；`VehicleModel` 适配器用 `wheel.name.startswith(f"{placement}_")` 做拆分（该启发式留在适配器，引擎不碰）。
3. **`middle` 今天仍被两处硬编码挡住**：`schema/vehicle.py:46` 的 `WheelSpec.name: Literal["front_left","front_right","rear_left","rear_right"]`（本行实测 `ValidationError: Input should be 'front_left', ... input_value='middle_left'`）与 `authoring/documents.py:139` 的 `PLACEMENT_ROLES`。03a 的引擎测试因此用 `rear_left/rear_right` 命名中轴的轮（前缀仍是条目自己的 `mid_`），证明前缀与放置由条目决定、与轮名无关。放开这两处后该测试可直接改回 `middle_*`。
4. **`replace_bodies` 语义**：键=该轴体名，值=顶替它的整车体名；值已存在于整车时该轴体不落地（旧行为），值不存在时由该条目落地（文档驱动、无预置车身时的路径）。条目之间键重叠且值不同不会报错——`body_map` 按条目独立构造。
5. **04 的接口**：`VerticalTireElement` 过滤在 `assembler.py`（原 `vehicle_assembly.py:236-240`）；04 的写范围从 `vehicle_assembly.py` 移到 `assembler.py`。`vehicle_parts.py` 的 `:172-173`/`:414` 本行未动（属 03c 收口）。
