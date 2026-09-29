# SPEC：03 通用装配引擎（条目清单驱动）

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `03`。

## Task Shape

- **Shape**: `single-full`

## Goals

把装配执行层的驱动源从"硬编码两轴 `VehicleModel`"换成"总成文件声明的子系统条目清单"（父 Epic 的 G1 / R1 / P1 / D1）。逐条承接 `SUBTASKS.csv` 第 4 行的 `acceptance_criteria`：

1. **3 悬挂（front / middle / rear 放置）整车总成装配成功并跑通一次 study**：这是 `EPIC.md` Done-When (a) 的最小实体；装配产物与文件声明的子系统条目**一一对应**。
2. **单轮（单侧悬架）与三轮（两悬架 + 一个单侧）各装配并跑通一次**：`EPIC.md` G1 判据 (b) 与 `EPIC.md` 行 156(f)；不只是"能构造"，要实跑。
3. **装配路径 `grep "front_axle\|rear_axle"` 无命中**：`EPIC.md` G1 判据 (a) 点名的路径为 `subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`、`preparation/`。今天是 `subsystems/vehicle_assembly.py:184-187` 把两轴硬编码成 `("front", model.front_axle, "front_")` / `("rear", model.rear_axle, "rear_")`（锚点见 `EPIC.md` F1）。
4. **`VehicleModel` 适配器路径产物与 01 快照逐项一致**（**零回归硬门**，D1）：**严禁原地改 `VehicleModel` schema**——`schema/vehicle.py:244` 的 `front_axle` / `rear_axle` 字段（`:252-253`）与 `:262-265` 的四角校验保持不变；本行的做法是**新增文档驱动装配器**（直接消费 `AssemblyDocument`，原生支持任意轴数与子系统拓扑）+ **把 `VehicleModel` 降级为向下兼容适配器**（绞杀者模式）。硬门判据 = 适配器路径产物与 01 的 `snapshot.py --check` 逐项一致，尤其 `model_dump(mode="json")` 的形状不得漂移（D1 的连锁风险：`api.py:116`、`:284` 用它算 `model_hash`）。
5. **`check_assembly_shape` 的 N 轴规则改为数据驱动，正例与负例各有断言**：今天 `connections/policy.py:202-217` 的 `full_vehicle` 规则写死 `role_counts={"suspension": 2, ...}` 与 `required_placements={("suspension","front"),("suspension","rear")}`，由 `:252-296 check_assembly_shape` 逐条执行（锚点见 `EPIC.md` F2）。正例 = 3 轴；负例 = 缺放置、重复放置，各自成功 / 失败。负例基线原文见 `tasks/20260929-01-freeze/raw/triaxle_refusal.md`（实测：`requires exactly two suspension subsystem(s), found 3`、`requires one front suspension`）。
6. **`assembly.schema.json` 接受 `middle` 放置与非四轮声明**：契约 schema 本身是一道**前置墙**，在装配逻辑之前生效——`placement_role` 枚举不含 `middle`、整车轮数被限定为四（锚点见 `EPIC.md` F2）；`EPIC.md` 行 156(e) 要求契约包测试同步通过。
7. **新入口不必与既有生产调用者兼容**：`authoring/vehicle.py:82 vehicle_model_from`（文件→`VehicleModel` 的唯一入口）在生产代码里**没有调用者**，只在 `tests/authoring/test_vehicle_assembly_documents.py` 闭环（锚点见 `EPIC.md` F3）——这是本行换入口的机会，也是 R1 的成因。
8. **产物变化判据**：本行落地前后各跑一次 01 的 `snapshot.py --check`；**硬门**是适配器路径产物与快照逐项一致，任何差异必须逐项登记（文件 + 步骤 + 前后值 + 等价性判定）。

## Non-Goals

- **不做 02 的配对机制**：本行**依赖 02 定下的配对契约**（`connections/matcher.py:102 match_requirements` 为跨子系统插接唯一通道、配对段字段已在 schema 中就位）。本行只**消费**该契约，不重定义它；配对段的字段与语义归 02。
- **不动试验台**：`subsystems/rig_link.py`、`rigs/` 一行不改（05 独占）。
- **不做轮端生命周期统一**：单轴/整车"删轮胎再建车轮"的消除与 K/C 凝结属 04；本行不改 `subsystems/wheel.py`、`subsystems/element_build.py`、`cases/kc_quasi_static/contract.py`、`preparation/vehicle_dynamic.py`。
- **不做镜像可配置与 `_SIDES` 收口**（属 06）。
- **不删除 `VehicleModel`**：降级为适配器，历史读取与既有调用者保持可用（`EPIC.md` Non-Goals）。
- **不改 C++ 内核**：ABI 七符号与版本常量（15/30/1/1）不变，`check_module_layering.py --strict --final` 保持绿。
- **不实现具体车型物理**：3 轴/拖挂只交付机制与端到端用例；拖挂所需的新副类型若真需要，属内核范围、本轮不做。
- **不重录任何基线**；不新增 skip/xfail。

## Constraints

- **写范围**（照 `SUBTASKS.csv` 第 4 行 `notes`，逐条列明）：
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_assembly.py`（**入口段**）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/vehicle_parts.py`（**前缀段**）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/entry.py`
  - `packages/suspension_multibody/src/suspension_multibody/authoring/vehicle.py`
  - `packages/suspension_multibody/src/suspension_multibody/authoring/solver.py`
  - `packages/suspension_multibody/src/suspension_multibody/connections/policy.py`
  - `packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py`（**仅新增适配器读取路径**）
  - `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`（**放置与轮数段**）
  - 对应测试：`packages/suspension_multibody/tests/authoring/`、`packages/suspension_multibody/tests/connections/`、`packages/suspension_multibody/tests/subsystems/`、`packages/suspension_multibody/tests/schema/`、`packages/suspension_multibody/tests/vehicle_assembly/`、`packages/suspension_contracts/tests/`
- **依赖 02 定下的配对契约**：`SUBTASKS.csv` 第 4 行 `depends_on=02`。跨子系统插接只经 02 交付的 `match_requirements` 通道；本行不得另立第二条推断路径，也不得改配对段字段的定义。
- **与 02 共享 `packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json`，不得并行**：02 改**配对段字段**、03 改**放置与轮数段**（`EPIC.md` 行 135）。本行必须在 02 落地之后才开始改该文件，且只动放置（`placement_role` 枚举 / `PLACEMENT_ROLES`）与轮数（`vehicle.wheels` 的 `minItems`/`maxItems`）两段。
- **与 04 串行**：03 与 04 都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，04 的 `depends_on=03`（`EPIC.md` 行 134），不得并行。
- **D1 硬约束**：严禁原地改 `VehicleModel` schema；`schema/vehicle.py` 只允许**新增适配器读取路径**，不得改 `front_axle` / `rear_axle` 字段形状（`EPIC.md` F1 锚点 `:244`、`:252-253`）与 `:262-265` 的四角校验。`model_dump(mode="json")` 的产物形状不得改变（`api.py:116`、`:284` 用它算 `model_hash`）。
- **过渡期兼容开关必须在 03 之前删除并登记**：`EPIC.md` 行 147——`"chassis"` / `"ground"` / 前缀拼接这类字符串改写是 R2 的消除对象；`"chassis"` 规则的三处锚点（`subsystems/vehicle_assembly.py:212-228`、`subsystems/vehicle_parts.py:172-173` 与 `:414`，见 `EPIC.md` F4）中，属于本行写范围的两处以本行落地为删除点；`preparation/vehicle_dynamic.py:373` 属 04。终局判据：`grep` 在装配路径无命中（`EPIC.md` 行 154(d) 与 G2）。
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
- [ ] **契约 schema 是前置墙，改晚了会发现正例在更早一层被拒** → 先放开 `placement_role` 的 `middle` 与 `wheels` 的四轮限制，再改形状规则；实测拒绝点见 `EPIC.md` F2 与 `tasks/20260929-01-freeze/raw/triaxle_refusal.md`。
- [ ] **N 轴规则改成数据驱动会同时放开既有负例** → 保留 `suspension_axle` 与 `full_vehicle` 的原有约束断言，新增正例（3 轴）与负例（缺放置、重复放置）各一条，且负例报错**点名**。
- [ ] **单轮/三轮用例可能与 06 的镜像契约耦合** → 本行只做"能装配并跑通"，镜像的默认行为与左右独立文件等价性归 06；本行用例不预设 06 的声明形状。
- [ ] **`"chassis"` 规则的第三处（`preparation/vehicle_dynamic.py:373`）属 04** → 即使本行把自有两处消除，`grep` 在**全装配路径**仍可能有命中；G2 的终局判定需在 04 之后复验，本行不得因此越界改 04 的文件，也不得把"已全部消除"写成已达成。
- [ ] **`authoring/vehicle.py:82 vehicle_model_from` 无生产调用者**（F3）→ 换入口不影响既有生产调用者，但 `tests/authoring/test_vehicle_assembly_documents.py` 的闭环必须继续通过（适配器路径的回归网）。

## Deliverables

- **新增文档驱动装配器**：直接消费 `AssemblyDocument`，按条目清单逐条实例化子系统（按 `placement_role` 打前缀；镜像与否按条目声明），原生支持任意轴数与非 front/rear 的放置。
- **`VehicleModel` 适配器**：`VehicleModel` →（适配）→ 文档驱动装配器；**产物与 01 快照逐项一致**。`schema/vehicle.py` 只新增读取路径，字段形状不动。
- **形状规则数据化**：`connections/policy.py` 的 `full_vehicle` 规则改为数据驱动（`authoring/documents.py:62-64 PLACEMENT_ROLES` 与 schema 枚举同步放开）。
- **契约 schema 改动**：`assembly.schema.json` 的放置段（接受 `middle`）与轮数段（接受非四轮声明）。
- **测试**：
  - 3 悬挂整车总成装配 + 跑通一次 study 的用例
  - 单轮（单侧悬架）与三轮（两悬架 + 一个单侧）各装配并跑通的用例
  - `check_assembly_shape` 的正例（3 轴）与负例（缺放置、重复放置）断言
  - 适配器路径产物对照 01 快照的用例
  - 契约包测试（接受 `middle` 与非四轮声明）
- **`raw/` 证据**（每步落地后追加，不预填）：
  - `raw/entry_grep.md` — 装配路径 `grep "front_axle\|rear_axle"` 与 `"chassis"`/`"ground"` 字符串改写的实测输出与未收口项
  - `raw/adapter_parity.md` — 适配器路径产物与 01 快照的逐项对照与差异登记
  - `raw/new_shape_cases.md` — 3 轴 / 单轮 / 三轮用例的实跑命令、退出码与产物摘要
  - `raw/snapshot_diff.md` — 各步前后 `snapshot.py --check` 结果
- **PROGRESS 登记**：与 02 的契约边界、与 04 的未收口项、非四轮声明下 `VehicleModel` 适配器的取值来源。

## Done-When

- [ ] 一份 **3 悬挂（front / middle / rear 放置）**整车总成文件装配成功并跑通一次 study；实体清单与文件声明的子系统条目一一对应。
- [ ] **单轮（单侧悬架）**与**三轮（两悬架 + 一个单侧）**各装配并跑通一次。
- [ ] 装配路径（`subsystems/vehicle_assembly.py`、`subsystems/vehicle_parts.py`、`subsystems/entry.py`、`preparation/`）`grep "front_axle\|rear_axle"` 无命中。
- [ ] **硬门**：`VehicleModel` 适配器路径产物与 01 快照**逐项一致**（体/点/约束/elements/connections 的集合与名字，含约束类型与端点）；`model_dump(mode="json")` 形状未变。
- [ ] `check_assembly_shape` 的 N 轴规则数据驱动，正例（3 轴）通过、负例（缺放置 / 重复放置）各自失败且报错点名。
- [ ] `assembly.schema.json` 接受 `middle` 放置与非四轮声明，`packages/suspension_contracts/tests` 通过。
- [ ] 本行写范围内的 `"chassis"` / `"ground"` 字符串改写规则消除并登记；`preparation/vehicle_dynamic.py` 的剩余一处点名交 04（G2 终局判定在 04 之后复验）。
- [ ] `tasks/20260929-01-freeze/snapshot.py --check` 通过（**待 01 的 `#1` 完成后再跑**；01 的快照脚本尚未生成，本行落地时若脚本仍不存在，此项登记为 BLOCKED 而不是通过）。
- [ ] 无新增 skip/xfail；未重录任何基线。

## Final Validation Command

```bash
cd /c/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests -q && \
uv run --no-sync pytest packages/suspension_contracts/tests -q && \
uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```

（最后一条为 01 交付的产物不变判据，**待 01 的 `#1` 完成后再跑**。）

## Demo Flow

1. 写一份 3 悬挂（front/middle/rear）整车总成文件 → 装配 → 跑一次 study → 导出实体清单，逐条对回文件条目。
2. 同一机制装配**单轮**（单侧悬架）与**三轮**（两悬架 + 一个单侧）各一次并跑通。
3. 走 `VehicleModel` 适配器路径装配既有组合 → 与 01 快照逐项比较 → 全等。
4. 对 `check_assembly_shape` 跑三条输入：3 轴（通过）、缺放置（失败）、重复放置（失败），记录报错原文。
5. `grep "front_axle\|rear_axle"` 与 `grep "chassis"` 在装配路径的输出，连同未收口项一并记入 `raw/`。
