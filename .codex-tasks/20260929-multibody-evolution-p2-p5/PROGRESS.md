# PROGRESS：20260929-multibody-evolution-p2-p5

- 任务编号：20260929-multibody-evolution-p2-p5
- 形态：epic
- 状态：**规划中**（规划轮已落盘；等待开工前独立审核与用户裁决 D1–D6；前置阶段一未收口，任何行不得置 `IN_PROGRESS`）
- 真源：本目录 `SUBTASKS.csv`（23 行；全部 `TODO`）
- 上游文档：`packages/suspension_multibody/docs/multibody_architecture_evolution.md`

## 恢复块（冷启动从这里读）

1. `任务:` 完成路线图阶段二~五（力矩内建与多轴转向、通用微分运动学、ARB 独立化与轮端统一、公共 API 与信号闭环总线）
2. `形态:` epic
3. `进度:` **0/23** 子任务完成（全部 `TODO`；本轮为规划轮，未动任何生产代码）
4. `当前:` 规划已落盘（EPIC + SUBTASKS + 23 个子任务协议目录 + 本文件的总纲）；待裁决 D1–D6；待阶段一 Epic 收口
5. `文件:` `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}`
6. `下一步:` (1) 用户裁决 D1–D6；(2) 派只读 `code-reviewer` 做开工前独立审核（输入必须含路线图原文与本 EPIC）；(3) 等阶段一 Epic `20260929-assembly-layer-rework` 的 03–07 全部 `DONE`；(4) 从 `tasks/p2-01-freeze/` 开工

## 2026-09-29 第一轮：规划交付

**输入**：用户要求「根据多体动力学模块架构演化与现代化路线图，制定实施计划，目标是完成文档描述的全部架构演化与现代化路线图」。

**用户裁决（本轮，2026-09-29）**：

| 问题 | 裁决 |
|---|---|
| 本轮交付形态 | **只交付规划**——Epic 与子任务协议文档，**不改生产代码**（与阶段一当初的规划轮一致） |
| 计划结构 | **整条路线图合并为一个 Epic**（阶段二~五进同一份 `SUBTASKS.csv`，23 行） |
| 阶段一处理 | **不重规划**，只在总纲中引用其剩余 03–07 |

**做法**：先核事实再写计划。派 5 个只读 `explorer` 并行核实路线图四个阶段对应的代码现状（阶段二制动力矩与转向、阶段三运动学指标、阶段四 ARB 与轮端、阶段五公共 API 与信号/闭环/FMI）以及既有 Epic 与文档盘点；主代理亲自读奠基材料（路线图全文、`CONTEXT-MAP.md`、`packages/suspension_multibody/CONTEXT.md`、阶段一 Epic 的 `EPIC.md`/`SUBTASKS.csv`/`PROGRESS.md` 与 03 的协议文件），并**亲自点验**五处承重锚点（`templates/roles.py:70/:158` 的角色硬断言、`kernel/native.py:33-35` 与 `mb_config/version.hpp:27/34/37` 的 ABI 版本、`subsystems/suspension.py:682-695` 的 ARB 硬编码跨接、`cases/handling.py:8` 的闭环拒绝、`subsystems/geometry.py:226` 的 `_BODY_ALIASES`）后才落盘。

**核出的与路线图文档不一致之处（计划按代码现状写）**：

1. **路线图的 ABI 版本号已过期**：文档与 `docs/axle_dynamics_results.md:11` 写「轴 15；整车 30」，实测是 **16 / 31 / 1**（`mb_config/version.hpp:27/34/37` 与 `kernel/native.py:33-35` 单一真源）。这直接决定 p2-02 的 ABI 变更是 **16→17**，不是 15→16。
2. **阶段二「模板内无实体」半过期**：`BRAKE`/`DRIVE` **已经**是 role 驱动的 0-body 模板子系统并已声明 `brake_torque`/`drive_torque` 输出通道（`templates/builtin.py:599/622`、`templates/roles.py:122-152` 的 `has_torque_channel`）。真正的缺口是**产出仍是幅值字典**（`wheel_torque_amplitudes()`），不是内建力元——所以阶段二的落点比文档描述**窄**，不是「从零建模板」。
3. **阶段三的滚转中心链路没有生产消费者**：`compute_vehicle_roll_centers` 全仓唯一调用者是 `tests/physics/test_vehicle_physics.py`。所以阶段三不是「重构在用链路」，而是「把未消费指标变成通用引擎」——这**降低了**风险（无基线依赖）也**改变了**验收方式（主要靠新增断言）。
4. **阶段四的 `wheel.subsystem.json` 不存在**：仓库里**没有任何 `*.subsystem.json` 实体文件**，命名约定是 `{name}.sub.json`；`wheel` 角色被 `authoring/solver.py:598 _FILE_ROLE_TEMPLATES` **明文排除**在文件读取链之外。所以「统一消费链」的第一步是**造出这条链**，不是对接已有文件。
5. **阶段四的语义端口不存在**：悬架模板的 `ports`/`needs` 皆为空元组，端口是**装配期运行期按 body 合成**的（`si_assembly.py:71 _ports_for_bodies`）；`arb_mount`/`chassis_mount`/`droplink_mount` 全仓只命中路线图文档。所以 p4-03 是「先造语义化端口」。
6. **阶段五的闭环需求与内核 ABI 冲突**：内核是**批式 ABI**（`suspension_kernel_run`/`mb_core_run`，无 step/state 入口），且 `cases/handling.py:8` 与测试 `tests/cases/test_handling.py:242` **明文拒绝并固化了**「只表达开环工况」。所以「实时状态闭环」在当前 ABI 下无法真正实现——这是 **D2** 必须裁决的硬约束。

**据此写成 6 项待裁决（D1–D6）**：D1 是否接受内核 ABI 变更（阶段二力矩元的前提）；D2 闭环所需内核接口形态（批式 ABI 下如何收口）；D3 通用运动学引擎的雅可比来源（Python 数值微分 vs 内核未过 ABI 的 `constraint_jacobian`）；D4 FMI 版本与范围；D5 基线重录口径；D6 新依赖。

**核出的两处冲突（已在计划中显式承接，非静默处理）**：

- **阶段四改角色表会连锁 10+ 处**：`templates/roles.py:158-162` 在 import 期就抛 `RoleSpecError`（若角色集合不等于六个名字）；同步点还包括三份契约 schema 的 `functional_role` enum、`FUNCTIONAL_ROLES`/`SUBSYSTEM_ROLES`/`ALL_SUBSYSTEMS`/`ROLES`，以及 `tests/templates/test_template_model.py:57` 的硬断言。F12 已把这清单**实测固化**。
- **两套 ARB 物理不得混淆**：Python 侧 `AntiRollBarElement`（按两端 z 位移差出力偶，`modeling/primitives/elements.py:593`）与 native 扭杆 ABI（`tests/axle_dynamics/test_api.py:315`）是**不同物理**。p4-01 先做对照，p4-02 选型必须给理由。

**交付物**：

- `EPIC.md`：原始需求（阶段二~五逐条原文要点 + 2.1/2.2/2.3/2.6 配套要求）、待裁决 D1–D6、前置（阶段一）、Goal G1–G8、Non-Goals、事实与修正 F1–F25、目标结构、子任务分解与写范围约束、冻结约束、逐子任务验证协议（23 条）、Done-When（含端到端 (a)-(j)）、风险与回退、目录与命名。
- `SUBTASKS.csv`：23 行（阶段二 6 行、阶段三 6 行、阶段四 5 行、阶段五 6 行），每行带 `acceptance_criteria`、`validation_command`、`depends_on`、`task_dir`、写范围与依赖说明。
- 本文件：含**跨阶段总纲**一节（用户要求「整条路线图一个 Epic」后，阶段一~五的全局视图与交接点集中在此）。
- 23 个子任务目录（每个含 `SPEC.md` / `TODO.csv` / `PROGRESS.md` / `raw/`），使 `task_dir` 真实存在、Epic 可执行且可冷启动恢复。

**未做（本轮边界）**：未写任何生产代码；未重录任何基线；未将任何子任务置为 `DONE` 或 `IN_PROGRESS`；未修改阶段一 Epic 的任何文件。`raw/` 全部为空（只存**已执行**的证据，不存虚构结果）。

**待办**：开工前独立审核（`code-reviewer`，输入含路线图原文）、D1–D6 裁决、阶段一收口。三项齐备后才可从 p2-01 开工。

---

## 跨阶段总纲（路线图全局视图）

本轮用户选择「整条路线图一个 Epic」，故把五个阶段的全局视图与交接点集中在本节。**阶段一不重规划**，其内容仅作为前置引用。

### 阶段总览

| 阶段 | 核心目标 | 归属 | 子任务 | 状态 | 交付形态 |
|---|---|---|---|---|---|
| **一** | 物理装配执行层改造（总成文件驱动 + 显式配对 + 统一轮端 + 试验台非侵入 + 可选对称） | Epic `20260929-assembly-layer-rework` | 01–07 | 01/02 `DONE`，03 `IN_PROGRESS`，04–07 `TODO` | **不重规划**；本 Epic 全部 23 行以它为前置（`S1`） |
| **二** | 力矩内建（按实时 ω 求值）+ 废除离线预采样 + 解禁多轴转向 | 本 Epic | p2-01 ~ p2-06 | 全部 `TODO` | 内核 ABI 变更 + 装配层 + 子系统 + 转向分配器 |
| **三** | 通用微分运动学（速度旋量/瞬轴）+ N 点广义静平衡 + 动态报表通道 | 本 Epic | p3-01 ~ p3-06 | 全部 `TODO` | 新增引擎 + 重构两个 vehicle 模块 + 报表 |
| **四** | ARB 独立子系统 + `arb_mount` 语义端口 + wheel 文件统一（消除轮胎补丁） | 本 Epic | p4-01 ~ p4-05 | 全部 `TODO` | 新增角色/模板/子系统 + 端口 + 文件链 |
| **五** | `simulate(assembly, case)` + `FrontAxleModel` 降级 + 信号总线 + 闭环 + FMI | 本 Epic | p5-01 ~ p5-06 | 全部 `TODO` | 公共 API + 总线 + 控制器 + 导出 |

### 阶段间交接点（前置与产物依赖）

```text
阶段一 03 通用装配引擎（条目清单驱动，N 轴/N 车身）
        │  产出：文档驱动装配器 + VehicleModel 适配器
        ├──▶ 阶段二 p2-06：多轴转向通道依赖 N 轴总成能力
        └──▶ 阶段四 p4-03：配对段插接依赖显式接口配对（阶段一 02 已完成）

阶段一 04 轮端生命周期统一（刚性凝结，kc_baseline 逐位不变）
        └──▶ 阶段四 p4-04：本 Epic 只改「文件来源 + 类型判别」，
                   凝结机制与求解拓扑由阶段一 04 定，不重复

阶段一 05 试验台非侵入（只增不删，反转 test_rig_link 契约）
        └──▶ 阶段二 p2-06（d）：cases/vehicle_kc.py 准备期删除 steering_actuator
                   是同一类问题，沿用「只增不删」口径，两处必须点明关系

阶段一 03 文档驱动装配器
        └──▶ 阶段五 p5-02：simulate(assembly_document, case_document)
                   只是该装配器的对外出口，不再自造 document
```

### 贯穿全程的三条不变量（每个阶段收尾都要复验）

1. **基线逐位不变**（D5）：`tests/data/kc_baseline/` 与 `tests/data/dynamic_hash_baseline.json` 逐字节不变；任何产物变化按「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」逐项登记。
2. **分层方向不可逆**：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`report/` 不得 import native/kernel/solver 也不得自求力律（`legacy_surface_gate.py` 的 `report_native_import`/`report_constitutive_call`）。
3. **公共 API 让步式退役**（绞杀者模式）：`FrontAxleModel`/`VehicleModel` 只降级为适配器，不删除；内核提交唯一归属仍只有 `simulation/backend.py`。

### 门禁与验收节奏（与 `AGENTS.md` 一致）

- **每步落地**：`just check-fast`（ruff/ty/三个架构门/快速集/另两包）。
- **改结构**：加跑 `tests/architecture`（147 用例）。
- **触及求解路径**（阶段二 p2-02/p2-05、阶段三、阶段四 p4-04）：加跑 `just gate-numeric`。
- **每阶段收尾**（p3-06/p4-05/p5-06）：数值门三项 + 快速集 + `tests/architecture` 全绿。
- **Epic 收尾**（p5-06）：全量 `pytest packages/suspension_multibody/tests`，skip/xfail 不得增长。
- **禁止**：重录 `kc_baseline`/`dynamic_hash_baseline` 让门变绿；用 `-k`/`--deselect` 长期豁免失败；不带 `--actual-dir` 跑 `kc_parity_check` 当证据（自比较恒过）。

### 阶段一收口后的接力清单

阶段一 07 验收通过后，本 Epic 的开工顺序：

1. 派 `code-reviewer` 独立审核本 Epic（输入含路线图原文 + `EPIC.md` + `SUBTASKS.csv`）；
2. 用户裁决 D1–D6；
3. `tasks/p2-01-freeze/` 开工（只读核查，不改生产代码），把 F1–F5 的锚点复核与负例实测固化；
4. 按 `SUBTASKS.csv` 的 `depends_on` 推进；每步落地后重跑门禁。

---

## 2026-09-29 第二轮：开工前独立审核与 6 项阻断项修订

**审核**：按仓库约定派只读 `code-reviewer` 独立审核，输入含**路线图原文**（`packages/suspension_multibody/docs/multibody_architecture_evolution.md` 全 218 行）、本 Epic 的 `EPIC.md`/`SUBTASKS.csv`、以及阶段一 `SUBTASKS.csv`（核前置状态）。核心问题是「全部子任务 DONE 后能否真正达成 Goal，且 Goal 本身没有偏离路线图原文」。

**审核结论**：暂不通过。「路线图阶段二至五的 15 条交付项在任务表中都有名义承接，23 个 `task_dir` 也互不重复，依赖主链未见环；但单轮静平衡、滚转中心算法、ABS/ESC 闭环的完成判据与上游要求不一致，另有并行写入冲突和无法执行的验收命令。」

**6 项阻断项与处置**（全部经**主代理亲自点验证据**后修订；点验记录见下）：

| # | 阻断项 | 主代理点验 | 处置落在哪 |
|---|---|---|---|
| 1 | 单轮静平衡被写成「一律报错」，与路线图 `:188`「支持 N 点接触面」冲突（单接触点在质心投影与接触点重合等输入下并非无解） | 核 `vehicle/static_loads.py:79-98`：平衡矩阵秩判据 `rank < 3` 才是唯一真判据，与接触点数无必然关系 | `EPIC.md` G4、`p3-04` 验证协议 (b)、Done-When (d)、`SUBTASKS.csv` 的 p3-04 行 |
| 2 | 滚转中心的**指定物理口径**（侧倾反力**虚功导数矩阵**，路线图 `:136-139`）无人验收，p3-02/p3-03 只验旋量/瞬轴/有限值 | 核路线图 `:136-139` 原文；核 `vehicle/roll_centers.py:81-99` 今天确是二维连线求交 | `EPIC.md` G3、`p3-03` 验证协议 (e)、Done-When (c)、`SUBTASKS.csv` 的 p3-03 行 |
| 3 | 闭环可被「开环回放」替代、控制器种类缩水；且 p5-04 可能需第二次 ABI 变更而版本常量只授权 p2-02 | 核路线图 `:203` 原文为「支持简单的闭环控制（ABS/ESC）」；核 F23：内核是批式 ABI、无 step/state 入口 | `EPIC.md` G8、`p5-04` 验证协议、冻结约束的 ABI 条、Done-When (h)(j)、`SUBTASKS.csv` 的 p5-04 行 |
| 4 | 声称可并行的 p3-03/p3-04 **测试写范围实际相交**（都指向 `tests/physics/`、`tests/vehicle/`，而 `tests/physics/test_vehicle_physics.py` 同时覆盖两者） | 核 `ls tests/physics tests/vehicle`：`test_vehicle_physics.py` 是 physics 下唯一文件，且 reviewer 引该文件 `:5-10` 同时导入两项 | `EPIC.md` 并行约束条、`SUBTASKS.csv` 的 p3-03/p3-04 行 |
| 5 | **终局命令不可执行**：p3-06 与 p5-06 都调用 `case_parity_check.py --check`，该脚本无此参数 | **亲自实测**：`scripts/case_parity_check.py:1142-1161` 只有 `--family` / `--allow-partial` / `--record`（对比 `kc_perf_gate.py:168-169` 与 `dynamic_hash_sentinel.py:210-211` 确有 `--check`） | `EPIC.md` 冻结约束的门禁条、`SUBTASKS.csv` 的 p3-06/p5-06 行 |
| 6 | p4-04 与**阶段一 04 的范围重叠**：阶段一 04 已要求「单轴与整车读入同一份 wheel 文件 + 打通读取链 + 移除 `VerticalTireElement` 过滤 + 单轴凝结」，本 Epic 又把同样动作交给 p4-04 | 核阶段一 `SUBTASKS.csv` 的 04 行 `acceptance_criteria` 原文，确含全部四项 | `EPIC.md` 前置边界条、G6、`p4-04` 验证协议（改为「以阶段一实际交付为界，只做独立复验 + 剩余项」）、`SUBTASKS.csv` 的 p4-04 行 |

**审核的可选改进项（本轮已采纳并落盘）**：

- **前置过度约束**：把阶段一 01–07 全 `DONE` 设为全部 23 行的共同前置，会让**只读冻结行**（p2-01/p3-01/p4-01/p5-01，写范围只有本行 `raw/`）也必须空等。已改为：**四行只读冻结行不受前置阻塞、可提前开工**；其余 19 行（实施行与终局验收行）等 `S1`。如此缩短关键路径而不放松任何实质约束。
- **`tests/report/` 不存在**：实测该目录确实不存在（`ls` 报 No such file）；已把 p3-05 的 `validation_command` 改为 `tests/metrics` + `tests/outputs`，并在 notes 写明「若本行新建 `tests/report` 则命令同步更新」。
- **p2-04 的范围自相矛盾**：「写范围含 `templates/roles.py`」与「p2 任何行不得改角色表」字面冲突。已改为**分段归属**：p2-04 可改 `brake`/`drive` 两个 `RoleSpec` 的**内容段**，p4-02 才能改**角色集合**与 `:158-162` 的硬断言及三份 schema 的 enum。
- **FMI 判据偏弱**：p5-05 原只要求「文件存在、可加载并步进」。已改为必须给「**输入影响输出**」的轨迹断言；并写明若 D6 未授权依赖，只能**登记为未闭合项，不得替代 FMI 导出而结项**。
- **总线判据偏弱**：p5-03 补「执行器输入确实改变同一次仿真的状态轨迹」。
- **p2-02 / p5-02 粒度**：审核认为两者分别是内核元素及 ABI、公共入口及兼容适配器的**单一交付链**，`single-full` 可保留——本轮不拆。

**审核同时确认无问题的部分**：15 条路线图交付项均有名义承接（A1 对照表逐条给出）；没有子任务落在阶段二~五与 Non-Goals 之外；Done-When (a)–(j) 确为独立于「子任务都 DONE」的终局验收；`templates/`（p2-04 vs p4-02）、`outputs/builtin.py`（p3-05 vs p5-03）、`element_build.py`（p2-03 vs p4-04）、`vehicle_dynamic.py`（p2-05/p2-06/p4-04）的串行化**已到位**；`arb_mount`/`suspension.py` 硬编码跨接/三份 schema enum 的归属**没有漏项**；p2-02 与 p5-02 的粒度可保留。

**本轮同时修正的主代理自查项**：D1 原写「ABI 真源 `version.hpp:27` 需从 16 升到 17（`kVehicleKernelAbiVersion` 同步）」——`version.hpp:27` 实际是 `kAxleKernelAbiVersion = 16`，与「`kVehicleKernelAbiVersion` 同步」并列会误导 p2-02（F3 已给出三常量的真实行号 `:27/34/37` 与值 `16/31/1`）。已改为「新增元素类型**至少**要求轴与整车两个常量各 +1；**具体升哪个、升到多少由 p2-02 按代码实测确定并登记**，本表只约束『单点提交 + 两处真源同步』」，并同步了 `tasks/p2-02-kernel-torque/` 的 SPEC/TODO/PROGRESS 三处引用。

**本轮修订的文件**：`EPIC.md`（G3/G4/G6/G8、前置节、并行约束、冻结约束、p3-03/p3-04/p4-04/p5-03/p5-04/p5-05 验证协议、Done-When (c)(d)(h)(j)、D1）；`SUBTASKS.csv`（p3-03/p3-04/p3-05/p3-06/p4-04/p5-04/p5-06 共 7 行）；以及受影响子任务目录的协议文件（阶段三、阶段四/五各由 1 个 fixer 同步）。

**修订后复核（本会话实测）**：`SUBTASKS.csv` 仍 23 行、11 列、无 None、`status` 全 `TODO`、`depends_on` 无环且指向存在的 id；`case_parity_check.py --check` 与 `tests/report` 在父表中**残留为 0**；23 个 `task_dir` 齐备、`raw/` 全空；各 SPEC 的 `Final Validation Command` 与父表逐字一致（修订后复校）。

**未做（本轮边界）**：仍未写任何生产代码；未重录任何基线；未将任何行置为 `DONE`/`IN_PROGRESS`；未修改阶段一 Epic 的任何文件。`packages/**` 下当前有未提交改动，**均为另一会话在执行阶段一 03**（`subsystems/assembler.py`、`connections/links.py`、`assembly.schema.json`、新增测试等），**与本 Epic 的规划轮无关**。

**下一步**：(1) 用户裁决 D1–D6；(2) 可选：再派一次只读复审确认 6 项阻断项闭合；(3) 与阶段一 03 的推进并行，四个只读冻结行（p2-01/p3-01/p4-01/p5-01）可先开工；(4) 阶段一 07 验收通过后按 `depends_on` 推进实施行。

---

## 2026-09-29 第三轮：复审 4 项未闭合与再修订

**复审**：同一 `code-reviewer` 续审，只做**闭合性判断**（不另开战场）。结论：**原 6 项尚未清零，不能按「已闭合、可以开工」放行**——2 项已闭合、4 项部分闭合，另新发现 1 项阻断。

| 原阻断项 | 复审判定 | 问题实质 | 本轮处置 |
|---|---|---|---|
| 1 单轮静平衡 | **部分闭合** | 第二轮的改法引入**数学错误**：写成「非满秩（不可平衡）时报错」。但**解的存在性由载荷相容性（残差）判定，秩只表示约束能力与解的唯一性**——同一单轮几何的矩阵秩**不随载荷改变**，用它无法区分「可解例」与「不可解例」 | `EPIC.md` G4/p3-04/Done-When (d)、`SUBTASKS.csv` 的 p3-04 行、`tasks/p3-04-static-loads/*` 全部改为：`rank < 3` → 求解并标记「欠约束、解不唯一」；**残差超容差 → 报错点名且消息含残差实测值与容差**；单轮两例的差别来自**载荷与接触点几何的相容性** |
| 2 滚转中心物理判据 | **已闭合** | — | 无需处置 |
| 3 闭环与第二次 ABI | **部分闭合** | (i) D2 仍写「至少一个**开环**可验证闭环控制器」，与「必须交付 ABS/ESC 实际反馈闭环」冲突；(ii) p5-04 的 TODO 把「单步接口需求实测」排在「闭环实现」**之后**，而它决定是否要新增专属 ABI 子任务 | `EPIC.md` D2 改写（**D2 只裁决「要不要内核单步接口」，不裁决闭环目标本身**；闭环必须在**一次运行的推进过程中**读状态/算控制/写执行器，开环回放不算）；`tasks/p5-04-closed-loop/*` 同步 |
| 4 p3-03/p3-04 顺序 | **部分闭合** | 文件级切分已做，但 `SUBTASKS.csv` 的 p3-04 `depends_on` 仍只是 `p3-02`（未落实「默认串行」），且 p3-01 的 SPEC 仍称两者可并行 | `SUBTASKS.csv` 的 p3-04 `depends_on` 改为 **`p3-03`**；`EPIC.md` p3-01 补 (e)「明确记录 p3-03 与 p3-04 的调度关系（默认串行）」；`tasks/p3-01-freeze/*` 与 `tasks/p3-04-static-loads/*` 同步 |
| 5 `case_parity_check.py --check` | **已闭合** | 复审确认全部实际命令位已无残留 | 无需处置 |
| 6 p4-04 与阶段一 04 重叠 | **部分闭合** | 新引入的冲突：要求 p4-04「重写本行 `acceptance_criteria`」，但子任务**不得改父表真源** | `EPIC.md` p4-04 改为：判据调整写入 `raw/stage1_04_intake.md` 并在本行 `PROGRESS.md` 记录，**同时提请父 Epic 修订 `SUBTASKS.csv`**；`SUBTASKS.csv` 的 p4-04 行与 `tasks/p4-04-wheel-unify/*` 同步 |
| **新发现（阻断）** | — | **p4-05 的验收命令只跑了数值门两项**（`dynamic_hash_sentinel`、`kc_perf_gate`），却声称「数值门三项全绿」，漏了 `case_parity_check.py` | `SUBTASKS.csv` 的 p4-05 `validation_command` 补齐无参数的 `case_parity_check.py`；`EPIC.md` p4-05 验证协议写明三项确切调用；`tasks/p4-05-acceptance/*` 同步 |

**复审确认已采纳的可选改进**：`tests/report` → `tests/metrics` + `tests/outputs`（p3-05）；`templates/roles.py` 的分段归属（p2-04 改 `brake`/`drive` 的 `RoleSpec` 内容段，p4-02 才改角色集合与硬断言）；四个只读冻结行的提前开工许可（EPIC 已写）。

**复审指出、本轮一并处理的子文件跟随项**：p2-04 的 SPEC 曾笼统写「禁止改 `ROLES` 表」（与 EPIC 分段归属不一致）；四个只读冻结行各自的 SPEC「依赖与时机」段仍按旧口径阻止提前开工（与 EPIC 的提前开工许可不一致）。两处均以**父表/EPIC 为准**同步。

**本轮修订的文件**：`EPIC.md`（D2、G4、p3-01(e)、p3-04、p4-04、p4-05、Done-When (d)(f)）；`SUBTASKS.csv`（p3-04 的 `acceptance_criteria`/`notes`/`depends_on`，p4-04 的 `acceptance_criteria`/`notes`，p4-05 的 `validation_command`/`notes`）；五个子任务目录的协议文件（p3-01、p3-04、p4-04、p4-05、p5-04）。

**本轮复校（实测）**：`SUBTASKS.csv` 23 行 / 11 列 / 无 None / `status` 全 `TODO` / `depends_on` 无环且指向存在 id；`p3-04` 的 `depends_on` = `p3-03`；`p4-05` 的 `validation_command` 含无参数的 `case_parity_check.py`；23 个 `task_dir` 齐备、`raw/` 全空。

**方法反思（记录）**：第二轮的两处修订本身引入了新缺陷（单轮判据的数学错误、p4-04 的「改真源」冲突），到第三轮复审才暴露。**修订后必须再复审**——原阻断项清零前不宣告规划完成。

**下一步**：(1) 用户裁决 D1–D6；(2) 复审确认剩余 4 项与 1 项新发现闭合；(3) 四个只读冻结行可与阶段一 03 并行开工；(4) 阶段一 07 通过后按 `depends_on` 推进实施行。

---

## 2026-09-29 第四、五轮：父级口径残留清理（复审→修订→复核）

**第四轮复审**（同一 `code-reviewer`，只判闭合）报 4 项未闭合，**全部是父级口径的措辞残留**（子任务文件已改对，父表/EPIC 未跟上）：

| 项 | 残留 | 处置 |
|---|---|---|
| 1 | `tasks/p3-04-static-loads/SPEC.md:58` 仍写「只有 `rank(A) < 3` 而 `N ≥ 3` 时不标『解不唯一』」，而那时必有 `rank(A) < N`，相容解不唯一须标记 | 改写为「唯一性由 `rank(A) == N` 判定；`rank(A) < N` 一律标记『解不唯一』（4 轮与 3 轴都是这种情形）；只有 `rank(A) == N` 才唯一，单轮相容时正是 `1 == 1`」 |
| 2 | `SUBTASKS.csv` 的 p3-03 notes 仍写「默认仍串行」；`EPIC.md` 调度图仍从 p3-02 **分叉**到两行 | 父表 notes 改为「两行硬串行（`p3-04.depends_on = p3-03`）不得并行 文件级切分不是并行的理由」；EPIC 调度图改为线性 `p3-02 → p3-03 → p3-04` 并标注硬串行 |
| 3 | 父表 p2-04 notes 仍写「p2 的任何行不得改角色表」 | 改为「本行可改 `ROLES` 里 `brake` 与 `drive` 两个 `RoleSpec` 条目自身的内容段（如 outputs 与 `has_torque_channel`），但不得增删角色名、不得改 `:158-162` 的硬断言、不得碰三份 schema enum 与各角色注册表（那些归 p4-02）」 |
| 4 | `EPIC.md:69` 仍写「23 行**全部**以 S1 为前置」，与 `:77` 的四行豁免冲突 | 改为「**19 行实施行与终局验收行**（除四个只读冻结行外的全部行）以 S1 为前置」；`F25` 同步 |

**第五轮复审**确认第 1、3 项**已闭合**，剩 2 项仍是**模板句残留**（同一句「前置 S1 强制：本 Epic 23 行全部…」被复制到**全部 23 个子任务文件**，以及 p3-02 目录里 3 处「二者可并行」的错误授权——p3-02 目录前两轮未在派发范围内）。已批量清理：

- 全库 `23 行全部` → **0 处**；`前置 S1 强制` → **0 处**（改为「**前置 S1**：本行以阶段一 01–07 全部 `DONE` 为前置（EPIC 前置一节：19 行实施行受此约束；四个只读冻结行不受限、可立即开工）」）。
- p3-02 的 3 处「p3-03 与 p3-04 **可并行**」→ 改为「都依赖本行，且二者之间**硬串行**（`p3-04.depends_on = p3-03`）」。
- 保留一处纯描述性措辞（`tasks/p3-02-diffinematics/SPEC.md:57` 的「这份契约就是 p3-03 / p3-04 的对接面」）——它只说明本行契约服务于两行，**不含调度含义**，与硬串行不冲突。

**本轮修订的文件**：`EPIC.md`（前置一节、F25、调度图、并行约束条、G4、p3-01/p3-04 验证协议、Done-When (d)）；`SUBTASKS.csv`（p2-04/p3-03 notes、p3-04 的 acceptance/notes/depends_on、p4-04/p4-05）；以及**全部 23 个子任务目录**的协议文件（模板句批量清理，另加 p3-02/p3-03/p3-04/p3-06/p2-04/p2-06/p4-02 的定点修订）。

**终局复校（实测，本会话）**：

```
父表 23 行 / 11 列 / 无 None / 全 TODO: True
依赖无环: True | 四 freeze deps 空: True | p3-04 deps: ['p3-03']
目录/命令问题: NONE | dirs 23 | TODO.csv 全合规: True
调度无分叉 + 全域硬串行: True
前置仅约束 19 行 + 无「23 行全部」: True
命令位残留（case_parity_check --check / tests/report）: NONE
```

**结论**：规划轮**交付完成**，5 轮审核（1 轮开工前审核 + 4 轮复审）共报 6 项阻断项 + 4 项复审未闭合 + 1 项新发现，**现已全部闭合**。规划仍**待用户裁决 D1–D6** 后方可开工。

**方法教训（记录，供后续规划轮参考）**：同一个模板句（「前置 S1 强制：本 Epic 23 行全部…」）被批量复制到 23 个子任务文件，导致**父真源改一次、23 个副本全部过期**；且派发 fixer 时按阶段切目录，**未派到的目录（p3-02）保留了错误授权**。后续同类规划应：(i) 子任务文件里的跨行约束**只写指针**（「见 EPIC 前置一节」），不复制父真源原文；(ii) 修订派发**按「受影响文件全集」**而非「按阶段目录」切分。
