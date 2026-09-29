# PROGRESS：20260929-assembly-layer-rework

- 任务编号：20260929-assembly-layer-rework
- 形态：epic
- 状态：**规划中**（本轮只交付规划，不实施代码）
- 真源：本目录 `SUBTASKS.csv`（7 行，全部 `TODO`）
- 原始需求：见 `EPIC.md` 的「状态与原始需求」（R1–R5 为用户原文要点）

## 恢复块（冷启动从这里读）

1. `任务:` 让物理装配执行层符合工程直觉——总成文件驱动、显式接口配对、统一轮端、试验台非侵入、对称可选（5 条痛点的改造计划）
2. `形态:` epic
3. `进度:` 0/7 子任务完成（01 已开工，5 步中 3 步 DONE：锚点复核、3 轴负例、门禁与数值门实测值；01 的 7 组合快照待做）；Epic 结构已齐（EPIC + SUBTASKS + 7 个子任务目录，每个含 SPEC/TODO/PROGRESS/raw）
4. `当前:` 规划轮已交付，**D1–D7 已全部裁决**（D1/D3 按建议、D2 为用户给出的第三方案：文件层统一 + 装配期刚性凝结、不重录 kc_baseline、D4–D7 按建议采纳）
5. `文件:` `.codex-tasks/20260929-assembly-layer-rework/EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`
6. `下一步:` 从 `tasks/20260929-01-freeze/` 开工（先落现状快照与负例证据，再动生产代码）

## 2026-09-29 第一轮：规划交付

**输入**：用户一份架构评审（5 条痛点 + 各自改造建议），要求「根据痛点和改造建议制定改造计划」，明确只规划、不实施代码。

**做法**：先核事实再写计划。派两个只读 explorer 分别核实痛点 1-2 与 3-5 的代码锚点，主代理复核三个承重锚点（`authoring/solver.py:632 file_axles_from`、`connections/policy.py:202-217` 的 `full_vehicle` 规则、`connections/matcher.py:102 match_requirements` 及其调用点 `composition.py:258/347`），再据此写 `EPIC.md` 与 `SUBTASKS.csv`。

**核出的三处与用户陈述不一致之处（计划按代码现状写）**：

1. 痛点 3 说整车「再重新挂载轮胎」——实际整车**不挂 `VerticalTireElement`**：整车轮胎走 native tire ABI（`preparation/vehicle_dynamic.py:1009-1025`），且 `:900-903` 明确拒绝 `VerticalTireElement`。真正的不对称是「同一种物理在两条路径上用两种表示」，而不是「删了又建同一个力元」。这反而支持用户的建议：统一**实体**，把「激活哪种受力」交给读数。
2. 痛点 2 的匹配机制**已经存在**：`match_requirements` 已有 explicit 映射、歧义检测与 required/optional 语义，且已用于试验台绑定与轮心需求；缺的是把它用在跨子系统插接上。所以改造是「扩展用途 + 文档配对段」，不是新建通讯器体系。
3. 痛点 4 的镜像**不是文档开关**：`subsystem.schema.json`/`template.schema.json` 无 `mirror` 字段，镜像是纯代码行为；且当前契约是「文件只写一侧、装配建两侧」。

**核出的两处与既有裁决/测试的冲突（已登记为待裁决 D2/D3）**：

- 痛点 3 要把单轴侧车轮体收回车轮子系统，与 20260922 Epic 的裁决 **D9**（单轴侧车轮归悬架试验台）冲突。
- 痛点 5 要试验台不再改零件所有权，与既有测试 `tests/subsystems/test_rig_link.py:172-199`（`test_the_tire_moves_to_the_bench_wheel` 断言「轮胎必须被改 owner」）冲突。

**起点事实（本轮实测，即 01 的输入）**：快速集 1015 passed / 1 xfailed；`tests/cases` 100 passed；`tests/architecture` 147 passed；`tests/adams` 160 passed / 47 skipped（环境跳过）；kernel+contracts 60 passed；ruff/ty/三个架构门/`check_composable_release` 全绿；数值门 3/3 绿（其中 `vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 经用户授权于 2026-09-29 重录）。详见 `EPIC.md` 的 F8。

**交付物**：

- `EPIC.md`：原始需求（R1–R5 原文要点）、待裁决 D1–D7、Goal G1–G6、Non-Goals、事实与修正 F1–F9、目标结构、子任务分解与写范围约束、冻结约束、逐子任务验证协议、Done-When（含端到端 (a)-(g)）、风险与回退、目录与命名。
- `SUBTASKS.csv`：7 个子任务（01 冻结现状 → 02 显式接口配对 → 03 通用装配引擎 → 04 轮端统一 → 05 试验台非侵入 → 06 可选对称 → 07 终局验收），每行带 `acceptance_criteria`、`validation_command`、写范围与依赖。
- 本文件。

**未做（本轮边界）**：未写任何生产代码；未创建 `tasks/<child>/` 目录（子任务的 `SPEC.md`/`TODO.csv`/`PROGRESS.md` 在各自开工时按其 `acceptance_criteria` 展开，避免用规划文本冒充实施记录）；未重录任何基线。

**独立审核（已执行）**：按仓库约定，EPIC 与子任务在开工前必须由只读子代理独立审核，审核输入含用户原始需求原文（`pasted-1d3a60ca-...txt`）。本轮派出 `code-reviewer`（只读）逐条回答「全部子任务 DONE 后能否真正达成 Goal、Goal 本身是否偏离用户需求」，结论如下，均已在本轮修入计划：

| 审核项 | 结论 | 本轮修订 |
|---|---|---|
| A1 目标覆盖 | 有缺口：R1 原文的「单轮/三轮试验平台」在验收里无人承接；「拖挂铰接车」被降为可选 | G1 判据加单轮/三轮最小装配用例；D6 改为「3 轴、拖挂铰接、单轮三轮都进硬判据」，拖挂由同机制（端口配对 + 现有副类型的铰接副）承接；Done-When (g) 由「可选」改为必验 |
| A2 反向多余 | 有一项：R3 未要求迁移轮胎质量，计划却把「质量从轮端 body 迁到轮胎」写成必交付 | 04 的 (d) 改为「质量/质心/惯量口径不变，不得迁移所有权；若确需迁移先经 D7 单独裁决」 |
| A3 可判定性 | 有缺口：`validation_command` 多为通用 `pytest`，不能证明快照/锚点/负例/登记已落盘；且 `kc_parity_check --check` 不带 `--actual-dir` 是自比较 | 每行命令改为指名测试路径 + 产物存在性检查；07 列出终局命令清单；「数值门为独立项」写明 kc_parity 的自比较性质与正确的对标做法 |
| A4 端到端验收 | 有缺口：(d) 只比实体集合与名字，而 `_reown_tires` 恰好「名字不变、所有权变」，无法证明 G4 | (d) 与 05 的 (a) 改为逐项比较所有权/参数/几何值；(b) 与 06 的 (a) 改为比较点坐标与约束端点，并加「独立文件不被二次镜像」断言 |
| A5 集成缝隙 | 有缺口：契约 schema（`suspension_contracts/.../assembly.schema.json`）禁止新增配对字段、不接受 `middle`、整车轮数限定为四，会在装配逻辑之前挡住 02/03；`vehicle_assembly.py` 的轮胎过滤与 wheel 文件读取链无人认领；`supplies_wheels` 改义后旧 rig 声明如何解释未定义 | F2 补 schema 锚点；「并行与写范围约束」新增「契约 schema 是 02/03 共同前置」；02 写范围加 schema 配对段，03 加放置与轮数段；04 写范围加 `vehicle_assembly.py`（轮胎过滤段）与 wheel 文件读取链；05 加 (e) 旧 rig 声明的解释规则；06 加 (e) 单侧文件的坐标系/镜像方向/命名契约 |
| A6 依赖与顺序 | 有缺口：06 只依赖 03，调度上可与 04/05 并发改写 `si_assembly.py`，与 EPIC 声明矛盾 | 06 的 `depends_on` 改为 `05`；EPIC 的并行约束同步（已用脚本校验 7 行无环） |
| B 事实核对 | 通过；F4 措辞需收紧 | F4 补「`composition.py:302-305` 只产出 bindings 记录、不据此建立物理插接」，因此 02 的落点是「写进文档 + 把 bindings 转成真实运动副/衬套」 |
| C 风险 | 有缺口：G6「零回归」与 04/05 的实体变化在原理上冲突，且「质量与质心相同」不足以证明等价 | G6 改为「03 之前逐位不变；04/05 允许受控变化」；D7 与冻结约束新增「重录前必须给出独立于结果字节的物理等价判据（自由度/约束行数、惯量、轮心与接触点几何、轮胎力路径）并列出允许差异」 |

审核未提出反对意见的部分：R1–R5 与 G1–G6 的对应关系、`SUBTASKS.csv` 的依赖图（除 A6）、`01 冻结现状` 的做法，以及 F2/F5/F7 三条事实判断（审核逐条复核为属实）。

## 2026-09-29 第二轮：D1–D7 裁决落表，计划按 D2 调整

**用户裁决（原文要点）**：

- **D1**：**严禁原地改 `VehicleModel` schema**；新增**文档驱动装配器**（直接消费 `AssemblyDocument`）作为未来唯一装配主管道，`VehicleModel` 降级为向下兼容适配器（绞杀者模式）。
- **D2**：**不反转 D9、严禁重录 `kc_baseline`**；采用**文件层统一 + 装配期刚性凝结**——单轴与整车可引用同一份 `wheel.subsystem.json`，装配单轴 K/C 试验台时检测轮心驱动工况，把车轮刚体与轮毂刚体在内存中刚性凝结，自由度拓扑与既有 K/C 基线逐位不变。依据：AGENTS.md 第 7 节禁止重录 `kc_baseline`；Adams 单轴装配同样不带独立车轮刚体。
- **D3**：**确认反转**——删除 `rig_link.py` 的所有权篡改（`_reown_tires`），断言改为验证被测总成不可变。
- **D4–D7**：按已沟通的建议无异议推进。

**据此对计划的调整（已落盘）**：

| 位置 | 调整 |
|---|---|
| `EPIC.md` 裁决登记表 | D1–D7 全部由「待裁决」改为「已裁决/已采纳」，逐条写入用户裁决原文要点与依据 |
| `EPIC.md` G3 | 由「单轴与整车产出同一套实体」改为「**文件统一 + 求解拓扑按读数凝结**」：单轴 K/C 期刚性凝结车轮与轮毂，整车侧保持独立车轮刚体与 native 轮胎 |
| `EPIC.md` G6 | 由「04/05 允许改变产物」收紧为「**04 不得改变任何既有产物**（凝结必须让自由度拓扑与 `kc_baseline` 逐位不变）；05 允许变化但须登记 + 物理等价判据」 |
| `EPIC.md` F5b（新增） | 记下 D2 的两个现成先例：`_merge_fixed_wheel`（`vehicle_parts.py:572-610`）与 `_fuse_welded_bodies`（`:133+`，`SUSPENSION_MULTIBODY_CONDENSE_WELDS=1`）；并点名与 20260921 Epic A3（整车侧"不凝聚、weld 交内核 fixed 关节"）的关系，避免两处"凝聚"语义混淆 |
| `EPIC.md` 04 验证协议 | 改为：两侧读同一份 wheel 文件；凝结的硬门是 `kc_baseline` 逐位不变，且必须用 `kc_native_probe.py` + `kc_native_c_probe.py` 生成 actual 后带 `--actual-dir` 比对（不是自比较） |
| `EPIC.md` 冻结约束 / 风险 / Done-When (c) | 同步：`kc_baseline` 不得重录；04 的风险由"数值必然变"改为"凝结若不等价必须退回" |
| `SUBTASKS.csv` 04 / 07 | 04 的验收与命令改为凝结 + K/C 逐位比对；07 增加"`kc_baseline` 逐位未变" |

**未变**：01/02/03/05/06 的行、依赖图与写范围划分不变；D3 已确认的反转仍在 05 承接（与 04 的凝结互不影响：04 管轮端生命周期，05 管试验台不改写被测实体）。

## 下一步

1. 从 `tasks/20260929-01-freeze/` 开工：落 7 组合装配快照、5 条痛点锚点复核、3 轴负例报错原文、门禁与数值门实测值。
2. 01 完成后按 `SUBTASKS.csv` 顺序推进 02 → 03 → 04 → 05 → 06 → 07；每步落地后重跑 `just check-fast` 与数值门，结构改动后加跑 `tests/architecture`。

## 2026-09-29 第三轮：子任务目录与协议文件落盘（用户指出后补齐）

**触发**：用户指出"子任务文件夹和 SPEC 都没有建立"。核实：上一轮我把"子任务 SPEC 在开工时再写"写进了 EPIC 的目录与命名一节——**这是判断失误**：taskmaster 的 Epic 形态要求子任务目录与协议文件随 Epic 落盘，否则 `SUBTASKS.csv` 的 `task_dir` 指向不存在的路径，Epic 既不可执行也无法冷启动恢复。

**本轮补齐**：

- 落盘 7 个子任务目录（`tasks/20260929-0{1..7}-*/`），每个含 `SPEC.md` / `TODO.csv` / `PROGRESS.md` / `raw/`（共 21 个协议文件）。
- 01 由主代理亲自写（已在执行中）；02–07 由 3 个 `fixer` 并行写（写范围按目录级切分，互不重叠），任务书要求：判据照抄 `SUBTASKS.csv` 对应行、只许引用 `EPIC.md` 已有事实（F1–F9）、每个 SPEC 必须写 `snapshot.py --check` 判据、03 写清 D1、04 写清 D2、05 写清 D3、06 写清 D5、07 写清独立终局验收与 K/C 对标口径。
- 主代理复核（抽查，非重读）：21 个文件齐备、`raw/` 除 01 外为空（无伪造证据）；7 个 `TODO.csv` 用 `csv` 解析全部 8 列且表头与 01 一致，02–07 全 `TODO`；6 个 SPEC 均含 `snapshot.py --check`（3–7 处）；D1/D2/D3/D5 的裁决措辞分别落在 03/04/05/06；抽查锚点 `rigs/rig.py:74 class RigSpec`、`vehicle_parts.py:133 _fuse_welded_bodies`、`:572 _merge_fixed_wheel` 均存在；父 EPIC/SUBTASKS/PROGRESS 未被 fixer 改动。
- 修掉 fixer 点出的一处父表归属歧义：`SUBTASKS.csv` 第 3 行（02）原写"装配路径 chassis/ground 规则全部消除"，但其中三处锚点在 03/04 的写范围内 → 改为"本行自有路径消除；全装配路径的终局 grep 判据在 03/04 收口后由 07 复验"，并在 notes 里点名三处锚点的归属。
- 同步 EPIC 的「目录与命名」末段：删去"SPEC 开工时再写"的说法，改为"随 Epic 落盘全部 7 个子任务目录"。

**01 的进展（本会话实做，非规划）**：`raw/painpoint_anchors.md`（5 条痛点锚点复核，含主代理亲自复核的 6 个承重锚点）、`raw/triaxle_refusal.md`（两层拒绝的报错原文）、`raw/baseline_commands.md`（门禁与数值门实测值 + 两项经授权重录基线的前后值）已落盘；`TODO.csv` 的 #2/#3/#4 置 DONE，#1（7 组合快照）与 #5 仍为 TODO，恢复块已写明下一步是写 `snapshot.py`（口径已确认：`rigs/rig.py:114 RIGS` 恰有 7 项）。
