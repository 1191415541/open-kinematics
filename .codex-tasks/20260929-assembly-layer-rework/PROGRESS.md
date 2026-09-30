# PROGRESS：20260929-assembly-layer-rework

- 任务编号：20260929-assembly-layer-rework
- 形态：epic
- 状态：**执行中**（计划已过开工前独立审核，第二轮 7 项阻断项已修入计划；01 已完成，02 起待续）
- 真源：本目录 `SUBTASKS.csv`（7 行；01 为 `DONE`，02–07 为 `TODO`）
- 原始需求：见 `EPIC.md` 的「状态与原始需求」（R1–R5 为用户原文要点）

## 恢复块（冷启动从这里读）

1. `任务:` 让物理装配执行层符合工程直觉——总成文件驱动、显式接口配对、统一轮端、试验台非侵入、对称可选（5 条痛点的改造计划）
2. `形态:` epic
3. `进度:` **1/7** 子任务完成（01 完成：5 步全 DONE——痛点锚点复核、3 轴负例、门禁与数值门实测值、7 组合装配产物快照 + `snapshot.py`（含登记口径）、快照判据接入 02–07 的 SPEC）；Epic 结构已齐（EPIC + SUBTASKS + 7 个子任务目录，每个含 SPEC/TODO/PROGRESS/raw）
4. `当前:` 开工前独立审核已走完两轮：第一轮 7 项阻断项全部修入；第二轮复核 6 项闭合、1 项（登记放行过宽）已重构为「四项精确匹配 + 归属校验」并实测，另补了 03 的 `authoring/documents.py` 放置段与 schema 整车 `required` 段、01 的按试验台产物键与覆盖边界
5. `文件:` `.codex-tasks/20260929-assembly-layer-rework/EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`
6. `下一步:` 从 `tasks/20260929-02-port-binding/` 开工（02 → 03 → 04 → 05 → 06 → 07；每步落地后重跑 `just check-fast` 与数值门；改结构后加跑 `tests/architecture`）

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

## 2026-09-29 第四轮：开工前独立审核（第二轮）与 7 项阻断项修订，01 完成

**审核**：按仓库约定派只读 `code-reviewer` 做开工前审核，输入含用户原始需求原文（R1–R5 与建议 P1–P5）。结论：7 项阻断项 + 1 项可选改进（另附事实核对表：12 处锚点抽查，2 处轻微行号漂移、1 处归属写错，均不影响定位）。阻断项与处置：

| 审核阻断项 | 处置 | 落在哪 |
|---|---|---|
| 1. 03 验收含「单轮/三轮」但单侧机制属 06，03 在 06 之前无法完成 | **用户裁决**：把这两条最小用例移到 06（G1 判据 (b) 原文不变，01 只做 N 轴与放置规则放开） | `SUBTASKS.csv` 03/06 行、`EPIC.md` 验证协议 03(f)/06(f)、03/06 的 SPEC 与 TODO |
| 2. G1(a) 要求 `preparation/` 零 `front_axle`/`rear_axle` 命中，但 D1 要保留 `VehicleModel` 适配器，两处 7 个访问点无人承接 | **用户裁决：保持 G1(a) 字面口径**；03 承接整个 `preparation/` 的图谱化，适配器落在 `preparation/` 之外，新增硬门「`dynamic_hash_sentinel --check` 26 artifact 逐字节不变」 | `EPIC.md` 段级划分/验证协议 03(b)(h)/风险、`SUBTASKS.csv` 03 行、03/04 的 SPEC 与 TODO |
| 3. 拖挂铰接（D6 必验）无子任务承接，且 `policy.py` 与 schema `:46` 只允许一个车身 | **用户裁决**：并入 03（放开单车身限制 + 用现有副类型建「牵引车 + 挂车」最小用例），实测不可行才登记内核范围并提请裁决 | `EPIC.md` Non-Goals/Done-When (g)、`SUBTASKS.csv` 03 行、03/07 的 SPEC 与 TODO |
| 4. 06 要求 `SIDES` 消费点全部收口却排除 `wheel.py`/`element_build.py`，并把「必要时改归 04」（04 已在前） | 这两个文件的**单侧展开段**改归 06，与 04 的轮端实体段分段串行 | `EPIC.md` 段级划分/验证协议 06(d)、`SUBTASKS.csv` 06 行、06 的 SPEC 与 TODO |
| 5. 04 的凝结判据「本次不产出车轮刚体」在文件层统一后失效 | **用户裁决**：改按试验台/请求声明（`supplies_wheels=True` 的单轴侧）判定；硬门扩为 `kc_baseline` **与** `dynamic_hash_baseline` 轴侧用例逐字节不变 | `EPIC.md` D2 行/验证协议 04(c)、`SUBTASKS.csv` 04 行、04 的 SPEC 与 TODO |
| 6. 05 允许改变产物却要求原始 `--check` 通过，会把已批准的变更误判为失败 | 01 交付的 `snapshot.py --check` 定为「未变化部分逐项相等 + 已登记差异」口径，登记处 `raw/approved_deltas.json`；未登记差异非零退出；**只有 05 可登记** | `EPIC.md` G6/验证协议 01(e)(05)(d)/07、`SUBTASKS.csv` 01/05/06/07 行、01/05/06/07 的 SPEC 与 TODO |
| 7. 01 的 `_meta` 要存生成时间又要求两次逐字节一致 | 快照载荷不含时间（时间写进 `raw/snapshot_notes.md`）；序列化排序键 + 集合转排序列表 | 01 的 SPEC、`snapshot.py`、`raw/snapshot_notes.md` |

**可选改进（已修）**：`EPIC.md` F1（两轴条目实际在 `:185-186`）、F2（schema 锚点更正为 `:18`/`:23`/`:46`/`:50`，并补「整车 `required` 只列一个 `chassis`」）、F6（`_SIDES` 今天只在 `si_assembly.py:56`，`suspension.py` 已无此符号；固定双侧展开在 `wheel.py:114-116` 与 `element_build.py:206/213`）、F7（实体开关定义在 `si_assembly.py:521`，应用点 `:467-482` 不变）；02 的 SPEC 措辞由「全路径消除」统一为「本行自有路径」。

**用户裁决（本轮新增两问）**：G1(a) 保持字面口径（选项 2）；凝结判据按声明判定（选项 1）；拖挂并入 03（选项 1）；单轮/三轮移入 06（选项 1）。

**01 完成（本会话实做）**：
- 新增 `tasks/20260929-01-freeze/snapshot.py`（生成与判定脚本）与 `raw/assembly_snapshot.json`（323,644 字节）、`raw/approved_deltas.json`（初始 `[]`）、`raw/snapshot_notes.md`（生成时间与实测输出）。
- 覆盖：`RIGS` 恰 7 项，映射到 3 个装配产物（`axle_K` 15 体/18 约束/0 力元、`axle_C` 15/14/16、`vehicle` 29/36/0）。
- 可重复性：两次运行 sha256 同为 `f6d3c3f2…`；过程中实测发现并修掉一处不确定性（`frozenset` 走 `str()` 兜底，哈希序随进程变化 → 改为排序列表）。
- 判据实测：未登记差异 → 退出 1；同一差异登记后 → 退出 0；还原后 → 退出 0。
- 事实修正（写进 SPEC 的风险项）：**K 读数不含力元**属 F9 分层（K 的受力列是 `ideal_constraints`），故非空判据按「受力列整体」判定，不要求 K 有 `elements`。

**未做（本轮边界）**：02–07 的实现一行未动；未重录任何基线；未把 02 起置为 `IN_PROGRESS`（等审核复核通过）。

## 2026-09-29 第五轮：第二轮独立审核复核，3 项缺口修订

**复核结论**（同一 `code-reviewer` 续审）：第一轮 7 项阻断项中 **6 项闭合**；**阻断项 6 未闭合**（登记放行过宽）；另发现 **2 处 03 的写范围缺口**与 **01 的覆盖不足**（后者写入 01 交付物判定为「不满足」）。

| 复核发现 | 处置 |
|---|---|
| 阻断项 6 未闭合：登记只比产品名与指针前缀，登记一条 `/elements` 即可放行其下任意变化，也没有归属校验 | `snapshot.py` 改为**四项精确匹配**（`product`/`pointer`/`before`/`after`，无前缀规则）+ 登记项必带 `reason`/`registered_by`/`evidence` 且 `registered_by` 必须为 05；缺字段或归属不符 → 退出 4。实测五条路径：未登记 1、精确登记 0、归属非 05 为 4、`before` 不符 1、缺字段 4 |
| 01 覆盖不足：7 个 rig 只映射到 3 个产物，两个供轮试验台共用 `kc_quasi_static` 的产物，无法证明每个试验台的挂接产物未变，撑不起 05 的逐组合登记门 | 产物键改为按试验台：`axle_<模式>@<试验台>`（`kc_quasi_static` 与 `axle_dynamic` 各自成键，都含夹具刚体 `wheel_carrier_L/R`）+ 整车 `vehicle`；每个 rig 新增 `bench_bound`/`bench_binding`，`_meta.coverage_boundary` 写明整车侧 5 个 rig 的试验台绑定在读数层、不在快照内（这 5 个 rig 必须由 05 自己的运行时对照证明非侵入） |
| 01 的判据文字与实现不符：SPEC/TODO 原写「每个组合的 `elements` 非空」，而 K 读数按 F9 本就没有力元 | 判据改为「`bodies`/`points`/`constraints` 非空 且受力列整体（`constraints + ideal_constraints + elements`）非空」，SPEC/TODO/`raw/snapshot_notes.md` 三处同步 |
| 03 的 `middle` 文档入口无人授权：`authoring/documents.py:139` 用 `PLACEMENT_ROLES`（`:62-64`）校验放置，只放开契约 schema 仍会在文档读取层被拒 | 03 写范围补 `authoring/documents.py`（**放置与角色枚举段**）；分段归属写入 `EPIC.md`（02 配对段 / 03 放置与角色枚举段 / 06 可选对称声明段，串行）与 `SUBTASKS.csv` 02/03 的 notes |
| 03 的 schema 段与拖挂 Goal 冲突：约束写「只动放置与轮数两段」，但拖挂需要放开整车 `required` 的多车身形态 | `EPIC.md` 契约前置条与 03 行改为「放置段 + 轮数段 + 整车 `required`（多车身）段」 |
| 05/07 的登记口径与 01 的过期恢复信息（「01 快照尚未生成」） | 05/07 的 SPEC/TODO 补四项精确匹配口径、归属校验与覆盖边界；02/06/07 的过期说法改为「01 已交付」（已派 fixer 同步） |

**实测（本会话）**：两次生成 sha256 同为 `b58d35acd93560fd52046228856307ccc0cd97655b37b69dcaad86e852b24566`（产物 539,636 字节）；`--check` 五条路径全部按设计；快照还原后 `--check` 退出 0；`just check-fast` 全绿（快速集 1015 passed / 1 xfailed、kernel+contracts 60、ruff/ty/三个架构门绿）；`git diff --check` 干净。

**口径变化 ≠ 产物变化**：快照因口径修订重新生成（`f6d3c3f2…` → `b58d35ac…`），`raw/approved_deltas.json` 仍为 `[]`——没有任何既有产物被登记为「已改变」。

**待办**：fixer 同步的子任务协议复核通过后，把 02 置 `IN_PROGRESS` 开工（02 → 03 → 04 → 05 → 06 → 07）。

## 2026-09-29 第六轮：第三轮复核与用户指示（不跑测试）

**复核结论**（同一 `code-reviewer` 续审）：**阻断项 6 已闭合**（四项精确匹配 + 归属校验 + 非对象项守卫，`--check` 返回 4 而不是放过）；**「01 覆盖不足」已解决**——两个供轮试验台各自成键且产物确含夹具实体 `wheel_carrier_L/R`，整车侧 5 个 rig 的覆盖边界在脚本、快照 `_meta`、`raw/snapshot_notes.md`、05 的 SPEC 四处说法一致；**03 的两处写范围缺口已闭合**；`depends_on` 无环、G1–G6 与终局验收仍有人承接。

**本轮又修 3 项（复核新报）**：

| 发现 | 处置 |
|---|---|
| `snapshot.py` 的登记文件若含非对象项（`null`、数字）会抛异常 | 加守卫：非对象项直接记为登记问题并返回 **4**（实测 `[null]` 与 `[7]` 均退出 4） |
| `03/SPEC.md` 的 `vehicle_parts.py` 写范围只写「前缀段」，而同文件又要求 03 删 `:172-173`/`:414` 的名字规则 | 写范围改为「**前缀段 + 名字规则段**（`:172-173` 的焊合根命名、`:414` 的 `referenced` 初值）」 |
| `03/SPEC.md` 要求兼容开关「在 03 **之前**」删除，却又把三处删除全交给 03，与 02 不改这些文件相冲突 | 时点改为「**03 完成之前**」并写明「02 明确不改这三个文件，删除只能落在本行」 |

**可选改进也已修**：供轮试验台若没有自有产物，`snapshot.py` 改为**显式报错**（避免将来新增供轮 rig 被静默记成共享的 `vehicle` 产物）；`SUBTASKS.csv` 的 01 行与 01 的 SPEC 里过期的文件大小/哈希（323,644 / `f6d3c3f2…`）更新为 **539,636 / `b58d35ac…`**。

**01 的状态与验证偏差**：证据与判据齐备（两次生成 sha256 一致；`--check` 七条路径实测：未登记 1、精确登记 0、归属非 05 为 4、`before` 不符 1、缺字段 4、非对象项 4、还原后 0）。复核要求补跑 `validation_command` 里的**全量 pytest 腿**；**用户本轮明确指示「不要跑测试」，故未执行**（曾启动后按指示停止，未采用其部分结果）。该偏差已登记在 `SUBTASKS.csv` 的 01 行、`tasks/20260929-01-freeze/TODO.csv` 第 1 行与 01 的 `PROGRESS.md`；全量留到 07 收尾时按用户指示决定。本会话实际跑过的检查：`just check-fast`（快速集 1015 passed / 1 xfailed、kernel+contracts 60、ruff/ty/三个架构门全绿）、`ruff check .codex-tasks/...`、`snapshot.py --check`（退出 0）、`git diff --check`（干净）。

**下一步**：02 开工（`tasks/20260929-02-port-binding/`），02 → 03 → 04 → 05 → 06 → 07；每步落地后跑 `just check-fast` 与数值门，改结构后加跑 `tests/architecture`。

## 2026-09-29 第七轮：02 落地（显式接口配对）

**状态**：`SUBTASKS.csv` 的 02 行置 **DONE**（2026-09-29），其 6 条 TODO 全部 DONE；03 起仍为 `TODO`。

**交付**（写范围与判据见 02 的 `SPEC.md` 与 `raw/`）：

- 新增 `connections/links.py`：`LinkSpec`（配方：role + kind ∈ weld/revolute/bushing + 需求方一端）+ `build_links`（把 bindings 变成 `Connection` + 对应 `WeldJoint`/`RevoluteJoint`/`BushingElement`）+ `explicit_bindings_from_pairings`（把文件里的端口名渲染成端口 id，一处完成）。**没有配方时产出为空**。
- `subsystems/composition.py`：`SubsystemContribution.links`（默认空）+ 匹配前接文档配对、匹配后 `build_links` 并入 fragment，生成的 `LinkRow` 随 `generated["links"]` 暴露（可审计）。
- `subsystems/types.py`：`AssemblyRequest.pairings`（默认 `{}`，不进任何 `model_dump`）。
- `assembly.schema.json`：条目层可选 `pairings` + `$defs/pairing`（两端必填、不许多余字段）；`authoring/documents.py` 读进 `AssemblyEntry.pairings`，同一 `requirement_role` 出现两次即报错点名。
- 测试：`tests/connections/test_links.py`（11 条）、`tests/authoring/test_assembly_pairings.py`（4 条）、`packages/suspension_contracts/tests/test_assembly_pairings.py`（5 条）。

**验证（真实数字）**：`just check-fast` 退出 0（快速集 **1030 passed / 1 xfailed**，比 01 收尾时的 1015 多出本轮的 15 条；kernel **33**、contracts **32**（27→32）；ruff/ty/三个架构门全绿）；`tests/connections + authoring + subsystems` **241 passed**；`snapshot.py --check` **零差异**（退出 0）。

**过程登记（不掩盖）**：

1. 两次派发的 fixer 都不可用：第一次 79 次调用**只设计不动手**、交付为空；第二次动了 `links.py`/`composition.py`/`types.py`/schema，却把 `compose_simulation_assembly` 的 `root_kind` 形参**误删**（`si_assembly_for_axle` 调用失败、`snapshot.py --check` 退出 1、`tests/subsystems` 收集失败），且没写测试与证据。主代理接管：修回形参、把配对接进 `AssemblyDocument`、补三个测试文件与三份 `raw/` 证据、跑门禁并登记（`raw/snapshot_diff.md` 末尾写明「曾经坏过、已修好并复验」）。
2. 父表 02 行的 `validation_command` 原文把 `packages/suspension_contracts/tests` 与 multibody 的三个目录写在**同一次** `pytest` 调用里，实测会改 rootdir 并使 multibody 侧的 `from tests.benchmark_fixture import ...` 失败（`AGENTS.md` 第 1 节记载的坑）。**已把父表命令改为两次调用**，本轮按修正后的命令实测。
3. 全量 `pytest packages/suspension_multibody/tests` 未跑（用户指示不跑）；01 的同一偏差仍登记在 01 行与 01 的 PROGRESS。

**未收口项（交 03）**：`authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES` 仍按名字放开引用校验（校验放宽、不改名），本行登记为「保留 + 03 改为按角色/端口推导」，见 `tasks/20260929-02-port-binding/raw/removal_register.md` 的 B 节。

**下一步**：03 开工（通用装配引擎 + 文档驱动入口 + N 轴/N 车身形状规则 + `preparation/` 图谱化，硬门是 01 快照逐项一致与 `dynamic_hash_sentinel --check` 26 artifact 逐字节不变）。

## 2026-09-29 第八轮：03a 落地（条目清单引擎 + 适配器，零回归）

**状态**：03 仍为 `IN_PROGRESS`（分期 03a 完成，03b/03c/03d 未完）；04–07 仍为 `TODO`。

**03a 交付**：

- 新增 `subsystems/assembler.py`（302 行）：`AxleEntry`（`placement`/`prefix`/`axle`/`replace_bodies`/`wheels`）+ `compose_entries_runtime(entries, *, chassis_name, mode, request, chassis_body)` —— 逐条 `si_assembly_for_axle`，按条目前缀与 `replace_bodies` 合并（点/约束/理想约束/力元/连接同步重命名），轮按条目挂接，末尾仍走 `_condense_welded_bodies` + `_drop_isolated_bodies`。**实测该文件里没有任何轴名/前缀/车身名字面量**（唯一命中是 docstring 里描述被删掉的旧规则）。
- `subsystems/vehicle_assembly.py`（310 → 198 行）：`compose_vehicle_runtime` **名字与签名不变**，改为适配器——把 `VehicleModel` 转成两条 `AxleEntry`，旧的隐式 `body_map`（名字等于 `chassis`/`ground` 就换成车身名）变成**显式声明** `replace_bodies={"chassis": …, "ground": …}`；轮按 `"<placement>_"` 前缀分组（该启发式留在适配器，03b 由文档声明取代）。
- 新增 `tests/subsystems/test_vehicle_assembler.py`：引擎不依赖 `VehicleModel`（`placement` = `front`/`middle`、前缀 `front_`/`mid_`，断言前缀与 `replace_bodies` 生效）；适配器与「手工构造条目再调引擎」的产物逐项相同（体/质量/点/约束/理想约束/力元/连接的名字与顺序、四个车轮表、总质量，K 与 C 各一遍）。
- `tasks/20260929-03-universal-assembler/raw/adapter_parity.md` 与 03 的 `PROGRESS.md`（03a 节 + 03b 交接）。

**独立验收（主代理自己跑的，不采信子代理自报）**：`just check-fast` **退出 0**（快速集 **1033 passed / 1 xfailed**，比 02 的 1030 多出本轮 3 条；kernel 33、contracts 32；ruff/ty 全绿；**三个架构门全绿，含 `legacy_surface_gate`——证明入口没有搬家**）；`tests/subsystems + vehicle + vehicle_assembly + model` **209 passed / 1 xfailed**；`snapshot.py --check` **`OK`，退出 0（零差异）**，`approved_deltas.json` 仍为 `[]`。→ **D1 的装配层零回归硬门在 03a 这一期已成立**（适配器路径与 01 快照逐项一致）。

**04 的写范围随 03a 变更（已同步父表）**：`VerticalTireElement` 的类型过滤从 `vehicle_assembly.py:233-240` 搬到了 `assembler.py`，所以「删除该过滤」这一项由 04 在 `subsystems/assembler.py` 上做，不再是 `vehicle_assembly.py`。

**03b 的已知前置墙**（子代理实测，交 03b 处理）：`middle` 今天被两处挡住——`schema/vehicle.py:46` 的 `WheelSpec.name: Literal["front_left","front_right","rear_left","rear_right"]` 与 `authoring/documents.py:139`/`PLACEMENT_ROLES`；03a 的中轴测试因此暂用 `rear_*` 命名而前缀仍是 `mid_`，放开后改回 `middle_*`。

**一个越界产物（需用户裁定）**：`packages/suspension_multibody/docs/multibody_architecture_evolution.md`（18 KB，2026-09-29 20:39 生成）是子代理在本轮**未受指派**写下的架构评审文档，不在任何子任务的写范围里。主代理没有删它，也没有把它计入任何交付物——留待用户决定去留。

## 2026-09-29 03b 落地（形状规则数据化 + 契约放开 + 三轴装配）

**状态**：03 仍为 `IN_PROGRESS`（03a、03b 完成；03c/03d 未完）；04–07 仍为 `TODO`。

**已做**：

- `connections/policy.py`：`AssemblyRule` 新增 `min_counts`（下限型角色数）并给其余字段默认值；`full_vehicle` 规则不再写死 `suspension: 2` 与 `required_placements={"front","rear"}`，改为「一个车身 / 一个转向 / 一套制动 / 一套驱动 / 至少一个悬架」+ 两条新判据：**同一放置的悬架不得重复**、**每个被声明的轮端都必须被某悬架覆盖**。`_covered_corners` 改为按 `_left`/`_right` 后缀推导端点（不再依赖四角常量 `WHEEL_ENDS` 的成员资格），`any` 的语义在调用处解析为「本装配声明的全部轮端」。
- `assembly.schema.json`：`placement_role` 由枚举改为非空字符串、`vehicle.wheels` 去掉 `4/4` 上下限（`minItems: 1`）、整车 `required` 去掉 `chassis`（为 03d 的多车身让路）；`authoring/documents.py` 的 `PLACEMENT_ROLES` 同步加 `middle`/`third` 及左右；`schema/vehicle.py` 的 `WheelSpec.name` 由四角字面量放宽为非空字符串（**D1 的四角校验仍写在 `VehicleModel._topology`，未放松**）。
- 新增 `tests/subsystems/test_three_axle_assembly.py`（7 条）：三轴装配与逐轴前缀、三轴/两轴形状都接受、中轴轮无人认领被点名拒绝、同放置重复被拒绝、无悬架被拒绝、条目自带轴仍可跑 K/C 读数、以及**边界登记**（三轴装配体本身今天还不能被 K/C 契约读取，见下）。
- 一条旧语义断言按新语义改写：`test_full_vehicle_rejects_two_suspensions_at_one_placement`。
- 证据：`tasks/20260929-03-universal-assembler/raw/policy_and_schema.md`。

**独立验收（主代理实跑）**：`just check-fast` **退出 0**（快速集 **1040 passed / 1 xfailed**，03a 为 1033；kernel 33、contracts 32；ruff/ty 与三个架构门全绿）；`snapshot.py --check` **零差异（退出 0）**；`dynamic_hash_sentinel.py --check` **退出 0**（26 artifacts 与冻结基线逐字节一致）。数值门输出里的 `acceptance exit: 1` 与 9 个 `failed cases` 是脚本文档化的预期（`docs/axle_dynamics_results.md` 记录的 `time_convergence` 失败），哨兵只按哈希漂移判定。

**一处真实缺口（交 03c，已用断言摆明）**：K/C 契约读取装配时仍按「左/右」找轮心——`cases/kc_quasi_static/contract.py:698` 的候选名形如 `<stem>_<side>`，三轴装配会给出每侧三个候选并报 `side L declares more than one wheel centre (...)`。因此**三轴整车目前能装配、能过形状规则、能生成条目级读数文档，但还不能被 K/C 契约整体读取**。`test_the_three_axle_assembly_itself_is_not_yet_readable_by_the_kc_contract` 把这条写进测试，避免绿色套件被读成「三轴已可扫掠」。放开该读者归 03c（同属 `preparation/` 与读数层的图谱化）。

**03c 的前置**：除 `preparation/vehicle_dynamic.py` 的 7 处模型访问与 `:373` 命名规则外，还要处理 `cases/kc_quasi_static/contract.py` 的轮心查找（按装配的轮表而非按侧名），以及 `vehicle_parts.py` 的 `:172-173`/`:414` 名字规则段。

## 2026-09-29 03c/03d 落地（preparation 图谱化 + 三轴/拖挂真跑）→ 03 DONE

**状态**：01、02、03 均为 `DONE`；04–07 为 `TODO`。

**已做（03c）**：`preparation/vehicle_dynamic.py` 的 7 处模型访问（`_select_assembly_mode` 原 `:188`、
`_validate_steering_topology` 原 `:207`/`:210`、`_validate_units` 原 `:341`、
`_build_static_rotation_gauges` 原 `:368-369`）改为读 `VehicleFacts`；facts 由
`subsystems/assembler.py::vehicle_facts(entries)` 从**条目清单**派生（含 `axle_units` 与已按最终命名给出的
`static_rotation_axes`）；`VehicleModel → AxleEntry` 的适配器搬到新增的
`subsystems/vehicle_model_adapter.py`（**不在**被 grep 判据覆盖的 `vehicle_assembly.py` 内），
入口 `compose_vehicle_runtime` 名字与签名不变；`EPIC.md` F4 的三处 `"chassis"`/`"ground"` 字符串改写规则
全部消除（`vehicle_dynamic.py:373` 删除，`vehicle_parts.py:172-173` 与 `:414` 改读新增的
`VehicleRuntime.chassis_name` 字段）。

**已做（03d）**：新增 `ArticulationSpec`（条目之间的显式铰接，仅 `weld`/`revolute` 两型——即仓库已在建的
副类型，其余在构造时拒绝并说明「新副类型是内核问题」）与
`compose_entries_runtime(..., articulations=..., unit_bodies=...)`；条目之外的整车级刚体由 `unit_bodies`
显式给出；新增 `runtime_for_study` 把整车 runtime 重投影为 study 读取的那一面。

**验证（主代理实跑，退出码全 0）**：`just check-fast` → 快速集 **1047 passed / 1 xfailed**（03b 为 1040，
增量 = 本两期新增 7 个用例）、kernel 33、contracts 32，三个架构门全绿；`snapshot.py --check`
**零差异**（`approved_deltas.json` 仍为 `[]`）；`dynamic_hash_sentinel.py --check` 26 artifact
**逐字节一致**（未重录任何基线）；`case_parity_check` 8 families accepted；`kc_perf_gate --check` 在预算内；
`ruff` / `ty` 全通过。无新增 skip/xfail。

**两处 grep 硬判据**（`EPIC.md` G1(a) 字面口径 = 装配层三文件 + 整个 `preparation/`）：
`front_axle|rear_axle` 与 `"chassis"|"ground"` 在该范围内**均零命中**（实测 exit=1）。

**真跑用例**：`tests/subsystems/test_three_axle_assembly.py`（8 条，含三轴整车装配 + 动态 study 解算完成）、
`tests/subsystems/test_articulated_tow.py`（6 条，含地面—牵引车 weld + 牵引车—挂车 revolute 装配后解算完成）。
实测中夹具带来的两点约束已写进用例注释：轴夹具的身/轮端不带质量（动态读数会拒绝）、且不带轮胎
（整车无支承，静态配平无解）——均为夹具属性，不是机制缺陷。

**未收口项（明确登记，不冒充覆盖）**：
1. **K/C 读数的多轴轮心选择**：`cases/kc_quasi_static/contract.py::wheel_centre_body` 仍按 `<stem>_<side>`
   单侧查找，三轴装配会被它按「歧义」拒绝。该文件的写范围归 **04**（`SUBTASKS.csv` 04 行 notes 列明），
   03 不动；缺口由测试断言写在明处。
2. **`authoring/documents.py:76 _ASSEMBLY_SUPPLIED_BODIES`** 白名单（02 登记交 03 收口）本轮未改：
   它服务文档读取层对「模板引用它不拥有的体」的放行，与装配层的命名规则收口是两件事，
   放开它需与 06 的子系统文档可选声明段一起定，故登记为 **06 的输入**。

**证据**：`tasks/20260929-03-universal-assembler/raw/entry_grep.md`、
`raw/preparation_graph_and_new_shapes.md`、`raw/policy_and_schema.md`、`raw/adapter_parity.md`。

## 2026-09-29：04 轮端生命周期统一（DONE）与 04b 的登记

**04 已 DONE**。口径经用户裁决为 **B**：04 的验收 (a)「单轴与整车都读入同一份 wheel 子系统**文件**」在 04 声明的写范围内不可实现（实测 `subsystems/types.py::_ROLE_TEMPLATE_FIELD` 无 `wheel` 项、`authoring/solver.py::_FILE_ROLE_TEMPLATES` 排除 wheel、内置 `WHEEL` 模板 `parts=()`，三文件均不在 04 写范围），故重读为「`subsystems/wheel.py` 是单轴与整车轮端（车轮刚体 + 轮胎声明）的唯一生产者」；跨文件读 wheel 模板另立 **04b**（`depends_on=04`，已写入 `SUBTASKS.csv`，规格见 `tasks/20260929-04b-wheel-file/`）。

改了什么（全部在 04 写范围内）：`subsystems/wheel.py` 重写为轮端唯一生产者（`build` 按 wheel 模板的 parts 产出车轮体；`template_instance` 一处决定模板来源并预留 `getattr(request, "wheel_template")` 文件通道；新增 `wheel_center_body`）；`subsystems/si_assembly.py` 新增 `_wheel_end_is_supplied`（**判据 = `RigSpec.supplies_wheels`**，无试验台的单轴按 D9 同样凝结）、`_wheel_bodies`、`_wheel_end_mounts`，并把 wheel 贡献块前移到悬架行构建之前；`subsystems/vehicle_parts.py` 新增 `_condense_wheel_end`（算术全部复用既有 `_merge_fixed_wheel`，未新造机制）；`subsystems/assembler.py` 删除 `isinstance(element, VerticalTireElement)` 过滤与其 import，改用角色 `_AXLE_ROLES_IN_A_VEHICLE = DEFAULT_AXLE_SUBSYSTEMS - {"wheel"}`；`subsystems/element_build.py` 的轮胎行只在 `carries("wheel")` 时产出；新增 `tests/subsystems/test_wheel_lifecycle.py`（6 条）。

验证（退出码全部 0）：`just check-fast` 1053 passed / 1 xfailed + kernel 33 + contracts 32 + 三个架构门；`kc_native_probe` 9 K states（worst ratio 1.65548e-05）、`kc_native_c_probe` 66 C states（worst ratio 0.000185873）、`kc_parity_check --check --actual-dir artifacts/kc-native-probe` → OK；`dynamic_hash_sentinel --check` 26 个 artifact 逐字节一致，combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（与 03 收尾相同，未重录）；01 快照 `--check` **零差异**（`approved_deltas.json` 保持 `[]`）；`tests/data` 未被写。证据：`tasks/20260929-04-wheel-lifecycle/raw/kc_condensation_parity.md`、`raw/wheel_file_unification.md`、`raw/mass_ownership.md`。

与 **20260921 Epic A3** 的关系（SPEC 要求点名）：A3 裁决**整车侧**不凝聚、weld 交内核 `fixed` 关节；04 做的是**单轴侧**（`supplies_wheels=True` 的读数）把车轮体折进轮毂，方向相反、作用域不相交，A3 的结论未被触碰。

未收口（已登记，供 07 审计）：(1) (a) 的跨文件部分 → 04b；(2) 03 登记的「K/C 多轴轮心选择」→ 04b；(3) 04 未跑全量 pytest（按用户指示留到 07）。

---

## 2026-09-29：04b/05/06/07 落地与终局验收（Epic 不结项）

**04b DONE**：`AssemblyRequest.wheel_template` + 角色表项 + `_FILE_ROLE_TEMPLATES` 纳入 wheel 落地（端到端用例证明文档的 wheel 子系统被读为当前模板、其车轮体被产出并被凝结）；`wheel_centre_body` 改读装配自身的轮端表，03 登记的多轴轮心缺口收口（歧义被拒并点名候选）；`si_assembly._declared_wheel_mounts` 从悬架模板取挂接体（文件模板该点标签为 `center`，纯标签搜索对它无效）。

**05 DONE**：`_reown_tires` / `_is_replaced_tire` 与其调用点删除，`merge_rig_link` 纯追加；`test_rig_link` 契约按 D3 反转；新增 `test_the_rig_is_not_invasive.py`（10 条：两个供轮台逐项指纹 + 7 rig 覆盖）。产物差异为空，`approved_deltas.json` 保持 `[]`。

**06 IN_PROGRESS（不结项）**：已交付 `AssemblyRequest.sides`（缺省＝对称对，D5）、`geometry.side_hardpoints` 的「镜像默认 + `name__R` 覆盖」、单轮装配并跑通一次 K/C study、不对称硬点可装配。未交付：文档级 `sides`/`mirror` 声明与 `runtime_template_from` 的不再镜像路径（Done-When (b) 的兑现行）、三轮整车用例（需 `AxleEntry` 携带每条目的侧）。

**07 终局验收**：命令全部实跑并记录（`tasks/20260929-07-acceptance/raw/final_validation.md`）——全量 1526 passed/1 skipped/1 xfailed、architecture 147、contracts+kernel 65、ruff/ty 全过、三架构门 OK、sentinel 26 artifact 逐字节一致、K/C probe + `--actual-dir` parity OK、case_parity 8 families、kc_perf 在预算内、snapshot 零差异、`tests/data` 未被写。Done-When：(a)(c)(d)(e)(f)(g) DONE，**(b) NOT MET**，故 **Epic 不结项**；`06` 置 `IN_PROGRESS`，06 的 `TODO.csv` 第 1/2/3/4 行与 07 的 `TODO.csv` 第 2/8 行保持 `TODO`。

---

## 2026-09-29 结项：06 闭环，Epic 关闭

06 的两项阻断项在本轮闭环：**(1) 文档级对称声明**（`template.schema.json` 的 `sides`/`mirror`、`documents.TemplateDocument` 的校验与 `declared_sides`/`mirrors`、`bridge._mirrored_parts(mirror=)`、`solver._mirrored_connections(mirror=)` + `runtime_template_from` 的 `_per_side_roles`、`_sides_from_file`、`_connection_name` 的按侧兜底）；**(2) 三轮整车用例**（`assembler.AxleEntry.sides` + 转发；front 双侧 + rear 单侧的装配与「右手单侧角跑通一次 K/C study」）。过程中修掉 `steering.py` 最后一处侧硬编码。

**Done-When (b) 实测一致**（`tests/authoring/test_symmetry_declaration.py`）：模板层 `parts`/`connections` 逐字段、运行层体集合与逐体点坐标与约束名；**登记的残余差异**：写两侧的文件其右侧点 label 带自身 token（坐标一致，仅点名拼写不同）。

**最终验证（最终代码上实跑）**：全量 1531 passed / 1 skipped / 1 xfailed（30:28）、architecture 147 passed（12:42）、contracts+kernel 65 passed、ruff/ty 全过、三个架构门 OK、`dynamic_hash_sentinel --check` 26 artifact 逐字节一致（combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，未重录）、K/C probe（9/66）+ `kc_parity_check --check --actual-dir` OK、`case_parity_check` 8 families、`kc_perf_gate --check` 在预算内、`snapshot.py --check` 零差异、`tests/data` 未被写、`approved_deltas.json` 全程 `[]`。

`SUBTASKS.csv`：01/02/03/04/04b/05/06/07 **全部 DONE**；各子任务 `TODO.csv` 全部 DONE。**G1–G6 逐条有证据，Epic 关闭**。
