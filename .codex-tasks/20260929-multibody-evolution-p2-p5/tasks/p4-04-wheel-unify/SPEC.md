# SPEC：p4-04 单轴与整车 wheel 文件统一并消除轮胎补丁

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-04`（`task_dir = tasks/p4-04-wheel-unify`，`depends_on = p4-03`）。
> 父 Epic 内对应段落：Goal G6（`EPIC.md:93`）、阶段四原文要点（`EPIC.md:36-42`，路线图 191–196 行）、wheel 文件链事实 F14（`EPIC.md:147`）、`VerticalTireElement` 引用点全清单 F15（`EPIC.md:149`）、p4-04 验证协议（`EPIC.md:271`）、写范围约束（`EPIC.md:220-221`）、前置与前置边界（`EPIC.md:69-79`）。

## Goal

**边界（先读）：以阶段一 04 的「实际交付」为界，不得重做**（父行 `acceptance_criteria` 已按审核阻断项 6 重定，见 `EPIC.md:75` 前置边界与 `EPIC.md:271` p4-04 验证协议）。阶段一 Epic `.codex-tasks/20260929-assembly-layer-rework/` 的 `04` 行（`tasks/20260929-04-wheel-lifecycle`，其 `acceptance_criteria` 见该 Epic `SUBTASKS.csv` 第 5 行）已经要求：**单轴与整车读入同一份 wheel 子系统文件（文件读取链打通）且两侧都由它产出车轮刚体与轮胎声明；装配阶段不再出现 `VerticalTireElement` 的类型过滤；单轴侧凝结（实体集合/约束行数/自由度不变，轴侧用例逐位不变）；K 试验台下轮胎不参与受力、C 垫板机下轮胎弹性生效各有断言**。**这些动作由阶段一 04 做，本行不得重做。**

本行只做三件事，逐条可判定：

1. **独立复验阶段一 04 的实际交付**（`EPIC.md:271(a)`）：自己重跑并实测单轴侧与整车侧消费的是**同一份** wheel 子系统文件——比对**文件路径与内容指纹**（sha256），**不看阶段一自报**（不采信其 `PROGRESS.md` / `raw/` 的结论）；且 `grep -rn "VerticalTireElement"` 在装配路径（`subsystems/assembler.py` 等）**无命中**（`EPIC.md:93` 的 G6 判据）。判据 = 两侧路径与指纹对照表 + grep 输出与退出码。
2. **剩余项清单（可为空）**（`EPIC.md:271(b)`）：本行开工时**实测**得的明确剩余项，逐条列出并登记来源（`file:line` + 实测输出）。若阶段一 04 已全部兑现，清单为空，本行收缩为「独立复验」。**不得**为凑交付而重做阶段一 04 的动作，也**不得**把阶段一 04 未覆盖的问题写成它的缺口。
3. **阶段一 04 未覆盖的部分**（`EPIC.md:271(c)`）：只做实测确认「阶段一 04 未覆盖」的部分，例如 `authoring/solver.py:596-598 _FILE_ROLE_TEMPLATES = ("steering", "chassis")` 若阶段一 04 未放开 wheel 角色，则由本行放开并登记；`AssemblyRequest`（`subsystems/types.py:151-160`，映射 `:249`）若仍缺 wheel 模板字段则补上。每项都要有「阶段一 04 未覆盖」的实测依据（阶段一 04 的 `PROGRESS.md`/`raw/` 原文 + 代码现状）与其来源登记。

> **F14 修正（必读，`EPIC.md:147`）**：仓库里**没有任何 `*.subsystem.json` 实体文件**——命名约定是 `{name}.tpl.json` / `{name}.sub.json` / `{name}.asy.json`（`authoring/security.py:168/242/292`），且 `git ls-files` 显示无任何真实文档实例被提交。路线图说的 `wheel.subsystem.json` 是**尚不存在的目标物**。本行以实际的命名约定为准，不得按不存在的前例施工。

## 写范围（允许改的路径）

父行 `notes` 原文：**写范围 `subsystems/wheel.py` 与 `subsystems/assembler.py` 的轮胎过滤段 与 `subsystems/element_build.py` 的轮胎段 与 `preparation/vehicle_dynamic.py` 的轮胎拒绝段 与 `authoring/solver.py`。F14 修正：仓库无任何 `*.subsystem.json` 实体文件，路线图说的 `wheel.subsystem.json` 是尚不存在的目标物。**

**边界（`EPIC.md:75` 前置边界）**：上述写范围**只在「阶段一 04 未覆盖」的剩余项上生效**；阶段一 04 已交付的动作（文件读取链打通、移除 `VerticalTireElement` 类型过滤、单轴侧凝结、K/C 受力激活断言）本行**不得重做**。开工第一步必须先读阶段一 04 的 `PROGRESS.md` 与 `raw/`，**据此把判据的实测结论与（若需调整的）判据文本写入 `raw/stage1_04_intake.md` 并在本行 `PROGRESS.md` 记录，同时提请父 Epic 修订 `SUBTASKS.csv`**——**本行不得直接改父表 `SUBTASKS.csv`（真源）**（2026-09-29 复审修订：原文「按其实际交付重写本行 `acceptance_criteria`」与「子任务不得改真源」冲突，已废）。

展开为：

- `packages/suspension_multibody/src/suspension_multibody/subsystems/wheel.py`（**文件来源段**：`_instance` 只读内置模板的现状；`:44-59`、`:22`）——**仅当阶段一 04 未把 wheel 角色接进文件读取链时**才由本行改动。
- `packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py`（**轮胎过滤段**：`:231` 的 `isinstance` 过滤；该文件其余段归阶段一 03/p4 其它行）——**该段属阶段一 04 的动作（`EPIC.md:75`），本行只复验其 grep 结果，不得重做**。
- `packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py`（**轮胎段**：构造分派 `:68-69` 与 `:31/136/138` 的轮胎构造；**构造分派段归 p2-03**，见 `EPIC.md:220`）——**仅当阶段一 04 未覆盖该段时**才由本行改动。
- `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py`（**轮胎拒绝段**：`:55/900`；其余段归 p2-05/p2-06，见 `EPIC.md:221`）——**仅当阶段一 04 未覆盖该段时**才由本行改动。
- `packages/suspension_multibody/src/suspension_multibody/authoring/solver.py`（`_FILE_ROLE_TEMPLATES` `:596-598`）——阶段一 04 的写范围未含此文件，若它未放开 wheel 角色，由本行放开并登记（`EPIC.md:271(c)`）。
- `packages/suspension_multibody/src/suspension_multibody/subsystems/types.py`（**仅 `AssemblyRequest` 的模板字段段**：`:151-160`，映射 `:249`；若必须新增 wheel 模板字段）——同上，属阶段一 04 未覆盖的收口项。
- 本行新增的测试：`packages/suspension_multibody/tests/subsystems/`、`tests/vehicle_assembly/`、`tests/authoring/`（复验与剩余项证据用）
- `raw/` 证据与 `PROGRESS.md` 的登记段

## 禁止触碰

- `.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`
- 其它行次的目录与 `raw/`（`tasks/p4-01-freeze/`、`tasks/p4-02-arb-subsystem/`、`tasks/p4-03-arb-ports/`、`tasks/p4-05-acceptance/`，以及全部 p2-*/p3-*/p5-*）
- **p4-02 / p4-03 的写范围**：`templates/roles.py`、`templates/builtin.py` 的角色/模板注册段、`subsystems/si_assembly.py` 的端口合成段、`subsystems/geometry.py`（`EPIC.md:267`、`EPIC.md:269`）
- **阶段一 04 / 05 的写范围**：`subsystems/rig_link.py` 的 `_reown_tires`（`EPIC.md:149` 明确「属阶段一 05 的范围，本 Epic 不碰」）、**阶段一 04 的刚性凝结实现与文件读取链打通/类型过滤移除动作（`EPIC.md:75` 前置边界：本行不得重做，只做独立复验）**
- **`subsystems/assembler.py` 与 `subsystems/element_build.py` 的其它段**：`element_build.py` 的**构造分派段归 p2-03**（`EPIC.md:220`）；`assembler.py` 的引擎/配对段归阶段一 03 与其后续行
- **`preparation/vehicle_dynamic.py` 的其它段**：归 p2-05（预采样与 bias）、p2-06（转向）（`EPIC.md:221`）
- `packages/suspension_kernel/**`（不改内核；`EPIC.md:229`）
- 任何基线文件（`EPIC.md:232`）
- `raw/` 内不得放入任何虚构结果；规划轮 `raw/` 必须为空

## 依赖与时机

- `depends_on = p4-03`。阶段四串行主线 p4-01 → p4-02 → p4-03 → p4-04 → p4-05（`EPIC.md:211`）；阶段四整段在**阶段三完成后**开工（`EPIC.md:215`）。
- 全局前置 `S1`：阶段一 Epic 的 01–07 全部 `DONE`；**阶段一未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:69`、`EPIC.md:79`）。本行直接依赖阶段一的**轮端生命周期统一（04）**与**试验台非侵入（05）**（`EPIC.md:72`）。**开工第一步必须先读阶段一 04 的 `PROGRESS.md` 与 `raw/`**（`EPIC.md:75`、`EPIC.md:271`）。
- **共享文件串行约束（必须遵守）**：
  - `subsystems/element_build.py` 被三行触及（p2-03 构造分派、p4-04 轮胎段、p3-02 只读）；**构造分派段归 p2-03、轮胎段归 p4-04，必须串行**（`EPIC.md:220`）。
  - `preparation/vehicle_dynamic.py` 被三行触及（p2-05、p2-06、p4-04）；**串行，p2-06 在 p2-05 后、p4-04 在 p2-06 后**（`EPIC.md:221`）。
  - `authoring/solver.py` 的 `_FILE_ROLE_TEMPLATES` 改动需注意阶段一 03 已使用该表；本行只**新增 wheel 角色**，不改已有角色语义。

## 判据与证据落点

逐条对应 `EPIC.md:271`「p4-04（wheel 统一 + 消除补丁）」的 (a)(b)(c)：

- **(a) 独立复验阶段一 04 的交付：两侧消费同一份 wheel 文件** → 自己重跑单轴 K/C 与整车各一次，记录两侧解析出的 wheel 子系统文件**路径**与**内容 sha256**，给出对照表（路径相同且指纹相同才算通过）；同时记录 `grep -rn VerticalTireElement` 在 `subsystems/` 与 `preparation/` 装配路径的输出与退出码。**不采信阶段一 04 的自报结论。** 证据落 `raw/stage1_04_reverification.md`。
- **(b) 剩余项清单** → 本行开工实测的剩余项逐条列出（`file:line` + 实测输出 + 来源归属），可为空；为空时显式写「阶段一 04 已全部兑现，本行收缩为独立复验」。证据落 `raw/remaining_items.md`。
- **(c) 阶段一 04 未覆盖的部分** → 每项给出「未覆盖」的实测依据（阶段一 04 的 `PROGRESS.md`/`raw/` 原文 + 代码现状）与本行的处置（放开 wheel 角色 / 补 `AssemblyRequest` 模板字段 / 无需处置），逐项登记。证据落 `raw/uncovered_scope.md`。
- **(d) 边界与凝结归属点名** → `PROGRESS.md` 写清「阶段一 04 = 求解拓扑的刚性凝结（本行不碰）vs 本行 = 独立复验 + 阶段一 04 未覆盖部分的文件来源收口」；并把「本行未触碰凝结实现」的证据（diff 范围或 grep）写入 `raw/condensation_boundary.md`。
- **数值与回归** → 每步落地后跑 `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check`（父行 `validation_command` 的最后一段）；`kc_baseline` 与 `dynamic_hash_baseline` 逐位未变。证据落 `raw/gates.txt`。

## Constraints（冻结约束）

- **以阶段一 04 的实际交付为界，不得重做（`EPIC.md:75`、`EPIC.md:271`）**：阶段一 04（`wheel-lifecycle`）定的是**求解拓扑的刚性凝结 + 文件读取链打通 + 类型过滤移除**；本行只做**独立复验**（路径与指纹比对 + grep，不看自报）与阶段一 04 **未覆盖**部分的收口。**不得**回改阶段一 04 的交付物，也不得把「文件统一」当作凝结的替代或补充（`EPIC.md:72`）。
- **父行 `notes` 的「两件事」已按修订重述**：阶段一 04 定求解拓扑，本行定**独立复验**；SPEC 原文里的「本行定文件来源与类型判别」只在「阶段一 04 未覆盖」的剩余项上成立。
- **两套 ARB 物理不得混淆**（`EPIC.md:141`）：本行不碰 ARB；不得借轮端改造顺手改 ARB 物理。
- **基线不得重录**（`EPIC.md:232`，D5 见 `EPIC.md:64`）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间**不再动**。任何产物变化按 D5 逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」。
- **分层方向不可逆**（`EPIC.md:230`）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。
- **不得把「简化/专用」的分支写进 role 接口**（`EPIC.md:236`）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层——本行的统一表示**不得**在 role 接口上分叉。
- **不得新增 skip/xfail**（`EPIC.md:233`）；`tests/adams` 的环境 skip 是既有的，不得增长。
- **不改轮胎力律本构**（`EPIC.md:101` Non-Goals）：本行只改「谁提供轮胎实体、按什么判别」，不改 PAC2002/Fiala 本构。
- **内核 ABI 单点提交**（`EPIC.md:229`）：本行不改内核、不改 ABI。
- **装配层不得出现按名字猜身份的规则**（`EPIC.md:237`）：统一表示必须经形式化契约，不得换成按名字/类型名的推断。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据。

## 风险与回退

- **复验发现阶段一 04 未兑现**：本行**不再自己删** `assembler.py:231` 的 `isinstance` 过滤（那是阶段一 04 的动作，`EPIC.md:75`）。缓解 = 用实测证据（路径/指纹/grep）把未兑现项登记进 `raw/remaining_items.md`，**不重做**阶段一 04 的动作；只有实测确认「阶段一 04 未覆盖」的部分才由本行处置（`EPIC.md:271(c)`）。
- **`preparation/vehicle_dynamic.py` 是共用文件（三行触及，`EPIC.md:221`）**：与 p2-05/p2-06 并行会静默覆盖。缓解 = 严格串行，本行只改**轮胎拒绝段**（且只在阶段一 04 未覆盖时）。
- **`element_build.py` 是共用文件（`EPIC.md:220`）**：本行只改**轮胎段**，构造分派段归 p2-03。
- **F14 的命名误导**：路线图说的 `wheel.subsystem.json` **不存在**（`EPIC.md:147`）。缓解 = 以实际命名约定（`{name}.tpl.json` / `{name}.sub.json` / `{name}.asy.json`）为准，并在证据里记录「目标物尚不存在，本行创造它」。
- **与阶段一 04 混为一谈**：把「文件统一」说成「凝结」会掩盖真实验证点。缓解 = 本行 `PROGRESS.md` 显式写出两者作用线；`raw/condensation_boundary.md` 记录本行 diff 未触凝结实现。
- **回退**：回退本行 diff（本行实际改动的「阶段一 04 未覆盖部分」），恢复改动前状态。**不得**以重录基线方式「回退」；**不得**把阶段一 04 的交付物纳入本行的回退范围。

## Done-When

- [ ] 阶段一 04 的 `PROGRESS.md` 与 `raw/` 已读；本行的实测结论与（若需调整的）判据文本已写入 `raw/stage1_04_intake.md`，调整项已在本行 `PROGRESS.md` 记录，并**提请父 Epic 修订 `SUBTASKS.csv`**；**本行未直接改父表 `SUBTASKS.csv`**（`EPIC.md:271` 复审修订）。
- [ ] **独立复验**：实测单轴侧与整车侧消费的是**同一份** wheel 子系统文件——文件路径与内容指纹一致（有对照表），且**未采信阶段一自报**。
- [ ] `grep -rn "VerticalTireElement"` 在装配路径（`subsystems/assembler.py` 等）无命中；父行的 `bash -c '! grep ...'` 退出码 0。
- [ ] 剩余项清单落盘（`raw/remaining_items.md`）且逐条带来源；若为空则显式声明本行收缩为独立复验。
- [ ] 阶段一 04 未覆盖的部分已按实测处置并登记（例如 `_FILE_ROLE_TEMPLATES` 的 wheel 角色、`AssemblyRequest` 的 wheel 模板字段）。
- [ ] 与阶段一 04 刚性凝结机制的边界已点名（求解拓扑 vs 本行复验与文件来源收口），且本行未触碰凝结实现。
- [ ] `dynamic_hash_sentinel.py --check` 通过（或在 D5 下逐项登记了变化）；未改轮胎本构；无新增 skip/xfail。
- [ ] 父行 `validation_command` 退出码为 0。
- [ ] **本行未重做阶段一 04 已要求的动作**（重做即违规）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/vehicle_assembly packages/suspension_multibody/tests/authoring -q && bash -c '! grep -rn VerticalTireElement packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py' && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
