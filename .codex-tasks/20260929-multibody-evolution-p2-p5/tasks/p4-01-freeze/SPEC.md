# SPEC：p4-01 冻结现状事实与判据（阶段四）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-01`（`task_dir = tasks/p4-01-freeze`，`depends_on` 为空）。
> 父 Epic 内对应段落：阶段四原文要点（`EPIC.md:36-42`，路线图 191–196 行）、`EPIC.md:207` 的阶段四串行主线、`EPIC.md:261` 的 p4-01 验证协议、`EPIC.md:316-317` 的风险条。

## Goal

把父行 `acceptance_criteria` 拆成编号分条，逐条可判定。本行是**只读冻结**行：只产出现状事实、锚点复核与两套物理的对照，不修任何生产代码。

1. **ARB 现状锚点复核 + 两套 ARB 物理的对照**。逐条复核 `EPIC.md` F11（`EPIC.md:136`）的锚点原文：`subsystems/suspension.py:682-695` 的 `global_elements` 里 `body_a="upright_L"` / `body_b="upright_R"` 硬编码跨接、`modeling/primitives/elements.py:593 AntiRollBarElement`（力元，按两端 z 位移差出力偶 `stiffness * difference`，无扭杆刚体、无小吊杆）、`schema/elements.py:224 AntiRollBar` 的 6 个硬点字段中装配期只读 `left_link_point`/`right_link_point`、装配期唯一入口 `element_build.py:211` 与构造分派 `element_build.py:66-67`。同时给出**两套 ARB 物理的对照表**（力律、自由度、施加对象、消费路径差异）：Python 侧 `AntiRollBarElement` 与 native 扭杆 ABI（`EPIC.md:137`；对照用例 `tests/axle_dynamics/test_api.py:315 test_anti_roll_bar_reports_physical_angle_rate_and_torque`）。判据 = 对照表两侧各列出「力律表达式 / 自由度 / 施力与反力体 / 谁的路径消费它」，且每格带 `file:line`。
2. **六角色真源与硬断言的全清单实测**。按 `EPIC.md` F12（`EPIC.md:139`）的完整同步清单逐项实测并在证据里逐行给出 `file:line` 与原文：`templates/roles.py:70 ROLES` 与 `:158-162` 的角色集合硬断言（集合不等于 `{"suspension","steering","wheel","chassis","brake","drive"}` 即在 import 期抛 `RoleSpecError`）、`authoring/documents.py:59 FUNCTIONAL_ROLES`、`subsystems/types.py:69 SUBSYSTEM_ROLES`（含 `:76`/`:83` 默认集合）、`subsystems/capabilities.py:38 ALL_SUBSYSTEMS`、`subsystems/composition.py:63 SUBSYSTEM_ROLES`、`connections/policy.py:48 ROLES`、三份契约 schema 的 `functional_role` enum（`template.schema.json:11`、`subsystem.schema.json:8`、`assembly.schema.json:22`）、测试 `tests/templates/test_template_model.py:57 test_six_roles_are_declared`。判据 = 清单每一处都有「实测命令 + 行号 + 原文」，且**项数记入证据**（F12 称 10+ 处）。
3. **`arb_mount` 端口不存在的事实复核**。按 `EPIC.md` F13（`EPIC.md:141`）复核：`builtin.py:397 DOUBLE_WISHBONE` 的 `ports`/`needs` 皆为空元组；端口在装配期运行期合成（`subsystems/si_assembly.py:71 _ports_for_bodies`、`:104 _wheel_centre_needs`）；`arb_mount` / `droplink_mount` / `chassis_mount` 只命中路线图文档 `:93`/`:195`。判据 = 全仓 grep 的**命中清单原文**（源码/测试零命中，仅路线图文档命中），并明确记录「p4-03 不是接已有端口，而是先造出语义化端口」。
4. **wheel 文件链现状复核 + `VerticalTireElement` 引用点全清单**。按 `EPIC.md` F14（`EPIC.md:143`）复核：`subsystems/wheel.py:44-59 _instance` 只读内置模板（`from ..templates.builtin import WHEEL`，`:22`）、`authoring/solver.py:596-598 _FILE_ROLE_TEMPLATES = ("steering","chassis")`、`AssemblyRequest` 无 wheel 模板字段（`subsystems/types.py:151-160`，映射 `:249`）、**仓库中不存在任何 `*.subsystem.json` 实体文件**（命名约定 `{name}.tpl.json` / `{name}.sub.json` / `{name}.asy.json`，`authoring/security.py:168/242/292`）。按 `EPIC.md` F15（`EPIC.md:145`）复核实测引用点全清单：定义 `modeling/primitives/elements.py:567`；构造 `subsystems/element_build.py:31/136/138`（分派 `:68-69`）；按类型过滤删除 `subsystems/assembler.py:231`（全仓唯一的 `isinstance` 过滤）；试验台期判定 `rig_link.py:278/308/333`；native 侧拒绝 `preparation/vehicle_dynamic.py:55/900`；其它按名分流 `compilation/model_view.py:179`、`studies/bridge.py:125/334`、`cases/kc_quasi_static/contract.py:354`。判据 = 每个引用点一行（`file:line` + 原文 + 归属行：本 Epic 的 p4-04 / 阶段一 05 / 其它）。
5. **ARB 冻结产物复核**。按 `EPIC.md` F17（`EPIC.md:150`）复核实测：`tests/data/axle_dynamics_baseline/sha256.json` 的 `anti_roll_output` 值（空字节 sha256 `e3b0c442...b7852b855`）、`tests/data/vehicle_dynamics_baseline/sha256.json` 同、`dynamic_hash_baseline.json` 无 ARB 字段、`tests/data/kc_baseline/` 中 `anti_roll|arb` 命中 0。判据 = 四个基线文件的实测命中/取值原文，并记录父行 `notes` 的口径：**不得据此放松**，任何产物变化仍按 D5（`EPIC.md:64`）逐项登记。

## 写范围（允许改的路径）

照抄父行 `notes`：**写范围仅本行 `raw/` 与脚本与会话 scratch**。

- `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-01-freeze/raw/`（证据落点，开工后写入；规划轮必须为空）
- 复核用临时脚本写到会话 scratch（`$PI_SCRATCH_DIR`），**不落进仓库**
- 唯一的仓库侧改动是 `tasks/p4-01-freeze/PROGRESS.md` 的实施记录段与 `raw/` 证据文件

父行 `notes` 补充口径（照抄）：F17：ARB 冻结产物记录的是无 ARB（空字节 sha256）故 ARB 独立化不太可能扰动基线 但不得据此放松 任何产物变化仍按 D5 登记。

## 禁止触碰

- `packages/**` 下任何文件（本行只读；`packages/suspension_multibody/**`、`packages/suspension_kernel/**`、`packages/suspension_contracts/**` 全部不改）
- `.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`
- 其它行次的 `tasks/p4-02-arb-subsystem/`、`tasks/p4-03-arb-ports/`、`tasks/p4-04-wheel-unify/`、`tasks/p4-05-acceptance/`（含它们的 `raw/`）
- 任何基线文件（`tests/data/kc_baseline/`、`tests/data/axle_dynamics_baseline/sha256.json`、`tests/data/vehicle_dynamics_baseline/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`）——本行**只读不改**（`EPIC.md:228`）
- `raw/` 内不得放入任何虚构结果；规划轮 `raw/` 必须为空

## 依赖与时机

- 父行 `depends_on` **为空**（`SUBTASKS.csv` 的 p4-01 行）：本行是**纯只读冻结行**（写范围只有本行 `raw/` 与会话 scratch，冻结的是**阶段四的现状**——ARB/wheel/角色表/端口，**不读前一阶段任何会被本 Epic 改写的产物**），故**本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。`EPIC.md` 行 219 的阶段四顺序（阶段四在阶段三完成后开工）**只对实施行成立**。
- **阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**（p4-02 ~ p4-05 与终局验收行），不影响本行长；本行**不写生产代码**（父行 `notes`：写范围仅本行 `raw/` 与脚本与会话 scratch）。
- 本行是阶段四的**冻结/起点**行，必须在 p4-02、p4-03、p4-04 之前完成；p4-01 的结论是后三行「改哪、怎么改、不混淆什么」的唯一事实依据（`EPIC.md:207`、`EPIC.md:316-317`）。
- 本行为只读行，不存在与并行行的写冲突；但结论应早于 p4-02 开工，因为 p4-02 的角色表改动清单直接引用本行验收的第 2 条。

## 判据与证据落点

逐条对应 `EPIC.md:261`「p4-01（冻结现状 · 阶段四）」的 (a)–(e)：

- **(a) ARB 现状锚点复核 + 两套 ARB 物理的对照** → 跑 `grep`/`sed` 读取 F11 各锚点原文与 `tests/axle_dynamics/test_api.py:315` 用例原文；把「力律 / 自由度 / 施力与反力体 / 消费路径」两侧对照表写入 `raw/arb_two_physics.md`。该文件同时是父行 `validation_command` 的 `test -s` 检查对象。
- **(b) 六角色真源与硬断言全清单实测** → 按 F12 清单逐项 `grep -n` 取行号与原文；把清单、实测命令、每处原文与**项数**写入 `raw/role_table_sync.md`。该文件同时是父行 `validation_command` 的 `test -s` 检查对象。
- **(c) `arb_mount` 端口不存在的事实复核** → 全仓 `grep -rn "arb_mount\|droplink_mount\|chassis_mount"`（含源码、测试、文档），把命中清单与「源码/测试零命中」的结论写入 `raw/arb_mount_grep.md`（含 `_ports_for_bodies` 的运行期合成路径原文）。
- **(d) wheel 文件链现状（F14）+ `VerticalTireElement` 引用点全清单（F15）** → 逐点 `grep -n` 取原文；写入 `raw/wheel_chain_and_tire_refs.md`（每个引用点标注归属：p4-04 / 阶段一 05 / 其它行 / 本 Epic 不改）。
- **(e) ARB 冻结产物复核（F17）** → 读四个基线文件的相关字段/命中；把实测值（sha256 原文、命中数 0）与 D5 登记口径写入 `raw/arb_baseline_freeze.md`。

证据文件全部落在 `tasks/p4-01-freeze/raw/`；命令与退出码一并记录（只记**已执行**的结果）。

## Constraints（冻结约束）

- **两套 ARB 物理不得混淆**（`EPIC.md:137`、`EPIC.md:317`）：Python `modeling/primitives/elements.py:593 AntiRollBarElement` 是「按两端 z 位移差出力偶」的**力元**；native 扭杆 ABI 是**另一套物理**（对照用例 `tests/axle_dynamics/test_api.py:315`）。本行必须把两者差异写清，供 p4-02 选型；**不得把任一侧的输出当作另一侧的等价物**。
- **基线不得重录**（`EPIC.md:228`，D5 见 `EPIC.md:64`）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；`vehicle_dynamics_baseline/sha256.json` 与 `kc_perf_baseline_native.json` 本 Epic 期间**不再动**。本行只读。
- **分层方向不可逆**（`EPIC.md:226`）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。本行不改代码，但复核结论要按此方向陈述。
- **不得新增 skip/xfail**（`EPIC.md:229`）；`tests/adams` 的环境 skip 是既有的，不得增长。
- **不许依据旧任务 DONE 结论**（`EPIC.md:237` 的口径，本行同口径）：所有事实必须在本行开工时**重新实测**，不得抄录父 Epic 的 F 编号文字当作实测结论。
- **装配层不得出现按名字猜身份的规则**（`EPIC.md:233`）：任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约——本行结论要按此口径记录 `_BODY_ALIASES`（`subsystems/geometry.py:226`，F16 见 `EPIC.md:147`）的现状。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据（`EPIC.md:355`）。

## 风险与回退

- **阶段四改角色表会连锁（`EPIC.md:316`）**：F12 列出的 10+ 处同步点，漏一处就在 import 期抛 `RoleSpecError` 或契约 schema 拒绝。缓解：本行的第 2 条先把全清单**实测固化**（含 `tests/templates/test_template_model.py:57`），成为 p4-02 的逐项改动依据；清单缺项即本行未完成。
- **阶段四 ARB 两套物理混淆（`EPIC.md:317`）**：缓解 = 本行第 1 条先做对照（力律、自由度、施力与反力体、消费路径四列），p4-02 选型时必须引用本行对照表。
- **F17 的「无 ARB 基线」会诱使放松（`EPIC.md:150`）**：空字节 sha256 只说明当前基线不含 ARB 产物，**不等于**改动无风险。缓解 = 结论里显式写明「不得据此放松」，任何产物变化仍按 D5 逐项登记。
- **F14 的命名误导**：路线图说的 `wheel.subsystem.json` 在仓库里**没有实体**（`EPIC.md:143`）。缓解 = 本行证据以 `git ls-files`/实际 grep 结果为准，明确记录「目标物尚不存在」，避免 p4-04 按不存在的前例施工。
- **回退**：本行为只读行，无生产改动可回退；若某项复核发现父 Epic 的锚点已过期，**不改父文件**，只在 `raw/` 与 `PROGRESS.md` 记录「锚点过期 + 实测新锚点」，并标注哪一行（p4-02/p4-03/p4-04）需据此调整。

## Done-When

- [ ] `raw/arb_two_physics.md` 已落盘且非空：F11 锚点原文复核完成，两套 ARB 物理的对照表四列（力律 / 自由度 / 施力与反力体 / 消费路径）每格带 `file:line`。
- [ ] `raw/role_table_sync.md` 已落盘且非空：F12 的每一处同步点都有实测行号与原文，项数已记录，含 `tests/templates/test_template_model.py:57`。
- [ ] `raw/arb_mount_grep.md` 已落盘且非空：`arb_mount` / `droplink_mount` / `chassis_mount` 的命中清单原文（源码/测试零命中），含 `si_assembly.py:71`/`:104` 的运行期合成路径。
- [ ] `raw/wheel_chain_and_tire_refs.md` 已落盘且非空：F14 文件链四处锚点与 F15 全部引用点（含 `assembler.py:231` 的唯一 `isinstance` 过滤）逐点带归属。
- [ ] `raw/arb_baseline_freeze.md` 已落盘且非空：F17 四个基线文件的实测取值/命中数与 D5 登记口径。
- [ ] 本行未修改 `packages/**` 下任何文件、未改父 Epic 三份文件、未写入其它行次的目录。
- [ ] 无新增 skip/xfail；未重录任何基线。
- [ ] 父行 `validation_command` 退出码为 0（含两个 `test -s` 证据文件存在且非空）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/modeling -q && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-01-freeze/raw/arb_two_physics.md && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-01-freeze/raw/role_table_sync.md
```
