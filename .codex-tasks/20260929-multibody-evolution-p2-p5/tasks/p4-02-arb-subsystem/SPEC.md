# SPEC：p4-02 anti_roll_bar 独立子系统

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-02`（`task_dir = tasks/p4-02-arb-subsystem`，`depends_on = p4-01`）。
> 父 Epic 内对应段落：Goal G5（`EPIC.md:87`）、阶段四原文要点（`EPIC.md:36-42`，路线图 191–196 行）、并行与写范围约束（`EPIC.md:215`、`EPIC.md:219`）、冻结约束（`EPIC.md:226-233`）、p4-02 验证协议（`EPIC.md:263`）、风险条（`EPIC.md:316-317`）。

## Goal

把父行 `acceptance_criteria` 拆成编号分条，逐条可判定。

1. **新增 `anti_roll_bar` 角色与模板（按 F12 全清单逐项同步）**。新增角色后按 `EPIC.md` F12（`EPIC.md:139`）的**完整同步清单**逐项改：`templates/roles.py:70 ROLES` 与 `:158-162` 的角色集合硬断言（集合不等于新集合即在 import 期抛 `RoleSpecError`）、`authoring/documents.py:59 FUNCTIONAL_ROLES`、`subsystems/types.py:69 SUBSYSTEM_ROLES`（含 `:76`/`:83` 默认集合）、`subsystems/capabilities.py:38 ALL_SUBSYSTEMS`、`subsystems/composition.py:63 SUBSYSTEM_ROLES`、`connections/policy.py:48 ROLES`、三份契约 schema 的 `functional_role` enum（`template.schema.json:11`、`subsystem.schema.json:8`、`assembly.schema.json:22`）、以及测试 `tests/templates/test_template_model.py:57 test_six_roles_are_declared`。判据 = 上列每一处都在 diff 里被改到，且 `import` 期不抛 `RoleSpecError`、契约 schema 接受新角色（有正例断言）。
2. **子系统含扭杆（刚体或等效扭簧力元）与左右小吊杆，选型给理由并与 native 扭杆关系说清**。选型必须在 SPEC/实现里写明「用扭杆刚体还是等效扭簧力元」的**理由**，并显式说明与 native 扭杆 ABI 的**关系**（`EPIC.md:137`：Python `modeling/primitives/elements.py:593 AntiRollBarElement` 是 z 位移差力偶力元；native 扭杆 ABI 是另一套物理，对照用例 `tests/axle_dynamics/test_api.py:315`）。判据 = 选型理由与两者关系写在落地记录中；子系统装配产物里能指出扭杆与左右小吊杆各自的实体。
3. **暴露 4 个端口 `chassis_mount_L/R`、`droplink_mount_L/R`**。端口必须在模板/子系统**声明**上具备（不是运行期临时字符串），命名逐字为 4 个；判据 = 读装配产物的端口集合，4 个名字齐全，且左右各自成立（`chassis_mount_L`/`chassis_mount_R`/`droplink_mount_L`/`droplink_mount_R`）。
4. **`suspension.py` 的 `upright_L` / `upright_R` 硬编码删除且 grep 无命中**。`subsystems/suspension.py:682-695` 的 `global_elements` 里 `body_a="upright_L"` / `body_b="upright_R"`（含 `context.local("upright_L", ...)`，锚点见 `EPIC.md:136`）必须删除；判据 = 父行 `validation_command` 里的 `bash -c '! grep -rn ... suspension.py'` 退出码 0，且防倾杆构造路径不再出现这两个名字（`grep -n "\"upright_L\"\|\"upright_R\""` 在防倾杆构造路径无命中，`EPIC.md:87` 的 G5 判据）。
5. **`test_template_model.py` 的角色断言相应更新且理由登记**。`tests/templates/test_template_model.py:57 test_six_roles_are_declared` 是硬断言六角色名元组的测试，角色集合变化后必须同步更新；判据 = 测试改名/改断言后通过，**理由在 `PROGRESS.md` 登记**（`EPIC.md:263(e)` 要求「理由登记」）。

## 写范围（允许改的路径）

照抄父行 `notes`：**写范围 `templates/roles.py` 与 `templates/builtin.py` 与 `subsystems/suspension.py` 与新增的 ARB 子系统与三份契约 schema 的 `functional_role` enum。共享注册文件不得与 p2-04 并行（p2-04 在前）。改角色表会连锁 F12 清单 10+ 处，漏一处即 import 期 `RoleSpecError`。**

按 F12 清单与父行 `acceptance_criteria`，展开为：

- `packages/suspension_multibody/src/suspension_multibody/templates/roles.py`（`ROLES` 与硬断言）
- `packages/suspension_multibody/src/suspension_multibody/templates/builtin.py`（新增 `anti_roll_bar` 模板；`BUILTINS` 元组 `:638` 与 `register_builtins()` `:648`）
- `packages/suspension_multibody/src/suspension_multibody/subsystems/suspension.py`（删除 `:682-695` 的 `upright_L`/`upright_R` 硬编码跨接）
- 新增的 ARB 子系统模块（`anti_roll_bar` 的构造/装配实现）
- **F12 全清单**（`EPIC.md:139`）的其余同步点，逐项点名：
  - `packages/suspension_multibody/src/suspension_multibody/authoring/documents.py`（`FUNCTIONAL_ROLES`）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/types.py`（`SUBSYSTEM_ROLES` 与默认集合）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/capabilities.py`（`ALL_SUBSYSTEMS`）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/composition.py`（`SUBSYSTEM_ROLES`）
  - `packages/suspension_multibody/src/suspension_multibody/connections/policy.py`（`ROLES`）
  - 三份契约 schema 的 `functional_role` enum：`packages/suspension_contracts/src/suspension_contracts/contracts/template.schema.json`、`subsystem.schema.json`、`assembly.schema.json`
  - `packages/suspension_multibody/tests/templates/test_template_model.py`（角色断言同步）
  - 本行新增的 ARB 测试（`tests/templates/`、`tests/subsystems/`、`tests/modeling/` 范围内）

## 禁止触碰

- `.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`
- 其它行次的目录与 `raw/`（`tasks/p4-01-freeze/`、`tasks/p4-03-arb-ports/`、`tasks/p4-04-wheel-unify/`、`tasks/p4-05-acceptance/`，以及全部 p2-*/p3-*/p5-*）
- **阶段二的行**：p2-04（brake/drive 模板改造）是本行前置，**本行不得回改 p2 的产物**（`EPIC.md:215`）
- **p4-03 与 p4-04 的写范围**：`subsystems/si_assembly.py` 的端口合成段与 `subsystems/geometry.py`（p4-03，`EPIC.md:265`）；`subsystems/wheel.py`、`subsystems/assembler.py`、`subsystems/element_build.py` 的轮胎段、`preparation/vehicle_dynamic.py` 的轮胎拒绝段、`authoring/solver.py`（p4-04，`EPIC.md:267`）
- `packages/suspension_kernel/**`（本行不动内核；`EPIC.md:225` 的「内核 ABI 单点提交」）
- 任何基线文件：`tests/data/kc_baseline/`、`tests/data/axle_dynamics_baseline/sha256.json`、`tests/data/vehicle_dynamics_baseline/sha256.json`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`（`EPIC.md:228`）
- `raw/` 内不得放入任何虚构结果；规划轮 `raw/` 必须为空

## 依赖与时机

- `depends_on = p4-01`（`SUBTASKS.csv` 的 p4-02 行）。p4-01 交付的 F11/F12/F13 实测清单是本行的**唯一**改动依据（`EPIC.md:261`、`EPIC.md:317`）。
- **`templates/roles.py` 与 `templates/builtin.py` 是共享注册文件，不得并行；p2-04 在前**（`EPIC.md:215` 逐字要求）。同时 `EPIC.md:227` 明确：**三份契约 schema 的 `functional_role` enum 与 `templates/roles.py` 的角色集合及硬断言同属 p4-02 的写范围；p2 的任何行不得增删角色名。**
- 阶段四整段在**阶段三完成后**开工（`EPIC.md:211`）；本行是阶段四的第二个行次，串行主线为 p4-01 → p4-02 → p4-03 → p4-04 → p4-05（`EPIC.md:207`）。
- 全局前置 `S1`：阶段一 Epic 的 01–07 全部 `DONE`；**阶段一未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:75`）。
- 契约 schema `assembly.schema.json` 与阶段一 02/03 的配对段/放置段改动**串行**（`EPIC.md:218`）；本行只动 `functional_role` enum。

## 判据与证据落点

逐条对应 `EPIC.md:263`「p4-02（ARB 独立子系统）」的 (a)–(e)：

- **(a) 新增角色与模板，按 F12 全清单同步** → 跑父行 `validation_command` 的 `tests/templates`、`tests/subsystems`、`tests/modeling`；把逐处同步的 `file:line` 与 diff 摘要写入 `raw/role_sync_manifest.md`；`import` 期不抛 `RoleSpecError` 的实测原文一并记录。
- **(b) 扭杆 + 左右小吊杆的选型与理由 + native 扭杆关系** → 把选型（刚体 vs 等效扭簧力元）、理由、与 native 扭杆 ABI 的关系（引用 p4-01 的 `raw/arb_two_physics.md`）写入 `raw/arb_topology.md`。
- **(c) 4 个端口暴露** → 装配产物里的端口集合实测（4 个名字逐字）写入 `raw/arb_ports_declared.md`。
- **(d) `upright_L`/`upright_R` 硬编码删除且 grep 无命中** → 跑父行 `validation_command` 里的 `bash -c '! grep -rn ... suspension.py'`（退出码 0）；同时记录防倾杆构造路径的 `grep -n "\"upright_L\"\|\"upright_R\""` 无命中结果，写入 `raw/upright_grep.md`。
- **(e) `test_template_model.py` 角色断言同步且理由登记** → 该测试通过的命令与输出写入 `raw/role_assertion_update.md`；**更新理由**写进本行 `PROGRESS.md`（登记要求来自 `EPIC.md:263(e)`）。

## Constraints（冻结约束）

- **两套 ARB 物理不得混淆**（`EPIC.md:137`、`EPIC.md:317`）：Python `modeling/primitives/elements.py:593 AntiRollBarElement` 是「按两端 z 位移差出力偶 `stiffness * difference`」的**力元**；native 扭杆 ABI 是**另一套物理**（`tests/axle_dynamics/test_api.py:315 test_anti_roll_bar_reports_physical_angle_rate_and_torque`）。本行的选型必须写明取哪一套、为什么，**不得把两侧输出当等价物**，不得在 Python 侧假造 native 扭杆的输出语义。
- **共享注册文件不得并行**（`EPIC.md:215`、`EPIC.md:219`）：`templates/roles.py` 与 `templates/builtin.py` 上不得与 p2-04 同时写入；p2-04 完成后本行才能开工。
- **F12 全清单漏一处即失败**（`EPIC.md:139`、`EPIC.md:316`）：`roles.py:158-162` 的硬断言与三份 schema enum 是**前置墙**——只改一处会在 import 期抛 `RoleSpecError` 或契约 schema 拒绝。
- **基线不得重录**（`EPIC.md:228`，D5 见 `EPIC.md:64`）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门。F17（`EPIC.md:150`）实测 ARB 冻结产物记录的是**无 ARB**（空字节 sha256），故 ARB 独立化不太可能扰动基线，但**不得据此放松**——任何产物变化仍按 D5 逐项登记。
- **分层方向不可逆**（`EPIC.md:226`）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。新增 ARB 子系统必须落在正确层。
- **内核 ABI 单点提交**（`EPIC.md:225`）：只有 p2-02 可以改 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量；**本行不改内核、不改 ABI**。
- **不得新增 skip/xfail**（`EPIC.md:229`）；`tests/adams` 的环境 skip 是既有的，不得增长。
- **装配层不得出现按名字猜身份的规则**（`EPIC.md:233`）：任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约；删除 `upright_L`/`upright_R` 后必须给出**替代机制**（端口配对），不得换成另一种按名字的推断。
- **不得把「简化/专用」的分支写进 role 接口**（`EPIC.md:232`）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。
- 每步落地后重跑 `just check-fast`；改结构后加跑 `tests/architecture`（`EPIC.md:230`）。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据。

## 风险与回退

- **阶段四改角色表会连锁（`EPIC.md:316`）**：F12 列出的 10+ 处同步点，漏一处就在 import 期抛 `RoleSpecError` 或契约 schema 拒绝。缓解 = 以 p4-01 实测固化的清单为唯一依据，逐项打勾；先改 `templates/roles.py` 与三份 schema enum（前置墙），再改其余同步点，每步跑测试。
- **阶段四 ARB 两套物理混淆（`EPIC.md:317`）**：Python `AntiRollBarElement`（z 位移差力偶）与 native 扭杆 ABI 是**不同物理**。缓解 = 选型必须引用 p4-01 的对照表并给理由；不得用「等价于 native」之类的说法掩盖差异。
- **删掉 `upright_L`/`upright_R` 会打断既有装配路径**：硬编码是今天防倾杆唯一的人口（`EPIC.md:136`）。缓解 = 先在新增的 ARB 子系统中建立端口通道（配合 p4-03 的语义化端口），再删除硬编码；删除后用 grep 判据钉住。
- **`builtin.py` 与 `roles.py` 同时被 p2-04 使用**：并行写入会静默覆盖。缓解 = 严格按 `EPIC.md:215` 的顺序，`p2-04` 在前；本行开工前确认 p2-04 已 `DONE`。
- **回退**：本行改动集中在角色表与新增子系统；回退方式 = 撤销本行 diff（角色表 + schema enum + 新增模块 + 测试断言），并恢复 `suspension.py:682-695` 的硬编码跨接。**不得**以重录基线的方式「回退」。

## Done-When

- [ ] `anti_roll_bar` 角色在 `templates/roles.py` 与 F12 全清单（含三份契约 schema 的 `functional_role` enum、`FUNCTIONAL_ROLES`、`SUBSYSTEM_ROLES`、`ALL_SUBSYSTEMS`、`ROLES`、`test_template_model.py:57`）逐项落地，import 期无 `RoleSpecError`，契约包测试接受新角色。
- [ ] 子系统含扭杆与左右小吊杆；选型（刚体 vs 等效扭簧力元）理由与 native 扭杆 ABI 的关系已书面说明（引用 p4-01 的 F11/两套物理对照）。
- [ ] 4 个端口 `chassis_mount_L`、`chassis_mount_R`、`droplink_mount_L`、`droplink_mount_R` 在装配产物中逐字存在。
- [ ] `subsystems/suspension.py` 的 `upright_L`/`upright_R` 硬编码删除，父行 `validation_command` 的 `bash -c '! grep ...'` 退出码 0。
- [ ] `tests/templates/test_template_model.py:57` 的角色断言同步更新并记录理由。
- [ ] 未修改其它行次写范围与任何基线；无新增 skip/xfail；`just check-fast` 通过。
- [ ] 父行 `validation_command` 退出码为 0。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/modeling -q && bash -c '! grep -rn \\\\"upright_L\\\"\" packages/suspension_multibody/src/suspension_multibody/subsystems/suspension.py'"
```
