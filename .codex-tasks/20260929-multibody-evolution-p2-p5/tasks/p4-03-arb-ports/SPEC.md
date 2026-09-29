# SPEC：p4-03 悬架 arb_mount 语义端口与配对插接

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-03`（`task_dir = tasks/p4-03-arb-ports`，`depends_on = p4-02`）。
> 父 Epic 内对应段落：Goal G5（`EPIC.md:87`）、阶段四原文要点（`EPIC.md:36-42`，路线图 191–196 行）、p4-03 验证协议（`EPIC.md:265`）、端口现状事实 F13（`EPIC.md:141`）、`_BODY_ALIASES` 修正 F16（`EPIC.md:147`）、冻结约束（`EPIC.md:233`）。

## Goal

把父行 `acceptance_criteria` 拆成编号分条，逐条可判定。

1. **悬架子系统声明语义化端口（含 `arb_mount_L/R`），使防倾杆可插**。今天端口是装配期按 body 合成（`EPIC.md:141` F13：`builtin.py:397 DOUBLE_WISHBONE` 的 `ports`/`needs` 皆为空元组；`subsystems/si_assembly.py:71 _ports_for_bodies` 给每个 body 一个 `role="body"` port，给 upright 额外一个 `role="wheel_centre"`；`:104 _wheel_centre_needs`）。判据 = 悬架子系统的**声明**里出现语义化端口（至少 `arb_mount_L` 与 `arb_mount_R`），装配产物的端口集合里能看到它们，且防倾杆（p4-02 交付的 4 端口子系统）能与之配对上。
2. **由总成配对段决定插接位置：双叉臂插下臂、麦弗逊插减振筒外筒，两者各有断言**。插接位置由总成**配对段**（阶段一 02 交付的显式接口配对，语义见 `EPIC.md:265(d)` 的 `match_requirements`）决定，不由代码里的硬点名字决定。判据 = 两个用例（双叉臂 → 下臂；麦弗逊 → 减振筒外筒）各自跑通，断言落在**配对段声明**所指定的实体上。配对消费函数的确切 `file:line` **锚点由本行开工时复核（见 EPIC.md 的 F 编号：本行相关事实为 F13/F16，均未给配对消费点的路径锚点）**。
3. **`subsystems/geometry.py` 的 `_BODY_ALIASES` 相应收口或删除并登记**。按 `EPIC.md:147` F16 的修正项：`subsystems/geometry.py:226 _BODY_ALIASES`（含 `"wheel": "upright"`，消费 `:258/265`）在本行造语义化端口时**收口或删除**，并登记到 `PROGRESS.md`。判据 = 该别名表被缩小/删除，`grep _BODY_ALIASES` 在装配路径无命中或仅剩已登记项；登记文字写清「为什么现在能收口」。
4. **缺失配对时行为有定义（报错点名或按角色唯一匹配，沿用 `match_requirements` 语义）**。判据 = 一条**负例**用例：构造缺少配对的输入，逃逸行为是「报错点名」或「按角色唯一匹配成功」二者之一，**且该行为被断言钉住**（不是未定义行为）。沿用阶段一 02 的 `match_requirements` 语义，不新造第二条推断路径（`EPIC.md:233`）。

## 写范围（允许改的路径）

照抄父行 `notes`：**写范围 `subsystems/suspension.py` 的端口声明段 与 `subsystems/si_assembly.py` 的端口合成段 与 `subsystems/geometry.py`。F16 修正项：`_BODY_ALIASES`（含 wheel→upright 别名）在本行造语义化端口时收口 不单独扩范围。**

展开为：

- `packages/suspension_multibody/src/suspension_multibody/subsystems/suspension.py`（**端口声明段**）
- `packages/suspension_multibody/src/suspension_multibody/subsystems/si_assembly.py`（**端口合成段**：`_ports_for_bodies`、`_wheel_centre_needs`）
- `packages/suspension_multibody/src/suspension_multibody/subsystems/geometry.py`（`_BODY_ALIASES` 收口或删除）
- 悬架模板的端口/needs 声明（若声明落在 `templates/builtin.py`，只动**端口/needs 段**；该文件是共享注册文件，见「禁止触碰」）
- 本行新增的端口/配对测试：`packages/suspension_multibody/tests/subsystems/`、`tests/connections/`、`tests/authoring/`
- `raw/` 证据与 `PROGRESS.md` 的登记段

## 禁止触碰

- `.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`
- 其它行次的目录与 `raw/`（`tasks/p4-01-freeze/`、`tasks/p4-02-arb-subsystem/`、`tasks/p4-04-wheel-unify/`、`tasks/p4-05-acceptance/`，以及全部 p2-*/p3-*/p5-*）
- **p4-02 的写范围**：`templates/roles.py`、`templates/builtin.py` 的**角色/模板注册段**、`subsystems/suspension.py` 的 `upright_L`/`upright_R` 硬编码删除段（p4-02 已做完，本行不得回改）、三份契约 schema 的 `functional_role` enum（`EPIC.md:263`）
- **p4-04 的写范围**：`subsystems/wheel.py`、`subsystems/assembler.py` 的轮胎过滤段、`subsystems/element_build.py` 的轮胎段、`preparation/vehicle_dynamic.py` 的轮胎拒绝段、`authoring/solver.py`（`EPIC.md:267`）
- **阶段一的写范围**：`subsystems/rig_link.py`（试验台非侵入归阶段一 05）、阶段一 02 交付的配对机制与其消费函数（本行只**消费**；其确切 `file:line` **锚点由本行开工时复核**，`EPIC.md` 未给路径锚点）
- `packages/suspension_kernel/**`（不改内核；`EPIC.md:225`）
- 任何基线文件（`EPIC.md:228`）
- `raw/` 内不得放入任何虚构结果；规划轮 `raw/` 必须为空

## 依赖与时机

- `depends_on = p4-02`。防倾杆子系统（4 端口）由 p4-02 交付；本行才可能让它「插得上」。
- 阶段四串行主线 p4-01 → p4-02 → p4-03 → p4-04 → p4-05（`EPIC.md:207`）；阶段四整段在**阶段三完成后**开工（`EPIC.md:211`）。
- 全局前置 `S1`：阶段一 Epic 的 01–07 全部 `DONE`（含 02 的配对机制与 03 的通用装配引擎）；**阶段一未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:75`、`EPIC.md:71`）。
- 本行的配对消费依赖阶段一 02 的 `match_requirements`；引擎依赖阶段一 03 的条目清单驱动装配（`EPIC.md:71`）。
- 本行与 p4-04 串行：p4-04 的 `depends_on = p4-03`（`SUBTASKS.csv`）。

## 判据与证据落点

逐条对应 `EPIC.md:265`「p4-03（arb_mount 端口 + 配对插接）」的 (a)–(d)：

- **(a) 悬架声明语义化端口（至少 `arb_mount_L/R`），防倾杆可插** → 读装配产物的端口集合并与 p4-02 的 4 端口比对；命令、退出码与端口清单写入 `raw/ports_declared.md`。
- **(b) 配对段决定插接位置；双叉臂插下臂、麦弗逊插减振筒外筒各一断言** → 跑两个用例（`tests/subsystems/`、`tests/authoring/`、`tests/connections/`），把配对段声明原文、装配产物与断言点写入 `raw/pairing_cases.md`。
- **(c) `_BODY_ALIASES` 收口或删除并登记** → 收口前后 `grep -n _BODY_ALIASES` 的命中清单写入 `raw/body_aliases.md`；**收口理由**写进 `PROGRESS.md`（F16 修正项，`EPIC.md:147`）。
- **(d) 缺失配对时行为有定义** → 负例用例的命令与输出（报错点名原文或唯一匹配的判定过程）写入 `raw/missing_pairing.md`。
- 命令与退出码一并记录（只记**已执行**的结果）。

## Constraints（冻结约束）

- **不得出现按名字猜身份的规则**（`EPIC.md:233`）：本行是这条约束的**直接执行行**——任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约。`arb_mount` 端口必须来自**声明**（不是运行期按 body 名拼出来的），插接位置必须来自**总成配对段**（不是代码里认 `LOWER_ARM`/`STRUT` 之类的名字）。这也是 `_BODY_ALIASES` 要收口的原因（`EPIC.md:147`）。
- **沿用阶段一 02 的 `match_requirements` 语义**：跨子系统插接只经该通道，**不得另立第二条推断路径**（`EPIC.md:265(d)` 的口径、阶段一 03 SPEC 的约束同源）。
- **两套 ARB 物理不得混淆**（`EPIC.md:137`）：本行只处理**端口与插接**，不实现力律；不得借端口改造顺手改 `AntiRollBarElement` 的物理语义或对接 native 扭杆 ABI。
- **基线不得重录**（`EPIC.md:228`，D5 见 `EPIC.md:64`）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门。
- **分层方向不可逆**（`EPIC.md:226`）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。
- **不得把「简化/专用」的分支写进 role 接口**（`EPIC.md:232`）。
- **不得新增 skip/xfail**（`EPIC.md:229`）；`tests/adams` 的环境 skip 是既有的，不得增长。
- 契约 schema 若必须同步（`assembly.schema.json` 的配对段字段），**不得与阶段一 02/03 的改动并行**（`EPIC.md:218`）；本行的 `functional_role` enum 不动（归 p4-02）。
- 每步落地后重跑 `just check-fast`；改结构后加跑 `tests/architecture`（`EPIC.md:230`）。
- 临时脚本与中间日志写会话 scratch；`raw/` 只存**已执行**的证据。

## 风险与回退

- **端口从「运行期合成」改为「声明」会改变既有装配产物**：`si_assembly.py:71 _ports_for_bodies` 今天给每个 body 一个 port（`EPIC.md:141`）。缓解 = 新增语义化端口**不删除**既有 body port 的可用性，或删除时逐项登记产物差异；每步跑 `tests/subsystems`、`tests/connections`、`tests/authoring` 与契约包测试。
- **`_BODY_ALIASES` 收口可能打断 `geometry.py` 的既有消费点**（消费点 `:258/265`，`EPIC.md:147`）。缓解 = 先收窄到「无调用者」或「调用者已改用端口」，再删除；两侧都无命中才删。
- **缺少配对的默认行为选择会决定后续可扩展性**：报错点名（严格）与按角色唯一匹配（宽松）二选一。缓解 = 沿用 `match_requirements` 既有语义，不新造规则；把选择与理由写在 `raw/missing_pairing.md`。
- **与 p4-02 的分段边界**：p4-02 负责「造出防倾杆的 4 端口」，本行负责「让悬架能接受它」。缓解 = 本行开工前先跑 p4-02 的验收命令确认 4 端口已声明，边界不符即停工回报，不自行扩范围。
- **回退**：回退本行 diff（端口声明段 + 端口合成段 + `_BODY_ALIASES`），恢复 `_ports_for_bodies` 的合成行为与配对消费点。**不得**以重录基线方式「回退」。

## Done-When

- [ ] 悬架子系统**声明**了语义化端口（至少 `arb_mount_L`/`arb_mount_R`），装配产物端口集合可见，p4-02 交付的防倾杆能与之配对。
- [ ] 一份总成文件的**配对段**决定插接位置；双叉臂（插下臂）与麦弗逊（插减振筒外筒）两个用例各有断言且通过。
- [ ] `_BODY_ALIASES` 已收口或删除，前后 grep 清单与理由已登记。
- [ ] 缺失配对时的行为被定义且有一条负例断言（报错点名或按角色唯一匹配，沿用 `match_requirements` 语义）。
- [ ] 未修改 p4-02 / p4-04 / 阶段一写范围与任何基线；无新增 skip/xfail；`just check-fast` 通过。
- [ ] 父行 `validation_command` 退出码为 0。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/connections packages/suspension_multibody/tests/authoring -q && uv run --no-sync pytest packages/suspension_contracts/tests -q
```
