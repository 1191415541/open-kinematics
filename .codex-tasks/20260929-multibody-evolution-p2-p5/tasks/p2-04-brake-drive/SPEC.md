# SPEC：p2-04 brake 与 drive 子系统改造为力矩元

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-04`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p2-04` 的 `acceptance_criteria` 拆成下面 4 条，逐条可判定：

1. **模板属性槽按路线图 2.1 标准化**：制动 = `piston_area` / `effective_radius` / `friction_coeff` / `rotor_inertia`；驱动 = `gear_ratio` / `efficiency` / `max_torque`（`EPIC.md` 行 24 路线图 2.1 节原文要点；行 243 (a)）。**缺槽必须报错点名**。
2. **`brake.py` 与 `drive.py` 产出力矩元而非幅值字典**：今天两者返回 `wheel_torque_amplitudes()` → `dict[str, tuple[float, ...]]`（`EPIC.md` F1 行 112 给的锚点：`subsystems/brake.py:141`、`drive.py:110`；`brake.py:160-161` 明说幅值交给内核的 `brake_torque` 通道按轮轴向速度反向）。改造后产出 p2-03 接入的力矩元；`wheel_torque_amplitudes()` 的**消费点迁移或删除并登记**（行 243 (b)）。
3. **施力体/反力体路径各有断言**：制动反力传到 `upright` 或卡钳支架；驱动反力传到副车架或车身（`EPIC.md` 行 24 路线图 2.1 节「制动施力于轮毂/车轮刚体、反力传到 `upright` 或卡钳支架……驱动施力于驱动轮、反力传到副车架或车身」；行 243 (c)）。
4. **既有契约测试相应反转/更新且理由登记**：`tests/subsystems/test_brake_subsystem.py` 与 `tests/subsystems/test_drive_subsystem.py`（行 243 (d)；锚点清单见 `EPIC.md` F6 行 122，制动 `:57/:78/:86/:109/:132/:147`，驱动 `:130/:162/:195/:214/:227`）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p2-04` `notes`：「写范围 `templates/roles.py` 与 `templates/builtin.py` 与 `subsystems/brake.py` 与 `subsystems/drive.py`；这两个模板文件是共享注册文件，不得与 p4-02（新增 `anti_roll_bar` 角色）并行」。

- `packages/suspension_multibody/src/suspension_multibody/templates/roles.py` —— **允许**改 `ROLES` 里 `"brake"` / `"drive"` 两个 `RoleSpec` **条目自身的内容**（例如 `outputs` 与 `has_torque_channel`，锚点 `:122-152`；分段归属见 `EPIC.md` 行 227）；**禁止**增删角色名、**禁止**改 `:158-162` 的六角色 import 期硬断言集合（见「禁止触碰」）。
- `packages/suspension_multibody/src/suspension_multibody/templates/builtin.py`：`BRAKE`（`EPIC.md` F1 行 112 的锚点 `:599`）与 `DRIVE`（`:622`）两个模板的属性槽与 `outputs`；`_BRAKE_MOUNTS`（F12 行 139 的锚点 `:594`，用 `for side in ("L","R")` 展开）。
- `packages/suspension_multibody/src/suspension_multibody/subsystems/brake.py`、`subsystems/drive.py`。
- 对应测试：`packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py`、`tests/subsystems/test_drive_subsystem.py`、`packages/suspension_multibody/tests/templates/`。
- 本目录 `raw/`。

## 禁止触碰

- **`templates/roles.py` 的角色集合与 import 期硬断言**：`ROLES`（`:70`）的角色**名字集合**与 `:158-162` 的六角色硬断言（`RoleSpecError`，`EPIC.md` F12 行 139）归 **p4-02**（`EPIC.md` 行 227：「**p2-04 可改 brake/drive 的 `RoleSpec` 内容段**（`ROLES` 里 `"brake"`/`"drive"` 两个条目自身，例如 outputs 与 `has_torque_channel`）；**p4-02 才能改 `ROLES` 的角色集合与 `:158-162` 的六角色 import 期硬断言**，以及三份契约 schema 的 `functional_role` enum。任何行**不得**在 p4-02 之前新增或删除角色名」）。本行**允许**改 `"brake"`/`"drive"` 两个条目自身的内容；**禁止**增删角色名、**禁止**改 `:158-162` 的断言集合。
- **新增 `anti_roll_bar` 角色与模板**：归 **p4-02**（`EPIC.md` 行 215：「p2-04（改造 brake/drive 模板及其 RoleSpec）与 p4-02（新增 `anti_roll_bar` 角色与模板）都改它们 → 串行，p4-02 在 p2-04 之后」；行 263）。本行**不得**新增角色，也不得预置 `anti_roll_bar` 相关字段。
- **三份契约 schema 的 `functional_role` enum**（`template.schema.json:11`、`subsystem.schema.json:8`、`assembly.schema.json:22`，F12 行 139）与 `FUNCTIONAL_ROLES`（`authoring/documents.py:59`）、`SUBSYSTEM_ROLES`（`subsystems/types.py:69`，`+`:76`/`:83` 默认集合）、`ALL_SUBSYSTEMS`（`subsystems/capabilities.py:38`）、`SUBSYSTEM_ROLES`（`subsystems/composition.py:63`）、`ROLES`（`connections/policy.py:48`）：**全部归 p4-02**（行 219 与 F12）——本行不得触碰。
- **`subsystems/element_build.py`**：构造分派段归 p2-03（`EPIC.md` 行 216），本行只**消费**。
- **`preparation/vehicle_dynamic.py`**：力矩段归 p2-05、转向段归 p2-06、轮胎拒绝段归 p4-04（行 217）。本行不得触碰该文件。
- **内核侧**：元素类型与 ABI 归 p2-02；版本常量只有 p2-02 可改（行 225）。
- **不动轮胎力律本构**（行 97 Non-Goals）：「阶段二只改『力矩由谁产生、按什么求值』，不改 PAC2002/Fiala 本构」。
- **不得重录任何基线**（行 228、D5 行 64）；**不得新增 skip/xfail**（行 229）。
- 其它子任务目录（p2-01 ~ p2-03、p2-05、p2-06、p3/p4/p5 各行）、`EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`——不得改。
- `raw/` 中不得放未执行的内容（行 355）。

## 依赖与时机

- `depends_on = p2-03`（`SUBTASKS.csv` `p2-04`）。上游 S1（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 67–75）；**阶段一未完成之前不得置 `IN_PROGRESS`**（行 75）。
- 串行链：`p2-01 → p2-02 → p2-03 → p2-04 → p2-05 → p2-06`（行 203–204）。本行必须在 p2-03 的力矩元构造分派与编译层接入落地之后（否则「产出力矩元」无目标类型可产）。
- **与 p4-02 强制串行**（`EPIC.md` 行 215）：`templates/roles.py` 与 `templates/builtin.py` 是共享注册文件，**不得并行**；**p4-02 在本行之后**。本行必须先落地，p4-02 才能动手改这两个文件。
- 与 p2-05 的关系：p2-05 废除 `_build_wheel_torque_signals` 与 `front_brake_bias`，**依赖本行先把 brake/drive 的力矩产出路径换成力矩元**（`EPIC.md` 行 245 (a)(b)）。
- 与阶段三/四/五：阶段三在阶段一之后、阶段四在阶段三之后（行 211）——**本行不得提前实现 `anti_roll_bar`**（那是阶段四 p4-02）。

## 判据与证据落点

逐条对应 `EPIC.md` 行 243 的 (a)(b)(c)(d)。

1. **(a) 属性槽标准化 + 缺槽报错点名** → `raw/template_slots.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/templates -q`；并构造一份缺槽的模板实例（负例）。
   - 看什么：`templates/builtin.py` 中 `BRAKE`（`:599`）与 `DRIVE`（`:622`）的 `property_slots` **改前/改后字段清单对照**（制动四槽、驱动三槽逐字）；缺槽负例的**报错原文**（必须**点名**缺失槽名，包括异常类型与消息全文），以及该报错**发生在哪一层**（`file:line`）。
   - 落点：`raw/template_slots.md`。
2. **(b) 产出力矩元 + `wheel_torque_amplitudes()` 消费点迁移/删除并登记** → `raw/wheel_torque_amplitudes.md`
   - 跑什么：`grep -rn "wheel_torque_amplitudes" packages/suspension_multibody/`（改造前记全量命中，改造后记剩余）。
   - 看什么：改造前命中清单（`file:line`）；改造后 `subsystems/brake.py:141` 与 `drive.py:110` 的定义是否删除或改签名；`brake.py:160-161` 的「幅值交给内核 `brake_torque` 通道」注释与行为是否随之改变；**每个被迁移/删除的消费点逐条登记**（文件:行 + 迁移到哪 / 为何可删）。
   - 落点：`raw/wheel_torque_amplitudes.md`。
3. **(c) 施力/反力路径断言** → `raw/reaction_paths.md`
   - 跑什么：本行新增的制动/驱动反力断言用例。
   - 看什么：制动反力落到 `upright` 或卡钳支架、驱动反力落到副车架或车身——各自的**断言文件:行 + 断言表达式原文 + 通过输出**；反力体的选择**必须经端口配对**（`EPIC.md` 行 233），不得是硬编码字符串（与 p2-03 判据 2 同口径，若本行触及配对点则同样要 `grep` 零命中证据）。
   - 落点：`raw/reaction_paths.md`。
4. **(d) 契约测试反转/更新 + 理由登记** → `raw/contract_test_updates.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py -q`。
   - 看什么：逐条列出被改动的断言（**改造前原文 → 改造后原文**，含 `file:line`）；每条给**反转/更新的理由**（不是「测试挂了所以改」）。`EPIC.md` F6 行 122 给的两组锚点必须逐条交代：制动 `:57/:78/:86/:109/:132/:147`、驱动 `:130/:162/:195/:214/:227`。另需交代 F6 提到的外部消费点：`tests/vehicle/test_native_vehicle.py:1607/:1721`（制动）与 `:1500/:1630`（驱动）、`tests/cases/test_vehicle_dynamic_contract.py:185`——**这些是否属本行改动范围、是否受影响**都要写明。
   - 落点：`raw/contract_test_updates.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **共享注册文件不得并行**（行 215）：`templates/roles.py` 与 `templates/builtin.py` 由本行与 p4-02 共用 → 串行，**p4-02 在本行之后**。
- **`templates/roles.py` 的分段归属**（行 227）：`ROLES` 的角色**名字集合**与 `:158-162` 的硬断言归 p4-02；本行只能改 `"brake"`/`"drive"` 两个条目自身的内容（如 outputs 与 has_torque_channel）。
- **装配层不得出现按名字猜身份的规则**（行 233）：任何新增跨子系统连接（含制动/驱动反力体）必须经形式化的 `Port` / `Connection` 契约。
- **不得把「简化/专用」的分支写进 role 接口**（行 232）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。
- **分层方向不可逆**（行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层。**本行改 `templates/` 与 `subsystems/`，必须尊重该方向**。
- **不动轮胎力律本构**（行 97）。
- **不得新增内核元素类型**（行 103、行 225）：本行的力矩元**消费** p2-02 的类型，不新增类型。
- **`model_dump(mode="json")` 的产物不得被改变**（行 227）：`api.py:116`/`:284` 用它算 `model_hash`。本行改模板属性槽，须确认 `FrontAxleModel`/`VehicleModel` 的字段形状未变。
- **基线不得重录**（行 228、D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；其余基线若确需变化，逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径），质量与质心相同**不足以**证明等价。
- **不得新增 skip/xfail**（行 229）。
- **每步落地后必须重跑**（行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。**本行触及求解路径（力矩产生路径），故数值门三项必须跑**。

## 风险与回退

- **共享注册文件冲突**（行 215）：本行与 p4-02 都改 `templates/roles.py` 与 `templates/builtin.py`。缓解：**本行先行**、p4-02 后行；本行改动范围**限定在 `BRAKE`/`DRIVE` 段**（`roles.py` 里 `"brake"`/`"drive"` 两个条目、`builtin.py:594/:599/:622`），并在 `raw/template_slots.md` 用 `git diff --stat` 与分段行号自证未越界到角色集合与 `:158-162` 的断言。
- **`wheel_torque_amplitudes()` 的消费点散落**（判据 2）：`EPIC.md` F1 行 112 指出 `brake.py:160-161` 明说幅值交给内核 `brake_torque` 通道按轮轴向速度反向；外部还有 `cases/vehicle_dynamic.py:579/582` 的契约表（归 p2-05）。缓解：本行先跑 `grep` 拿**全量命中清单**再改，逐条登记迁移/删除；**不属本行写范围的消费点（如 `cases/vehicle_dynamic.py`）只登记、不修改**。
- **契约测试反转的理由必须可追溯**（行 243 (d)）：缓解 = `raw/contract_test_updates.md` 逐条「改前原文 → 改后原文 → 理由」；**不得只写「测试挂了所以改」**。
- **`tests/vehicle/test_native_vehicle.py` 与 `tests/cases/test_vehicle_dynamic_contract.py` 可能被连带影响**（F6 行 122 列出的外部消费点）：本行需**实测并说明**这些点是否受影响；受影响的改动若落在 p2-05 的写范围（`cases/vehicle_dynamic.py`），登记给 p2-05，不在本行改。
- **既有失败**（行 320）：起点以 p2-01 `raw/baseline_notes.md` 为准；任何新增失败阻断完成，不相关既有失败独立列明。

## Done-When

- [ ] `BRAKE` 模板的 `property_slots` 含 `piston_area` / `effective_radius` / `friction_coeff` / `rotor_inertia`；`DRIVE` 含 `gear_ratio` / `efficiency` / `max_torque`（`raw/template_slots.md` 给改前/改后字段清单对照）。
- [ ] 缺槽时**报错点名**（含异常类型、消息全文、发生层次 `file:line`）。
- [ ] `subsystems/brake.py` 与 `drive.py` 产出**力矩元**而非 `wheel_torque_amplitudes()` 的幅值字典；该函数的每个消费点已迁移或删除并逐条登记（`raw/wheel_torque_amplitudes.md`）。
- [ ] 制动反力传到 `upright` 或卡钳支架、驱动反力传到副车架或车身，**各有断言**（`raw/reaction_paths.md`），且反力体经端口配对决定、无按名字猜身份的硬编码。
- [ ] `test_brake_subsystem.py` 与 `test_drive_subsystem.py` 的改动逐条有「改前 → 改后 → 理由」；F6 行 122 的两组锚点（制动 `:57/:78/:86/:109/:132/:147`、驱动 `:130/:162/:195/:214/:227`）逐条交代（`raw/contract_test_updates.md`）。
- [ ] 未增删任何角色名、未改 `templates/roles.py:158-162` 的断言集合、未动三份契约 schema 的 `functional_role` enum 与 `FUNCTIONAL_ROLES` / `SUBSYSTEM_ROLES` / `ALL_SUBSYSTEMS` / `ROLES`（`connections/policy.py`）；`ROLES` 里 `"brake"`/`"drive"` 两个条目**若有改动须逐条给「改前 → 改后 → 理由」**（`git diff --stat` 自证，`EPIC.md` 行 227）。
- [ ] `just check-fast` 绿；触及求解路径故 `just gate-numeric` 三项全绿；`tests/architecture` 绿；无新增 skip/xfail；未重录任何基线。
- [ ] 与 p4-02 的串行关系已在 PROGRESS 登记（本行先落地，p4-02 方可在同一批文件上动手）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_brake_subsystem.py packages/suspension_multibody/tests/subsystems/test_drive_subsystem.py packages/suspension_multibody/tests/templates -q
```
