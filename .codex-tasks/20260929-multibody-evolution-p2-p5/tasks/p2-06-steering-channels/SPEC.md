# SPEC：p2-06 分布式转向通道与转向分配器

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-06`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p2-06` 的 `acceptance_criteria` 拆成下面 5 条，逐条可判定：

1. **解除两层后轮转向限制，且 `must be true` 校验无命中**：`preparation/vehicle_dynamic.py:205-211 _validate_steering_topology` 的抛错（调用点 `:222`）与 `authoring/vehicle.py:112-119` 的强制写死（`:119` 为写死点）两层都要解除（`EPIC.md` F4 行 118；行 247 (a)；行 291）。判据：`grep` 在准备层无「必须为真」的校验（G2 判据行 81）。
2. **转向从单例改为通道列表，且单通道声明向后兼容**：`schema/vehicle.py:255` 今天只有单个 `steering: SteeringSystemSpec`（`:143-153` 只有单个 `rack_body`/`actuator_body`/`actuator_reaction_body`，F4 行 118）；改为**通道列表**；**单通道声明仍等价于旧单例，产物逐项一致**（`EPIC.md` 行 247 (b)）。
3. **转向分配器三种分配律各有断言**：阿克曼、4WS（高速同向 / 低速对向）、多轴随动；且**相同方向盘输入下**各通道转角符合分配律（`EPIC.md` 行 25 路线图 2.3 节要求、行 247 (c)、行 290）。
4. **`cases/vehicle_kc.py` 的准备期删除改为「只增不删」或按裁决登记**：`cases/vehicle_kc.py:128-136` 今天在准备期过滤删除 `steering_actuator`（`EPIC.md` F5 行 120，路线图 `:135` 锚点**未过期**）；改为「只增不删」的边界驱动，与阶段一 05 的试验台非侵入同口径（`EPIC.md` 行 247 (d)、行 313）。
5. **一份两通道总成装配并跑通一次 study**：前 + 后两通道（`EPIC.md` G2 判据行 81、Done-When (b) 行 290）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p2-06` `notes`：「写范围 `schema/vehicle.py` 的转向段、`preparation/vehicle_dynamic.py` 的转向段、`authoring/vehicle.py`、新增的转向分配器模块、`cases/vehicle_kc.py` 的准备期删除段」。

- `packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py` 的**转向段**——F4 行 118 的锚点 `:255`（`steering: SteeringSystemSpec`）与 `:143-153`（`SteeringSystemSpec` 的字段）。**只改转向段**。
- `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py` 的**转向段**——F4 行 118 的锚点 `:205-211`（`_validate_steering_topology`）与调用点 `:222`。
- `packages/suspension_multibody/src/suspension_multibody/authoring/vehicle.py`（F4 行 118 的锚点 `:112-119`，`:119` 为强制写死点）。
- **新增的转向分配器模块**（Steering Allocator）——`EPIC.md` 行 25 路线图 2.3 节要求「准备层引入转向分配器，按方向盘转角、车速与模式解算各通道输入，原生支持阿克曼、4WS 高速同向/低速对向、多桥重卡随动转向」。模块落点由本行决定，**必须遵守分层方向**（行 226：`modeling -> templates -> subsystems -> preparation/studies -> cases`）——候选落点属准备层或其上游，具体由本行开工时确认（`EPIC.md` 未给该新模块的路径锚点）。
- `packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py` 的**准备期删除段**（F5 行 120 的 `:128-136`）。
- 对应测试：`packages/suspension_multibody/tests/authoring/`、`tests/subsystems/`、`tests/cases/`、`packages/suspension_contracts/tests/`。
- 本目录 `raw/`。

## 禁止触碰

- **`preparation/vehicle_dynamic.py` 的力矩段**（归 p2-05）与**轮胎拒绝段**（归 p4-04，F15 行 145 的 `:55/:900`）：`EPIC.md` 行 217 的分段裁决——「`preparation/vehicle_dynamic.py` 被三行触及：p2-05（废除预采样与 bias）、p2-06（转向）、p4-04（轮胎拒绝段）→ 串行」。本行只动转向段。
- **`templates/roles.py` 的角色表**（含 `:70 ROLES` 与 `:158-162` 硬断言，F12 行 139）与 `templates/builtin.py`：归 p4-02，p2 的任何行不得改（`EPIC.md` 行 219、行 215）。
- **`subsystems/steering.py` 的角色与端口语义**：`EPIC.md` F4 行 118 给的锚点 `:194/:234`（由 `model.rack_fixed_to_chassis` 在 `WeldJoint`/`PrismaticJoint` 之间二选一）——本行**消费**下层行为，但该文件的角色表相关内容归 p4-02。**若本行确需改 `subsystems/steering.py` 的非角色段**，必须在 PROGRESS 登记，并确认不与 p2-04/p4-02 的写范围相交。
- **`subsystems/element_build.py`**：构造分派段归 p2-03（行 216）。
- **三份契约 schema 的 `functional_role` enum**（`template.schema.json:11`、`subsystem.schema.json:8`、`assembly.schema.json:22`，F12 行 139）与 `FUNCTIONAL_ROLES`、`SUBSYSTEM_ROLES`、`ALL_SUBSYSTEMS`、`SUBSYSTEM_ROLES`（`composition.py`）、`ROLES`（`connections/policy.py:48`）：**全部归 p4-02**（行 219）——本行不得触碰。**注意**：若本行的通道列表需要在 `assembly.schema.json` 的**配对段/放置段之外**新增字段，必须先确认该段归属（放置段属阶段一 03、配对段属阶段一 02，行 218）并登记串行关系。
- **内核侧**（元素类型与 ABI）归 p2-02；**版本常量只有 p2-02 可改**（行 225）。
- **阶段一 05 的写范围**：`subsystems/rig_link.py`、`rigs/**`（`EPIC.md` 行 71–72 与阶段一 Epic 的 05）；本行**不得**在试验台层做同样的删除处理——同口径但不同文件（行 313：「两处改动分属不同 Epic，必须点明关系避免重复」）。
- **不动轮胎力律本构**（行 97）。
- **不得重录任何基线**（行 228、D5 行 64）；**不得新增 skip/xfail**（行 229）。
- 其它子任务目录（p2-01 ~ p2-05、p3/p4/p5 各行）、`EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`——不得改。
- `raw/` 中不得放未执行的内容（行 355）。

## 依赖与时机

- `depends_on = p2-05`（`SUBTASKS.csv` `p2-06`）。上游 S1（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 67–75）；**阶段一未完成之前不得置 `IN_PROGRESS`**（行 75）。
- 串行链：`p2-01 → p2-02 → p2-03 → p2-04 → p2-05 → p2-06`（行 203–204）。本行是阶段二的**收尾行**。
- **`preparation/vehicle_dynamic.py` 三段串行**（`EPIC.md` 行 217）：**本行（转向段）在 p2-05（力矩段）之后**，**p4-04（轮胎拒绝段）在本行之后**。三段不得并行。
- **与阶段一 05 同口径**（行 313）：「`cases/vehicle_kc.py:128-136` 的准备期删除与阶段一的试验台非侵入是同一类问题。缓解：沿用阶段一『只增不删』的口径；**两处改动分属不同 Epic，必须点明关系避免重复**」。
- 与阶段一 03 的关系：本行的通道列表若触及 `assembly.schema.json` 的放置段或其相邻段，必须与阶段一 03 的改动**串行**（行 218）。
- 与阶段四的关系：阶段四在阶段三之后（行 215）——**本行不得提前实现 `anti_roll_bar`**，也不得增删任何角色名。
- **本行是 G2 的实现行**：G2 判据（行 81）的三项——两通道总成装配跑通、4WS 与阿克曼在相同方向盘输入下符合分配律、`rack_fixed_to_chassis` 无「必须为真」校验——全部由本行交证据；p3/p4/p5 各行不得代做。

## 判据与证据落点

逐条对应 `EPIC.md` 行 247 的 (a)(b)(c)(d)(e)。

1. **(a) 两层限制解除 + grep 零命中** → `raw/steering_restriction_removed.md`
   - 跑什么：`grep -rn "rack_fixed_to_chassis" packages/suspension_multibody/src`、`grep -rn "must be true" packages/suspension_multibody/src`（口径见 G2 判据行 81 与行 247 (a)）；并跑 p2-01 交付的负例复现（`tasks/p2-01-freeze/raw/rear_steer_refusal.md` 的两层拒绝原文作为改造前基准）。
   - 看什么：改造前两层的**拒绝原文**（引用 p2-01 证据文件）与改造后的行为对照；两条 grep 的退出码与输出原文（准备层不得再有「必须为真」校验）；`authoring/vehicle.py:112-119` 的强制写死段改前/改后原文对照。
   - 落点：`raw/steering_restriction_removed.md`。
2. **(b) 通道列表 + 单通道向后兼容** → `raw/steering_channels.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/authoring packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/cases -q`；另跑 `uv run --no-sync pytest packages/suspension_contracts/tests -q`。
   - 看什么：`schema/vehicle.py` 转向段（`:255` 与 `:143-153`）的**改前/改后字段形状对照**；单通道声明（= 旧单例）的**产物逐项对照**（这是向后兼容硬门，必须给逐项结论而非「测试通过」）；`tests/authoring/test_vehicle_assembly_documents.py:218-220`（F6 行 122 指出它断言 `model.rear_axle.rack_fixed_to_chassis is True`）的改动「改前原文 → 改后原文 → 理由」。
   - 落点：`raw/steering_channels.md`。
3. **(c) 分配器三种分配律断言** → `raw/allocator_assertions.md`
   - 跑什么：本行新增的分配器用例（阿克曼 / 4WS 高速同向 / 4WS 低速对向 / 多轴随动）。
   - 看什么：**分配律的声明**（每通道转角对方向盘转角、车速与模式的解析关系，写明公式/规则）与**每条断言的文件:行 + 断言表达式原文 + 通过输出**；**相同方向盘输入下**各通道转角符合分配律的数值对照表（不是「用例通过」这类弱证据）。`EPIC.md` F6 行 122 指出「**没有任何 4WS / 多通道转向用例**（`grep "4WS|four.wheel.steer"` 零命中）」——本行必须补上。
   - 落点：`raw/allocator_assertions.md`。
4. **(d) `cases/vehicle_kc.py` 准备期删除 → 只增不删** → `raw/vehicle_kc_boundary.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/cases -q`；并对照阶段一 05 的「只增不删」口径。
   - 看什么：`cases/vehicle_kc.py:128-136`（F5 行 120，路线图 `:135` 锚点**未过期**）的**改前/改后原文对照**（今天是把 `type == "steering_actuator"` 的元素从 `document["elements"]` 里过滤掉）；改为边界驱动后的判据证据；**与阶段一 05 的关系点名**（行 313：分属不同 Epic、同口径、不得重复改同一处）。
   - 落点：`raw/vehicle_kc_boundary.md`。
5. **(e) 两通道总成装配 + 跑通 study** → `raw/two_channel_assembly.md`
   - 跑什么：一份**两通道（前 + 后）总成文件**装配并跑通一次 study。
   - 看什么：总成文件的**通道声明片段原文**、装配产物中两个通道的实体清单（逐通道列出体/约束/执行器）、study 实跑命令与退出码、以及**每个通道的转角读数**（这是「两通道真的都通了」的证据）。`subsystems/steering.py:194/234`（F4 行 118）在 `WeldJoint`/`PrismaticJoint` 之间的二选一在**后通道自由**时的行为必须写明。
   - 落点：`raw/two_channel_assembly.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **分层方向不可逆**（行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。**新增的转向分配器必须落在正确的层**——它的输入是方向盘转角、车速与模式，输出是各通道输入，属准备层功能；不得放进 `modeling/`，也不得让低层反向依赖它。
- **不得把「简化/专用」的分支写进 role 接口**（行 232）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。**本行不得用「通道数 == 2 特判」之类的分支替代通用机制**。
- **装配层不得出现按名字猜身份的规则**（行 233）：任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约。**通道与车轴/悬架的挂接必须经契约**，不得按名字（如「rear」字面量）决定通道归属。
- **角色名集合与硬断言归 p4-02**（`EPIC.md` 行 227）：`templates/roles.py` 的 `ROLES` 角色集合与 `:158-162`、三份契约 schema 的 `functional_role` enum 全部归 p4-02；**p2 的任何行不得增删角色名**。
- **内核 ABI 单点提交**（行 225）：只有 p2-02 可改版本常量。本行**不得**触碰 `mb_config/version.hpp` 与 `kernel/native.py`。
- **`model_dump(mode="json")` 的产物不得被改变**（行 227）：`api.py:116`/`:284` 用它算 `model_hash`。**本行改 `schema/vehicle.py` 的转向段，最容易踩这条**——通道列表改造后，单通道声明的 `model_dump(mode="json")` 形状必须与改造前**逐项一致**（判据 2 的向后兼容硬门即由此而来）。
- **基线不得重录**（行 228、D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；其余基线若确需变化，逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径），质量与质心相同**不足以**证明等价。**本行在 `cases/vehicle_kc.py` 准备期删除段上的改动直接触及 K 台路径**，必须跑 `kc_parity_check.py --check --actual-dir`（口径见下）与 `dynamic_hash_sentinel.py --check`。
- **`kc_parity_check.py` 的口径**（行 231）：不带 `--actual-dir` 时拿冻结快照与自身比较（恒过），**不构成证据**——本行若做 K/C 对标，必须先跑 `kc_native_probe.py` + `kc_native_c_probe.py` 再带 `--actual-dir artifacts/kc-native-probe` 判定。
- **不得新增 skip/xfail**（行 229）。
- **每步落地后必须重跑**（行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。**本行触及准备与 case 层，`just gate-numeric` 三项必须全绿**。
- **不动轮胎力律本构**（行 97）。

## 风险与回退

- **与阶段一 05 同口径但属不同 Epic**（行 313）：`cases/vehicle_kc.py:128-136` 的准备期删除与阶段一的试验台非侵入是同一类问题。缓解：沿用阶段一「只增不删」的口径；**两处改动分属不同 Epic，必须点明关系避免重复改同一处**——本行在 `raw/vehicle_kc_boundary.md` 写明与阶段一 05 的边界。
- **`model_dump(mode="json")` 形状漂移是最大连锁风险**（行 227）：`schema/vehicle.py` 转向段从单例改通道列表，若单通道声明的 `model_dump` 形状变了，`api.py:116`/`:284` 的 `model_hash` 随之改变。缓解：判据 2 的**单通道产物逐项对照**为硬门；**形状变化即回退**，或按 D5 逐项登记。
- **无既有 4WS 用例**（F6 行 122：「没有任何 4WS / 多通道转向用例（`grep "4WS|four.wheel.steer"` 零命中）」）：**风险在「无用例保护」**。缓解：本行必须从零补阿克曼/4WS 高速同向/4WS 低速对向/多轴随动四类断言（判据 3）；既有转向用例（F6 行 122 的 `tests/subsystems/test_steering_can_be_absent.py`、`tests/api/test_no_steering_shrinks_rack.py`、`tests/subsystems/test_assembly_matches_snapshot.py:40`）**必须继续通过**，任何改动都按「改前 → 改后 → 理由」登记。
- **多轴随动只交付机制 + 最小端到端用例**（行 98 Non-Goals）：「不实现具体车型物理：4WS 与多轴转向只交付机制 + 最小端到端用例」。**不得**扩展成完整整车对标。
- **`authoring/vehicle.py:112-119` 的写死是为了兼容既有文档**（F4 行 118 原文注释：「leaving the rear rack free would describe a four-wheel-steered car that the document does not」）：解除写死后**既有文档的导出产物可能变化**。缓解：以 p2-01 的负例证据为改造前基准；既有文档产物若变化，逐项登记理由（判据 1 的证据文件）。
- **契约 schema 的串行约束**（行 218）：本行若需改 `assembly.schema.json`（放置段或配对段相邻），必须与阶段一 02/03 串行；**不得并行**。
- **既有失败**（行 320）：起点以 p2-01 `raw/baseline_notes.md` 为准；任何新增失败阻断完成，不相关既有失败独立列明。

## Done-When

- [ ] `preparation/vehicle_dynamic.py:205-211`（调用点 `:222`）与 `authoring/vehicle.py:112-119`（`:119`）**两层限制均解除**；`grep -rn "must be true" packages/suspension_multibody/src` 在准备层零命中（`raw/steering_restriction_removed.md` 含改前拒绝原文与改后对照）。
- [ ] `schema/vehicle.py:255` 的转向从单例改为**通道列表**（`:143-153` 的字段相应扩展）；**单通道声明的产物与改造前逐项一致**（含 `model_dump(mode="json")` 形状）（`raw/steering_channels.md`）。
- [ ] 转向分配器实现，**阿克曼 / 4WS 高速同向 / 4WS 低速对向 / 多轴随动**四类各有断言；**相同方向盘输入下**各通道转角符合分配律（数值对照表）（`raw/allocator_assertions.md`）。`grep "4WS|four.wheel.steer"` 不再零命中（F6 行 122）。
- [ ] `cases/vehicle_kc.py:128-136` 的准备期 `steering_actuator` 删除改为**只增不删**的边界驱动（或按裁决登记）；与阶段一 05 的关系已在证据中**点名**（`raw/vehicle_kc_boundary.md`）。
- [ ] 一份**两通道（前 + 后）总成文件**装配成功并跑通一次 study，两个通道各有转角读数（`raw/two_channel_assembly.md`）。
- [ ] 既有转向用例继续通过：`tests/subsystems/test_steering_can_be_absent.py`、`tests/api/test_no_steering_shrinks_rack.py`、`tests/subsystems/test_assembly_matches_snapshot.py:40`、`tests/authoring/test_vehicle_assembly_documents.py:218-220`（F6 行 122）；改动逐条「改前 → 改后 → 理由」登记。
- [ ] `just check-fast` 绿；`just gate-numeric` 三项全绿（K 台路径被触及）；`tests/architecture` 绿；`packages/suspension_contracts/tests` 绿。
- [ ] 无新增 skip/xfail；未重录任何基线；`kc_baseline` 逐字节未变（用带 `--actual-dir` 的口径判定，见 Constraints）。
- [ ] 未触碰 `preparation/vehicle_dynamic.py` 的力矩段（p2-05）与轮胎拒绝段（p4-04）、`templates/roles.py` 的角色表、三份契约 schema 的 `functional_role` enum、`subsystems/rig_link.py` 与 `rigs/**`（阶段一 05）、内核版本常量（`git diff` 分段自证）。
- [ ] G2 的三项判据（`EPIC.md` 行 81）全部可指本行证据。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/authoring packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/cases -q && uv run --no-sync pytest packages/suspension_contracts/tests -q
```
