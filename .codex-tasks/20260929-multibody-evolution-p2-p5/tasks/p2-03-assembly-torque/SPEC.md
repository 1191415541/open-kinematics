# SPEC：p2-03 多体层力矩元接入

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-03`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p2-03` 的 `acceptance_criteria` 拆成下面 4 条，逐条可判定：

1. **力矩元素在多体层落地**：在 `modeling/primitives/` 有元素声明；在 `subsystems/element_build.py` 的**构造分派段**（`EPIC.md` 行 241 (a) 给的锚点是 `:66-71`）有分支；**能编进内核输入**（即编译层能把该元素写进内核输入，与 p2-02 新增的 ABI 编组对接）。
2. **施力体与反力体由端口配对决定**：施力体/反力体的选择必须来自形式化的 `Port` / `Connection` 契约，**不得硬编码 `upright`/`chassis` 字符串**（`EPIC.md` 行 233 冻结约束：「装配层不得出现按名字猜身份的规则」；行 241 (b)；G2 判据行 81）。
3. **最小装配体跑通并读到力矩贡献**：一份**含一个力矩元**的最小装配体跑通一次，且能读到该力矩元对结果的贡献（`EPIC.md` 行 241 (c)）。
4. **既有元素类型装配产物不变**：以 p2-01 的路径快照为对照，既有元素类型的装配产物不变（`EPIC.md` 行 241 (d)）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p2-03` `notes`：「写范围 `subsystems/element_build.py` 的构造分派段与 `modeling/primitives/`（新增元素声明）；轮胎段归 p4-04 不得并行」。

- `packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py` 的**构造分派段**——`EPIC.md` 行 216 的分段裁决：「本节指定 `element_build.py` 的**构造分派段**归 p2-03、**轮胎段**归 p4-04」。
- `packages/suspension_multibody/src/suspension_multibody/modeling/primitives/`（新增力矩元声明；参照现有 `modeling/primitives/elements.py` 的形态，如 `EPIC.md` F11 行 136 点到的 `:593 AntiRollBarElement` 与 F15 行 145 点到的 `:567 VerticalTireElement`——**只参照，不修改它们**）。
- 编译层：使新元素能编进内核输入。`SUBTASKS.csv` 的 `notes` 未列具体编译层路径——**具体落点由本行开工时复核**（`EPIC.md` F1 行 111 指出编译消费点是 `cases/vehicle_dynamic.py:579/582` 的契约表交内核按样本消费；本行需要的编译层入口锚点 `EPIC.md` 未给）。
- 新增测试（判据 3 要求「跑通一次」，必然需要用例）：`SUBTASKS.csv` 的 `notes` 未列测试目录，**落点按项目现有布局在开工时确认（`tests/modeling/` 与 `tests/subsystems/` 是候选，见本行 `validation_command`）**。
- 本目录 `raw/`。

## 禁止触碰

- **`subsystems/element_build.py` 的轮胎段**：归 **p4-04**（`EPIC.md` 行 216：「轮胎段归 p4-04」；行 267 (c)）。本行只改构造分派段。
- **`templates/roles.py` 的角色表**（含 `ROLES` 与 `:158-162` 的 import 期硬断言，F12 行 139）：p2 的任何行**不得**改（`EPIC.md` 行 219）。**`templates/builtin.py` 同理归 p2-04**（行 215）。
- **`subsystems/brake.py`、`subsystems/drive.py`、`templates/builtin.py`**：归 p2-04（`EPIC.md` 行 215、`SUBTASKS.csv` `p2-04` `notes`）。
- **`preparation/vehicle_dynamic.py`**：力矩段归 p2-05、转向段归 p2-06、轮胎拒绝段归 p4-04（`EPIC.md` 行 217）。本行不得触碰该文件。
- **`subsystems/assembler.py` 的 `VerticalTireElement` 过滤段**：归 p4-04（`EPIC.md` F15 行 145 与行 267 (b)）。
- **内核侧**（`packages/suspension_kernel/cpp/**`）：内核元素类型与 ABI 归 p2-02，本行只**消费**；**`mb_config/version.hpp` 与 `kernel/native.py` 的版本常量只有 p2-02 可改**（`EPIC.md` 行 225）。
- **`connections/` 的配对契约定义**：配对机制是阶段一 02 交付的、跨子系统插接的唯一通道（`EPIC.md` 行 71 与行 233）。本行只**消费**，不重定义、不另立第二条推断路径。
- **不得硬编码 `upright`/`chassis` 字符串**（判据 2；`EPIC.md` 行 233 与行 241 (b)）。
- **不得并行改 `assembly.schema.json`**：p2-02/p2-03 若需新增元素类型字段，必须与阶段一 03 的放置段改动串行（`EPIC.md` 行 218）。
- **不得重录任何基线**（行 228、D5 行 64）；**不得新增 skip/xfail**（行 229）。
- 其它子任务目录（p2-01、p2-02、p2-04 ~ p2-06、p3/p4/p5 各行）、`EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`——不得改。
- `raw/` 中不得放未执行的内容（行 355）。

## 依赖与时机

- `depends_on = p2-02`（`SUBTASKS.csv` `p2-03`）。上游 S1（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 67–75）；**阶段一未完成之前不得置 `IN_PROGRESS`**（行 75）。
- 串行链：`p2-01 → p2-02 → p2-03 → p2-04 → p2-05 → p2-06`（行 203–204）。本行**必须**在 p2-02 的内核元素类型与 ABI 编组落地之后（否则编译层无目标可编）。
- **`element_build.py` 三段串行**（`EPIC.md` 行 216）：本行占**构造分派段**，p4-04 占**轮胎段**，p3-02 若要读构造后的约束集合则**只读**。三段不得并行。
- **与阶段一 03 的关系**：p2-03 若需新增契约 schema 的元素类型字段，必须与阶段一 03 的放置段改动**串行**（行 218，`S1` 的第二个理由）。
- 与 p2-04 的关系：p2-04 的 `brake.py`/`drive.py` 产出力矩元**依赖本行的构造分派分支已存在**；本行必须在 p2-04 之前落地（行 203）。
- 与阶段三/四/五：阶段三在阶段一之后、阶段四在阶段三之后、阶段五在阶段四之后（行 211）；本行不得提前触及其它阶段的写范围。

## 判据与证据落点

逐条对应 `EPIC.md` 行 241 的 (a)(b)(c)(d)。

1. **(a) 元素声明 + 构造分派 + 编进内核输入** → `raw/element_declaration.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/subsystems -q`。
   - 看什么：`modeling/primitives/` 新声明的**文件:行 + 类名/符号名**；`subsystems/element_build.py` **构造分派段**新增分支的**文件:行 + 分支条件原文**（对照 `EPIC.md` 行 241 (a) 给的 `:66-71`）；编译层能被内核输入消费的证据（契约表/编组输出的实测片段，锚点见 `EPIC.md` F1 行 111 的 `cases/vehicle_dynamic.py:579/582`）。
   - 落点：`raw/element_declaration.md`。
2. **(b) 施力/反力由端口配对决定** → `raw/port_pairing.md`
   - 跑什么：对本行新增的连接路径跑 `grep` 检索 `upright` / `chassis` 字符串与 `grep` 检索 `Port` / `Connection` 的使用点；跑 `uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q`。
   - 看什么：**`grep` 零命中**的原文（本行新增/修改的构造路径内不得出现按名字猜身份的字符串）；施力体与反力体的选择点来自端口配对的证据（`file:line` + 代码片段）；`test_import_boundaries` 与分层门绿。
   - 落点：`raw/port_pairing.md`（含 grep 命令与零命中输出）。
3. **(c) 最小装配体跑通并读到力矩贡献** → `raw/min_assembly_torque.md`
   - 跑什么：本行新增的最小装配体用例（含**一个**力矩元）；读出该力矩元对结果的贡献。
   - 看什么：用例**文件:行**、实跑命令与退出码；力矩贡献的读数（不是「用例通过」这类弱证据——必须给出被读出的量、其值、以及「去掉力矩元则读数改变」或等价的因果证据）。
   - 落点：`raw/min_assembly_torque.md`。
4. **(d) 既有元素类型产物不变** → `raw/legacy_elements_parity.md`
   - 跑什么：与 p2-01 的路径快照对照（`tasks/p2-01-freeze/raw/torque_path_snapshot.json` 的对照口径），并跑 `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check`。
   - 看什么：既有元素类型（`EPIC.md` F1 行 111 列的 spring/damper/bump_stop/anti_roll/tire/bushing，声明在 `subsystems/types.py:451`）的装配产物与快照逐项对照结论；任一差异**必须**是已登记项，否则失败。
   - 落点：`raw/legacy_elements_parity.md`。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **装配层不得出现按名字猜身份的规则**（行 233）：任何新增跨子系统连接必须经形式化的 `Port` / `Connection` 契约（「这是 G3/G5 的判据来源」）。本行的判据 2 直接来源于此。
- **不得把「简化/专用」的分支写进 role 接口**（行 232）：`if supplies_wheels:` 之类的分支只允许出现在试验台自己的层。
- **分层方向不可逆**（行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层。**本行在 `modeling/primitives/` 新增声明，是这一方向的关键落点**——新元素不得从 `modeling/` 反向引用 `subsystems/` 或作者层。
- **内核 ABI 单点提交**（行 225）：**只有 p2-02** 可改 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量。本行不得触碰版本常量。
- **`model_dump(mode="json")` 的产物不得被改变**（行 227）：`api.py:116`/`:284` 用它算 `model_hash`。
- **基线不得重录**（行 228、D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；其余基线若确需变化，逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」。本行**不应**触发基线变化。
- **不得新增 skip/xfail**（行 229）。
- **每步落地后必须重跑**（行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。**本行触及求解路径（新增力元进入内核输入），故数值门三项必须跑**。
- **`kc_parity_check.py` 的口径**（行 231）：不带 `--actual-dir` 时恒过，不构成证据。

## 风险与回退

- **硬编码字符串会以「顺手改写」的形式复发**（`EPIC.md` 行 233 与 G2 判据行 81）：缓解 = 判据 2 的 `grep` 零命中 + 端口配对的使用点证据；**不得用字符串常量表替代硬编码**。
- **`element_build.py` 三行触及**（行 216）：本行只占构造分派段；**误改轮胎段会与 p4-04 冲突**（行 267 (b)(c)）。缓解：改动前后 `git diff --stat` 与分段 `file:line` 逐项自证，记入 `raw/element_declaration.md`。
- **契约 schema 与阶段一 03 串行**（行 218）：本行若需新增元素类型字段，**不得并行**改 `assembly.schema.json`。
- **既有元素产物漂移**（判据 4）：新增元素类型可能改变装配顺序或遍历顺序。缓解：以 p2-01 快照对照；`dynamic_hash_sentinel.py --check` 逐字节为硬门；**差异非空即失败**（对照口径见 p2-01 的 `raw/torque_path_snapshot.json`）。
- **既有失败**（行 320）：起点以 p2-01 `raw/baseline_notes.md` 为准；任何新增失败阻断完成，不相关既有失败独立列明。

## Done-When

- [ ] 力矩元素在 `modeling/primitives/` 有声明（`raw/element_declaration.md` 给 `file:line` + 符号名）。
- [ ] `subsystems/element_build.py` **构造分派段**有该元素的构造分支（`file:line` + 分支条件原文），且能编进内核输入（实测片段）。
- [ ] 施力体与反力体由 `Port` / `Connection` 配对决定；本行新增/修改的构造路径内 `grep "upright"` 与 `grep "chassis"` **零命中**（`raw/port_pairing.md`）。
- [ ] 一份含**一个**力矩元的最小装配体跑通一次，且读到力矩贡献（`raw/min_assembly_torque.md` 含读数与因果证据）。
- [ ] 既有元素类型的装配产物与 p2-01 路径快照对照无未登记差异；`dynamic_hash_sentinel.py --check` 逐字节一致（`raw/legacy_elements_parity.md`）。
- [ ] `just check-fast` 绿；触及求解路径故 `just gate-numeric` 三项全绿；`tests/architecture` 绿。
- [ ] 未触碰 `element_build.py` 轮胎段、`templates/roles.py`、`templates/builtin.py`、`subsystems/brake.py`、`subsystems/drive.py`、`preparation/vehicle_dynamic.py`、内核版本常量（用 `git diff --stat` 自证）。
- [ ] 无新增 skip/xfail；未重录任何基线。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/subsystems -q && uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q
```
