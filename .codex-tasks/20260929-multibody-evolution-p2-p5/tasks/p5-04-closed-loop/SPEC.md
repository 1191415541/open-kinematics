# SPEC：p5-04 闭环控制（ABS 或 ESC 的实际反馈闭环）

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-04`

## Goal

**硬性范围（先读）**：本行**必须交付 ABS 或 ESC 之一的实际反馈闭环**。这是路线图 `packages/suspension_multibody/docs/multibody_architecture_evolution.md:203` 的原文要求（「支持简单的闭环控制（ABS/ESC）与标准 FMI 联合仿真导出」），也是 `EPIC.md:97` 的 G8 判据。**D2 只约束「是否需内核单步接口」，不授权把 ABS/ESC 降级为可变阻尼开环回放**（`EPIC.md:61` D2 行、`EPIC.md:97` G8、`EPIC.md:281` 验证协议 (a)）。父行标题里的「（ABS 或可变阻尼）」不构成降级授权：可变阻尼只是**执行器通道**，不是闭环目标。

拆自 `SUBTASKS.csv` 第 22 行 `acceptance_criteria`（已按审核阻断项 3 修订），逐条可判定：

1. **ABS 或 ESC 之一的实际反馈闭环存在，且证据是「同一次运行内的三段链」**（`EPIC.md:281(a)`、`EPIC.md:97`）：控制器在给定工况下使被测量收敛到目标，形式是**控制器在一次运行的推进过程中读状态、算控制、写执行器**（`EPIC.md:61` D2 行的口径）。**证据必须是同一次运行内的「状态 → 控制 → 执行器 → 状态」链，三段都有可读数值记录**（状态段的原始测点读数、控制段的控制量计算、执行器段的写入值、以及回到状态段的下一拍读数变化），且**执行器输入确实改变了同一次仿真的状态轨迹**（不是只把值写进对象）。**开环回放不算**：把预置时间序列喂进执行器而不读状态、不按状态改变执行器输入（含**跨次回放**：多次运行之间分步喂值），即判为不满足。
2. **`cases/handling.py:8` 的「只表达开环工况」契约反转，`tests/cases/test_handling.py:242` 相应更新，理由登记**（`EPIC.md:281(b)`）：`EPIC.md:164` F21 实测——`cases/handling.py:8` **明文声明本层只表达开环工况**，测试 `tests/cases/test_handling.py:242 test_a_closed_loop_manoeuvre_is_refused_by_name` **固化**了这一拒绝。判据：契约原文反转后的文本 + 测试改动原文 + **理由登记**（含作用线/力路径对照，顺序要求见风险节）。
3. **D2 裁决必须在开工前完成；「要不要内核单步接口」的实测须先于或并列于闭环实现；若需内核单步接口，先提请裁决并新增一行专属子任务**（`EPIC.md:229` 冻结约束、`EPIC.md:281(c)`）：`EPIC.md:168` F23 实测内核是**批式 ABI**（`cpp/axle_dynamics/core_abi.hpp:135-136`，无 step/state 入口）。**D2 只裁决「要不要内核单步接口」，不裁决闭环目标本身**（`EPIC.md:61`）。**时点要求**：「是否需要内核按步状态读取」的实测必须**先于或并列于闭环实现**——因为它决定是否要**先新增一行专属 ABI 子任务**（若排在闭环之后，闭环实现会卡在 ABI 缺口上）。若实测确需内核单步接口，必须**先提请裁决并新增一行专属子任务**（该行须含 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量归属、对 `p2-02` 的前置依赖、自身验收），**不得由本行改版本常量**（`EPIC.md:229` 明说「不得由 p5-04 自身改版本常量」），也**不得只登记缺口就放行 p5-05**（不得静默降级为「登记即收口」）。判据：D2 裁决结论原文 + 该实测结论（含时点关系）+ 若为「需要」则给出**新增子任务的落盘产物**（`SUBTASKS.csv` 的新行与 `EPIC.md` 的对应登记，由父 Epic 侧执行）与裁决请求文本，而不是收口声明。
4. **闭环不破坏开环用例（开环结果逐项一致）**（`EPIC.md:281(d)`）：判据：开环工况在改动前后结果逐项一致；`dynamic_hash_sentinel.py --check` 保持逐字节（`validation_command` 已含该条）。

> **仍未闭合时的正确收口**：若 D2 裁决为「需内核单步接口」，本行**停下来**——新增专属子任务并提请裁决，p5-05 在本行闭环证据成立之前**不得开工**（`EPIC.md:229`：不得只登记缺口就放行 p5-05）。**不得**用「可变阻尼开环回放」充当闭环交付。

## 写范围（允许改的路径）

`SUBTASKS.csv` 第 22 行 `notes` 原文（已按审核修订）：

> D2 只约束 是否需内核单步接口 不授权降级闭环目标。反转既有契约同阶段一 05 的做法：先登记反转理由与作用线力路径对照 再改契约。若需内核单步接口 属本 Epic 第二次 ABI 变更 须单独裁决并新增专属子任务（不得由 p5-04 改版本常量）

**目标（不可缩水）**：ABS 或 ESC 之一的**实际反馈闭环**（路线图 `:203`）；可变阻尼只是执行器通道。

展开为（承接 p5-03 的信号总线与 `api.py` 暴露段）：

- **新增闭环控制器模块**（本行自定落点与命名；须符合 `EPIC.md:230` 冻结约束的分层方向）
- **外部控制器契约**：控制器的输入/输出面（读测点、写执行器），由 p5-03 的总线提供
- `packages/suspension_multibody/src/suspension_multibody/cases/handling.py`（**契约反转段**，锚点 `:8`）
- `packages/suspension_multibody/tests/cases/test_handling.py`（`:242` 的拒绝用例相应更新）
- `packages/suspension_multibody/src/suspension_multibody/api.py`（暴露段的接线）
- 对应测试：`packages/suspension_multibody/tests/cases/`、`tests/api/`
- `tasks/p5-04-closed-loop/raw/**` 与临时脚本与会话 scratch

## 禁止触碰

- **本行不得改 ABI 版本常量**：`mb_config/version.hpp` 与 `kernel/native.py` 的版本常量**只能由 p2-02 与「D2 裁决后新增的专属子任务」修改**（`EPIC.md:229`）。**若 D2 裁决为「需要内核单步接口」，必须先提请裁决并新增一行专属子任务（含版本常量归属、对 p2-02 的前置依赖、自身验收），该行不在本行的既有写范围里；本行自身绝不动版本常量，也绝不以「登记缺口」代替新增子任务**（`EPIC.md:229`）。
- **不得删除既有公开入口**（绞杀者模式，`EPIC.md:104` Non-Goals）：`run_case` / `run_dynamic_case` / `FrontAxleModel` 保持可用。
- **不得改 `FrontAxleModel` 字段形状与 `model_dump(mode="json")` 的产物**（`EPIC.md:231` 冻结约束）。
- `simulation/backend.py` 之外不得直接调 `run_contract`（`EPIC.md:170` F24 与冻结约束 `EPIC.md:230`）。
- **不得改门禁脚本与 allowlist**（`EPIC.md:170` F24）；**不得改 `results/channels.py:22 ChannelRegistry` 与 `adams/axle_channels.yaml:20`**（冻结的 Adams 输出通道表，`EPIC.md:160` F19）。
- **不得重录任何基线**（`EPIC.md:232` 冻结约束；D5 见 `EPIC.md:64`）。
- **不得新增 skip/xfail**（`EPIC.md:233`）——`:242` 的拒绝用例是**更新**（反转契约），不是跳过或删除。
- `.codex-tasks/20260929-multibody-evolution-p2-p5/{EPIC.md,SUBTASKS.csv,PROGRESS.md}` 禁止修改。
- 若 p5-03 已改 `outputs/builtin.py`，本行不得再动该文件（同一文件归 p3-05 与 p5-03）。

## 依赖与时机

- `depends_on = p5-03`（`SUBTASKS.csv` 第 22 行）：闭环保控制器的输入/输出面依赖总线（测点与执行器输入）。
- 全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md:69`、`EPIC.md:75`）；阶段五在阶段四完成后开工（`EPIC.md:215`）。
- **D2 裁决必须在开工前完成**（`EPIC.md:61`、`EPIC.md:281(c)`）：它决定本行是「Python 侧总线 + 外部控制器契约 + **ABS/ESC 实际反馈闭环**」，还是「先补内核单步接口」。**D2 未裁决前本行不得置 `IN_PROGRESS`**。注意：D2 的裁决范围**只有**「是否需内核单步接口」，**不包含**「是否可把闭环降级为开环回放」——降级不在可裁决项内（`EPIC.md:97`）。**D2 的旧建议「只交付总线 + 外部控制器契约 + 仅开环回放的控制器」已废**（`EPIC.md:61` 复审修订）——闭环目标必须交付 ABS/ESC 的实际反馈闭环。
- **时点关系（可执行顺序）**：`TODO.csv` 第 4 步「内核单步接口需求实测与范围处置」**先于**第 5 步「ABS 或 ESC 控制器实现与闭环证据链」——即「要不要内核单步接口」的实测先于或并列于闭环实现，因为它决定是否要**先**新增一行专属 ABI 子任务（`EPIC.md:229`）；本行的契约反转（第 2、3 步）不受此顺序影响。
- **若裁决为「需内核单步接口」**：本行**先提请裁决并新增一行专属子任务**（含 `mb_config/version.hpp` 与 `kernel/native.py` 的常量归属、对 `p2-02` 的前置依赖、自身验收），**不得由本行改版本常量**，**也不得只登记缺口就放行 p5-05**（`EPIC.md:229`）。
- 反转既有契约的做法**同阶段一 05**（`EPIC.md:325`）：先登记反转理由与「作用线/力路径对照」，再改契约。阶段一 05 反转的是 `test_rig_link` 契约，这是同类先例。

## 判据与证据落点

逐条对应 `EPIC.md:281` 的 (a)(b)(c)(d)：

| 父判据 | 做什么 | 看什么 | 证据文件 |
|---|---|---|---|
| (a) **ABS/ESC 实际反馈闭环**（不是开环回放） | 实现 ABS 或 ESC 控制器；在给定工况下按总线读状态、算控制量、写执行器 | **同一次运行内三段均有可读数值记录**：状态段（原始测点读数）→ 控制段（控制量）→ 执行器段（写入值）→ 回到状态段（下一拍读数变化）；执行器输入确实改变同次仿真轨迹；被测量收敛（含初值、终值、目标、容差） | `raw/closed_loop_chain.md` |
| (b) 反转 `cases/handling.py:8` 契约 | 改契约文本；更新 `tests/cases/test_handling.py:242` | 契约原文（前/后）、测试改动原文、**理由登记**（含作用线/力路径对照） | `raw/contract_reversal.md` |
| (c) D2 裁决确认与内核单步接口范围 | 确认 D2 已裁决（开工前置）；**先于或并列于闭环实现**实测闭环是否需要内核按步状态读取（`EPIC.md:168` F23：内核是批式 ABI，无 step/state 入口） | D2 裁决结论原文；该实测结论**含时点关系**（为何必须先于闭环实现：它决定是否要先新增专属 ABI 子任务）；若结论为「需要」，给出**新增专属子任务的落盘产物**（版本常量归属、对 p2-02 的前置依赖、自身验收）与裁决请求文本；**不是**收口声明 | `raw/d2_ruling.md`、`raw/kernel_step_scope.md` |
| (d) 闭环不破坏开环 | 开环工况改动前后对照；跑 `dynamic_hash_sentinel.py --check` | 开环结果逐项一致；`--check` 逐字节 | `raw/open_loop_regression.md` |
| 门禁 | 跑本行 `validation_command` | 退出码 0 + 输出原文 | `raw/gates.txt` |

**闭环三段链的记录格式（最低要求）**：一张表，同一 `run_id` 下逐拍列出——`t`、状态段测点值（例如轮速/滑移率）、控制段控制量（例如制动压力指令）、执行器段写入值（例如 `brake_torque` / 阻尼系数）、以及该写入后下一拍的状态读数值。**三段缺一即不满足 (a)。**

## Constraints（冻结约束）

- **内核提交唯一归属仍只有 `simulation/backend.py`**（`EPIC.md:170` F24 与冻结约束 `EPIC.md:230`）。
- **三个公共 API 门禁全绿**（`EPIC.md:170` F24）：allowlist `mode = "strict"` 且零条目，不得加白名单。
- **不得把闭环降级为开环回放**（`EPIC.md:97` G8、`EPIC.md:281(a)`）：D2 的裁决范围只到「是否需内核单步接口」，**不含**「ABS/ESC 可否降级」。任何以可变阻尼预置时间序列代替 ABS/ESC 反馈闭环的做法都判为未达成 G8。
- **内核 ABI：本行不改版本常量**（`EPIC.md:229`）：本 Epic 最多两次 ABI 变更，第二次（仅当 D2 裁决要求内核单步接口）**必须由新增的专属子任务承担**（含 `mb_config/version.hpp` 与 `kernel/native.py` 的常量归属、对 `p2-02` 的前置依赖、自身验收）。本行**不得**自行改版本常量，**不得**只登记缺口就放行 p5-05。
- **`model_dump(mode="json")` 产物不得被改变**（`EPIC.md:231` 冻结约束）。
- **基线不得重录**（`EPIC.md:232` 冻结约束；D5 见 `EPIC.md:64`）：开环结果逐项一致是本行硬门；任何变化按 D5 逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径）」。
- **不得新增 skip/xfail**（`EPIC.md:233`）。
- **每步落地后重跑 `just check-fast`**；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`（`EPIC.md:234`）。
- **`raw/` 只存已执行证据**；临时脚本与中间日志写会话 scratch。

## 风险与回退

- **反转既有契约**（`EPIC.md:325` 原文）：「`cases/handling.py:8` 明文拒绝闭环，测试固化。缓解：同阶段一 05 的做法——**先登记反转理由与作用线/力路径对照，再改契约**。」本行必须严格按此顺序执行：`raw/contract_reversal.md` 先落「反转理由 + 作用线/力路径对照」，之后才改 `cases/handling.py:8` 与 `tests/cases/test_handling.py:242`。
- **需要内核单步接口意味着又一次 ABI 变更**（`EPIC.md:61`、`EPIC.md:168` F23、`EPIC.md:229`）：内核是批式 ABI（`cpp/axle_dynamics/core_abi.hpp:135-136`），实时闭环在当前 ABI 下无法真正实现。**正确处置**：先提请裁决 → **新增一行专属子任务**（版本常量归属 + 对 p2-02 的前置依赖 + 自身验收）→ 该子任务落地后本行再交付 ABS/ESC 反馈闭环。因此「要不要内核单步接口」的实测在 `TODO.csv` 中排在**第 4 步（先于第 5 步的闭环实现）**——若排在闭环之后，闭环会卡在 ABI 缺口上。**严禁**「登记即收口」，**严禁**把闭环降级为可变阻尼开环回放，**严禁**在本行内改版本常量。
- **风险：开环结果被控制器吸管式改动**（`cases/` 路径与动态路径共用）。回退：先跑 `dynamic_hash_sentinel.py --check` 取改动前基线（p5-01 已记录起点值）；有变化即按 D5 登记并给物理等价判据。
- **风险：为让 `:242` 变绿而删测试或加 skip**（`EPIC.md:233` 禁止新增 skip/xfail）。回退：`:242` 是**更新**成反转后的契约用例，不是删除或豁免。
- **风险：控制器读测点值时重算而没有走总线**（`EPIC.md:279(b)` 的口径）。回退：控制器只经 p5-03 的总线读写；`grep` 证明控制路径不直接触结果文档或求解内核。

## Done-When

- [ ] **ABS 或 ESC 之一的实际反馈闭环已交付**（路线图 `:203`、`EPIC.md:97` G8），且证据是**同一次运行内的「状态 → 控制 → 执行器 → 状态」三段数值记录**（`raw/closed_loop_chain.md`），执行器输入确实改变了同次仿真轨迹；**未用开环回放代替**。
- [ ] D2 裁决已确认（开工前置完成，结论落 `raw/d2_ruling.md`）；「是否需要内核单步接口」有实测结论且该实测**先于或并列于闭环实现**（`TODO.csv` 第 4 步早于第 5 步）；若为需要，已**先提请裁决并新增一行专属子任务**（含版本常量归属、对 p2-02 的前置依赖、自身验收），**既未在本行改版本常量，也未只登记缺口就放行 p5-05**。
- [ ] `cases/handling.py:8` 的「只表达开环工况」契约已反转；`tests/cases/test_handling.py:242` 相应更新；**反转理由与作用线/力路径对照先落盘再改契约**。
- [ ] 闭环不破坏开环用例：开环结果逐项一致；`dynamic_hash_sentinel.py --check` 逐字节通过。
- [ ] 未改 ABI 版本常量（`mb_config/version.hpp` / `kernel/native.py` 未动）；内核提交唯一归属仍只有 `simulation/backend.py`。
- [ ] 未重录任何基线；未新增 skip/xfail；`model_dump(mode="json")` 形状与 `model_hash` 未变。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/cases packages/suspension_multibody/tests/api -q && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
