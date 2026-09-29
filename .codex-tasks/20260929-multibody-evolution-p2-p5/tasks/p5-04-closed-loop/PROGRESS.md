# PROGRESS：p5-04 闭环控制（ABS 或 ESC 的实际反馈闭环）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-04`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p5-04-closed-loop
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-04-closed-loop/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: `p5-03`（`SUBTASKS.csv` 第 22 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md:69`、`EPIC.md:75`）约束

## Context Recovery Block

- **Current milestone**: #1 — D2 裁决确认（开工前置）
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - **交付目标不可缩水（2026-09-29 审核修订）**：本行**必须交付 ABS 或 ESC 之一的实际反馈闭环**（路线图 `packages/suspension_multibody/docs/multibody_architecture_evolution.md:203` 原文；`EPIC.md:97` G8；`EPIC.md:281(a)`）。**D2 只约束「是否需内核单步接口」，不授权把 ABS/ESC 降级为可变阻尼开环回放。** 父行标题里的「（ABS 或可变阻尼）」不是降级授权——可变阻尼只是**执行器通道**。
  - **闭环证据口径**：必须是**同一次运行内的「状态 → 控制 → 执行器 → 状态」链**，三段都有可读数值记录（状态段原始测点读数 / 控制段控制量 / 执行器段写入值 / 回到状态段的下一拍读数），且执行器输入确实改变了同次仿真的状态轨迹。**控制器在一次运行的推进过程中读状态、算控制、写执行器；喂预置时间序列的开环回放（含跨次回放）判为不满足。**
  - `cases/handling.py:8` **明文声明本层只表达开环工况**，测试 `tests/cases/test_handling.py:242 test_a_closed_loop_manoeuvre_is_refused_by_name` **固化**了这一拒绝（`EPIC.md:164` F21）。本行必须反转这条既有契约。
  - 全仓无 ABS/ESC/PID 实现：`\babs\b` 命中全是 `abs()`；`\besc\b`、`PID` 零命中；`controller` 仅指内核积分器的 local-error 步长控制器（`EPIC.md:164` F21）。
  - 内核是**批式 ABI**（`cpp/axle_dynamics/core_abi.hpp:135-136`，无 step/state 入口），实时闭环在当前 ABI 下无法真正实现——这是 **D2 的硬约束**（`EPIC.md:168` F23 与 `EPIC.md:61`）。
  - **ABI 归属**：本 Epic 最多两次 ABI 变更；第二次（仅当 D2 裁决要求内核单步接口）**必须由「先提请裁决并新增的一行专属子任务」承担**（含 `mb_config/version.hpp` 与 `kernel/native.py` 的常量归属、对 `p2-02` 的前置依赖、自身验收），**不得由本行改版本常量**（`EPIC.md:229`）。
  - **D2 口径（2026-09-29 复审修订）**：D2 **只裁决「要不要内核单步接口」**，**不裁决闭环目标本身**；其旧建议「只交付总线 + 外部控制器契约 + 以开环回放充当闭环的控制器」**已废**（`EPIC.md:61`）——ABS/ESC 的实际反馈闭环必须交付。
- **Known issues**:
  - **D2 未裁决前本行不得置 `IN_PROGRESS`**（`EPIC.md:61`、`EPIC.md:281(c)`）。
  - **降级风险（本行最大风险）**：用「可变阻尼预置时间序列」充当闭环交付。判定标准 = 是否有同一次运行内的三段数值记录且执行器输入改变了后续状态；没有即判未达成 G8，**不得以 D2 为借口降级**（D2 的裁决范围不含降级）。
  - **不得只登记缺口就放行 p5-05**（`EPIC.md:229` 审核阻断项 3）：若确需内核单步接口，必须**新增一行专属子任务**（落进父 `SUBTASKS.csv` / `EPIC.md`，由父 Epic 侧执行），p5-05 在本行闭环证据成立前不得开工。
  - **时点关系（本行步骤顺序）**：「要不要内核单步接口」的实测排在 `TODO.csv` **第 4 步**，**先于**第 5 步的闭环实现——它决定是否要先新增专属 ABI 子任务；若排在闭环之后，闭环会卡在 ABI 缺口上（`EPIC.md:229`）。
  - 反转契约必须**先登记理由与作用线/力路径对照，再改契约**（`EPIC.md:325`，同阶段一 05 的做法）。
  - `tests/cases/test_handling.py:242` 是**更新**用例，不是删除或 skip（`EPIC.md:233` 禁止新增 skip/xfail）。
- **Next action**: 先确认 D2 裁决结论并落盘 `raw/d2_ruling.md`（裁决范围只到「是否需内核单步接口」，旧建议已废），再读 `cases/handling.py:8` 与 `tests/cases/test_handling.py:242` 的原文、确认 p5-03 的总线已落地，然后按 `TODO.csv` 第 2 步（前置登记）起展开——**前置登记完成前不得改契约文件**；第 4 步先实测内核单步接口需求并落 `raw/kernel_step_scope.md`，之后才做第 5 步的 ABS/ESC 控制器实现与三段证据链。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划轮产物（已含 2026-09-29 审核修订：闭环目标不得缩水、D2 只约束内核单步接口、新增闭环三段链步骤）：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
