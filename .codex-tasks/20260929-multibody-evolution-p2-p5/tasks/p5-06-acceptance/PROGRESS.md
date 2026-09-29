# PROGRESS：p5-06 终局独立验收（阶段五）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-06`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p5-06-acceptance
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-06-acceptance/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: `p5-05`（`SUBTASKS.csv` 第 24 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md:69`、`EPIC.md:75`）约束

## Context Recovery Block

- **Current milestone**: #1 — G7 与 G8 逐条实跑
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 本行是**Epic 收口**：`EPIC.md:287-314` 的 Done-When (a)-(j) **逐条实跑并逐条记录自己的退出码**（不是只记录测试退出码），且「端到端独立验收（**不依赖子任务自证**）」（`EPIC.md:289`）。
  - 判据真源：G7 在 `EPIC.md:95`，G8 在 `EPIC.md:97`，阶段五验证协议在 `EPIC.md:285`，Done-When 在 `EPIC.md:287-314`。
  - **(f) 轮端统一按「独立复验阶段一 04 的实际交付」判**（`EPIC.md:75`、`EPIC.md:93` G6、`EPIC.md:271`）：实测单轴侧与整车侧消费的是**同一份** wheel 子系统文件（比对**文件路径与内容指纹**，**不看阶段一自报**），`grep VerticalTireElement` 在装配路径无命中。
  - **(h) 总线与闭环按「ABS/ESC 实际反馈闭环 + 同次运行三段数值记录」判**（`EPIC.md:97` G8、`EPIC.md:308`）：状态 → 控制 → 执行器 → 状态四段都有可读数值，**开环回放不算**；ABI 若需第二次变更，必须已按 D2 裁决**新增专属子任务**，不得只登记缺口。
  - 全量基线（`AGENTS.md` 第 3 节）：**1297 passed, 1 skipped, 1 xfailed**，skip 与 xfail **不得增长**（`EPIC.md:233`）。
  - 本行是**只读验收轮**：不改生产代码、不改测试、不改基线、不改门禁；发现失败退回对应子任务修。
- **Known issues**:
  - **数值门命令口径（已修正）**：`case_parity_check.py` **只接受 `--family` / `--allow-partial` / `--record`**（实测 `packages/suspension_multibody/scripts/case_parity_check.py:1142-1161`），**没有 `--check`**（`EPIC.md:234` 审核阻断项 5）——本行一律用**无参数**调用；若命令报「unrecognized arguments」即说明写错。
  - **两次 pytest 调用必须分开**：`suspension_kernel/tests` 与 `suspension_contracts/tests` 并进 multibody 目录会改 `rootdir`，使 `tests.benchmark_fixture` 解析失败（`SUBTASKS.csv` 第 24 行 `notes` 与 `AGENTS.md` 第 1 节）。
  - **`kc_parity_check.py` 不带 `--actual-dir` 时是自比较（恒过），不构成证据**（`EPIC.md:235`）。
  - 全量回归约 33 分钟（`AGENTS.md` 第 3 节），需预留时长并保留原始输出。
  - 禁止用 `-k` / `--deselect` 豁免失败（`AGENTS.md` 第 7 节）。
- **Next action**: 先收集 p5-02 ~ p5-05 的 `raw/` 证据清单与 p5-01 的起点值（`model_hash`、门禁规则、调用者台账），再按 `TODO.csv` 第 1 步起逐条**自己重跑**并把命令、退出码与产物落到 `raw/`；Done-When (a)-(j) 的十条各写一行，含命令与退出码。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划轮产物（已含 2026-09-29 审核修订：`case_parity_check.py` 无参数调用、Done-When (a)-(j) 逐条实跑并记录退出码、(f)/(h) 判别口径）：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
