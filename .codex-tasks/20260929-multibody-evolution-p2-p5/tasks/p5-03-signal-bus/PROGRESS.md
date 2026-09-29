# PROGRESS：p5-03 信号总线（测点与执行器）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-03`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p5-03-signal-bus
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-03-signal-bus/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: `p5-02`（`SUBTASKS.csv` 第 21 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）约束

## Context Recovery Block

- **Current milestone**: #1 — 信号总线模块骨架与 `outputs` 声明关系定义
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 今天**不存在** SignalBus/Sensor/Measurement 抽象（`EPIC.md` F19，全仓 `class SignalBus|class Measurement|class Sensor|signal_bus` 零命中）。
  - 现存两套互不相通：`outputs/` 静态声明（`outputs/builtin.py:154 ASSEMBLY_OUTPUTS`、`:220 RIG_OUTPUTS`、`outputs/declarations.py:1`，**未被 api.py 或 rigs 生产路径消费**，仅测试引用）与 `results/channels.py:22 ChannelRegistry`（只读，读 `adams/axle_channels.yaml:20`，是**冻结的 Adams 输出通道表**）。
  - 今天**无可变阻尼/CDC**（`EPIC.md` F20，`grep -i "damper_ratio|variable_damp"` 零命中）；既有 actuator 全是开环/预设的 `TimeSignal`。
  - 写范围按 `SUBTASKS.csv` 第 21 行 `notes`：新增信号总线模块 + `outputs/builtin.py` 的派生输出声明 + `api.py` 的暴露段。
  - **与 p3-05 共享 `outputs/builtin.py`，必须串行**（`EPIC.md` 行 221：该文件的派生输出声明归 p3-05）。
- **Known issues**:
  - 新增测点会**改变输出集合**——D5 口径（`EPIC.md` 行 64）已点名阶段五这种情形：必须逐项登记，**禁止重录**基线。
  - 总线**不得重算**测点值（`EPIC.md` 行 275(b)）；只能读既有结果文档。
  - D2（`EPIC.md` 行 61）：内核是批式 ABI（F23），本行只做 Python 侧总线与开环，闭环与内核单步需求交 p5-04 登记。
- **Next action**: 先读 `outputs/builtin.py:154/:220`、`outputs/declarations.py:1`、`results/channels.py:22` 与 `api.py` 的暴露段，确认「谁声明、谁提供值」，再按 `TODO.csv` 第 1 步起展开；动 `outputs/builtin.py` 前先确认 p3-05 已落地。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
