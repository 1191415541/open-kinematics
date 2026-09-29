# PROGRESS：p3-05 报表通道按安装角色动态注册

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-05`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p3-05-dynamic-channels
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-05-dynamic-channels/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步，全部 `TODO`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #1 — 前后轴硬编码字段改为按安装角色动态生成
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on = p3-04`（`SUBTASKS.csv` 的 `p3-05`）；**p3-05 在 p3-04 之后**（`EPIC.md` 行 220），因为报表要消费广义静平衡的字段。字段形状以 `tasks/p3-04-static-loads/raw/static_loads_contract.md` 为准，本行不重定义。
  - 跨阶段顺序为「阶段三在第一阶段完成后开工」（`EPIC.md` 行 211）；**前置 S1**——阶段一 Epic 01–07 全部 `DONE` 之前本行不得置 `IN_PROGRESS`（`EPIC.md` 前置一节：19 行实施行受此约束，四个只读冻结行不受限）。
  - **`outputs/builtin.py` 的派生输出声明归本行**（`EPIC.md` 行 221），且 **p5-03 若需新增测点声明必须与本行串行**（同一文件）——故本行必须把改动段落精确登记在本文件里。
  - **`report/` 分层约束不可破**（`EPIC.md` 行 226）：不得 import native/kernel/solver（`legacy_surface_gate.py` 的 `report_native_import`），也不得自求力律（`report_constitutive_call`）；本行 `validation_command` 第二段就是跑该门。
  - **验收命令口径（`SUBTASKS.csv` 的 `p3-05` `notes`）**：`uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q`——**实测 `packages/suspension_multibody/tests/report/` 目录今天不存在**，写成 `tests/report` 的命令必失败；若本行新建 `tests/report/`，则命令与 `SPEC.md` / `TODO.csv` 的 notes 同步更新。
  - **F8（`EPIC.md` 行 128）**：`report/wheel_loads.py:17 _WHEELS`、`:26-31` 的六个前后轴/左右侧字段、`:36-38` 的「强制恰好四角」校验，以及 `report/metrics/vehicle.py:21-43` 中**重复定义**的同一批字段。
- **Known issues**:
  - **4 轮向后兼容是本行硬门**（`EPIC.md` 行 257(b)）：既有字段名与值一个都不能少、不能变；`normal_load_axle_{placement}` 只能作为**新增**命名并存。改造前基准**尚未落盘**。
  - **「动态」可能退化为换一种硬编码**：判据是「非 `front`/`middle`/`rear` 放置名的改名实验」，该实验**尚未执行**；缺证据即视为未达成。
- **`outputs/builtin.py` 段落归属**：本行只改派生输出声明段；其它段与 p5-03 的后续改动不属本行。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/report/wheel_loads.py`（`:17`、`:26-31`、`:36-38`）与 `.../report/metrics/vehicle.py`（`:21-43`），再读 `tasks/p3-04-static-loads/raw/static_loads_contract.md`（静平衡字段形状）；**先把 4 轮的完整字段名与值落盘**（`raw/four_wheel_compat.md` 前半）作为兼容硬门的基准，再把两处硬编码字段改为按 `placement` 动态生成。验收命令用 `tests/metrics` 与 `tests/outputs`（`tests/report` 今天不存在）。

## 登记（本行特有：`outputs/builtin.py` 改动段落与测试同步理由）

- **`outputs/builtin.py` 改动段落**：待开工后填写（段落名 + 改动内容），供 p5-03 在其后串行追加。
- **`tests/metrics/test_outputs_match_legacy.py` 同步理由**：待开工后填写；断言不得减弱（`AGENTS.md` 第 7 节）。
- **报表字段名集合变化登记（D5 口径）**：待开工后填写；4 轮情形必须零变化。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记**已执行**的结果，不存虚构结果；口径见 `EPIC.md` 行 355）。
