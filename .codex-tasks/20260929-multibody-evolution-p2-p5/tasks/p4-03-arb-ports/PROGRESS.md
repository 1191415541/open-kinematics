# PROGRESS：p4-03 悬架 arb_mount 语义端口与配对插接

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p4-03`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p4-03-arb-ports
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-03-arb-ports/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 悬架声明语义化端口（含 `arb_mount_L/R`）与端口合成段改造
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - `depends_on = p4-02`；防倾杆的 4 个端口由 p4-02 交付，本行才可能让它「插得上」。
  - **F13 现状**（`EPIC.md:141`）：悬架今天声明**零个 port**（`builtin.py:397 DOUBLE_WISHBONE` 的 `ports`/`needs` 皆为空元组）；端口在装配期运行期合成（`subsystems/si_assembly.py:71 _ports_for_bodies`、`:104 _wheel_centre_needs`）；`arb_mount`/`droplink_mount`/`chassis_mount` 在源码与测试里**不存在**。所以本行是「先造出语义化端口」。
  - **`_BODY_ALIASES` 归本行收口**：`subsystems/geometry.py:226`（含 `"wheel": "upright"`，消费 `:258/265`），F16 修正项（`EPIC.md:147`），不单独扩范围。
  - **插接只经阶段一 02 的 `match_requirements`**，不得另立第二条推断路径；不得出现按名字猜身份的规则（`EPIC.md:233`）。
  - 全局前置 `S1`：阶段一 01–07 全部 `DONE`（含 02 配对机制与 03 通用装配引擎）；**未完成之前不得置 `IN_PROGRESS`**（`EPIC.md:75`）。
- **Known issues**:
  - 尚未开工，故 F13/F16 各锚点为**待复核**；任何锚点过期只在本行 `raw/` 与本节记录，不改父文件。
  - 与 p4-02 的边界：本行开工前先跑 p4-02 的验收命令确认 4 端口已声明；边界不符即停工回报，不自行扩范围。
  - `_ports_for_bodies` 改为声明后可能改变既有装配产物；需逐项对照并在必要时登记。
- **Next action**: 先读 `subsystems/si_assembly.py:71 _ports_for_bodies` 与 `:104 _wheel_centre_needs` 的现状，以及 `subsystems/geometry.py:226 _BODY_ALIASES` 与其消费点 `:258/265`；确认 p4-02 的 4 端口已落地，然后在悬架的端口**声明**里加入 `arb_mount_L/R` 并跑 `tests/subsystems`、`tests/connections`、`tests/authoring` 与契约包测试。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
