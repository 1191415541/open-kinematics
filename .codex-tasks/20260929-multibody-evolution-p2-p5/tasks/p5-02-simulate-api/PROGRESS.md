# PROGRESS：p5-02 simulate 公共入口与 FrontAxleModel 降级

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-02`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p5-02-simulate-api
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-02-simulate-api/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: `p5-01`（`SUBTASKS.csv` 第 20 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 69/75）约束

## Context Recovery Block

- **Current milestone**: #1 — `simulate` 公共入口：签名、文档与 `__all__` 登记
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 公共入口今天是 `api.py:100 run_case(model: FrontAxleModel, ...)` 与 `:154 run_dynamic_case(model: FrontAxleModel, ...)`；`simulate` 不存在（`EPIC.md` F18）。
  - 写范围按 `SUBTASKS.csv` 第 20 行 `notes`：`api.py` 与 `__init__.py` 与 `__all__` 与 examples 文档。
  - **`model_dump(mode="json")` 的形状不得改**：`api.py:116` 与 `:284` 用它算 `model_hash`（`EPIC.md` 冻结约束行 227）。
  - 三个公共 API 门禁的 allowlist 为 `mode = "strict"` 且**无条目**、registry 的 `entry` 为 `[]`（`EPIC.md` F24）——零容忍。
  - 内核提交唯一归属仍只有 `simulation/backend.py`（`EPIC.md` 行 100；`_is_owner` 在 `test_public_api_boundary_gate.py:74`）。
- **Known issues**:
  - `simulate` 不得另起第二条装配路径：它只是阶段一「文档驱动装配器」的对外出口（`EPIC.md` 行 73）。
  - 本行与 p5-03 串行：p5-03 要在 `api.py` 的**暴露段**新增信号总线出口（`SUBTASKS.csv` 第 21 行 `notes`），故本行须先把公开出口结构定稳。
  - `composable_extension_examples.md` 的 `runnable` 代码块会被 `check_composable_release.py` 执行（`EPIC.md` F24）——示例文档改动直接进门禁。
- **Next action**: 先读 `api.py:100/:154/:116/:284` 与 `__init__.py:43-80`，以及 p5-01 落盘的 `raw/public_surface.md`（17 个名字、调用者台账、`model_hash` 基准），确认公开出口与降级面，再按 `TODO.csv` 第 1 步起展开。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
