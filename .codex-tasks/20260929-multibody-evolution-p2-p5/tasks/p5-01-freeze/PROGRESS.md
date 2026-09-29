# PROGRESS：p5-01 冻结现状事实与判据（阶段五）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-01`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p5-01-freeze
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-01-freeze/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest；仓库根 `c:/杂件/open-kinematics`
- **Depends on**: 父行 `depends_on` **为空**——本行是纯只读冻结行（写范围仅本行 `raw/` 与脚本与会话 scratch），**不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行（p5-02 ~ p5-06 与终局验收行）。

## Context Recovery Block

- **Current milestone**: #1 — 公共面清单与调用者全量盘点
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 本行是**只读冻结轮**：写范围仅 `raw/` 与脚本与会话 scratch，不写生产代码（`SUBTASKS.csv` 第 19 行 `notes`）。
  - 全仓公共面今天只有 `api.py:100 run_case(model: FrontAxleModel, ...)` 与 `:154 run_dynamic_case(model: FrontAxleModel, ...)`；`simulate` 不存在，`FrontAxleModel` 未退役（仍被 `schema/vehicle.py:252-253` 使用，`EPIC.md` F18）。
  - 三个公共 API 门禁的 allowlist 为 `mode = "strict"` 且**无条目**、registry 的 `entry` 为 `[]`——**零容忍**（`EPIC.md` F24）。
  - 内核是**批式 ABI**（`cpp/axle_dynamics/core_abi.hpp:135-136`，无 step/state 入口），故实时闭环在当前 ABI 下无法真正实现（`EPIC.md` F23，D2 的硬约束）。
- **Known issues**:
  - `48 个测试文件` 与 `生产约 20 处` 是 `EPIC.md` F18 的实测值，本行必须重新实测；数量不一致时以本行实测为准并写明差异来源。
  - 本行需要 `schema/model.py:157` 的字段形状与 `model_dump(mode="json")` 的哈希实测值作为 p5-02 的硬门基准；若该哈希在 p5-02 之前已被别的行改动，须在 p5-02 的 Known issues 中登记。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/__init__.py:43-80` 的 `_PUBLIC_NAMES`/`__all__` 与 `api.py:100/:154`，确认 17 个公开名与两个入口的原文，再按 `TODO.csv` 第 1 步起逐条实测并把结果写入 `raw/`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
