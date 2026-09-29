# PROGRESS：04 轮端生命周期统一

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `04`

## Session Start

- **Date**: （未开工）
- **Task name**: 20260929-04-wheel-lifecycle
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-04-wheel-lifecycle/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 单轴与整车引用同一份 wheel 子系统文件
- **Current status**: NOT_STARTED
- **Last completed**: 无
- **Current artifact**: `SPEC.md`
- **Key context**: 本子任务**尚未开工**——`SPEC.md` / `TODO.csv` 是规划产物，本文件是它的恢复块，未执行任何步骤、未产生任何证据。开工前置：`03` 完成（`03` 与 `04` 都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，必须串行）；`01` 的 `#1` 完成（`snapshot.py --check` 才可用作"产物是否变化"的判据）。
- **Known issues**: 本行唯一硬门是 D2——凝结后 `kc_baseline` 必须逐位不变；**严禁重录**。`kc_parity_check.py` 不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），不构成证据。
- **Next action**: 先跑一次 `tasks/20260929-01-freeze/snapshot.py --check`（待 01 的 `#1` 完成后）与数值门三项记录起点，再按 `TODO.csv` 从 `#1` 开始。

## 待开工登记（规划产物，非实施记录）

- 本子任务尚未开工；`SPEC.md` 与 `TODO.csv` 为规划产物，所有 `TODO.csv` 行 `status` 均为 `TODO`、`completed_at` 为空。
- 本文件只记录开工所需的恢复信息与已确定的口径，**不得**用规划文本冒充实施记录；`raw/` 只存**已执行**的证据。

## 已确定的口径（来自父 Epic 裁决，不待实施确认）

- **D2（裁决）**：不反转既有裁决 **D9**（单轴侧车轮归悬架试验台），**严禁重录 `kc_baseline`**。走"文件层统一 + 装配期刚性凝结"：单轴与整车引用**同一份** `wheel.subsystem.json`；装配单轴 K/C 试验台（轮心驱动工况）时把车轮刚体与轮毂刚体在内存中刚性凝结。硬门是凝结后**实体集合 / 约束行数 / 自由度不变且 `kc_baseline` 逐位不变**。
- **凝结复用既有机制**：`subsystems/vehicle_parts.py` 的 `_merge_fixed_wheel`（`:572-610`；`:482-508` 是 `wheel.mount_joint_kind == "fixed"` 的分支，按复合质量属性把车轮质量/惯量并入 mount body 且不建独立车轮体）与同文件的 `_fuse_welded_bodies`（由 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复，F5b）。不新造凝结机制。
- **与 20260921 Epic A3 的关系（必须点名，避免两处"凝聚"语义混淆）**：20260921 Epic 的 A3 裁决把**整车侧**生产路径改为"不凝聚、weld 交内核 `fixed` 关节"；那是**另一处**的取舍，与本 Epic **单轴 K/C 侧**要凝结不冲突。本行只动单轴 K/C 侧，不改整车侧的 A3 结论。
- **不得迁移轮胎质量所有权**：凝结只改求解拓扑的表示，质量 / 质心 / 惯量口径不变（`raw/mass_ownership.md` 为证据载体）。
- **K/C 对标口径**：先 `scripts/kc_native_probe.py` + `scripts/kc_native_c_probe.py` 生成 actual，再 `scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` 判定；**不得**用不带 `--actual-dir` 的自比较。

---

## Final Summary（未开工）

（`TODO.csv` 的 6 步全部为 `TODO`；本子任务尚未开工，未执行任何步骤、未产生任何证据）
