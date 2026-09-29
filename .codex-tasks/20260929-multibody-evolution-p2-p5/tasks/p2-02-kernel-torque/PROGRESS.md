# PROGRESS：p2-02 内核旋转主动力矩元与 ABI 变更

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-02`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p2-02-kernel-torque
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-02-kernel-torque/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest / C++ 内核（本行改内核源码）

## Context Recovery Block

- **Current milestone**: #1 — 内核新增旋转主动力矩元素类型并打通五处
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - **D1 必须先裁决**（`EPIC.md` 行 60 与行 311，「是否接受内核 ABI 变更 16→17」）。D1 未裁决前本行不得置 `IN_PROGRESS`。
  - 前置 `S1`：阶段一 Epic 01–07 全部 `DONE`；**阶段一未完成之前本 Epic 任何一行不得置 `IN_PROGRESS`**（`EPIC.md` 行 67–75）。
  - **本行是本 Epic 唯一被授权改 `mb_config/version.hpp` 与 `kernel/native.py` 的行**（`EPIC.md` 行 225；`SUBTASKS.csv` `p2-02` `notes`）；版本常量单一真源由 `tests/architecture/test_kernel_abi_version_single_source.py` 守护。本 Epic 最多两次 ABI 变更（本行一次，p5-04 一次视 D2）。
  - **只新增、不动既有**：本行只新增一个元素类型、不改既有元素的装配顺序，故 13 个轴侧动态用例的 `dynamic_hash_sentinel.py --check` 应仍逐字节一致；**若变化即回退**（`EPIC.md` 行 239 (e)、行 311）。
  - 禁止触碰 `subsystems/element_build.py`（p2-03）、`subsystems/brake.py`/`drive.py`/`templates/builtin.py`（p2-04）、`templates/roles.py` 的角色表（p4-02，行 219）、`preparation/vehicle_dynamic.py`（p2-05/p2-06/p4-04）。
- **Known issues**:
  - **ABI 常量升法待实测确认**：`EPIC.md` D1（行 60）要求「轴与整车两个常量至少各 +1」（`version.hpp:18` 注释表明 `kVehicleKernelAbiVersion` 随轴结构联动），F3（行 116）给出今天的值 `:27=16` / `:34=31` / `:37=1`。**具体升哪个、升到多少由本行按代码实测确定并登记**；本行只受「单点提交 + 两处真源同步」两条约束。
  - 契约 schema 若需新增元素类型字段，必须与阶段一 03 的放置段改动**串行**（`EPIC.md` 行 218）——这是 `S1` 前置的第二个理由。
  - `just gate-numeric` 本行必须跑（触及内核求解路径，`EPIC.md` 行 230）；数值门三项的起点值取自 p2-01 的 `raw/gates_baseline.md` 与 `raw/baseline_notes.md`。
- **Next action**: 先读 `packages/suspension_kernel/cpp/include/mb_config/version.hpp:27/34/37` 与 `packages/suspension_multibody/src/suspension_multibody/kernel/native.py:33-35` 与 `packages/suspension_kernel/cpp/include/mb_input/types.hpp:65-76`（`ElementKind` 七类）与 `mb_model/types.hpp:311-333`（`Model` 元素向量），确认 D1 裁决结论后再决定升哪个常量，然后按 `raw/kernel_element_path.md` 的五处清单展开。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
