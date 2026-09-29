# PROGRESS：p2-04 brake 与 drive 子系统改造为力矩元

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-04`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p2-04-brake-drive
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — BRAKE 与 DRIVE 模板属性槽按路线图 2.1 标准化
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 依赖 p2-03（`SUBTASKS.csv` `p2-04` `depends_on`）：力矩元的构造分派与编译层接入必须先落地，本行才有可产出的目标类型。
  - 前置 `S1`：阶段一 Epic 01–07 全部 `DONE`；**阶段一未完成之前本 Epic 任何一行不得置 `IN_PROGRESS`**（`EPIC.md` 行 67–75）。
  - **与 p4-02 强制串行**（`EPIC.md` 行 215）：`templates/roles.py` 与 `templates/builtin.py` 是共享注册文件，**p4-02 在本行之后**。
  - **`templates/roles.py` 的分段归属**（`EPIC.md` 行 227）：`ROLES` 的角色**名字集合**与 `:158-162` 的 import 期硬断言归 p4-02；本行**允许**改 `"brake"`/`"drive"` 两个条目自身的内容（如 outputs 与 `has_torque_channel`，`EPIC.md` F1（行 119）的锚点 `:122-152`），**禁止**增删角色名、**禁止**改 `:158-162` 的断言集合、**禁止**碰三份契约 schema 的 `functional_role` enum 与 `FUNCTIONAL_ROLES`/`SUBSYSTEM_ROLES`/`ALL_SUBSYSTEMS`/`ROLES`（`connections/policy.py`）。
  - 判据 (b) 的落点：`subsystems/brake.py:141` 与 `drive.py:110` 的 `wheel_torque_amplitudes()` 返回 `dict[str, tuple[float, ...]]`（`EPIC.md` F1 行 112）；`brake.py:160-161` 明说幅值交给内核的 `brake_torque` 通道按轮轴向速度反向。
- **Known issues**:
  - **`wheel_torque_amplitudes()` 的消费点散落**：除 `brake.py`/`drive.py` 自身，外部还有 `cases/vehicle_dynamic.py:579/582` 的契约表——那属 **p2-05** 的写范围，本行**只登记、不修改**。
  - **`templates/roles.py` 的写法边界已由父真源界定**：`EPIC.md` 行 227明确「p2-04 可改 brake/drive 的 `RoleSpec` 内容段，p4-02 才能改角色集合与 `:158-162` 断言」；本行开工时仍须先读 `roles.py:122-152` 与 `:158-162`，把实际改动的条目与字段在 PROGRESS 里显式写清，并对 `"brake"`/`"drive"` 条目的每处改动给「改前 → 改后 → 理由」。
  - **外部消费点是否受影响需实测**：`EPIC.md` F6 行 122 列出 `tests/vehicle/test_native_vehicle.py:1607/:1721`（制动）与 `:1500/:1630`（驱动）、`tests/cases/test_vehicle_dynamic_contract.py:185`——本行须实测说明，不得默认「不受影响」。
  - `tests/authoring/test_vehicle_assembly_documents.py:218-220` 断言 `model.rear_axle.rack_fixed_to_chassis is True`（F6 行 122），那是 **p2-06** 的改动对象；本行不得触碰。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/templates/builtin.py:594`（`_BRAKE_MOUNTS`）、`:599`（`BRAKE`）、`:622`（`DRIVE`）与 `templates/roles.py:122-152`、`:158-162`，以及 `subsystems/brake.py:141`、`:160-161`、`drive.py:110`，确认属性槽改前的字段清单、`"brake"`/`"drive"` 两个 `RoleSpec` 条目可改的内容段，以及 `:158-162` 断言的不可改边界，再按 TODO 第 1 步展开。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
