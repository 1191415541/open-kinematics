# PROGRESS：p2-06 分布式转向通道与转向分配器

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-06`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p2-06-steering-channels
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-06-steering-channels/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 解除准备层与文档导出层两层后轮转向限制
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 依赖 p2-05（`SUBTASKS.csv` `p2-06` `depends_on`）：同一个 `preparation/vehicle_dynamic.py` 上三段串行，**本行（转向段）必须在 p2-05（力矩段）之后**；**p4-04（轮胎拒绝段）在本行之后**（`EPIC.md` 行 217）。
  - 本行是**阶段二的收尾行**，也是 **G2 的实现行**（`EPIC.md` 行 81）：三条判据（两通道装配跑通、4WS/阿克曼在相同方向盘输入下符合分配律、无「必须为真」校验）全部由本行交证据。
  - **与阶段一 05 同口径**（`EPIC.md` 行 313）：`cases/vehicle_kc.py:128-136` 的准备期删除与阶段一的试验台非侵入是同一类问题，沿用阶段一「只增不删」的口径；**两处改动分属不同 Epic，必须点明关系避免重复**。
  - **最大连锁风险是 `model_dump(mode="json")` 形状**（`EPIC.md` 行 227）：`schema/vehicle.py` 转向段从单例改通道列表后，单通道声明的 `model_dump` 形状必须与改造前逐项一致（`api.py:116`/`:284` 用它算 `model_hash`）。
  - **角色名集合与硬断言归 p4-02**（`EPIC.md` 行 227）：`templates/roles.py` 的 `ROLES` 角色集合与 `:158-162`、三份契约 schema 的 `functional_role` enum 全部归 p4-02；**p2 的任何行不得增删角色名**。
- **Known issues**:
  - **新增转向分配器模块的落点 `EPIC.md` 未给路径锚点**：只给了职责（按方向盘转角、车速与模式解算各通道输入，路线图 2.3 节，`EPIC.md` 行 25）。落点属**开工时复核项**，必须遵守分层方向（行 226）。
  - **无既有 4WS / 多通道转向用例**（`EPIC.md` F6 行 122：`grep "4WS|four.wheel.steer"` 零命中）——本行是从零建保护，不能靠既有用例。
  - `tests/authoring/test_vehicle_assembly_documents.py:218-220` 断言 `model.rear_axle.rack_fixed_to_chassis is True`（F6 行 122），本行解除限制时**必然要改它**，须按「改前 → 改后 → 理由」登记。
  - `subsystems/steering.py:194/:234`（F4 行 118）由 `model.rack_fixed_to_chassis` 在 `WeldJoint`/`PrismaticJoint` 之间二选一；该文件的角色相关部分归 p4-02，本行若需改非角色段必须在 PROGRESS 登记并与 p2-04/p4-02 的写范围核对不相交。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py:205-211`（`_validate_steering_topology`，调用点 `:222`）与 `authoring/vehicle.py:112-119`（`:119` 写死点）与 `schema/vehicle.py:143-153`/`:255`（`SteeringSystemSpec` 与单例字段），并对照 p2-01 的 `raw/rear_steer_refusal.md`（改造前拒绝原文），再按 TODO 第 1 步解除两层限制。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记已执行的结果）。
