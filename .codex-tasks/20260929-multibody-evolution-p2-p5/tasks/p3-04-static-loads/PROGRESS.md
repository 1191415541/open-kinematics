# PROGRESS：p3-04 N 点接触面广义静平衡求解

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-04`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p3-04-static-loads
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（7 步，全部 `TODO`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #1 — `_WHEELS` 删除且静平衡按实际接触点集合构造方程（解的存在性由载荷相容性判定、唯一性由 `rank(A) == N` 判定：`rank(A) < N` 取最小范数解并标记「解不唯一」，残差超容差才报错点名）
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on = p3-03`（`SUBTASKS.csv` 的 `p3-04`；两者都依赖 p3-02）；跨阶段顺序为「阶段三在第一阶段完成后开工」（`EPIC.md` 行 211），且**前置 S1**——阶段一 Epic 01–07 全部 `DONE` 之前本行不得置 `IN_PROGRESS`（`EPIC.md` 前置一节：19 行实施行受此约束，四个只读冻结行不受限）
  - **生产写范围与 p3-03 不相交，但测试写范围相交**（`EPIC.md` 行 228）：本行写 `vehicle/static_loads.py`，p3-03 写 `vehicle/roll_centers.py`；`tests/physics/test_vehicle_physics.py` **归 p3-03**，本行**不改它**，静平衡断言全部写入**新建**的 `tests/physics/test_static_loads.py`（`tests/vehicle/` 下同理新建）。**两行硬串行**（`SUBTASKS.csv` 的 `p3-04.depends_on = p3-03`，**不得并行**；测试文件已按文件级切分，但文件级切分不是并行的理由）。
  - **4 轮逐位一致是本行硬门**（`SUBTASKS.csv` 的 `p3-04` `notes`、`EPIC.md` 行 255(c)）：因为 `static_loads.py` 有**真实生产调用者** `vehicle/service.py:40`（F8 `EPIC.md` 行 128），本行是阶段三唯一「改在用链路」的行。
  - **p3-05 依赖本行**（`SUBTASKS.csv` 的 `p3-05.depends_on=p3-04`）：报表要消费本行的广义静平衡字段，故本行**必须冻结输出字段形状**（`raw/static_loads_contract.md`）。
  - **F8（`EPIC.md` 行 128）**：`_WHEELS` 在 `static_loads.py:28`；3×4 平衡矩阵与 `lstsq` 在 `:79-95`；四点不张成抛 `ValueError` 在 `:98`；唯一生产调用点是 `vehicle/service.py:40`。
  - **G4 判据（`EPIC.md` 行 89 与行 259(b)，路线图 `docs/multibody_architecture_evolution.md:188`）**：**单轮（N = 1）在载荷相容输入下必须可解、在载荷不相容输入下报错点名，不得一律写成报错**——本行必须给「可解数值例」与「不可解报错例」各一个用例。**可判定边界分两层：解是否存在由载荷相容性（残差 `‖A x − b‖` 在容差内）判定；解是否唯一由 `rank(A) == N` 判定**。`rank(A) < N` 时取最小范数解并标记「解不唯一」（4 轮 `rank = 3 < 4` 是常态）；**`rank(A) < 3` 只表示三条平衡方程不独立，与存在性、唯一性无关，不得据此报错**；**单轮方程相容时 `rank(A) = 1 = N`，解唯一，不得标成「解不唯一」**。同一单轮几何的矩阵秩不随载荷改变，用秩无法区分两例（2026-09-29 第三轮复审修订）。
- **Known issues**:
  - **接触点来源未定**：实际接触点集合从哪个既有事实面取尚未实测确定；若必须改 `vehicle/service.py` 才能传接触点，按 `SPEC.md` 登记为范围外并回退给主代理裁决。
  - **`lstsq` 最小范数解口径**：N=4 必须保持今天的解；N>4 的求解口径允许变化但须说明（说明独立于结果字节：自由度、约束行数、接触点几何、力路径，`EPIC.md` 行 228）。
  - **报错语义已改**：今天 `static_loads.py:97-98` 的 `ValueError` 判据实测是 `rank < 3`，语义是「四点不张成」；新的报错语义是「**载荷不相容（残差超容差）**」，判据是**残差**。既有测试若断言旧消息，须检查语义等价性并登记理由；改测试只改本行新建的文件，**不得动 p3-03 的 `tests/physics/test_vehicle_physics.py`**。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py` 全文件（含 `:39-40 rank/residual` 字段、`:95-98` 的 `lstsq` 与抛错段）与 `.../vehicle/service.py:40` 的调用点，确认今天的输入（四轮名常量 + 偏移）与输出形状；随后**先落盘 4 轮的改造前逐位基准**（`raw/four_wheel_bitwise.md` 前半），再改 `_WHEELS` 与方程构造，并按「**解的存在性看残差**（在容差内即求解）/ **解的唯一性看 `rank(A) == N`**（`rank(A) < N` 取最小范数解并标记「解不唯一」；`rank(A) < 3` 只表示方程不独立）/ 残差超容差才报错点名（消息含残差实测值与容差）」实现，最后在新建的 `tests/physics/test_static_loads.py` 里补 3 轴 6 点与单轮两例（载荷相容可解例 + 载荷不相容报错例）。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记**已执行**的结果，不存虚构结果；口径见 `EPIC.md` 行 355）。
