# PROGRESS：p3-01 冻结现状事实与判据（阶段三）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-01`

## Session Start

- **Date**: （未开工；本文件为规划轮产物）
- **Task name**: p3-01-freeze
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步，全部 `TODO`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #1 — `roll_centers.py` 与 `static_loads.py` 的现状锚点全表复核
- **Current status**: NOT_STARTED
- **Last completed**: 无（本子任务尚未开工）
- **Current artifact**: `SPEC.md` / `TODO.csv`（规划产物）
- **Key context**:
  - 父行 `depends_on` **为空**（`SUBTASKS.csv` 的 `p3-01`）：本行是**纯只读冻结行**（写范围只有本行 `raw/` 与会话 scratch，冻结的是**阶段三的现状**——`roll_centers.py` 与 `static_loads.py`，不读前一阶段任何会被本 Epic 改写的产物），故**本行不受 `S1` 阻塞、也不受本 Epic 前序阶段阻塞，可立即开工**（`EPIC.md` 行 77「只读冻结行的提前开工」）。`EPIC.md` 行 219 的跨阶段顺序（阶段三在第一阶段完成后开工）**只对实施行成立**；**阶段一 `S1`（01–07 全 `DONE`）的交付只影响实施行**，不影响本行。
  - **F7（`EPIC.md` 行 126）**：`compute_vehicle_roll_centers` 在生产代码里**没有任何调用者**，唯一调用者是 `tests/physics/test_vehicle_physics.py`。所以本阶段不是重构在用链路，而是把一个未被消费的指标变成通用引擎——风险面小（无 K/C 基线依赖），但**无用例保护**。
  - **F8 与 p3-04 的判据口径（`EPIC.md` 行 128 与行 89 的 G4，2026-09-29 复审修订）**：`static_loads.py` 今天的抛错判据**实测是 `rank < 3`**（`:97-98`），`residual`（`:96`，字段声明 `:39-40`）**只记录、不参与抛错**。p3-04 要把判据改成两层：**解是否存在由载荷相容性判定**（残差超容差才报错，报错消息含残差实测值与容差）；**解是否唯一由 `rank(A) == N` 判定**（`rank(A) < N` 取最小范数解并标记「解不唯一」，4 轮 `rank = 3 < 4` 是常态）。**`rank < 3` 只表示三条平衡方程不独立，与存在性、唯一性无关、不得据此报错**；**单轮方程相容时 `rank(A) = 1 = N`、解唯一，不得标成「解不唯一」**（2026-09-29 第三轮复审修订）。故本行必须把这两处的**原文与行号**逐条固化（`EPIC.md` 行 249(a)），否则 p3-04 无从对照。
  - **p3-03 与 p3-04 的调度关系：硬串行（p3-04 在 p3-03 之后，不得并行）**（`EPIC.md` 行 228 与行 253(e)）：生产写范围 `vehicle/roll_centers.py` 与 `vehicle/static_loads.py` 不相交，但**测试写范围相交**（`tests/physics/test_vehicle_physics.py` 同时覆盖滚转中心与静平衡，归 p3-03）；`SUBTASKS.csv` 的 `p3-04.depends_on` 实测为 `p3-03`。本行记录口径时**不得写「可并行」**——测试文件已按文件级切分，但**文件级切分不是并行的理由**（`EPIC.md` 行 228 明写「两行硬串行……不得并行」）。
  - **F10（`EPIC.md` 行 132）**：`kc_baseline` 与 `dynamic_hash_baseline.json` **不含** roll center 字段；`roll_stiffness`、`track_change` 只有声明无计算实现。
  - 本行写范围仅本目录 `raw/` + 临时脚本 + 会话 scratch（`SUBTASKS.csv` 的 `p3-01` `notes`），不写任何生产代码。
- **Known issues**:
  - 四种构型（双叉臂 / 5 连杆 / 麦弗逊 / 扭梁）今天能否构造**尚未实测**；`EPIC.md` 行 314 要求本行先盘清楚，该结论是 p3-03 三构型断言的输入（`EPIC.md` 行 83 的 G3 要求四种构型各有断言）。
  - `EPIC.md` F7/F8/F9/F10 的 `file:line` 锚点**未在本行复核**；若有过期项，按 `EPIC.md` 行 107 的口径记「已过期」并写明当前真实行号。F8 侧还要额外固化两处（`rank < 3` 抛错段与 `residual` 计算口径），这两处是 p3-04 判据纠正的依据（`EPIC.md` 行 249(a)）。
  - 既有失败清单与起点 skip/xfail 计数**未实测**（`EPIC.md` 行 320 要求各 p*-01 记录起点值）。
- **Next action**: 先读 `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py` 与 `.../vehicle/static_loads.py`，按 `EPIC.md` F7（行 126）/ F8（行 128）的 `file:line` 逐条取原文并把硬点别名表与 `_WHEELS` **全表**落盘到 `raw/anchors_roll_centers_static_loads.md`（**含 `:97-98` 的 `rank < 3` 抛错段与 `:96` 的 `residual` 计算口径**）；随后对四种构型各写一份最小几何模型声明并实跑构造，把「可构造 / 不可构造 + 拒绝原文」落到 `raw/config_refusals.md`。

---

## Final Summary（未开工）

本子任务**尚未开工**。`SPEC.md` 与 `TODO.csv` 是规划产物：未执行任何步骤、未修改任何生产代码或测试、未产生任何证据（`raw/` 为空）。所有 `TODO.csv` 行保持 `TODO`，`completed_at` 为空，`retry_count` 为 `0`。开工时按 `TODO.csv` 顺序展开，并把每一步的实际命令、退出码与产物落到 `raw/`（只记**已执行**的结果，不存虚构结果；口径见 `EPIC.md` 行 355）。
