# PROGRESS：p3-03 通用滚转中心（摆脱硬点名称嗅探）

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-03`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p3-03-roll-centers
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-03-roll-centers/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）
- **Status**: **DONE**

## Context Recovery Block

- **Current milestone**: #6 — 依赖方向与零回归自证（全 6 步完成）
- **Current status**: DONE
- **Last completed**: 六份证据落 `raw/`，测试与三条架构门实跑通过
- **Current artifact**: `raw/`（6 份 `.md` + 5 个复现脚本 + 1 份完整 stdout）
- **Key context**:
  - 前置 p3-02 DONE，引擎契约 `tasks/p3-02-diffkinematics/raw/engine_contract.md` 已冻结；本行只消费，未改引擎。
  - 与 p3-04 硬串行：本行写 `vehicle/roll_centers.py`，p3-04 写 `vehicle/static_loads.py`；
    `tests/physics/test_vehicle_physics.py` 归本行，p3-04 只写它新建的 `tests/physics/test_static_loads.py`。
  - G3 四条判据（(a) 前后对照、(b) 三种构型断言、(c) 名称嗅探删除、(d) 通道命名、
    (e) 虚功导数矩阵）全部达成，逐条见下述证据。
  - **本行不复现 K/C 读数变化**：`compute_vehicle_roll_centers` 在生产代码里无调用者；
    数值门三项归 p3-06 与 Epic 收尾统一实跑（本文件第 8 节明确记账「未跑」）。

## 交付物

| 文件 | 性质 |
|---|---|
| `packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py` | 重写（132 → 273 行） |
| `packages/suspension_multibody/tests/physics/test_roll_centres_by_topology.py` | 新增，11 用例 |
| `packages/suspension_multibody/tests/physics/test_vehicle_physics.py` | 滚转中心断言更新 |
| `packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py` | 路由钩子用例改名 + 新字段名 |

## 六条判据的落点

| `SPEC.md` 判据 | 证据文件 | 结果 |
|---|---|---|
| (a) 改造前后双叉臂并列对照 | `raw/before_after_double_wishbone.md` | 4 处差异、**未解释差异 0 条**，全部归入数值微分步长 1e-6 的截断误差 |
| (b) 三种构型各有断言 | `raw/three_topologies.md` | 5 连杆 h=+37.908、麦弗逊 h≈−3.6e-7、扭梁 h≈−5.2e-9；11 用例通过 |
| (c) 硬点别名表与二维交点路径删除 | `raw/name_sniffing_removed.md` | `grep` 退出码 1（零命中）；5 个旧符号均已删 |
| (d) 报表侧通道命名 | `raw/channel_naming.md` | **未暴露**（`report/`/`outputs/`/`studies/` 零命中、零改动） |
| (e) 侧倾反力虚功导数矩阵 | `raw/roll_center_virtual_work.md` | 矩阵构造 + 瞬心对照 + 两条独立数值判据（有限位移、ΔF_y 力矩乘积） |
| 依赖方向与零回归 | `raw/run_log.md` §6 | `check_module_layering --strict --final` 0 环；`legacy_surface_gate --check` findings 0；`test_import_boundaries` 56 passed |

## 实跑命令与结果（原文见 `raw/run_log.md`）

| 命令 | 结果 |
|---|---|
| `pytest tests/physics tests/vehicle -q` | `93 passed, 1 xfailed`（xfail 为既有项） |
| `pytest tests/physics/test_roll_centres_by_topology.py --collect-only` | `11 tests collected` |
| `check_module_layering.py --strict --final` | exit 0，`module cycles (SCC size>1) : 0` |
| `legacy_surface_gate.py --check` | exit 0，`findings : 0` |
| `pytest tests/architecture/test_import_boundaries.py -q` | `56 passed in 574.46s` |
| SPEC 的 Final Validation Command | 两段均退出 0 |

## 本行自查发现并修正的两处缺陷（诚实记账）

1. **字段名与实际内容不符**：`RollCenterResult.force_line_slope` 存的是接地点自身轨迹的
   `d y / d z`（即 `r`），而力线的 `d z / d y` 等于 `-r`。名字与内容对不上。
   写判据 (a) 的对照表时按 `1/r` 换算得到 `y=0` 交点 **3125.0 mm**（与瞬心的 −180 差 3305 mm）才暴露。
   → 改名 `contact_patch_slope`，修正 docstring。**断言与容差一律未动**。
2. **扭梁夹具的梁关节两点不重合**：`_twist_beam` 把梁关节写成 `y = ∓60`，
   位置残差实测 `max|C| = 1.2e+02 mm`——机构根本没装上，此前那条读数是在未装配机构上取的噪声，
   而三条 parametrize 断言全都过了（**没有一条能发现它**）。
   → 两点同取 `_BEAM = (1180, 0, 250)`，残差归零。读数变为 `-5.25e-09`（结论不变，但这次是真的）。
   装配残差已作为 4b 节写进 `raw/three_topologies.md`——**「有限值 + 对称」不足以证明机构成立**。

## 未做（明确记账，不声称已做）

- **未跑 `just gate-numeric` 三项**（`dynamic_hash_sentinel --check` / `case_parity_check` /
  `kc_perf_gate --check`）。本行不是阶段收尾行；这三项按 `EPIC.md` 归 p3-06 与 Epic 收尾。
- **未跑 `just check-fast` 全量**。本行只跑了受影响的两个测试目录与三条架构门。
  完整快速集在 p3-06 阶段收尾统一跑。
- **未重录任何基线**，也未改 `tests/data/` 下任何文件。

---

## Final Summary

p3-03 **DONE**。`compute_vehicle_roll_centers` 改为读装配自身的约束集（经 p3-02 引擎），
滚转中心高由 `h = -Q_phi / Q_uy` 解出，其中 `B = [[-1, y_i·r_i]]`、
`Q = B^T dF`。硬点别名表、`_hardpoint`、`_instant_center`、`_line_intersection`
与 `side_hardpoints` 的 import 全部删除，`grep` 在 `vehicle/` 零命中。
四种构型（双叉臂 + 5 连杆 + 麦弗逊 + 扭梁）各有可判定的数值断言。
双叉臂与改造前瞬心法对照，差异 2.688e−07 mm，**唯一来源**是引擎 1e-6 步长的截断误差，
未解释差异为零。两条独立数值判据（有限位移法不经过 `twist_of`；ΔF_y 与侧倾力矩的
乘积关系）与主路径一致到 1e-4 mm 量级，容差与理由均写在 `raw/roll_center_virtual_work.md`。
