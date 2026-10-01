# PROGRESS：p3-04 N 点接触面广义静平衡求解

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p3-04`

## Session Start

- **Date**: 2026-09-29
- **Task name**: p3-04-static-loads
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（7 步，全部 `DONE`）
- **Environment**: Python 3.12 / uv / pytest（`uv run --no-sync`）

## Context Recovery Block

- **Current milestone**: #7 — 依赖方向与零回归自证（全部完成）
- **Current status**: DONE
- **Last completed**: 步骤 7（`check_module_layering --strict --final`、`legacy_surface_gate --check`、
  `check_composable_release --skip-isolation`、`dynamic_hash_sentinel --check` 全部退出码 0；
  快速集 `1162 passed, 1 xfailed`；未新增 skip/xfail）
- **Current artifact**: `raw/` 下 10 个文件（4 篇证据 + 2 个 probe 脚本 + 2 份 probe 输出 +
  改造前文件存档）
- **Key context**:
  - **改造后的形状**：`vehicle/static_loads.py` 现在有两个入口。
    `compute_static_wheel_loads(vehicle, *, acceleration, gravity, road_z)`（旧签名逐字未变，
    `vehicle/service.py:40` 走这里）只做装配，真正求解在
    `compute_static_wheel_loads_for_assembly(assembly, *, ...)`（新增，任意 N）。
  - **接触点来源（实测确定）**：装配值的 `VehicleRuntime.wheel_centers`
    （`_support_points`，`static_loads.py:265`）。集合与顺序都是车辆自己的：
    两轴车 4 点、三轴车 6 点、单轮台架 1 点。
  - **两个判定分开**：存在性 = 载荷相容 = `residual <= residual_tolerance`
    （容差 = `1e-9 * max(|total_mass*(gravity+accel_z)|, 1.0)`）；
    唯一性 = `rank(A) == N`，结果字段 `unique`。`rank(A) < 3` 不报错、不参与判定。
  - **报错**：`IncompatibleStaticLoadsError`（`ValueError` 子类）取代旧的
    `ValueError("four wheel support points do not span force/moment balance")`；
    消息含残差实测值、容差、接触点数 N，不含秩。
  - **4 轮逐位一致**：硬门已实跑通过（`raw/four_wheel_bitwise.md`，
    `DIFFERENCES: none`，6 个工况逐字段 `struct.pack('>d')` 比较）。
  - **给 p3-05**：输出字段形状冻结在 `raw/static_loads_contract.md`。注意
    `result.summary` 仍只支持恰好四角（`report/wheel_loads.py` 归 p3-05），
    三轴/单轮请直接读 `wheel_loads`。
- **Known issues**:
  - `uv run --no-sync ruff check .` 最终退出码 **0**（`All checks passed!`）。更早一轮为 1，
    2 个发现均在 `.codex-tasks/.../tasks/p2-07-doc-route/raw/document_route_probe.py:3`
    （`E401` + `F401`）：属 p2-07 的文件，本行不碰，其归属者随后自行修好。
  - 执行期间遇到两次**并发写入干扰**（都与本行改动无关，重跑即恢复）：
    另一个写入者重建 native kernel 时 `tests/vehicle/test_pac2002_contact_mass.py` 收集期抛
    `NativeKernelUnavailableError`；另一个写入者改 `subsystems/*.py` 与 `templates/builtin.py` 时
    `test_import_boundaries.py::test_every_ordered_pair_of_entry_points_imports` 报 19 个
    `SyntaxError: keyword argument repeated: note`。后者重跑 **`56 passed`**，即该文件已通过。

## Final Summary

7 步全部完成，`TODO.csv` 全 `DONE`。

**做了什么**

1. `vehicle/static_loads.py`：
   - 删除 `_WHEELS` 四轮名字常量；
   - 静平衡方程改由装配值 `wheel_centers` 的**数据**构造（N 个接触点 = N 个未知量、3 条方程）；
   - 新增 `compute_static_wheel_loads_for_assembly`（N 点求解的正式入口）；
   - 新增结果字段 `unique`（`rank(A) == N`）与 `residual_tolerance`；
   - 新增 `IncompatibleStaticLoadsError`，判据改为**残差超容差**，消息含残差、容差与 N；
   - 容差 = `1e-9 × max(竖直载荷, 1 N)`，与点数、结果字节无关。
2. 新建 `tests/physics/test_static_loads.py`（436 行、9 个用例）：三轴 6 点两例、
   单轮可解例、单轮不可解报错例两例、秩无法区分两例的显式断言、4 轮最小范数解回归例、
   接触点 == 装配轮端表的断言、`VehicleModel` 入口仍工作的断言。**未改**
   `tests/physics/test_vehicle_physics.py`。

**验证（全部实跑，原始输出见 `raw/`）**

| 项 | 结果 |
|---|---|
| `pytest tests/physics tests/vehicle -q -p no:cacheprovider` | **0**：`102 passed, 1 xfailed` |
| `dynamic_hash_sentinel.py --check` | **0**：`OK: dynamic output matches the frozen baseline byte-for-byte` |
| grep `_WHEELS = ("front_left"` 于 `vehicle/`（取反） | **0**：零命中 |
| `ruff check .` | **0**：`All checks passed!` |
| `ty check .` | **0**：`All checks passed!` |
| `check_module_layering.py --strict --final` | 0：0 环 |
| `legacy_surface_gate.py --check` | 0：findings 0 |
| `check_composable_release.py --skip-isolation` | 0：3 项通过 |
| 快速集（除 adams/architecture/cases） | 0：`1162 passed, 1 xfailed` |
| `pytest suspension_kernel/tests suspension_contracts/tests` | 0：`73 passed` |
| 4 轮逐位比较 | `DIFFERENCES: none`（6 工况 × 全字段双精度字节） |

**未做**：`tests/architecture/` 全目录、`tests/adams/`、`tests/cases/`、
`just gate-numeric` 的 `case_parity_check.py` 与 `kc_perf_gate.py`、全量 `just test-all`
—— 理由均为 `AGENTS.md` 的触发条件不满足，逐条写在 `raw/run_log.md` 末尾。

**基线**：未重录 `tests/data/**`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`；
未新增 skip/xfail。

**未改的禁止面**：`vehicle/service.py`、`vehicle/roll_centers.py`、
`tests/physics/test_vehicle_physics.py`、`report/wheel_loads.py`、`report/metrics/vehicle.py`、
`outputs/builtin.py`、`packages/suspension_kernel/**`、`EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`。
