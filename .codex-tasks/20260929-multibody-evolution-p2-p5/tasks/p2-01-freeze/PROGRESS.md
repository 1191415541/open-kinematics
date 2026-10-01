# PROGRESS：p2-01 冻结现状事实与判据（阶段二）

## 状态

`DONE`（2026-10-01）。五条判据全部实测，证据落 `raw/`。

## 交付物

| 判据 | 文件 | 内容 |
|---|---|---|
| (a) 力矩路径快照 | `raw/torque_path_snapshot.json` + `raw/torque_path_probe.py` | `_build_wheel_torque_signals` 的输入（时间栅格 3 样本、长度尺度、driveline 参数）与输出（每轮样本数与首样本值）；契约表来源对象 `PreparedVehicleRun.wheel_torque/.brake_torque` 的 4 行表与每轮样本数 |
| (b) 后轮转向限制负例 | `raw/rear_steer_refusal.md` + `raw/rear_steer_probe.py` | 两层的原文：准备层 `ValueError`（含触发帧行号）、authoring 层是**静默覆写**而非拒绝 |
| (c) 门禁与数值门起点 | `raw/gates_baseline.md` | 三条架构门 + 数值门三项的命令、退出码、关键输出；`kc_parity_check` 使用口径；内核重建后必须刷新 mirror |
| (d) F1–F5 锚点复核 | `raw/anchors_F1_F5.md` | 逐条 `file:line` + 原文 + 是否过期 |
| (5) 起点状态 | `raw/baseline_notes.md` | skip/xfail 起点计数、既有失败（无）、未改 `packages/**` 与未重录基线的自证 |

## 实测要点（供后续行使用）

- **ABI 现值 16 / 31 / 1**（`mb_config/version.hpp:27/34/37` 与 `kernel/native.py:33-35`），路线图写的「15 / 30」已过期。
- **EPIC F4 锚点过期**：`preparation/vehicle_dynamic.py:205-211` 现为 `_select_assembly_mode`；后轮转向限制在 `_validate_steering_topology`（定义 `:216-234`、调用 `:252`），原文为 `native vehicle dynamics actuates the rack of 'front' only; rack_fixed_to_chassis must be true on rear`。第二层 `authoring/vehicle.py:119` 是**静默覆写**（实测：文档声明 rear 条目被写成 bolted，无异常）。p2-06 必须两层都放开。
- **`case_parity_check.py` 没有 `--check`**：实测退出 2 并打印 `unrecognized arguments: --check`。父表与部分 SPEC 里写成 `--check` 的验收命令一律按**无参数**运行。
- **内核重建后必须跑 `scripts/build_axle_native.py`**：重建内核后 `check_composable_release --skip-isolation` 因 mirror 过期退出 1，跑该脚本后恢复 0。p2-02（必然重建）与 p5-04（可能重建）必须遵守。
- **既有失败：无**。fast set 基线 1078 passed / 1 xfailed；Epic 级基线 1531 passed / 1 skipped / 1 xfailed。

## 未做（本行边界）

未改任何 `packages/**` 文件（`git status` 自证见 `raw/baseline_notes.md`）；未重录任何基线；未改 `EPIC.md`/`SUBTASKS.csv`。
