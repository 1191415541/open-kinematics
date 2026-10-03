# p5-06 证据索引

> **终态提示（2026-10-03 收口）**：Epic **28/28 DONE，已关闭**。下方 `.final.txt` 后缀的文件是
> 收口后重跑的原文；不带后缀的是收口过程中的历史读数，保留为过程痕迹，**不代表终态**。
> 主验收记录请直接读 `acceptance.md` 第 6 节「终态结论」。

| 文件 | 内容 | 命令 | 退出码 |
|---|---|---|---|
| `acceptance.md` | **主验收记录**：Done-When (a)–(j)、缺陷 A/B/C；**第 6 节为终态结论** | — | — |
| `done_when_a_to_j.md` | (a)–(j) 逐条命令、退出码、依据行号（终态） | — | — |
| `p504_convergence.md` | p5-04 ABS 固定目标收敛的达成过程与独立复核（§10） | 见文内 | 0 |
| `full_regression.txt` | 全量原文（末尾追加 2026-10-03 收口后重跑：**1755 passed, 1 xfailed**） | `pytest packages/suspension_multibody/tests -q` | 0 |
| `numeric_sentinel.final.txt` | sentinel 26 artifact / combined sha256（终态） | `dynamic_hash_sentinel.py --check` | 0 |
| `numeric_case_parity.final.txt` | 8 families PASS（终态） | `case_parity_check.py`（无参数） | 0 |
| `gate_kc_perf.final.txt` | 性能预算内（终态） | `kc_perf_gate.py --check` | 0 |
| `gate_legacy_surface.final.txt` | findings 0（终态） | `legacy_surface_gate.py --check` | 0 |
| `gate_module_layering.final.txt` | 0 环（终态） | `check_module_layering.py --strict --final` | 0 |
| `gate_composable_release.final.txt` | 3 PASS（终态） | `check_composable_release.py --skip-isolation` | 0 |
| `evidence_signal_bus.final.txt` | **16 passed**（终态） | `pytest tests/api/test_signal_bus.py -q` | 0 |
| `evidence_abs_closed_loop.final.txt` | **7 passed**（终态） | `pytest tests/cases/test_abs_closed_loop.py -q` | 0 |
| `evidence_fmu_export.final.txt` | **17 passed**（终态） | `pytest tests/api/test_fmu_export.py -q` | 0 |
| `evidence_bus_fmu.final.txt` | **33 passed**（终态） | 总线 + FMU 合并 | 0 |
| `skip_register.txt` | **历史**：`artifacts/` 缺失期间的 47 个 skip（已被取代） | 同上 `-rs` | 0 |
| `numeric_sentinel.txt` | sentinel 26 artifact / combined sha256 | `dynamic_hash_sentinel.py --check` | 0 |
| `numeric_case_parity.txt` | 8 families accepted | `case_parity_check.py`（无参数） | 0 |
| `gate_kc_perf.txt` | 性能预算内 | `kc_perf_gate.py --check` | 0 |
| `gate_legacy_surface.txt` | findings 0 | `legacy_surface_gate.py --check` | 0 |
| `gate_module_layering.txt` | 0 环 | `check_module_layering.py --strict --final` | 0 |
| `gate_composable_release.txt` | 3 PASS | `check_composable_release.py --skip-isolation` | 0 |
| `architecture.txt` | 147 passed | `pytest tests/architecture -q` | 0 |
| `contracts_kernel.txt` | 79 passed | `pytest packages/suspension_contracts/tests packages/suspension_kernel/tests -q` | 0 |
| `evidence_simulate.txt` | 18 passed | `pytest tests/api/test_simulate.py -q` | 0 |
| `evidence_signal_bus.txt` | 13 passed | `pytest tests/api/test_signal_bus.py -q` | 0 |
| `evidence_abs_closed_loop.txt` | 6 passed | `pytest tests/cases/test_abs_closed_loop.py -q` | 0 |
| `evidence_fmu_export.txt` | 16 passed | `pytest tests/api/test_fmu_export.py -q` | 0 |
| `dw_b_steering.txt` | 46 passed | 转向通道三文件合并 | 0 |
| `dw_c_d_physics.txt` | 37 passed | `tests/physics` + `test_screw_kinematics.py` | 0 |
| `dw_a_e_torque_arb.txt` | 30 passed | 力矩元 + ARB 四文件 | 0 |
| `done_when_a_to_j.md` | (a)–(j) 逐条命令、退出码、依据行号 | — | — |

## 独立复验（不看子任务自报）

| 项 | 命令 | 结果 |
|---|---|---|
| (d) 动态通道子项 | `pytest tests/metrics/test_placement_channels.py -q` | **7 passed**；断言加速度下 front 降 / rear 升 / `load_transfer_front_minus_rear < 0` |
| (h) 总线写改变轨迹 | `pytest tests/api/test_signal_bus.py -q` | **16 passed**；含 `test_a_damping_write_changes_the_solved_trajectory`、`test_a_motor_torque_write_changes_the_solved_trajectory`（断言 `delta > _TRAJECTORY_TOLERANCE`） |
| (h) FMU 时间因果性 | `pytest tests/api/test_fmu_export.py -q` | **17 passed**；含 `test_the_clock_is_causal_a_later_input_does_not_rewrite_the_past` |
| (h) ABS 收敛 | `raw/p504_convergence.md` §10 | 独立复核（`explorer 8d135895`）实跑复算：ON `mean(e)=0.017964`；ratio `0.084274`，与主代理读数吻合 |
| (f) 全仓只有一个 `role = "wheel"` 子系统文件 | `grep -rln '^role = "wheel"' src/` | 唯一命中 `subsystems/wheel.py` |
| (f) 该文件内容指纹 | `sha256sum src/suspension_multibody/subsystems/wheel.py` | `6313b9c8898f8d4b2ec0b35539d45f42c2c3c00bfdb6ddd3b9d01ebda3091729` |
| (f) `VerticalTireElement` 在装配路径无命中 | `grep -c VerticalTireElement src/.../assembler.py` | `0` |
| (f) `assembler.py` 无 isinstance 过滤 | `grep -c isinstance src/.../assembler.py` | `0` |
| (e) `upright_L`/`upright_R` 在 ARB 路径无命中 | `grep -rn 'upright_L\|upright_R' src/.../anti_roll_bar.py` | `0` 命中（grep 退出 1） |
| (a) `front_brake_bias` 只在注释/退役说明中出现 | `grep -rn front_brake_bias src/.../subsystems/*.py` | 无读取，仅 `brake.py:52`、`torque_elements.py:423` 的说明文字 |
| (j) ABI 常量 | `grep -n kAxleKernelAbiVersion src/.../mb_config/version.hpp` | `= 17`；vehicle `= 32`、core `= 1` |
| (i) `tests/data` 未写 | `git status --short -- packages/suspension_multibody/tests/data/` | 空 |

## 缺陷 A/B 的独立复现脚本（会话 scratch，不随仓库提交）

| 脚本 | 用途 | 结果 |
|---|---|---|
| `$PI_SCRATCH_DIR/p506/probe_scaling.py` | 复现缺陷 B：旋转输出被误缩放 | 偏小 5.7296 倍 |
| `$PI_SCRATCH_DIR/p506/verify_point3.py` | 验证缺陷 B 修复 | 逐样本等于信号，最大差 1.03e-18 |
| `$PI_SCRATCH_DIR/p506/verify_fix_shape.py` | 确认缺陷 A 的修法形状（暂存 + copy） | 失败态下 staged 成功、逐字节相等 |
