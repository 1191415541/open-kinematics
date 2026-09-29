# 门禁与数值门实测值（01 第 4 行的证据）

- 实测时间：2026-09-29
- 执行者：主代理（本会话逐条实跑，非采信子任务自报）
- 用途：本 Epic 的起点事实与"零回归"的对照基准；04 的 `kc_baseline` 逐位不变判据以本文件为起点

## 日常门禁（`just check-fast` 的展开项）

| 命令 | 结果 |
|---|---|
| `uv run --no-sync ruff check .` | All checks passed |
| `uv run --no-sync ty check .` | All checks passed |
| `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | findings 0；`OK: no unregistered Python boundary violation` |
| `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | module cycles 0；`OK: layering matches the recorded baseline` |
| `uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation` | 3/3（migration list / documentation roots / documentation examples E-1,E-2,E-3） |
| `uv run --no-sync pytest packages/suspension_multibody/tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q` | **1015 passed, 1 xfailed**（44 s） |
| `uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests -q` | **60 passed** |

## 其余测试目录（快速集排除的三处，逐目录实跑）

| 命令 | 结果 |
|---|---|
| `uv run --no-sync pytest packages/suspension_multibody/tests/cases -q` | **100 passed** |
| `uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q` | **147 passed**（7 分钟） |
| `uv run --no-sync pytest packages/suspension_multibody/tests/adams -q` | **160 passed, 47 skipped**；47 个 skip 全部是"Adams 参考 artifacts 不在本副本"的环境跳过（`test_pacific2002_adams_correlation_gate.py:899` 等），非新增 |

## 数值门（`just gate-numeric`）

| 命令 | 结果 |
|---|---|
| `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | 26 artifacts 逐字节一致；`OK: dynamic output matches the frozen baseline byte-for-byte` |
| `uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py` | `OK: 8 families accepted`（kc_quasi_static / axle_dynamic / vehicle_kc / vehicle_dynamic / handling / ride_four_post / ride_random_road / comparison=N/A） |
| `uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check` | k-100 x1.008、c-66 x0.999（预算 x1.25）→ `OK: benchmarks are within the recorded budget` |

## 本轮（2026-09-29）经用户授权重录的两项基线

| 基线文件 | 重录前 | 重录后 | 依据 |
|---|---|---|---|
| `tests/data/vehicle_dynamics_baseline/sha256.json` | 8 用例按 23 刚体的旧模型 | 8 用例按 29 刚体（方式 A 的轮毂 + 转向外壳） | 用户授权；轴侧 13 用例逐字节未变（证明轴动态路径未被触碰） |
| `tests/data/kc_perf_baseline_native.json` | k-100 best 0.7221 s、c-66 best 0.9958 s | k-100 best 1.0230 s、c-66 best 1.4904 s | 用户授权；同机 HEAD 源码实测 x0.85/x0.82（预算内），当前 x1.44/x1.52（自由体 10→13 的稠密解成本） |

**本 Epic 不得再动 `kc_baseline/`**（D2）：单轴 K/C 的凝结必须让它逐位不变。
