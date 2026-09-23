# 01 冻结命令集实测记录

任务：20260922-01-baseline
实测时间：本轮（2026-09-22 会话）
环境：Windows / git-bash，`uv 0.11.11`，`Python 3.13.5`
仓库：`C:\杂件\open-kinematics`，HEAD = `c684591`

## 14 条命令逐条实测退出码

| # | 命令 | 退出码 | 结果摘要 |
|---|---|---|---|
| 1 | `uv run python packages/suspension_multibody/scripts/build_axle_native.py` | **0** | 产出 `packages/suspension_multibody/src/suspension_multibody/native/suspension_kernel.dll` |
| 2 | `uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | **0** | `OK: layering matches the recorded baseline`；legacy 0 / 反向边 0 / 自包含头 0 / 跨聚合 0 / 环 0 |
| 3 | `uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q` | **0** | `15 passed in 0.34s` |
| 4 | `uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q` | **0** | `22 passed in 0.15s` |
| 5 | `uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` | **0** | **`737 passed, 47 skipped, 1 xfailed in 461.58s`** |
| 6 | `uv run --all-packages ruff check .` | **0** | `All checks passed!` |
| 7 | `uv run --all-packages ty check .` | **0** | `All checks passed!`（ty 自身打印 pre-release 警告，非失败） |
| 8 | `uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | **0** | `OK: dynamic output matches the frozen baseline byte-for-byte`；26 artifact；`combined sha256 = e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e` |
| 9 | `uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check` | **0** | `OK: candidate matches the frozen K/C snapshot within tolerance` |
| 10 | `uv run python packages/suspension_multibody/scripts/case_parity_check.py` | **0** | `OK: 8 families accepted` |
| 11 | `uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | **0** | `OK: no unregistered Python boundary violation`；findings 8（均为已注册的 `legacy_module_import`） |
| 12 | `uv build --package suspension-kernel` | **0** | 产出 `suspension_kernel-0.1.0.tar.gz` + `.whl` |
| 13 | `uv build --package suspension-multibody` | **0** | 产出 `suspension_multibody-0.1.0.tar.gz` + `.whl` |
| 14 | `git diff --check` | **0** | 无空白错误（仅 CRLF 提示，非错误） |

**全部 14 条退出码为 0，无阻断项。**

## 命令 5 与命令 8 的细节

### 命令 5：全量套件计数

```
737 passed, 47 skipped, 1 xfailed in 461.58s (0:07:41)
```

**与计划记录的差异**：`EPIC.md` 与父 `PROGRESS.md` 记的是「783 passed / 1 skipped / 1 xfailed」（制定计划时实测），本次为 **737 passed / 47 skipped**。

- 差异定性：**环境差异**，非回归。skip 数从 1 升到 47、passed 从 783 降到 737，合计 784 = 784，说明是同一批用例在本次环境下被 skip 而非失败（本机缺少 Adams 参考工件等可选依赖）。
- 该差异已登记到 `baseline_values.md` 的「既有失败」节，作为后续「新增失败为零」的对照底线。

### 命令 8：acceptance exit code

```
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency',
                    'opposite_phase_road', 'road_pulse', 'road_sine',
                    'road_step_finite_rise', 'single_wheel_road',
                    'tire_liftoff_and_recontact']
```

**这是基线已记录的既有状态，不是本次新增失败**：`dynamic_hash_baseline.json` 自身记的 `acceptance_exit_code` 就是 `1`，而门禁脚本比的是 26 个 artifact 的字节哈希（`OK: dynamic output matches the frozen baseline byte-for-byte`），退出码 0。9 个 FAILED 是 Adams 对标类用例的既有 BLOCKED 状态。

## 无法执行的命令

无。14 条全部实际执行并记录退出码。

## 证据文件

- 本文件：命令原文 + 退出码 + 摘要
- `joint_inventory.md`：8 种副在三处的落点与截断点
- `tire_mass_inventory.md`：质量现状路径
- `baseline_values.md`：各基线文件当前值与哈希
