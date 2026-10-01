# p4-05 第 5 步：快速集 + `tests/architecture` + 数值门三项

| 门 | 命令 | 退出码 | 计数原文 |
|---|---|---|---|
| 快速集 | `pytest packages/suspension_multibody/tests --ignore=…/adams --ignore=…/architecture --ignore=…/cases -q -p no:cacheprovider` | **0** | `1188 passed, 1 xfailed in 41.10s` |
| `tests/architecture` 整目录 | `pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider` | **0** | `147 passed in 633.84s (0:10:33)` |
| 数值门 1/3 | `dynamic_hash_sentinel.py --check` | **0** | `OK: dynamic output matches the frozen baseline byte-for-byte`；sha256 `fdfd5a6b…eebc9` |
| 数值门 2/3 | `case_parity_check.py`（**无参数**） | **0** | `OK: 8 families accepted`（`vehicle_dynamic` / `handling` / `ride_four_post` / `ride_random_road` 各 PASS） |
| 数值门 3/3 | `kc_perf_gate.py --check` | **0** | `OK: benchmarks are within the recorded budget`（k-100 ×0.794、c-66 ×0.775） |

三项数值门**缺一不算全绿**，本行三项都跑了。

`case_parity_check.py` **不带任何参数**：该脚本只有 `--family` / `--allow-partial` /
`--record`，**不存在 `--check`**。

**未做 K/C 对标**：本行**没有**运行不带 `--actual-dir` 的 `kc_parity_check.py`
（那是自比较、恒过，`EPIC.md:231` 禁止当证据）。若后续需要 K/C 对标，
必须先跑 `kc_native_probe.py` + `kc_native_c_probe.py` 再带
`--actual-dir artifacts/kc-native-probe`。

## 三条架构门（额外交付）

```
$ legacy_surface_gate.py --check                     → findings: 0 / OK   exit=0
$ check_module_layering.py --strict --final          → cycles: 0 / OK     exit=0
$ check_composable_release.py --skip-isolation       → 3 release checks passed  exit=0
```
