# p3-05 运行日志（只记已执行）

仓库根：`/e/杂件/open-kinematics`（Windows: `E:\杂件\open-kinematics`）。全部命令在仓库根、`uv run --no-sync` 下执行。
时间：2026-10-01，11:00–11:2x（本地）。

| # | 命令 | 退出码 | 证据 |
|---|---|---|---|
| B1 | `uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q -p no:cacheprovider`（**改造前**基线） | 0 | `61 passed in 3.37s` |
| C1 | `uv run --no-sync python .../raw/channel_probe.py --emit .../raw/four_wheel_before.json` | 0 | `raw/four_wheel_before.json` |
| C2 | `uv run --no-sync python .../raw/channel_probe.py --emit .../raw/four_wheel_after.json` | 0 | `raw/four_wheel_after.json`、`raw/channel_probe_emit_after.txt` |
| C3 | `uv run --no-sync python .../raw/channel_probe.py --compare .../raw/four_wheel_before.json .../raw/four_wheel_after.json` | 0 | `raw/channel_probe_compare.txt`：`moved channels: 0` |
| C4 | `uv run --no-sync python -` （内联脚本，转储 3 轴 / 单轮 / 改名实验的通道与汇总） | 0 | `raw/three_axle_dump.txt` |
| C5 | `uv run --no-sync python -c "from suspension_multibody.outputs import builtin; print([...])"` | 0 | 13 个派生输出名，见 `four_wheel_compat.md` 或 `dynamic_channel_registration.md` |
| V1 | `uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q -p no:cacheprovider` | 0 | `75 passed in 3.49s` |
| V2 | `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | 0 | `mode: migration`、`findings: 0`、`OK: no unregistered Python boundary violation` |
| V3 | `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | 0 | `combined sha256 : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`；`OK: dynamic output matches the frozen baseline byte-for-byte` |
| V4 | `uv run --no-sync ruff check .` | 见下 | 收尾复跑时**失败**（退出码 1，`Found 2 errors`），但两条都在 `.codex-tasks/.../p4-03-arb-ports/raw/ports_probe.py`（另一会话的临时探针，mtime 11:17:37，F401 + E402），**与本行无关**；本行文件单跑 `ruff check <本行 5 个文件 + raw/channel_probe.py>` 退出码 0（`All checks passed!`）。同一命令在本行改动落地时（11:12）曾退出码 0 |
| V5 | `uv run --no-sync ty check .` | 0 | `All checks passed!`（全仓） |
| V6 | `uv run --no-sync pytest packages/suspension_multibody/tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q -p no:cacheprovider` | 0 | 收尾复跑 `1188 passed, 1 xfailed in 39.82s`（比 11:12 的 1182 多 6，是并发写入者新增的用例）；`1 xfailed` 为既有 |
| V7 | `uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests -q -p no:cacheprovider` | 0 | `73 passed in 16.44s` |
| V8 | `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | 0 | `OK: layering matches the recorded baseline` |
| V9 | `uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation` | 0 | `OK: 3 release checks passed` |

## 关于 V3 输出的两点说明

1. `acceptance exit : 1` 与 `failed cases : [...]` 是**冻结基线自身**的内容（sentinel 把 acceptance 报告一起哈希），不是本次改动引入的；`--check` 的退出码是 0，sha256 与冻结值一致。
2. `metrics` 不在 `LEGACY_MANIFEST_KEYS` 里，因此报表通道的新增不影响该哈希；实测 sha 未变，即「未重录基线」。

## 未执行（说明）

- `just gate-numeric` 的 `case_parity_check.py`（162 s）与 `kc_perf_gate.py`：本行不触及求解路径、不改 `cases/`、不改轮胎/力元，也未重录任何基线；数值门的证据按任务口径由 V3 的逐位哈希承担。
- `tests/architecture`（149 用例，约 10 分钟）：按 `AGENTS.md` 第 1 节，只在结构大重构收尾时跑；本行改为 `just check-fast` 的等价集合（V4/V5/V8/V9 + V6），三条秒级架构门（V2/V8/V9）已单独跑并退出码 0。
- `tests/adams`、`tests/cases`：不在本行改动范围（不改轮胎力律、不改 `cases/`）。

## 一次环境干扰（记录，非本行缺陷）

11:11 首次跑 V1 时 4 个用例失败，报 `module 'suspension_multibody.subsystems.suspension' has no attribute 'global_elements'`。核对文件时间戳：`subsystems/suspension.py` 的 mtime 与我的运行时间相差数秒，`def global_elements` 在同一分钟内出现/消失——即**另一会话正在并发改该文件**（`AGENTS.md` 第 7 节提到的同一风险）。等 45 s 后原命令重跑：`70 passed`（当时新增测试已落地），随后补完测试为 `75 passed`。上述失败与 p3-05 的改动无关。
