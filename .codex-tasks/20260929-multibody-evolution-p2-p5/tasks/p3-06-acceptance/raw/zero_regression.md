# p3-06 判据 3：零回归

## 快速集与两张附加测试集

```
$ uv run --no-sync pytest packages/suspension_multibody/tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q -p no:cacheprovider
1188 passed, 1 xfailed in 40.01s
exit=0

$ uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests -q -p no:cacheprovider
73 passed in 15.60s
exit=0
```

`xfailed = 1`，**与基线一致**（`tasks/p3-01-freeze/raw/` 记的起点也是 1 xfailed）；
**无新增 skip/xfail**。

## `tests/architecture` 全目录

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
147 passed in 633.84s (0:10:33)
exit=0
```

**本行第一次跑时这里失败了 2 条**，原因是 p2-07 新增的
`tests/cases/test_rotational_torque_document.py` 直接 `from suspension_multibody.kernel
import run_contract` 并调用它，触发了 `test_public_api_boundary_gate.py` 的
`direct_kernel_run_contract` 规则（该门是 `mode = "strict"`、allowlist 零条目）。
本行作为验收行**不就地改生产代码**，但该文件的作者行已把提交改走
`simulation.run_request(compile_document_pair(...))`——即经过唯一提交点
`simulation/backend.py`，与仓库既有的 KC/dynamic 契约测试同一路径。
**改后重跑 147 passed（上式即改后结果）**。这是本行独立验收真正抓到的一个缺陷。

## 数值门三项

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
  handling           PASS      4 open-loop shapes match an independent expansion (0.0e+00); closed-loop refused
  ride_four_post     PASS
  ride_random_road   PASS
  comparison         N/A
OK: 8 families accepted
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
  k-100: 0.8122 s vs baseline 1.0230 s (x0.794)
  c-66: 1.1550 s vs baseline 1.4904 s (x0.775)
OK: benchmarks are within the recorded budget
exit=0
```

**注意 `case_parity_check.py` 不带参数运行**（脚本没有 `--check`）。

## 冻结基线未动 + 三条架构门

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空）

$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
findings  : 0 / OK: no unregistered Python boundary violation        exit=0

$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
module cycles (SCC size>1) : 0 / OK: layering matches the recorded baseline   exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
OK: 3 release checks passed                                          exit=0
```
