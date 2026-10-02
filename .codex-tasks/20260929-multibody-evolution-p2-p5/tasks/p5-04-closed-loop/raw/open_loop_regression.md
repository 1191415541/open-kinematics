# p5-04 证据 (d)：闭环不破坏开环（零回归对照）

> 实测日期 2026-10-02。父判据 `EPIC.md:281(d)`。

## 1. `dynamic_hash_sentinel.py --check`（逐字节硬门）

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency',
                    'opposite_phase_road', 'road_pulse', 'road_sine',
                    'road_step_finite_rise', 'single_wheel_road',
                    'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
```

combined sha256 与 p2-01 起点值**逐字节相同**（`fdfd5a6b…eebc9`）。
`acceptance exit 1` 与那 9 个 FAILED 是**基线自身记录的状态**（哨兵判定依据是 sha256 而非退出码），
与 p2-05 起点的读数一字不差。

## 2. `case_parity_check.py`（无参数，8 个族）

```
$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
  kc_quasi_static    PASS      worst error / tolerance 0.000185873 over the frozen K/C snapshot
  axle_dynamic       PASS      13 cases, bit-identical to the frozen snapshot
  vehicle_kc         PASS      grid matches an independent expansion; 10 mm reaches every wheel drive
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
  handling           PASS      4 open-loop shapes match an independent expansion (0.0e+00); closed-loop refused
  ride_four_post     PASS      expansion matches an independently sampled excitation (0.0e+00)
  ride_random_road   PASS      expansion matches an independently expanded profile (0.0e+00)
  comparison         N/A       a per-target gate, not a solve: the kernel never reads a reference
OK: 8 families accepted
```

**`handling` 族仍是「4 open-loop shapes … closed-loop refused」**：契约反转改的是**它解释这件事的方式**
（拒绝输入形状 ≠ 闭环不存在），**没有**把拒绝本身去掉。这正是不破坏开环的证据。

## 3. `kc_perf_gate.py --check`

```
$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
  k-100: 0.8198 s vs baseline 1.0230 s (x0.801)
  c-66: 1.1639 s vs baseline 1.4904 s (x0.781)

OK: benchmarks are within the recorded budget
```

## 4. 基线未重录

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空）
```

`kc_baseline/`、`dynamic_hash_baseline.json`、`vehicle_dynamics_baseline/sha256.json`、
`kc_perf_baseline_native.json` 均未被写。

## 5. 未改 ABI、未改唯一归属

```
$ grep -n "kAxleKernelAbiVersion\|kVehicleKernelAbiVersion\|kCoreKernelAbiVersion" \
    packages/suspension_kernel/cpp/include/mb_config/version.hpp
33:inline constexpr int kAxleKernelAbiVersion = 17;
43:inline constexpr int kVehicleKernelAbiVersion = 32;
46:inline constexpr int kCoreKernelAbiVersion = 1;

$ grep -n "_NATIVE_KERNEL_ABI_VERSION\|_NATIVE_VEHICLE_KERNEL_ABI_VERSION\|_NATIVE_CORE_ABI_VERSION" \
    packages/suspension_multibody/src/suspension_multibody/kernel/native.py
33:_NATIVE_KERNEL_ABI_VERSION = 17
34:_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 32
35:_NATIVE_CORE_ABI_VERSION = 1
```

两处真源同步且未动。`legacy_surface_gate.py --check` findings **0** ⇒
内核提交唯一归属仍只有 `simulation/backend.py`。

## 6. 新增测试

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases/test_abs_closed_loop.py -q
......                                                                   [100%]
6 passed in 1.06s
```

`tests/cases` 全目录、`tests/api` 全目录、`tests/architecture`（147 passed）、
`packages/suspension_kernel/tests`（47 passed）均通过；**无新增 skip/xfail**。
