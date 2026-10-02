# p5-05 零回归：导出不影响既有运行路径

> 对应父判据 `EPIC.md` 行 279(c)：**导出不影响既有运行路径**。
> 本文件只记**已执行**的命令与其结论。

## 1. 为什么这条判据对本行重要

导出要写 `.fmu`、要读 `blobs` 与结果块、要用 `model_dump`/`canonical_hash` 算 GUID。
这些都在已有的数据结构上操作，所以「顺手改一下文档形状」「顺手加一个输出字段」
是这里最自然的失误。判据就是：**导出前后，既有路径的产物与结果逐字节相同**。

本行按「非侵入旁路」实现：`fmi/` 只 import 标准库与本包，
**没有任何既有模块 import 它**——`grep -rn "from ..*fmi\|import fmi\|fmi\." <src>`
在生产路径上零命中（见第 3 节）。因此既有路径不可能读到它。

## 2. 命令与结论

### 2.1 `tests/api` 全通过

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/api -q -p no:cacheprovider
........................................................................ [ 96%]
...                                                                      [100%]
75 passed in 5.03s
```

（其中 `test_fmu_export.py` 16 个是本行新增；其余 59 个是既有 API 用例，未改动、未删除。）

### 2.2 `dynamic_hash_sentinel.py --check` 逐字节未变

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency',
                    'opposite_phase_road', 'road_pulse', 'road_sine', 'road_step_finite_rise',
                    'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
```

`combined sha256` 与冻结值 `fdfd5a6b…eebc9` 一致，26 个 artifact 逐字节相同。

### 2.3 `case_parity_check.py`（无参数）8 families PASS

```
$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
  handling           PASS      4 open-loop shapes match an independent expansion (0.0e+00); closed-loop refused
  ride_four_post     PASS      expansion matches an independently sampled excitation (0.0e+00)
  ride_random_road   PASS      expansion matches an independently expanded profile (0.0e+00)
  comparison         N/A       a per-target gate, not a solve: the kernel never reads a reference
OK: 8 families accepted
```

### 2.4 `kc_perf_gate.py --check` 在预算内

```
$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
  k-100: median 0.8794 s, best 0.8440 s over 5 runs (100 states)
  c-66: median 1.2556 s, best 1.1812 s over 5 runs (66 states)
  k-100: 0.8440 s vs baseline 1.0230 s (x0.825)
  c-66: 1.1812 s vs baseline 1.4904 s (x0.793)

OK: benchmarks are within the recorded budget
```

### 2.5 基线未被重录

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空）
```

`tests/data/kc_baseline/`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`
均未被写入。

### 2.6 内核与契约包

```
$ uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests -q -p no:cacheprovider
79 passed in 14.90s
```

### 2.7 依赖未动（D6 的落地事实）

```
$ git diff --stat -- '**/pyproject.toml' pyproject.toml
（空）
$ git status --short -- uv.lock '**/uv.lock'
（空）
$ grep -rn "fmpy" pyproject.toml packages/*/pyproject.toml uv.lock
（空）
```

详见 `raw/dependency_decision.md`。

## 3. 导出是同归一处、且不是被既有路径引用的

- **提交点唯一**：导出经由 `simulation.compiler.compile_document_pair` 组装容器，
  与其它所有调用者走同一处；`fmi/` 本身不 import 内核，也不调 `run_contract`。
- **既有路径不 import 它**：`fmi` 是新包，没有任何既有模块引用。
- **`model_dump(mode="json")` 的形状未动**：本行不改 `schema/`，也不改 `api.py`；
  `model_hash` 基准 `72bd9f30330cce22fe672fc63c4248df09261928942443e2aedd98102b38c60b`（19 键）
  由 `tests/api` 的既有用例继续约束。
- **ABI 未动**：`mb_config/version.hpp` 仍为 `17/32/1`，`kernel/native.py` 同步未动；
  `test_kernel_abi_version_single_source` 在内核测试里（2.6 节 79 passed 覆盖）。
- **未新增 skip/xfail**：`tests/api` 的 75 passed 不含 skip；本行新增的 16 个用例
  全部实跑通过，无 xfail。
