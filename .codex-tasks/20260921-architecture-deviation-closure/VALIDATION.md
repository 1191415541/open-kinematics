# 架构偏差收敛基线验证

本文件是 `20260921-architecture-deviation-closure` 的冻结验证真源。所有命令均须实际执行后写入；未执行项不得记为通过。当前记录来自子任务 01 的基线实测，后续步骤继续补充本文件。

## 1. 工具链与构建配置

执行时间：2026-09-21T10:02:29+08:00

命令：

```text
uv run python packages/suspension_multibody/scripts/build_axle_native.py
```

结果：退出码 `0`；产物：`packages/suspension_multibody/src/suspension_multibody/native/suspension_kernel.dll`。

归档输出：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/build_axle_native.log`。

工具链实测：

| 项目 | 冻结值 | 证据 |
|---|---|---|
| uv | `uv 0.11.11 (ed7b06001 2026-05-06 x86_64-pc-windows-msvc)` | `uv --version` |
| Python | `Python 3.13.5` | `python --version` |
| CMake | `cmake version 4.4.3` | `cmake --version` |
| Ninja | `1.13.2.git.kitware.jobserver-pipe-1` | `ninja --version` |
| 编译器 | `C:\\msys64\\ucrt64\\bin\\x86_64-w64-mingw32-g++.EXE` | `native_build.json` |
| 编译器版本 | `x86_64-w64-mingw32-g++.EXE (Rev3, Built by MSYS2 project) 16.2.0` | `native_build.json` 与 `g++.EXE --version` |
| 配置 | `Release` | `native_build.json` |
| ABI | axle `15`；core `1`；vehicle `30` | `native_build.json` |
| 平台/架构 | `Windows-11-10.0.26200-SP0` / `AMD64` | `native_build.json` |

冻结编译 flags（来自构建后生成的 `native_build.json`）：

```text
-O3 -DNDEBUG -std=c++17 -flto=auto -fno-fat-lto-objects -Wall -Wextra -Werror -fno-fast-math -fopenmp
```

构建配置由 `build_axle_native.py` 的 `--configuration {Release,Debug}` 控制，本次使用默认 `Release`。CMake 对 `Release` 开启 IPO/LTO；OpenMP 使用静态 `libgomp.a` 链接，避免运行时依赖漂移。

## 2. 基线运行时线程、后端与计算顺序

基线 shell 中 `SUSPENSION_*`、`OMP_*`、`MKL_*` 环境变量均未设置。默认运行时策略冻结如下：

- OpenMP 已编译启用；解析 Jacobian 的 analytic team 为 `max(1, min(4, dim/4))`，可由 `SUSPENSION_AXLE_ANALYTIC_THREADS` 覆盖。
- 有限差分 Jacobian 使用 `max(1, min(8, column_count/4))`，OpenMP 循环为 `schedule(static)`；无 reduction，线程数不改变列结果顺序。
- 线性求解线程数为 `max(1, min(4, omp_get_max_threads()))`，可由 `SUSPENSION_AXLE_LINEAR_THREADS` 覆盖；PARDISO 默认线程上限为 `min(4, available)`。
- 默认线性后端为 dense `LuFactorization`。`blocked_lu`、行列平衡和 exact Fiala relaxation 默认开启；sparse GMRES、sparse LU、MKL dense/PARDISO 和 acceleration Schur probe 默认关闭。后端选择顺序为 sparse GMRES、Windows PARDISO、MKL dense、sparse LU，最终回退 dense LU。
- Jacobian/列运算和相关并行循环使用固定输入顺序与 `schedule(static)`；本次冻结不改变编译 flags、环境覆盖、后端、线程策略或浮点计算顺序。

源码证据：

- `packages/suspension_kernel/cpp/src/base/kernel_base.cpp:67-78,81-132,162-186`
- `packages/suspension_kernel/cpp/src/integrator/kernel_integrator_newton.cpp:144-150,217-258,970-1017`
- `packages/suspension_kernel/cpp/include/mb_linalg/factorization_types.hpp:1832-1874`
- `packages/suspension_kernel/CMakeLists.txt:149-160,259-295,298-310`

## 3. 基线步骤状态

步骤 1-8 的实测已完成；步骤 6 的动态哈希兼容性已修复并通过，子任务 01 已闭合，02-09 可按依赖启动。

- ABI 七符号与调用签名：已实测，详见第 4 节；预设 `pefile` 命令因依赖缺失退出 1，替代 `objdump`/ctypes 核验通过。
- 五个门禁脚本的 `--help` 及可执行参数：已实测，详见第 5 节。
- kernel/contracts/multibody、ruff、ty 基线：已实测通过，详见第 6 节及步骤 8 收尾日志。
- K/C parity：候选 native probe 与冻结快照均在容差内，退出码 0，详见第 9 节。
- family parity：八个 family 逐项接受，退出码 0；`comparison` 按设计为 N/A，详见第 9 节。
- native K/C 性能门：退出码 0，详见第 9 节。
- 动态哈希：兼容写出修复后连续两次哨兵退出码均为 0，26/26 artifact 与冻结组合哈希一致；未重录基线。详见第 9 节。
- 报告实体映射、native 输出缺口和凝聚身份映射：已冻结，详见本文件“报告契约缺口”表；缺口仍阻断 05-08 的实际迁移。
- 真实 Adams 执行需要现有安装和许可证；在实测环境不可用前阻断整车数值等价声明。

步骤 6 的动态哈希门已闭合，但报告契约缺口和真实 Adams 限制仍按后续任务边界保留；不得将这两项声明为已消除。
## 4. ABI 七符号与版本

目标 DLL：`packages/suspension_kernel/src/suspension_kernel/native/suspension_kernel.dll`。

任务表预设命令实际执行结果：退出码 `1`，原因是当前 uv 环境没有 `pefile`：`ModuleNotFoundError: No module named 'pefile'`。该失败已保留，未把它伪装成通过。

替代 PE 导出核验命令（系统已有 MinGW `objdump`）：

```text
objdump -p packages/suspension_kernel/src/suspension_kernel/native/suspension_kernel.dll
```

实测 Export Address Table 和 Name Pointer Table 均为 `00000007`，导出集合按名称排序为：

```text
axle_kernel_abi_version
mb_core_abi_version
mb_core_run
suspension_kernel_capabilities
suspension_kernel_contract_version
suspension_kernel_run
vehicle_kernel_abi_version
```

ctypes 运行时核验命令：

```text
uv run python -c 'import ctypes,sys; p=sys.argv[1]; lib=ctypes.CDLL(p); names=("suspension_kernel_run","suspension_kernel_capabilities","suspension_kernel_contract_version","axle_kernel_abi_version","vehicle_kernel_abi_version","mb_core_abi_version","mb_core_run"); print("symbols:"); print("\\n".join([n+": "+str(hasattr(lib,n)) for n in names])); print("versions:"); [(setattr(getattr(lib,n),"argtypes",[]), setattr(getattr(lib,n),"restype",ctypes.c_int32), print(n+"="+str(getattr(lib,n)()))) for n in ("suspension_kernel_contract_version","axle_kernel_abi_version","vehicle_kernel_abi_version","mb_core_abi_version")]; print("suspension_kernel_free="+str(hasattr(lib,"suspension_kernel_free")))' packages/suspension_kernel/src/suspension_kernel/native/suspension_kernel.dll
```

结果：退出码 `0`；七个符号全部为 `True`；`suspension_kernel_contract_version=1`、`axle_kernel_abi_version=15`、`vehicle_kernel_abi_version=30`、`mb_core_abi_version=1`；`suspension_kernel_free=False`。本次不新增 `suspension_kernel_free`。

真实 C ABI 签名：

| 符号 | 签名事实 | 声明/实现证据 |
|---|---|---|
| `suspension_kernel_run` | `int32_t(const uint8_t* model_payload, size_t model_length, const uint8_t* case_payload, size_t case_length, uint8_t* result_out, size_t* result_length_in_out, char* error_buffer, size_t error_capacity)` | `cpp/axle_dynamics/axle_kernel.hpp:43-51`；`cpp/src/abi/kernel_contract_run.cpp:242-247` |
| `suspension_kernel_capabilities` | `int32_t(char* buffer, size_t capacity, size_t* written)` | `cpp/axle_dynamics/axle_kernel.hpp:40-41`；`cpp/src/abi/kernel_contract_run.cpp:226-227` |
| `suspension_kernel_contract_version` | `int32_t()`，实测返回 `1` | `cpp/axle_dynamics/axle_kernel.hpp:31`；`cpp/src/abi/kernel_contract_run.cpp:179-180` |
| `axle_kernel_abi_version` | `int()`，实测返回 `15` | `cpp/axle_dynamics/axle_kernel.hpp:19`；`cpp/src/abi/kernel_abi.cpp:857-859` |
| `vehicle_kernel_abi_version` | `int()`，实测返回 `30` | `cpp/axle_dynamics/axle_kernel.hpp:21`；`cpp/src/abi/kernel_abi.cpp:871-874` |
| `mb_core_abi_version` | `int()`，实测返回 `1` | `cpp/axle_dynamics/core_abi.hpp:135`；`cpp/src/abi/kernel_core.cpp:209-211` |
| `mb_core_run` | `int(const MbCoreInput*, MbCoreOutput*, char* error_buffer, size_t error_capacity)` | `cpp/axle_dynamics/core_abi.hpp:136-141`；`cpp/src/abi/kernel_core.cpp:213-218` |

`suspension_kernel_run` 和 `suspension_kernel_capabilities` 的 ctypes 绑定分别设置为上述指针/长度参数和 `c_int32` 返回值，证据为 `packages/suspension_multibody/src/suspension_multibody/kernel/__init__.py:125-136`。版本常量由构建后 DLL 回读写入 `native_build.json`，不是 Python 手工副本。

## 5. 门禁脚本参数冻结

五个独立 `--help` 命令均退出码 `0`；原始输出归档于：

- `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/help_dynamic_hash.log`
- `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/help_kc_parity.log`
- `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/help_kc_perf.log`
- `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/help_case_parity.log`
- `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/help_layering.log`

逐项实测命令与真实参数：

```text
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --help
```

真实参数：`--record`、`--check`、`--output OUTPUT`、`--baseline BASELINE`、`--skip-run`。门禁脚本说明非零 acceptance 退出码可能来自已知 time_convergence 失败，不能据此放宽门。

```text
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --help
```

真实参数：`--check`、`--baseline BASELINE`、`--actual-dir ACTUAL_DIR`。候选目录需要 `k_states.json` 和 `c_states.json`。

```text
uv run python packages/suspension_multibody/scripts/kc_perf_gate.py --help
```

真实参数：`--record`/`--record-native`、`--check`/`--check-native`、`--baseline BASELINE`、`--repeats REPEATS`。

```text
uv run python packages/suspension_multibody/scripts/case_parity_check.py --help
```

真实参数：`--family {kc_quasi_static,axle_dynamic,vehicle_kc,vehicle_dynamic,handling,ride_four_post,ride_random_road,comparison}`、`--allow-partial`；省略 `--family` 时检查全部 family。

```text
uv run python packages/suspension_kernel/scripts/check_module_layering.py --help
```

真实参数：`--record-baseline`、`--check`、`--strict`、`--report`。

TODO 中的原始串联命令也已执行，退出码 `0`；串联输出归档于 `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/help_chain.log`。步骤 3 冻结的数值门命令只能使用本节实际列出的参数。

## 6. 测试与静态检查基线

以下命令均在未修改生产代码的基线上实际执行，原始输出归档于子任务 01 的 `raw/`：

| 命令 | 退出码 | 实测结果 |
|---|---:|---|
| `uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q` | `0` | `15 passed` |
| `uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q` | `0` | `22 passed` |
| `uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` | `0` | `668 passed, 47 skipped, 1 xfailed`，用 `-rsx` 补充原因报告也为 `0` |
| `uv run --all-packages ruff check .` | `0` | `All checks passed!` |
| `uv run --all-packages ty check .` | `0` | `All checks passed!`；仅有 ty 预发布软件警告 |

multibody 的 skip/xfail 原因均为基线既有状态，不是本任务引入：

- 1 项因 Adams 参考轮胎不可用；15 项因 strict Adams source artifacts 或 Fiala/PAC2002 source case 不可用。
- 其余 skip 为 Adams mode/reference tire/parking reference 工件不可用，包括 USE_MODE 3、4、13、23、24、25 的既有证据缺失。
- 1 项 xfail 是 `test_native_brake_opposes_the_instantaneous_wheel_spin` 的退化制动夹具：无悬架刚度且轮胎无载荷，求解器无可接受步长；测试说明要求 fixture redesign 或可细分 solver，不通过调容差伪装通过。

详细原因：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/pytest_multibody_reasons.log`。汇总：`raw/test-summary.log`；各命令完整日志为 `raw/pytest_kernel.log`、`raw/pytest_contracts.log`、`raw/pytest_multibody.log`、`raw/ruff_all.log`、`raw/ty_all.log`。

## 7. 仍未冻结项与阻断范围

以下项目尚未完成，不得标记为通过：

- 动态哈希门已通过兼容修复：统一 writer 保留当前 `metrics`、`failure_evidence`、`partial_evidence` 等字段；sentinel canonicalization 仅将动态验收 manifest 投影回冻结的旧语义字段集合，不修改真实 manifest 契约。兼容修复后 26/26 artifact 的数组 bytes 与 canonical manifest 均恢复冻结值。
- 报告契约缺口表已冻结于本文件末尾，但其 native 输出补齐、decoder 接线和 API 切换仍属于 05-07 的后续交付，不得提前声明消除。
- 真实 Adams 执行需要现有安装和许可证；在环境不可用前，阻断整车数值等价声明。

动态哈希阻断已解除；报告契约仍待 05-07 消除，真实 Adams 证据仍不可用。不得通过补写未执行证据解除后两项限制。 

## 8. 逐符号迁移矩阵

步骤 5 于 `2026-09-21T11:23:02+08:00` 完成。交付物：`.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/SYMBOL_MATRIX.csv`。

生成命令：

```text
uv run python "$PI_SCRATCH_DIR/generate_symbol_matrix.py"
```

结果：退出码 `0`；生成 379 行。归档后重新执行 CSV 校验，退出码 `0`：

```text
uv run python -c 'import csv,sys; p=sys.argv[1]; rows=list(csv.DictReader(open(p,encoding="utf-8"))); assert len(rows)==379; assert len({r["id"] for r in rows})==379; counts={s:sum(r["scope"]==s for r in rows) for s in sorted({r["scope"] for r in rows})}; assert counts=={"cpp-header-edge":86,"cpp-module":20,"cpp-source-edge":63,"python-symbol":210}; assert all(all(r[k].strip() for k in ("old_owner","new_owner","callers","disposition")) for r in rows); print("archived matrix validation passed",counts)' .codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/SYMBOL_MATRIX.csv
```

实测覆盖：20 个 C++ 模块、86 条检查器头文件依赖边、63 条 `.cpp` include 边、210 个 Python 顶层现役符号。每行的 `old_owner`、`new_owner`、`callers`、`disposition` 非空；关键 `external_force_vector`、`evaluate_generalized_forces`、`compute_static_wheel_loads`、指标调用者和反向 `.cpp` 边均有证据行。

边界结论：`packages/suspension_kernel/scripts/check_module_layering.py:70-73` 只读取头文件，不能证明 `.cpp` 实现依赖；矩阵因此单独登记 `.cpp` source-edge。`mb_static -> mb_vehicle` 的 `external_force_vector` 调用必须迁为 `mb_solve_static -> mb_force`；`mb_vehicle -> mb_static` 在 `kernel_registration.cpp:18` 当前无调用命中，迁移前必须裁定为死 include 或真实依赖。`mb_energy` 的最终目标落点未在 Epic 目标中明确，矩阵保留为中性数据并标记待审，不据此删除。

步骤 5 的交付不解除结构迁移阻断；步骤 6 动态哈希门已通过，步骤 7 的报告契约缺口已冻结但待 05-07 消除。
## 9. 步骤 6 数值门实测

步骤 6 实测时间：`2026-09-21`；原始输出归档于 `.codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-01-baseline/raw/`。本节记录真实执行结果，不把非零退出码改写为通过。

### 9.1 动态数组逐位哈希

命令：

```text
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```

产物路径：`artifacts/axle-dynamics-acceptance/acceptance_report.json`；冻结基线：`packages/suspension_multibody/tests/data/dynamic_hash_baseline.json`；比较粒度为 `arrays.npz` bytes SHA-256 加兼容 canonical manifest SHA-256。统一 artifact 的真实 manifest 保留当前契约字段；sentinel 在比较时投影到冻结的旧动态 manifest 字段集合，并移除 `performance`、`native_build`。

首次和重复执行的哨兵命令均退出码 `0`；两次 acceptance 均为退出码 `1`，26 个 artifact 均生成；冻结组合哈希和修复后当前组合哈希均为 `e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`。当前失败 case 为 `combined_load`、`in_phase_road`、`large_amplitude_high_frequency`、`opposite_phase_road`、`road_pulse`、`road_sine`、`road_step_finite_rise`、`single_wheel_road`、`tire_liftoff_and_recontact`；acceptance 报告中的 `static_equilibrium`、`braking`、`driving`、`lateral_or_steering` 为 PASSED。

兼容修复的根因证据：旧 writer 的 NPZ 成员顺序为 `... anti_roll_output, diagnostics, tire_output, energy, ...`，统一 writer 曾写成 `... anti_roll_output, tire_output, energy, diagnostics, ...`；修复后恢复旧顺序。旧 manifest 的 `case_name`、`model_name`、`completed_sample_count`、`native_status` 等字段由当前统一 manifest 的 `case`、`model`、`time_grid` 和 `failure_evidence` 可无损投影，新增 `metrics`、`failure_evidence`、`partial_evidence` 等真实字段未删除。

结构化结果：arrays SHA-256 和兼容 canonical manifest SHA-256 均为 `26/26` 一致，status 与 completed sample 均为 `26/26` 一致；重复执行仍得到相同组合哈希。证据：`raw/dynamic_hash_check_compat_fix.log`、`raw/dynamic_hash_check_compat_fix_repeat.log`。

日志同时记录 `solver self-convergence: FAILED`、`adams accuracy: BLOCKED`、无真实 Adams evidence，以及未运行 acceptance 的 frozen median-of-N performance protocol。上述是独立的既有限制，不影响动态字节契约门；未重录 `dynamic_hash_baseline.json`。

### 9.2 K/C parity

候选产物由以下实际 probe 生成：

```text
uv run python packages/suspension_multibody/scripts/kc_native_probe.py
uv run python packages/suspension_multibody/scripts/kc_native_c_probe.py
```

两条命令退出码均为 `0`，输出目录为 `artifacts/kc-native-probe/`，分别生成 `k_states.json`（9 states）和 `c_states.json`（66 states）。probe 报告 K 最差误差/容差比 `1.65548e-05`，C 最差误差/容差比 `0.000186038`。证据：`raw/kc_native_probe.log`、`raw/kc_native_c_probe.log`。

候选 parity 命令：

```text
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe
```

退出码 `0`，输出 `OK: candidate matches the frozen K/C snapshot within tolerance`。冻结容差为平移字段 `0.1 + 0.002*abs(reference)` mm、转角字段 `0.02 + 0.005*abs(reference)` deg；C 形变比较使用脚本的平移 `1e-6 + 1e-4*abs(reference)` 和转动 `1e-8 + 1e-4*abs(reference)` 规则。TODO 原始命令也实际执行：

```text
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
```

退出码 `0`；该默认调用比较冻结目录自身，候选实现的有效证据以上述 `--actual-dir artifacts/kc-native-probe` 为准。证据：`raw/kc_parity_check.log`、`raw/kc_parity_check_default.log`。

### 9.3 八个 family parity

命令：

```text
uv run python packages/suspension_multibody/scripts/case_parity_check.py
```

退出码 `0`，未使用 `--allow-partial`。结果：`kc_quasi_static` PASS（最差误差/容差比 `0.000186038`）、`axle_dynamic` PASS（13 cases，bit-identical）、`vehicle_kc` PASS、`vehicle_dynamic` PASS（8 cases，bit-identical）、`handling` PASS、`ride_four_post` PASS、`ride_random_road` PASS、`comparison` N/A（按设计是 per-target gate，不是 solve）。脚本最终输出 `OK: 8 families accepted`。证据：`raw/case_parity_check.log`。

### 9.4 native K/C 性能门

命令：

```text
uv run python packages/suspension_multibody/scripts/kc_perf_gate.py --check
```

退出码 `0`；默认 repeats `5`，预算因子为 `1.25`，冻结基线文件为 `packages/suspension_multibody/tests/data/kc_perf_baseline_native.json`。实测 K-100 为 100 states，median `0.6697 s`，best `0.6480 s`；基线 best `0.7221 s`，倍率 `0.897`。实测 C-66 为 66 states，median `0.9085 s`，best `0.8889 s`；基线 best `0.9958 s`，倍率 `0.893`。输出为 `OK: benchmarks are within the recorded budget`。证据：`raw/kc_perf_check.log`。

### 9.5 步骤 6 结论

K/C parity、八 family parity 和 native 性能门均通过；动态数组逐位兼容门已通过，真实 Adams accuracy 仍不可执行。步骤 6 状态为 `DONE`；统一 artifact 字段、历史读取和失败证据契约保留；子任务 02-09 不再受动态哈希阻断。
## 报告契约缺口

步骤 7 的报告实体映射基于 native 输出块、decoder 和当前 Python API 的源码实测。native 的 element block 由 `kernel_contract_run.cpp:788-796` 声明，块行按调用方 Python model 的位置切片（`axle_dynamics/contract_run.py:238-254`）；C++ `Spring`、`Bushing`、`AntiRollBar` 结构只有 body/point/frame 等数据，没有 name 字段（`cpp/include/mb_model/types.hpp:57-99`）。当前用户可见报告仍由 `api.py:737-777` 调用 Python `evaluate_generalized_forces` 构造，调用者为 `api.py:453,559`；不能把当前报告路径当作 native 事实接管已经完成。

| 通道/实体 | 已证实现状与证据 | 缺口 | 归属子任务 | 阻断结论 |
|---|---|---|---|---|
| `spring_output`、`bushing_output`、`anti_roll_output`、`tire_output` 的名称/ID | native 输出列和 block 存在；名称列表由 Python model positional 映射提供（`contract_run.py:238-254`；`result.py:253-261`） | C++ 结构没有 name，输出没有独立 name/ID 通道；顺序变化会改变身份解释 | 05 冻结/补 native 身份，06 切 API | 阻断逐元件 native 对照和 API 切换；禁止用位置变化掩盖身份缺口 |
| 所有力元件的两端 `body_a/body_b` | 两端只存在于模型定义，如 `schema.py:604-605,670-671,794-795,864-865`；Python 元件也保存端点（`elastic.py:235-238,341-342,566-568,598-600`） | native element block 不输出两端 | 05 | 阻断两端载荷、凝聚后归属和 Adams 渲染的逐通道证明 |
| `spring_output` / `anti_roll_output` 坐标系 | spring 是 a→b 标量轴，anti-roll 是相对 a→b 轴（`spring.cpp:156-158`、`anti_roll.cpp:40-50`） | 输出不带坐标系声明；bushing 仅在实现中明确 body_a local（`bushing.cpp:63-66`），轮胎是接触/patch 语义 | 05 | 阻断跨通道方向和参考系等价声明 |
| 所有元件作用点 | model 文档保存 `pa/pb/center` 等点，但输出 block 没有作用点列；轮胎作用半径只在 native assembly 逻辑中使用（`kernel_tire_assembly.cpp:241-265`） | 缺少作用点/参考点通道 | 05 | 阻断力矩、反力作用位置和报告参考点核验 |
| 所有元件符号约定 | spring 的 b 端正力/a 端反力（`spring.cpp:156-158`）；bushing wrench、anti-roll torque 的符号在实现中定义（`bushing.cpp:55-77`、`anti_roll.cpp:46-50`） | 符号是隐含实现规则，不在 output descriptor 或结果字段中声明 | 05 冻结，07 文档化 | 阻断报告方向/作用反向的逐通道验收 |
| 所有数值字段单位 | native result map 已记录 SI 列含义（`axle_dynamics/result.py:39-124`）；model 声明 SI 与车辆坐标系（`schema.py:1070-1073`） | native descriptor 不携带单位；K/C 报告仍有 mm/N 缩放（`api.py:434`），存在 SI 与用户报告口径转换边界 | 05 冻结，06 接线，07 保持展示口径 | 阻断单位等价声明；禁止以数值接近代替单位核验 |
| 逐元件能量 | native `energy` 只有按类型聚合的 storage（`kernel_output.cpp:391-436`、`result.py:169-191`）；Python `ForceEvaluation.energy` 仍逐元件存在（`elements/base.py:10-19`、`elastic.py:283,326,488,554,582,634`） | 没有逐元件能量；当前 `ComponentLoad`/`BushingResult` 不承载该字段（`schema/result.py:68-85`） | 05 决定并补输出，06 切换，07 只读消费 | 阻断能量字段保持和报告事实切换；不得缺字段填零 |
| active 状态 | native `tire_output[0]` 有 active，另有 `active_contacts` diagnostics（`kernel_tire_assembly.cpp:36-80,184-188,383-391,429-457`；`result.py:68,206`） | spring/bushing/anti-roll/stop 无统一 active；stop 只能由 stop force 列间接推断（`spring.cpp:174-177`）；Python active 仍来自 `ForceEvaluation.active` | 05 | 阻断 active/事件状态等价，尤其 stop 与接触启停报告 |
| body ID → 凝聚体 ID | `VehicleAssembly.body_aliases` 在 `model/vehicle.py:52,296-303,517` 生成，并由 `preparation/vehicle_dynamic.py:338,405-411` 消费 | native 模型收到展平后的 body 名，manifest 只带 body names（`kernel_contract_run.cpp:813`），不输出 alias 身份映射 | 05 | 阻断凝聚后实体身份、native 输出和 Adams 渲染的一致性 |
| K/C family 元件事实覆盖 | K/C native contract 当前模型文档仅发送 bushing（`cases/kc_quasi_static/contract.py:118-120`）；native block map 对动态 family 有 spring/bushing/anti-roll/tire | K/C 中 spring/tire/anti-roll/gravity 仍依赖 Python 现役本构，不能声称 K/C 报告已全面 native | 05、06 | 阻断 K/C 元件事实切换及旧 `elements/elastic.py` 删除 |
| 用户报告构造路径 | `_collect_element_results` 从 `assembly.elements` 和 `evaluate_generalized_forces` 生成 `ComponentLoad`/`BushingResult`（`api.py:737-777`；`assembly.py:24`） | native result decoder 尚未成为唯一元件事实来源；energy、active、event 等字段在构造中被丢弃 | 06 | 阻断作者层切换和 08 删除旧本构/装配实现 |

步骤 7 的结论：缺口均有源码证据和归属；名称/端点/坐标/作用点/符号/单位/能量/active/凝聚身份不得靠报告层猜测或填零，需由 05 完成 native 输出与 decoder 对照后，06 切 API，07 只消费冻结结果。真实 Adams 执行仍受安装与许可证限制，缺口表不构成整车 Adams 数值等价声明。
