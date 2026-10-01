# p2-06 证据 (f)：命令与退出码

> 全部命令在仓库根 `/e/杂件/open-kinematics` 执行（bash）。所有 pytest 均带 `-p no:cacheprovider`；未使用 `-k` / `--deselect`；未新增 skip/xfail；`packages/suspension_multibody/tests/data/` 全程未写。

## 1. 验收命令（规格第 3 条逐条）

| # | 命令 | 退出码 | 关键输出原文 |
|---|---|---|---|
| 1 | `uv run --no-sync pytest packages/suspension_multibody/tests/authoring packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/cases -q -p no:cacheprovider` | **0** | `385 passed in 58.37s` |
| 2 | `uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider` | **0** | `32 passed in 0.10s` |
| 3 | `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | **0** | `artifacts hashed : 26` / `combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` / `OK: dynamic output matches the frozen baseline byte-for-byte` |
| 4 | `uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py`（**无参数**） | **0** | 8 families 逐行 PASS，详见下文 |
| 5 | `uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check` | **0** | `k-100: 0.8245 s vs baseline 1.0230 s (x0.806)` / `c-66: 1.1726 s vs baseline 1.4904 s (x0.787)` / `OK: benchmarks are within the recorded budget` |
| 6 | `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | **0** | `cross-aggregate includes   : 0` / `module cycles (SCC size>1) : 0` / `OK: layering matches the recorded baseline` |
| 7 | `uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check` | **0** | `mode      : migration` / `findings  : 0` / `OK: no unregistered Python boundary violation` |
| 8 | `uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider` | **0** | `147 passed in 596.88s (0:09:56)` |
| 9 | `uv run --no-sync ruff check .` | **0** | `All checks passed!` |
| 10 | `uv run --no-sync ty check .` | **0** | `All checks passed!`（前置一行 ty 的 pre-release 提示） |
| 11 | `git status --short -- packages/suspension_multibody/tests/data/` | **0** | 输出为空（冻结基线未写） |

### 1.1 `case_parity_check.py`（无参数）原样输出

```
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

### 1.2 `dynamic_hash_sentinel.py --check` 原样输出（末段）

```
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency', 'opposite_phase_road', 'road_pulse', 'road_sine', 'road_step_finite_rise', 'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
```

口径说明：`acceptance exit : 1` 与 `failed cases` 那一段是**该脚本内部对上一个 artifact（单轴验收报告）的转述块**，不是本次检查的判定。本脚本自身的判定行是最后一行 `OK: dynamic output matches the frozen baseline byte-for-byte`，并**以退出码 0 结束**。26 个 artifact 逐字节一致，包含整车侧 8 例。

## 2. 本行新增测试

| 命令 | 退出码 | 输出 |
|---|---|---|
| `uv run --no-sync pytest packages/suspension_multibody/tests/preparation packages/suspension_multibody/tests/schema -q -p no:cacheprovider` | **0** | `89 passed in 2.34s` |
| `uv run --no-sync pytest packages/suspension_multibody/tests/preparation packages/suspension_multibody/tests/schema/test_vehicle_steering_channels.py -q -p no:cacheprovider` | **0** | `46 passed in 2.04s` |
| `uv run --no-sync pytest packages/suspension_multibody/tests/preparation/test_steering_allocator.py -q -p no:cacheprovider --collect-only` | **0** | `24 tests collected in 0.05s` |
| `uv run --no-sync pytest packages/suspension_multibody/tests/preparation/test_steering_channels.py -q -p no:cacheprovider --collect-only` | **0** | `8 tests collected in 0.53s` |
| `uv run --no-sync pytest packages/suspension_multibody/tests/schema/test_vehicle_steering_channels.py -q -p no:cacheprovider --collect-only` | **0** | `14 tests collected in 0.12s` |
| `uv run --no-sync pytest packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider` | **0** | `77 passed, 1 xfailed in 8.75s`（该 1 个 xfail 是既有用例，非本行新增） |
| `uv run --no-sync pytest packages/suspension_multibody/tests/authoring packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/cases packages/suspension_multibody/tests/preparation packages/suspension_multibody/tests/schema packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider` | **0** | `551 passed, 1 xfailed in 67.01s (0:01:07)` |

## 3. 逐条证据命令

| 命令 | 退出码 | 输出 |
|---|---|---|
| `grep -rn "must be true" packages/suspension_multibody/src --include=*.py` | **1** | 无输出（零命中）。准备层与文档导出层已无「必须为真」的校验 |
| `grep -rn "rack_fixed_to_chassis" packages/suspension_multibody/src --include=*.py` | **0** | 17 处命中：`schema/model.py:177`（字段定义）、`subsystems/assembler.py:191/206/228/229`（读成装配事实）、`subsystems/steering.py:21/165/199/239/276`（`WeldJoint`/`PrismaticJoint` 二选一，归 p4-02）、`subsystems/explicit.py:138/144`、`subsystems/types.py:239`（注释）、`templates/builtin.py:448`（注释，归 p4-02）、`adams/full_vehicle_model.py:1670/2811`、`preparation/vehicle_dynamic.py:251`（**本行新增的 docstring**，说明这次解除）。**没有一处是「要求它为真」** |
| `grep -n 'if element\["type"\] != "steering_actuator"' packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py` | **1** | 无输出（过滤表达式已不存在） |
| `grep -rniE 'steering_actuator\.(type|filter)' packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py` | **1** | 无输出 |
| `uv run --no-sync python <scratch>/alloc_evidence.py` | **0** | 四种分配律数值表（见 `allocator_assertions.md` §3） |
| `uv run --no-sync python <scratch>/final_two_channel.py` | **0** | 两通道装配 + study 实跑（见 `two_channel_assembly.md` §3） |
| `uv run --no-sync python <scratch>/dump_evidence.py` | **0** | 单通道 dump 的键集合 / 字节数 / canonical_hash 三项对照（见 `steering_channels.md` 与下文 §4） |

## 4. 单通道向后兼容三项实测（原样输出）

```
single channel (default)         keys=12 bytes=11835 hash=75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
single + law=ackermann           keys=12 bytes=11835 hash=75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
single + 1 additional channel    keys=12 bytes=11835 hash=75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9
bytes identical single vs +law  : True
bytes identical single vs +chan : True
steering inner keys ( 13 ) = ['actuator_axis_local', 'actuator_body', 'actuator_mode', 'actuator_reaction_body', 'actuator_reference_rotation', 'input', 'max_rack_displacement', 'max_steering_angle', 'rack_body', 'rack_damping', 'rack_displacement_per_steering_wheel_angle', 'rack_stiffness', 'ratio']
top keys = ['aerodynamic_drag', 'chassis', 'coordinate_couplers', 'coordinate_system', 'driveline', 'front_axle', 'name', 'rear_axle', 'schema_version', 'steering', 'units', 'wheels']
steering required: True
exported top keys (one): ['chassis', 'driveline', 'steering', 'wheels']
exported top keys (two): ['chassis', 'driveline', 'steering', 'steering_channels', 'wheels']
```

## 5. 改动清单自证（`git status --short` / `git diff --numstat`）

```
 M packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json
 M packages/suspension_multibody/src/suspension_multibody/authoring/vehicle.py
 M packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py
 M packages/suspension_multibody/src/suspension_multibody/cases/vehicle_kc.py
 M packages/suspension_multibody/src/suspension_multibody/preparation/vehicle_dynamic.py
 M packages/suspension_multibody/src/suspension_multibody/schema/__init__.py
 M packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py
 M packages/suspension_multibody/tests/authoring/test_vehicle_assembly_documents.py
?? packages/suspension_multibody/src/suspension_multibody/preparation/steering_allocator.py
?? packages/suspension_multibody/tests/preparation/
?? packages/suspension_multibody/tests/schema/test_vehicle_steering_channels.py
```

（仓库根另有 p2-05 / p2-09 / p2-10 / p2-11 等在途改动与另一个 Epic 的规划文档，不属于本行。）

**未出现在清单中的关键文件**（硬约束点名不动的）：`mb_config/version.hpp`、`kernel/native.py`、`templates/roles.py`、`templates/builtin.py`、`subsystems/rig_link.py`、`rigs/**`、`packages/suspension_multibody/tests/data/**`。

`schema/__init__.py` 的 `__all__` 变化：**只增**（`"SteeringChannelSpec"`），未删任何名字 —— `tests/vehicle/test_service_contract.py` 的名字表用例全绿（见第 2 节 `tests/vehicle` 一行）。`preparation/vehicle_dynamic.py` 的 `__all__` 同样只增 5 个新符号（`_steering_channel_specs`、`_steering_allocation_angles`、`_steering_channel_signals`、`_allocation_channels`、`_build_steering_channel`），`_validate_steering_topology` 保留原名与位置。
