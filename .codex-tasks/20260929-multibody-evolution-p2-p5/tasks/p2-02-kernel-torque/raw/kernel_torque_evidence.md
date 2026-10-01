# p2-02 证据（主管整理，2026-10-01）

> 本文件由主管在实现代理交付后整理。**所有锚点与原文本文件都是当前工作树的实读结果**；
> 命令与退出码是主管本人实跑（见文末「主管实跑」，与实现代理自报分开列）。
> 实现代理的原始交付说明见会话记录；本文件只记工作树里**可复核**的事实。

## (a) 内核新增元素类型：五处打通

| 处 | 文件:行 | 原文 / 事实 |
|---|---|---|
| 1. `ElementKind` | `packages/suspension_kernel/cpp/include/mb_input/types.hpp:74-80` | `ELEMENT_BUMP_STOP = 6,` 之后新增 `ELEMENT_ROTATIONAL_TORQUE = 7`（注释说明「pure couple about one body-fixed axis，方向按实时相对角速度、幅值被 MAX_TORQUE 限住」，且**追加在末尾不移动既有 kind**） |
| 2. 参数索引 | 同文件 `:280-307` | 新增 `ELEMENT_ROTATIONAL_TORQUE_STIFFNESS = 128`、`_DAMPING = 129`、`_AXIS_A = 130`（3 连）、`_REFERENCE_QUATERNION = 133`（4 连）、`_MAX_TORQUE = 137` |
| 3. 布局表 | 同文件 `:343-346` | `kElementLayouts[]` 新增 `{ELEMENT_ROTATIONAL_TORQUE, ELEMENT_ROTATIONAL_TORQUE_STIFFNESS, ELEMENT_ROTATIONAL_TORQUE_DAMPING, ELEMENT_ROTATIONAL_TORQUE_AXIS_A, ELEMENT_ROTATIONAL_TORQUE_MAX_TORQUE, /*curve_slots=*/0, /*int_count=*/0}` |
| 4. 模型结构 | `packages/suspension_kernel/cpp/include/mb_model/types.hpp:158`（`struct RotationalTorque`）、`:353`（`std::vector<RotationalTorque> rotational_torques;` 追加在 `anti_roll_bars` 之后） | 字段 `a, b, axis_a, reference, stiffness, damping, max_torque` |
| 5. 元素读取 | `packages/suspension_kernel/cpp/src/assembly/element_reader.cpp:354-390` | `if (block.kind == ELEMENT_ROTATIONAL_TORQUE)` 分支：读 5 个参数索引（`:359/:361/:363-365/:368-371/:374`），校验轴范数与负值（`"invalid rotational torque parameters"`），normalize 轴 |
| 6. wrench 码 | `packages/suspension_kernel/cpp/include/mb_config/element_wrench.hpp:51`（`kElementWrenchRotationalTorque = 10`）、`:60`（`kElementWrenchCodeCount` 跟随该常量）、`:76`（`ElementWrenchCounts.rotational_torques`） | 通道登记 |
| 7. 求值（标量路径） | `packages/suspension_kernel/cpp/src/element/anti_roll.cpp:109-172`（`assemble_rotational_torque_forces`），调用点 `packages/suspension_kernel/cpp/src/force/external_vector.cpp:104`（在 `assemble_anti_roll_forces` 之后、`assemble_steering_forces` 之前） | 见下「三态」 |
| 8. 求值（方向/导数路径） | `packages/suspension_kernel/cpp/src/element/directional.cpp:458` 起（`external_force_rotational_torque_directional`），登记点 `:712-716` | 与标量路径成对，`kEps` 内或穿越判 `smooth = false` |
| 9. 编组计数 | `packages/suspension_kernel/cpp/src/config/kernel_config.cpp:411`（`element_wrench_rows_per_element` 新码返回 2）、`:447`（`element_wrench_element_count` 返回 `counts.rotational_torques`）；`cpp/src/abi/kernel_contract_run.cpp:591`；`cpp/src/output/kernel_output.cpp:396-397` | ABI 编组/输出 |
| 10. ABI 版本镜像 | `packages/suspension_kernel/cpp/src/abi/kernel_abi.cpp:857-858`（`kAxleInputFieldAbiVersion 17` / `kVehicleInputFieldAbiVersion 32`） | 编译期 `static_assert` 镜像 |

**参数槽位落在 128..137 而非块尾（实测理由）**：`kElementBlockSize = 216`（`types.hpp:46`）是 ABI 的一部分，
且被产品侧镜像钉住（`packages/suspension_multibody/tests/architecture/test_core_abi.py:108` 的
`ctypes.c_double * 216`）。bump-stop run 之后的空槽只有 210..215（六个），少于本族需要的十个；
加宽块等于让每个 C ABI 调用方重编译。故本族取 bushing 声明带的**未使用尾部** 128..137
（bushing 自身字段止于 `ELEMENT_BUSHING_REFERENCE_QUATERNION = 111`，即 111..114）。这与
`ELEMENT_SPRING_PRELOAD` 复用 spring run 内退役槽位是同一做法。C++ `static_assert`
（`types.hpp:505-518`）钉住该切片与两侧邻居不重叠：
`ELEMENT_BUSHING_REFERENCE_QUATERNION + 4 <= ELEMENT_ROTATIONAL_TORQUE_STIFFNESS`、
`ELEMENT_ROTATIONAL_TORQUE_MAX_TORQUE < ELEMENT_ANTI_ROLL_STIFFNESS`。

## (b) ABI 真源单点同步

| 真源 | 改前 | 改后 | 依据 |
|---|---|---|---|
| `cpp/include/mb_config/version.hpp:33` `kAxleKernelAbiVersion` | 16 | **17** | 元素面新增一个族 |
| 同文件 `:40` `kVehicleKernelAbiVersion` | 31 | **32** | `VehicleInput` 按值嵌入 `AxleInput`，故随轴常量联动 |
| 同文件 `:43` `kCoreKernelAbiVersion` | 1 | **1**（未动） | 通用 core 面未变 |
| `packages/suspension_multibody/src/suspension_multibody/kernel/native.py:33-35` | 16 / 31 / 1 | **17 / 32 / 1** | Python 侧单一真源，与 C++ 成对同步 |

受 ABI 联动影响的既有 pin 已同步（**这是 p2-02 变更的必然结果，非判据弱化**——只改期望值并附理由注释）：

| 文件:行 | 改前 | 改后 |
|---|---|---|
| `packages/suspension_kernel/tests/test_binding.py:43-44`（`EXPECTED_ABI`/`EXPECTED_VEHICLE_ABI`） | 16 / 31 | 17 / 32 |
| `packages/suspension_kernel/tests/test_tire_mass.py:97-99`（`FROZEN_ABI_VERSIONS`） | 16 / 31 / 1 | 17 / 32 / 1 |
| `packages/suspension_multibody/tests/axle_dynamics/test_packaging.py:52-53` | 16 / 31 | **17 / 32**（主管补） |
| `packages/suspension_multibody/tests/axle_dynamics/test_performance_metrics.py:168-169` | 16 / 31 | **17 / 32**（主管补） |
| 已构建镜像 `packages/suspension_multibody/src/suspension_multibody/native/native_build.json` | `"abi_version": 16` | **17**（重建内核写入） |

**另外的元素布局表登记**（主管补）：`packages/suspension_multibody/tests/architecture/test_element_block_layout.py`
的 `FAMILIES` 新增 `"ROTATIONAL_TORQUE": "ELEMENT_ROTATIONAL_TORQUE"`、`FAMILY_RUNS` 新增
`"ROTATIONAL_TORQUE": (128, 144)`，并把 `BUSHING` 的声明带从 `(16, 144)` 收正为 `(16, 116)`
（bushing 实际字段止于 115）。该文件是「两族不得共用槽位」的声明重叠检查表——新族不登记就**不在**那张
检查表里（实现代理正确地指出了这一点，但它无权改 `packages/suspension_multibody/tests/**`，由主管补）。

## (c) 力元求值三态断言

测试文件：`packages/suspension_kernel/tests/test_rotational_torque.py`（8 个用例），
经 `packages/suspension_kernel/tests/fixtures/rotational_torque_probe.cpp`（C++ 探针）在测试期编译运行，
打印 `reader ...` 与 `eval ...` 两类行。

| 态 | 测试 | 断言原文 | 力律依据（`anti_roll.cpp`） |
|---|---|---|---|
| **静止**（ω ≈ 0） | `:168 test_a_stationary_pair_gets_no_couple` | `:179 assert tau_a == 0.0`；`:180 assert tau_b == 0.0` | `:135 double tau = 0.0;`——`rate > kEps` 与 `rate < -kEps` 都不成立时保持 0，`:150/:157` 施加 `axis_world * 0`。**即静止不产生反向加速。** |
| **正转反向** | `:183 test_the_couple_opposes_a_forward_rate` | `:189 assert tau_b < 0.0`；`:191 assert tau_a == -tau_b`；`:193 assert tau_b == -400.0` | `:136-137 tau = -magnitude`（`rate > kEps`） |
| **倒车符号翻转** | `:196 test_the_couple_reverses_with_a_reverse_rate` | `:208 assert reverse_b > 0.0`；`:209 assert reverse_b == -forward_b`；`:210 assert reverse_a == -reverse_b` | `:138-139 tau = magnitude`（`rate < -kEps`），与正向成对；与 `drive_brake.cpp` 的 ω 符号律同口径 |
| **饱和（抱死）** | `:213 test_an_over_demand_couple_saturates_at_the_cap` | `:227 assert capped_b == -1000.0`；`:229 assert uncapped_b == -400.0`；`:232-233 assert abs(capped_b) == 1000.0` 且 `> abs(uncapped_b)` | `:133-134 std::min(actuator.stiffness * demand, actuator.max_torque)`——**并与未饱和态对照，证明是 cap 生效而非力律丢需求** |
| **接地反力** | `:236 test_a_grounded_reaction_lands_on_the_one_body` | `:246 assert tau_b == 0.0`；`:247 assert tau_a < 0.0` | `actuator.b < 0` 分支 `:129-131`：`relative_omega = state.omega[a] * (-1.0)` |
| **读取往返** | `:147 test_the_block_reaches_the_built_model`、`:154 test_the_reader_keeps_the_values_the_block_carried` | `:149-151 "ok 1"/"count 1"/"bodies 0 1"`；`:161-165 "stiffness 250"/"max_torque 900"/"axis 0 1 0"` | `element_reader.cpp:354-390` |
| **布局行未位移既有 kind** | `:250 test_the_family_has_its_own_layout_row` | `:259 "ELEMENT_ROTATIONAL_TORQUE = 7"`；`:260` 布局行原文；`:271` 既有 7 个 kind 逐条仍在 | `types.hpp` |

**已知限制（如实登记，不是缺口）**：求值里的 `demand` 目前硬编码 `1.0`（`anti_roll.cpp:132` 与注释），
即 `stiffness` 直接就是幅值。注释写明 demand 将于 **p2-03（装配侧）/ p2-04（模板槽读出）** 接入。
`damping` 已读取并存入模型，但本行求值不使用（符号律本身已给出方向），`struct` 注释已注明。

## (d) 分层门与退役面门

| 命令 | 退出码 | 关键输出 |
|---|---|---|
| `check_module_layering.py --strict --final` | **0** | `module cycles (SCC size>1) : 0`、`mutual (reverse) edges : 0`、`legacy modules unregistered: 0 []`、`OK: layering matches the recorded baseline` |
| `legacy_surface_gate.py --check` | **0** | `findings : 0`（主管另跑 `--check`，见下） |

## (e) 动态哈希零回归

| 命令 | 退出码 | 关键输出 |
|---|---|---|
| `dynamic_hash_sentinel.py --check` | **0** | `combined sha256 : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`、`OK: dynamic output matches the frozen baseline byte-for-byte` |

**与 p2-01 起点值逐字节一致**（`tasks/p2-01-freeze/raw/gates_baseline.md` 记录的同一 combined sha256）。
`git status --short -- packages/suspension_multibody/tests/data/` 实测**为空**（未重录任何基线）。

## 主管实跑（与实现代理自报分开列，2026-10-01）

```text
$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
legacy modules unregistered: 0 []
mutual (reverse) edges     : 0
self-including headers     : 0
cross-aggregate includes   : 0
module cycles (SCC size>1) : 0

OK: layering matches the recorded baseline
LAYER_EXIT=0

$ uv run --no-sync pytest packages/suspension_kernel/tests -q -p no:cacheprovider
41 passed in 15.82s
KERNEL_EXIT=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_element_block_layout.py -q -p no:cacheprovider
7 passed in 0.10s
EXIT=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/axle_dynamics/test_performance_metrics.py::test_native_build_metadata_keeps_safe_optimization_flags -q -p no:cacheprovider
1 passed in 0.22s
EXIT=0
```

`dynamic_hash_sentinel.py --check` 主管实跑（在两条并行写入链收敛后）：

```text
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : [... 9 个 ...]

OK: dynamic output matches the frozen baseline byte-for-byte
EXIT=0
```

**关于 `acceptance exit : 1`**：这是 `dynamic_hash_sentinel` 内部**先跑一次 acceptance 生成产物**时的
退出码，与 p2-01 记录的起点行为一致（该脚本以 `--check` 判定 26 个 artifact 的 sha256，判定结论是
`OK: ... byte-for-byte`）。**组合哈希与冻结基线一致**，故零回归成立；`failed cases` 一栏是 acceptance
套件在无 Adams 参考证据时的既有状态，不是本行引入。

## 未做到 / 与规格字面不符之处（全部已处置，如实登记）

1. **参数槽位未追加到块尾**（取值 128..137，理由见 (a)）。规格第 3 条写「追加到该枚举末尾」——
   索引常量确实追加在 `enum ElementParameter` 末尾，但**数值**落在 bushing 带的未用尾部。
   理由与 `static_assert` 已写进代码注释。
2. **4 处 ABI pin 由主管补齐**：实现代理无权改 `packages/suspension_multibody/tests/**`，
   它如实报告了 `axle_dynamics` 下 2 处会失败。主管补齐后才跑通。
3. **元素布局表登记由主管补齐**（同上原因）。
4. **`test_rotational_torque.py` 的静态库链接列表取自 `CMakeFiles/TargetDirectories.txt`
   而非 `build.glob("libmb_*.a")`**：本机 `build/Release/` 残留模块拆分**之前**的旧静态库
   （`libmb_base.a` 等 7 个，Sep 18），原样 glob 会 `multiple definition` 链接失败。
   这不是本次改动引入的重复定义，是构建目录陈旧所致。**风险登记**：若清理构建目录后重跑该测试，
   需确认链接列表来源仍成立。
5. **`demand` 硬编码 1.0**（见 (c) 的已知限制）。

## 未改动的边界（自证）

`git diff --stat` 显示本行只动 `packages/suspension_kernel/cpp/**`（14 个文件）、
`packages/suspension_kernel/tests/`（3 个文件，2 改 1 新增 + 1 fixture）、
`packages/suspension_multibody/src/suspension_multibody/kernel/native.py`（版本常量）、
以及主管补的 3 个 `packages/suspension_multibody/tests/**` 文件。
**未触碰** `subsystems/element_build.py`、`subsystems/brake.py`、`subsystems/drive.py`、
`templates/roles.py`、`templates/builtin.py`、`preparation/vehicle_dynamic.py`。
