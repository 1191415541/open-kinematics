# p2-08 证据：内核 rotational_torque 的驾驶员需求通道与滑移判定

> 来源：`code-reviewer` 裁决（delegation `ea64ca92`，据 p2-02/p2-04 实测缺口）——G1 要求力矩元
> 「在求解期按实时旋转角速度与打滑状态求值」、D1 要求「按模板参数与当时驾驶员信号计算、限幅并施加」，
> 但 p2-02 只交付了元素族且 `demand` 硬编码 1.0，p2-04 的 docstring 把该通道显式推回 p2-02，
> 而 p2-05 的 SPEC 明写「内核侧归 p2-02、版本常量只有 p2-02 可改」——该缺口无人负责，故单独成行。
> 取路径 A（先登记本行再实施）。裁决要点：**不改 ABI 版本常量**（保持 17/32/1）。

## 1. 改前的两处硬编码（实测原文）

`packages/suspension_kernel/cpp/src/element/anti_roll.cpp:128-134`（改前）：

```cpp
        // The driver demand is not wired yet: p2-03 lands the assembly side and
        // p2-04 reads it out of the template slot.  Until then `demand` is the
        // unit demand, so the block's `stiffness` *is* the amplitude this step
        // asks for -- which is the whole of what this row has to deliver.
        const double demand = 1.0;
        const double magnitude =
            std::min(actuator.stiffness * demand, actuator.max_torque);
```

`packages/suspension_kernel/cpp/src/element/directional.cpp:486`（改前）：同样一句 `const double demand = 1.0;`。

`packages/suspension_kernel/cpp/src/force/external_vector.cpp:104-107`（改前）——**手里有 `input` 却没往下传**：

```cpp
    assemble_rotational_torque_forces(
        model, state, torque, energy_rates, record_energy, brush_only,
        external_power
    );
```

对照：同文件里 `assemble_drive_brake_torques` 与 `assemble_steering_forces` 都收到了 `const SampleInput& input`。

## 2. 改动清单（本行写范围内的文件）

| # | 文件 | 改动 |
|---|---|---|
| 1 | `cpp/include/mb_input/types.hpp:311-332` | `enum ElementInteger` 追加 `ELEMENT_INT_TORQUE_DEMAND_SOURCE = 6`、`ELEMENT_INT_TORQUE_TIRE = 7`（**追加**，既有 0..5 未动；`kElementIntBlockSize` 仍是 16） |
| 2 | `cpp/include/mb_model/enums.hpp:83-100` | 新增 `enum TorqueDemandSource { TORQUE_DEMAND_UNIT = 0, TORQUE_DEMAND_WHEEL = 1, TORQUE_DEMAND_BRAKE = 2 }`。该文件自带「只能追加、不得重编号」的约定，`Unit` 取 0 正是旧块的含义 |
| 3 | `cpp/include/mb_model/types.hpp:169-190` | `struct RotationalTorque` 追加 `int demand_source{TORQUE_DEMAND_UNIT};` 与 `int demand_tire{-1};`，并改掉「Until the demand channel lands (p2-03) this *is* the magnitude」的过期注释 |
| 4 | `cpp/include/mb_element/functions.hpp:34-41` | `assemble_rotational_torque_forces` 加 `const SampleInput& input` 形参 |
| 5 | 同文件 `:81-89` | `external_force_rotational_torque_directional` 加同一形参 |
| 6 | `cpp/src/force/external_vector.cpp:104-107` | 调用点补传 `input` |
| 7 | `cpp/src/element/anti_roll.cpp:109-169` | 需求源求值 + 滑移分支（见 §3） |
| 8 | `cpp/src/element/directional.cpp:450-529` | 同律的方向导数版本（见 §4） |
| 9 | `cpp/src/element/directional.cpp:738-744` | 注册点补传 `input` |
| 10 | `cpp/src/assembly/element_reader.cpp:375-405` | 读两个整数槽并校验（见 §5） |
| 11 | `packages/suspension_multibody/.../compilation/element_blocks.py` | 两个整数槽常量 + 三个源常量 + `rotational_torque_block` 写入这两个槽（见 §6） |
| 12 | `packages/suspension_multibody/.../modeling/primitives/elements.py:660-670` | `RotationalTorqueParameters` 加 `demand_source: int = 0` / `demand_tire: int = -1`（默认即旧含义） |
| 13 | `cpp/tests/fixtures/rotational_torque_probe.cpp` | 探针接受 demand/slip 参数并打印 `reader demand_source` 与新增的 `eval` 行 |
| 14 | `cpp/tests/../tests/test_rotational_torque.py`（`packages/suspension_kernel/tests/`） | 4 条新断言（见 §7） |

**未改**：`mb_config/version.hpp`、`kernel/native.py`、`kernel_abi.cpp` 的版本常量——仍是 **17/32/1**（§8 实测）。

## 3. 标量路径的力律（改后）

```cpp
        double demand = 1.0;
        if (actuator.demand_source == TORQUE_DEMAND_WHEEL) {
            demand = slot_value(input.torque, actuator.demand_tire);
        } else if (actuator.demand_source == TORQUE_DEMAND_BRAKE) {
            demand = slot_value(input.brake_torque, actuator.demand_tire);
        }
        const double magnitude =
            std::min(actuator.stiffness * demand, actuator.max_torque);
        const double slip = actuator.demand_tire >= 0 &&
                actuator.demand_source != TORQUE_DEMAND_UNIT &&
                static_cast<std::size_t>(actuator.demand_tire) < state.tire_sx.size()
            ? state.tire_sx[static_cast<std::size_t>(actuator.demand_tire)]
            : 0.0;
        double tau = 0.0;
        if (rate > kEps) {
            tau = -magnitude;
        } else if (rate < -kEps) {
            tau = magnitude;
        } else if (slip > kEps) {
            tau = -magnitude;
        } else if (slip < -kEps) {
            tau = magnitude;
        }
```

三点设计理由：

1. **`slot_value` 而不是裸下标**：`slot_value`（`mb_model/types.hpp:479-481`）本来就是「越界取 0」的既有工具，与驱动/制动力矩路径读同一张表用的是同一个函数；
2. **滑移已在 `State` 里，不需要新传参**：`std::vector<double> tire_sx`（`mb_model/types.hpp:428`，注释原文「Linear transient relaxation pair, present for every PAC2002 tire.」）；`assemble_rotational_torque_forces` 本来就收 `const State&`；
3. **`TORQUE_DEMAND_UNIT` 分支保持原律逐字不变**：没有绑定轮胎的元素走前两个 `rate` 分支，与 p2-02 交付的力律**逐位一致**——这是 §9 的 `dynamic_hash_sentinel` 仍逐字节的机制原因。

**为什么抱死要单列一个分支**：D1 给出的物理理由是「抱死时现有路径输出零制动力矩」（`ω≈0` → `brake_torque=0`），而真实抱死轮仍在滑移并承受满制动力矩。滑移分支正是修掉这一点：`rate` 在 `kEps` 内而 `slip` 显著时，力偶是**满需求幅值**、方向随滑移。

## 4. 方向导数路径

同律，且读同一个 `input`（否则残差与雅可比会对不同的需求求值）。差分的非光滑性判定把滑移分支折进同一个 `smooth` 决策：

```cpp
        if (
            std::abs(rate.value) <= kEps ||
            rate.value*trial_rate <= 0.0 ||
            (std::abs(rate.value) <= kEps && std::abs(slip) > kEps)
        ) {
            smooth = false;
        }
```

需求本身在一个步内是常数（它是**采样值**，不是状态的函数），所以它不携带导数；只有轴与符号分支带导数，这一点与 p2-02 的注释一致。

## 5. 读取端的两条校验

```cpp
            actuator.demand_source = block.ints[ELEMENT_INT_TORQUE_DEMAND_SOURCE];
            actuator.demand_tire = block.ints[ELEMENT_INT_TORQUE_TIRE];
            if (actuator.demand_source < TORQUE_DEMAND_UNIT ||
                actuator.demand_source > TORQUE_DEMAND_BRAKE) {
                error = "unknown rotational torque demand source";
                return false;
            }
            if (actuator.demand_source != TORQUE_DEMAND_UNIT &&
                actuator.demand_tire < 0) {
                error = "a driven rotational torque needs the tire it follows";
                return false;
            }
```

两个整数槽的**默认值都是 0**（`ElementBlock` 由调用方零填充），所以：
- 旧块（`ints` 全零）读出来是 `source=0, tire=0`——单位为需求、轮胎 0。**`tire=0` 在单位需求下不被读取**（滑移分支要求 `source != UNIT`），所以旧块的行为与改前逐位一致；
- 有源的块必须显式给出轮胎，否则由名字拒绝，而不是静默读第 0 列。

**未在读取端校验轮胎上界**：该族的块可能在轮胎块之前被读入（`read_element_blocks` 逐块顺序处理），此时 `model.tires` 还是空的。力律用 `slot_value` 对越界取 0 需求，行为是可定义的（无需求即无力偶），注释里写明了这个取舍。

## 6. Python 侧的槽编码

`compilation/element_blocks.py` 新增：

```python
ROTATIONAL_TORQUE_DEMAND_SOURCE_INT = 6
ROTATIONAL_TORQUE_DEMAND_TIRE_INT = 7

TORQUE_DEMAND_UNIT = 0
TORQUE_DEMAND_WHEEL = 1
TORQUE_DEMAND_BRAKE = 2
```

`rotational_torque_block` 末尾：

```python
    ints = [0] * ELEMENT_BLOCK_INT_SIZE
    ints[ROTATIONAL_TORQUE_DEMAND_SOURCE_INT] = int(parameters.demand_source)
    ints[ROTATIONAL_TORQUE_DEMAND_TIRE_INT] = int(parameters.demand_tire)
```

改前该行是 `ints=tuple([0] * ELEMENT_BLOCK_INT_SIZE)`——即**全零**，恰好是本族的新默认（单位需求、无轮胎）。所以既有调用者（`tests/subsystems/test_rotational_torque_element.py` 的 29 用例）产物逐项不变。

## 7. 新增断言（`packages/suspension_kernel/tests/test_rotational_torque.py`）

四条，逐条对应本行的判据：

| 测试 | 断言原文 | 判定什么 |
|---|---|---|
| `test_the_demand_channel_follows_the_driver_signal` | `assert driven_b == -200.0` | 增益 400、驾驶员要 0.5 → 力偶 200。单位需求会给 400，所以这条判的是「读的是哪个数」 |
| `test_a_zero_demand_applies_no_couple` | `assert released_a == 0.0` / `assert released_b == 0.0` | 驾驶员要零即零。同块、同速率、不同需求给不同力偶——这才是「需求是幅值而非常数」 |
| `test_a_locked_wheel_is_still_braked` | `assert locked_b == -400.0`；`assert reverse_b == -locked_b` | **抱死态**：`ω` 恰为 0 而滑移 0.8，力偶是**满需求幅值**（400×1.0）而非零。反向滑移时符号翻转 |
| `test_the_reader_keeps_the_demand_slots` | `assert "demand_source 2 2" in probe["reader"]` | 两个整数槽确实进了模型，不是被读后丢弃 |

另有既有的 8 条（静止/正转/倒转/饱和/接地反力/读取往返/布局行）**一条未删未弱化**。文件总用例 12。

静止态（`unit_still`，单位需求 + ω=0 + 无滑移）仍为 `tau_a 0 tau_b 0`——与 p2-02 的三态断言口径一致，新增的滑移分支不改变它。

## 8. ABI 版本常量未动（实读）

```
$ grep -n "kAxleKernelAbiVersion\|kVehicleKernelAbiVersion\|kCoreKernelAbiVersion" packages/suspension_kernel/cpp/include/mb_config/version.hpp
33:  kAxleKernelAbiVersion = 17,
40:  kVehicleKernelAbiVersion = 32,
43:  kCoreKernelAbiVersion = 1

$ grep -n "_NATIVE_KERNEL_ABI_VERSION\|_NATIVE_VEHICLE_KERNEL_ABI_VERSION\|_NATIVE_CORE_ABI_VERSION" packages/suspension_multibody/src/suspension_multibody/kernel/native.py
33:_NATIVE_KERNEL_ABI_VERSION = 17
34:_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 32
35:_NATIVE_CORE_ABI_VERSION = 1
```

依据（裁决第 2 条，与 p2-07 同一先例）：该族是本 Epic 新增、唯一生产者是 `compilation/element_blocks.py` 的
`rotational_torque_block`；新增的两个整数槽取自 `kElementIntBlockSize = 16` 的既有**空位**（既有枚举只用到 0..5），
参数块宽度与结构体布局一字未改。所以没有别的 ABI 消费者会受影响。

## 9. 主管实跑（命令与退出码）

```text
$ uv run --no-sync python packages/suspension_kernel/scripts/build_suspension_kernel.py
E:\杂件\open-kinematics\packages\suspension_kernel\src\suspension_kernel\native\suspension_kernel.dll
BUILD_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/scripts/build_axle_native.py
E:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\native\suspension_kernel.dll
MIRROR_EXIT=0

$ uv run --no-sync pytest packages/suspension_kernel/tests -q -p no:cacheprovider
45 passed in 14.95s                                         KERNEL_EXIT=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py \
    packages/suspension_multibody/tests/architecture/test_element_block_layout.py \
    packages/suspension_multibody/tests/architecture/test_core_abi.py -q -p no:cacheprovider
29 passed in 1.46s                                          PY_EXIT=0

$ uv run --no-sync ruff check packages/suspension_multibody/src packages/suspension_kernel
All checks passed!                                          RUFF_EXIT=0
$ uv run --no-sync ty check packages/suspension_multibody/src
All checks passed!                                          TY_EXIT=0

$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
module cycles (SCC size>1) : 0
OK: layering matches the recorded baseline                  LAYER_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
findings  : 0
OK: no unregistered Python boundary violation               LEGACY_EXIT=0

$ uv run --no-sync pytest packages/suspension_multibody/tests \
    --ignore=.../tests/adams --ignore=.../tests/architecture --ignore=.../tests/cases -q -p no:cacheprovider
1201 passed, 1 xfailed in 44.64s                            FAST_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte  SENTINEL_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py   # 无参数
OK: 8 families accepted                                     PARITY_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py --check
OK: benchmarks are within the recorded budget                KCPERF_EXIT=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
147 passed in 627.75s                                       ARCH_EXIT=0

$ uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
OK: 3 release checks passed                                 COMPOSABLE_EXIT=0

$ git status --short -- packages/suspension_multibody/tests/data/
（空）                                                       DATA_CLEAN
```

`dynamic_hash_sentinel --check` 的 combined sha256 与 p2-01/p2-02/p2-07 记录的起点值**逐字节一致**，未重录任何基线。

## 10. 探针的完整输出（新态的实测数字）

```text
reader ok 1
reader count 1
reader stiffness 250
reader damping 0
reader max_torque 900
reader axis 0 1 0
reader bodies 0 1
reader demand_source 2 2
eval still tau_a 0 tau_b 0
eval forward tau_a 400 tau_b -400
eval reverse tau_a -400 tau_b 400
eval capped tau_a 1000 tau_b -1000
eval uncapped tau_a 400 tau_b -400
eval grounded tau_a -400 tau_b 0
eval driven tau_a 200 tau_b -200
eval no_demand tau_a 0 tau_b 0
eval locked tau_a 400 tau_b -400
eval locked_reverse tau_a -400 tau_b 400
eval unit_still tau_a 0 tau_b 0
```

`still`/`forward`/`reverse`/`capped`/`uncapped`/`grounded` 六条与 p2-02 的八个用例读数**逐字相同**（该行未弱化任何既有断言）；
`driven`/`no_demand`/`locked`/`locked_reverse`/`unit_still` 是本行新增的五条。

## 11. 本行与相邻行的接口

- **p2-05 依赖本行**：删除 `_build_wheel_torque_signals` 后，工况文档里的 `wheel_torque` / `brake_torque` 列从
  「已经乘好、单位 N·m 的力矩」变为「归一化驾驶员需求」，元素块用 `demand_source` 声明自己读哪一列、
  `demand_tire` 声明读哪一条轮胎的列；
- **p5-04 依赖本行的滑移通道**：ABS 要在 `ω≈0` 而滑移大时仍能制动，靠的就是这里的滑移分支——没有它，
  闭环控制器无论怎么调都会被力律的 `kEps` 分支归零；
- **未做（如实在此登记）**：本行只交付**内核求值**与**编码/读取**。把 brake/drive 的力矩元真正接进整车装配的
  元素面、并把契约表的语义从 N·m 改为归一化需求，是 **p2-05** 的写范围，不在本行。
