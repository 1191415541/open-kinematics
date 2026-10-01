# PROGRESS：p2-02 内核旋转主动力矩元与 ABI 变更

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-02`

## 状态

`DONE`（2026-10-01）。五条判据全部实测，证据落 `raw/kernel_torque_evidence.md`（159 行）。

**本行是本 Epic 唯一被授权改 `mb_config/version.hpp` 与 `kernel/native.py` 版本常量的行**（另一次视 D2 的 p5-04）。

## 交付物

| 判据 | 证据位置 | 内容 |
|---|---|---|
| (a) 元素类型五处打通 | `raw/kernel_torque_evidence.md` §(a) | 10 处锚点表：ElementKind、参数索引、布局表、模型结构、元素读取、wrench 码、标量求值、方向/导数路径、编组计数、ABI 镜像 |
| (b) ABI 真源单点 | 同 §(b) | `version.hpp` 16/31/1 → **17/32/1** 与 `kernel/native.py:33-35` 成对同步；受联动影响的 4 处既有 pin 的改前/改后表 |
| (c) 三态断言 | 同 §(c) | 静止 / 正转 / 倒车 / 饱和 / 接地反力 / 读取往返 / 布局行，共 8 用例（含断言原文与力律依据行号） |
| (d) 分层与退役面门 | 同 §(d) | `check_module_layering --strict --final` 0 环退出码 0；`legacy_surface_gate --check` findings 0 |
| (e) 动态哈希零回归 | 同 §(e) | `dynamic_hash_sentinel --check` 退出码 0、combined sha256 与 p2-01 起点逐字节一致；`tests/data` 未被写 |

## 实测要点

### 内核新元素（`ELEMENT_ROTATIONAL_TORQUE = 7`）

绕 body 固定轴的**纯力偶**，无作用点（照 `anti_roll.cpp` 的力偶范式）：对 `b` 施加 `axis_world * tau`、
对 `a` 施加 `axis_world * (-tau)`。符号律照 `drive_brake.cpp:91-98`：`rate > kEps → tau = -magnitude`、
`rate < -kEps → tau = magnitude`、其余为 0；`magnitude = min(stiffness * demand, max_torque)`。
`actuator.b < 0` 时视为接地（只对 `a` 施力），`relative_omega = omega[a] * (-1)`。

**这解决了 D1 的物理决定性理由**：现有制动路径在抱死（`ω≈0`）时输出**零**制动力矩，而真实抱死轮仍在滑移、
应承受满制动力矩；本元素按实时相对角速度求值、以 `max_torque` 表达饱和，故抱死态有力矩。

### 参数槽位为什么是 128..137（如实登记的字面偏差）

规格第 3 条写「追加到该枚举末尾」。索引常量确实追加在 `enum ElementParameter` 末尾，但**数值**取
bushing 声明带的未用尾部 128..137，理由：`kElementBlockSize = 216`（`types.hpp:46`）是 ABI 的一部分，
且被产品侧 ctypes 镜像钉住（`tests/architecture/test_core_abi.py:108` 的 `ctypes.c_double * 216`）；
bump-stop run 后的空槽只有 210..215（六个），少于本族需要的十个；加宽块等于让每个 C ABI 调用方重编译。
bushing 自身字段止于 `ELEMENT_BUSHING_REFERENCE_QUATERNION = 111`（即 111..114），115..143 从未被写过。
这与 `ELEMENT_SPRING_PRELOAD` 复用 spring run 内退役槽位是同一做法。C++ `static_assert`（`types.hpp:505-518`）
钉住该切片与两侧邻居不重叠。

### ABI 联动

| 真源 | 改前 | 改后 |
|---|---|---|
| `mb_config/version.hpp:33` `kAxleKernelAbiVersion` | 16 | **17** |
| 同 `:40` `kVehicleKernelAbiVersion` | 31 | **32** |
| 同 `:43` `kCoreKernelAbiVersion` | 1 | 1（未动） |
| `kernel/native.py:33-35` | 16 / 31 / 1 | **17 / 32 / 1** |

整车常量随轴联动，因为 `VehicleInput` 按值嵌入 `AxleInput`。

### 三态断言（判据 3 的直接交付）

| 态 | 断言 |
|---|---|
| 静止 ω ≈ 0 | `tau_a == 0.0`、`tau_b == 0.0`（**不产生反向加速**） |
| 抱死/饱和 | `capped_b == -1000.0` 且与未饱和态 `uncapped_b == -400.0` 对照，证明 cap 生效而非力律丢需求 |
| 倒车 ω < 0 | `reverse_b > 0.0`、`reverse_b == -forward_b`、`reverse_a == -reverse_b`（**符号正确**） |

## 主管补的两处缺口（实现代理无权改 `packages/suspension_multibody/tests/**`）

1. **4 处 ABI pin**：`tests/axle_dynamics/test_packaging.py:52-53` 与 `test_performance_metrics.py:168-169`
   仍 pin 16/31 → 已改 17/32（附理由注释）。实现代理如实报告了这两处会失败。
2. **元素布局表登记**：`tests/architecture/test_element_block_layout.py` 的 `FAMILIES`/`FAMILY_RUNS`
   未登记新族——不登记则新族**不在**「两族不得共用槽位」的声明重叠检查表里。已补
   `"ROTATIONAL_TORQUE": "ELEMENT_ROTATIONAL_TORQUE"` 与 `(128, 144)`，并把 `BUSHING` 的声明带由
   `(16, 144)` 收正为 `(16, 116)`。该文件实测 **7 passed**。

## 主管实跑（与实现代理自报分开列）

```text
check_module_layering.py --strict --final  → EXIT=0（0 环、0 反向边、0 未登记）
pytest packages/suspension_kernel/tests -q → EXIT=0，41 passed in 15.82s
pytest tests/architecture/test_element_block_layout.py → EXIT=0，7 passed
pytest tests/axle_dynamics/test_performance_metrics.py::test_native_build_metadata_keeps_safe_optimization_flags → EXIT=0，1 passed
legacy_surface_gate.py --check → EXIT=0，findings : 0
dynamic_hash_sentinel.py --check → EXIT=0，combined sha256 : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
                                   OK: dynamic output matches the frozen baseline byte-for-byte
git status --short -- packages/suspension_multibody/tests/data/ → 空（未重录基线）
```

## 已知限制与风险（不掩盖）

1. **求值的 `demand` 硬编码 `1.0`**（`anti_roll.cpp:132`）：即 `stiffness` 直接就是幅值。代码注释写明
   demand 将于 **p2-03（装配侧）/ p2-04（模板槽读出）** 接入。`damping` 已读取并存入模型，
   但本行求值不使用（符号律本身已给出方向），`struct` 注释已注明。
2. **`test_rotational_torque.py` 的静态库链接列表**取自 `CMakeFiles/TargetDirectories.txt` 而非
   `build.glob("libmb_*.a")`：本机 `build/Release/` 残留模块拆分**之前**的旧静态库（`libmb_base.a` 等 7 个），
   原样 glob 会 `multiple definition` 链接失败。非本次改动引入，是构建目录陈旧所致；清理构建目录后需复验。
3. **`dynamic_hash_sentinel --check` 输出里的 `acceptance exit : 1`**：那是脚本内部先跑一次 acceptance
   生成产物时的退出码，与 p2-01 记录的起点行为一致；`--check` 的判定结论是 `OK: ... byte-for-byte`，
   combined sha256 与冻结基线一致，故零回归成立。

## 未改动的边界（`git diff --stat` 自证）

只动 `packages/suspension_kernel/cpp/**`（14 文件）、`packages/suspension_kernel/tests/`（2 改 + 1 新增 + 1 fixture）、
`packages/suspension_multibody/src/suspension_multibody/kernel/native.py`（版本常量）、
以及主管补的 3 个 `packages/suspension_multibody/tests/**` 文件。
**未触碰** `subsystems/element_build.py`、`subsystems/brake.py`、`subsystems/drive.py`、`templates/roles.py`、
`templates/builtin.py`、`preparation/vehicle_dynamic.py`。
