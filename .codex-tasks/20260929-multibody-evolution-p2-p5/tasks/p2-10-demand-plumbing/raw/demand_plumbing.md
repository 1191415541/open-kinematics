# p2-10 证据：内核 normalized demand 独立缓冲与旧 N·m 路由隔离

> 全部为 2026-10-01 本机实跑/实读结果。来源：`code-reviewer` 裁决 `5e01b75d`
> （p2-09 开工前实测缺口的裁决；该裁决否决了「同缓冲让位守卫」方案 B、采纳方案 A）。

## 1. 缺口的实测证据（改前）

同一个 `SampleInput::torque` / `SampleInput::brake_torque` 被两族消费：

| 消费方 | file:line | 语义 |
|---|---|---|
| rotational_torque 标量路径 | `cpp/src/element/anti_roll.cpp:137` / `:139` | `min(stiffness*demand, max_torque)`，`demand` 是无量纲分数 |
| rotational_torque 方向导数路径 | `cpp/src/element/directional.cpp:493` / `:495` | 同上 |
| 旧 drive/brake 直给路径 | `cpp/src/element/drive_brake.cpp:46-47` / `:89-90` | 按 N·m 直接 `add_torque_on_body` |
| 旧直给的方向导数路径 | `cpp/src/element/directional.cpp:777-788` | 同上 |

两族写同一 `torque` 累加器（`cpp/src/force/external_vector.cpp:104` 的 rotational_torque 与
`:120` 的 drive_brake）。把无量纲需求写进旧表，内核会**同时**（a）按需求通道施加正确的力偶，
（b）按 N·m 再施加一次该数值本身的寄生力偶（≤1 N·m），且两族无法靠 role 名区分。

方案 B（旧路径按声明让位）被裁决否决：它解决不了同一缓冲的单位混用，也覆盖不了
`directional.cpp` 的旧直给路径。故取方案 A：**独立缓冲 + 旧表语义不变**。

## 2. 改动清单（逐文件）

| 文件 | 改动 |
|---|---|
| `cpp/include/mb_model/types.hpp` | `SampleInput` 增 `wheel_demand` / `brake_demand` 两个 vector（与旧 `torque`/`brake_torque` 并列）；`Model` 增两个注册期 `const double*`（照 `vehicle_brake_torque` 先例，null = 未声明） |
| `cpp/include/mb_cases/functions.hpp` | `ContractCase` 增 `wheel_demand` / `brake_demand` 两个 per-tire 表（**不**在构造时填充，空 = 未声明） |
| `cpp/src/input/kernel_input.cpp` | `interpolate_input(model, ...)` 按与 brake_torque 相同的括取与插值填两张新表；未注册时保持零 |
| `cpp/src/abi/kernel_contract_run.cpp` | 循环外分配两张稳定缓冲；仅当有 case 声明该表时注册模型指针；每 case 全量 memcpy，未声明时显式清零（防跨 case 泄漏） |
| `cpp/src/element/anti_roll.cpp` | 需求分支改读 `input.wheel_demand` / `input.brake_demand` |
| `cpp/src/element/directional.cpp` | 同一分支同步改读（标量路径与导数路径必须同源） |
| `cpp/src/cases/vehicle_dynamic.cpp` | `tire_role_of` 新增 `throttle_demand` / `brake_pressure` 两个归一化 role（scale = 1.0，不做量纲缩放）；首次出现时惰性分配目标表；按**通道**（wheel / brake）追踪已声明的单位制，同通道混用两制按名拒绝；需求值超出 `[0,1]`（brake）或 `[-1,1]`（throttle）或非有限时按名拒绝 |
| `cpp/tests/fixtures/rotational_torque_probe.cpp` | 新增 `decoupled`（同轮给满需求 + 陈旧 250 N·m）与 `isolate none/both`（四目标插值读数） |
| `packages/suspension_kernel/tests/test_rotational_torque.py` | 新增 2 条断言（见 §4） |

**未改**：`AxleInput` / `VehicleInput` / `AxleOutput` / `VehicleOutput` / `ElementBlock` /
`abi/functions.hpp` / `mb_config/version.hpp` / `kernel/native.py`。版本常量仍 **17/32/1**。

## 3. 实跑门禁

| 命令 | 结果 |
|---|---|
| `build_suspension_kernel.py` | 退出码 0，产出 `.../native/suspension_kernel.dll` |
| `build_axle_native.py`（**必须**，否则镜像过期） | 退出码 0 |
| `pytest packages/suspension_kernel/tests -q` | **47 passed**（起点 45；+2 为本行新增） |
| 快速集（除 adams/architecture/cases） | **1201 passed / 1 xfailed**（与起点逐项一致） |
| `dynamic_hash_sentinel.py --check` | combined sha256 = `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，**逐字节一致** |
| `case_parity_check.py`（无参数） | **8 families accepted** |
| `kc_perf_gate.py --check` | 在记录预算内 |
| `check_module_layering.py --strict --final` | 0 环，退出码 0 |
| `legacy_surface_gate.py --check` | findings 0，退出码 0 |
| `pytest tests/architecture -q` | **147 passed**（与起点一致） |
| `pytest packages/suspension_contracts/tests -q` | **32 passed** |
| `check_composable_release.py --skip-isolation` | 3 release checks passed |
| `ruff check .` / `ty check .` | 全仓 exit 0 |
| `git status --short -- packages/suspension_multibody/tests/data/` | **空**（未重录任何基线） |

## 4. 新断言（不是弱证据）

### 4.1 两套单位在采样层就不混（`test_the_sampler_keeps_the_two_unit_systems_apart`）

探针 `dump_input_isolation` 用同一份输入（wheel torque `10→20`、brake torque `30→40`、
wheel demand `0.25→0.75`、brake demand `0.5→1.0`）在 `t = 0.5` 读四个目标，**逐字**断言：

```
isolate none torque 15 brake_torque 35 wheel_demand 0 brake_demand 0
isolate both torque 15 brake_torque 35 wheel_demand 0.5 brake_demand 0.75
```

即：声明需求后，两个需求目标携带 case 自己的分数（中点），而两个 N·m 目标
**一字未变**。归一化绝不渗进力矩缓冲。

### 4.2 需求通道不读力矩表（`test_the_demand_channel_does_not_read_the_torque_tables`）

探针 `dump_eval("decoupled", ...)` 给同一轮**同时**满需求 1.0 与陈旧 250 N·m：

```
eval decoupled tau_a 400 tau_b -400
```

读旧表会把 250 当分数（被 `max_torque = 1000` 截成满幅 → `-1000`）；读需求表则是
增益 400 × 需求 1.0 = 400。实测 **-400**，证明需求通道读的是新缓冲。

## 5. 文档路由端到端实测（生产入口 `simulation.run_request`）

用真实整车夹具（`tests/vehicle/test_native_vehicle.py` 的 `_positioned_vehicle(_vehicle())`）
的模型文档与 case 文档，把 `brake_torque` role 改名成 `brake_pressure` 后提交：

| 场景 | 实测结果 |
|---|---|
| 原样（baseline） | `success` |
| `brake_torque` → `brake_pressure`（同 payload、同数值） | **`success`** —— 归一化 role 可达且不报「unknown blob role」 |
| 同通道同时声明两制 | **拒绝**：`case document: role brake_torque on tire front_left states a unit its channel was already given in the other system; a wheel's torque is stated in newton-metres or as a normalized demand, not both` |
| 需求值 1.5（越界） | **拒绝**：`case document: role brake_pressure on tire front_left must be within [0, 1]; got 1.500000` |

**注意（本行范围边界）**：`vehicle_dynamic` 家族已接上两个新 role；
`axle_dynamic` 家族的 role 表未动（实测其提交含 `brake_pressure` 时按名拒绝
`case document: unknown blob role brake_pressure`），那是 p2-09 的装配/product 路径接线范围。

## 6. 诚实登记（未做/未声称）

- **未**验收 rotor inertia / damping / reference quaternion 已参与本构：
  `modeling/primitives/elements.py:637-643` 与 `templates/builtin.py:630-645` 明说内核力律不读它们。
- **未**声称 forward/reverse signed throttle parity：内核力律仍是按相对角速度反向的
  resistance law（`cpp/src/element/anti_roll.cpp:104-108`），drive 侧负油门的完整签名语义未实现。
- `throttle_demand` 的 role 与范围校验已落地并有越界拒绝路径，但**整车端到端只实测了
  `brake_pressure`**（整车夹具声明制动、未声明驱动）；`throttle_demand` 的读数
  由 `dump_input_isolation` 在采样层覆盖。
- 本行**未**改 `axle_dynamic` 家族 role 表、未改任何 Python 生产代码、未改任何基线。
