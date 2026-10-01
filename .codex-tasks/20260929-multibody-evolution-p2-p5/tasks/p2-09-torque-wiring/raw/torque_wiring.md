# p2-09 证据：brake/drive 力矩元接入整车装配元素面

> 全部为 2026-10-01 本机实跑/实读结果。来源：`code-reviewer` 裁决 `84c8f264`（登记本行）
> 与 `5e01b75d`（本行的接口与写范围裁决）。
>
> **本行状态：部分交付（未标 DONE）。** 见 §6 的诚实登记——判据中「同一份模型经
> `simulation.run_request` 跑通一次且读回两端等大反向的**力矩响应**」只达成了
> 「等大反向」与「被求值」，**幅值在现有夹具下恒为 0**，原因已定位为拓扑事实而非实现缺陷。

## 1. 缺口（改前，实测）

```
$ compose_vehicle_runtime(model, mode="K").elements   -> 长度 0
$ grep -rn "brake.wheel_torque_element|drive.wheel_torque_element" src/  -> 零命中
```

`subsystems/element_build.py` 的 `element_rows()` 只从 suspension/wheel 取行；brake 与 drive
两个角色的唯一调用者是它们自己的测试。故「模板内建的主动力矩元」在整车产物里不存在。

## 2. 改动清单（逐文件）

| 文件 | 改动 |
|---|---|
| `schema/vehicle.py` | `DrivelineSpec` 新增 `torque_demand: Literal["none","drive","brake","both"]`，默认 `none`，`exclude=True`（与 `brake_mu` 等四字段同规，**不进 `model_dump`**，故 `model_hash` 不变） |
| `subsystems/torque_elements.py` | 重写：`wheel_demand_wheels` 读声明；`_reaction_ports` 每轮端一个反力端口（owner = `runtime.wheel_centers[wheel][0]`）；`rotational_torque_rows` 走 `match_requirements` 配对；新增 `MM_TO_M`（模板槽 N·mm → 内核 SI 的**唯一一次**换算）与 `brake_share`（复刻退役 builder 的「bias 在该轴制轮间均分」） |
| `subsystems/brake.py` / `drive.py` | `torque_parameters` 与 `wheel_torque_element` 接收 `demand_source` / `demand_tire` / `gain_scale` / `reaction_role`；gain 按 `gain_scale` 换算 |
| `subsystems/vehicle_assembly.py` | 新增 `_with_torque_elements(model, runtime)`：装配完成后把行建成元素追加到 `VehicleRuntime.elements`。**默认 `none` 时原样返回**，产物逐项不变 |
| `preparation/vehicle_dynamic.py` | `_NativeVehicleModel.rotational_torques`（本行前述已落位）由新增 `_build_rotational_torques` 填充；`PreparedVehicleRun` 新增 `wheel_demand` / `brake_demand`；新增 `_build_demand_signals`（按声明采样驾驶员信号并做范围校验）；opt-in 时把该轮端从旧 N·m 表移除；`_build_elements` 对 `RotationalTorqueElement` 走 `continue`（该族由 `rotational_torques` 单独发射，避免重复描述） |
| `cases/vehicle_dynamic.py` | 新增 `_rotational_torque_element`；模型文档发射该族（含 `demand_source` / `demand_tire`）；case 文档在旧 role 之外新增 `brake_pressure` / `throttle_demand` |
| `results/element_wrench.py` | 新增 `(10, "rotational_torque", 2)`（该族每元素两端各一行） |
| `results/kc_state.py` | `_CHANNEL_DOCUMENT_TYPES` 新增 `10: ("elements", "rotational_torque")` |
| `tests/subsystems/test_torque_element_wiring.py` | **新增 9 条用例**（本行验收） |
| `tests/results/test_element_wrench.py` | 既有断言更新：码表由 `(1..9)` 改为 `(1..10)` 并补 `10 == "rotational_torque"`（**理由已写在测试 docstring**：新增码一律追加、旧码不移动） |

## 3. 实跑门禁

| 命令 | 结果 |
|---|---|
| `build_suspension_kernel.py` + `build_axle_native.py` | 退出码 0（本行内核改动见 p2-10 的提交） |
| `pytest tests/subsystems/test_torque_element_wiring.py -q` | **9 passed** |
| 快速集（除 adams/architecture/cases） | **1210 passed / 1 xfailed**（起点 1201；+9 为本行新增） |
| `pytest tests/cases tests/vehicle -q` | **185 passed / 1 xfailed** |
| `pytest tests/architecture -q` | **147 passed** |
| `dynamic_hash_sentinel.py --check` | combined sha256 = `fdfd5a6b…eebc9`，**逐字节一致** |
| `case_parity_check.py`（无参数） | **8 families accepted** |
| `kc_perf_gate.py --check` | 在记录预算内 |
| `check_module_layering.py --strict --final` / `legacy_surface_gate.py --check` / `check_composable_release.py --skip-isolation` | 全 0（0 环 / findings 0 / 3 checks passed） |
| `ruff check .` / `ty check .` | 全仓 exit 0 |
| `git status --short -- tests/data/` | **空**（未重录任何基线） |

## 4. 已实测成立的判据

1. **默认路径逐项不变**：`torque_demand == "none"` 时 `elements` 中该族数量为 0，
   case 文档的 role 集合仍是 `{brake_torque, steering_rate, steering_target, wheel_torque}`，
   `dynamic_hash_sentinel` 逐字节一致。（`test_the_default_model_gets_no_torque_element`、
   `test_a_document_that_declares_nothing_is_unchanged`）
2. **元素数量等于声明的轮端数**：`torque_demand="brake"` → 4 个元素（= 该模型的 braked 轮数，
   由模型自身声明读出而非字面量）；`"drive"` 且 `driven_wheels=("rear_left","rear_right")`
   → 2 个；`"both"` → 6 个（4 制动 + 2 驱动，**两个通道互不冲突**）。
3. **两端由端口配对决定，无体名字面量**：`torque_elements` 模块的 AST 字符串常量中不含
   `upright_L` / `upright_R` / `chassis` / `wheel_front_left`（`test_the_two_ends_are_resolved_by_a_port_match_not_by_a_name`）。
4. **单位只换算一次**：模板槽满需求幅值实测 `29000 N·mm`；前轮份额 `0.6/2 = 0.3`；
   换算后 `8.7 N·m`，与 `AxleRotationalTorque.stiffness_n_m_per_rad` / `max_torque_n_m`
   实测值一致（`test_the_gain_is_converted_once_to_the_kernel_units`，期望值在测试内独立推导）。
5. **生产文档路由双向可达**：模型文档含 4 个 `rotational_torque`（带 `demand_source=2`），
   case 文档含 `brake_pressure` 且**不含** `brake_torque`（同轮不重复声明两制）。
6. **内核确实求值了该族**：打开 `element_wrench` 通道后，opt-in 模型有 **8 行 code 10**
   （4 元素 × 2 端；两样本下共 16 行），对照组 **0 行**；每对的力矩向量**等大反向**
   （`first == -second`，`atol=0`）。

## 5. 未达成的判据（未掩盖）

**「读回两端等大反向的力矩响应」中的非零幅值未达成。** 实测：所有尝试下 code-10 行的力矩
列（3:6）恒为 `0.0`（等大反向成立：`0` 与 `-0`）。

**根因（已定位，是拓扑事实而非实现缺陷）**：

- 该夹具的轮端装配出的是 `WeldJoint front_wheel_mount_front_left (front_wheel_hub_L, wheel_front_left)`
  ——轮与轮毂**焊死**，属同一刚性组件。**这不是夹具的选择**：`subsystems/vehicle_parts.py:536-543`
  在 `mount_body` 名字含 `wheel_hub` 时**强制**建 `WeldJoint`（`if "wheel_hub" in mount_body:`），
  与 `WheelSpec.mount_joint_kind` 无关——实测把四个轮的 `mount_joint_kind` 全设为 `"revolute"`
  后，关节仍是 `AxleJoint ... fixed`；
- 力矩元的两端正是 `wheel_front_left`（实车侧）与 `front_wheel_hub_L`（反力侧，来自
  `runtime.wheel_centers[wheel][0]`），二者相对角速度恒为 0；
- 内核力律（p2-08）对 `|rate| <= kEps` 的对施加**零**力偶（这正是「静止对不被舍入误差的符号
  加速」那条设计），且该工况下轮胎滑移亦为 0；
- 焊死的关节把力偶完全吸收，故 `body_state`、`constraint_wrench`、`energy` 三个结果块与
  对照组**逐位相同**（实测 diff 恰为 `0.0`）。

**这一条不能靠换一个 `mount_joint_kind` 绕过**（实测：四个轮全设为 `"revolute"` 后关节仍是
`fixed`，力矩仍为 `0.0`）。真正的根因是**反力体的选择口径**：本行按裁决取
`runtime.wheel_centers[wheel][0]`（该夹具给出 `front_wheel_hub_L`），而**轮毂与轮体之间是
强制焊缝**，故力偶两端同属一个刚性组件，被焊缝完全吸收。实车制动反力应落在**不随轮旋转的
转向节**（`front_upright_L`，`drive_brake.cpp` 的注释亦如此），而转向节与轮毂之间才是真正的
自旋关节。

**这条属于接口口径问题，不是本行能自行改的范围**（改 `_reaction_ports` 的反力体来源会改变
裁决 `5e01b75d` 冻结的契约，也可能影响 `wheel_centers` 的语义）。按契约「不擅自扩范围」，
本行**不自行改口径**，而是：

1. `SUBTASKS.csv` 的 `p2-09` **保持 `TODO`**，不标 `DONE`；
2. 把该阻断项连同两级根因（`vehicle_parts.py:536-543` 的强制焊缝、反力体取自 `wheel_centers`）
   写进本文件与父 `PROGRESS.md`；
3. 请裁决：应否把 `_reaction_ports` 的反力体改为「轮端的不旋转承载件」（需给出**不靠名字**的
   判定方式），或把该判据移交给一行新子任务。

**在裁决落地前，本行不得声称 Done-When 的「力矩响应」已达成。**

## 6. 诚实登记（未做/未声称）

- 未验收 rotor inertia / damping / reference quaternion 参与本构（内核力律不读，见
  `modeling/primitives/elements.py:637-643`、`templates/builtin.py:630-645`）。
- 未声称 forward/reverse signed throttle parity（内核仍是 resistance law）。
- 未改内核 ABI（版本仍 17/32/1）；未改 `mb_config/version.hpp` 与 `kernel/native.py`。
- 未重录任何基线；未新增 skip/xfail。
- 本行改动已在工作树中，**尚未提交**（因判据未全达成，等处置裁决）。
