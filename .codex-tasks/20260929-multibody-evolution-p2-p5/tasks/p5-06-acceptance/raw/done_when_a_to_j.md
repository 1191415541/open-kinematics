# Done-When (a)–(j) 逐条实跑

依据 `EPIC.md:291-314`。每条给命令、退出码、依据的 `EPIC.md` 行号。
所有命令由主代理本人执行，未采信子任务自报（`EPIC.md:289`）。
「满足」= 该条全部子项实跑通过；「未完全满足」= 有子项实证不达标，如实标注。

---

## (a) 力矩内建 — `EPIC.md:292-295` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 旧路径隔离（裁决 `ddc3f952` 的「声明分支隔离 + 数据流隔离」） | `grep -n "_build_wheel_torque_signals" src/.../preparation/vehicle_dynamic.py` | 0 | 定义 `:1966`、调用 `:342`；紧接 `:345-351` 对**有 demand 信号的轮**从 N·m 表 `pop` 掉——旧表对这些轮不再进内核 |
| 静止/倒车/抱死三态求值断言 | `pytest tests/subsystems/test_torque_element_wiring.py -q` | 0 | 通过 |
| 力矩时程一致 | 同上 + `pytest tests/cases/test_rotational_torque_document.py -q` | 0 | 通过 |
| `front_brake_bias` 在 opt-in 路径读取次数 0 | `grep -rn "front_brake_bias" src/.../subsystems/*.py` | 0 | 无读取；仅 `brake.py:52`、`torque_elements.py:423` 的退役说明 |
| 非零响应独立夹具（p2-11） | `pytest tests/subsystems/test_torque_element_wiring.py -q` | 0 | code10 rows 42，首样本两端范数 1.0 N·m（rel=1e-12），逐样本 atol=0；对照组 0 行 |

判据修订见裁决 `ddc3f952`：原「两符号 grep 全仓无命中」与「8 个冻结用例逐位一致」不可
同时成立，改为**声明分支隔离 + 数据流隔离**，已同步 `SUBTASKS.csv`、p2-05 `SPEC.md`、
`EPIC.md` Done-When (a)。

## (b) 转向通道 — `EPIC.md:296-299` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 两通道总成装配并跑通一次 | `pytest tests/preparation/test_steering_channels.py -q` | 0 | 8 passed；`steering_output.shape (5, 2, 4)` |
| 阿克曼 / 4WS / 多轴随动各有断言 | `pytest tests/preparation/test_steering_allocator.py -q` | 0 | 24 passed |
| 相同方向盘输入下各通道转角符合分配律 | 同上 | 0 | `direct`→`[2e-05,2e-05]`；`four_wheel_steer` 低速→`[2e-05,-5e-06]`（比值恰 −0.25） |
| 后轮转向限制两层均解除 | `pytest tests/schema/test_vehicle_steering_channels.py -q` | 0 | 14 passed |
| `steering` 必填单例三项逐项不变 | 同上 | 0 | 12 键 dump / 11835 B / canonical_hash `75cb96f5…3fd9` 逐字节未变 |
| 附加通道经 `exclude=True` 声明 | 同上 | 0 | `steering_channels`/`allocation_law` 不在 `model_dump` 输出中 |

判据修订见裁决 `1331b13d`（方案 B）。

**契约反转已登记**（裁决 `442ffad6` 点 1、2）：p2-06 把 actuator 寻址从既有字面量
`"steering_input"` 改为 `steering_spec.channel_name`（默认 `"front_rack"`）。新契约是所有
actuator 以声明的 `channel_name` 寻址。
`tests/adams/test_full_vehicle_model.py::test_source_prescribed_steering_reports_rate_in_actuator_coordinates`
改调 `model.steering.channel_name`，并补 `assert result.steering_names == (model.steering.channel_name,)`
把新契约显式锁死；**数值断言未放宽**（1 passed）。

**未达成项如实登记**（p2-06 自报，本行复核一致）：
(1) 4WS 高速端未跑求解器——整车非零初速时内核报 `status 7: initial velocity violates
velocity constraints`；(2) `ackermann`/`multi_axle_follow` 未在整车路径落地。
这两项**不属 (b) 的判据**（(b) 要求「三种分配律各有断言」，分配器层已满足），故 (b) 判满足，
但事实在此登记。

## (c) 通用运动学 — `EPIC.md:300-302` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 四种构型各有滚转中心断言 | `pytest tests/physics/test_roll_centres_by_topology.py -q` | 0 | 通过（双叉臂 / 5 连杆 / 麦弗逊 / 扭梁） |
| 新引擎路径无硬点名称嗅探 | `pytest tests/vehicle/test_screw_kinematics.py -q` | 0 | 通过 |
| 瞬轴在已知解析解构型上验证 | 同上 | 0 | 通过 |
| 滚转中心高由侧倾反力虚功导数矩阵解算 | `pytest tests/physics -q` | 0 | 通过 |

（路由 `pytest tests/physics packages/suspension_multibody/tests/vehicle/test_screw_kinematics.py -q`
合并：**37 passed**。）

## (d) 广义静平衡 — `EPIC.md:303-305` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 3 轴（6 点）跑通 | `pytest tests/physics/test_static_loads.py -q` | 0 | 通过 |
| 可解数值例（载荷相容、rank(A)=1=N、解唯一） | 同上 | 0 | 通过 |
| 不可解报错例（残差超容差） | 同上 | 0 | 通过 |
| 4 轮与改造前逐位一致（零回归硬门） | 同上 | 0 | 通过 |

## (e) ARB 独立 — `EPIC.md:306-307` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| ARB 插到双叉臂下臂与麦弗逊减振筒外筒 | `pytest tests/subsystems/test_anti_roll_bar_subsystem.py tests/subsystems/test_arb_mount_ports.py -q` | 0 | 通过 |
| `upright_L`/`upright_R` 在 ARB 路径无命中 | `grep -rn 'upright_L\|upright_R' src/.../anti_roll_bar.py` | 1 | **0 命中** |

（路由含力矩元两文件的合并：**30 passed**。）

## (f) 轮端统一（独立复验阶段一 04）— `EPIC.md:75`、`:93`、`:271` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 单轴与整车引用同一份 wheel 子系统文件 | `grep -rln '^role = "wheel"' src/suspension_multibody/` | 0 | 全仓**唯一**命中 `subsystems/wheel.py`——两侧只可能消费这一份 |
| 该文件内容指纹 | `sha256sum src/.../subsystems/wheel.py` | 0 | `6313b9c8898f8d4b2ec0b35539d45f42c2c3c00bfdb6ddd3b9d01ebda3091729` |
| `VerticalTireElement` 在装配路径无命中 | `grep -c VerticalTireElement src/.../assembler.py` | 1 | **0** |
| `assembler.py` 的 isinstance 过滤不存在 | `grep -c isinstance src/.../assembler.py` | 1 | **0** |

**未看阶段一自报**：判据是「全仓只有一个 `role = "wheel"` 的子系统文件」这一结构性事实，
单轴与整车都按 role 解析到同一文件，因此不可能存在第二份实现。
（`element_build.py` 的构建性引用与 `rig_link.py` 的宿主属阶段一 05 范围，不在 (f) 判据内。）

## (g) API — `EPIC.md:95`、`:310` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| `simulate(assembly_document, case_document)` 端到端 | `pytest tests/api/test_simulate.py -q` | 0 | **18 passed** |
| `FrontAxleModel` 历史调用者全部可用 | `pytest tests/api tests/authoring -q` | 0 | 通过 |
| 三个公共 API 门禁全绿 | `legacy_surface_gate.py --check` | 0 | findings 0 |
| | `check_composable_release.py --skip-isolation` | 0 | 3 PASS |
| | `pytest tests/architecture -q` | 0 | 147 passed |

## (h) 总线与闭环 — `EPIC.md:97`、`:308-309` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 总线双向读写有断言 | `pytest tests/api/test_signal_bus.py -q` | 0 | **13 passed** |
| 测点可与结果文档逐项对照 | 同上 | 0 | 通过 |
| 执行器输入改变同次仿真状态轨迹 | 同上 | 0 | 前后两次运行轨迹对照 |
| **ABS 实际反馈闭环 + 同次运行三段数值记录** | `pytest tests/cases/test_abs_closed_loop.py -q` | 0 | **6 passed** |
| ABI 第二次变更已按 D2 裁决 | `pytest tests/architecture/test_kernel_abi_version_single_source.py -q` | 0 | 绿 |
| FMI 产物可被仓库外独立脚本加载并步进 | 仓库外 `ctypes`+`zipfile` 脚本 | 0 | 轨迹断言通过（见 `acceptance.md` 第 3 节） |

**闭环三段可读数值**：`controller_output` 结果块（宽 4）由内核在**同一次运行**内写出：
`[0]measured_slip`（状态）、`[1]target_slip`（控制目标）、`[2]control_demand`（执行器需求）、
`[3]driver_demand`（驾驶员需求）。控制律 `authority = clamp(1 + gain*(target - |slip|), 0, 1)`、
`demand = driver_demand * authority`，**符号与形式均经实测修正**（原稿正反馈使 t=0.05 的
Newton 不收敛）。载体为自建滚动轮试验台（既有四个夹具在全零初速下均无法体现滑移收敛）。
**开环回放不算**，故载体是实际闭环。

**D2 终裁**（裁决 `a8312da6`）：不需要内核单步接口；闭环走内核力元求值路径；
ABI 不变更。故「ABI 第二次变更」未发生——是裁决结论，非登记缺口。

## (i) 零回归 — `EPIC.md:310` → **未完全满足**（skip 计数增长，见下）

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| `kc_baseline` 逐字节未变 | `git status --short -- packages/suspension_multibody/tests/data/` | 0 | 空 |
| `dynamic_hash_baseline` 逐字节未变 | `dynamic_hash_sentinel.py --check` | 0 | 26 artifact；combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（等于冻结值） |
| 快速集全绿 | `pytest tests --ignore=adams --ignore=architecture --ignore=cases -q` | 0 | 全绿 |
| `tests/architecture` 全绿 | `pytest tests/architecture -q` | 0 | 147 passed |
| 数值门三项全绿 | sentinel / case_parity（无参数）/ kc_perf | 0/0/0 | 全绿 |
| 全量回归全绿 | `pytest packages/suspension_multibody/tests -q` | 0 | **1700 passed, 47 skipped, 1 xfailed, 0 failed, 0 errors** |
| **无新增 skip/xfail** | 对照基线 1 skipped / 1 xfailed | — | **xfail 1 = 基线**；**skip 1 → 47（+46）** |

**不达标的具体项**：skip 从 1 增至 47。全部 46 条新增来自 `artifacts/` 目录被误删
（缺陷 C，见 `acceptance.md` 0.3）——这些用例用
`skipif(not <dir>.is_dir(), ...)` 守卫，测试代码本身未改、未新增 skip 语句，但**计数确已增长**，
按 `EPIC.md:233` 与本节 (i) 的字面要求判定为**不满足**，逐条登记于 `skip_register.txt`。

数值门与冻结基线**不受影响**：sentinel 自己跑求解并写
`artifacts/axle-dynamics-acceptance`，产出的 26 artifact 哈希与冻结值一致；
`case_parity_check.py` 自足（`vehicle_dynamic` family 走 `_vehicle_diagnostics_digest`，
不经 adams 参考层）。

## (j) ABI — `EPIC.md:311-312` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 本 Epic 的 ABI 变更全部登记 | `grep -n kAxleKernelAbiVersion src/.../mb_config/version.hpp` | 0 | `:33 = 17`（p2-02 登记 16/31/1 → 17/32/1） |
| 单一真源门绿 | `pytest tests/architecture/test_kernel_abi_version_single_source.py -q` | 0 | 绿 |
| 车辆/核心常量同步 | `grep -n "kVehicleKernelAbiVersion\|kCoreKernelAbiVersion"` | 0 | `= 32`、`= 1` |

---

## 汇总

| 条 | 结论 |
|---|---|
| (a) 力矩内建 | 满足 |
| (b) 转向通道 | 满足（契约反转已登记） |
| (c) 通用运动学 | 满足 |
| (d) 广义静平衡 | 满足 |
| (e) ARB 独立 | 满足 |
| (f) 轮端统一 | 满足（独立复验，未看自报） |
| (g) API | 满足 |
| (h) 总线与闭环 | 满足 |
| (i) 零回归 | **未完全满足**：skip 1→47，全部源于 `artifacts/` 误删，本机不可恢复 |
| (j) ABI | 满足 |

**9/10 满足，(i) 有一条子项实证不达标并已逐条登记。**
