# Done-When (a)–(j) 逐条实跑（终态，2026-10-03）

依据 `EPIC.md:291-314`。每条给命令、退出码、依据的 `EPIC.md` 行号。
所有命令由主代理本人执行，未采信子任务自报（`EPIC.md:289`）。

**本文件是终态版**：此前版本按复核裁决 `8ab15196` 逐条更正过（(a) 调用隔离、
(f) 证据对象、(h) 两处证据不足、(i) skip 计数）。本次在 (a)(f)(h) 的缺口补齐、
p5-04 收敛判据达成之后重写。

结论摘要：(a)–(j) **十条全部满足**；其中 (a)(f)(h) 的达成方式与证据对象在本版写明。

---

## (a) 力矩内建 — `EPIC.md:292-295` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 旧 helper **只在** none 分支被调用 | `grep -n "_build_wheel_torque_signals\|_build_torque_and_demand_signals" src/.../preparation/vehicle_dynamic.py` | 0 | 定义 `:1957`；唯一**调用**在 `:2073`，位于 `_build_torque_and_demand_signals` 内 `if declared == "none":` 分支之下（函数定义 `:2033`，`declared` 读自 `:2071`）。生产路径 `:343` 调的是新的 `_build_torque_and_demand_signals`，不是旧 helper |
| opt-in 路径旧表行数为 0 | `pytest tests/cases/test_vehicle_dynamic_contract.py -q` | 0 | 通过；`_build_torque_and_demand_signals` 的 opt-in 分支 `return {}, {}, wheel_demand, brake_demand`（`:2089`），旧表**根本不构造** |
| 声明会被舍弃的显式表按名拒绝 | 同上 | 0 | `:2076-2087` 对 `case.wheel_drive_torque`/`wheel_brake_torque` 抛 `ValueError`，消息点名 `torque_demand` |
| 静止/倒车/抱死三态求值断言 | `pytest tests/subsystems/test_torque_element_wiring.py -q` | 0 | 通过 |
| `front_brake_bias` 在 opt-in 路径读取次数 0 | `grep -rn "front_brake_bias" src/.../subsystems/*.py` | 0 | 无读取；仅 `brake.py:52`、`torque_elements.py:423` 的退役说明 |
| 非零响应独立夹具（p2-11） | `pytest tests/subsystems/test_torque_element_wiring.py -q` | 0 | code-10 rows 42，首样本两端范数 1.0 N·m（rel=1e-12），逐样本 `atol=0`；对照组 0 行 |

**更正说明（裁决 `8ab15196` 发现 1）**：此前版本把「事后 `pop` 丢弃」写成「声明分支隔离」。
那不是调用隔离——旧 helper 仍在每次 opt-in 运行中被调用，只是结果被丢弃。本次实测
`_build_torque_and_demand_signals` 的形态是真正的**条件调用**（`if declared == "none"` 之外
不调旧 helper、也不构造旧表），并**新增 3 条测试**锁定。p2-05 证据文件里那段**从未存在**的
`declared_demand` 分支已更正为实际形态，痕迹保留。

判据修订见裁决 `ddc3f952`（「两符号 grep 全仓无命中」与「8 个冻结用例逐位一致」不可同时成立）。

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
`"steering_input"` 改为 `steering_spec.channel_name`（默认 `"front_rack"`）。
`tests/adams/test_full_vehicle_model.py::test_source_prescribed_steering_reports_rate_in_actuator_coordinates`
改调 `model.steering.channel_name` 并补 `assert result.steering_names == (model.steering.channel_name,)`
把新契约显式锁死；**数值断言未放宽**。

**p5-06 期间修掉的一个真实数值缺陷**（裁决 `442ffad6` 点 3）：`scripts/full_vehicle_correlation.py`
按名字猜物理类型，p2-06 后 prescribed rotation 也叫 `front_rack`，角度被按 m/rad 误缩放
**5.7296 倍**（恰为齿条比）。已改为按声明的 `actuator_mode` 决定换算、按声明的 `channel_name`
取输出，两个调用点同步（提交 `7e9f2dd`）。

**未达成项如实登记**（p2-06 自报，本行复核一致）：(1) 4WS 高速端未跑求解器——整车非零初速时
内核报 `status 7`；(2) `ackermann`/`multi_axle_follow` 未在整车路径落地。这两项**不属 (b) 判据**
（(b) 要求「三种分配律各有断言」，分配器层已满足），故 (b) 判满足。

## (c) 通用运动学 — `EPIC.md:300-302` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 四种构型各有滚转中心断言 | `pytest tests/physics/test_roll_centres_by_topology.py -q` | 0 | 通过（双叉臂 / 5 连杆 / 麦弗逊 / 扭梁） |
| 新引擎路径无硬点名称嗅探 | `pytest tests/vehicle/test_screw_kinematics.py -q` | 0 | 通过 |
| 瞬轴在已知解析解构型上验证 | 同上 | 0 | 通过 |
| 滚转中心高由侧倾反力虚功导数矩阵解算 | `pytest tests/physics -q` | 0 | 通过 |

（合并路由：**37 passed**。）

## (d) 广义静平衡 — `EPIC.md:303-305` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 3 轴（6 点）跑通 | `pytest tests/physics/test_static_loads.py -q` | 0 | 通过 |
| 可解数值例（载荷相容、rank(A)=1=N、解唯一） | 同上 | 0 | 通过 |
| 不可解报错例（残差超容差） | 同上 | 0 | 通过 |
| 4 轮与改造前逐位一致（零回归硬门） | 同上 | 0 | 通过 |
| 报表通道按安装角色动态注册 | `grep -n "normal_load_axle_{placement}" src/.../outputs/builtin.py src/.../report/wheel_loads.py` | 0 | `outputs/builtin.py:401`、`report/wheel_loads.py:156` 均有该 f-string 通道名；由 `pytest tests/physics/test_static_loads.py` 与 `tests/report` 覆盖 |

**补证（裁决 `8ab15196` 发现 5）**：上一版 (d) 漏了 G4 的「报表通道按安装角色动态注册」
子项。本版补上其代码位置与测试覆盖。

**再补证（2026-10-03，收口轮实跑）**：动态通道子项不止于「代码位置存在」，有实跑断言：
`pytest tests/metrics/test_placement_channels.py -q` → **7 passed**（exit 0），其中
`test_the_third_axle_of_a_six_wheel_run_is_a_real_placement_not_a_rounding`
断言纵向加速度下 `normal_load_axle_front` 下降、`normal_load_axle_rear` 上升、
`load_transfer_front_minus_rear < 0`（即中间通道是**中轴自己的总量**而非邻轴副本）。
故 (d) 的 G4 子项由**实跑断言**覆盖，不是仅靠源码确认。

## (e) ARB 独立 — `EPIC.md:306-307` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| ARB 插到双叉臂下臂与麦弗逊减振筒外筒 | `pytest tests/subsystems/test_anti_roll_bar_subsystem.py tests/subsystems/test_arb_mount_ports.py -q` | 0 | 通过 |
| `upright_L`/`upright_R` 在 ARB 路径无命中 | `grep -rn 'upright_L\|upright_R' src/.../anti_roll_bar.py` | 1 | **0 命中** |

（合并路由：**30 passed**。）

## (f) 轮端统一（独立复验阶段一 04）— `EPIC.md:75`、`:93`、`:271` → **满足**

判据原文要求「比对**读到的文件路径与内容指纹**，不是比对代码注释」。

| 子项 | 证据 | 实测结果 |
|---|---|---|
| 两侧读到的**同一个** wheel 子系统文档 | `raw/dw_f_wheel_document.txt` | 单轴路由与整车路由 `SubsystemDocument.load` 实际打开的文档路径**相同**、sha256 **相同** = `7510feade8e993e74df8d15e670d6e0d9ddb78043e168204fd737ab5d3f34d7f` |
| 该文档确实被装配消费 | 同上 | probe 体名 `probe_wheel_L`/`probe_wheel_R` 到达两侧装配产物；`neither side used the built-in` = True |
| `VerticalTireElement` 在装配路径无命中 | `grep -c VerticalTireElement src/.../assembler.py` | 0 |
| `assembler.py` 的 isinstance 过滤不存在 | `grep -c isinstance src/.../assembler.py` | 0 |

**更正说明（裁决 `8ab15196` 发现 2）**：上一版用的是「全仓只有一个 `role = "wheel"` 的实现文件」
这一结构性论证——那证明不了两侧读到**同一份文档**，因为同一实现可以实例化不同模板
（`subsystems/wheel.py:57` 的 `_requested()` 从 `context.request.wheel_template` 取模板名）。
本次改用**运行期实探**：在同一项目内分别驱动单轴与整车两条入口，追踪
`SubsystemDocument.load` 真正打开的文件路径与内容指纹。证据落 `raw/dw_f_wheel_document.txt`。

## (g) API — `EPIC.md:95`、`:310` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| `simulate(assembly_document, case_document)` 端到端 | `pytest tests/api/test_simulate.py -q` | 0 | **18 passed** |
| `FrontAxleModel` 历史调用者全部可用 | `pytest tests/api tests/authoring -q` | 0 | 通过 |
| 三个公共 API 门禁全绿 | `legacy_surface_gate.py --check` | 0 | findings **0** |
| | `check_composable_release.py --skip-isolation` | 0 | **3 PASS** |
| | `pytest tests/architecture -q` | 0 | **147 passed** |

## (h) 总线与闭环 — `EPIC.md:97`、`:308-309` → **满足**

分三个子项，逐项给独立证据。

### (h-1) 总线双向读写 + 执行器写入改变同次仿真轨迹

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 总线读测点并与结果文档逐项对照 | `pytest tests/api/test_signal_bus.py -q` | 0 | **16 passed** |
| 总线写执行器输入 | 同上 | 0 | 通过 |
| **写入确实改变同次仿真轨迹** | 同上 | 0 | **2 条轨迹对照**：仅经 `bus.write` 改一项 → 重提交 → `max|state delta| > 1e-6` |

**更正说明（裁决 `8ab15196` 发现 3a，并由裁决 `003e00a0` 判「阻断解除」）**：上一版的
`test_signal_bus.py` **一次求解都没有**（`grep -c "run_request\|simulate\|run_case"` = 0），
13 个用例全是「读文档数字 / 写文档返回新对象」。该缺口是**真缺陷**，已在提交 `bd8d84c` 修掉：

- `variable_damping_L` 的写入原先直接 `BusError`——`elements` 是**数组**，而原 path 写的是
  嵌套 mapping；
- `motor_torque_FL` 写入成功但 `max|state delta| = 0.000000e+00`——内核不读那个键。

修法是给 `ActuatorChannel` 增加 `document`(`"model"`/`"case"`) 声明与二选一定位方式：
`element`+`parameters`（模型文档 `elements` 数组内**按名**定位并改 `parameters`），或
`role`+`entity`+`component`（case 文档 `blobs` 描述符所指**字节区间**，按 `offset`/`length`
写 float64）。`variable_damping_L` 同时设 `compression_damping`+`rebound_damping`——内核
`element/directional.cpp:157` 按相对速率**符号**二选一，只设一个则另一方向无响应（实测差 0.0）。
`motor_torque_FL` 只写 wrench 行 `3:6` 力矩列。

新增的 2 条轨迹对照测试**鉴别力已验证**：禁用 blob 写入后立即失败。
证据落 `raw/dw_h_bus_write.txt`。

### (h-2) ABS 实际反馈闭环 + 同次运行三段数值记录

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 三段链同一次运行可读 | `pytest tests/cases/test_abs_closed_loop.py -q` | 0 | **7 passed** |
| **被测量收敛到目标**（固定目标绝对误差） | 同上 | 0 | ON 尾段 `mean(e)=0.0180`、`max(e)=0.0180`（`ptp=2e-6`，已安定）；OFF `mean(e)=0.2132`；ratio `0.0843` |
| 需求调制在工作 | 同上 | 0 | 同 driver 信号下 target 0.15/0.30 的安定 demand = 0.2536/0.4611（差 0.2075） |
| 非零执行器力矩、两端等大反向 | 同上 | 0 | code-10 每样本 2 行，`atol=0`；幅值 `0 < max <= 400` |
| **独立复核** | `explorer 8d135895` 自建脚本复算 | 0 | ON `mean(e)=0.017964` / `max(e)=0.017965`；OFF `mean(e)=0.213156`；ratio `0.084274`——与本行读数吻合 |

**更正说明（裁决 `8ab15196` 发现 3b → `003e00a0` 判「现有数字不通过」）**：上一版的收敛断言
只断言「目标移动、滑移同向跟随」，且措辞「收敛到目标」与实际断言不符。`003e00a0` 明确
**不接受仅以 ON 误差小于 OFF 代替收敛**，并给出预置验收线。本版按预置线做成**绝对误差**断言，
其构造过程、参数由来、读数与鉴别力实验全部落 `raw/p504_convergence.md`。

**载体是本次达成的关键**：`003e00a0` 要求「先让被测量收敛为真」。实测发现两个必须修的真缺陷
（均非「换 target 挑选通过值」）：

1. **轮胎接触帧挂在自转的车轮上**：`kernel_model_accessors.cpp:33-35` 的 `tire_frame_body()`
   在 `tire.frame_body < 0` 时回退到 `tire.body`；而 `forward`（`tire/assemble.cpp:162`）与
   `loaded_radius`/`delta`（`:197-232`）都在这个帧里。车轮自转时 `forward` 随之翻滚、滑移每半圈
   变号。轴族此前未声明 `frame_body`，整车族早已声明（`preparation/vehicle_dynamic.py:1162-1209`）。
2. **开环滑移必须越过设定值**：`element/anti_roll.cpp:243-251` 的 `authority` 在
   `|slip| <= target` 时**恒为 1**，控制器只能削减。开环平衡若落在 target 以下，ON 与 OFF
   逐位相同（实测 `np.array_equal == True`），任何「ON 好于 OFF」的断言都会在什么都没测的情况下通过。

修法是给出**有效工况**（载体经 prismatic 前进 + 车轮经 revolute 自转 + Fiala 轮胎 + 接触帧设为
载体），参数全部为既有声明字段（`frame_body`/`relax`/`controller_gain`/`target_slip`），
**未改内核、未改 ABI、未新增旋钮**。

### (h-3) FMI 产物可被仓库外独立脚本加载并步进

| 子项 | 证据 | 实测结果 |
|---|---|---|
| 仓库外 `ctypes`+`zipfile` 脚本 | `$PI_SCRATCH_DIR/p505b/outside/validate_fmu.py` | `fmi2GetVersion()==2.0`；`EXIT=0` |
| **输入影响输出的轨迹断言** | 同上 | 输入 `0.0→0.9` 使纵向滑移时程逐样本最大差 **2.188280**；同一次运行内滑移极差 **3.933832** |
| 输入方向不是自称 | 同上 | 对输出调 `fmi2SetReal` 被拒 |
| **输入的时间因果性** | `pytest tests/api/test_fmu_export.py -q` | **17 passed**（含新增因果性测试） |

**更正说明（裁决 `003e00a0` 判「阻断解除」，修复见提交 `bd8d84c`）**：`fmi2SetReal` 原先把新值
写入**全部样本**，即改写输入的历史。批式 ABI 下这会让「时钟已经走过的时刻」被追溯修改，
违反了 Co-Simulation 的因果性。修法：从**首个严格晚于时钟的网格节点**起写
（`first = floor(position)+1`），保留时钟之前的输入历史。

新增的因果性测试：在样本 60 设输入后**回读** `[0,60)` 必须与未改动运行**逐位一致**、
`[60,201)` 必须改变；**鉴别力已验证**（还原「写全部样本」后立即失败，回读值 `-1.9798 ≠ -1.9932`）。
证据落 `raw/dw_h_fmu_causality.txt`。

**能力边界如实登记（裁决 `8ab15196` 发现 6b）**：内核 ABI 是**批式**的（一次提交跑完整个时间
网格），故「接续先前状态」在批式 ABI 下**不可能**实现。这不声称「加载并步进」等价于标准的
逐步 Co-Simulation；D4 范围内的边界止于此。

### ABI

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| ABI 第二次变更已按 D2 裁决 | `pytest tests/architecture/test_kernel_abi_version_single_source.py -q` | 0 | 绿；`mb_config/version.hpp:33` = **17**（p2-02 登记 16/31/1 → 17/32/1） |

**D2 终裁**（裁决 `a8312da6`）：**不需要**内核单步接口；闭环走内核力元求值路径（`rotational_torque`
的求值函数同时接收实时 `State` 与 `SampleInput`）；结果块 `controller_output` 是**结果契约扩展**
而非 ABI 变更。故「ABI 第二次变更」**未发生**——是裁决结论，不是登记缺口。

## (i) 零回归 — `EPIC.md:310` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| `kc_baseline` 逐字节未变 | `git status --short -- packages/suspension_multibody/tests/data/` | 0 | **空** |
| 冻结文件最后改动在 Epic 起点之前 | `git log -1 --format="%h %ad %s" -- packages/suspension_multibody/tests/data/` | 0 | `db22f9e`（2026-09-29），早于 Epic 起点 |
| `dynamic_hash_baseline` 逐字节未变 | `dynamic_hash_sentinel.py --check` | 0 | 26 artifact；combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（等于冻结值） |
| p5-01 冻结起点逐字复跑 | p5-01 原脚本 | 0 | 19 键 / 1007 字节 / `72bd9f30…c60b`，三项一致 |
| 快速集全绿 | `pytest tests --ignore=adams --ignore=architecture --ignore=cases -q` | 0 | 全绿 |
| `tests/architecture` 全绿 | `pytest tests/architecture -q` | 0 | 147 passed |
| 数值门三项全绿 | sentinel / case_parity（无参数）/ kc_perf | 0/0/0 | 全绿 |
| 全量回归全绿 | `pytest packages/suspension_multibody/tests -q` | 0 | **1755 passed, 0 skipped, 1 xfailed, 0 failed, 0 errors**（2026-10-03 收口后重跑，1443.82 s） |
| `tests/adams` 单独全绿 | `pytest tests/adams -q` | 0 | **213 passed / 0 skipped** |
| **无新增 skip/xfail** | 对照基线 1 skipped / 1 xfailed | — | **skip 1 → 0**；**xfail 1 = 基线** |

**更正说明（裁决 `8ab15196` 发现 4 与发现 5）**：

- **skip 阻断已解除**：上一版 skip 从 1 增至 47，源于 `artifacts/` 目录被误删（本机不可恢复）。
  本次用本机 Adams 2025.1.1（`G:\MSC.Software\Adams\2025_1_1`）**重建了 `artifacts/` 全部参考
  数据**，skip 由 47 归零（提交 `a19c2a5`）。重建命令与产物落 `raw/artifacts_rebuild.txt`。
- **sentinel 的证明范围要区分**：`dynamic_hash_sentinel.txt` 里同时有 self-convergence 状态与
  acceptance 退出码；哈希一致证明的是**冻结输出未漂移**，**不是**「所有动力学验收成功」。
  本版把这两件事分开写。
- **基线对照的判据**：不以 `git status` 干净作证据（仓库曾重建），而以「最后改动提交落在
  Epic 起点之前」+「p5-01 原脚本逐字复跑三项一致」为准。

## (j) ABI — `EPIC.md:311-312` → **满足**

| 子项 | 命令 | 退出码 | 实测结果 |
|---|---|---|---|
| 本 Epic 的 ABI 变更全部登记 | `grep -n kAxleKernelAbiVersion src/.../mb_config/version.hpp` | 0 | `:33 = 17`（p2-02 登记 16/31/1 → 17/32/1） |
| 单一真源门绿 | `pytest tests/architecture/test_kernel_abi_version_single_source.py -q` | 0 | 绿 |
| 车辆/核心常量同步 | `grep -n "kVehicleKernelAbiVersion\|kCoreKernelAbiVersion"` | 0 | `:43 = 32`、`:46 = 1` |

---

## 汇总

| 条 | 结论 | 达成方式（(a)(f)(h)(i) 的要点） |
|---|---|---|
| (a) 力矩内建 | **满足** | 旧 helper 改真条件调用（只在 `none` 分支），opt-in 分支按名拒绝显式表 |
| (b) 转向通道 | **满足** | 契约反转已登记；p5-06 另修掉一个按名猜类型的静默数值缺陷 |
| (c) 通用运动学 | **满足** | — |
| (d) 广义静平衡 | **满足** | 报表通道按安装角色动态注册：代码位置 + `tests/metrics/test_placement_channels.py` **7 passed** 的实跑断言 |
| (e) ARB 独立 | **满足** | — |
| (f) 轮端统一 | **满足** | 改为**运行期实探**：两侧打开同一文档路径 + 同一 sha256 |
| (g) API | **满足** | — |
| (h) 总线与闭环 | **满足** | 三项各自修完并验证：总线写执行器（轨迹对照）、ABS 固定目标收敛、FMU 输入时间因果性 |
| (i) 零回归 | **满足** | 用本机 Adams 重建 `artifacts/`，skip 47→0；sentinel 逐字节；xfail 未增长 |
| (j) ABI | **满足** | 仍 17/32/1，单一真源门绿 |

**(a)–(j) 十条全部满足。**
