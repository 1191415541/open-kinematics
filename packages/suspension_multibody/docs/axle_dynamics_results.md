# 整轴动力学结果报告与已知限制

本文件记录已实际执行的整轴结果、真实 Adams/Car 启动证据和当前限制；速度门只对记录的
同一导入 manifest、同一初态、输入和物理时长负责，不等同于所有车辆工况的性能承诺。

## 源码与构建

| 项目 | 值 |
| --- | --- |
| 内核源码 | `cpp/axle_dynamics/axle_kernel.cpp`（单翻译单元） |
| C ABI 版本 | 14；整车 ABI 15 |
| 构建入口 | `packages/suspension_multibody/scripts/build_axle_native.py`（Windows 委派 `.ps1`） |
| 编译旗标 | `-std=c++17 -Wall -Wextra -Werror -fno-fast-math -O3 -flto -fopenmp`，静态链接 libgcc/libstdc++ |
| 共享库位置 | `src/suspension_multibody/native/`，随 wheel 一同打包 |
| 构建元数据 | `native/native_build.json`，进入结果 artifact 与 Adams 证据包 |

编译器与版本随构建机记录在 `native_build.json`，不在本文写死。

## 验收运行

```bash
uv run --package suspension-multibody python \
  packages/suspension_multibody/scripts/run_axle_dynamics_acceptance.py \
  --output artifacts/axle-dynamics-acceptance
```

- 模型：`synthetic-twin-strut-axle`。簧载体 + 左右转向节 + 左右车轮，滑柱移动副、
  车轮转动副、含压缩/回弹限位的弹簧减振器、瞬态刷毛轮胎，以及一个把簧载体
  纵向/侧向/俯仰/横摆方向约束住、放开升沉与侧倾的试验台衬套。
- 参数为**合成且已标注**（`parameter_provenance=synthetic_labelled_not_for_formal_adams_accuracy`），
  按 SPEC 不得用于任何正式 Adams 精度结论。
- 工况：`ACCEPTANCE.yaml` 冻结的全部 13 个工况，全部落在 1 kHz 公共输出网格上。

### 逐工况门禁结果

13/13 工况的解算门、能量门与步长收敛门全部通过。能量闭合相对误差（门限为
smooth 5e-3、contact_event 2e-2）：

| 工况 | 能量类 | 闭合误差 | 门限 |
| --- | --- | --- | --- |
| static_equilibrium | smooth | 3.1e-31 | 5e-3 |
| road_step_finite_rise | smooth | 8.3e-4 | 5e-3 |
| road_pulse | smooth | 4.5e-4 | 5e-3 |
| road_sine | smooth | 1.1e-4 | 5e-3 |
| single_wheel_road | smooth | 4.1e-4 | 5e-3 |
| in_phase_road | smooth | 3.8e-4 | 5e-3 |
| opposite_phase_road | smooth | 2.3e-3 | 5e-3 |
| braking | smooth | 3.3e-6 | 5e-3 |
| driving | smooth | 3.3e-6 | 5e-3 |
| lateral_or_steering | smooth | 2.7e-7 | 5e-3 |
| combined_load | smooth | 3.0e-4 | 5e-3 |
| tire_liftoff_and_recontact | contact_event | 1.1e-3 | 2e-2 |
| large_amplitude_high_frequency | smooth | 1.3e-4 | 5e-3 |

全部工况的归一化动力学残差 <= 1.0e-8，位置/速度约束残差与四元数范数误差均在冻结门内，
物理耗散分项非负。每个工况另独立执行半步长复算并通过 `time_convergence` 门。

### 真实 Adams/Car 对标

严格 K 运行使用真实 Adams/Car 2024.1 的 `acar ru-acar`，报告为
`artifacts/real-adams-car/strict-k-final/comparison_report.json`：9 个状态、90 个字段逐点
比较通过，并保留了许可证、启动标记、返回码和结果文件证据。动态墙钟对比使用同一 Adams Car
数据库导入的 manifest 运行真实 Adams/Solver 2024.1 `ru-standard`，不是合成参数或离线替代。

```powershell
$env:HOME='C:\adams_work\analytic_adams_car_home'
$env:MSC_USE_WD='disabled'
$env:SUSPENSION_AXLE_ANALYTIC_THREADS='1'
$env:SUSPENSION_AXLE_PROFILE='1'
uv run --project packages/suspension_multibody python `
  packages/suspension_multibody/scripts/run_real_adams_car_benchmark.py `
  --output artifacts/real-adams-car/benchmark-realtime-final `
  --strict-k-report artifacts/real-adams-car/strict-k-final/comparison_report.json `
  --fixed-step --warmup-runs 1 --runs 3
```

| 项目 | Adams/Solver 2024.1 | 原生整轴 |
| --- | ---: | ---: |
| 中位墙钟 (s) | 4.5148664 | 0.4904139 |
| 物理时长 (s) | 0.5 | 0.5 |
| 墙钟 / 物理时长 | 9.0297328x | 0.9808278x |

原生/Adams 墙钟比为 `0.1086220`，`speed_gate_passed=true`，原生 energy、solver-internal、
time-convergence 三项物理门均通过。完整证据位于
`artifacts/real-adams-car/benchmark-realtime-final/summary.json`。

### 动态时域逐通道对比

本轮先执行严格等价条件审计；只有审计通过且两侧固定步长时间收敛后，才允许逐点比较 33 个
canonical 通道。严格运行使用同一 Adams/Car 导入 manifest、同一道路输入、同一共同初态、
固定步长 `h=0.00025 s` 和同一公共输出网格。证据位于
`artifacts/real-adams-car/benchmark-equivalence-strict-final/road_sine/`。

| 项目 | 结果 |
| --- | ---: |
| 共同初态审计 | `true` |
| Adams 实际积分器 | `HHT`，显式 `alpha=-0.3` |
| 固定步长条件 | `true` |
| 离散积分器完全等价 | `false` |
| 原生时间收敛 | `true` |
| Adams 时间收敛 | `false` |
| 等价条件审计 | `BLOCKED` |
| 33 通道精度比较 | 未执行 |
| 原生/Adams 墙钟比 | `0.0982` |

共同初态由同一次原生静态配平生成并写入两侧模型，配平时间不计入动态墙钟；初态审计的最大
位置误差和最大元素输出误差均通过冻结门限。源参数来自 Adams/Car 数据库，但源数据库
provenance 只作溯源记录，不替代等价条件门。对完整 Adams 源 `step_steer` 的进一步审计还发现：
源 `.adm` 含驱动力 SFORCE `27/29`（状态相关）和制动力 SFORCE `31--34`（其中 `31/34`
为恒定非零力矩）；native 默认配对仍可声明为零输入，但现已支持从源 `.res` 提取逐轮转矩
进行哈希校验后的回放。未使用回放时，该差异仍单独记为 `source_drive_brake_input_equivalence`
阻断项；使用回放只能解除输入通道阻塞，不能替代源控制律。

阻断原因是原生 GGL `rho_inf=0.8` 与 Adams HHT 的离散系数不一致：原生二阶系数为
`alpha_m=0.3333333, alpha_f=0.4444444, gamma=0.6111111, beta=0.3086420`，Adams
HHT `alpha=-0.3` 对应 `alpha_m=0, alpha_f=0.3, gamma=0.8, beta=0.4225`；轮胎一阶状态
格式也没有可核验的 Adams 对应配置。即使按连续问题收敛条件复核，Adams `h` 到 `h/2`
的最大载荷 NRMSE 为 `0.0367213`，超过冻结门限 `0.02`，最大状态 NRMSE 为 `0.0033612`。

因此本轮没有生成可宣称为正式结果的 33 通道 NRMSE、峰值误差或相位滞后；旧的
`benchmark-equivalent-dynamic` 结果不再作为等价比较依据。

### 性能剖析

正式平滑工况的剖析来自同一 benchmark 的 `native_result/manifest.json`：

| 指标 | 数值 |
| --- | ---: |
| residual calls / time | 6945 / 0.1276188 s |
| constraint Jacobian calls / time | 13890 / 0.0429839 s |
| force evaluations / time | 13890 / 0.0151571 s |
| LU factorization calls / time | 53 / 0.0516344 s |
| linear solves / time | 4945 / 0.1880140 s |
| reaction time | 0.0389791 s |
| analytic Jacobian columns | 18656 |
| finite-difference Jacobian columns | 0 |

本轮保留的优化包括显式 `ResidualWorkspace`、解析位姿/速度/刷毛 Jacobian、跨步 LU/Jacobian
缓存、按约束端点遍历反力、接受试探点后的残差复用和确定性的 worker 缓冲。参考本地 Chrono
源码中的统一系统描述、约束/KKT 装配、静态 Newton 增量分析和分层整车子系统编排；没有
复制 Chrono 源码，也没有加入 Chrono 依赖。没有移植其 NSC 约束或 GMRES 路径；当前整轴
混合单位 GGL 系统保留已验证的密集 LU 后端，整车动态 Newton 另使用物理等价的块消元并保留
密集回退。非光滑接触、摩擦、限位和分段阻尼边界保留原分支语义及必要的差分回退。

下表是整轴正式 benchmark，不代表整车性能。此前 40 刚体源整车的短窗口块消元复核
（100 步、`h=0.00005 s`）仅用于验证回退路径一致性：块消元和密集路径均全部接受，状态
最大绝对差约 `1.53e-6`，轮胎输出最大绝对差约 `4.68e-11`；该数据不作为 Adams 速度门。

### 2026-08-29 整车源模型复核

参考 Chrono 的分层车辆子系统和单一全局系统组织，当前 native 将车身、前后悬架、转向、
四个轮端和轮胎装配到一个全局约束系统；Adams 源模型还保留 6 个内部传动刚体，驱动和制动
仍按项目规格作为轮端直接输入。PAC2002
路径已加入选定 `RBX/RBY/RVY` 联合滑移缩放、基于 `QBZ/QCZ/QDZ/QEZ/QHZ/SSZ` 的法向回正力矩，
同时保留松弛状态；它仍不是完整 TIR 力律。源模型
硬转向 5 s 运行生成 501 个输出点、53 个刚体状态，501 个内部步全部接受，最大 Newton
迭代 8 次；最大位置、速度和动力学残量分别为 `9.41e-9`、`9.94e-8`、`1.00e-4`。

此前运行墙钟约 `61.7 s`；最终同一源构型复测墙钟为 `66.40 s`。最终性能计数器记录：
线性分解 `34.01 s`、线性求解 `13.50 s`、残量 `23.49 s`；这些计时存在调用嵌套，不能
相加作为总墙钟。仍有 `114101` 个方向列使用差分回退，这是接触、摩擦、符号函数和分段
限位边界的分支导数，不将非光滑点伪装为经典解析导数。该单次墙钟受机器负载影响，且
尚未与 Adams 冻结同一计时边界，因此不能作为速度门通过证据。

本轮参考 Chrono 的 `ChWheeledVehicle::Synchronize/Advance`、统一系统描述器、约束
Jacobian/KKT 装配和增量 Newton 结构；native 仍按项目已有的单一残差系统实现，没有复制
Chrono 源码，也没有加入 Chrono 依赖。刷毛导数路径跳过未使用的广义力/能量装配，降维
Newton 求解复用线性工作区；接触边界保留必要的差分回退。

## 失败与修正案例

以下问题都是在本任务的测试与验收运行中暴露并修正的，均为可复现的物理或数值缺陷。

1. **陀螺项重复计入**。`external_force_vector` 已把 `-omega x (I omega)` 移到右端，
   瞬态残量又加了一次，自由刚体转动以两倍速率演化；无力矩非对称陀螺的世界系角动量
   漂移约 19% 且不随步长收敛。删除重复项后角动量与动能二阶守恒。
2. **约束 Jacobian 差分精度**。单侧差分 `h=1e-7` 在残量中留下 ~1e-7 地板，球铰单摆在
   `t=0.209 s` 硬失败。改为 `h=1e-6` 中心差分后 2 次 Newton 迭代收敛。
3. **收敛判据量纲不变性**。动力学残量原按绝对值与 `1e-8` 比较，重模型无法满足；
   改为按行内最大项归一化后，2 kg 到 2000 kg 的单摆均 2 次迭代收敛。
4. **一阶格式权重取反**。刷毛内部状态残量把 Jansen 一阶格式的 `alpha_m_z/alpha_f_z`
   按二阶约定使用（权重加在旧值上）。放大矩阵谱半径在全部 `z` 上恒为 1.571，无条件
   不稳定；表现为摩擦饱和后纵向力与滑移**同向**、向系统注入能量，并使普通粘着工况
   Newton 不收敛。修正后饱和力严格反向、粘着段与解析串联刚度振子解相对误差 1e-7。
   该缺陷在 `rho_inf=1` 时不可见，而当时全部物理测试都固定 `rho_inf=1`。
5. **静态配平零影响方向**。转动副车轮的自转角对静力学无影响，配平矩阵必然奇异，
   任何含转动副车轮的正式整轴模型都无法完成静平衡初始化。见 formulation 文档的
   “静力学零影响方向”。
6. **相对输出目录二次拼接**。`_hash_declared_artifacts` 把已相对于工作目录的路径
   再次拼到证据目录下，导致以相对路径运行 runner 时误报缺失 artifact。
7. **工况自身违反冻结分类**。`large_amplitude_high_frequency` 原设计 20 Hz / 25 mm，
   实测触发 32 次接触事件与 192 个离地采样，能量闭合 5.6% 超出 smooth 门；而该工况在
   `ACCEPTANCE.yaml` 中被冻结为 `smooth`（定义为无接触主动集变化）。正弦路面保持接地
   要求峰值加速度小于 `g + static_preload/unsprung_mass`（本模型 93.2 m/s^2，对应
   20 Hz 仅 5.9 mm）。已改为 8 Hz / 12 mm，在满足 smooth 定义下取最大行程；判据未放宽。
8. **`thread_local` 缓冲导致堆破坏（已放弃该优化）**。为消除残量中的反复分配，曾把
   `State next/evaluation` 与多个中间向量改为 `thread_local` 复用缓冲。该版本在
   `road_sine` 上确实更快（2.74 s -> 2.23 s），逐位结果也与基线相同，但压力测试中约
   1/3 的运行触发 Windows `0xc0000374`（堆损坏）。原因是缓冲被跨调用层共享：
   `fill_analytic_jacobian_columns` 经 `state_from_unknown` 写入的 `dy`/`next`，
   正是调用方 `residual` 结果仍在引用的同一对象。逐层加锁式排查后仍有别名路径无法穷尽，
   已整体回退。教训：单次通过的测试不足以判定内存安全，须以重复压力运行确认；
   保留的等效优化改为纯局部作用域（常量引用替代整块拷贝），8/8 压力运行无异常。
9. **“位姿行恒为零”跳过（已放弃该优化）**。速度列与刷毛列不改变任何刚体位姿、
   `a_next` 与 `mu`，故其位姿行差分在数学上为零，据此尝试跳过 `dy_ga` 与
   `M^-1 J^T mu` 的组装。插桩显示这些项并非逐位为零而是 ~1e-32 量级的舍入残留，
   跳过后 `braking` 的输出哈希由 `aed85cc8…` 变为 `b68b54a1…`、内部步数由 1530 变为
   1518——即改变了 Newton 迭代路径。虽然两者都在收敛容差内，但违反“结果不变”的前提，
   已回退。判定依据是逐位哈希，而非测试是否通过。

## 已知限制

### 当前正式验收限制

- **已执行真实 Adams/Car 启动与严格 K 运行**。`acar ru-acar` 的 9 个状态、90 个字段
  逐点比较通过；动态墙钟对比另使用同一数据库导入 manifest 的 Adams/Solver `ru-standard`
  运行。两者的执行证据、`.msg` 和 `.res` 均保留在 `artifacts/real-adams-car/`。
- **当前结论限定于已导入模型和 road_sine 工况**。本工况的严格等价条件审计被阻断，因而
  没有正式动态逐通道精度结论；不能把旧结果中的速度倍率或通过通道外推到所有车辆参数、
  接触状态或道路输入。
- **部分元素无法在 Adams 中逐项表达**，生成器对其直接拒绝而非近似：衬套转动刚度与
  阻尼、衬套预载、非单位参考姿态、稳定杆、共享数值初态、断点数超限的道路输入。
- **刷毛塑性返回映射在 Adams 中只能表达为代数饱和**。因此写入有效性条件：只有当本
  求解器全程报告摩擦利用率严格小于 1 时，该工况才允许比较。

### 求解器自身限制

- **整轴线性代数仍是密集 LU 后端**。上表对应的整轴 Newton 系统通过跨步缓存复用分解
  结果；Chrono 的矩阵无关 GMRES 路径已做过真实工况试验，但混合单位 GGL 系统的收敛
  稳定性不足，因此未保留。整车动态路径已加入块消元；此前 40 刚体源模型的线性系统由约
  `990` 维降为约 `522` 维，密集回退仍可用于逐位结果核验。
- **当前整车性能瓶颈仍是密集 KKT 分解**。早期 1.5 s 源 `step_steer` 基线曾记录墙钟
  `34.24 s`，其中 671 次线性分解累计 `25.36 s`；最新完整源 5.0 s 硬转向运行墙钟约
  `61.7 s`，线性分解约 `33.67 s`，因此当前不能宣称达到 Adams/Car 速度。
- **整轴平滑工作区的 Newton Jacobian 已全部解析**。该正式整轴样本计数为 `18656` 个
  解析列、`0` 个有限差分列、`0` 个非光滑回退列；整车源硬转向样本仍有 `114101` 个
  非光滑方向列使用差分回退。接触、摩擦、限位和分段阻尼切换点没有伪造经典导数，继续
  使用保持原分支语义的回退。
- **整轴样本的主要可见成本**是残量本身（`6945` 次、`0.1276 s`）及其约束 Jacobian/反力
  组装；整车源样本的当前主要成本见上面的 2026-08-29 复核，计时项存在嵌套，不能直接相加
  为总墙钟。后续若要继续扩大模型规模，优先考虑
  稀疏/结构化 KKT 后端和更细粒度的残量临时对象复用，但不能以牺牲物理或精度为代价。
- **并行仅用于仍存在的差分回退列**；平滑正式样本没有差分列，使用单线程解析路径以保持
  确定性。
- **首版不含**柔性体、液压回路、热衰减、材料滞回、关节间隙与黑盒 Adams/Car 轮胎。
- 外倾推力与自旋阻尼仍未实现；PAC2002 的滚动阻力矩和倾覆矩仅在已有 `QSY/QSX` 参数时
  计算，回正力矩仅在已有 `QBZ/QCZ/QDZ/QEZ/QHZ/SSZ` 参数时计算，完整 TIR 参数和 Adams
  接触力律仍未实现。
- 路面首版为平面加时变垂向输入；不接受时间上瞬时的位移跳变。
- 试验台约束以衬套刚度实现，因此簧载体在被约束方向上并非严格刚性固定；
  该刚度是 manifest 的显式物理参数，会进入哈希。

## 相关文件

- 物理模型与数值方法：`axle_dynamics_formulation.md`
- 软件结构与 C ABI：`axle_dynamics_architecture.md`
- 旧实现审查与删除清单：`axle_dynamics_legacy_audit.md`
- 冻结判据：`.codex-tasks/20260819-axle-dynamics-rewrite/ACCEPTANCE.yaml`
- 冻结通道：`src/suspension_multibody/adams/axle_channels.yaml`
