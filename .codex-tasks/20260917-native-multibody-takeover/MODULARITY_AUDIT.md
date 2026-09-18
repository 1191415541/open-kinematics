# C++ 内核模块化审计（2026-09-17）

审计对象：`packages/suspension_kernel/cpp/`（55 个翻译单元、`src` 约 18.9k 行、`include` 约 6.0k 行）。
方法：6 路并行只读审计 —— `mb_static` 深挖、头文件包含图、求解层内聚性、基础层内聚性、
语义层内聚性、跨层反向依赖。全部结论带 file:line。

## 1. 结论摘要

1. **`mb_static` 确实不独立**：它导出 18 个函数，分属 6 类职责，9 项拆分建议见 §3。
2. **更严重的是全内核「名义分层、实际耦合」**：11 个模块的 `functions.hpp` 全是聚合头，
   每个都包含 7~8 个其他模块的头；头图存在非平凡强连通分量
   `{mb_integrator, mb_static, mb_vehicle}`，**不是 DAG**。
   架构文档声称 K6 已删除「一个头看到全内核」的形态（`docs/axle_dynamics_architecture.md:19-21`），与事实不符。
3. **存在 4 条真实的反向依赖边**（不是 include 噪声）：`mb_model→mb_constraint`、
   `mb_suspension→mb_vehicle`、`mb_tire→mb_vehicle`、`mb_vehicle→mb_static`。
4. **多个大文件是「按代码连续段」拆出来的，不是按职责**：`kernel_output.cpp`（5 类输出）、
   `kernel_base.cpp`（配置 + 数值几何 + 四元数 + 曲线）、`vehicle/kernel_registration.cpp`（11 个注册函数跨 6 域）、
   `tire/assemble.cpp`（模型分发 + 力装配 + 输出通道）、`pac2002/law.cpp`（本构 + 限幅 + 力矩 + 状态导数）。
5. **有两处跨模块职责放错位置**：输入插值 `interpolate_input` 定义在 `mb_integrator`，却被
   `static` 与 `output` 调用；运行时/后端配置读取挤在 `mb_base` 里。
6. **分层检查器在仓库中不存在**：架构文档引用的 `check_module_layering.py` 与冻结边集均无实现
   （全仓检索为空），所以上述所有越界都没有门禁在管。

## 2. 证据：模块头包含图（谁包含谁）

`cpp/include/<module>/functions.hpp` 触达的其他模块数：

| 模块头 | 触达其他模块 | 备注 |
|---|---:|---|
| mb_base | 0 | 无外部模块，但**自包含** `mb_base/functions.hpp:21` |
| mb_linalg | 1 | 仅 mb_base |
| mb_model | 1 | 仅 mb_base |
| mb_constraint | 3 | base/linalg/model |
| mb_suspension | 3 | base/model/**vehicle**（反向边） |
| mb_tire_state | 2 | base/model |
| mb_tire | 4 | base/model/tire_state/**vehicle**（反向边） |
| mb_vehicle | 7 | 含 **mb_static**（反向边） |
| mb_integrator | 8 | 触达 8/10 个其他模块 |
| mb_static | 8 | 触达 8/10 个其他模块 |
| mb_output | 6 | 含 integrator |

- 非平凡强连通分量：`{mb_integrator, mb_static, mb_vehicle}`；简单环包括
  `mb_integrator ↔ mb_static`、`mb_static ↔ mb_vehicle`。
- 每个模块头都存在**自包含**（如 `mb_integrator/functions.hpp:39`、`mb_static/functions.hpp:36`）。
- 大量**重复 include**（`mb_static/functions.hpp:41-48` 里同一批头出现两次）。
- 翻译单元层看起来是干净的（48/55 只包含本模块头），耦合全部隐藏在聚合头里 —— 即「越靠上层的头，
  看到的越多」，任何 `.cpp` 只要包含本模块头就自动获得全内核可见性。

## 3. `mb_static` 的职责分类与拆分建议

| 职责类别 | 函数 | 当前位置 |
|---|---|---|
| 残差/Jacobian 组装 | `static_residual`、`static_jacobian` | kernel_static_trim.cpp:16, :350 |
| 约束行提取 | `static_position_constraint_mask/_rows/_jacobian` | kernel_static_trim.cpp:310,324,334 |
| 约束矩阵归一化 | `normalized_static_constraint_matrix` | kernel_static_projection.cpp:13 |
| gauge / 零空间 | `static_rotation_gauge_for_pivot`、`static_rotation_gauge_value`、`static_gauge_coordinates`、`pin_null_pose_directions` | kernel_static_trim.cpp:283,292,265,232 |
| 投影/最小二乘/流形 | `static_tangent_projection`、`project_static_pose`、`solve_static_least_squares`、`static_manifold_relaxation_step` | kernel_static_projection.cpp:68,117,204,296 |
| 接触活动集/预平衡 | `static_global_contact_pretrim`、`static_trim` | kernel_static_contact.cpp:15,165 |
| 接触阈值 | `static_contact_tolerance` | kernel_static_trim.cpp:221 |
| 诊断/审计 | `audit_constraint_system`、`constraint_residual_maxima` | kernel_static_contact.cpp:624、kernel_static_trim.cpp:177 |

混合情况：

- `kernel_static_contact.cpp` 混 4 类：接触预平衡（15-163）、trim 顶层驱动（165-622）、
  活动集/载荷步进（嵌在 trim 内）、约束系统审计（624-658）。**审计函数与求解无关**：
  它只构造 audit_state、查典型 Jacobian 秩、比对中心差分，然后经 `std::string& error` 报错。
- `kernel_static_trim.cpp` 混 5 类：残差（16-220）、接触阈值（221-230）、gauge（232-308）、
  位置约束提取（310-348）、Jacobian（350-542）。文件名与内容不符。
- `kernel_static_projection.cpp` 混 2 类相近职责（投影/最小二乘 + 流形松弛），**耦合最轻**。

拆分建议（保持 `mb_static/functions.hpp` 作为兼容聚合头时，调用方无需改动 —— 实测只有 5 个调用点：
`abi/kernel_abi.cpp:170,214,1027`、`abi/kernel_model_build.cpp:369`、`vehicle/kernel_registration.cpp:407`）：

```text
kernel_static_residual.cpp        static_residual, static_jacobian
kernel_static_constraints.cpp     static_position_constraint_*, normalized_static_constraint_matrix
kernel_static_gauge.cpp           static_*_gauge*, pin_null_pose_directions
kernel_static_projection.cpp      static_tangent_projection, project_static_pose, solve_static_least_squares
kernel_static_manifold.cpp        static_manifold_relaxation_step
kernel_static_contact.cpp         static_global_contact_pretrim（仅接触）
kernel_static_trim.cpp            static_trim（仅顶层编排）
kernel_static_audit.cpp           audit_constraint_system, constraint_residual_maxima
```

依赖方向：`trim → {contact, residual, constraints, gauge, projection}`；
`projection → {constraints, gauge, residual}`；`audit` 不被任何求解路径依赖。

## 4. 其他文件的耦合（按严重程度）

**最严重（一个 TU 多个子域）**

- `output/kernel_output.cpp`（506 行，9 函数）＝ 状态复制 + 转向观测 + 约束力观测 + 能量账本 +
  物理输出 + 性能指标。其中 `kinetic_energy`/`accumulate_energy_step` 是**物理量计算**不是写 buffer；
  `write_constraint_wrenches:174` 重算 `constraint_jacobian`，`write_physics_output:370` 重算轮胎力
  → 输出层在重复执行物理。
  建议拆：`output_state / output_vehicle / output_constraints / output_energy / output_physics / output_performance`。
- `base/kernel_base.cpp`（约 724 行）＝ 有限性工具 + profiling/debug 开关 + 线性后端配置（LU/GMRES/MKL/Pardiso）
  + 车辆动力学开关 + Vec3/Mat3 + 四元数/旋转 + Cardan + 曲线插值。
  建议拆：`numeric_basic / runtime_config / solver_backend_config / geometry_linear / rotation / curves`，
  且**配置读取不应属于 base**。
- `vehicle/kernel_registration.cpp`（约 930 行，11 函数）＝ 转向、驱动坐标、气动、道路、静态姿态量、
  轮胎 frame、轮胎模型、驱动扭矩、弹簧曲线、衬套曲线、衬套旋转坐标。建议按域拆 6 个注册文件 + 一个薄编排。
- `vehicle/layout.cpp`＝ 气动力 + 输出清零 + 外力/重力 + 广义力投影，四件不相关的事，文件名也不符。
- `tire/assemble.cpp`＝ 接触 frame + 模型分发 + 各模型力装配 + 输出/能量通道。
- `tire/pac2002/law.cpp`（约 914 行）＝ capability 查询 + clamp + 峰值力 + 纯滑移 + 组合滑移 +
  回正/倾覆/滚阻力矩 + turn-slip 律 + 状态导数。建议拆 capabilities/limits/pure_slip/combined_slip/moments/turn_slip/derivatives。

**求解层（integrator，3153 行）**

- `kernel_integrator.cpp`（688 行）＝ 初始化（速度校验/加速度初值/初始未知量）+ 轮胎内部状态初始化
  + brush return mapping。其中 `initialize_internal_derivatives` 单函数 **279 行**。
- `kernel_integrator_residual.cpp`：`residual` 单函数 **573 行**。
- `kernel_integrator_newton.cpp`：`fill_analytic_jacobian_columns` **694 行**、`newton_step` **390 行**。
- `kernel_integrator_step.cpp`：`contact_penetrations` 属于事件层，放错文件。
- `kernel_events.cpp`：检测/定位/记录三类职责同 TU。

**已足够内聚、不建议动**

`base/angles.cpp`、`base/dual_geometry.cpp`、`base/kernel_dual_algebra.cpp`、`linalg/kernel_linalg.cpp`、
`model/directional.cpp`、`model/kernel_model_accessors.cpp`、`constraint/kernel_model_constraint.cpp`、
`constraint/registry.cpp`、`suspension/{anti_roll,curves,spring}.cpp`、`tire_state/kernel_tire_state.cpp`、
`tire/common/kinematics.cpp`、`tire/brush/model.cpp`、`fiala/*`、`pac2002/{parameters,turn_slip,contact_mass}.cpp`。

## 5. 真实的反向依赖边（必须消除）

| 边 | 证据 | 性质 | 处置方向 |
|---|---|---|---|
| `mb_model → mb_constraint` | `model/kernel_model_accessors.cpp:12` 使用 `ConstraintType`/`constraint_rows` | 真实 | 约束类型枚举/行数表下沉到共享契约模块 |
| `mb_suspension → mb_vehicle` | `mb_suspension/functions.hpp:27` 使用 `EnergyRates/EnergyStorage` | 真实 | 能量账本类型下沉 |
| `mb_tire → mb_vehicle` | `mb_tire/force_context.hpp:35`、`mb_tire/functions.hpp:41` | 真实 | 同上 |
| `mb_vehicle → mb_static` | `mb_vehicle/functions.hpp:39,55` 使用 `StaticContactOverride` | 真实 | 静态接触覆盖类型下沉；或改为前置声明 + 窄接口头 |

另有两处职责错位（非反向边，但影响模块化）：

- `interpolate_input` 定义在 `mb_integrator`（`kernel_integrator_input.cpp`），被
  `static`（`kernel_static_contact.cpp:171`）与 `output`（`kernel_output.cpp:143,276,357`）调用
  → 应抽成独立低层模块 `mb_input`（输入采样/时间表）。
- 运行时与后端配置挤在 `mb_base/kernel_base.cpp:27-221` → 应抽出 `mb_config`（或由 ABI 注入）。

## 6. 目标模块架构（建议）

规则（建议写成可执行门禁）：

- R1 一个翻译单元只实现一类职责；单函数超过 ~150 行必须把「编排」与「实现」分开。
- R2 模块头只包含其允许依赖层的**类型头**；禁止包含其他模块的 `functions.hpp`；禁止自包含。
- R3 模块依赖图必须是 **DAG**，由检查脚本强制；边集快照入库，只允许按计划收敛。
- R4 跨层共享类型必须放在明确的低层共享模块，不允许「谁先写就归谁」。

目标层次：

```text
L0  mb_base(数值/几何/旋转/曲线)   mb_config(运行时与后端配置)
L1  mb_contracts(共享契约: 约束类型表, 能量账本, 静态接触覆盖, 采样输入类型)   mb_input(输入采样)
L2  mb_model(+road)  mb_constraint  mb_suspension  mb_tire_state  mb_tire*
L3  mb_vehicle
L4  mb_static(residual/jacobian/gauge/projection/contact/trim/audit)
    mb_integrator(events/step/newton/residual/init/internal_state)
L5  mb_output(state/vehicle/constraints/energy/physics/performance)
L6  mb_cases(工况层, 多文件, 多工况族)  →  仅依赖 L4 的求解接口
L7  abi(聚合)
```

## 7. 与现有 Epic 的关系

模块化重构必须**先于** K/C native 化：在头图非 DAG、模块头聚合全内核的前提下新增 `mb_kc_cases`
只会放大混乱；反之，先恢复真实分层，K/C 工况层就是一个干净的新模块。

因此 Epic 改为两阶段：

- **Phase A 模块化重构（行为不变）**：唯一验收标准是「动态输出哈希逐位不变 + K/C 与 Python 参考等价」，
  即纯结构重构，不得改变数值行为。
- **Phase B K/C native 接管**：在干净分层上新增求解能力与工况层。

安全网：先在 child 3 冻结动态哈希与 K/C 快照，再动任何结构；每个重构 child 都必须跑
`dynamic_hash_sentinel --check`，任何哈希漂移立即回退该步。
## 8. 工况层（`mb_cases`）的范围与文件划分

**修正（2026-09-17，用户意见）**：工况层不只是 K/C，还包括整车仿真工况、四柱实验等。
因此它必须是一个**多文件的工况模块**，而不是单一的 K/C 工况模块。

### 8.1 现有工况定义散落在四处（收敛目标）

| 工况族 | 今天的定义位置 |
|---|---|
| 准静态 K/C | `schema/case.py`（`CaseSpec.mode: K\|C`、`RangeSweep`/`ExplicitSweep`、`left_right_mode`）、`analysis/sweeps.py` 的 `KGrid/CGrid`、`analysis/c_mode.py` 的 `LoadPath` |
| 整轴动态 | `schema/dynamic.py`（`DynamicCaseSpec.mode="axle_dynamic"`）、`analysis/axle_quasi_static.py` |
| 整车 K/C 时域 | `schema/dynamic.py`（`mode="vehicle_kc_dynamic"`）、`analysis/vehicle_kc_time_domain.py` |
| 整车动态 | `schema/dynamic.py`（`mode="vehicle_dynamic"`）、`vehicle_dynamics.py` |
| 操稳 handling | `adams/vehicle_acceptance.py: HANDLING_CASES`、`adams/vehicle_handling.py`（`EXPERIMENT_NAME='Low Speed Parking Steer'` :616、`'Open Loop Sine Steer'` :671，手写 DCF） |
| 平顺 ride | `adams/vehicle_acceptance.py: RIDE_CASES`、`adams/vehicle_ride.py`（`four_post` 四柱台 :330/:363/:385/:399、`random_road` :432，手写 DCF/ACF） |
| 路面/激励 | `schema/vehicle.py: RoadSurfaceSpec.kind`（`plane\|sine\|bump\|random_fourier\|four_post`）—— 四柱台已在 schema 里作为路面类型存在 |
| 对标/相关性 | `adams/full_vehicle_mbd_comparison.py`、`adams/vehicle_correlation.py`、`adams/axle_equivalence.py` |

**问题**：同一类工况在多处重复定义。例如四柱台既在 `schema/vehicle.py` 的 `RoadSurfaceSpec.kind` 里，
又在 `adams/vehicle_ride.py` 里以手写 Adams 命令 + ACF 改写的形式重新实现一遍；
`HANDLING_CASES`/`RIDE_CASES` 只存在于 Adams 侧的验收矩阵里，native 侧并不知道这些工况。

### 8.2 目标：工况模块成为工况的单一事实来源

```text
mb_cases（L6，多文件）
├─ case_types.hpp / case_registry.cpp     工况族与种类描述符、注册与分派（薄）
├─ case_identity.cpp                      工况身份、有序网格规范化与哈希
├─ case_validate.cpp                      声明式 CaseSpec 的校验（C++ 侧）
├─ kc_quasi_static.cpp                    K/C：轮跳×齿条网格、六分量载荷路径、single/symmetric/opposite
├─ axle_dynamic.cpp                       整轴动态：路面时程、轮端扭矩、驱动坐标
├─ vehicle_kc.cpp                         整车 K/C 时域
├─ vehicle_dynamic.cpp                    整车动态基座（初速、转向、驱动/制动、路面）
├─ vehicle_handling.cpp                   操稳：驻车转向、开环正弦转向
├─ vehicle_ride.cpp                       平顺：随机路面
├─ four_post.cpp                          四柱台激励（被 ride 与独立四柱实验共用）
├─ comparison.cpp                         对标/相关性工况（与 Adams 证据包对应）
└─ metrics.cpp                            工况级结果指标（K/C 指标、C−K、柔度矩阵提取）
```

依赖方向：`mb_cases → mb_static / mb_integrator`，**不含求解实现**，不进动态 integrator 内部路径。

### 8.3 附带收益：Adams 侧从「重新实现工况」变为「渲染工况」

今天 `adams/vehicle_handling.py`、`adams/vehicle_ride.py` 各自手写 Adams DCF/ACF 与激励函数。
工况模块成为单一事实来源后，Adams 侧应改为**同一份工况定义的渲染器**（case definition → Adams dataset/DCF/ACF），
这与 `CONTEXT.md` 里既有的 **Canonical Equivalence Model**（两侧独立求解、共享同一工况定义）概念一致。

### 8.4 对分层与门禁的影响

- 新增 `mb_cases` 是第 17 个静态库目标：需登记源列表、`add_library`、模块属性循环
  （`CMakeLists.txt:185-197`）、聚合链接 group（`:212-230`，置于 `mb_static` 之前）、
  Release IPO 列表（`:282-283`）。CMake 不 glob（`:7-10`）。
- 工况身份（含有序网格、载荷路径、侧模式、四柱激励定义）必须由 `mb_cases` 产出并写入结果 manifest；
  parity 门与 Adams 门都以该身份做前置校验，禁止 Python 侧临时拼装。