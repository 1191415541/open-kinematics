# P4a 步骤 1 前置侦察：冻结协议可复用面与真实缺口

> 本文件是步骤 1 的侦察结论。目的是把"设计目标当当前目录"的风险压掉——
> 即先实测**哪些已经存在、哪些完全没有**，再决定第 1 步实现什么。
> 全部结论均带文件:行号，取自实读源码，非设计文档转述。

## 1. 一句话结论

**冻结协议的"数据形态"和"装配机制"已经存在且已经解耦；缺的是三件**：
①`B_w = -dR/dw` 的装配（**全仓库零实现**）、②把动态样本映射进装配的适配层、
③跨 ABI 的矩阵返回路径 + capability 声明。

这不是从零造，但**也不是纯 Python 能交付的**——矩阵生产者必须落在 native 侧。

## 2. 已经存在、必须复用（D6）

### 2.1 `WorkingSample` 就是冻结协议的数据结构（关键发现）

`packages/suspension_kernel/cpp/include/mb_linearization/types.hpp:37-58`

| 字段 | 行 | 与冻结协议的关系 |
|---|---|---|
| `gauge` | :39 | 逐坐标 gauge 标志——SPEC"活动分支"的载体 |
| `active_tires` | :41 | 活动轮胎集**按列序**——SPEC"活动分支"的载体 |
| `load_scale` | :43 | 外载缩放——SPEC"未扰动输入"的载体 |
| `pose` | :51 | **装配所在的物理位姿，以数据形式到达** |
| `position_constraint` | :57 | `rows×ndof`，position 行策略 |

**`pose` 的注释（:44-50）直接说明了为什么它就是冻结协议所需形态**：它是**数据**而非在本模块推导，
因为推导意味着施加增量、而增量参数化是求解器自己的约定（静态用左 SO(3)，动态用另一套映射），
"共享模块若自己选一套就会把它静默强加给另一个"。**这正是 SPEC §34-35 冻结要求
（固定 world 速度/内部状态/未扰动输入/活动分支）的现成落地形态。**

### 2.2 `assemble_physical_tangent` 已与 Newton 迭代解耦

`packages/suspension_kernel/cpp/include/mb_linearization/types.hpp:78-84`，
实现在 `cpp/src/linearization/physical.cpp:72-248`。

**实测证据（解耦是真的，不是注释声称）**：函数体内**无循环迭代、无收敛判据、
无迭代计数**，返回 `bool`（smooth 且有限）。它只做：
- `:93` `constraint_jacobian(model, candidate)` —— 物理
- `:127-140` 逐列方向导数（`external_force_directional` / `constraint_jacobian_directional`）
- `:150-156` 物质切线 + 预载几何项

高斯消元式的"求解器策略"一律不在其中。**分层契约见
`types.hpp:8-16`**：`mb_linearization` 在求解器**之下**，`mb_solve_static` 与
`mb_solve_dynamic` 向下调用它，"它从不向上回调"。

### 2.3 `x` 是通用增量向量，装配可在任意状态求值

`physical.cpp:80` 校验 `x.size() == dimension`；`:101` 读 contact 压缩
（`x[n+m+index]`）；`:107` 取 `lambda`（`x[n..n+m]`）；
**`:132` `set_direction_for_coordinate(direction, body, component, x, free_body)`** ——
方向导数**读 `x`**。

即：装配**不要求 `x` 来自某次迭代**，任意满足长度与语义的 `x` 都可求值。
`dimension = ndof + rows + active_tires.size()`，块序
`[pose | multipliers | contact compressions]`（`types.hpp:63-65`）。

### 2.4 拒绝语义已经是"报告式"而非"静默式"

`types.hpp:74-77`：返回 `false` 时**矩阵仍被填充**，调用方可用回退路径
（有限差分列、更小步长）。这与 SPEC §41"非光滑报告 nonsmooth"、
§98"不填零/截断/复用前点"一致，**无需新建拒绝机制**。

### 2.5 消费层完整（P2 已交付），只缺输入

| 文件 | 符号 | 行 |
|---|---|---|
| `results/equilibrium.py` | `equilibrium_response` | :184 |
| | `load_mode_matrix` | :155 |
| | `audited_modes` | :368 |
| | `response_request` | :484 |
| `results/compliance.py` | `wheel_rate` | :179 |
| | `ride_rate_series` | :229 |
| | `ride_rate` | :265 |
| | `roll_rate` | :312 |
| | `load_angle` | :373 |
| | `metrics_request` | :459 |

`equilibrium.py:211-221` 首先校验 `tangent` 方阵、`load_jacobian` 形状
`(dimension, n_load)`——**这两个矩阵今天没有任何生产者**。

### 2.6 缓存键已预留冻结字段

`results/cache.py:113-114` `FactorizationKey` 含 `frozen_protocol: str = ""`、
`frozen_version: int = 1`；`:135-136` 校验 `frozen_version >= 1`。
`cache.py:1-8` 模块文档首句即"a characteristics request asks a *set* of questions about one
**frozen mechanism state**"。**步骤 4 的"同 A 分解共享"所需的键已就位。**

## 3. 完全没有、必须新建

### 3.1 `B_w = -dR/dw` 零实现（最关键的缺口）

全仓库检索 `B_w|load_jacobian|dR_dw`：

- `results/equilibrium.py` —— `B_w` 只出现在**文档串与注释**（:2、:12、:30、:195、:257）
  与**形参名** `load_jacobian`（:186、:370）中，**从不被计算**。
- `results/compliance.py:94-96` —— 注释里提到"response layer multiplies by `B_w`"。
- C++ 侧 —— **零命中**。

**结论：`B_w` 只有消费者，没有生产者。这是 P4a 的核心新增。**

### 3.2 物理 tangent 不跨 ABI

`cpp/src/abi/` 下只有 6 个 `extern "C"`（`kernel_abi.cpp:874,888`；
`kernel_core.cpp:212,216`；`kernel_contract_run.cpp:190,256,272`），
**没有任何一个接收状态、返回矩阵**。

唯一被写出的矩阵是**约束 Jacobian**（`kernel_contract_run.cpp:1119`
`push_block("constraint_jacobian", ...)`），它是 `rows×ndof` 的约束行 Jacobian，
**不是**物理 tangent，**也不是** `B_w`。

Python 侧 ctypes 签名只声明了 `run`（`kernel/__init__.py:125-135`）。

### 3.3 capability 明确声明"应答能力为零"（且这是刻意的诚实）

`kernel_contract_run.cpp:229-243`：`characteristics_response_kinds` 是
**空数组**，注释原文——

> "Reading a request and *answering* it are different capabilities, and this build
> does the first without the second. ... claiming a response kind here would be the
> empty-shell capability the contract forbids, because the reader would accept a
> request this kernel cannot answer."

**这是 P0 已确立的 fail-closed 诚实模式。** 因此 P4a 在真正实现应答之前
**不得**把 response kind 加进该数组；一旦实现，则必须**同时**加入
（否则就是"接受一个答不了的请求"）。

### 3.4 请求解析侧已有 frozen 协议校验（但只是校验）

`cpp/src/cases/case_characteristics.cpp`：

- `:87` 白名单含 `"frozen_force_tangent"` / `"frozen_load_compliance"` / `"local_response"`
- `:101-104` `known_dynamic_kind` 含前两者
- `:38` 请求体允许 `"frozen_conditions"`
- `:61-64` `frozen_fields()` = `{"protocol", "version"}`
- `:341-366` 校验类型与正版本号
- **`:359-365` 强制配对**：kind 是 frozen 而**无** `frozen_conditions` 直接报错，
  注释原文"没有协议就允许内核自选一个，这正是 frozen response 不再意味着已知
  frozen response 的方式"

**这些是校验，不是计算。** 本步不需要重建它们。

契约侧同样完整：`suspension_contracts/characteristics.py:35-37`
`DYNAMIC_DEFINITION_KINDS`、`:99-102` 字段白名单（`_FROZEN_FIELDS = {"protocol","version"}`）、
`:273-281` 强制配对。`solve_plan.schema.json:61` 的 `kind` enum 已含两者。

### 3.5 动态残差/Jacobian 纯内部，不可复用

`residual`（`mb_solve_dynamic/functions.hpp:41,63`；定义
`kernel_integrator_residual.cpp:28`）与 `fill_analytic_jacobian_columns`
（`functions.hpp:65`；定义 `kernel_integrator_newton.cpp:24`）的**调用者全部落在
`kernel_integrator_newton.cpp` 内部**，由 `newton_step`（:780）驱动，
`newton_step` 的唯一调用者是 `solve_one_step`（`kernel_integrator_step.cpp:86`）。

`ResidualContext`（`mb_solve_dynamic/context.hpp:26-38`）需要**三个不同 `SampleInput` 指针**
加 `h`/`alpha_m`/`alpha_f`/`beta`/`gamma` 等**九个时间积分系数**——
它是"某一步的积分上下文"，**不是**"某个样本的冻结状态"。
**因此动态路径的残差/Jacobian 不能作为 P4a 的装配来源**（否则离散系数泄漏，
违反设计 §555 与 SPEC §41）。

## 4. 由侦察推出的第 1 笔实现决策

**装配应复用 `assemble_physical_tangent`（静态侧已解耦的物理），
而不是动态侧 `ResidualContext`。** 依据：

1. `WorkingSample` 已是冻结协议所需的数据形态（§2.1）；
2. 动态侧的 `ResidualContext` 携带九个离散系数，用它会把离散量带进物理响应，
   直接违反 SPEC §41"不重放 return mapping"与设计 §555"无离散系数泄漏"；
3. `WorkingSample` 的 `pose` 明确以数据到达，正是为了不把参数化约定强加给第二个调用方。

**仍待步骤 1 解决的开放问题（不预先声称已解决）**：

- **A. gauge 与 position_constraint 的来源**：今天由静态求解器私有函数
  `static_gauge_coordinates`（`kernel_static_trim.cpp:281`）与
  `static_position_constraint_jacobian`（:342）生成，**无外部参数入口**。
  动态样本要复用装配，必须为这两者定义冻结态的供给方式——是"冻结静态策略"、
  还是"从动态样本的活动分支推导"，**必须由证据决定，不能猜**。
- **B. 乘子基值**：SPEC §36"lambda 基值取原态，delta lambda 联立求，不默认冻结"。
  动态样本的乘子是约束力，需确认它在哪个输出块、是否可用。
- **C. 惯性 wrench**：SPEC §34 要求"固定每刚体质心处 world 六分量 wrench"，
  其来源是该样本的质量/惯量/加速度/角速度/陀螺项。需确认加速度是否在样本状态中。

**A/B/C 三项是步骤 1 的实测任务**，本文件不预先给出结论。

### 4.1 开放问题 C 已解决：加速度**在**样本状态里

实测 `cpp/src/output/kernel_output.cpp:27-38`（`copy_state`，19 列的唯一写入点）：

```
[0:3]   r      位置
[3:7]   q      四元数（scalar-first：w,x,y,z）
[7:10]  v      world 线速度
[10:13] omega  world 角速度
[13:16] a      world 线加速度      ← SPEC §34 惯量 wrench 所需
[16:19] alpha  world 角加速度      ← 所需
```

`kStatePerBody = 19`（`mb_config/constants.hpp:19`）。

**重要修正**：`kernel_output.cpp:150-153` 的**读取**路径只消费 0..12 列，
**第 13..18 列在该读取点未被解释**——但这不是"列不存在"，
而是那个特定调用点只重建 `r/q/v/omega`（它服务于转向测量）。
列的**权威定义**是 `copy_state`（写入点）。故
"样本状态含加速度"成立，**惯量 wrench 可从样本直接构造，无需重新积分**。

另注：`kernel_output.cpp:605-618` 的 `row[13..18]` 是 **diagnostics ledger 的行**，
与 body_state 的 13..18 列无关——两处同号不同物，易被误读，故显式区分。

### 4.2 开放问题 B 已解决：乘子有现成输出块与 Python 访问器

- C++ 写入：`abi/kernel_contract_run.cpp:1115-1116`
  `push_block("constraint_multiplier", ..., {total_samples, built.rows})`
  ——形状 **`(samples, rows)`**，逐样本逐约束行。
- Python 访问：`results/envelope.py:172-178`
  `constraint_multiplier(constraint_id)` → `raw.block("constraint_multiplier")[:, start:start+count]`，
  按约束名切片。
- 既有使用例：`tests/results/test_universal_result_channels.py:40`、
  `tests/simulation/test_generic_native_model_coverage.py:64`
  （后者断言形状 `(len(samples), 1)`）。

**故 SPEC §36"lambda 基值取原态"有数据来源，无需新增输出。**

### 4.3 开放问题 A 缩小：position 约束数据也有现成输出块

- `abi/kernel_contract_run.cpp:1119-1120`
  `push_block("constraint_jacobian", ..., {total_samples, built.rows, built.ndof})`
  ——形状 **`(samples, rows, ndof)`**，即逐样本的约束 Jacobian。
- Python 访问：`results/envelope.py:310` `self.raw.block("constraint_jacobian")`；
  另有 `vehicle/roll_centers.py:42` 同款用法。
- 请求开关：`compilation/resolved.py:146` `case["outputs"]["constraint_jacobian"] = True`。

**这把问题 A 从"无外部入口"缩小为"如何在冻结态供给 `gauge` 与
`position_constraint` 两个 `WorkingSample` 字段"**——数据本身（`pose`、`lambda`、
约束 Jacobian、活动轮胎）都可从样本取得；剩下的纯粹是**策略选择**：
`gauge`（哪些坐标被声明为 gauge 以移除刚体模式）与
`position_constraint`（哪些约束行是 position 行）在冻结态应当取什么值。
**这仍是一个必须由证据决定的设计问题，本文件不预先给出答案**，
但已确认它不是"缺数据"，而是"缺策略定义"。

### 4.4 开放问题 A 已解决：`gauge` 与 `position_constraint` 都是**模型级数据**，
**不需要发明新策略**

实测（这是本轮最关键的发现）：

**`gauge` 的来源是 Model 字段，不是求解器私有状态。**

- `static_gauge_body` / `static_gauge_dof_mask` 定义在
  `mb_model/types.hpp:441`（`std::uint32_t static_gauge_dof_mask{0}`）
  与 `mb_model/types.hpp` 的 `static_gauge_body`。
- 它们由**契约计划**写入：`abi/kernel_contract_run.cpp:586-603`
  （`if (plan.cases.front().has_static_gauge)` → `built.static_gauge_body/dof_mask`），
  注释原文：*"A static gauge removes a declared global null space rather than adding a
  physical constraint, so it belongs to the model the solver sees and has to be
  installed before the first run. Every case in one document shares it."*
- `static_gauge_coordinates(model)`（`solve_static/kernel_static_trim.cpp:281-297`）
  是**模型的纯函数**——只读 `model.static_gauge_body` 与 `model.static_gauge_dof_mask`，
  不读任何迭代状态。

**`position_constraint` 是"每行都是 position 行"。**

- `static_position_constraint_mask`（`:318-330`）先构造 `rows` 长的全 1 向量，
  再遍历 `coordinate_couplers` 把对应行**再设为 1**——即**该循环是空操作**，
  结果恒为全 1。
- `static_position_constraint_jacobian`（`:342-356`）因此就是
  `constraint_jacobian(model, state)` 本身（mask 全 1 ⇒ 无行被清零）。

**结论：冻结态的 `WorkingSample` 可以完全由"模型 + 冻结状态"构造，
无需引入任何新的策略概念。** 具体：

| 字段 | 冻结态取值 | 依据 |
|---|---|---|
| `gauge` | `static_gauge_coordinates(model)` 的 0/1 展开 | `:281-297`，模型纯函数 |
| `position_constraint` | `constraint_jacobian(model, pose)` | `:342-356`，mask 恒全 1 |
| `active_tires` | 该样本的活动轮胎集（按列序） | SPEC §34"活动分支" |
| `load_scale` | 1.0（未扰动输入） | SPEC §35 |
| `pose` | 该样本的 `r`/`q` 重建的 `State` | `copy_state` 列 0..12 |

**这消除了 P4a 的最大设计不确定性**：不是"要在两种策略中猜一个"，
而是"沿用模型已声明的 gauge、沿用恒全 1 的 position mask"。
`has_static_gauge` 为假时 `gauge` 自然全 0（`:283-285` 提前返回空集），
这也是**如实反映**而非默认值。

**分层要求（决定代码放哪）**：`mb_linearization` 的契约明写
"it never calls back up"（`types.hpp:9`），且
"the gauge/pin policy ... must not move here"（`:11-16`）。
因此**冻结样本的构造器不能放进 `mb_linearization`**——它是 policy，
必须放在能向下调用 `mb_linearization`、又能读 Model 的新位置。

## 5. 本步**未**做的事情（诚实边界）

- 未写任何生产代码；未改任何文件。
- 未跑任何门禁（本步是只读侦察）。
- 未验证 §4.3 遗留给步骤 1 的策略问题（冻结态 `gauge` / `position_constraint` 取什么值）
  ——A/B/C 三个数据可得性问题均已解决（§4.1/§4.2/§4.3），
  剩下的**不是缺数据而是缺策略定义**，需要实跑探针与行为断言才能确定。
- 本文件不能作为步骤 1 的完成证据；步骤 1 的验收落点是
  `tests/characteristics/test_frozen_protocol.py` 与行为断言。
