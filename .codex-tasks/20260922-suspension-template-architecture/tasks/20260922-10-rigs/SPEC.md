# 子任务 10：试验台抽取与总成 × 试验台正交组合

## 目标

把现有 7 个写死的 `(assembly, family)` 枚举，改成「总成」与「试验台」两个独立注册轴的正交组合。

1. **试验台抽取**：把 case 侧职责（驱动哪些坐标、时间/载荷怎么施加、求解器默认、结果怎么排布）抽出为独立的「试验台」概念，与总成的模型侧职责分离。
2. **总成注册**：总成提供体、副、元件、轮心、可驱动坐标的名义空间、以及**自己的最小单位输出**（07 已定义输出声明）。
3. **试验台注册**：试验台声明它要求总成提供哪些坐标、如何驱动、以及**自己独有的输出**。
4. **正交组合**：运行 = 选一个总成 + 选一个试验台；组合矩阵可查询；无效组合在准备阶段报错并点名（哪个坐标、由谁提供）。
   - **能力驱动而非名字探测**：组合解析消费 04 暴露的 `AssemblyCapabilities`，判据只有一条——`coordinate in capabilities.drive_coordinates`（命名空间是模型文档里的驱动坐标名 `wheel_drive_L`/`wheel_drive_R`/`rack_drive`/`rack_neutral`，**不是** `axis_map` 的分组键 `"wheel"`/`"rack"`，混用即接口错位）。不得靠检查 `bodies` 里有没有 `"rack"` 之类的名字推断。两种总成（含转向 / 不含转向）必须能配**同一个**试验台实例。
   - **写边界**：`AssemblyCapabilities` 的结构与取值由 04 建立并冻结；本步只**消费**，可按需扩展字段但不得改其语义或把判据换成名字探测。若确需扩展，须在 PROGRESS 中登记并与 04 的契约对齐。
5. **试验台接口按总成实际能力自适应（需求 15 / D6）**：试验台声明的是**要求**，但组合解析时必须以总成实际提供的子系统为准做**收缩**，而不是硬要求后失败：
   - 总成不含转向子系统时，**rack 驱动轴整体消失**（不是填零、不是跳过该轴但保留占位列），K 网格降维为纯轮跳；试验台仍可运行。
   - rack 相关输出（如 `steering_output`/rack 位移通道）随之消失；结果对象的形状随总成能力变化，须由能力描述驱动，不得让下游猜。
   - **当前实现不是这样（改造点已实测逐处定位，实施时复核）**：
     - 模型文档侧：`cases/kc_quasi_static/contract.py:284-293`（函数 `_driven_coordinates` 内） 无条件 add `rack_drive`（K）/`rack_neutral`（C），取 `assembly.point("rack","center")`——无 rack 时即报错。
     - case 侧：同文件 `:360-362`（函数 `case_document` 内）的 `axis_map["rack"]` 用 `next(value for value in driven_names if value.startswith("rack_"))`——无 rack 驱动时抛 `StopIteration`（**当前无转向总成的直接崩溃点**）。
     - K 网格侧：`api.py:334-338` 的 `_K_COORDINATES` 写死 `{"left","right","rack"}`，且被两处消费——`:344-389` 的 `_k_grid`（`:362-375` 对称简写分支恒产出 `rack_values_mm` 与 `axis_map["rack"]`；`:376-388` 三元组分支恒产出 rack 轴并构造 `(left,right,rack)` 三元组）与 `:379` 的 axes 列表。
     - K 结果侧：`api.py:440` 解包三元组 `left, right, rack = combinations[index]`；`:453-457` 写 `drives["rack_displacement"]`；`:273` 无条件为 body `"rack"` 取位姿。
     - 时间序列结果侧：`api.py:256` 写 `rack_displacement` metric。
     - 控制轴解析：`api.py:693-701` 的 `_k_control_axis` 把 rack 目标映射为 `"rack"`。
     - **准静态重放路径（原清单漏项）**：`api.py:202-224` 的 `_run_axle_quasi_static`（`run_dynamic_case` 的 K 准静态重放）在 `:206` 取 `motion(case, "rack")`、`:221` 无条件写 `{"coordinate": "rack_drive", ...}`、`:256` 产 `rack_displacement` 指标、`:273` 对 `("upright_L","upright_R","rack")` 采样位姿——无转向总成走此路径会对不存在的 `rack` 体报错。
     - **结果键集**：`api.py:453-457` 的 `StateResult.drives` 无条件含 `"rack_displacement"`；收缩后该键集须随能力变化（不是留一个恒为 0 的键）。`results/**` 经查无 rack 硬依赖，rack 通道都在这两处 api 层填充。
     本步须使模型文档侧、case 侧、K 网格与 K 结果侧**按总成能力收缩**（`_K_COORDINATES` 的硬编码改为由能力构造，不是删掉常量）。
   - **范围边界（勿过度收缩）**：C 模式一直固定 rack neutral（`api.py:542-546` 的 `drives["rack_displacement"]=0.0`），`adams/`、`simulation/replay.py`、`preparation/signals.py` 中的 rack 名字属**既有含转向路径**，均不在无转向收缩范围；收缩只针对「总成不提供 rack 坐标」这一新路径。
   - **收缩与「无效组合报错」不冲突**：收缩用于**可选子系统**（转向在单轴侧可缺席，D5）；若试验台要求的是总成**结构上不可能提供**的东西，仍按第 4 条在准备阶段报错点名。两者判据不同，不得用收缩掩盖错误配置。
6. **单轴侧车轮由试验台提供（需求 19 / D9）**：单轴悬架实验总成的车轮**不是**总成产出的子系统，而是试验台的一部分——对标 Adams `__MDI_SUSPENSION_TESTRIG`（`acar_gs_front.asy` 的 `[TESTRIG] USAGE='__MDI_SUSPENSION_TESTRIG'`）：
   - 试验台声明车轮的刚性轮属性与轮胎刚度：`testrig_tire_property_file`（Adams 取 `'RIGID_WHEEL'`）、`testrig_wheel_radius`（300.0）、`tire_stiffness`（200.0，注释「Used by suspension testrig tire」）；
   - 现役单轴侧车轮表示是 `VerticalTireElement`（`front_axle.py:561-574`，挂 `upright_{side}`），**本步维持其物理表示不变**，只把归属声明移到试验台侧；
   - **整车侧不受影响**：wheel 子系统由总成提供（`_add_wheel`），试验台不声明车轮。
7. **新增组合**：至少落地一个此前不存在的组合，证明正交性有效（候选：`axle × ride_four_post` 或 `axle × ride_random_road`）。

## 非目标

- 不改各 family 的物理语义（驱动坐标的含义、时间网格的意义、求解器默认值）。
- 不删除现有 7 个组合；它们必须继续可用且行为不变。
- 不实现输出求值本身（07 已做）；本步只把总成输出与试验台输出**接进组合**。
- 不改 C++ 内核与契约格式。
- 不做"总成组合"（如整车 = 两个轴总成之和）——用户的子系统组装分层由 04 的子系统与 05 的实例化承担；本步只做**总成 × 试验台**这一维正交。
- **需求 20/D11 对试验台的约束**：试验台**不得**因制动/驱动是简化模板还是复杂模板而分支；试验台消费的是 role 的力矩通道与参数槽，不是模板身份。

## 现状事实（制定计划时实测，实施时复核）

现有 7 个写死的组合（每个一个 compiler，`simulation/dispatch.py:20-33`）：

| assembly | family | compiler |
|---|---|---|
| `axle` | `kc_quasi_static` | `KcQuasiStaticCompiler` |
| `axle` | `axle_dynamic` | `AxleDynamicCompiler` |
| `vehicle` | `vehicle_dynamic` | `VehicleDynamicCompiler` |
| `vehicle` | `vehicle_kc` | `VehicleKcCompiler` |
| `vehicle` | `handling` | `HandlingCompiler` |
| `vehicle` | `ride_four_post` | `RideFourPostCompiler` |
| `vehicle` | `ride_random_road` | `RideRandomRoadCompiler` |

内核侧 family 由 `cpp/src/cases/case_dispatch.cpp:17-19` 的 `contract_case_supported` 枚举（7 个 + `comparison` 按设计 N/A）。

**"试验台"目前散落在各 family 内部**：
- `axle × kc_quasi_static`：驱动坐标由模型侧声明（`cases/kc_quasi_static/contract.py:177-222` 的 `_driven_coordinates`），case 给网格值。
- `vehicle × ride_four_post`／`ride_random_road`／`handling`：case 侧只产出 `case_document`（如 `cases/ride_four_post.py:39`），模型文档复用 `cases/vehicle_dynamic.py` 的 `model_document`。
- 这就是"试验台职责"尚未成型的证据：模型侧与 case 侧的边界因 family 而异。

## 约束

- **现有 7 个组合的行为必须不变**：默认路径的文档字节与结果不应改变；若某组合因重构而必须变，须单独裁决并登记基线重录。
- **总成与试验台的输出归属要分清**：总成输出是「这个总成本身有什么」，试验台输出是「这个试验台额外测量什么」；合并规则与冲突检测在 07 已定义，本步接入。
- 组合查询必须是**声明式**的（可列出所有合法组合），不是靠读 compiler 代码推断。
- 无效组合的报错必须**点名具体坐标**与提供方，不得只说"不支持"。
- 不引入新依赖。
- 内核 family 名与 Python 侧 `SimulationRequest.family` 的兼容性保持（改名或加别名，不静默改变既有调用）。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/rigs/**`（试验台定义与注册表）
  - `packages/suspension_multibody/src/suspension_multibody/simulation/**`（dispatch/compiler/preparation 改为按组合解析）
  - `packages/suspension_multibody/src/suspension_multibody/cases/**`（case 侧按试验台归并）
  - `packages/suspension_multibody/src/suspension_multibody/subsystems/**`（总成注册其输出与可驱动坐标）
  - `packages/suspension_multibody/src/suspension_multibody/api.py`、`vehicle/service.py`（公开入口走组合路由；**含需求 15/D6 的 K 网格与 K 结果收缩**：`_K_COORDINATES`（`:334-338`）、`_k_grid`（`:344-389`）、`:440` 三元组解包、`:453-457` drives、`:256` 时间序列 metric）
  - 新增测试 `packages/suspension_multibody/tests/rigs/**`
  - `packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/**`（若确受影响，须登记）
- 只读（新增）：`results/**`（kc 结果对象字段——若收缩需要在结果层去掉 rack 通道，须与 07 的输出声明对齐后再动，不得在本步私改结果对象契约）。
- 依赖（新增）：`subsystems/**` 暴露的「总成实际提供的子系统与坐标系集合」（04 建立）——试验台收缩接口的唯一依据，不得退回按 `bodies` 名字探测。
- 只读：`joints/`、`templates/`、`outputs/`（07）、`report/`、C++ 内核。
- 不写：`preparation/assembly/**` 的几何生成（04 的范围）；父级计划文件（归主代理）。

## 依赖

- 前置：09（准静态/动态 study 合并——试验台要建立在统一的 study 概念上）。
- 后续：12（终局验收的端到端第 (d) 项要求两个组合可跑、其中一个此前不存在；第 (g) 项要求无转向总成跑通；第 (h) 项要求简化→复杂模板可替换）。

## 验收标准

1. 组合矩阵可查询：能列出所有合法的（总成 × 试验台）组合及其提供方。
2. 现有 7 个组合全部继续可用，且**行为不变**（默认路径文档字节与结果不改变；若有变化须逐项登记）。
3. 无效组合在**准备阶段**报错，错误信息点名具体坐标与提供方（有负例测试）。
4. 至少一个此前不存在的组合跑通（`axle × ride_four_post` 或 `axle × ride_random_road`），且不需要为新组合复制驱动坐标/时间网格/结果排布的代码。
5. **试验台接口按总成能力自适应（需求 15 / D6）**：同一个悬架 KC 试验台在含转向与不含转向两种单轴总成上都能跑；不含转向时 rack 驱动轴与 rack 相关输出整体消失、K 网格降维为纯轮跳；含转向时行为不变。**不得以填零、跳过但保留占位、或捕获内部异常的方式实现**。
6. 门禁：`case_parity_check.py` 8 family 通过（现有 family 全走含转向路径，不得因收缩逻辑而变）；`--strict --final` 保持绿；ruff/ty 通过；`dynamic_hash_sentinel` 与 `kc_parity_check` 保持绿或登记重录；`tests/data/**` 除已登记项外无 diff。
7. **单轴侧车轮由试验台声明（需求 19 / D9）**：试验台可声明 `tire_property_file`/`wheel_radius`/`tire_stiffness`（对标 Adams `testrig_*` 参数）；单轴组合下这些声明生效，整车组合下 wheel 子系统由总成提供、试验台不声明车轮。有测试断言两种组合下「谁提供车轮」清晰且不重复。
8. **对制动/驱动模板身份无分支（需求 20 / D11）**：试验台代码中对 `brake`/`drive` 的处理**不得**出现按模板名、按有无刚体、按 `simplified` 之类的判断；有断言（静态检查或测试）证明试验台路径与模板身份无关。

## 验证协议

1. 试验台定义与注册表落地后：跑组合矩阵查询测试。
2. 路由改造后：逐组合跑现有 7 个 family 的既有测试，确认行为不变。
3. 无效组合检测落地后：跑负例（试验台要求总成不提供的坐标）。
4. 新增组合落地后：跑通该组合的端到端（含结果可比）。
5. 需求 15/D6 落地后：跑「同一试验台 × 含转向总成」与「同一试验台 × 不含转向总成」两组端到端；后者断言 `axis_map` 不含 rack 轴、K 网格无 rack 维度、结果对象无 rack 通道，且**不抛 `StopIteration`/`KeyError` 等内部异常**；前者断言行为与改造前逐位一致。
6. 单轴侧车轮声明落地后：跑「单轴组合下车轮来自试验台」与「整车组合下车轮来自总成」两组测试，确认不重复提供。
7. 需求 20/D11 落地后：用复杂桩模板替换简化制动/驱动模板，断言试验台代码零改动即可跑通。
8. 收尾：三套 pytest、ruff、ty、`--strict --final`、三个数值门、`git diff --check`。

**若现有 7 个组合中任何一个的行为发生变化**，先判断是"重构引入的回归"还是"物理改变"：前者必须修掉，后者必须单独裁决并登记——不得以"重构"为名顺手重录 vehicle 系列基线。
