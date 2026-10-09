# Suspension MBD Equivalence

The suspension MBD context models a symmetric front double-wishbone axle and compares it with independently generated Adams/Car results.

## Language

**Canonical Equivalence Model**:
The complete physical front-axle definition consumed independently by suspension_multibody and Adams/Car.
_Avoid_: demo model, reference copy

**Equivalence Manifest**:
The immutable, hashed record of a Canonical Equivalence Model, boundary conditions, load grid, and required result fields.
_Avoid_: configuration, baseline

**Strict K**:
An ideal-joint, load-free kinematic comparison over a fixed displacement and rack grid.
_Avoid_: geometry smoke test

**Strict C**:
A compliant, loaded comparison over a fixed six-component load grid that evaluates response magnitudes and common component loads.
_Avoid_: symmetry check, compliance smoke test

**Common Entity**:
A body, joint, force element, bushing, metric, or load represented identically by both generated solvers.
_Avoid_: approximately similar component

**C Reference State**:
The unloaded, converged pose at the same prescribed wheel and rack inputs as a strict C state.
_Avoid_: default pose, nominal offset

**Strict C Basis**:
The six wheel-center wrench axes, their unit conventions, application frames, levels, and side mode fixed in an Equivalence Manifest.
_Avoid_: report setting, compliance summary

## Composable Simulation Language

**Template（模板）**:
定义一类模型的构件、连接关系、参数、属性和对外接口的可复用规则；不限于固定硬点或固定拓扑。
_Avoid_: 参数文件、某一次运行的模型

**Subsystem（子系统）**:
模板实例化得到的物理组成单元，例如悬架、转向、车轮、车身、制动或驱动；允许没有刚体的简化力元子系统。
_Avoid_: 固定拓扑名称、求解器类型

**Wheel Subsystem（车轮子系统）**:
包含车轮刚体、轮胎接触力元及其属性和接口的物理组成单元；轮胎属于该子系统，不是与车轮并列的总成成员。
_Avoid_: 仅轮体、独立轮胎力律总成成员

**Tire Element（轮胎力元）**:
车轮子系统内根据轮胎属性、轮端运动和道路状态产生接触力与力矩的元素；轮体惯性与轮胎模型内部惯性各有唯一归属。
_Avoid_: 车轮子系统、重复轮体

**Brake Subsystem（制动子系统）**:
由模板定义制动力元、属性、控制输入和机械接口的物理组成单元；详细模型还可以包含制动盘、卡钳及其连接。
_Avoid_: 仅制动力律实例

**Drive Subsystem（驱动子系统）**:
由模板定义驱动力元、属性、控制输入和机械接口的物理组成单元；详细模型还可以包含轴系、传动机构及其连接。
_Avoid_: 仅驱动力律实例

**Rig（试验台）**:
用于支承、加载、驱动和测量被测对象的模型，可包含刚体、运动副、运动和力元；与子系统共享模型构建概念。
_Avoid_: 工况别名、只有输入曲线的配置

**Assembly（被测总成）**:
子系统通过接口连接形成的被测对象；例如单轴悬架总成或整车总成，尚未与试验台构成完整仿真模型。
_Avoid_: SimulationAssembly

**Simulation Assembly（仿真总成）**:
被测总成与试验台完成连接后形成的完整物理模型；结合研究方式和具体工况后可进行仿真。
_Avoid_: 工况名称、求解结果

**Port（连接端口）**:
模型对外提供的带物理语义的连接位置或通道，具有明确的归属、坐标系和连接能力；不是任意一个硬点。
_Avoid_: 根据名称猜测的连接点

**Adaptive Connection（自适应连接）**:
根据当前被测总成的端口和几何配置确定试验台位置、连接和可选分支；不改变必需试验内容或绕过总成规则。
_Avoid_: 任意拓扑自动兼容、缺少必需接口时静默删掉试验

**Study（研究方式）**:
规定如何求解同一仿真总成的物理问题，例如独立平衡态序列或有历史依赖的动态过程；与 K/C 连接模式不同。
_Avoid_: Rig、悬架类型

**Case（工况）**:
一次研究中的具体激励、载荷、扫描或采样安排及求解设置。
_Avoid_: 模板、被测总成

**Assembly Policy（总成规则）**:
对某类总成的必需或禁止子系统及实体归属施加的全局规则；接口兼容不代表可以绕过这些规则。
_Avoid_: 可由模板覆盖的默认配置

**Family（族）**:
激励协议的名称，例如载荷扫描、四柱输入或转向操作；规定输入如何展开，不决定被测对象的实体、编译器或结果类型。
_Avoid_: 试验台名称、模板名、业务求解路径

**Solve Plan（求解计划）**:
一次运行的研究方式、采样、运动边界、载荷、输入和求解设置；描述对同一个物理模型提出的问题。
_Avoid_: 工况、配置对象

**Resolved Model（解析后模型）**:
完成属性、单位、端口和实体引用解析后的不可变物理模型，具有稳定实体标识、来源和指纹；不依赖悬架或整车等业务类别。
_Avoid_: 第二份装配、业务运行时对象

**Emitter（发射器）**:
把解析后模型与求解计划表达为版本化原生契约的过程；对所有子系统使用相同的实体语义。
_Avoid_: 求解器、处理器

**Constitutive Law（本构力律）**:
根据端口状态、控制输入和属性参数计算力、力矩及必要的内部状态；不负责创建刚体、连接或决定子系统角色。
_Avoid_: 业务子系统构建器、模板拓扑

**Law Property（力律属性）**:
一个可复用、带单位和版本的本构参数数据集，例如轮胎参数、制动摩擦参数或阻尼曲线；不包含可执行 Python 代码，也不包含某个实例的几何连接。
_Avoid_: 工况、运行时状态、模板

**Law Instance（力律实例）**:
子系统内部力元使用的力律与属性绑定；有状态力律的实例状态相互独立，无状态力律无需内部状态。
_Avoid_: 属性文件、共享参数对象、子系统替代物

**Law Adapter（力律适配器）**:
校验 Law Property 并将其编译为内核支持的原生力元、轮胎项或输入通道；适配器按 `kind/model/version` 注册，不按 suspension、wheel、brake 等业务角色分支。
_Avoid_: 通用装配器里的角色特例

## Characteristic Analysis Language

**Characteristic Analysis（特性分析）**:
在指定多体工作样本、参考系和响应边界下，对测量变化率、允许运动及载荷响应的分析；可作用于悬架、整车及其他机构。
_Avoid_: 第二套仿真、仅静态 SVC

**Operating Sample（工作样本）**:
一次求解中的有效构型、运动、力元内部状态和当时输入；独立平衡样本满足静态平衡，动态样本属于一致的已接受轨迹。
_Avoid_: 只有位姿的工作点、动态平衡点

**Perturbation（扰动）**:
围绕工作样本施加的坐标、载荷、输入或状态增量，其作用实体、方向、参考系和单位明确。
_Avoid_: 未指定方向的单位力、另一个非线性工况

**Response Boundary（响应边界）**:
定义特性扰动中哪些运动保持、哪些已声明驱动释放，以及响应相对哪个参考体测量的条件。
_Avoid_: 修改被测总成、默认固定所有车身

**Kinematic Tangent（运动学切线）**:
在当前约束和响应边界下，对指定驱动的局部允许运动；它与当前真实运动速度及静态柔度具有不同含义。
_Avoid_: 柔度、实际侧倾轨迹

**Equilibrium Compliance（平衡柔度）**:
满足静态平衡的工作样本附近，指定载荷增量产生的局部位移响应，包含实际力律、预载和约束的影响。
_Avoid_: 动态 Newton 逆矩阵、只由约束雅可比确定的柔度

**Trajectory Rate（轨迹变化率）**:
测量沿实际动态轨迹的时间导数，包含被测体和参考体运动，以及所依赖的输入和内部状态变化。
_Avoid_: 对轮行程的偏导、准静态网格时间斜率

**Dynamic Perturbation（动态小扰动）**:
实际轨迹附近满足连续运动及约束方程的状态和输入变化；其响应保留惯性、耗散和力元内部状态的影响。
_Avoid_: 瞬时静态柔度、离散积分步的 Newton 导数
