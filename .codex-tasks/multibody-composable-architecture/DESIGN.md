# 目标架构与冻结契约

状态：正式目标方案已冻结，非当前实现说明。需求与边界以 [EPIC.md](EPIC.md) 为准；领域术语见 [CONTEXT.md](../../packages/suspension_multibody/CONTEXT.md)。

## 1. 当前依据与问题落点

以下定位来自本轮及前轮亲自读取；实施 01 须重新核验，不依赖旧任务结论。

| 当前文件（包内相对路径） | 已核查事实 | 目标处理 |
|---|---|---|
| multibody `api.py:35` | 通过导入顺序避开 elements/core/assembly 初始化循环 | 02 提取基础模型，10 薄化入口 |
| multibody `templates/model.py:132` | 单一 Template 已存在，但 PartDefinition 仅有名称/质量/固定标记 | 03 演进为能产生完整物理图的模板底座 |
| multibody `subsystems/types.py:27` | 子系统类型依赖 preparation.assembly；后续子系统读取共享可变构建上下文 | 02/06 改成显式实例片段、端口和组合，避免隐式构建顺序 |
| multibody `simulation/request.py:61` | rig 与 family 相互补值并强制相等 | 07 独立 rig/study，旧名称仅留适配 |
| multibody `studies/bridge.py:4` | K/C 装配经单位和类型映射转换为动态模型 | 06 收口 SI 模型，迁移完成删除生产桥接依赖 |
| kernel `cpp/src/contract/contract_registry.cpp:38` 与 `cpp/src/cases/case_dispatch.cpp:14` | 协议名称、支持列表与分派多点维护 | 08 描述表绑定支持状态和处理器 |
| kernel `cpp/include/mb_model/types.hpp:102` 与 `cpp/src/solve_dynamic/kernel_integrator.cpp:411` | 基础模型和积分器了解多种轮胎专用状态 | 09 抽取已有状态行为，不新增算法 |
| 根 CONTEXT-MAP 与 multibody 架构说明 | 保留旧目录与旧入口叙述 | 12 迁移后更新现状文档，当前只新增方案导航 |

内核已有 23 模块 DAG，前轮 strict 分层检查通过；保留这一基础，不以“更通用”为由推倒内核。

## 2. 目标模块结构

```text
packages/suspension_contracts/
  src/suspension_contracts/          # 保留现有公开入口和协议格式
    geometry / multibody 协议职责    # 不要求为目录对称进行搬迁

packages/suspension_multibody/src/suspension_multibody/
  api.py                            # 稳定薄入口
  schema/                           # 用户输入校验，可保留产品差异
  modeling/
    primitives/                     # 刚体、标记点、关节、驱动、力元声明与空间代数
    ports.py                        # 端口、需求及绑定的值对象
    instance.py                     # ModelFragment、实例标识与来源
    assembly.py                     # Assembly / SimulationAssembly 值对象
    validation.py                   # 引用、单位、所有权和结构检查
  templates/                        # 统一 Template/RoleSpec、注册与实例化协议
  subsystems/                       # 悬架、转向、车轮、车身、制动、驱动模板实现
  rigs/                             # 物理试验台模板、激励/测量声明
  connections/                      # 匹配、几何适配、连接生成与全局 policy
  studies/                          # 时间语义、轮胎激活、求解问题选择
  preparation/                      # 输入归一化与实例化/组合用例编排
  compilation/                      # 物理图及 Study/Case 到内核契约
  simulation/                       # 请求、执行生命周期、失败与元数据
  kernel/                           # 产品侧契约组包/拆包、能力适配
  results/                          # typed result、布局及产品兼容解码
  outputs/                          # 输出声明、绑定、衍生表达式
  report/                           # 消费输出的指标及报告
  adapters/、adams/                 # 外部格式导入与离线验证
  io/                               # 模型/产物/检查点读写

packages/suspension_kernel/
  src/suspension_kernel/binding/     # 产品无关加载、版本与错误
  cpp/include/mb_*/ + cpp/src/*/     # 保留现有编译模块，按职责收敛
    config / numeric / dual / linear / energy
    model / input / assembly
    joint / element / force / tire / tire_state
    solve_static / solve_dynamic / output
    contract / cases / abi
```

### 依赖规则

- modeling 的基础声明/代数不得导入具体模板、subsystems、rigs、preparation、simulation、kernel 或 report。
- templates 和 connections 可依赖 modeling；具体模板依赖模板底座与模型构建工具；底座不反向导入内置实现。
- 内置模板注册由显式 bootstrap 完成，不在基础类型模块导入时扫描或自动注册。
- preparation 编排模板与连接；compilation 只读已完成的物理图，不调用 build_front_axle 或查询具体拓扑名。
- simulation 编排 preparation/compilation/kernel/results；results 不调用 solver；report 只消费结果/输出。
- contracts 不依赖任何求解产品；两个 Python 求解产品不互相导入。
- C++ 保持无环；新依赖必须说明职责并更新受审查基线，不能为压低告警关闭检查。

## 3. 统一模板底座

保留一个 Template 声明格式，role/kind 表示用途，不建 SubsystemTemplate/RigTemplate 的继承树。它们在文档中可以作为用途称呼，但不是两套互不兼容的模型系统。

模板包括：标识/修订、用途、参数与属性槽、构件/连接/驱动/力元/输出声明、提供的端口与需要的端口。支持两个等价作者入口：直接填写声明，或由显式注册的 Python builder 生成相同声明。两者都进入相同验证与序列化路径，不让可执行函数进入模型契约。

Builder 逻辑契约：`build(parameters, properties, mode, bound_ports) -> ModelFragment`。被测子系统可先暴露连接需求，组合后绑定；试验台在被测总成端口已解析后构建。输入为只读快照，不读取其他子系统的私有 dict，不依赖注册顺序，不调用求解器。

同输入确定性产出；属性文件参与指纹。构建器可创建数量可变的构件并循环生成拓扑，不限于双横臂硬点集。RoleSpec 约束对外端口与能力，不要求模板具有 upper_arm 等特定内部构件。

允许 0 刚体力元子系统；复杂模板可有刚体和内部运动副。相同 role 的替换以端口能力一致为条件，不要求内部数量一致。所有质量、实体与输出须真实落到 ModelFragment，不能只在注册元数据声明存在。

K/C 为实例化连接选择，与 Study 正交：joint+bushing 双列择一；单列两模式保留；双空允许作为纯几何点。切换重建模型但保持未改变实体的稳定 ID、几何和质量，不能修改正在运行的状态。

## 4. 模型与坐标契约

| 值对象 | 最小内容与不变量 |
|---|---|
| EntityId | 实例路径 + 局部稳定 ID；不以向量下标或显示名称作为跨实例引用 |
| ModelFragment | 刚体、marker、joint、motion、force、tire、端口、输出及未绑定连接需求；所有权明确 |
| Assembly | 被测对象的已连接片段与外部端口；可嵌套，展开时保留来源 |
| SimulationAssembly | Assembly + Rig 及连接实体；所有必需引用完成、策略校验通过；可独立检查和序列化 |
| Provenance | 模板标识/修订、参数/属性指纹、实体来源和绑定结果；不改变旧公开结果格式，新增记录先放新请求内部元数据 |

内部物理量统一 SI，世界/刚体/marker 坐标系必须显式。输入毫米制由边界适配一次；历史毫米制输出由对应结果适配一次。不得在 Study 中藏单位转换。

端口挂在拥有它的刚体上，存局部位置/姿态。空间代数沿用项目四元数和轴约定；02 将现有约定与单位测试固定，不另造第二套约定。

硬点修改 -> 重新实例化受影响子系统 -> 重算端口与试验台 -> 重编译；不复用旧坐标缓存。仿真过程中端口随所属刚体运动，不逐步重新装配或瞬移试验台。相同参数重建必须确定性；无需实现增量构建缓存。

## 5. 端口、自适应与连接

端口至少包含：稳定 ID、所属实体、语义角色、局部坐标系、侧/轴等适用标签、能力和允许连接数量。几何端口与信号/力矩通道分型，不能把名字匹配当作物理兼容。

需求包含：期望角色/能力、标签、连接数量、required/optional、可选分支关联的激励与输出。只有明确声明 optional 的分支可整体消失；必需试验不能自动降级。

绑定顺序：显式端口映射优先；否则按语义/能力/标签筛选；零候选按必需性处理；多个候选报歧义并列出端口，禁止按名称相似度或距离悄悄选择。匹配结果保存用于追踪。

几何适配：由被测端口的世界变换与试验台模板的局部安装变换求试验台实例位姿；多安装点由模板声明对应的几何构建规则。非刚性匹配不能用一个刚体变换强行拟合；模板无法满足时明确报几何不兼容。

连接所有权：内部连接由实例拥有，跨实例 joint/motion/force 由组合阶段创建并拥有，ID 由稳定绑定标识生成。连接不会复制被测刚体或轮胎质量；共享 ground 通过显式引用归一化。

检查包含：不存在的实体、错误单位/坐标系、重复 ID、同一自由度重复驱动、端口超额连接、遗漏必需需求；完整约束秩由已有内核 audit 检查，作者层不冒充通用运动学证明器。

## 6. 全局总成规则与实体归属

本节遵循本轮 D3，不可被模板、配方、嵌套包装或用户注册覆盖。policy 按根总成类别判断；整车中的前后轴是内部片段，不误当成独立单轴仿真重复应用单轴轮归属。

| 根类别 | 被测总成 | 试验台 | 禁止/必需 |
|---|---|---|---|
| 单轴悬架 | 左右悬架、固定车身支承、可选转向；不自建车轮子系统 | 提供车轮/轮胎及支承加载测量所需实体 | 禁制动与驱动子系统；缺转向时仅删除声明为可选的 rack 分支 |
| 整车 | 前后悬架、车身、车轮、转向、制动、驱动 | 引用车辆轮端或接地点，可提供加载装置但不再创建一份车辆车轮 | 转向/制动/驱动必须存在；车轮由被测总成拥有 |

规则在组合预检和完整展开后复检，检查实体来源与角色而非仅检查顶层角色字符串。禁止通过把被禁止子系统包进 Rig 绕过；试验所需一般加载力仍按声明的试验激励处理，不等同于新增车辆驱动子系统。

车轮包含轮体及轮胎引用，不代表二者质量可以重复。轮胎质量由 tire 拥有；凝聚到等效惯量时保留来源并只累计一次。偏心质量等现有未支持物理保持拒绝。

同一 Rig 模板可通过声明的“提供车轮/使用被测车轮”能力分支适应两类根总成；分支由全局 policy 与端口能力决定，不按具体悬架模板名决定。新试验台与新拓扑均不得扩充这个规则范围。

## 7. 请求、Study 与契约编译

新请求逻辑组成：`assembly_spec + rig_spec + study + case + outputs`。稳定运行入口接收显式 SimulationAssembly 时不重新构建模型；family 是内部契约兼容路由，不是试验台身份，不要求同名。

原有 run_case/run_dynamic_case/run_axle_dynamics/run_vehicle_dynamics 及 CLI 作为薄适配器，将旧模型/工况转换到统一流水线；原调用方式与返回类型保持。注册表按已编译的求解问题能力匹配，未知组合明确拒绝，不靠模板名称写 if/elif。

模型契约来自同一 SI 图；准静态/动态的时间安排、求解方式和轮胎激活属于求解计划。相同 SimulationAssembly 的结构/参数指纹不随 Study 变动；派生运行 payload 可包含不同执行投影，并记录它如何来自原模型，禁止改写原模型。

准静态是独立平衡态集合，动态有历史状态；不要求两者结果数值相等。准静态接受现有 fiala/pac2002/native_brush 的垂向退化，必须真实进入求解，不能只在 Python 声明存在。先利用现有原生能力；若当前协议不能无歧义表达激活，07 必须提出协议变更供确认，不能私自换成另一个力律或静默丢弃轮胎。

failure/partial result、输出布局、采样顺序与历史元数据由显式结果适配保持，不把旧 family 分支散落回 API。

## 8. C++ 注册与状态职责

08 分离“协议已知类型”与“本构建可执行能力”。工况描述表提供名称、实现状态、展开函数；支持查询与分派由同表得出。协议层不能为了查询能力反向依赖求解器。Python 依据 native capabilities 与契约版本校验，不再手写第二份可执行清单。

对关节、力元、轮胎逐类绑定到已有处理器/描述数据；通过一致性测试保证登记、解析、标量与方向导数路径完整。保留静态注册与直接计算路径，不引入运行时动态库插件或强制所有元件虚函数化。

09 只提取已有状态行为：状态布局、初始值、局部残差/方向导数、接受步投影、接触事件与精确松弛处理。所属 tire/element 模块实现这些行为，assembly 将现有类型接入状态块执行接口；solve 层掌握迭代与步进，不直接判断 PAC2002 枚举或遍历专用状态数组。

物理参数仍可为编译期具体类型；不为“通用”抹掉类型信息或强加 type-erasure。首先拆分大型类型头，保留共享类型的低层归属；状态操作的接口头只依赖基础值类型，避免制造 model/tire/solve 环。

标量与方向导数成对迁移；保留状态顺序、步进、接触事件和收敛标准。新增性能开销必须用 01 基线验证，不能用改变收敛容差抵消。

## 9. 输出与兼容迁移

ModelFragment 提供最小输出声明，组合时命名空间化并绑定真实实体。相同语义量来源有冲突时拒绝，不取最后一次覆盖。Rig 的缺席分支同步删激励和输出；结果不能伪造 0。

results 负责原生列布局/单位/失败解码，outputs 负责绑定与派生计算，report 消费输出，api 只编排。Python 现存 elements/analysis 若因固定体端反力或静力能力仍有生产用途，先集中在具名兼容计算适配中并登记条件，不能直接删除或声称 native 已覆盖。

| 当前范围 | 目标归属 | 完成条件 |
|---|---|---|
| preparation.assembly.types / core 空间类型 | modeling.primitives | 单独导入不闭环，所有生产调用迁移 |
| templates / subsystems | 同名目录演进 | 模板真正生成可编译实体，非只传属性 |
| rigs 声明 / compose | rigs 模板 + connections | 不只是 family 改名；物理实体进入 native |
| studies.bridge | 输入适配 + modeling + compilation | 不再有 K/C 模型转动态模型的生产路径 |
| cases/* 的模型发射 | compilation/* | 面向统一物理图，不按悬架拓扑分支 |
| simulation/compiler 与 dispatch | 保留编排、委派 compilation | 不维护 rig==family 的平行逻辑 |
| api 结果解码/报告/检查点 | results / report / io | 公共签名、历史结果及失败语义不变 |
| C++ registry / case_dispatch | 各职责层静态描述表 | 能力查询与实际执行一致，未知明确拒绝 |

保留公开兼容入口，不长期保留两套生产装配；内部旧路径仅在调用方迁移完成后删除。现有测试不得靠整批删改期望维持绿色。12 负责旧路径清单、文档与构建，13 再独立验证。

## 10. 验收与实施约束

详见 EPIC A1-A10 和 TASKS 各任务。新测试路径是计划交付物，不是声称已存在；当前本轮仅做文档一致性验证。

冻结接口先行，默认串行主线。08 可在 01 后独立推进，但 09 涉及共用 C++ 和契约，禁止与其他触及这些文件的写任务并行；CSV 依赖仅描述逻辑前置，不自动授权并发。

不确定性处理：01 冻结实测值；07 若发现协议表达不足提问；09 若发现算法必须改变提问；其他未涉及业务语义的局部命名/文件切分由实现者依现有风格决定，并同步文档，不重新引入抽象层。

## 11. 迁移切换时点

06 先产出并验证统一 SI 装配入口，旧公共运行入口在编译器未接通前保持可用；07 完成新编译链路后切换生产入口并消除旧桥接依赖；10 收敛 API 与结果职责；12 仅删除已无调用的内部文件。中间兼容路径有明确退役任务，不能演变为永久双实现。
