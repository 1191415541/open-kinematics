# 通用子系统与唯一仿真路径方案

状态：方案已通过独立审查；未开始源码实施。日期：2026-10-06。

Taskmaster 真源：`.codex-tasks/20261006-unified-generic-subsystems/SUBTASKS.csv`，含 10 个子任务、34 个叶步骤；审查见该目录 `planning/review.md`。本方案是上轮通用多体演进之后的新重构目标，不改写上轮已完成工作的历史记录。

## 1. 交付目标

所有模型都使用 Template -> Subsystem -> Assembly 组织，包括悬架、转向、车身、车轮、制动、驱动和试验台。Python 对象与文件只在加载来源上不同，生产装配、准备、编译、提交和结果解码全部统一。

```text
文件路径 / Python 文档对象
  -> Document Loader + Resource Manifest
  -> 通用 Resolver：实例、镜像、单位、属性、端口和拓扑变体
  -> ResolvedModel：唯一 SI 实体图
  -> Study + Case：初始状态、激励、边界条件和输出计划
  -> CompiledSimulation：统一版本化 native 契约
  -> NativeContractBackend
  -> ResultEnvelope + Measurement
```

沿用当前 `simulate()`、`CompiledSimulation`、`run_compiled()` 和现有 native solver，不另起一套 Simulator 框架。`ResolvedModel` 是当前 `ModelView`/实体图的统一替代职责，不再与多个 runtime 同时成为权威模型。

## 2. 子系统边界

| 对象 | 声明内容 |
|---|---|
| Template | 通用实体、坐标系、连接、力元、属性槽、端口、测量和拓扑变体 |
| Subsystem | 模板的几何、惯性、属性绑定与实例参数 |
| Assembly | 任意数量的子系统实例、安装变换与显式端口连接，可嵌套并确定性展开 |
| Rig | 普通子系统的试验用途，能含支座、平台、作动器、传感器及其端口 |
| Study / Case | 求解方式、边界、初值、激励、采样和所请求测量 |
| Property | 参数、单位、模型版本、曲线及资源来源 |

- `wheel` 子系统包含 wheel 刚体与 tire 力元、TIR 属性、安装/接触/道路接口。总成不另装配与 wheel 平级的 tire law 实例。
- `brake` 与 `drive` 是普通子系统，内部声明力矩元素、公式、属性和控制/机械接口；允许零刚体的简化模板，也允许带 rotor/caliper/shaft 的详细模板。
- 力律是内部力元的计算规则，内部运行状态归具体元素，不替代用户建模的 subsystem 层级。
- wheel 的惯性、详细 brake 的转子惯性不得重复。总质量、质心、完整空间惯量均须有唯一归属。
- `role` 和产品名称只用于检索、分类与可选总成策略；通用装配器不得据此创建实体。非汽车机构无需伪装成 axle/vehicle。

## 3. 唯一解析后模型和编译

`ResolvedModel` 使用稳定实体 ID 和 SI，包含 bodies、frames、joints、elements、element properties、ports、signals、measurements、初始状态，以及模板/实例/资源来源。引用必须指向 ID，不能靠 wheel 名字、L/R 后缀或第几个轮胎决定归属。

`SolvePlan` 只保存本次求解事实，包括 study、边界条件、求解设置、采样与输出选择；不携带另一份 dynamic_model 或业务 runtime。

文件与内存输入：`simulate(assembly_or_path, case_or_path)`。case 路径也由同一 loader 读取。相对引用以显式资源根解析；纯内存对象在引用绑定完成后可脱离原建模文件运行。资源 manifest 固定实际消费的引用、版本与内容哈希，拒绝歧义；本轮沿用现有项目资源机制，不建立远程包管理器。

`validate` 与 `simulate` 使用同一个 compile 路径；前者止于成功生成 `CompiledSimulation`，后者再提交，避免 validate 通过但运行在不同入口被拒绝。

family 若仍为内核协议所需，只允许在 native emitter 映射；作者接口用 study 和输入语义描述求解。所有已注册汽车分析先解析为普通激励和测量数据再编译，不保留车辆 preparation 私自重建实体。新 emitter 只能按契约能力选择，不能按车辆角色选择。

## 4. 拓扑变体与研究方式

K/C 是已有模型的连接配置选项，例如理想运动副或柔性衬套；不等同于运动学、准静态或动态 study。同一配置能运行不同 study，台架和整车使用同一组选中的子系统定义。

通用变体用显式激活/参数数据，不把 K/C 写成内核强制的两种模型类别。现有 K/C 表达可作为文档层的命名配置，编译后只剩实际实体。

轮胎全力律、垂向激活或停用必须显式请求，不因 `quasi_static` 自动停用全部纵向/侧向力。Study 不自动固定所有转动坐标，也不自动把欠约束机构焊接。

## 5. Wheel 自转与同一悬架定义

轮端模板或装配连接明确声明 wheel 与非自转承载件之间的轴承转动副，整图只出现一次。转动副归属由模板定义：可以在悬架中声明 spindle bearing，wheel 固定到 spindle；也可以在 wheel mount 连接中生成 carrier-wheel bearing。禁止装配器自动补第二份轴承。

`Spin Port` 指向该转动副的相对坐标 q 和正方向，而不是 wheel 的世界姿态。两端 frame、局部轴、参考相位、单位和 source joint 都显式声明；镜像时对旋转轴按轴向量变换。

| 边界 | 语义 |
|---|---|
| free | 该坐标不附加运动约束，保留转动惯性 |
| locked | q = q_ref，只增加一个独立标量约束 |
| prescribed angle | q = f(t)，函数与初始化位置/速度一致 |
| prescribed speed | 仅在 native 有真实速度约束支持时使用，否则明确拒绝，不能用每步冻结转角冒充 |

`torque` 是 load，不是与 free/locked 并列的边界模式。锁止或指定运动与转矩可以共存，求解器必须输出反力矩；重复或矛盾运动约束、重复施力和无法解析的坐标应拒绝。

悬架 K/C 台架通过明确端口固定轴承相对 q。锁止不会固定转向、外倾或悬架跳动。车轮与承载件的绝对姿态可随悬架改变。整车动态对同一端口不附加 spin 约束，brake/drive 子系统通过机械端口施加力矩。

Tire 的接触 frame 跟随承载件或由明确非自转 frame 派生；轮体 spin 状态仍进入滑移计算。承载件可以转向/外倾，所谓非自转只是不跟随轴承相对 spin。不能把接触 frame 绑定到固定世界坐标来消除自转。

native 支持必须涵盖位置残差、速度一致性、Scalar/Dual Jacobian、初始化、动态 trial/输出时刻和约束反力；保留超过一周的角度/参考相位一致性，不隐藏更新状态于残差回调。确实需要新状态或契约字段时更新版本/capability 和 ABI 单一真源。

## 6. 普通台架和端口连接

被测总成唯一声明 wheel 子系统，台架不创建替代 wheel/tire。实验中的支撑盘或平台可以是 rig 自有刚体，通过普通关节/接触连接，它们不冒用 wheel 身份。

台架提供明确的 wheel-center motion/load、spin boundary、road 和 measurement 接口，连接表绑定端口 ID。实体安装变换、两端局部 frame、轴和输入单位由通用 resolver 处理。可选端口的裁剪规则必须随数据声明，同步裁剪其激励、耦合项与输出；缺必需端口不静默删除试验。

固定连接只生成显式约束并保留质量。质量凝聚仅可作为通用可选优化，在质量/质心/空间惯量、端口与结果 ID 映射均可证明守恒时执行，禁止按 wheel 或 chassis 角色特殊合并。

四立柱沿用当前理想道路激励，普通 Rig 能表达它但不伪造平台刚体动力学。实体平台、接触和作动器的新增物理需要独立选择和验证，不由这次路径收口暗中增加。

## 7. 力元和状态

弹簧、阻尼、衬套、tire、制动、驱动等元素通过 primitive/model/version 适配已有 native 模型。复用现有受限 AST、单位校验、函数程序和 Scalar/Dual，保留现有力律及乘法/缩放行为，不把重构变成力律重写。

无状态力元只做纯求值；已有有状态模型复用 solver 的 trial/accepted-step 机制。初始化、力/Jacobian 求值、接受步提交和拒步回滚必须明确，并能按元素 ID 验证实例隔离。本轮不引入通用任意状态方程、用户 DLL 或新的控制器物理。

内部元素具有双端 action/reaction、reference frame、力臂搬运、功率和输出身份。外载必须明确 ground 反作用边界，锁止力矩仍计入约束反力及功率账。

## 8. 统一结果

沿用当前原始结果/提交对象，收敛为一个 ResultEnvelope：metadata、model/resource fingerprints、body/joint state、constraint reactions、element wrench、element state、measurements、diagnostics。

只允许按结果 schema/channel version 解码，不按 generic/axle/vehicle 选择解码流程。Toe、camber、轮荷、滚心等为声明式测量或独立结果查询，显式引用 frame/实体/单位；可以保留结果读取便利类，但不得触发专有建模或第二次物理解码。

## 9. 生产切换和删除

实施过程中允许保存旧模型快照、独立测试 oracle 和迁移器，不新增或延长兼容生产路由。统一路径完成前不发布半迁移接口；完成后一次切换公开入口并删除旧可达执行链。

删除对象按调用证据逐一确认：`presets.legacy` 的运行时转换、`si_assembly`/vehicle 专用实体创建、`rig_link` 名称推断、`compile_generic`/传统准备分支、`simulate_generic` 公开旁路、业务 decoder 分派。相关模块若还有已迁移后仍有效的通用定义，先搬到已有职责层，再删除旧构建器；不能只改名保留实体算法。

所有源码、CLI、Adams 导入/对标、脚本、公开导出和文档示例都要更新调用；Adams 源导入也是新文档的作者输入，不得绕过 resolver 直接构造业务 runtime。离线 v1 迁移器只能输出文档，无 submit/solver 导入。

## 10. 验收与物理变更边界

- 同一 Python 与文件输入的有效模型指纹、编译文档和结果通道一致，不能只比较运行成功。
- 同一悬架/wheel 子系统在台架和整车的安装无关子图指纹相同；安装/边界不同不要求整图指纹相同。
- locked spin 不漂移并保留转向、外倾、跳动；free spin 可滚动；locked + torque 的约束反力平衡；多周旋转及左右镜像符号正确。
- 显式相对角约束与接触 frame 的 Scalar/Dual 和有限差分误差门沿用现有 <= 1e-6 归一化阈值；质量/反力/功率误差采用现有物理门并在测试说明记录。
- 全部已注册 study/bench 的旧可观察输出逐项映射到新测量，包含 K/C 网格、耦合/裁剪、handling/ride、dynamic 和 comparison。
- 对物理等价的迁移继续执行冻结逐位、族 parity 和性能门；新增 spin 边界若纠正了旧物理模型，另存新场景证据及影响清单，不重录原冻结基线，不宣称不同物理模型逐位相同。发布默认值或基线更改需另行明确授权；本方案先保持旧等价场景和新显式 spin 场景可区分。
- 检查生产调用 DAG 和实际 compiled metadata，证明唯一 resolver/compiler/submit/decode；增加退役 AST 正反例，禁止新旧分叉复活。

用户要求跳过无关测试：本轮规划只自检文档/CSV/审查，不运行求解回归。实施时按 `planning/validation.md` 的相关性矩阵选已有和新增测试；数值/性能与分层门保留，不默认重复全量 pytest，不在实施期使用 -k/--deselect 掩盖失败。
