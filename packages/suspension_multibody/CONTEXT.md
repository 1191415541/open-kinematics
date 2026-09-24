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
