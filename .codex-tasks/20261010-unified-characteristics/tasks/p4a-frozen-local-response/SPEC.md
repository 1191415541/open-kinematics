# P4a：原动态样本冻结态刚度、柔度与派生指标

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：在原动态工作样本实现 frozen_force_tangent 和 frozen_load_compliance，按具名载荷/测量直接输出局部响应及 Wheel/Ride/Roll/Toe/Camber 序列。
- 直接依赖：P2;P3；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §3.4、§5.3、§6.5–§6.7、§8–§9、§11。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P2 完整物理切线/多 RHS/转置/指标，P3完整一致接受态；P0 native冻结/profile与目标力律预算。
- Adams 分支只有规则审计通过才可启用；native定义验证不需要 P4b或 Adams动态规则推定。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- mb_linearization physical/response 工作样本与冻结协议处理、请求合并/缓存/native 接受态调度。
- characteristics/compliance.py 指标依赖与映射、capability 力律审计、结果诊断/来源；复用 P2算法，不造动态指标副本。
- R8共享物理片段与R9去重；可选矩阵视图沿同契约独立登记。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- frozen_inertial_load_v1：固定 world 速度/内部状态/未扰动输入/活动分支和质心惯性 wrench 的可审计协议。
- 坐标驱动 frozen_force_tangent 输出外部功共轭阻止位移反力导数及分量单位。
- 载荷驱动 frozen_load_compliance / local_response_v1：任意具名轮端模式和测量数量，完整整车响应及耦合。
- 逐原样本 Wheel/Ride/Suspension Roll/Total Roll/载荷 Toe/Camber、有效性/残量/秩/来源及实际策略/成本；几何序列复用 P3。
- 同冻结独立有限扰动、解析/非对称/奇异组合及直接/转置验证证据；P4a 核心独立完成记录。

## 实现规则

- 惯性快照来自该样本质量/惯量/线加速度/角速度/角加速度/陀螺项；固定每刚体质心处world六分量wrench，作用点随材料质心移动。
- 广义载荷 B_com(q)^T w_inertia* 对 B_com求导，不把广义协向量数值固定，也不重算惯性 wrench；body 固定速度等变化需另定义版本。
- 含负惯性载荷的原样本机械残量必须合格；lambda基值取原态，delta lambda联立求，不默认冻结。
- force tangent 坐标驱动与 load compliance 载荷驱动分别组装边界，不能用倒数/刚度矩阵求逆代替不同边界。
- 局部载荷不重新配平、不时间传播；非平衡点不接受 equilibrium。描述符/离散Newton/轨迹F/s/邻点斜率不能验收。
- A X=B_w P，Y=G X+D P，直接按同相/反相等模式求；整车 full model 提取前后/车身，保留后轴/转向/反作用力臂与移动参考体。
- 按实际组合模式检查相容/唯一性：组合有效而单列无效仍应给有效指标；matrix_view才逐请求单位基列审计，未求项不填零。
- 事件/接触/摩擦/限位/历史力律逐种审计，非光滑报告 nonsmooth；不重放return mapping/重置内部状态。
- 独立 oracle 只在隔离副本，同速度/内部状态/惯性wrench/分支/边界/反作用位置；不提交生产状态。
- 与P2共用指标公式/合并分解，矩阵适配、P4b、P7都不是核心能力前置。

## 执行步骤

1. 实现冻结协议和原样本残量审计：惯性wrench/world速度/材料点/乘子规则可验证，目标力律能力准确。证据/落点：tests/characteristics/test_frozen_protocol.py（拟新增）。
2. 实现坐标驱动的冻结态反力切线：功共轭反力与预载/参考体/端口导数完整，K=k解析通过。证据/落点：tests/characteristics/test_frozen_force_tangent.py（拟新增）。
3. 实现具名载荷模式响应及可辨识诊断：C响应不重配平，组合奇异正确，完整整车耦合保留。证据/落点：tests/characteristics/test_frozen_load_response.py（拟新增）。
4. 贯通逐样本派生指标与请求共享：实际Wheel/Ride/Roll/Toe/Camber序列；同A分解共享、来源可追溯。证据/落点：动态指标/缓存专项测试。
5. 核对独立有限扰动和直接/转置结果：同冻结oracle步长序列、非对称/单位/frame等价，原状态无修改。证据/落点：raw/frozen-oracle-report.md。
6. 验证四组合覆盖及成本并归档：P0动态目标不缩减，矩阵capability独立，R8/R9阶段证据与门通过。证据/落点：raw/coverage.csv；raw/validation.md。

## 验收条件

- [ ] 动态线性质量弹簧阻尼固定速度/惯性时 K=k、C=1/k；左右防倾杆、预载、轴间耦合与移动参考体通过。
- [ ] 实际原样本逐点响应及核心指标输出，冻结协议/残量/状态不变明确，独立于 P4b。
- [ ] 固定/自由车身、转向/其他轮端、轮胎/力元范围及反作用载荷均匹配profile；不重复计轮胎。
- [ ] 组合模式相容/唯一性、直接/转置同冻结oracle、预算缓存失效通过；Adams未核实仍pending。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- 线性动态K=k/C=1/k、预载几何项、world速度固定、COM惯性wrench材料作用点导数。
- 左右交叉/整车前后与车身耦合；固定/自由移动参考体、转向hold/release、载荷反作用力臂。
- 单列不相容但同相/反相组合相容、测量不唯一、轮胎卸载/限位/摩擦/历史分支不可微。
- 直接/真实转置/全基等价（仅全基可定义时），同冻结有限扰动、没有请求不变与开启后轨迹不变。

## 验证命令与证据

未来实施时在仓库根执行，每条检查退出码，失败即停止放行；不能把这些命令记成规划阶段已执行：

```powershell
just build-kernel
uv run --no-sync pytest packages/suspension_multibody/tests/characteristics -q -p no:cacheprovider
just check-fast
just gate-numeric
git diff --check
```



相关新增专项断言必须覆盖上列场景，不能以旧测试全过代替新能力验收。

## 风险与阻断

动态状态及惯性冻结表示差异会改变物理定义，必须逐力律、逐参考体验证；未审计力律编译拒绝，但不能从P0核心目标清单删除来通过发布。
- 不依赖缺口的产物可继续推进，但任务不标DONE。范围/依赖或验收策略变更记录于PROGRESS并复核父依赖。

## 公共约束与环境

- 形态 single-full；当前执行状态以本任务 TODO.csv/PROGRESS.md 为准。实施状态、原始证据与未实现能力须如实同步，不能把规划目标描述为已存在能力。
- 项目根 E:/杂件/open-kinematics；Python >=3.12、C++/CMake+Ninja、uv、pytest、ruff/ty/AST。
- 恢复先读父 EPIC/SUBTASKS/PROGRESS、本 SPEC/TODO/PROGRESS；每次进入新步骤重读当前 CSV。依赖验收证据未通过不得标 DONE。
- 唯一模型→计划→编译→native→结果链；特性是输出请求，不造 Study/Case family/第二入口。characteristics 只依赖低层数据/空间代数。
- 只改直接范围；无新力律/依赖/频响/二阶导数/硬点优化，不清理无关死代码，不重录冻结基线，不增长 skip/xfail，不削弱断言。
- 子代理只做只读探索/核验，主代理负责修改/取舍/最终验证；写代码前完整阅读确切代码。
- 未知实体/字段/版本/单位/不支持能力/矛盾边界编译拒绝。物理无效用固定形状缺失值及原因，不填零/截断/复用前点。
- 样本状态 valid/unidentifiable/singular/nonsmooth/not_equilibrium/inconsistent_sample/undefined_geometry；缺运行时通道是查询错误。缺实现/核心状态/参考不是物理退化。
- TODO 的 acceptance_criteria 是行为判定；命令通过/证据文件存在不足以单独标 DONE。raw/validation.md 必须记录实际行为断言、命令退出码、误差/预算、缺口。
- 新 tests/characteristics/ 自动纳入 check-fast；kernel/contracts 仍分开调用，不混 rootdir。
- 日常 just check-fast（依当前justfile，含ruff/ty/架构门及附加旧模型测试、快速集/另外两包）；不默认慢测试。结构变动必跑AST门；native/求解变化先构建再gate-numeric。
- 连续三次同类失败暂停重评，保留FAILED和retry_count；不静默放宽容差/弱化验证。P0预算与容差后续不得为通过而放宽。
- 影响结果/数据/范围的歧义、新依赖、商业/外部服务、核心文件删除、高风险Git按用户确认边界处理；规格不构成这些操作授权。
