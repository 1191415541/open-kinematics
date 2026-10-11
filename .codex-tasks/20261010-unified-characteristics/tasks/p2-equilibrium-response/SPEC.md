# P2：共享物理切线、平衡响应与指标

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：提取共享物理线性化；实现可复用分解、多右端/转置及平衡响应服务，从完整模型按需生成 Wheel/Ride/Roll/载荷角度指标。
- 直接依赖：P1；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §5.2–§5.4、§6.2、§6.6–§6.7、§8、§10 R7–R9。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P0 定义/profile/预算/backend 能力与 DAG；P1 统一测量、native 响应边界、查询/持久化。
- 现有 static_residual/static_jacobian、LuFactorization 等单 RHS 分解对象；现有 Newton 语义类型不可作为服务公共输入。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- 拟新增 kernel cpp/include/mb_linearization/{types,functions}.hpp、src/linearization/{physical,response}.cpp；mb_linear 通用接口/分解能力及必要构建/DAG 清单。
- cpp/src/solve_static/kernel_static_trim.cpp 的 static_residual/static_jacobian 共享物理部分，保留 solver 适配及策略。
- characteristics/compliance.py、registry.py 与编译响应依赖合并；结果沿用 P0/P1 通道。
- R7/R9 迁移台账和 R8 精确候选共享片段；不更改本构规则。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- mb_linearization 单一物理组装与工作样本/边界接口，solver/ABI/输出向下调用；无 solver 反向依赖。
- 通用 analyze_pattern/factorize_numeric/solve_many/solve_transpose_many 的真实能力；不支持转置的后端记录等价前向策略原因。
- 平衡 A X=B_w P、Y=G X+D P；具名载荷/测量模式可合并、精确线性相关模式可重构，保留完整整车耦合。
- Wheel 同相、Ride 完整接地点/可辨识串联分解、Suspension/Total Roll 反相及实际角度/力臂、载荷 Toe/Camber 指标。
- 独立特性缓存与计量；可选矩阵视图有独立 capability/逐列有效性，不是核心前置；R7/R9 委托记录及 R8 精确片段清单。

## 实现规则

- R_x 包含力律导数、接触/内部变量与 d(J^T lambda) 预载几何项；最终收敛态重新评估，不能复用上一轮 Newton 切线。
- static 的 load_scale/行选择/gauge/pin/活动集/尺度仍归 solver；服务不引用 ResidualContext/NewtonLinearizationCache。
- 自由车身与固定车身不同 profile，full model 先组装再提取；Schur 仅在子块可逆、同边界等价时使用。
- 奇异系统按实际 RHS 相容和 gN=0 测量唯一性分别返回 singular/unidentifiable；世界 pin 不得掩盖刚体模态。
- 同 A 数值分解一次；跨样本只复用经核实不变的结构分析，事件/边界/布局/非零模式变化失效；计量实际 RHS/投影。
- 转置实际解 A^T，含尺度/置换还原，非对称系统不能假设对称；后端选择不改变物理 kind/version。
- Wheel=1/h_left、1/h_right（heave_load）；不拿 Kzz 或单轮柔度替代。Ride 不二次串联轮胎；Roll 实际角度/力臂，N·m/rad 转 N·m/deg 乘 pi/180。
- 载荷角度使用 G_angle，不把 rx/ry/rz 直接当 Toe/Camber；负切线保留与稳定性诊断。
- R7 原处只委托新共享组装；R9 只治理新增特性路径的重复，不移除其他用途 solve_linear。

## 执行步骤

1. 提取共享物理组装并更新 DAG：完整物理切线唯一归属，solver 策略保留；无力律复制/环。证据/落点：physical.cpp；raw/module-dag.md；R7证据。
2. 实现可复用多 RHS 与真实转置接口：非对称、缩放/置换、多 RHS 回代准确；backend 能力真实。证据/落点：kernel 线性后端专项测试。
3. 实现平衡模式响应和可辨识审计：A X=B_w P 与 G/D映射保留整车耦合，奇异按模式/测量判定。证据/落点：tests/characteristics/test_equilibrium_response.py（拟新增）。
4. 实现 Wheel/Ride/Roll 与载荷角度纯算子：同相/反相载荷正确；轮胎不重计、移动参考体与单位正确。证据/落点：tests/characteristics/test_compliance_metrics.py（拟新增）。
5. 实现兼容请求共享缓存和计量：同 A分解一次，结构/数值失效正确，精确重构有来源。证据/落点：tests/characteristics/test_response_cache.py（拟新增）。
6. 完成独立 oracle、R7/R9及R8边界锁定：直接/转置/全基/有限扰动等价；R8列精确符号/片段。证据/落点：raw/retirement-evidence.md；raw/validation.md。

## 验收条件

- [ ] 线性系统 K=k、C=1/k；防倾杆左右交叉、预载非线性、固定/自由车身、车身/轴间耦合通过。
- [ ] 直接模式/转置/独立全单位基投影在 P0 容差内等价，含非对称与单位/frame；奇异组合单独验收。
- [ ] 同 A分解共享及失效次数符合 P0 预算，组装/分解/回代/序列化/内存/体积分项可追溯。
- [ ] R7 移出部分唯一归属且旧 solver 委托，R8候选片段精确锁定，R9新增路径不重复求解/分解。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- 解析线性/防倾杆/预载系统、有限扰动步长序列，保守且功共轭时互易性；非保守切线不强制对称。
- 单列不相容而组合模式相容、测量对零空间不变、gauge 请求冲突拒绝。
- 固定/自由车身/后轴/转向/轮胎范围、实际反作用位置与随动 frame；Ride 串联条件成立/不成立。
- 同点跨指标合并、不同域/边界禁止复用、跨点新数值分解、事件/结构失效与原请求无变化。

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

共享提取与线性后端变化属于求解路径，必须以冻结门证明原路径不变。现有后端是否支持转置要逐个验证；不得把同尺寸解 A 当成解 A^T。
- 不依赖缺口的产物可继续推进，但任务不标DONE。范围/依赖或验收策略变更记录于PROGRESS并复核父依赖。

## 公共约束与环境

- 形态 single-full；当前各步骤 TODO。用户本轮只要求规划落盘，未启动产品实施；未来测试/新文件不是当前已经存在的能力。
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
