# P4b：连续动态描述符与状态/载荷扰动

## 任务与依赖

- 形态：single-full；发布归属：独立。
- 目标：独立交付 continuous_descriptor 的连续导数和测量映射，分别验证约束一致状态/载荷扰动，不把它作为 P4a/P5/P6前置。
- 直接依赖：P0;P3；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §3、§5、§6.4、§8、§9、§11。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P0连续力律协议/单独预算及请求定义，P3完整一致轨迹；通过P3已具备P2共享物理服务。
- 原连续方程 F(x,xdot,u,t)=0 及各力律真实状态演化；离散 Newton 导数不是输入。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- mb_linearization 连续物理方程方向导数接口，既有有状态力律连续能力审计。
- 请求/结果 kind/version/capability 和 named blocks；复用现有采样/查询/持久化，不造新积分器。
- 若落实有限时域传播，仅复用已存在积分路径并单列支持范围/初扰动/区间，不默认承诺全部传播。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- 请求方向 F_xdot、F_x、F_u 与测量映射，显式连续状态/约束坐标定义。
- 约束一致状态扰动和载荷扰动各自测试与支持表；有步长依赖/return mapping/缺连续方程的力律单列能力。
- 有限时域响应的实际交付范围声明：未实现时不宣称时间响应；实现时保存传播区间、初扰动、既有积分复用协议与误差证据。
- 独立预算/验证日志，不改变核心 P4a指标和发布依赖。

## 实现规则

- F_xdot delta xdot+F_x delta x+F_u delta u=0为连续系统，不一般性求质量/描述符逆。
- 不将广义alpha步长/前步状态/离散混合参数写入continuous_descriptor；连续/离散分别命名和证据。
- 具有已有方向导数不等于具有公开连续状态方程；逐力律真实capability，未知/不支持编译拒绝。
- 描述符导数只定义局部动态系统，不代表当前唯一位移响应；不能拿它替代冻结载荷柔度。
- 频域工具/新轨迹积分器不在本任务，未定义的时间传播不自行增加。
- 即使本任务FAILED/TODO，核心P5/P6可以在其各自条件满足时验收；整Epic仍要等所有已登记子任务。

## 执行步骤

1. 审计连续状态/力律和一致扰动坐标：每力律连续方程/方向导数/不支持原因明确。证据/落点：raw/continuous-capabilities.csv。
2. 实现连续描述符请求方向与测量映射：F_xdot/F_x/F_u来自连续物理方程，无离散系数泄漏。证据/落点：tests/characteristics/test_continuous_descriptor.py（拟新增）。
3. 分别验证约束一致状态和载荷扰动：解析系统/约束/内部状态审计通过，残量与单位完整。证据/落点：状态/载荷扰动专项测试。
4. 锁定有限时域实际支持范围：实现则验证初扰动/区间/原积分复用，未实现则明确无时间响应能力。证据/落点：raw/time-response-scope.md。
5. 独立性能与仓库门验收：预算独立；核心局部特性语义/原轨迹无变化。证据/落点：raw/validation.md。

## 验收条件

- [ ] 连续解析模型与约束一致性验证通过，状态扰动和载荷扰动能力分别登记。
- [ ] 内部状态、接触/限位/步长依赖逐力律审计，不虚报连续支持。
- [ ] 有限时域传播若列为交付必须真实运行验证；否则只有描述符能力，不声称时间响应。
- [ ] 本任务独立验收且没有成为P4a/P5/P6前置，P4a冻态响应定义不变。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- 质量弹簧阻尼连续描述符及独立有限扰动，带约束状态扰动/载荷扰动。
- 内部状态连续方程、接触/限位分支、仅有return mapping力律拒绝。
- 改变积分步长/generalized-alpha参数不应改变同物理连续导数定义。
- 实际传播若支持：相同初扰动/载荷历史/区间独立对照；未支持不得返回伪响应。

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

部分历史力律没有可用连续方程，可能只能明确拒绝。不能按现有离散方向导数宣称全覆盖；独立范围和预算由P0冻结。
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
