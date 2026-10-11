# P5：Adams Car 2024.1 SVC 逐项独立对标

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：用独立 Adams Car 2024.1 参考逐样本对标核心四组合局部响应与指标，仅证据齐全且误差通过的条目变为 validated。
- 直接依赖：P0;P1;P2;P3;P4a；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §3.2–§3.4、§6.6–§6.7、§11.1。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P0核心目录、预锁容差和六组Adams规则审计；P1–P4a实际四组合数值。
- 独立同完整模型/边界/样本/内部状态/载荷反作用协议的Adams参考；数据取得和许可使用尚未获本轮授权。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- 既有 suspension_multibody/adams 验证工具及 tests/adams 中新增SVC定向验收，统一特性查询/数据投影只读消费。
- 本任务 raw/ 保存官方审计、参考指纹、样本映射、转换、完整误差分布和报告；不改项目冻结数值/性能基线。
- 响应/指标/冻结差异需要另一kind/version时回写P0契约与相关实现并记录，不覆盖native协议。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- raw/compatibility.csv：metric_id/kind/version、response/frozen/profile、load/measurement模式、官方出处、模型/力律范围、采样映射、单位符号、容差、reference fingerprint、状态/报告位置。
- raw/reference-manifest.json及独立原始参考：来源、生成方式、文件指纹、完整模型与模式/反作用位置、状态/事件分支一致性。
- 逐样本/分量响应、交叉项与Wheel/Ride/Roll/角度/几何量差异及误差分布；无效性/退化原因也比较。
- matrix_view 若启用，单独登记其布局及显式行/列验收；不要求核心先输出完整12×12。
- Adams与native差异分析及发布阻断清单；核心未知项未解决时任务不DONE。

## 实现规则

- P0六组规则都须完成核实才启用Adams兼容局部柔度路径；规则不同另设kind/version，不推定frozen_inertial_load_v1一致。
- 参考须独立且同完整物理模型，不以项目自产数据/旧冻结快照与自身比较代替。
- 动态匹配原求解样本/事件，不用新配平态、连续传播或跨事件插值替代。
- Adams矩阵可投影到同模式/测量基再比较，完整矩阵表示不是核心定义；不能漏验交叉效果或核心指标。
- 绝对/相对容差在P0按尺度锁定，不按P5现有误差放宽。比较全部目标点/分量，不能只取均值或正常子集。
- 未核实为pending_definition、缺参考为pending_reference；两者都阻断完整核心Adams/P6发布，与样本物理valid独立。
- 读取本地help/已存在参考直接执行；启动商业软件、许可服务或生成外部数据需按用户规则先确认。不能把数据缺口写成已验证。

## 执行步骤

1. 关闭Adams定义审计缺口：六组规则有官方出处/匹配实验与指纹，差异定义单列。证据/落点：raw/adams-rule-audit.csv。
2. 整理独立参考及同模型采样映射：参考来源独立、模型/profile/冻结/事件与模式匹配；缺数据显式阻断。证据/落点：raw/reference-manifest.json。
3. 实现模式/测量基与单位符号转换：按各分量正确转换，Toe/Camber参数化/track/roll角不猜测。证据/落点：SVC参考投影专项测试。
4. 执行四组合逐点响应/核心指标比较：包含交叉效果、全部目标样本/分量、有效性/原因及误差分布。证据/落点：raw/parity-report.md。
5. 逐项更新兼容状态并验证发布门：只通过条目标validated，核心pending阻断，矩阵/连续独立登记。证据/落点：raw/compatibility.csv；raw/validation.md。

## 验收条件

- [ ] P0全部核心目标仍保留，Adams官方规则/独立参考/指纹/转换/同样本同边界证据齐全。
- [ ] 四组合局部模式响应、交叉效果及核心指标在P0预锁容差内，事件/退化按真实分支比较。
- [ ] 未知或缺参考项不能标validated或DONE；不能用native独立验证解除Adams发布阻断。
- [ ] 可选矩阵仅验显式请求内容并独立记录；P4b不作为对标前置。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- Wheel同相两侧响应、Ride串联适用/完整接地点与轮胎不重计、Roll实际力臂/角度、角度载荷交叉项。
- Adams矩阵投影到原生模式、m/mm及rad/deg和N·m转换、左右符号/移动frame/全车边界。
- 四组合各目标样本、事件两侧、无效原因/退化，负切线与零响应处理。
- 参考缺失/指纹不匹配/冻结或边界不同/未知公式/超容差时发布门确实失败。

## 验证命令与证据

未来实施时在仓库根执行，每条检查退出码，失败即停止放行；不能把这些命令记成规划阶段已执行：

```powershell
uv run --no-sync pytest packages/suspension_multibody/tests/adams/test_characteristics_svc_parity.py -q -p no:cacheprovider
just check-fast
git diff --check
```

SVC对标测试文件为拟新增。若适配触及物理解算追加build-kernel与gate-numeric；独立参考缺失不得跳过核心对标。

相关新增专项断言必须覆盖上列场景，不能以旧测试全过代替新能力验收。

## 风险与阻断

本地help不足以核实全部动态规则且独立Adams数据尚未确认，是实际外部证据依赖。缺口未关闭时保持TODO/FAILED及pending，不缩目标，也不阻止native P1–P4a按自有协议验收。
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
