# P3：动态接受态调度、几何值与时间变化率

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：在原 native 一致接受态只读提取动态几何/输入导数/真实时间导数，提供 all_output、all_accepted、indices 的真实样本身份与序列。
- 直接依赖：P1;P2；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §3.1、§6.3、§7、§8、§9、§10 R8。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P1 几何与运动学算子、P2 物理服务/缓存；P0 核心动态网格、目标状态力律和完整状态预算。
- 原动态 State、输入、物理乘子、活动模式与 return mapping 提交时序；不可假设当前结果块无损保存所有状态。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- kernel cpp/src/abi/kernel_abi.cpp 的 state 提交/正式输出采样、kernel_contract_run.cpp 块布局/调度，必要 dynamic solver 物理原语接入。
- 统一 characteristics 测量/中心和 results/characteristics.py 采样解码；原生持久化保存不同网格映射。
- R8 由 P2 定位的物理调用片段接入；不抽出积分状态混合/步长系数/Newton策略。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- 输出/接受步/指定集合调度，初始一致样本 sample_kind=initial，试探拒绝步不产生正式特性。
- case/sample/time/state/mode identity，accepted-step/output ID和实际映射；两网格独立存储。
- 动态当前构型几何值/虚拟运动输入偏导、Toe/Camber/轮心等真实时间导数；显式 sampled_time_difference 中心迁移率。
- 一致状态审计、缺状态逐样本原因；P0 核心网格必须实际可用，系统性缺状态不能验收。
- 有/无特性请求的积分状态、能量、事件、接受/拒绝步序、通道布局与成本比较及 R8记录。

## 实现规则

- 接受且内部状态/乘子/活动模式同步后、下一积分步前只读提取；不执行新 return mapping/修改状态/推进样本。
- all_accepted 包含一致初始点与每个接受步；quasi_static 拒绝该选择；indices 显式指出样本集合和 ID。
- all_output 使用原采样机制提供完整一致状态；只有插值位姿时，切线/力律/反力相关请求标 inconsistent_sample，不换到邻近接受态。
- g_q qdot+g_v vdot+g_z zdot+g_u udot+g_t按真实依赖计算，相对 frame 扣除参考体运动；零行程速度不做 toe_dot/travel_dot 冒充输入偏导。
- 中心 sampled_time_difference 记录真实时间/模板/误差/端点单侧标识，不跨事件/活动模式/无效点差分；不声称解析。
- 无请求无额外切线/块，开启不改变原轨迹/事件/接受拒绝次数；超全接受步预算报告，不降采样。
- 逐样本物理无效可返回原因，缺实现/核心状态缺失记部分交付，不由缓存上一点填补。

## 执行步骤

1. 核实接受态和输出状态同步时序：只读审计状态/乘子/模式/内部变量，核心采样完整性证据明确。证据/落点：raw/sample-state-audit.md。
2. 接入三种采样选择与独立身份表：initial/accepted/output/indices可追溯，拒绝步与不同网格不混淆。证据/落点：tests/characteristics/test_sample_selection.py（拟新增）。
3. 计算动态几何/输入偏导与真实时间导数：移动 frame 与停车/零行程速度正确；不是伪时间/轨迹比值。证据/落点：tests/characteristics/test_trajectory_rates.py（拟新增）。
4. 实现显式中心序列时间差分：模板/端点/误差元数据齐全，不跨事件/无效点。证据/落点：sampled_time_difference 专项测试。
5. 验证状态不变与性能预算：有/无请求原状态/能量/事件/步序一致且核心数值覆盖目标网格。证据/落点：raw/runtime-comparison.md。
6. 完成 R8 阶段迁移与退出验证：共享片段精确记录；离散参数未进入物理服务；专项与回归通过。证据/落点：raw/retirement-evidence.md；raw/validation.md。

## 验收条件

- [ ] 悬架与整车动态核心网格每个选中样本产出实际几何/导数或合法物理原因，完整状态系统性缺失阻断 DONE。
- [ ] all_accepted 和 all_output 保留真实采样身份，不假造一对一映射；初始、未输出接受步、部分失败序列均保留。
- [ ] 相对移动 frame、真实速度/导数和停车点正确，中心差分不跨事件且来源如实。
- [ ] 无请求基线不变；开启功能也不改变原积分/事件/步序，性能符合 P0 且不静默抽样。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- 输出网格与接受步不重合、拒绝试探步、初始一致点、指定各集合 ID、失败后部分结果。
- 完整输出态与仅位姿插值：后者正确报 inconsistent_sample，核心场景需补齐原采样机制。
- 移动/旋转参考体、多输入轨迹、停车/零行程速度、准静态伪时间拒绝。
- 事件/活动分支切换、不可微点、序列端点/非均匀真实采样时间；状态/内部变量 hash 与事件统计比较。

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

当前原生结果不是完整检查点，必须从原生已接受状态取值；若原输出机制不能提供一致核心网格状态，须落实采样路径，不能以全点 inconsistent_sample 结束任务。
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
