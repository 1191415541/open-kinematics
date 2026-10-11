# P0：SVC 定义、边界与端到端契约

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：冻结核心四组合、指标/目标力律、边界及冻结协议；让特性请求穿过唯一编译/native reader/结果契约，未知内容逐层拒绝。
- 直接依赖：无；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §3、§4、§5、§8、§9、§10.3、§11.1。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- 原设计全文与当前源码；设计中的示例不是现有可执行接口。
- Adams Car 2024.1 官方本地 help（原设计 §11.1 的两页及图片）；只能读取本地证据，启动商业软件/许可证或获取外部数据需先确认。
- 已有旧对象、公共符号、模型/计划指纹与模块 DAG；不沿用历史任务 DONE 作为当前证据。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- packages/suspension_contracts/src/suspension_contracts/contracts/solve_plan.schema.json、multibody_case.schema.json、multibody_result.schema.json 及对应校验器；capability 以既有文档扩展。
- packages/suspension_multibody/src/suspension_multibody/modeling/resolved.py、compilation/resolved.py、kernel/capabilities.py；拟新增 modeling/characteristics.py 仅放请求纯数据。
- packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp 的请求读取、capability_document 和 named block 描述；不另增每指标 ABI。
- 本任务 raw/ 下建立指标目录、边界/冻结协议、预算、后端现状和 R1–R9 台账；契约字段单一真源归 contracts。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- raw/metric-catalog.csv：覆盖 §3.2 七类目录，逐条列核心四组合、metric/response kind/version、测量/扰动、力元、frame、分量单位、native/Adams 状态与独立 oracle。
- raw/boundary-profiles.json、raw/frozen-protocols.json：显式 body_increment、steering、wheel_end_boundaries、load_application、tire_scope、measurement_reference、force_scope；本项目与 Adams 协议分别命名。
- raw/adams-rule-audit.csv：阻尼/速度、预载/乘子、惯性、内部状态、整车边界、表示/公式六组，出处/证据指纹/未知项/阻断范围。
- raw/acceptance-budgets.json：物理尺度、绝对/相对容差、秩/残量/零响应阈值、四组合输出/全接受步的模式数、组装/分解次数、时间/内存/体积预算及测量环境；数值由 P0 实测锁定，不在本 SPEC 猜值。
- raw/backend-capabilities.csv、raw/module-dag.md、raw/retirement-ledger.csv：已具备/拟增接口分开；R1–R9 精确旧符号、调用者、新归属、兼容入口、状态和退出证据。
- 可验证的请求编码/结果通道/采样身份/capability 闭环与正反契约测试；不支持的能力明确拒绝，不能发布空壳 capability。

## 实现规则

- 请求字段完整覆盖 id/definition_version/domain/samples/measurements/perturbations/boundary/definition/frozen_conditions/response_operator/wheel_end_response；samples 拒绝 all。
- 公式、响应类型、冻结规则独立版本化。local_response_v1 是具名模式/测量通道；矩阵布局仅对显式 matrix_view 请求登记。
- 模型和原求解计划指纹策略保持；特性请求另记规范化指纹。case 重建不得丢请求，native 不得静默忽略未知字段/版本。
- 指标与边界不能从实体名称猜测；释放只引用已有运动边界，不停用关节/衬套，不以独立前轴代替整车。
- Adams 未知规则保持 pending_definition，缺数据保持 pending_reference。本项目 native 协议可独立向 P1–P4a推进；Adams 路径严格受规则审计前置约束。
- P0 DONE 表示目标与契约冻结、未知项已如实登记，不表示 Adams 未知项已解决；P5/P6 仍由核心逐项兼容门阻断。
- 实测确认 mb_linear 单 RHS 分解句柄、稀疏结构复用和 transpose 能力；不把 NewtonLinearizationCache 当特性物理缓存。
- R5 逐种旧对象/约束类型记录合法实体与切空间映射，R8 由 P2 补共享片段，不泛称整目录。

## 执行步骤

1. 锁定目录、四组合和力律清单：每条核心指标绑定场景、独立定义/单位/验证方法；任何未核实公式有显式状态。证据/落点：raw/metric-catalog.csv。
2. 登记 native/Adams 边界及冻结证据：完整 profile 和六组 Adams 规则审计，未知项不启用兼容 profile。证据/落点：raw/boundary-profiles.json；raw/frozen-protocols.json；raw/adams-rule-audit.csv。
3. 实测冻结容差、性能预算与后端能力：预算有尺度、环境与原始测量来源，现有/拟增接口可区分。证据/落点：raw/acceptance-budgets.json；raw/backend-capabilities.csv。
4. 定位 R1–R9 调用与目标 DAG：各项旧公共符号/消费者/保留理由可追溯，R5映射范围明确。证据/落点：raw/retirement-ledger.csv；raw/module-dag.md。
5. 实现纯数据声明及全链契约读取：solve-plan/case/native/result/capability 正向穿透；未知字段/版本/边界/单位逐层拒绝。证据/落点：tests/characteristics/test_request_contract.py（拟新增）及 contracts/kernel 契约测试。
6. 验证并归档 P0 退出证据：契约负例、请求不丢失、无请求兼容通过；实际支持与预留定义不混淆。证据/落点：raw/validation.md。

## 验收条件

- [ ] §3.2 七类指标和 §3.3 四组合均有固定目标项；后续不得删除失败项缩小发布范围。
- [ ] 请求身份、单位、版本、完整边界/冻结协议可往返，非法或未支持请求在编译/reader 明确拒绝。
- [ ] Adams 六组规则未知项有出处缺口和阻断清单；native 验收与 Adams 兼容发布判定分别记录。
- [ ] R1–R9 与后端/DAG/导出描述可支撑后续任务；R8 待 P2 精确化不记 retired。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- 已知/未知实体与单位、未知字段/版本、矛盾 hold/release、缺 profile 字段。
- case→plan→compile→native 请求编码不丢失；输出 metadata/采样 identity 与契约一致。
- 准静态 all_accepted 与伪时间 trajectory 请求拒绝；能力未知/未实现拒绝；无请求旧通道不变。

## 验证命令与证据

未来实施时在仓库根执行，每条检查退出码，失败即停止放行；不能把这些命令记成规划阶段已执行：

```powershell
just check-fast
git diff --check
```



契约实现若触及物理求解代码，追加build-kernel和gate-numeric；文档审计须按字段和出处人工复核，不只检查文件存在。

## 风险与阻断

本地 help 未必足以说明动态冻结规则，参考数据取得也未确认。作为显式证据缺口保留；不得用 native 验证替代 Adams 审计。仅契约变更不触物理解算；若实际接入改变求解代码，追加构建与 gate-numeric。
- P0完成前必须关闭本任务的目录冻结、契约实现及证据登记缺口；尚未核实的Adams动态规则/独立参考，在完整登记为pending并明确阻断兼容路径后，不阻断P0 DONE或native后续任务。它们仍阻断P5/P6完整兼容发布。范围/依赖或验收策略变更记录于PROGRESS并复核父依赖。

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
