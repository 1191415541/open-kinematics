# P6：查询/实际表导出、退役收口与核心发布

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：完成统一消费/查询/JSON-NPZ-CSV-Parquet真实往返、R1–R9唯一归属和核心四组合发布门；明确核心发布与整个Epic完成的区别。
- 直接依赖：P1;P2;P3;P4a;P5；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §4、§5.1–§5.2、§8–§11。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P1最小查询/原生持久化，P2/P4a局部响应与指标，P3真实采样身份，P5全部核心兼容逐项通过。
- P0完整指标/力律清单、manifest政策与预算、R1–R9兼容入口/生产调用台账。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- results/envelope.py、results/characteristics.py、outputs/derived.py/builtin.py、vehicle和report消费适配。
- io/artifacts.py唯一持久化入口；拟新增io/characteristics.py只从同通道描述生成表布局，不计算物理。
- contracts通道/manifest版本及原生各通道，不另造schema或特性写入体系。
- R1–R9退役台账、架构定向检查及核心四组合集成/性能/全量验收。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- result.characteristic(id)、统一基础通道到derived映射、vehicle/report消费统一算法；旧measure/表达式/报告字段继续兼容。
- write_artifact/read_artifact 原生JSON/NPZ和真正CSV/Parquet写表/读取：模式/指标/身份/单位/有效性及边界/冻结/来源/版本关联可往返。
- 接受步独立采样表、部分结果/无效点不丢行；仅显式矩阵请求保存实际行列、布局和分量单位。
- R1–R9 complete台账：旧精确代码段、新归属、已迁消费者、保留公共入口、pending/migrated/verified/retired、测试/DAG/数值/性能证据与退出提交。
- 核心四组合发布报告/示例/性能预算及目标结构落地证据；P4b/P7和可选矩阵capability分别登记。

## 实现规则

- formats必须真实产生所选CSV/Parquet或明确拒绝；目前已有读取不算写表完成。复用已依赖pyarrow与csv，不引入新依赖。
- 表布局只依赖已存通道描述，查询/导出/读取不重新求解/配平/取新切线/加载建模资源；未存模式报缺失。
- 原生与表格来自同一值/单位/身份/有效性；六轴数据分量单位明确，接受步不按长度拼到输出网格。
- R1–R6消费者全部走统一算法，R3默认/真实注入路径调用证据齐全；R5旧对象/切空间映射未完成不能退役。
- R7/R8移出物理组装旧处仅委托；solver离散策略/其他solve_linear保留。R9新增特性指标/表布局/分解不能产生副本。
- 退役表示生产算法退出旧位置，不删除公共API/整个文件；删除核心文件需确认，无关死代码不清理。
- P6完整核心发布要求P5核心Adams通过，核心缺公式/实现/参考或系统性缺状态均阻断；单点物理奇异/不可微原因可按契约交付。
- P4b/P7未交付不阻断核心P6；Epic全部9项DONE才整体complete。可选矩阵能力独立支持或明确拒绝，不偷加为核心前置。
- 不重录数值/性能基线，不增长skip/xfail，不削弱断言或长期豁免失败。

## 执行步骤

1. 收口查询与全部消费者：旧measure/表达式/报告/replay兼容，消费者无私有/重复算法。证据/落点：消费/兼容专项测试及R1–R6证据。
2. 实现同通道表布局与实际CSV/Parquet写表：formats真实写入，JSON/NPZ和表格同值/单位/身份/原因。证据/落点：tests/characteristics/test_table_export.py（拟新增）。
3. 验证冷读取/部分结果/独立网格往返：禁求解探针下查询导出读取通过，无效点/接受步映射不丢。证据/落点：tests/characteristics/test_artifact_roundtrip.py（拟新增）。
4. 收口R1–R9唯一归属和保留入口：定向调用/DAG证明无旧生产副本或未迁调用，公共符号理由明确。证据/落点：raw/retirement-ledger.csv。
5. 运行四组合与性能发布门：P0目标清单和P5核心兼容全通过，完整一致采样/预算达标。证据/落点：raw/core-release-report.md。
6. 运行跨系统收尾回归并同步文档：构建/快速/数值/完整架构/全量通过且skip/xfail不增长，示例可执行。证据/落点：raw/validation.md；用户文档与保留入口清单。

## 验收条件

- [ ] 四组合P0核心量有实际数值/合法物理原因且P5全部核心validated，不能以部分交付宣布完整SVC。
- [ ] 原生/CSV/Parquet往返保留模式/指标/采样、单位、profile/frozen/version/source/invalid原因；请求矩阵则实际行列同样保留。
- [ ] formats实际执行写表，冷读取/查询/导出不产生物理求解，未存模式明确缺失。
- [ ] R1–R9生产算法唯一归属/全部消费者迁完，旧值/公共符号兼容，DAG无环。
- [ ] 结构、数值、性能与全量/完整架构收尾通过，冻结基线/skip/xfail不增长；P4b/P7独立状态如实。
- [ ] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- 悬架/整车×准静态/动态全组合，固定/自由车身、移动车身、不同网格及部分失败。
- CSV/Parquet实际文件存在/表行值与原通道一致、manifest链接完整、无效点不丢、分量单位不笼统。
- 读取后禁native/建模资源探针；未请求/未持久化模式报错，可选矩阵部分列缺失不填零。
- R3默认/注入真实入口、R5旧对象所有兼容类型、R7/R8委托、R9分解次数；结构静态门验证唯一归属。

## 验证命令与证据

未来实施时在仓库根执行，每条检查退出码，失败即停止放行；不能把这些命令记成规划阶段已执行：

```powershell
just build-kernel
uv run --no-sync pytest packages/suspension_multibody/tests/characteristics -q -p no:cacheprovider
just check-fast
just gate-numeric
uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
just test-all
git diff --check
```


完整architecture与test-all仅在跨系统核心收尾执行，其他阶段日常不默认全量；最终准备提交也需适用全量门。
相关新增专项断言必须覆盖上列场景，不能以旧测试全过代替新能力验收。

## 风险与阻断

P5未获得独立参考或未核实规则时P6核心发布仍被阻断；导出/迁移局部可推进但不能标任务DONE。旧R5兼容映射、真实报告注入和系统性缺状态是必须处理的收口项。
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
