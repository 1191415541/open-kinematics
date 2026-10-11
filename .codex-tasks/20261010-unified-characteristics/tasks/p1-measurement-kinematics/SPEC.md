# P1：准静态测量、运动学与兼容迁移

## 任务与依赖

- 形态：single-full；发布归属：核心。
- 目标：建立唯一测量/Twist/中心算子与 native 运动学响应，逐准静态样本提供几何值及受控输入偏导；贯通最小只读查询和原生持久化。
- 直接依赖：P0；以父 SUBTASKS.csv 为真源。
- 设计依据：[统一多体特性分析设计](../../../../packages/suspension_multibody/docs/unified_characteristics_design.md) §3.1、§4–§7、§9、§10 R1–R6、§11。
- P4b/P7与可选矩阵视图不是核心P6前置。

## 输入与启动条件

- P0 请求/结果契约、核心几何/转向目录、尺度/容差、运动学 profile。
- P0 R1–R6 调用台账及 R5 旧对象映射；已有测量、滚心拓扑、screw、metrics/replay 测试。
- 启动前核对依赖DONE与实际证据；接口/范围变更先修SPEC/父计划并记录理由。

## 修改范围与责任

- 拟新增 characteristics/{values,measurements,motion,centers,registry}.py；results/characteristics.py 适配具名结果。
- results/envelope.py、vehicle/roll_centers.py、vehicle/screw_kinematics.py、outputs/builtin.py、report/metrics/case_specific.py 的 R1–R6 部分；逐入口委托，保留公共符号。
- kernel 的原生约束导数/请求调度与最小运动学服务；mb_linearization 物理组装在 P2 完成，不在 P1 再复制力律。
- io/artifacts.py 的最小 JSON/NPZ 特性通道保存/读取；完整 CSV/Parquet 在 P6。
- 目标新文件仅在职责需要时建立；实施前重查精确源码符号/代码段，不把设计目标当当前目录。

## 必须交付的产物

- 统一 frame_position/Toe/Camber/主销参数、轮心测量与导数，显式转向输入/Ackermann 约定及前后/整车分组。
- 运动学约束 J/D/H 去重和原边界驱动替换、尺度/秩/零空间审计；基于实际测量的可辨识响应。
- 前视/侧视摆臂、单轮侧视瞬心、几何侧倾/纵倾中心、空间瞬时轴；roll_center_legacy_v1 单独保留原数值语义。
- result.characteristic(id) 最小具名查询、JSON/NPZ 请求/结果/身份/无效原因往返，冷读取不启动求解。
- R1–R6 已迁算法及兼容委托记录；尚未完成的 R5 适配显式 pending，P6 不可放行。

## 实现规则

- characteristics 只依赖低层数据/空间代数，不反向导入 results、作者层、Case 或 solver；modeling 不导入计算逻辑。
- 已有运动驱动替换增量目标，不同时叠加零增量行；释放仅作用于响应问题，全部允许虚拟运动基用于中心审计。
- 剩余零空间 N 中 g_q N=0 才可辨识；最小范数解不能冒充唯一运动。柔性 C 模型不得偷偷替换成理想关节。
- roll_force_equivalent_v2/pitch_force_equivalent_v1 使用显式权重、接地点完整高度与截面，返回作用线/合力/力矩；总力零、自由力偶和截面退化分别诊断。
- Twist 有效但纯平移有限轴不可定义；几何中心、单轮侧视瞬心与纵倾中心分别命名。
- 旧滚心最小二乘/符号保留在 legacy 版本，不给旧值虚增新可辨识性保证；旧 measure 无新增请求仍可用已持久化基础量。
- 生产差分 J/约束律副本迁为 native 服务；差分仅在测试 oracle 中保留，R5 映射未覆盖则保持 pending，不能先删旧入口。
- 旧表达式 ID、末样本选择、mm/deg、左右符号、report/replay 汇总维持；outputs 只消费声明映射。

## 执行步骤

1. 实现统一测量与只读值类型：位置/角度/单位和移动参考体覆盖旧测试。证据/落点：measurements.py；values.py；测量兼容测试。
2. 贯通 native 运动学响应与边界审计：J/D/H无重复矛盾，欠驱动按测量判定，不用生产差分 J。证据/落点：tests/characteristics/test_kinematic_response.py（拟新增）。
3. 实现 Twist、几何中心及转向目录算子：中心/轴/主销/Ackermann 定义和退化原因符合 P0，旧新版分开。证据/落点：motion.py；centers.py；registry.py。
4. 迁移 R1–R6 并验证真实报告入口：default/instant_center_engine 注入路径、旧对象映射与 metrics/replay 均有调用证据。证据/落点：raw/retirement-evidence.md。
5. 实现最小查询及原生工件往返：每准静态样本值/原因有身份，冷读取查询不重新求解。证据/落点：tests/characteristics/test_query_artifact.py（拟新增）。
6. 运行专项及仓库门并归档：无请求/旧值不变，结构与数值门通过。证据/落点：raw/validation.md。

## 验收条件

- [x] slider/四杆/双叉臂/空间机构逐准静态样本有实际值或物理无效原因；不以缺实现记合法无效。
- [x] 坐标/frame旋转、镜像、移动参考体、输入偏导与欠驱动审计正确；全部允许运动均参与中心可辨识性检查。
- [x] R1–R6 各已迁项旧数值、异常与公共符号兼容；R3 真正调用 compute_vehicle_roll_centers 的默认和注入路径。
- [x] 查询/原生持久化随 P1 可用，运行时通道缺失报告查询错误；查询不调用 native 或资源重建。
- [x] TODO全部步骤完成行为验收，raw/validation.md有真实证据，再更新父状态。

## 专项验证场景

- slider/四杆/双叉臂/空间机构的解析或独立差分响应；旋转/镜像/单位协变。
- 欠驱动但目标测量唯一、目标不唯一、不相容驱动；接地点高度不齐/多轴/权重缩放/零合力。
- 纯平移/有限轴退化、作用线平行或位于截面、自由力偶；legacy 数值逐项等价。
- tests/physics/test_roll_centres_by_topology.py、tests/vehicle/test_screw_kinematics.py（含真实报告注入）、tests/results/test_measurement_queries.py、tests/metrics/test_outputs_match_legacy.py、tests/simulation/test_replay.py。

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

R5 对旧 Python 对象的映射可能未覆盖全部兼容语义，须依据 P0 逐类验收并如实阻断退役；原生请求接入可能改变结构，须证明 DAG 与无请求数值不变。
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
