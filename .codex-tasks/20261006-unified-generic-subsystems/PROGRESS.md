# 通用子系统单一路径重构进度

## 当前恢复块

- 任务：删除兼容生产路径，统一通用多体建模、轮胎/制动/驱动子系统、试验台边界和结果路径
- 形态：epic
- 进度：10/10；所有子任务及叶步骤DONE；方案审核与最终验收通过
- 当前：统一路径、兼容路径退役、数值验收和文档交付完成；Taskmaster终局校验通过
- 真源：`SUBTASKS.csv`；计划审查记录：`planning/review.md`
- 关键约束：Wheel Subsystem 拥有 Wheel Body 和 Tire Element；Brake/Drive 是同级 Subsystem；同一 wheel/suspension 文档用于试验台与整车；spin 由边界条件而非另一份悬架定义决定
- 验证：最终快速1553 passed/1 xfailed、完整Cases101项、完整架构295项、kernel/contracts161项、关键集73项、退役AST与五个文档示例通过；数值三门通过且9份冻结基线文件不变
- 下一步：无剩余实施任务；本次会话按用户要求提交全部更改，提交状态由git log核对

## 计划记录

- 2026-10-06：用户指定本EPIC开始执行；恢复CSV、完整设计与接口契约，计划validator通过。两名只读探子核实7组注册键与主要消费者；消费者结果为partial，库存脚本需继续采集完整AST引用和证据，不能把探子结果直接视为验收。

- 2026-10-06：根据用户修正，将 tire 定义为 Wheel Subsystem 内部力元，将 Brake/Drive 提升为普通 Subsystem；移除保留兼容生产路径的方案。
- 2026-10-06：新增显式 Wheel Spin Port 和 Spin Boundary；悬架 K/C 默认锁定相对 spin，整车动态默认释放，同一悬架和车轮定义不变。
- 2026-10-06：首轮审查发现任务7 loader依赖遗漏、跨包pytest和不存在目录、收尾门命令不完整、库存及删除门/计划校验不足；修订接口、责任清单和可执行门禁后复审。
- 2026-10-06：领域/切换终审PASS，执行协议终审PASS；计划validator、ruff及关键命令dry-run通过。原始规划证据见raw/plan-validation.md，未运行源码或数值回归。
- 2026-10-07：任务1至8已验收；任务9离线Axle/Vehicle/Case声明迁移25项、kernel/contracts158项通过；生产消费者尚未迁完，未通过删除前退出门，未切换或删除旧生产链。新接触frame的spin修正与旧数值不变证据分别记录。
- 2026-10-07：普通K/C Rig、显式力元激活表与原型/文件输入一致性已补齐；任务9当前验收53项、单模型及迁移联合60项、结构五门通过。K/C性能及9 K/66 C冻结状态门已通过。动态实际生产者已迁入普通文档，26份冻结产物逐字节不变；原9项自收敛失败与Adams缺证据状态保留。车辆/Adams消费者和全部库存映射仍待迁移，任务9保持IN_PROGRESS。
- 2026-10-07：补齐普通整车K/C Rig和pad高度输入；八族parity已全部改为普通文档提交，实际运行除vehicle_dynamic外均PASS。Adams K/C、单胎Rig、整车诊断和计时消费者迁入统一提交；ResultEnvelope报告按显式实体ID查询。接触frame校验补充wheel及固定hub代理反例。当前步骤55项、快速集1613 passed/1 xfailed、kernel/contracts158项、结构五门通过；9份库存冻结文件哈希不变。
- 2026-10-07：vehicle_dynamic原严格门仍FAIL。诊断恢复原实体顺序后5个工况逐位相同，另外3个因非自转contact frame改变物理；只读历史frame诊断全部8项原哈希一致，但该诊断在resolver后改IR，不是生产数值门证据。验收建议已撤销不合法历史参照图承诺，待用户批准默认物理修正及三个严格不匹配工况的登记方式；完整机器验收尚未实现。未调整原门、未重录baseline、未切换旧生产路径。任务9仍0/4叶步骤完成，Epic仍8/10。
- 2026-10-07：两个Adams时域生产入口移除replay_case，分别通过普通Rig独立平衡与普通Body/Rig角位移边界实际提交；相关7项通过，ruff及ty通过，未启动外部Adams。数值验收修订已再次独立审查，同步接口和Epic的待批准边界。
- 2026-10-07：用户明确批准数值验收修订；开始实施五项严格等价和三项独立物理验收。批准不代表删除前退出门通过，不修改原冻结文件。
- 2026-10-07：批准的数值修订机器门及只读复审完成，5 STRICT原哈希一致、3 PHYSICAL_DIFFERENCE保留原不匹配且独立物理门通过；21项相关physics、快速1624 passed/1 xfailed、kernel/contracts158项、结构五门和完整数值三门通过。原26动态产物哈希不变；新参考NPZ逐通道锚定原冻结哈希。仍未切换旧公开执行链，剩余消费者和库存映射继续执行；Epic 8/10、任务9步骤1 IN_PROGRESS。
- 2026-10-07：车辆共享验收fixture与旧生产测试入口解耦；全部八项原输入指纹保持一致，新解释器阻断旧构建入口验证通过，135项相关及快速1626 passed/1 xfailed通过，只读复核无确证问题。flow显式拓扑验收迁入普通文档，两分支PASS；其它架构及报告消费者未迁完，任务9仍IN_PROGRESS，记录见raw/validation-20261007-fixture-migration.md。
- 2026-10-07：继续任务9步骤3。三轴/单侧/制动驱动接线29项、文件作者与非汽车扩展39项、轮胎与同级力元35项、公开API17项通过。修正静态轮胎夹具为显式Spin锁定，并按TIR载荷曲线计算平衡高度，未改变载荷断言。最新完整快速诊断为68 failed/1525 passed/1 xfailed/8 errors（raw/fast-current.log），此后已修复其中多个文件，尚未重跑完整快速门。旧准备/装配链和部分历史测试消费者仍未退役，任务9保持IN_PROGRESS。
- 2026-10-07：删除旧preparation、subsystems、authoring.solver、studies bridge及旧bench执行面；纯信号和转向分配工具移入authoring。API解码、无齿条场景、SignalBus真实扰动、文件来源等40项通过。删除后首次collect诊断暴露51个旧测试导入错误，正在保留不变量并迁移消费者；任务9步骤3仍IN_PROGRESS。
- 2026-10-08：全部测试消费者迁移后快速1552 passed/1 xfailed、kernel/contracts161项、切换128项与AST门通过。完整cases初次100 passed/1 failed，完整架构277 passed/4 failed；五处均已修复并定向158项通过，完整架构复验中。四个旧family实体emitter及公开导出已删除，57项相关通过。冻结数值首次因统一保存布局改变失败，新增真实回归先红后绿，对标生产者仅从ResultEnvelope投影冻结布局；26产物原字节哈希、八族门和K/C预算均恢复PASS，失败日志保留。任务9四步DONE，任务10IN_PROGRESS。
- 2026-10-08：最终独立核验补齐真实整车轮胎质量/惯量及非默认摩擦回归，修复离线车辆迁移摩擦更新覆盖其它属性；158项相关、73项关键通过。最终快速1553 passed/1 xfailed，完整架构295 passed，完整Cases串行101 passed，kernel/contracts161 passed，五个文档示例和文件物理读数通过。Cases先前native abort在单例及完整串行复验未复现，原始记录保留。9份冻结文件不变；数值三门通过，5 STRICT/3 PHYSICAL_DIFFERENCE及既有FAILED/BLOCKED如实保留。Taskmaster10/10全部DONE。
- 2026-10-08：Taskmaster终局validator通过：10个子任务、34个叶步骤、依赖、路径归属及所有DONE日志哈希有效。删除前门档案已只读复核并生成统一格式叶记录，原验收报告和当时时间不变。本次计划、原始证据与代码一并提交；构建产物仍遵循仓库忽略规则。
- 2026-10-08：Git暂存会自动规范化日志换行，实测child-10-step-2.log暂存字节与验收原文件不同；使用仅针对本次raw证据的.gitattributes binary规则保留原字节，不改源码差异检查。重新暂存并核对证据字节及staged diff-check。
