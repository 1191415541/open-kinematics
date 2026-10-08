# 单次生产切换与兼容执行退役进度

## 当前恢复块

- 当前步骤：4；状态：DONE；进度：4/4。
- 真源：TODO.csv。
- 验证：删除后快速1552 passed/1 xfailed；步骤3的128项和步骤4 AST findings0通过，日志与哈希保存raw/validation；kernel/contracts161项通过。
- 删除前退出门已通过：raw/cutover-validation.json 保存数值三门、结构五门及61项路径测试的实际返回码和日志哈希；消费者映射196项使用实际源码引用和哈希。步骤1、2已DONE。
- 已切换：api仅validate/simulate，run_compiled唯一提交且只返回ResultEnvelope；旧preparation/subsystems/authoring.solver/studies bridge/rig compose和bench/旧编译及结果分派/vehicle service和axle contract_run及四个family实体emitter已退役。
- 下一步：任务10完成最终结构、数值、文档与证据同步。下方记录为按时间保存的历史状态，当前以恢复块和TODO.csv为准。

## 已核实的部分交付

- 本轮静载荷改读ResolvedModel和显式接触frame，滚心报告改读ResultEnvelope/native Jacobian；独立螺旋/滚心17项、三拓扑11项、测量4项通过。合成五连杆方向退化与刚性beam冗余已在夹具修正，不放宽native秩检查。双轮pad4项、wrench/静载/报告22项、Study18项、无转向网格8项、普通Body/Rig运动9项、analysis4项通过。simulation/replay已删除；Adams输入自转投影和PAC诊断脚本移除旧装配依赖。全快速、结构和数值最终验收尚未通过。

- 公共切换后车轴140项、整车52项和原1项xfail、编译/准备/运行42项、七协议真实运行与结果30项通过；旧服务契约迁移与一次解析计数26项通过。PAC独立刚体高级力元能力标记已修复，原外倾力矩断言保持。
- 首次退出门因GBK打印Unicode失败，数值实际通过但退出非零；已设置UTF8并完整重跑PASS，retry_count=1。冻结基线未改，既有自收敛FAILED和AdamsBLOCKED保留。

- `authoring/migration.py` 离线转换显式/代理AxleDeclaration和VehicleDeclaration；输出普通模板、子系统、总成、属性文档，不调用旧装配器或求解器。
- K/C代理输出同一套模板，用modes切换连接；Wheel声明hub、bearing坐标及Tire，Brake/Drive声明两端力矩；整车接触frame显式在upright上。
- 转换保留COM、惯量、安装姿态、Tire附加质量、PAC/Fiala系数、静态gauge、road及coupler；核对现有整车生产者的实体数据发现并修正转向平移SI倍率和Fiala镜像差异。
- 旧spring曲线由伸长正值转为native压缩正值；旧bushing局部四元数按实际执行语义保留；固定轮默认mount和aero的COM力臂修正。
- native输出补齐coupler稳定约束ID及四端wrench/power，ResultEnvelope按其reaction_block与ends读取；实际部署二进制相关提交测试通过。
- `migrate_v1_case` 将已声明native Case转换为SI SolvePlan，要求显式实体ID映射，采样表转为可保存JSON值；七种协议、非均匀采样和载荷单位测试通过。这尚不等于旧业务Case对象已全部迁移。
- `migrate_v1_dynamic_axle` 将 AxleDynamicsModel/Case 离线转换为普通子系统和 CaseDocument；刚体、关节、力元、轮胎及输入表均进入统一 compiler。`raw/dynamic-document-parity.json` 记录 13 个冻结工况各 primary/refined 共 26 项实际 native 逐位对比，状态、轮胎、力元块、能量和诊断均无差异。
- CLI `validate`、`run` 和动态/整车命令统一调用文档编译与 `run_compiled`；ResultEnvelope 可写入完整 native channel artifact，失败时保留 partial evidence。
- `migrate_v1_kc_case` 以普通Rig子系统声明轮心/齿条作动关节和明确端口配对，Study显式锁定spin；同一Wheel模板保留bearing。修复Vec3字典输入镜像与空bushing曲线被native拒绝的问题。`element_activation`按实体ID控制力元，重复或未知ID拒绝，不改变原模型指纹。
- `kc_perf_gate.py`、`kc_native_probe.py`、`kc_native_c_probe.py` 已经从普通文档提交；K/C报告通过ResultEnvelope的frame ID查询。K-100/C-66性能相对冻结基线为0.938/0.882，9 K与66 C状态均通过原容差；基线未修改。
- `run_native_axle_manifest` 主/细化输入及acceptance计时入口已迁入普通文档；`adams/axle_evidence_view.py`仅投影ResultEnvelope稳定ID到冻结证据布局，无solver/旧decoder调用。动态哈希门26份产物逐字节一致，combined SHA256为`fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`。
- `migrate_v1_vehicle_kc_case`声明四个wheel-center和一个rack驱动，通过Study锁定原Wheel的4个spin，不更换悬架/Wheel模板；实体和文件/Python提交一致性测试通过。
- K/C硬点先在原单位中计算body-local点再转SI，消除了原世界坐标相减的浮点差异；partition保留显式owner，避免合并不同body的同坐标硬点。
- `adams/reference.py`、`strict_k.py`、`strict_c.py`已移除旧装配/提交；共享K/C报告只读显式wheel frame。相关11项含9 K/66 C原冻结容差通过。`run_dynamic_kc_correlation.py`、`run_real_adams_car_benchmark.py`、`run_native_tire_rig.py`已迁入普通文档，单胎Rig14项通过。
- 整车对比、PAC力矩诊断和Fiala计时脚本已移除vehicle.service；车身ID为`body.chassis`，转向结果ID为`steering_<channel>.actuator`。实际普通文档提交输出132个轮胎及4个操稳通道；报告8项通过。计时仅围绕共享native backend，区分完整Python wrapper。
- 接触frame按声明Spin Port的bearing及fixed连通分量验证；拒绝直接wheel frame和固定hub代理frame继承相对spin。Spin/迁移/普通模板联合64项通过。
- `migrate_v1_kc_case`补齐pad_height_mm离线输入，禁止与wheel/rack sweep或载荷路径混用；`acceptance_pad_driven_kc.py`的执行、模式激活和测量迁入普通文档，相关7项验收通过，接触高度最大误差1.208e-13 mm。新增文件/Python编译payload一致性与pad实际提交测试。
- `adams/time_domain_gate.py`的车轴参考按时刻提交普通Rig的独立K平衡，显式传入左右轮、齿条和刚体载荷；`adams/vehicle_kc_time_domain.py`以普通Body+Rig、revolute和prescribed_angle提交车身横摆。两个生产校验入口不再调用`replay_case`。相关7项通过，包含左右不同位移、载荷实体ID和非零初始角；没有运行外部Adams。

## 验证和待完成边界

- `planning/checks.py step 9 1`: 55 passed；离线迁移34项通过；快速集1613 passed/1 xfailed，未新增skip/xfail。
- `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests`: 158 passed。
- 当前结构五门PASS；八族parity已由普通文档生产，实际运行kc_quasi_static、axle_dynamic、vehicle_kc、handling、ride_four_post、ride_random_road PASS，comparison N/A。vehicle_dynamic原哈希门FAIL，不能据其余通过宣布退出门成功。
- 现有9项自收敛FAILED及Adams缺证据BLOCKED仍保留。
- 新旧接触frame有明确物理差异：旧hub随spin旋转，新upright不随spin。已核对其余基础实体、转向参数及Fiala二进制系数相同，尚需独立动态误差和输入/输出报告。
- 已保存四变体实际诊断 `raw/vehicle_order_probe.py`、`raw/vehicle-order-probe.json` 和 `raw/spin-physical-difference.md`：排序不能消除误差，恢复旧frame后状态全部逐位相同；位置差约2.53e-14 m，角加速度差约0.00520 rad/s²。普通整车fixture旧路径本身积分失败，因此诊断改用已有native整车fixture；未调整误差阈值。
- K/C诊断 `raw/kc-document-probe.py/.json` 已分别检验普通图、恢复旧固定bearing、恢复旧惯量。默认K位置差1.66e-9 m在原冻结门内；C位姿差约4.1e-15，线加速度差2.57e-8 m/s²、角加速度差4.33e-5 rad/s²。恢复旧固定bearing未消除差异，恢复旧惯量仅部分降低；不能声称全部C状态逐位等价，需进一步核对数值与参考frame。
- 库存667项中196项非测试引用；CLI、动态冻结生产者与K/C状态/性能脚本已迁移；车辆 preparation、公开 API、其他Adams消费者与旧结果投影仍待迁移，映射未完成；步骤1不能标DONE。
- 最新完整验证见`raw/validation-20261007-continuation.md`。`planning/vehicle_numeric_change.md`已由只读子代理审查，调整验收须用户批准；历史诊断的resolver后IR修改不能作为新生产门，实施时必须文档级fixture。只读oracle脚本仍在raw中，不用于生产。
- 尚未迁移：`scripts/acceptance_composable_flow.py`的模板、Rig和驱动验收；`scripts/accept_composable_architecture.py` A2/A4/A5/A6/A7/A8 中旧作者/编译依赖；公共metrics、旧结果导出；api/simulation/preparation/compile旧公开执行链由步骤3在退出门后切换。`raw/migration-consumers.json`和`test_unified_model_cutover.py`尚未实现。pad模式A冻结artifact在当前checkout缺失，未创建或重录。
- 数值验收建议修订：不再承诺把旧随spin接触frame作为合法新文档重现八项旧哈希。历史IR诊断违反当前Spin声明关系，不能进入生产数值门，也不能通过移除Spin Port绕过。五个等价工况保留严格门，三个物理差异须单独报告并获用户批准；原门当前仍FAIL。
- 修订终审未发现内部矛盾，记录见`planning/vehicle_numeric_revision_review.md`；该结论不等于用户批准或机器门完成。最新两个时域入口7项、结构五门、git diff --check通过。
- 用户于2026-10-07明确批准数值修订。`vehicle_physical_evidence.py`和`case_parity_check.py`实现完整输入比较、原数组哈希复核和独立物理门，实际整车结果为5 STRICT及3 PHYSICAL_DIFFERENCE；原冻结文件未改。15项相关测试通过，复审和集成验证待完成；全部消费者未迁完，叶步骤1仍IN_PROGRESS。
- 数值门实现补齐约束覆盖反例、切向brush力/滑移、转向及衬套本构/几何/储能；21项相关测试、快速1624 passed/1 xfailed、kernel/contracts158项、结构五门和完整数值三门均通过。只读复审无新确证阻断，限制见`planning/vehicle_gate_implementation_review.md`；验证记录见`raw/validation-20261007-numeric-revision.md`。旧执行链和剩余消费者待迁移，步骤1仍IN_PROGRESS。
- 车辆fixture已抽取为共享离线声明，数值门及十个测试数据消费者解除对旧native车辆测试入口的导入；八个原输入指纹一致，冷解释器禁用旧入口验证通过。135项相关、快速1626 passed/1 xfailed及只读复核通过。flow显式拓扑门已改用普通Loader/ResolvedModel，首次漏传Case失败已修复，两种齿条声明PASS。其余template/Rig验收、公共报告及全部库存映射待迁移，步骤1仍IN_PROGRESS，完整记录见`raw/validation-20261007-fixture-migration.md`。
- 2026-10-08：所有collection和测试消费者已迁移。动态SI模型断言直接核对原声明，轮胎质量/惯量经唯一编译器验证。公开保存器仅接受ResultEnvelope；Adams冻结证据由稳定ID投影实际通道，不形成另一条求解或解码路径。完整数值门通过，冻结26份产物逐字节一致；3个整车已批准差异保留独立证据。无新增skip/xfail。
- 2026-10-08：原删除前退出门保存了cutover-validation.json及三组实际日志，却未生成raw/validation/child-9-step-2.json。最终validator揭露缺口；将步骤9-2的收尾命令改为只读核验原库存/映射/三日志/9份冻结文件哈希，生成当前档案验证的真实机器记录。原删除前命令、返回码和通过报告保留，不重演已删除源码，也不伪造当时时间。
