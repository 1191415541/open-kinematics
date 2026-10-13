# Epic 实施进度

## 恢复块

- 日期：2026-10-13；状态：IN_PROGRESS。当前真源：**3/9 子任务 DONE（P0、P1、P2）**；
  叶子步骤以各任务 TODO 为准（P0 6/6）。
- 原目标及关闭条件不变；P3/P4a 因原范围生产缺口重开。历史完成数字与下文
  2026-10-10/11/12 证据不代表当前状态。
- **P0 于 2026-10-13 重新收口为 DONE**（步骤 5/6 于 2026-10-12 重开、retry_count=1）：
  native 完整消费 `held`/`released`（字符串数组，拒绝非字符串/空串/列表内重复/同时出现在
  `held` 与 `released`）与 `wheel_end_response`（对象原样存回），并把解码后的请求**回显到
  结果 manifest 的 `characteristics` 字段**；测试 15→25 passed，反证（冲突检查置
  `if(false)`）可判 FAILED。门禁：build 两条 exit 0；kernel 129 / contracts 78 /
  no_legacy 5 全 passed；快速集 **2051 passed / 1 xfailed**（skip 0，xfail 未增长）；
  characteristics 489；ruff/ty/三架构门 0；sentinel/parity exit 0；
  `kc_perf_gate` FAIL 已按用户裁定登记为环境抖动（比值自身在 ×1.288–×1.467 抖动，
  且 perf 用例不含 `characteristics` 键即新分支不执行），未重录基线、未放宽 BUDGET_FACTOR。
  证据 [raw/validation.md](tasks/p0-definition-contract/raw/validation.md) 第 9 节。
- **P0 的 Adams 审计已修订（2026-10-13）**：原「本机无 Adams Car 2024.1」前提**被证伪**
  （`C:\Program Files\MSC.Software\Adams\2024_1` 存在；`probe_profile` 返回
  `available=True`/`version="2024.1"`/`license_probe="passed"`；`validate-adams --smoke
  --require-installed` exit 0）。按用户裁定以 P5 权威 2024.1 审计重写
  `raw/adams-rule-audit.csv`/`.md`：**G6 由 `pending_definition` 升为 `stated`**、
  **G3 改为 `partial`**（`adams_solver/gtcmat.html` 明确给出 C++ Solver 动态柔度算法，
  旧「动态段无完整算法」对 Solver 手册不成立）、G1/G2/G5 为 `partial`、G4 为
  `not_stated`；`source_sha256` 由脚本对本机 2024.1 help **实时计算**（非手写），与 P5 版
  逐字一致。原版保留于 `backups/20261013-p0-adams-2024-1-audit/`。**口径未放宽**：
  Adams 兼容的完整动态冻结等价声称仍被阻断。
- **P1/P2 保留库级成果 DONE，P6 真实消费者仍未完成。**
- **P4a 步骤 4 第 1 层已落地（2026-10-13，IN_PROGRESS）**：新增 native
  `FrozenLoadMode`/`FrozenMeasurement`/`frozen_measurement_gradient`/`frozen_local_response`
  （声明 `mb_solve_static/functions.hpp:148-278`，实现 `kernel_static_trim.cpp:717-963`），
  实现 `A X = B_w P`、`Y = G X`；**关键结论：`G` 不需要新物理**（`A` 的前 `ndof` 行/列即各自由体
  6 个增量，与 `pose_candidate` 的增量语义严格一致）。自测
  `cpp/tests/frozen_local_response_selftest.cpp` 已注册进 CMake 与 registry。
  实测：自测 exit 0、registry 20 / kernel 130 / contracts 78 passed、快速集 2051 passed/1 xfailed
  （skip 0 未增长）、sentinel/parity exit 0、ruff/ty/三架构门 0。两轮反证均被断言抓住。
  独立只读复审 4 项阻断（相容/可辨识缺失被过度宣称、无效项写 0、`solve_many` 整批失败、
  测试覆盖缺口）**全部核实成立并已修**。
  证据 [raw/local-response-layer1-evidence.md](tasks/p4a-frozen-local-response/raw/local-response-layer1-evidence.md)。
  **仍未交付**：相容/可辨识审计、契约绑定（需新 kind/version）、`G` 宽度与 Python 消费层对齐、
  固定/自由车身 profile、ABI 结果块、`characteristics_response_kinds`、Python 读侧与单位/frame、
  四组合端到端验收。
- **P4a 步骤 4 第 2/3 层已落地（2026-10-13，IN_PROGRESS）**：
  第 2 层：①`local_response_v1` 契约新增**显式实体绑定**段（`characteristics.py` +
  `solve_plan.schema.json` + native `case_characteristics.cpp`，含 8 条 native 负例）；
  ②`G` 宽度由 `ndof` 改为 `dimension`，与 `results/equilibrium.py` 的
  `gradient.shape[1] == dimension` 对齐；③新增相容/可辨识审计 `FrozenResponseAudit`
  （秩 / 增广秩判相容 / 零空间判可辨识）+ 奇异样本的 `consistent_solution` 一致规约
  （因 `LuFactorization` 在主元 `<1e-14` 时直接失败，奇异样本只能走该规约）；
  自测扩到 14 个测试；实测 4 处变异全部被抓。
  第 3 层：①**契约绑定被 native 求解器真实消费**——新增 `resolve_local_response_bindings`
  （ABI 层）把 `binding` 解析为 `FrozenMeasurement`/`FrozenLoadMode`（标记经 `find_marker`
  查表、空 reference = world、固定体测量点名拒绝）；②逐样本 A 生产者接入 accepted 观察者
  （`frozen_working_sample` + `interpolate_input` + `assemble_physical_tangent`，`x` 乘子块取
  样本自身，**不冻结 delta lambda**）；③ABI 结果块 `local_response_value/status/`
  `modes_per_sample/audit/identity` + manifest `local_response` 段（push_block 与 append_doubles
  顺序一致）；④`characteristics_response_kinds` 由空改 `["local_response"]`，与 Python 门
  同字段口径（`measurements[].response_kind`）。
  **实测边界（本轮如实标注）**：第 2、3 层只验证到 `kernel` 构建 exit 0；
  **第 3 层未跑任何自测/测试/反证/全量门禁**（按用户裁定「先提交代码与任务记录」收工），
  故「行为正确」与「无请求时不改变原结果」两条**尚无证据**，不得当作已验证能力。
  **仍缺**：Python 读侧与逐测量单位/frame、指标层（Wheel/Ride/Roll/Toe/Camber）、
  固定/自由车身 profile、四组合端到端（步骤 6 重试）；`D` 恒为 0 仍为登记缺口。
- **用户裁定（2026-10-13，C5）两项**：
  1. **P3 步骤 2 剩余项**：用户选择「**单独立项修改 native 动态展开器使多 case 可产，
     再达到原条件后关闭**」。口径：P3 原条件（多 case 真实 accepted 生产者）**不放宽**；
     该 native 能力缺口**另立工作项**（移交方/接收任务/验收条件/最迟集成门须在 P3
     PROGRESS/TODO 中登记），完成后再按原条件关闭 P3 步骤 2。
  2. **P5 Adams 外部前置**：用户授权「**启动本机 Adams 2024.1（含许可）做只读诊断与
     必要本地验证，并尝试生成独立参考**」。据此 P5 步骤 1（定义审计重建）**已 DONE**
     （2026-10-13，retry 1，见 [tasks/p5-adams-svc-parity/TODO.csv](tasks/p5-adams-svc-parity/TODO.csv)）；
     步骤 2–5 仍 TODO，其中步骤 2 需生成独立参考（`reference-manifest.json` 仍
     `pending_reference`）。
- P3：重开2（真实 accepted 生产者）和5（同计划开关不变性与预算）。**本轮已把真实 accepted 生产者落地并行为验收**：
  native 只读观察原提交点（`RunSampleObserver`，导出 C 结构布局不变、无请求零工作量）、
  五个 accepted 结果块 + manifest 列名、Python `select_samples("accepted")` 校验后枚举与按检查点身份导出；
  同一次 `run_model` 有/无观察的 state/diagnostics/energy/力元素/contact_events **逐位相同**（含接触事件与自适应两半）。
  证据 [raw/accepted-producer-evidence.md](tasks/p3-dynamic-sampling/raw/accepted-producer-evidence.md)。
  步骤 2 仍 IN_PROGRESS：`indices` 显式集合选择**已落地**（读侧库 API `select_samples(..., ids=, case_index=)`，
  `SAMPLE_KINDS` 不变；拒绝重排/重复/越界/跨 case 歧义，`initial` 带 ids 一律拒绝），
  真实三 case K 网格的 output/initial 选择已纳入读侧验收，另修 `sample_identities`/`require_channel`
  写死 `result.raw` 导致 raw 形状崩溃的 2 处缺陷（红-修-绿 + sabotage 反证）；
  剩余项经实测**重新定性为「本版本动态多 case 无生产者」**（五个动态族展开器各只有一处
  `out.cases.push_back`，单 case 动态计划 `case_count == 1`），属 native 能力、非漏测，
  已如实登记；关闭口径待用户裁定。步骤 5 仍 TODO（预算分项计量未复测）。
- P4a：重开1/3/4/5；步骤6保持 FAILED。**本轮补齐 `frozen_inertial_load_v1`**：
  `frozen_working_sample` 读 a/alpha、逐自由体算 m*a 与 I*alpha+omega×I*omega 存入 `inertial_wrench`，
  切维改 `ndof+rows`，`static_residual` 以公共力总线同一算术替换总线惯性项，新增 `frozen_inertial_residual`。
  证据 [raw/frozen-inertial-load-evidence.md](tasks/p4a-frozen-local-response/raw/frozen-inertial-load-evidence.md)。
  仍缺：原生 `local_response_v1` 结果块、`characteristics_response_kinds`、具名轮端→body_wrench 映射、
  逐测量单位/frame、组合相容性诊断与整车耦合、设计 §6.5 的 delta lambda 联立求。
- 本轮实证修复：①`frozen_load_column` 原写 dR/dw，与声明 Bw=-dR/dw 相反——新增真实 static_residual 六分量
  中心差分回归先红，修正为正单位列 + 正力臂后 exit 0；②gauge 在力的平移 slot 上整列 return 会丢弃力臂力矩，
  已改为只抑制自身条目并补 gauge 力臂回归；③`sampling.py` 时间容差硬编码 1e-12 误拒合法长运行、
  状态列接受非前缀/越界值、输出映射未与成功样本数核对、manifest 列名未校验——四条均红-修-绿，
  2 处 sabotage 反证判别力后已还原。
- 门禁（本轮实际执行）：ruff/ty 0；三架构门 0；快速集 **2051 passed / 1 xfailed**（skip 0、xfail 未增长）；
  characteristics **488 passed**；`test_native_accepted_sampling.py` 与 `test_sample_selection.py` 各 **30 passed**；
  kernel+contracts **182 passed**（新注册 mb_run_sample_selftest 与
  mb_frozen_sample_selftest）；五个内核自测 exit 0；sentinel 逐位不变、parity 8 族。
  **kc_perf_gate 高负载时超预算，但已用 `git stash` 退回全部 packages 改动重建后同样 FAIL 的对照实验证明
  与本次改动无关**，负载回落后 OK；未重录基线、未放宽 BUDGET_FACTOR。
- 环境：开始时 native DLL/metadata 是旧 ABI16，源码期望17，初次快速集收集失败；已备份并重建两个包的
  native 镜像。历史红项不写成首次命令 PASS。
- Adams：用户提供安装目录 C:/Program Files/MSC.Software/Adams/2024_1；已核实2024.1安装。
  最小 Car 批处理 exit0 但无成功标记，含 license 类输出，启动/许可具体原因待安全诊断，
  不能据此宣称可运行或已对标。六组规则 G1–G6 保持 `pending_definition`。
- 下步：**先补 P4a 步骤 4 第 3 层的验证**（本轮按用户裁定只提交未验证的接线：
  注册/更新自测与负例、跑 kernel 自测 + kernel/contracts/characteristics 测试、
  并证明「无请求时结果逐位不变」），再推进 Python 读侧与单位/frame、指标层与四组合端到端；
  另推进 P3 步骤 5（预算分项计量）与 `indices` 显式集合选择；P4a 步骤 4（原生 local_response_v1 已接线，
  具名轮端映射 + 单位/frame）以解除步骤 6 的 FAILED。P4b/P5/P6/P7 保持未完成。
- 真源：[SUBTASKS.csv](SUBTASKS.csv)；范围：[EPIC.md](EPIC.md)；目标对账由本轮独立只读复审确认，未放宽任何原验收条件。

## 本轮完成剩余任务的执行计划（2026-10-11）

1. **P4a 端到端接线**：在既有冻结态物理原语上补齐 case 级动态样本适配（保留原样本身份、`all_accepted/all_output` 与部分失败行）、具名轮端载荷模式及组合相容性/唯一性诊断、逐测量单位/frame 映射、native ABI/result block、请求身份和 Python 查询；`characteristics_response_kinds` 只有与真实求解和结果块同时接通后才能声明，不能先发布空壳 capability。
2. **P4a 行为收尾**：覆盖四组合、组合有效但单列不相容的拒绝场景与预锁核心力律，实测动态样本→装配适配、请求身份、逐测量单位/frame、四组合端到端结果和成本；验证必须包含 `just build-kernel`、characteristics 定向测试、`just check-fast`、`just gate-numeric` 与 `git diff --check`，只有满足 SPEC 行为验收才关闭步骤 6。
3. **P4b 连续能力**：以已完成 17 类台账逐力律审计并只对有真实连续方程、`F_xdot/F_x/F_u`、测量映射和一致状态/载荷扰动证据的 capability 提供回答；不支持力律保持 `unsupported` 或编译拒绝，不能用离散 Newton/冻结切线冒充。范围只包含连续 descriptor，不宣称有限时域传播；P4b 与 P5/P6 语义独立，不作为其前置，只有共享 native 文件时按文件串行。
4. **P5 Adams 对标**：核实本机 Adams Car 2024.1 与独立参考数据；按该版本官方规则单独定义协议，不能默认等同 `frozen_inertial_load_v1`。缺软件/许可/参考时停止逐项对标并保持 pending，不能用其他版本冒充；验证包含 `test_characteristics_svc_parity.py`、`just check-fast` 与 `git diff --check`。
5. **P6 发布收口**：在 P4a/P5 条件满足后完成 JSON/NPZ/CSV/Parquet 查询往返、生产消费者迁移、R1–R9 唯一归属、冷读取不触发求解与兼容回归；收尾必跑 build、characteristics 定向测试、`just check-fast`、`just gate-numeric`、完整 architecture、`just test-all` 与 `git diff --check`，不以“适用时”省略。
6. **P7 后端扩展**：在 P4a 直接基准可用后，以同一物理问题验证无矩阵/预条件路线的正确性、稳定性、成本与收益；步骤 5 必须跑 build、characteristics 定向测试、`just check-fast`、`just gate-numeric` 与 `git diff --check`；无收益则按 SPEC 记录不推荐，不引入新依赖。

串行顺序为 P4a → P5 → P6；P4b 不门控 P5/P6，仅在共享 native 文件上与其他实现按文件串行；P7 等 P4a 基准完成后实施。所有依赖外部商业软件、许可或参考数据的结论均须可复核，不能将缺失条件改写成 DONE。

该执行计划由独立只读审查复核；审查发现的阻断项已先修订计划，再进入后续实现。

- 已完成：[P0](tasks/p0-definition-contract/PROGRESS.md) —— 契约+读取链+拒绝路径；
  [P1](tasks/p1-measurement-kinematics/PROGRESS.md) —— 统一测量/运动学算子与查询层。
  P1 验收：SPEC §53–59 五条全 `[x]`，证据 [raw/validation.md](tasks/p1-measurement-kinematics/raw/validation.md)。
  P2 步骤 1（共享物理组装+DAG，[证据](tasks/p2-equilibrium-response/raw/module-dag.md)）、
  步骤 2（多 RHS+真实转置，[证据](tasks/p2-equilibrium-response/raw/linear-batch-evidence.md)）、
  步骤 3（平衡模式响应与可辨识审计，[证据](tasks/p2-equilibrium-response/raw/equilibrium-response-evidence.md)）、
  步骤 4（Wheel/Ride/Roll 与载荷角度纯算子，[证据](tasks/p2-equilibrium-response/raw/compliance-metrics-evidence.md)）、
  步骤 5（兼容请求共享缓存与计量，[证据](tasks/p2-equilibrium-response/raw/response-cache-evidence.md)）、
  步骤 6（独立 oracle、R7/R9 及 R8 边界锁定，
  [证据](tasks/p2-equilibrium-response/raw/retirement-evidence.md)、
  [验证](tasks/p2-equilibrium-response/raw/validation.md)）。
  P3 步骤 1（接受态与输出状态同步时序只读审计，
  [证据](tasks/p3-dynamic-sampling/raw/sample-state-audit.md)）、
  步骤 2（三种采样选择与独立身份表，
  [证据](tasks/p3-dynamic-sampling/raw/sample-selection-evidence.md)）、
  步骤 3（动态几何/输入偏导与真实时间导数，
  [证据](tasks/p3-dynamic-sampling/raw/trajectory-rates-evidence.md)）、
  步骤 4（显式中心序列时间差分，
  [证据](tasks/p3-dynamic-sampling/raw/sampled-difference-evidence.md)）、
  步骤 5（状态不变性与性能预算，
  [证据](tasks/p3-dynamic-sampling/raw/runtime-comparison.md)）、
  步骤 6（R8 迁移与退出验证，
  [证据](tasks/p3-dynamic-sampling/raw/retirement-evidence.md)、
  [验证](tasks/p3-dynamic-sampling/raw/validation.md)）。
  **历史状态：P3 于 2026-10-10 曾以 6/6 关闭；2026-10-12 因原范围生产缺口重开步骤 2/5**
  （真实 accepted 生产者已补，`indices` 与预算分项计量仍缺）。关闭条件未放宽。
- 历史推进块：P2 共享物理切线、平衡响应与指标（依赖 P1，已就绪）。
  P1 关键结论：R1/R2/R3/R4/R6 已迁移，R5 按设计保持 pending；两个中心聚合与两个 camber/toe
  定义**各自具名保留**（非疏漏，实测差异见 P1 证据）；引擎秩判定与边界审计策略不可共享。
  P2 关键结论：8 个分解后端实测**全部单 RHS、全部无转置**，批量/转置为新增能力；
  无后端有调用方可见的模式分析步（PARDISO 的符号分析内嵌于首次 factor），
  故 `analyze_pattern` 对 6 个后端一律返回 false，不声称拿不到的规划步；
  平衡响应的拒绝路径一律返回 NaN 而非 0（零填充的拒绝与真实零响应数值上无区别，
  调用方漏读 status 就会发布貌似合理的错数）。
  P2 步骤 4 起指标层为"纯算子"：只消费已算出的响应、自身不触发新求解——
  率是**倒数**，极小响应会变成极大率，是"貌似合理的错数"最易产生处。
  载荷角度只做 `G_angle @ X + D_angle P`，签名**不接受任何位姿**，
  故"把 `rx/ry/rz` 当 Toe/Camber"在接口上不可能发生（非靠注释禁止）。
  相对阈值原则再次适用：`RATE_FLOOR` 由绝对改为 `RATE_FLOOR * scale`
  （步骤 3 已在响应层确立同一原则），否则合法的软机构会被误报为退化。
  P2 步骤 5 起缓存分两层且**响应缓存在上、分解缓存在下**：分解键不含指标/测量/
  请求 ID，故不同指标不阻断同一 `A` 的分解共享（设计 515 行），已用装配计数器证明；
  更根本的一条是**非零模式一律由矩阵观测、键不接受调用方声明**——
  可选字段一旦省略，复用判定就静默退化为"假定模式未变"，
  把真实模式变化记成合法的结构复用并发放失效符号分析的许可。
  P2 步骤 6 的关键结论是**"读请求"与"答请求"是两种能力，且 oracle 必须能失败**：
  一个等价性测试只有在实现写错时**会失败**才构成证据；四条路线中
  direct 与 transpose 共享 `B_w`，只有 `unit_basis`/`finite_perturbation` 能
  捕获 `B_w` 级错误，故"三条路线一致"本身不证明 `B_w` 正确。
  同一步还暴露了**精度类缺陷与结构类缺陷需要不同的搜寻手段**：
  结构类靠蓄意破坏（改矩阵、删校验）能抓，精度类（A1 的浮点吸收）
  只有用量级离谱的输入去跑才暴露——18 项自设破坏全部漏过它。
- P3 步骤 5 关键结论：SPEC §56「开启功能不改变原积分/事件/步序」被实测**劈成两层**，
  不能笼统声称"不变"：①无请求基线**精确不变**、②端点状态**逐位不变**——两者成立；
  ③**接受步数确实随请求网格改变**（2/3/3/5/9 点 → 44/46/46/46/50）——**不成立**。
  成因在 native 采样循环：外层按 sample 迭代（`kernel_abi.cpp:347`），步长被 `remaining`
  与 `next_prescribed_input_breakpoint`（`kernel_input.cpp:169-184`）双重截断，
  **请求的输出网格就是积分步长的硬上界**（请求点即同步点）。这是采样架构后果，
  非物理不一致；但**缺陷面真实存在**，已如实登记并界定归属——它在 native 循环内，
  P3 是读侧任务，无权也无需修改；若全局口径要求严格步序不变应单独立项。
  同一步的**诚实修正**：初版写"轨迹逐位不变"且只比对端点，实测**内部共享时刻并不逐位
  相同**（差 1–4 ULP），该断言**名字过强、断言过弱**，已改并补与解析解的独立比对。
  另有**自查发现的空洞断言**：初版比对用的两个网格在 `(0,end)` 内**无任何共享时刻**，
  实际只验证了 2 个端点（其中 `t=0` 还是初值），新增中间网格后才非空。
  性能预算的**通道与预期不同**：`raw.performance` 在本路径**完全未填充**
  （`available=False`、计数器全 0），而 ledger 第 1 列求和为 50——预算数字只能取自 ledger；
  且 `block("diagnostics")` **不受** `_DIAGNOSTIC_TAIL_ROWS` 影响而 `diagnostics` property
  受，只断言前者时把该常数改成 0 测试**照样通过**（已实测）。
- P3 步骤 5 破坏验证：3 条突变 2 抓到、1 未抓到；未抓到的 `_DIAGNOSTIC_TAIL_ROWS: 2→0`
  经查**不是 harness 陈旧、也不是测试缺陷，而是单 case 下语义等价**
  （`_diagnostic_samples` 逐 case 切 `[offset:offset+count]`，单 case、offset=0 时两种常数
  都给 `stored` 行），须多 case 计划才能观测——即 P3 步骤 2 抓到裸 ledger 偏移缺陷的同一手法。
  **已如实写入证据与代码注释，不假装覆盖**；这点与 P2/P3 前几步的"测试缺陷"性质不同，须区分。
- P3 步骤 5 的 D2 复审：8 项意见逐项核验后**全部成立**并全部处置，**无被反驳的主张**
  （与 P3 步骤 4 不同）。复审另**漏掉**"两网格无内部共享时刻"一点，由自查发现。
- P3 步骤 6 关键结论：**P2 锁定的 R8 片段被实测修正**——P2 只点名
  `kernel_integrator_newton.cpp` 的 64、116 两行，本步按 D8 完整口径做全内核检索，
  实测 `rotation * body_effective_inertia_body(...) * transpose(rotation)` 出现
  **7 次、跨 6 个翻译单元**。抽出 `body_world_inertia`（放 `mb_model`），7 处**全部委托**、
  无双算；计数核对 **7 迁移 + 1 定义 = 8**，全仓无第 9 处。
  放在 `mb_model` 而非 solver 内，因为 `mb_model → mb_numeric` 是**已登记**边，
  **新增符号未产生新分层边、未修改基线**（分层门独立复核 0 边 0 环）；
  语义理由是"世界系旋转是模型+位姿的性质，不是求解器离散化的性质"。
  **"无离散系数泄漏"有签名/文本/残留三重证据**（形参不含步长与 alpha 系数；
  共享单元内检索 `alpha_f/alpha_m/beta/gamma/ctx.h/ResidualContext/step_size` 0 命中；
  离散系数仍留 solver 侧且该文件属 `model` 模块）。
  **步骤 6 复审 0 缺陷 + 1 死重载**：复审逐点核对 **7/7 语义等价**，并特别核查最危险的
  **静默互换**——两个 R8 目标点分别取 `evaluation.q[bi]` 与 `next.q[bi]`，是不同引用、
  不同循环，**未发生互换**（若互换照样编译而物理错误）；另证伪重载歧义担忧
  （`Mat3`/`Quat` 均为无构造函数无转换运算符的聚合体，互不可转换，**只可能编译失败**）。
  复审指出的唯一实质问题——我添加的四元数重载**零调用者**（9 处调用点全传 `Mat3`），
  属投机 API 且与 D8 相悖——**已删除**并把理由写入头文件注释。
  这是**第八次**验证同一模式：自设破坏覆盖不到"我没想到"的维度。
  **诚实局限**：性能**无法由现有门禁排除小回归**——sentinel 逐位比对只证数值恒等、
  证不了时间；`kc_perf_gate` 只给比值且本次朝"变快"方向波动，噪声不可忽略，
  **若该门禁无方差界则任何 <10% 回归都能藏进去**（门禁固有局限，非本次缺陷）。
  本步**不声称"性能无回归"**。
- P3 步骤 6 的 sentinel 判读：非零 acceptance exit code 是**既有事实、非本步回归**——
  脚本自述非零退出**属预期**（三个 road case 已知在 `fixture.force_z` 单通道不过
  `time_convergence` 门，项目如实记录而非放宽用例），sentinel **只在漂移时失败**；
  实测失败清单与冻结基线 `failed_cases` **逐项相同（9 个，MATCH: True）**。
- P4a 侦察关键结论（步骤 1 前置，只读未改文件）：冻结协议的**数据形态与装配机制已存在且已解耦**，
  缺的是**矩阵生产者**。①`WorkingSample`（`mb_linearization/types.hpp:37-58`）**就是**冻结协议
  所需数据结构——`gauge`/`active_tires`/`load_scale`/`pose`/`position_constraint` 全以**数据**到达；
  ②`assemble_physical_tangent`（`types.hpp:78`，实现 `physical.cpp:72-248`）**已与 Newton 解耦**
  ——实测无迭代、无收敛判据、返回 `bool`、`x` 是通用增量（`:132` 方向导数读 `x`），
  **可在任意状态求值**；拒绝语义已是"报告式"（`:74-77` false 时矩阵仍填充）；
  ③消费层（`equilibrium.py`/`compliance.py`）完整，但**无输入生产者**。
  **三个真实缺口**：①**`B_w = -dR/dw` 全仓库零实现**（只在文档串/注释/形参名出现，从不被计算）；
  ②物理 tangent 不跨 ABI（`cpp/src/abi/` 仅 6 个 `extern "C"`，无一返回矩阵）；
  ③capability 刻意声明应答能力为零（`kernel_contract_run.cpp:229-243`
  `characteristics_response_kinds` 空数组，注释说明声称 response kind 会是契约禁止的空壳能力）
  ——故**实现后必须同时加入该数组**，否则就是"接受一个答不了的请求"。
  **实现决策**：装配复用静态侧 `assemble_physical_tangent`，**不用**动态侧 `ResidualContext`
  （后者携带九个离散系数，会违反 SPEC §41 与设计 §555）。
  **三个数据可得性问题全部已解决**：样本 19 列含加速度/角加速度
  （权威定义 `kernel_output.cpp:27-38` `copy_state`：`r|q|v|omega|a|alpha`），
  故惯量 wrench 可从样本直接构造、无需重新积分；乘子有现成块 `constraint_multiplier`
  `(samples,rows)`（`:1115`，Python 经 `envelope.py:172`）；约束 Jacobian 有现成块
  `(samples,rows,ndof)`（`:1119`，Python 经 `envelope.py:310`）。
  **剩余待定项不是缺数据而是缺策略**：冻结态 `gauge` 与 `position_constraint` 取什么值。
- 当前任务：P4b 连续动态描述符与状态/载荷扰动（独立于 P4a/P5/P6）。步骤 1、4 DONE（2/5）；步骤 2、3、5 TODO。
  17 类连续方程/力律与一致状态/载荷扰动坐标已审计，有限时域传播范围已锁定为当前不提供，证据见
  [continuous-capabilities.csv](tasks/p4b-continuous-descriptor/raw/continuous-capabilities.csv)、
  [continuous-audit.md](tasks/p4b-continuous-descriptor/raw/continuous-audit.md)、
  [time-response-scope.md](tasks/p4b-continuous-descriptor/raw/time-response-scope.md)。
  schema/reader 可解析 continuous_descriptor，但 native capability response kinds 为空，运行链没有 evaluator、F_xdot/F_x/F_u 结果块、测量映射或 Python 查询通道；因此不宣称能力已实现。
- 核心链 P0→P1→P2→P3→P4a→P5→P6；P4b/P7 独立，矩阵按 capability。
- 已知缺口：**Adams Car 2024.1 未安装**（本机仅 `G:\MSC.Software\Adams\2025_1_1`），
  六组规则 G1–G6 保持 `pending_definition`，独立参考数据保持 `pending_reference`。
  按 EPIC §36 不阻断 P0/P1 DONE，但**严格阻断 P5 与 P6 的 Adams 兼容发布**。
  R7/R8 精确片段已在 P2 步骤 6 锁定（R7 `migrated → verified`、
  R9 机制 `verified`、R8 片段 `pending`，均记在
  [retirement-evidence](tasks/p2-equilibrium-response/raw/retirement-evidence.md)）；
  R1–R6 生产消费者为 0 已登记；
  P2 步骤 5 交付的缓存与步骤 6 的 oracle 均尚无生产消费者（响应层未接线），
  输出体积计量需序列化层参与，两者均留待接线时补。
  R8 的实际迁移阶段为 P3/P4a/P6，本步**只锁定片段、不实施**。
- 最新实施验证：快速集 **1958 passed / 1 xfailed**（skip 0，xfail 未增长；
  较 P2 收尾 +115 为 P2 步骤 3–6 新增，其后 P3 步骤 1–5 再 +95）；
  kernel+contracts **176 passed**；characteristics **401 passed**（P3 步骤 4 后 385，
  步骤 5 新增 test_runtime_invariance.py 16 条）；
  ruff/ty/三架构门全 0；`git diff --check` 干净；未重录任何基线；
  P1 步骤 6 又跑 gate-numeric 三条全通过；
  P2 步骤 1–6 后各复跑 gate-numeric 三条全通过（步骤 6 实测 sentinel 逐位相等、
  parity 8 族、perf c-66 ×0.922），并新增
  `mb_linear_batch_selftest.exe` exit 0（含两种蓄意破坏的反向验证）。
  P3 步骤 1–5 均为纯 Python 读侧与测试，未触及 native/求解路径，故未跑 gate-numeric；
  **P3 步骤 6 触及 native（R8 抽取），已先 native 重建再跑 gate-numeric 三条全通过**
  （sentinel 逐位不变、parity 8 族、perf k-100 ×0.935 / c-66 ×0.893），
  另 `mb_linear_batch_selftest.exe` exit 0。
  本轮任务账本校验通过（9 children、51 leaf steps、143 links、DAG acyclic）；P4b 连续能力台账
  17 行/10 列、41 个证据文件路径全部通过；`require_characteristics_answerable('continuous_descriptor')`
  实测按 capability 空响应 fail-closed；`git diff --check` 退出 0（仅 Git 的 LF→CRLF 提示）。
- 本轮最终验证（2026-10-11）：导出/查询定向测试 **35 passed**；快速集
  **2009 passed / 1 xfailed**；kernel+contracts **180 passed**；ruff、ty、三条架构门和
  `git diff --check` 全部通过；taskmaster 账本校验为 9 children、51 leaf steps、147 links、DAG
  acyclic。导出表已实际写入 CSV/Parquet，并保留 JSON/NPZ、样本身份、已知单位/frame 和非有限值原因；
  既有 `manifest.layouts` 结构保持兼容，新增信息放在 `table_layouts`。本轮未修改求解路径，未重录数值基线。
- 交付后复审：P1 交付代码经独立只读复审，修复 2 个真实缺陷（多 case 时间身份、块 dtype 强转）。
  P2 采用"蓄意破坏验证"作为常规手段：步骤 2 破坏 2 处、步骤 3 破坏 3 处，
  全部被测试逮住；其中步骤 3 的破坏暴露了 2 个真实自身缺陷（死分支、零填充拒绝）。
  P2 步骤 2/3 交付后又经独立只读复审，**发现并修复 8 个真实缺陷**
  （3 条会产出错误结果或崩溃：溢出无理由崩溃、逐模式绝对阈值误拒合法载荷、
  `native_transpose` 分支复制前向分解）；已逐条补回归测试。
  复审另确认 1 处非问题（`mutable` 写入经 `&` 正确落在调用方对象上）并为其补了断言。
  P2 步骤 4 交付后又经第二次独立只读复审，**再发现并修复 6 个真实缺陷**，
  其中 1 条在步骤 3 且会静默产生物理错误：`LoadMode.kind` 经 `LOAD_MODE_COMPONENTS`
  **决定扰动落在哪一行**，旧的 `roll_moment:5` 在 `[Fx,Fy,Fz,Mx,My,Mz]` 下是
  **Mz（偏航）**而非侧倾，会让侧倾指标求解偏航系统却返回格式完好的数。
  **教训（第二次确认）**：该缺陷逃过了本步自测的 6 处破坏，因为旧测试只断言
  `kind` 的字面值、从不检查 `component`，也从不经 `load_mode_matrix`，
  即**断言了部署而非行为**；自测蓄意破坏不足以覆盖盲区，独立复审是必要的第二道。
  步骤 4 的反向验证共 12 处破坏全部被逮住，其中"车轮两侧只判左侧"一处
  **首轮存活**并暴露了非有限响应检查的覆盖缺口（`1/inf == 0.0` 会静默产生
  有限零、读作"无限软"，与真意相反），补测试后才逮住。
  P2 步骤 5 交付后又经第三次独立只读复审，**再发现并修复 3 个真实缺陷 + 6 项
  契约/命名/覆盖问题**，最严重的一条是键上可选 `pattern_digest` 默认空串：
  稠密→稀疏的真实模式变化被记成 `structural_reuse`，即设计 §5.3 明令必须失效的
  情形被记成其反面并发放"可复用符号分析"的许可。该步自测 14 处破坏**全部被捕获**，
  但其中 3 项复审缺陷自测的 8 处破坏一个都没碰到——它们都落在"我没想到要破坏"
  的语义维度上（可选字段的省略、第二条返回值路径、字段表示与同包既有契约的一致性）。
  **教训（第三次确认）**：自测蓄意破坏不足以覆盖盲区，独立复审是必要的第二道。
   P2 步骤 6 交付后又经第四次独立只读复审，**再发现 3 个真实缺陷 + 3 处过度声称**。  最严重的是 **A1：差商用绝对步长 `h = 1e-7`**——载荷量级 ≳ `1e4` 后被浮点吸收，
  `loads = 1e8` 时相对误差达 **1.08e-02**，且**减小 `h` 会使其更差**
  （`h=1e-12` → 3.7e-05，`h=1e-13` → 4.0e-04）；A1b 更隐蔽：步长缩放基准必须是
  **整个载荷矩阵**而非被扰动那一行，否则 `loads=[[1e8,0],[-1,3]]` 中第 0 列精确、
  第 1 列误差 **6.3e-04**。修后横跨 `1e-8..1e12`（20 个数量级）最差 **3.8e-11**。
  复审还纠正了台账三处**过度声称**（`h` 安全区间下界、病态矩阵下转置路线行为的
  方向写反、`unit_basis` 能捕获转置 `A` 的夸大）。
  **教训（第四次确认，这次更尖锐）**：A1 **躲过了 18 项自设破坏**——那些破坏改的是
  结构与校验，而 A1 是**数值量级**问题，需要量级离谱的输入才暴露，
  自测夹具最大只到 `1e3`、距失效起点差一个数量级。
  **自设破坏的覆盖面受限于设计者自己的想象，只有独立复审能突破它。**
  步骤 6 另判定**三个语义等价变异**并如实记录理由，而非当作漏检去"修"
  （把等价变异当漏检修 = 记录假阴性）。
- P2 步骤 6 关键结论：SPEC §57 点名的四条等价路线**没有一条是万能 oracle**，
  价值来自覆盖并集的互补——`transpose` 独有捕获"`A` 被转置"，
  `unit_basis` 与 `finite_perturbation` 独有捕获"`B_w` 列被合并/错序"
  （direct 与 transpose 都消费 `B_w`，会**同错同过**）。
  等价性检验的成立需要三件事**分别**为真：夹具非对称（否则转置路线真空真）、
  路线比较的是**同一对象**（算子 `C`，而非导数与响应）、至少一条路线对该错误免疫。
  R7 状态记 `migrated → verified` 而**不记 `retired`**（P6 才收口全部消费者）；
  R8 只做精确片段锁定、状态 `pending`，不越权声称已迁移。
- 核心 P6 未发布，Epic 未完成；规划 DONE 与产品 DONE 分开。

## 同步规则

每次子任务状态改变同步父CSV/本恢复块，记录实际验收证据/完成时间/retry_count/缺口，无证据不DONE。规划过程位于planning与REVIEW。
