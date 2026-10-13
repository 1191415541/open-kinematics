# P3 实施进度

## 恢复入口

- 日期：2026-10-12（Asia/Shanghai）。
- 任务：在原 native 一致接受态只读提取动态几何/输入导数/真实时间导数，提供 all_output、all_accepted、indices 的真实样本身份与序列。
- 形态：single-full；状态：**IN_PROGRESS**（2026-10-12 重开步骤 2/5）。
- 进度：**4/6 步 DONE**；步骤 1/3/4/6 DONE；步骤 2 IN_PROGRESS（`indices` 显式集合选择已落地、真实多 case 读侧验收已补、两处 raw 形状缺陷已修；剩余项经实测定性为「本版本动态多 case 无生产者」，等用户裁定关单口径）；步骤 5 TODO（同计划开关不变性已实测，预算分项计量未复测）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 验收：[raw/validation.md](raw/validation.md)（SPEC §57 要求的真实证据）。
- 已完成：
  - 步骤 1 DONE（2026-10-10）——核实接受态与输出状态同步时序（只读审计，未改文件）。
    证据：[raw/sample-state-audit.md](raw/sample-state-audit.md)。
  - 步骤 2 IN_PROGRESS（2026-10-12 重开）——三种采样选择与独立身份表。
    历史证据：[raw/sample-selection-evidence.md](raw/sample-selection-evidence.md)；
    本轮证据：[raw/accepted-producer-evidence.md](raw/accepted-producer-evidence.md)。
  - 步骤 3 DONE（2026-10-10）——动态几何/输入偏导与真实时间导数。
    证据：[raw/trajectory-rates-evidence.md](raw/trajectory-rates-evidence.md)。
  - 步骤 4 DONE（2026-10-10）——显式中心序列时间差分。
    证据：[raw/sampled-difference-evidence.md](raw/sampled-difference-evidence.md)。
  - 步骤 5 TODO（2026-10-12 重开）——状态不变性与性能预算。
    历史证据：[raw/runtime-comparison.md](raw/runtime-comparison.md)；
    本轮只实测同计划开关不变性（见 accepted-producer-evidence.md §3），预算分项计量未复测。
  - 步骤 6 DONE（2026-10-10）——R8 阶段迁移与退出验证。
    证据：[raw/retirement-evidence.md](raw/retirement-evidence.md)、
    [raw/validation.md](raw/validation.md)。
- 启动依赖：P1、P2 均已 DONE 并关闭（P2 于 2026-10-10 关闭，6/6 步）。
- **步骤 1 的关键结论（决定后续工作量）**：原 native 结果**不是**逐接受步的完整检查点，
  而是**逐输出样本的一致状态快照**。采样循环（`kernel_abi.cpp:347`）以 `sample_count`
  为外层粒度，区间内被接受的多个步只有**最后一步**存活到写出（`:794`
  `copy_state(..., sample_index)`）；逐接受步状态**不落在任何输出数组**中，
  仅 `accepted_steps`/`rejected_attempts` 标量进 diagnostics（`:749-750`/`:805-806`）。
  故 `all_output` 可直接映射现有 `body_state` 行、`sample_kind=initial` 可直接用索引 0，
  但 **`all_accepted` 必须新增逐接受步提取路径**——这是步骤 2 的主要工作量，
  且 SPEC §84 的预警（"不能以全点 inconsistent_sample 结束任务"）**实测成立**。
- **实现契约（来自 SPEC §39，四条不可违反）**：新增提取必须在提交**之后**、纯只读、
  不触碰 `suggested_step`/`h`/缓存失效语义（固定步长路径 `:675` 尤其不得改步长，
  因为固定步序正是冻结基线记录的内容）、无请求时**零额外工作**。
- **步骤 2 的关键结论**：实测真实动态运行**存储 21 个状态、接受 406 步**（≈20:1），
  三种网格 3/9/21 样本对应 46/166/406 接受步。`output` 与 `initial` 可回答，
  **`accepted` 如实拒绝**（状态不在任何数组，且集合无法枚举——只有计数没有身份）。
  拒绝消息带两个实际数字，因为笼统的"不支持"会诱导调用方假设缺口小或可替代。
  本步**不改 native/C++**：把事实变成可追溯的选择语义，而不是伪造数据。
- **步骤 2 的复审教训（第五次验证同一模式）**：D2 独立复审发现 3 个真实缺陷
  （ledger 行偏移漏加每 case 尾部行、`initial` 依赖两个独立 offset 来源恰好一致、
  `refused` 元组语义可颠倒且拒绝分支无覆盖），而**破坏验证第一轮只捕获 1/3**。
  两项 MISSED 是**测试的缺陷而非等价变异**（与 P2 的三个等价变异性质不同）：
  ①整行求和测不出，因为 fixture 除第 1 列全 0，而**实测真实 ledger 样本行第 2..15 列
  确有非零值**——即 fixture 比真实数据"干净"就测不出真实缺陷；
  ②`nansum`→`sum` 测不出，因为 NaN 被放在选区之外，而失败运行的真实形状是
  已写样本之间夹一个未写样本行。两处修的都是**测试**，修后 3/3 全捕获。
- **步骤 3 的关键结论**：**真实时间导数不需要差分**。速度（状态列 `[7:10]`）与角速度
  （`[10:13]`）是求解器**已存下来的**、不是从位姿重建，故几何量对时间的导数可由其**自身定义**
  经链式法则直接得出——这正是 P1 `measurements.py:245` 要求的（差分只属于测试 oracle）。
  差分是**采样网格的性质而非运动的性质**，改变请求样本数就改变答案即使物理没变；
  故 7 列纯位姿输入被**明确拒绝**。设 `v = R_ref^T R_body a`，则
  `v̇ = R_ref^T(ω_body × R_body a) − ω_ref × v`，**两项都必需**：只留第一项时，
  参考体静止下答案**完全正确**——这正是静止夹具无法区分的情形。
- **步骤 3 的复审教训（同一模式第二次出现，且这次复审没找到实现缺陷）**：D2 复审手推核对了
  坐标系推导、两个导数公式与分母、世界系 `omega` 约定、力臂帧，**判定实现无缺陷**，
  但找到 3 组真实覆盖漏洞（两组是**恒真的空洞断言**）：轮心相对位置路径完全未被测试
  （`origin=[0,0,0]` 使 `lever=0`，参考位置列从未进入表达式）、移动参考下的 camber 无覆盖、
  轮心分支输入速率校验无覆盖。**我用独立破坏验证确认了 R-1**（未只凭复审结论），
  修后 12/12 全捕获。
- **步骤 3 破坏验证的教训**：第一轮 6/8，两项 MISSED 且**两项都是测试的缺陷**
  （与 P2 的三个等价变异性质不同）：夹具用**单位轴**使 `h = h² = 1`，除以 `h` 与 `h²`
  数值完全相同——缺陷住在"夹具整齐"与真实之间的缝里。修法是加长度 2 的轴用例
  （`atan2` 对轴长不变，**取值看不出差别、只有速率能作证**）。这与步骤 2 的
  "除第 1 列全 0"是同一模式的第二次出现。
- **步骤 4 的关键结论**：`sampled_time_difference` 的存在理由**不是算术而是沉默**——差商
  会安静地假定四件事（时间严格递增、邻居同分支、邻居可用、结果就是导数），而这四件事
  常常为假。四项**全部拒绝**并**记下**每样本的模板/单侧标记/真实跨度/曲率尺度。
- **步骤 4 的诚实修正（复审纠正了我的过度声称）**：我原写"非均匀加权换来二阶精度"，
  **实测不成立**——光滑非均匀网格上朴素商每一档都略优（两者都是二阶）。选用加权式的
  理由是**对二次式精确且均匀网格退化为中心商**，不是因为它更优。
- **步骤 4 字段命名的教训**：我把一个曲率量命名为 `error_scale`，**这本身就是一次过度
  声称——名字比代码做了更多承诺**。它恒等于 `span²·f''/8`，而 `f=t³` 的真实截断误差是
  与 `f''` 无关的常数（实测误差恒定 `0.002500`、该量比值 19.0），**既不是误差的量级也
  不是界**。已改名 `curvature_scale` 并写明"不是误差估计"。
- **步骤 4 破坏验证的教训（同一模式第三次出现）**：第一轮 7/8，MISSED 的
  "有效性掩码被忽略"是**测试的缺陷**——我的全部无效样本都携带 NaN，于是掩码与"是否有限"
  在整个套件里**完全等价、从未被真正检验**。修法是加入**有限值但 `valid=False`** 的哨兵。
  步骤 2 是"除第 1 列全 0"、步骤 3 是"轴长恒为 1"、本步是"无效样本必然是 NaN"。
- **复审的一条主张被我独立反驳**：复审称"权重对调的实现仍会通过非均匀测试"，我用破坏
  验证证明它 **CAUGHT**。复审把"测试没检验二阶收敛**阶**"误推为"测试看不见权重错误"。
  已记录，**未据此改代码**。
- **步骤 5 的关键结论（SPEC §56 被实测劈成两层，不能笼统声称"不变"）**：
  ①**无请求基线精确不变**、②**端点状态逐位不变**、③**接受步数确实随请求网格改变**
  （44/46/46/46/50）。①②成立；③**不成立**。成因是 native 采样循环**按 sample 迭代**
  （`kernel_abi.cpp:347`）、步长被 `remaining` 与 `next_prescribed_input_breakpoint`
  （`kernel_input.cpp:169-184`）双重截断——**请求的输出网格就是积分步长的硬上界**，
  请求点即同步点。这是采样架构后果，非物理不一致。本步**如实记录该缺陷面并界定归属**：
  它在 native 采样循环内，P3 是读侧任务，无权也无需修改；若全局口径要求严格步序不变
  应单独立项，且须先与 §39"超全接受步预算报告，不降采样"对齐。
- **步骤 5 的诚实修正（我的强主张被实测反驳）**：初版写"轨迹逐位不变"并只比对端点。
  实测**内部共享时刻并不逐位相同**——`t=end/2` 处 dense 给 `0.0025000000000000005`、
  mid 给 `0.0025000000000000014`，差 **1–4 ULP**。故该测试**名字过强、断言过弱**，
  两者都已改：断言降为"到舍入误差一致"，并**另加与解析解的独立比对**（防止两次运行
  共享同一错误而互相印证）。
- **步骤 5 自查发现的空洞断言**：初版"整条轨迹"测试用 `SPARSE` vs `DENSE`，而这两个
  网格在 `(0, end)` 内**没有任何共享时刻**——实际只验证了 2 个端点，其中 `t=0` 还是
  初值。新增 `MID` 网格（与 `DENSE` 共享 3 个内部时刻）后才非空；该测试**首次运行即
  失败**并暴露此点。
- **步骤 5 的性能预算通道与预期不同**：`raw.performance` 在本路径上**完全未填充**
  （`available=False`，全部计数器 0），而同一结果 ledger 第 1 列求和是 50。预算数字
  只能从 ledger 取——`sampling.py:221 _accepted_step_total` 正是直接读 ledger。
  另实测两条 ledger 视图不等价：`block("diagnostics")` **不受** `_DIAGNOSTIC_TAIL_ROWS`
  影响，而 `diagnostics` property 受；只断言前者时把该常数改成 0 **测试照样通过**。
- **步骤 5 破坏验证的教训（第四次出现同一模式，但这次结论不同）**：3 条突变 2 抓到、
  1 未抓到。未抓到的 `_DIAGNOSTIC_TAIL_ROWS: 2→0` 经查**不是 harness 陈旧、也不是测试
  缺陷**，而是**单 case 下语义等价**：`_diagnostic_samples` 逐 case 切
  `[offset : offset+count]`，单 case、offset=0 时两种常数都给 `stored` 行。要覆盖它必须
  用多 case 计划（即步骤 2 抓到裸 ledger 偏移缺陷的同一手法）。已**如实写入证据文件与
  代码注释，不假装覆盖**——与步骤 2/3/4 三次"测试缺陷"性质不同，这点必须区分清楚。
- **步骤 5 的 D2 复审**：8 项意见逐项核验，**8 项全部成立**，均已处置（子串检查→import
  图可达性并加 sabotage 反证；`ratio > 1.0` 近恒真→对字面量断言；`< 2.0`→显式上界；
  `>= 13`→`== 19`；`+ 2`→`2 * cases`；因果解释补 native 行号并标注非本测试所证；
  新增 §56 前半句基线测试 2 条）。复审还漏了 `SPARSE`/`DENSE` 无内部共享时刻这一点，
  由我自查发现（见上）。**未出现被反驳的复审主张**（与步骤 4 不同）。
- **两次被否决的更弱检查（记录以免重蹈）**：①在 `api.py`+`compilation/resolved.py` 做
  子串搜索——真正的路径还经过 `simulation/runner.py` 与 `results/envelope.py`，两文件
  从未被搜，函数体内惰性 import 也完全逃逸；②运行后查 `sys.modules`——**实测无效**，
  `results/__init__.py` 无条件 import 了这三个模块，它们永远在 `sys.modules` 里。
- **步骤 6 的关键结论（P2 锁定的 R8 片段被实测修正为 7 处）**：P2 只点名
  `kernel_integrator_newton.cpp` 的 64、116 两行；本步按 D8 完整口径做全内核检索，实测
  `rotation * body_effective_inertia_body(...) * transpose(rotation)` 出现 **7 次、跨 6 个
  翻译单元**（`force/layout.cpp`、`joint/kernel_directional_constraint.cpp`×2、
  `output/kernel_output.cpp`、`solve_dynamic/kernel_integrator.cpp`、
  `kernel_integrator_newton.cpp`×2、`kernel_integrator_residual.cpp`）。7 处**全部改为委托**，
  无一处双算（设计 §576 禁止"长期双算并把其中一份称为校验实现"）。计数核对：
  **7 迁移 + 1 定义 = 8**，全仓无第 9 处。
- **步骤 6 位置选择的依据**：放在 `mb_model`（最低层）而非任何 solver 内，因为
  `mb_model → mb_numeric` 是**已登记**的 source edge，故**新增符号未产生新分层边、
  未修改基线**（`check_module_layering.py --strict --final` 独立复核 0 边 0 环）。
  语义理由：世界系旋转是"模型+位姿"的性质，不是求解器离散化的性质。
- **步骤 6 的"无离散系数泄漏"三重证据**：①**签名**——形参只有
  `(model, body, rotation)`，无步长/`alpha_m`/`alpha_f`/`beta`/`gamma`/前一步状态；
  ②**文本**——共享单元与其头文件内检索上述符号 **0 命中**；③**残留**——离散系数仍留在
  solver 侧乘在结果上（`kernel_integrator_newton.cpp` 的 `-h²β`、`(1-α_m)` 位置未变），
  且 `kernel_model_accessors.cpp` 属 `model` 模块而非 solver 模块。
- **步骤 6 的 D2 复审（0 缺陷 + 1 死重载）**：复审逐点核对 7 个迁移点，结论
  **7/7 语义等价**，并特别核查了最危险的**静默互换**风险——两个 R8 目标点分别取
  `evaluation.q[bi]` 与 `next.q[bi]`，是不同引用、不同循环，**未发生互换**（若互换照样
  编译而物理错误）。复审另证伪了重载歧义担忧：`Mat3`/`Quat` 均为无构造函数无转换运算符
  的聚合体，互不可转换，**只可能编译失败、不可能静默选错**。
  复审指出的**唯一实质问题**是我添加的四元数重载**零调用者**（9 处调用点全传 `Mat3`）——
  属投机 API，与 D8"第二处真实使用才抽取"相悖，**已删除**并把理由写入头文件注释，
  防止后人"补全"它。这是**第八次**验证同一模式：自设破坏覆盖不到"我没想到"的维度。
- **步骤 6 的诚实局限**：**性能无法由现有门禁排除小回归**。sentinel 是逐位比对
  （能证数值恒等、证不了时间），`kc_perf_gate` 只给两个比值且本次朝"变快"方向波动
  （×0.935/×0.893），说明噪声不可忽略；**若该门禁无方差界，任何 <10% 回归都能藏进去**。
  这是门禁固有局限、非本次缺陷。本步**不声称"性能无回归"**，只声称"在门禁分辨率内未检出"。
  另记录：`kernel_integrator_newton.cpp:114` 位于 `for(fi)` 内层 `for(row)` 体内，
  每 body 重算 `m` 次——**原代码既有形状**，迁移未引入也未顺手修掉（范围外）。
- **sentinel 非零 acceptance exit code 是既有事实、非本步回归**：脚本自述非零退出
  **属预期**（三个 road case 已知在 `fixture.force_z` 单通道不过 `time_convergence` 门，
  项目如实记录而非放宽用例），sentinel **只在漂移时失败**。实测失败清单与冻结基线
  `dynamic_hash_baseline.json` 的 `failed_cases` **逐项相同（9 个，MATCH: True）**。
  同报告显示 `adams_accuracy_status: BLOCKED`、`accuracy_claim_permitted: False`，
  是 Adams 参考数据缺失的既有阻断，与本步无关、也不因本步缓解。
- 下一步（2026-10-12 重开后）：**P3 未关闭**，步骤 2/5 仍未验收完成（本文件
  「2026-10-12 重开本轮」小节）。同时按核心链推进 **P4a 冻结态刚度柔度**（6 步），
  先读 [P4a SPEC/TODO](../p4a-frozen-local-response/PROGRESS.md) 与
  [SUBTASKS.csv](../../SUBTASKS.csv) 确认依赖就绪。
- 最新实施验证：**characteristics 401 passed；快速集 1958 passed/1 xfailed（skip 0、
  xfail 未增长）；kernel+contracts 176 passed；ruff/ty 0；三架构门 0；
  characteristics 专项 95 条新增；git diff --check 干净；未重录任何基线。**
  步骤 6 触及 native，故已跑 **gate-numeric 三条全通过**：sentinel 逐位不变、
  parity 8 族、perf k-100 ×0.935 / c-66 ×0.893，另 `mb_linear_batch_selftest.exe` exit 0。
- **缺口（步骤 5，不声称已解决）**：
  ①**步数不随请求不变**是真实缺陷面，位于 native 采样循环，P3 读侧无权修改（见步骤 5
  关键结论）；②**无事件场景未覆盖**——本 rig 无接触事件、rejected 恒 0，故 §56 的"事件
  不变"与"步序不变"中**"事件"一半未被本步证据覆盖**（与步骤 4 记录的"事件时间到样本下标
  的映射尚未实现"是同一缺口）；③**多 case 计划下的 `_DIAGNOSTIC_TAIL_ROWS` 未覆盖**
  （单 case 下语义等价，须多 case 计划才能观测）；④**`raw.performance` 全零**是独立可
  改进点——调用方若直接信任它会把预算算成 0，当前 `sampling.py` 绕开了它（正确），但该
  陷阱未在任何地方拦截，建议后续在 `available is False` 时显式说明而非静默返回 0。
- **缺口（步骤 4，不声称已解决）**：`sampled_time_difference` **与真实动态运行未接线**——只吃
  `times`+`values`，真实事件时间来自 `ResultEnvelope.contact_events`
  （`ContactEvent(time_s, element_id, transition)`, `envelope.py:47`），
  **从事件时间映射到样本下标这一步尚未实现**，调用方目前须自己给 `blocks`
  （与 R9「生产消费方」缺口同处，`trajectory.py` 亦无生产消费方）；
  活动模式切换的边界未接入（事件可来自 `contact_events`，模式切换尚无对应通道）；
  **误差量级不可得**（三维模板看不到 `f'''`，只给曲率尺度）；
  **探测**不可微点未实现（`blocks` 只让调用方**声明**，自动识别属 SPEC §64）；
  本模块不处理伪时间（SPEC §39 属采样选择层）；
  `curvature_scale` 含 `span²` 故跨网格不可比（当前无调用方）。
- 步骤 3 遗留缺口（仍未解决）：`trajectory.py` 只在手工构造的一致状态上验证，未接真实
  `body_state` 输出；SPEC §38 的完整偏导矩阵 `g_q q̇+g_v v̇+g_z ż+g_u u̇+g_t` **未**实现，
  只做到可区分、可拒绝。
- 最新实施验证：**test_sample_selection.py 19 passed；characteristics 325 passed（原 306）；
  快速集 1882 passed/1 xfailed（skip 0、xfail 未增长）；kernel+contracts 176 passed；
  ruff/ty 0；三架构门 0；git diff --check 干净；未重录任何基线；破坏 caught 3/3。**
  未跑 gate-numeric：本步只改 Python 读取层，未触及 native/求解路径，不满足
  AGENTS.md §2 触发条件（步骤 6 会跑）。
- **缺口（不声称已解决）**：`accepted` 与拒绝步身份仍需 native 侧逐接受步持久化
  （SPEC §84 点名的风险，本步以如实拒绝回应、未缩小它）；`require_selection` 的拒绝分支
  当前不可达（无生产者产生部分选择），保留为防御性代码并如实标注；
  `_FakeResult` 的 `times_s` 与真实的按 case 截断拼接有一处保真度差异；
  多 case 用例基于手工 ledger 布局，偏移约定由 C++ 三处互证 + `raw.py` 三处既有读取确认，
  非端到端复现。

## 2026-10-12 重开本轮：真实 accepted 生产者（步骤 2 主体 / 步骤 5 一部分）

- **交付**：`RunSampleObserver`/`observe_sample` 只读观察**原提交点**（导出 C 结构布局不变、
  `observer == nullptr` 时零工作量）；`accepted_state` 输出请求（准静态族显式拒绝）；
  五个结果块 + manifest 列名与版本；Python `select_samples("accepted")` 校验后枚举；
  导出按检查点自身身份出表。
- **本轮证据**：[raw/accepted-producer-evidence.md](raw/accepted-producer-evidence.md)。
- **实测（全部本轮执行）**：`test_native_accepted_sampling.py` **25 passed**；
  `mb_run_sample_selftest` exit 0（同一次 `run_model` 有/无 observer，state/diagnostics/energy/
  力元素账本/contact_events **逐位相同**，含接触事件与自适应两半）；
  characteristics **472 passed**；快速集 **2034 passed / 1 xfailed**（skip 0、xfail 未增长）；
  kernel+contracts **182 passed**（新注册两条自测）；ruff/ty 与三架构门 exit 0；
  sentinel 逐位不变、parity 8 族、perf 见下。
- **缺陷修复 5 处（红-修-绿，全部实测）**：时间容差硬编码 `1e-12` 误拒合法长运行；
  状态列接受非前缀/越界值（`[1,2]`、`[NaN,1]`）；输出映射完整性未与**成功样本数**核对；
  manifest 列名未校验；`frozen_working_sample` 文档称"不读 a/alpha"与冻结协议矛盾。
  2 处 sabotage 反证判别力后已还原（`grep -c SABOTAGE == 0`）。
- **perf 归因（对照实验）**：`kc_perf_gate` 在高负载时 `x1.30~x1.47` 超 `BUDGET_FACTOR=1.25`；
  `git stash` 退回**全部** `packages/` 改动（HEAD `5808f53`）重建后立即测同一门**同样 FAIL**；
  恢复改动、负载回落后 **OK**（`x1.189`/`x1.127`）。故与本次改动无关；
  **未重录基线、未放宽 `BUDGET_FACTOR`**。
- **仍缺（故步骤 2 不 DONE）**：`indices` 显式集合选择无入口；
  多 case 动态运行的 accepted 身份无端到端用例。
- **步骤 5 仍 TODO**：本轮只补同计划开关不变性（§3）；输出体积/峰值内存/组装分解分项计时未复测。

## 2026-10-12 续本轮：`indices` 落地、真实多 case 读侧验收、两处形状缺陷

- **`indices` 已落地**（用户裁定 A：读侧库 API，不改 `SAMPLE_KINDS`）：
  `select_samples(result, kind, *, ids=..., case_index=...)`，`ids` 是**网格自身 per-case ID**
  （`output`→`output_sample_id`，`accepted`→`accepted_step_id`），**不是数组下标**；
  拒绝非整数/负数、重复、不存在、**跨 case 歧义**（须给 `case_index`）、**未按采样顺序列出**；
  `initial` 无自有 ID，带 `ids` 一律拒绝。`require_selection` 透传。
- **上一轮登记的第二个缺口（多 case accepted 端到端）经实测重新定性：不是缺测试，是本版本无生产者。**
  五个动态族展开器各只有**一处** `out.cases.push_back`（`axle_dynamic.cpp:248`、
  `vehicle_dynamic.cpp:322`、`handling.cpp:215`、`ride_four_post.cpp:154`、`ride_random_road.cpp:155`），
  即每个动态族只产出一个 case；多 case 只出现在准静态网格族（`kc_quasi_static.cpp` 4 处，按轴值展开）。
  真实运行实测：单 case 动态计划 `manifest.case_count == 1`；三值 K 网格 `case_count == 3`。
  **已如实登记为「本版本无生产者」，不记为已覆盖**；要补须单独立项改 native 展开器并同步 SPEC。
- **把真实多 case 生产结果纳入读侧验收**（缺口 2 中真正可做的部分）：
  `tests/characteristics/test_sample_selection.py` 新增 5 条，跑真实三 case K 网格；
  测试内先断言 `case_count == 3`，防止断言在退化为单 case 时变成空洞。
- **本轮新发现并修复 2 处真实缺陷（红-修-绿）**：`results/characteristics.py` 的
  `sample_identities` 与 `require_channel` 写死 `result.raw`，传原生 `RawContractResult` 时抛
  `AttributeError`（同模块 `sampling.py` 三处均用 `getattr(result, "raw", result)`）。
  后果：`select_samples` 在**同一个 run 上**按 kind 决定成败——`accepted` 可用、
  `output`/`initial` 崩溃，调用方会把它读成实现 bug 而非契约限制。
  sabotage 改回原式 3 条用例 FAIL，还原后 `grep SABOTAGE == 0`。
- **另补步骤 2 验收断言**：自适应运行**确有拒绝试探**（`rejected` 合计 3，先断言 `> 0` 防空洞）
  而 accepted 身份数**只由接受步计数决定**（`== 1 + sum(accepted_steps) == 39`，
  `< 1 + accepted + rejected`，ID 连续、时间严格递增）；
  同一 `id=1` 在 accepted 网格（`t=0.0005`）与 output 网格（`t=0.01`）指向不同状态，
  两个网格的 ID **不可互换**。
- **本轮门禁（实跑退出码）**：`ruff` 0；`ty` 0；三架构门 0/0/0；
  characteristics **488 passed**；`test_native_accepted_sampling.py` **30 passed**；
  `test_sample_selection.py` **30 passed**；快速集 **2051 passed / 1 xfailed**（xfail 未增长）；
  kernel+contracts **182 passed**。本轮只改 Python 读取层与测试，未触 native/求解路径，
  按 `AGENTS.md` §2 不触发 `gate-numeric`；未重录基线、未放宽阈值。
- **步骤 2 关闭口径（用户裁定已下达，2026-10-13）**：原验收条件「initial/accepted/output/indices
  可追溯、拒绝步与不同网格不混淆」四项均已落地并有真实证据；剩余项是**范围外**的 native 展开器
  能力（本版本动态族单 case）。用户裁定：**「单独立项修改 native 动态展开器使多 case 可产，
  再达到原条件后关闭」**。据此已在
  [raw/deferred-native-multi-case.md](raw/deferred-native-multi-case.md) 按 EPIC §3「跨包移交门」
  登记延期工作项（移交方/接收任务/交付内容/验收条件/依赖/最迟集成门齐全）。
  **P3 步骤 2 原条件不放宽，在延期工作项交付并通过验收前保持 `IN_PROGRESS`**。
  是否把该工作项提升为第 10 个顶层子任务（会改变 Epic「9 项全 DONE」的关闭条件）
  须用户对关闭条件本身另作决定；在此之前保持 9 项结构不变。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
