# P4a 实施进度

## 恢复入口

- 日期：2026-10-13（Asia/Shanghai）。
- 任务：在原动态工作样本实现 frozen_force_tangent 和 frozen_load_compliance，按具名载荷/测量直接输出局部响应及 Wheel/Ride/Roll/Toe/Camber 序列。
- 形态：single-full；状态：**IN_PROGRESS**。
- 进度：**3/6 步 DONE**（2/5 DONE）；步骤 1/3 IN_PROGRESS（本轮补齐 `frozen_inertial_load_v1`、B_w 符号与 gauge 力臂，剩余见文末）；**步骤 4 IN_PROGRESS（2026-10-13 起：第 1 层 native 求解核心已落地，
  见下「2026-10-13 本轮」）**；步骤 6 FAILED（retry_count=1）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。

## 步骤 1：DONE（冻结协议与原样本残量审计）

- **交付物**：`frozen_working_sample`（`mb_linearization`）+ `gauge_coordinates`
  （从 `mb_solve_static` 下沉到 `mb_model`，旧位置改为委托）+ 独立自测
  `cpp/tests/frozen_sample_selftest.cpp`（8 组断言，注册进 CMake）。
- **验收证据**：[raw/frozen-protocol-evidence.md](raw/frozen-protocol-evidence.md)（8 节）、
  前置侦察 [raw/frozen-recon.md](raw/frozen-recon.md)。
- **关键结论**：`WorkingSample` 的五个字段全部由**模型级数据**供给，无新策略概念；
  冻结样本的 `pose` 取样本 19 列的 `r`/`q`/`v`/`omega`，加速度/角加速度在样本里
  （`copy_state`，`output/kernel_output.cpp:27-38`）但**切线装配路径不读**，故不在必需字段内。
- **分层**：`mb_linearization` 在求解器之下、契约"never calls back up"，故
  `gauge_coordinates` 必须下沉到 `mb_model`；实测 **0 环、无新边**。
- **D2 独立复审**：发现 **4 个真实缺陷，全部已修并补测**——
  ①最严重：`false` 返回时 `working` 已被部分填充，会让调用方拿着**上一状态**的矩阵
  且尺寸仍匹配模型、装配器会放行（静默复用陈旧数据）→ 已把所有拒绝测试前移到写入之前；
  ②`v`/`omega` 未做有限性检查（力律会读）→ 已补；③`constraint_jacobian` 维度前置条件
  未检查（`int` 乘积可回绕成巨量分配）→ 已加防御性校验；
  ④原注释论证对象错误（"mask 恒全 1"的真正要害是"无路径置 0"，非"循环是空操作"）→ 已重写并标脆弱点。
- **投毒验证**：**12/12 全部被抓、0 存活**（含"拒绝前就写 sample"的顺序回归）。
  其中发现并修正一次**无效变异**（插在所有拒绝测试之后 ⇒ 语义等价），
  以及一次**真实测试缺口**（`rows=0` 时"留空"与"正确计算"都是空矩阵，形状断言假过 →
  已给模型加真实约束行）。还原后 SHA256 校验通过。
- **门禁**：全绿，退出码全 0。

## 步骤 2：DONE（坐标驱动的冻结态反力切线）

- **交付物**：`frozen_force_tangent_column`（`mb_solve_static`）+ 独立自测
  `cpp/tests/frozen_tangent_selftest.cpp`（8 组闭式断言，注册进 CMake 与 pytest）。
- **验收证据**：[raw/frozen-tangent-evidence.md](raw/frozen-tangent-evidence.md)（8 节）。
- **算法**：分区 Dirichlet 解。设 `A = dR/dx`，`delta = [ds ; dq_free]`，
  `A_freefree · dq_free = -A_freeheld · ds`，输出 `Q_s = (A·delta)[driven]`。
  功共轭**由构造保证**（`ds^T Q_s` 即保持力作的功）。
- **K=k 解析通过**（SPEC §56）：单弹簧 `[[500]]`、`ds=0.01` → **+5.0**；
  串联 `[[200,-200],[-200,500]]` → **120.0** = `k1*k2/(k1+k2)`（对角写法会得 200，
  故此值**证明分区求解真的发生**）；耦合端口 → `{5.0,-25.0}`（含非对角项）。
- **符号约定**：返回**保持该位移所需的外部力**，故恢复弹簧报**负**值；
  在契约文档中**只陈述一次**，避免被翻转两次。
- **拒绝语义**：越界/重复端口、长度不符、切线尺寸过短与过长、NaN、奇异自由块、
  乘子坐标留自由块——**共 7 类，全部在写入之前完成**，断言语料逐字节未被覆写；
  奇异**不做最小二乘近似**（那会把欠约束机构呈现为"有刚度"）。
  零反力是**合法成功**，不与失败信号复用（反证变异被抓住）。
- **分层纠正（本步最重要的设计决定）**：初版把声明放在 `mb_linearization`，
  分层门**立即失败**。根因是位置错误——端口求解需要 `mb_linear`，
  而 `mb_linearization -> mb_linear` **非登记边**，且该模块契约明写"永不向上调用"。
  "哪个坐标被按住"是**求解器策略**而非物理装配，故移至 `mb_solve_static`
  （该模块 `-> mb_linear` 与 `-> mb_linearization` 两条边**均已登记**）。
- **头文件边如实登记**：新增 `mb_solve_static -> mb_linearization` **头文件**边。
  实测 `reviewed_new_edges` 中**已存在**同一 `edge` 条目（R7 委托遗留），
  故只补 `header_edges`，**未改任何 ledger**；变更仅 **+67 字节**，
  `source_edges`/`reviewed_new_edges`/`cycles`/`mutual_edges` 逐字节不变。
- **D2 独立复审发现 1 个严重缺陷（已修）**：新自测**没有任何调用者**——
  `KERNEL_SELF_TESTS` 只登记三个既有自测。**已独立复核**（全仓检索四个自测名，
  结果只有 `CMakeLists.txt`）：断言买到的质量是 **0**。
  已在本步于 `tests/test_registry_consistency.py` 注册，kernel+contracts **176 → 177**。
- **复审同时确认正确的点**：Dirichlet 消元符号与转置结构（复审以串联弹簧手工复算
  `Q = 200 - 200*0.4 = 120`，与实现一致）；`model.ndof <= dimension` 恒成立故端口索引安全；
  `mb_linearization` 的"永不向上调用"契约未被破坏。
- **投毒验证**：**10/10 全部被抓、0 未解释存活**（符号、非对角、held 未移除、
  右端未移项、自由解丢弃、重复/越界端口、长度不符、尺寸检查放宽、零反力当拒绝）。
  另有 **1 条语义等价变异如实登记**：忽略 `factor()` 失败后自测**仍然通过**；
  经**探针实测**机制为——`solve` 读未消元因子给出**非有限**值被 `isfinite` 拦下，
  两条防线在所有可达输入上重合。故**单列为 `EQUIVALENT` 并每轮重测**
  （行为变化会显示 "NO LONGER EQUIVALENT"），**不声称能抓住它**。
- **门禁**：全绿，退出码全 0。

## 步骤 3：DONE（具名载荷模式响应及可辨识诊断）

- **交付物**：`frozen_load_column`（`B_w = -dR/dw`，`mb_solve_static`）+ 共享谓词
  `frozen_load_gauge_coordinate` + 独立自测 `cpp/tests/frozen_load_selftest.cpp`
  （7 组断言，注册进 CMake 与 pytest）。
- **验收证据**：[raw/frozen-load-evidence.md](raw/frozen-load-evidence.md)（8 节）。
- **意义**：`B_w = -dR/dw` 此前**全仓库零实现**，是 P4a 的三个真实缺口之一。
  本步补上载荷侧输入后，SPEC §57 的 `C = 1/k` 第一次成为可算的量。
- **复用而非新造（D6）**：载荷通道就是既有的 `SampleInput::body_wrench`
  （`types.hpp:533`，`bodies*6`，每体 `[Fx,Fy,Fz,Mx,My,Mz]`），且端到端已存在——
  施加 `force/layout.cpp:148-190`、力臂 `:172` 在求解位姿处解开（正是 SPEC §35/§42
  要的几何项）、具名模式与同相/反相镜像 `kc_quasi_static.cpp:545-597`、
  Python 侧 `signal_bus.py:461`/`axle_channels.py:562` 均已接。**未新增载荷概念。**
- **列是解析的**：`B_w[k] = +1` 在坐标 `6*fi + slot(k)`（`load_scale = 1`），
  `slot = Fx..Fz → 0..2, Mx..Mz → 3..5`；力在声明点另加
  `+cross(R*p, e_k)` 于转动块，**在给定位姿处求值**。
- **C=1/k 解析通过**（SPEC §57）：用载荷列作右端真解一次，弹簧坐标移动 `1/500`。
  **同处断言坐标驱动路径报 `k`**（500）——把 `k` 与 `1/k` 放在一个测试里，
  使"用求逆或倒数切换物理边界"必然失败（SPEC §37、设计 §422 明确禁止）。
- **六个分量逐一断言落位**：只测 `Fx` 会放过把所有力映射到第一个坐标的实现；
  置换实现只有逐个断言才抓得住。力矩模式的力臂断言落在**转动块**——
  只看平动块会放过"力臂错误地对力矩生效"。
- **D2 独立复审发现 2 个 high 缺陷，均已修并补回归**：
  ①**gauge 行被写入 `-1`**——该行是恒等行（`physical.cpp:144-147`）/纯位姿量
  （`kernel_static_trim.cpp:77-82`），真实 `dR/dw = 0`。后果**不是"一个错元素"**：
  它给一个**不能承载右端的行**伪造右端，`A X = B_w P` 解出 `X[gauge] = -1`
  并经 `Y = G X` 泄漏到所有测量。已提取共享谓词同时识别全局 gauge mask 与
  `static_rotation_gauge_for_pivot`（后者已在 `mb_model/functions.hpp:72`，**0 新边**）。
  ②**缺 `load_scale` 因子**——真实调用者会传非 1 值
  （`kernel_static_contact.cpp:270-271`）。**不加参数而改为明确契约**：冻结协议自身
  把 `working.load_scale` 钉为 `1.0` 并写明理由（`physical.cpp:350-353`），
  加一个只有一种合法取值的旋钮反而制造误用面；原注释"scaled by unity"读起来像在
  陈述 `layout.cpp`，已重写为对入参的约束。
- **复审还指出我的 `C=1/k` 用例近乎恒真**（手写对角除法，信息量等于已断言过的
  `column[0]==-1`），判定准确，**已如实登记**；新增的 gauge 回归用例是当前唯一能
  抓住缺陷①的构造。
- **复审确认正确**：映射无转置/off-by-one；符号未被翻两次（`B_w = +1` 与设计 §410 一致）；
  力臂符号自洽；**五道守卫逐条完备、不存在可达越界读写**；两个索引空间
  （`pose.q` 按体、`column` 按自由坐标）未被混淆；与步骤 2 的路径**无共享可变状态**。
- **投毒验证**：**13/13 全部被抓、0 未解释存活**（含 4 条 gauge 变异：
  gauge 坐标获得列、旋转 gauge 规则去掉、全局 mask 忽略、力臂忽略 gauge 规则）。
  另 **3 条语义等价变异如实登记并每轮重测**：固定体（`base < 0` 二次守卫）、
  零标记（`cross(0,e) = 0`）、负 body（越界读值不确定、恰落在界内——
  **它能逃过断言的原因正是这条守卫存在的理由**）。
- **门禁**：全绿，退出码全 0。**`layering_baseline.json` 未改（0 新边）**；
  `dynamic_hash_sentinel` **逐位不变**——这是"只增能力、不改既有物理"的最强证据。

## 步骤 4：DONE（贯通逐样本派生指标与请求共享）

- **交付物**：`results/per_sample.py`（新增）——`SampleQuestion`/`SampleAnswer`/
  `SampleSequence`/`PerSampleReport`/`drive_sample_questions`；
  `results/equilibrium.py` 改（`audited_modes` 接受调用方 `mode_matrix`、
  解除测量行数限制）；`tests/characteristics/test_per_sample_sequences.py`
  （新增，**46 用例**，含 11 个第二轮复审回归用例）。
- **验收证据**：[raw/frozen-derived-evidence.md](raw/frozen-derived-evidence.md)（9 节）。
- **本步交付的是调度与簿记，不是新的物理算子**。求解算子 P2 已有、载荷侧输入
  步骤 3 已补；本步回答三个此前无人回答的问题：
  ①**同一份 A 分解几次**——循环**样本在外、问题在内**，逐样本一次；
  `PerSampleReport.counters` 让共享**可被读出**而非只是承诺。
  ②**某样本解不出怎么办**——记为该样本**拒绝**（NaN + 非空原因），不抛、不丢邻居。
  ③**两条序列同源凭什么**——`provenance` 记边界 profile/version + model/plan 指纹 +
  冻结协议与版本；序列公开 `mode_ids`/`units`/`sample_indices` 与逐样本 identity。
- **复用而非新造（D6）**：求解复用 P2 的 `audited_modes`，因为它是**全仓唯一**
  逐模式拒绝权威。其下的 `equilibrium_response` 是**全局**审计，会把
  `B_w P[:,j] = 0` 的情形发布为 `X = A^{-1} 0 = 0`——一个格式良好的零，与
  "机构确实不响应这个载荷"**无法区分**。任何把调用点下沉的重构都会静默丢掉这条拒绝。
  本步**未**新造求解器、未平行实现。
- **关键实现决策**：切线是 `drive_sample_questions(tangents=[...])` 的**参数**而非
  问题字段（它属于**样本**；做成问题字段会让同一采样点的两个问题对"在问哪个机构"
  产生分歧而答案照样合理）；`mode_ids` 从 `question.modes` 取、**不从数组宽度反推**。
- **本步最重要的产出是一个设计缺陷，而不是功能**。首轮交付已通过 35 个自测、
  11 道静态门、23/23 蓄意破坏，仍被**第二份独立对抗性复审**找出 **6 处真实可复现缺陷**，
  其中 3 处由探针**当场复现**。共同形态是**"合法输入 ⇒ 静默的错误有限值"或
  "拒绝路径形状不一致"**，而不是崩溃：
  - **1a/1b（同一根因）**：响应块行宽原先**从"哪个模式答出来了"反推**，于是"全部拒绝"
    退化为 `(1, n_modes)`，且"拒绝"与"已答"两种样本宽度不同 ⇒
    `IndexError` / `np.stack ValueError`。**已修**：行宽改为**请求的属性**
    （`_answer_rows()`：有测量 ⇒ 测量数；无测量 ⇒ 切线维）。
    **形状必须来自请求，不能来自答案。**
  - **2**：具名测量缺 `G` 时 `_stack_measurements` 返回 `None`，下层**提前返回机构运动
    `X`**，把机构坐标冒充为该测量名下的**有限有效值**（实测 `[[0.5]] valid=[True]`）。
    这是本模块**第三个**"合法输入 ⇒ 静默错误有限值"（前两个：兄弟缺 `D` 丢整块、
    逐模式拒绝缺失）。**已修**：请求期拒绝缺 `G` 的测量。
  - **3**：`mode_matrix` **装饰性**——`audited_modes` 原先只用 `load_mode_matrix(modes)`
    重建 `P`，调用方数值**从不进入** `A X = B_w P`，`LoadMode.point` **从不被读**；
    单位基与 ×3 幅值给出**同一答案**。**已修**：加 `mode_matrix` 具名参数并贯通
    （实测 `[0.5,0.0]` vs `[1.5,0.0]`）。
  - **4**：曾**拒绝设计自己的规范请求**——设计 §6.7:447 要求"一个同相模式、一次求解、
    同时测 `h_left` 与 `h_right`"（Wheel Rate 形状），原实现要求
    `measurement_gradient` 行数 ∈ {1, 模式数}，把**测量数**与**模式数**混为一谈；
    而 `G` 是 `(测量, ndof)`、`X` 是 `(ndof, 模式)`，`G X` 对任意测量数都有定义。
    **已修**，实测一模式两测量 `factorization_count == 1`。
  - **5**：`SampleAnswer.answered` 测**从不出现**的 `"underdetermined"` 串 ⇒ 恒 `True`。
    **已修**：读 `audit['status']` 且无 `refused_modes`。
  - 另修：序列数组存**转换后**的值并**冻结**、已答样本不得携带 NaN、
    问题 id 含保留分隔符拒绝（键必须可逆）、测量项形状错误按名拒绝而非逃逸 `IndexError`。
- **复审者核实为正确、未被改动**：切线**按样本**取（非按"样本×问题"）与 `sample_index`
  缓存键；请求形状校验各按名抛；`_row` 的**模式主序**展平；`statuses_for` 关闭了
  "`valid=True` 却全 NaN"的门。
- **投毒验证**：**32/32 全部被抓、0 存活、0 跳过**；`restore verified: True`、
  `sha256 match: True`、还原后基线仍绿。**`EQUIVALENT` 清单清空**——上一版登记的
  "被拒列保留响应返回的任何内容"经**重新测量**已**不再等价**（新的逐模式逻辑使该守卫
  承载语义），故移入突变表，这正是"等价声明必须随代码变化重测"的例子。
- **一次真实的断言缺口**：第 32 条突变（"校验转换局部量、却存调用方原件"）首轮
  **SURVIVED**，原因是全部用例都传 `np.ndarray` 而 `np.asarray(ndarray)` 返回同一对象，
  两种情况不可区分；补"传 list + 断言字段是 `ndarray` 且 `dtype` 已归一"的用例后转为
  **CAUGHT**——**证明幸存突变确实指向断言缺口，而不是噪声。**
- **方法学局限（如实保留）**：复审者**无法运行代码**，其结论中"异常类型"与
  "`P` 行基约定"两类由我**探针实测**确认；其余读推导结论我已按代码核实，
  但**未**为每条都构造独立探针。
- **门禁**：全绿，退出码全 0。**`layering_baseline.json` 未改（本轮 0 新边）**；
  `dynamic_hash_sentinel` **逐位不变**。快速集 **2004 passed/1 xfailed**，
  较 1993 的增量**恰为本步新增的 11 个回归用例**，**不是**产品行为变化；
  kernel+contracts **178 passed 不变**。

## 步骤 5：DONE（独立有限扰动 oracle 与直接/转置核对）

- **交付物**：独立 oracle 自测 `cpp/tests/frozen_oracle_selftest.cpp`（6 个测试，
  本步在既有 5 个之上新增测试 4b）+ `CMakeLists.txt` 目标注册 +
  `tests/test_registry_consistency.py` 门禁注册（**解析**输出而非 grep）。
- **验收证据**：[raw/frozen-oracle-report.md](raw/frozen-oracle-report.md)（8 节）。
- **意义**：步骤 1–3 各自对照"由构造已知的答案"，共同盲区是**三者互相自洽却联合错误**。
  本步用一条**不信任解析切线**的路径补上盲区：perturb 冻结状态、重新求值
  **与被测对象不同源**的 `static_residual`，要求解析切线预测残量变化且误差
  **按二阶速率下降**（实测 4.136e-04/4.138e-06/4.138e-08，比值 100/100）。
  速率是判别性的：加性误差 `dA` 只降 ~10 倍，阈值 50 落在两态之间。
- **本步最重要的产出是一个覆盖缺口**：交付时测试 3/4 用手工矩阵，
  而 SPEC §51 / 设计 §439 要求"**同一物理问题**含**非对称切线**"。
  探针实测 **oracle 物理夹具（`rows == 0`）的装配切线完全对称**
  （`worst|A-A^T| = 0.000000e+00`）——转置核对在其上**真空**。
  已新增**测试 4b** 在**真实且受支持**的非对称冻结切线上核对
  （对称衬套 + 一条 `AXLE_INPLANE` 约束行，`dim = 7`，`worst|A-A^T| = 1.44`）：
  ①直接/转置一致 **1.110e-16**；②单位缩放等价 **0.0**（未补偿 `G` 反证相对 1.448）；
  ③前向/转置解**确实不同**（1.49）；④`A` 预测**残量**变化（1.809e-13）而 `A^T` **不能**
  （1.011e-03，判别力约 1e10）。
- **交付后经独立只读复审（`code-reviewer`）发现并修复两处真实缺陷**：
  ①**夹具不可达**——初版用非对称刚度衬套制造切线非对称，但 `build_model.cpp:326` 与
  `element_reader.cpp:311` **都拒绝**非对称 bushing 刚度，等于在受支持模型无法产生的矩阵上
  检验装配器；已改为上面的受支持来源。
  ②**切线只与自身比较**——初版只比较同一装配矩阵上的两条路线，`K^T` 写法会全过；
  已加"切线必须是残量的导数"判别块（上一条 ④）。
- **修 ② 的连带发现**：突变"伴随误用前向"**再次 SURVIVED**，探针实测根因是**结构性**的——
  右端与测量都只触及位姿坐标时，`A^{-1}` 的位姿块对称（对称因子的 Schur 补），
  故两条路线**恒等**；已把测量 `G` 改为同时读**乘子（约束反力）坐标**，
  误用前向即差 **1.34**（随即 CAUGHT）。
- **自设破坏 7 条：6 CAUGHT / 1 SURVIVED（预期）**。唯一存活的突变 4
  （放宽**测试自身**阈值）为等价变异，如实登记、不假装能抓住
  （真实差异 1.49 远高于两种阈值）。
- **诚实局限**：oracle **有界**——加性误差 ≲1e-1 藏于真实的 `O(h^2)` 截断项下，
  分辨率约 **1e-4 相对切线误差**。这是分辨率、不是漏检；
  **不声称"切线精确到机器精度"**。
- **清理**：`packages/suspension_kernel/cpp/tests/probe_minimal.cpp` 是**上一轮遗留的
  调试探针**（无任何引用、未注册进 CMake），已移出工作区。
- **门禁**：全绿，退出码全 0。`build-kernel` + `build-axle-native` 均 0
  （kernel 改动后产品 DLL 必须重跑）；kernel registry **16 passed**；
  characteristics **447 passed**；快速集 **2004 passed/1 xfailed**（同基线）；
  `gate-architecture` 0 环 0 新边、基线未改；`gate-numeric` sentinel **逐位不变**、
  parity 8 族、perf 预算内；`ruff`/`ty` All checks passed；`git diff --check` 干净。
  **未重录任何冻结基线**；新增分层边 **0**（改动全在 `cpp/tests/` 与 Python 测试）。

## 依赖与启动

- **启动依赖核对（已通过）**：P2、P3 均 6/6 步 DONE 并关闭，有 `raw/validation.md` 真实证据；
  `SUBTASKS.csv` 中两者状态为 `DONE`。
- **真源**：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- **下一步**：步骤 6 — 验证四组合覆盖及成本并归档。需交付"P0 动态目标不缩减、矩阵
  capability 独立、R8/R9 阶段证据与门通过"，证据落点 `raw/coverage.csv`、
  `raw/validation.md`。步骤 5 已完成：独立有限扰动 oracle、直接/转置与单位/frame
  等价均已行为验收（见上）。剩余缺口见下「遗留缺口」。
- **最新实施验证**（2026-10-11，步骤 5）：`build-kernel` 与 `build-axle-native` exit 0；
  `mb_frozen_oracle_selftest`（6 测试）/`mb_frozen_tangent_selftest`/
  `mb_frozen_load_selftest`/`mb_linear_batch_selftest` 全部 exit 0；
  kernel registry **16 passed**；characteristics **447 passed**；
  快速集 **2004 passed/1 xfailed**（同基线，skip 0 未增长）；kernel 116 / contracts 64 passed；
  `gate-architecture` 0 环、0 新边、基线未改；`gate-numeric` sentinel **逐位不变**、
  parity 8 族、perf 预算内；ruff/ty **All checks passed**；`git diff --check` 干净；
  本步新增自设破坏 **6 条 5 抓 1 等价**（见 raw/frozen-oracle-report.md §3.1）。
  未跑 `just test-all`（按 AGENTS.md 属 P6 收尾项）。

## 遗留缺口（随步骤推进更新）

## 步骤 6：FAILED（2026-10-11，retry_count=1）

- **已完成**：按 P0 目录逐场景生成 [raw/coverage.csv](raw/coverage.csv)，并记录 R8/R9 阶段状态、矩阵视图独立性、P0 成本项及本轮实际门禁；汇总见 [raw/validation.md](raw/validation.md)。
- **实际验证**：内核/native 构建 exit 0；五个冻结/批量自测 exit 0；动态哈希哨兵 exit 0 且冻结基线逐位一致；8 类 parity exit 0；性能门 exit 0；ruff/ty、三架构门、447 个 characteristics 测试、2004 passed/1 xfailed 快速集、kernel 116、contracts 64 全通过。
- **阻断**：缺少原动态样本到冻结装配的生产适配、具名轮端到 `body_wrench` 映射、原生 `local_response_v1`/capability/result 块和输出侧逐测量单位/frame 接线；因此四组合动态端到端行为和 P0 特性路径组装/分解/回代/序列化/内存/体积预算不可测。步骤 6 不得 DONE，P4a 保持 IN_PROGRESS。
- **下一步**：补齐上述共享 ABI/结果链后重试步骤 6；不以非空 capability 声明替代实际应答，不将矩阵 `pending_reference` 或 Adams `pending_definition` 当作已验证。

1. **载荷列未跨 ABI 导出**：导出矩阵与声明 `characteristics_response_kinds`
   **必须同时做**；当前该数组仍为**空**。
2. **`local_response_v1` 结果通道未接线**（SPEC §30/§33 的模式 ID/测量 ID/
   分量单位/有效性）。**步骤 4 已在 Python 层给出等价的具名序列**
   （`SampleSequence` 带 `mode_ids`/`units`/`sample_indices`/identity 与 provenance），
   但**未跨 ABI 输出**；逐测量的单位仍未出现在序列上（多测量单位不一致时返回空串）。
3. **命名轮端模式 → `body_wrench` 的映射层未实现**（左/右、力/力矩轴、作用点）；
   设计 §436 的 `adams_wheel_center_12_v1` 布局属可选项，独立登记。
4. **组合相容性/可辨识诊断**：消费层 `audited_modes` 的**逐模式**拒绝已在步骤 4 接入
   （含"组合有效而单列无效"的 `refused_modes` 记录）；但**组合载荷**（设计 §437
   "某组合相容而各单列不相容时应直接求组合"）仍未实现，且 `load_mode_matrix`
   硬编码 `load_dimension=6`，使 `compliance.heave_mode()` 的 2 分量
   `p=[left,right]` 不能用于本路径（步骤 4 调用方传 `(6, n_modes)` 绕开）。
5. **单位与 frame *换算*（characteristics 层）未做**（设计 §435：作用于相应测量与载荷
   映射，不能对整组输出乘常数）——这是**输出侧**的单位/frame 转换。
   **步骤 5 已给出求解侧的证据**：在真实冻结切线上，坐标单位缩放 `x = S x'` 下
   物理响应不变（相对 4.337e-19），frame 旋转下切线按 `A' = T A T^T` 同余
   （worst 1.137e-13）。二者证明**求解层不泄漏内部 scaling / frame**；
   输出层的单位换算与命名仍缺。
6. ~~转置/伴随求解路径未验证~~ → **步骤 5 已覆盖**：手工非对称矩阵上
   `A^T x = b` 真解（伴随与直接一致 1.110e-16），真实非对称冻结切线上
   直接/转置一致 2.168e-19 且前向/转置解确实不同（2.06e-04）。
7. ~~有限扰动独立 oracle 未做~~ → **步骤 5 已覆盖**：见
   [raw/frozen-oracle-report.md](raw/frozen-oracle-report.md)；**诚实局限**是
   oracle **有界**（分辨率约 1e-4 相对切线误差，更小的加性误差藏于 `O(h^2)` 截断项下）。
8. **动态样本 → 装配的适配器未实现**：本步接受已装配的 `pose`/`model`，
   从 `AxleDynamicsResult` 样本的接线尚缺。
9. ~~`mb_frozen_sample_selftest` 在 `test_registry_consistency.py` 进程内崩溃（0xC0000005）~~
   → **2026-10-12 已注册且本模块内连跑 3 次不复现**（每次 18 passed，1.0~1.2 s）；
   `pytest packages/suspension_kernel/tests` 由 180 增至 **182 passed**。
   **根因仍未定位**（曾推断的 `active_mask` 越界写已被证伪：HEAD 版该自测
   `assemble_physical_tangent` 调用次数为 0），故只陈述"不复现"而不声称已解释。
10. **`factor`/`isfinite` 双防线冗余无注释**（步骤 2 遗留），当前无缺陷。
11. **`mb_linear_batch_selftest` 同样无调用者**：既有缺口，不属本步权限。
12. **`frozen_protocol`/`frozen_version` 缓存键已预留但未接线**。
13. **P2 步骤 1 / P3 步骤 1 的专项 D2 复审仍未做**（历史遗留）。
14. `position_constraint` 与静态路径的等价性**依赖 mask 不变化**，脆弱点已在两处注释标出。
15. **coupler 路径未由步骤 1/2/3 测试覆盖**（刻意：合成 coupler 只能证明"我搭的那两条能过"，
    真实覆盖由步骤 6 的 `case_parity_check.py` 8 族承担）。
- **侦察关键结论（详见 [raw/frozen-recon.md](raw/frozen-recon.md)，不在本块重复）**：
  ①`WorkingSample`（`mb_linearization/types.hpp:37-58`）**就是**冻结协议所需的数据结构，
  五个字段全以**数据**到达；②`assemble_physical_tangent` 已与 Newton 迭代解耦，
  `x` 是通用增量、**可在任意状态求值**，拒绝语义已是"报告式"；
  ③消费层（`equilibrium.py`/`compliance.py`）完整但**无输入生产者**。
  **三个真实缺口**：①`B_w = -dR/dw` 全仓库零实现（**步骤 3 已补**）；
  ②物理 tangent 不跨 ABI；③capability 刻意声明应答能力为零
  （`kernel_contract_run.cpp:229-243`），**实现后必须同时加入该数组**。
  **实现决策**：复用静态侧 `assemble_physical_tangent`，**不用**动态侧 `ResidualContext`
  （后者携带九个离散系数，会违反 SPEC §41 与设计 §555）。
  **三个数据可得性问题全部已解决**：样本 19 列含加速度/角加速度
  （权威定义 `output/kernel_output.cpp:27-38` `copy_state`）；
  乘子有现成块 `constraint_multiplier` `(samples,rows)`（`:1115`，经 `envelope.py:172`）；
  约束 Jacobian 有现成块 `(samples,rows,ndof)`（`:1119`，经 `envelope.py:310`）。
  **`gauge`/`position_constraint` 已确定为模型级数据，无新策略概念**（步骤 1 已实现，见上）。

## 2026-10-12 本轮：`frozen_inertial_load_v1`、B_w 符号与 gauge 力臂

- **证据**：[raw/frozen-inertial-load-evidence.md](raw/frozen-inertial-load-evidence.md)；
  旧文档的符号口径纠正见 [raw/frozen-load-evidence.md](raw/frozen-load-evidence.md) 文末追加段。
- **步骤 1 补齐**：`frozen_working_sample` 现读 `a/alpha`（尺寸 + 有限性）、拒绝越界/重复
  `active_tires`，用既有共享 `body_world_inertia` 逐自由体算
  `f = m*a`、`moment = I_world*alpha + omega × I_world*omega` 存入 `inertial_wrench`
  并置 `frozen_inertial_load`；切维改为 `ndof + rows`；`static_residual` 新增可选
  `frozen_inertial_wrench` 参数，以**公共力总线的同一套算术**替换总线惯性项；
  新增报告式 `frozen_inertial_residual`。
- **步骤 3 修复两处真实缺陷（红-修-绿）**：
  ① **符号**：原 `frozen_load_column` 写 `dR/dw`，与声明的 `B_w = -dR/dw` 相反。
  新增 `test_load_column_is_negative_derivative_of_real_residual`，对真实 `static_residual`
  六分量中心差分逐行断言 `B_w == -dR/dw`，旧符号下先红；修正后 exit 0。
  ② **gauge 作用域**：原实现遇 gauge 平移坐标**整列 return**，丢掉力臂力矩。
  新增 `test_a_gauge_translation_still_carries_its_lever_arm_moment` 先红后绿，带无 gauge 反向对照。
- **步骤 1/5 新 oracle**：惯性 wrench 逐分量精确等于 `(4,-6,8,12.3,-9.4,2.4)`（`1e-12`）；
  6 坐标中心差分断言**实际残量导数 == 解析切线**，并断言"把惯性当活量"的版本差异 > 0.1（判别力）；
  轮胎用例断言世界速度固定下 `tangent = 10000`、分析后 `v`/`tire_sx`/`tire_sy_dot` 逐位未变。
- **真实动态运行侧**：`mb_run_sample_selftest` 对**真实运行**的每个非输出接受态求冻结残量，
  初始一致点残量 `1.14e-16`（平衡原惯性），自适应与固定步长两条路径都跑过。
- **实测**：`mb_frozen_sample/load/oracle/tangent/runsample_selftest` 全部 exit 0；
  characteristics 472 passed；快速集 2034 passed / 1 xfailed；kernel+contracts 182 passed；
  ruff/ty 与三架构门 exit 0；sentinel 逐位不变、parity 8 族、perf 见 P3 证据 §6 的对照实验。
- **仍缺（故步骤 1/3 不 DONE）**：设计 §6.5 的"lambda 基值取原态、`delta lambda` 联立求"
  无独立断言；目标力律 capability 审计未由本步产出；组合模式相容/唯一性诊断、完整整车耦合、
  逐测量单位/frame 换算、原生 `local_response_v1` 结果块、`characteristics_response_kinds`、
  具名轮端→`body_wrench` 映射层均未实现。

## 2026-10-13 本轮：步骤 4 第 1 层（native 载荷驱动求解核心）

- **证据**：[raw/local-response-layer1-evidence.md](raw/local-response-layer1-evidence.md)（9 节）；
  只读设计 [raw/local-response-design.md](raw/local-response-design.md)。
- **用户裁定（2026-10-13，C5）**：本轮直接开始 P4a 步骤 4 实施（先写只读设计，再逐层改 native/Python）。
- **交付（第 1 层）**：`FrozenLoadMode` / `FrozenMeasurement` /
  `frozen_measurement_gradient` / `frozen_local_response`
  （声明 `mb_solve_static/functions.hpp:148-278`；实现 `kernel_static_trim.cpp:717-963`）；
  自测 `cpp/tests/frozen_local_response_selftest.cpp`（7 组断言，注册进 `CMakeLists.txt` 与
  `tests/test_registry_consistency.py`）。
- **关键结论（省下大量重复侦察）**：**`G` 不需要新物理**。`A` 的前 `ndof` 行/列就是各自由体的
  6 个增量，而 `pose_candidate`（`solve_dynamic/kernel_integrator_state.cpp:39-55`）规定
  `r += dr`（世界系平动）、`q = exp(dtheta)*q`（世界系转动，绕体原点），故材料点位移为
  `dr + dtheta x (R(q) p)`，其沿世界轴 `e` 的分量即 `dr·e + dtheta·(a x e)`（`a = R(q) p`）。
  转角测量取相对姿态的世界系小转角分量。`A`/`B_w`/`G` 因此描述**同一组坐标**。
- **`A X = B_w P` 的求解复用 `mb_linear`**：`LinearFactorHandle::factorize_numeric` 一次，
  逐模式回代；`factorizations_used == 1` 由自测断言（设计 `shared_factorization` 由构造成立）。
- **两轮反证（判别力）**：①力臂 `cross(arm,unit)` 与参考体符号 `-=` 破坏 → **exit 1，4 条断言失败**；
  ②参考体力臂符号 + 空请求拒绝破坏 → **exit 1，3 条断言失败**；两次均逐字节还原。
- **独立只读复审（D2）4 项阻断全部核实成立并已修**：
  ①头文件宣称「相容/可辨识已实现」但**根本不存在** → 改为如实登记缺口（不解除要求）；
  ②无效项被写 `0.0`，违反 SPEC §98「不填零」→ 改 **NaN**；
  ③`solve_many` 任一列非有限即**整批失败**（`linear/kernel_linear_batch.cpp:161-167`），
  使「逐模式隔离」与实现相反且我的分支不可达 → 改为**逐模式单列求解**；
  ④补转动测量（component 3..5）、`has_reference_point` 力臂、纯固定体零响应、拒绝哨兵共 4 类断言。
- **复审未指出、由我自查发现的测试缺陷（第 3 次同一模式）**：第一版新增断言把「体 1 的 rz」
  写成 `row+5`、把「参考体（体 0）的 rz」写成 `row+11`，且单测量时用 `row+列` **越界读** `G`
  （C++ 不报错）。**是断言写错、不是实现错**；已改绝对列号并先断言 `gradient.size()`。
- **门禁（本轮实跑）**：`build_suspension_kernel` + `build_axle_native` exit 0；
  `mb_frozen_local_response_selftest` exit 0；registry **20 passed**；kernel **130 passed**；
  contracts **78 passed**；快速集 **2051 passed/1 xfailed**（skip 0，未增长）；
  sentinel 逐位不变、parity 8 族；ruff/ty/三架构门 0；`git diff --check` 干净；
  perf 为**同一负载抖动**（k-100 ×1.265）且本层代码**无生产调用者**；**未重录任何基线**，
  `layering_baseline.json` 未改（`mb_solve_static → mb_linear` 已是登记边）。
- **仍未交付（故步骤 4 不 DONE）**：相容/可辨识审计（`singular`/`unidentifiable`/
  `combined_mode_rule`）；`D` 恒为 0；**`G` 宽度 `ndof` 与 Python 消费层要求的 `dimension` 不一致**
  （接线前必须对齐，否则接线即报错）；固定/自由车身 profile 无落地处；
  契约绑定（测量/模式的实体与分量声明，需按 SPEC §21 以新 kind/version 回写契约）；
  ABI `local_response_v1` 结果块与 `characteristics_response_kinds`；Python 读侧与单位/frame；
  四组合端到端验收（步骤 6）。

## 2026-10-13 本轮：步骤 4 第 3 层（契约绑定被 native 消费 + local_response_v1 跨 ABI）

- **状态**：**已实现并编译通过，未经测试验证**。用户于本轮中途裁定「先提交更改代码和任务记录」，
  故本层在移植完成、`kernel` 构建 exit 0 之后即收工，**未跑任何自测、未做反证、未跑全量门禁**。
  下列描述是**实现内容**而非已验证能力；行为正确性与「无请求时不改变原结果」两条都**尚无证据**。
- **交付 ①：契约绑定被求解器真实消费**（此前最需要登记的诚实缺口）。
  新增 `resolve_local_response_bindings`（`cpp/src/abi/kernel_contract_run.cpp`，ABI 层）：
  - 测量 `binding{marker,component,reference}` → `FrozenMeasurement`：标记经
    `ContractModel::find_marker` 查表、体名经 `body_index`，**空 `reference` = world**（显式空串已在
    `case_characteristics.cpp` 被拒，两者因此不可混同），测量落在**固定体**（`body_to_free < 0`）
    在请求期拒绝并**点名该声明**。
  - 扰动 `binding.terms[{body,wrench,amplitude}]` → `FrozenLoadMode`：`amplitude` 稠密铺在
    `bodies*6` 的 `[Fx,Fy,Fz,Mx,My,Mz]` 上，同一 (体, 分量) 相加而非丢弃。
  - 落点在 ABI 是因为只有这一层同时看得见两侧：`mb_cases` 拥有文档且**被禁止**触达求解器，
    `mb_solve_static` 拥有冻结描述且读不了文档。**契约禁令遵守**：无任何按名字前缀/后缀推断。
- **交付 ②：逐样本 A 的生产者**。接在既有 `accepted_observer` 内，只用该观察者已经拿到的
  `RunSample`：`frozen_working_sample(model, sample.state, {})` + `interpolate_input` +
  `assemble_physical_tangent(..., x)`，其中 `x` 的乘子块**取自样本自身的 `constraint_multiplier`**
  （`lambda` 基值取原态，**不冻结 delta lambda**，符合 `frozen-protocols.json` 的 `lambda_rule`），
  `active_tires` 为空（冻结样本自带接触态，不引入压缩未知量）。
  **无请求零额外工作**：整段 gate 在 `answers_local_response` 上，`false` 时既不分配缓冲也不进循环。
- **交付 ③：`local_response_v1` 跨 ABI**。结果块
  `local_response_value` / `_status` / `_modes_per_sample` / `_audit` / `_identity`
  （`push_block` 与 `append_doubles` 顺序一致），manifest 新增 `local_response` 段
  （`load_modes`/`measurements`/`measurement_units`/`measurement_components`/`status_vocabulary`/
  `audit_columns`/`identity_columns`/`refused_samples`/`frozen_protocol`/`frozen_version`）。
  状态码为**冻结枚举** 0 `valid` / 1 `unidentifiable` / 2 `singular` / 3 `refused`；
  值块保留 NaN、状态块承载**原因**，两者不互相推断。
- **交付 ④：`characteristics_response_kinds` 由空数组改为 `["local_response"]`**。
  口径与 Python 编译期门**同一字段**（`measurements[].response_kind`，
  `compilation/resolved.py:309-310`），故两门不会互相打架；native 应答门亦按同字段判定。
- **未做（下一步）**：Python 读侧与逐测量单位/frame 换算、指标层
  （Wheel/Ride/Roll/Toe/Camber）、固定/自由车身 profile 落地、四组合端到端验收（步骤 6 重试）；
  `D` 恒为 0 仍是已登记缺口。**步骤 4 保持 IN_PROGRESS**。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
