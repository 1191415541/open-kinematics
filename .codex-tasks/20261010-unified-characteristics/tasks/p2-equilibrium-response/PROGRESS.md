# P2 实施进度

## 恢复入口

- 日期：2026-10-10（Asia/Shanghai）。
- 任务：提取共享物理线性化；实现可复用分解、多右端/转置及平衡响应服务，从完整模型按需生成 Wheel/Ride/Roll/载荷角度指标。
- 形态：single-full；状态：IN_PROGRESS。
- 进度：6/6；当前步骤：无（全部步骤已 DONE，待父任务收口）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- 已完成：步骤 1 DONE（2026-10-10）——提取共享物理组装并更新 DAG。
  证据：[raw/module-dag.md](raw/module-dag.md)。
  步骤 2 DONE（2026-10-10）——可复用多 RHS 与真实转置接口。
  证据：[raw/linear-batch-evidence.md](raw/linear-batch-evidence.md)。
  步骤 3 DONE（2026-10-10）——平衡模式响应与可辨识审计。
  证据：[raw/equilibrium-response-evidence.md](raw/equilibrium-response-evidence.md)。
  步骤 4 DONE（2026-10-10）——Wheel/Ride/Roll 与载荷角度纯算子。
  证据：[raw/compliance-metrics-evidence.md](raw/compliance-metrics-evidence.md)。
  步骤 5 DONE（2026-10-10）——兼容请求共享缓存与计量。
  证据：[raw/response-cache-evidence.md](raw/response-cache-evidence.md)。
  步骤 6 DONE（2026-10-10）——独立 oracle、R7/R9 及 R8 边界锁定。
  证据：[raw/retirement-evidence.md](raw/retirement-evidence.md)、
  [raw/validation.md](raw/validation.md)。
- 启动依赖：P1 已 DONE（可用）。
- 下一步：本子任务 6/6 全部 DONE。父任务层面 P2 行仍为 TODO，需按 SPEC §60 更新父状态
  （[SUBTASKS.csv](../../SUBTASKS.csv) 的 P2 行）。之后进入 P3 动态接受态调度。
- 最新实施验证：快速集 **1863 passed / 1 xfailed**（skip 0，xfail 未增长）；
  characteristics **306 passed**（其中 `test_route_equivalence.py` 33 passed）；
  kernel+contracts **176 passed**；ruff/ty 0；
  三架构门 0；gate-numeric 三条全通过（sentinel 逐位相等、parity 8 族、perf c-66 ×0.922）；
  `git diff --check` 0；未重录任何基线。
- 交付后复审（D2）：步骤 2/3 交付代码经独立只读复审，**发现并修复 8 个真实缺陷**
  （Python 5 个 + C++ 2 个；另确认 1 处非问题并为其补断言）。其中 3 条会产出错误结果
  或崩溃：溢出后无理由崩溃、逐模式绝对阈值误拒合法载荷、`native_transpose` 分支
  复制的是前向分解。每条都补了会失败的回归测试，详见
  [raw/equilibrium-response-evidence.md](raw/equilibrium-response-evidence.md) §7。
  步骤 4 交付代码经第二次独立只读复审，**再发现并修复 6 个真实缺陷**，
  最严重的一条在步骤 3：`LoadMode.kind` 经 `LOAD_MODE_COMPONENTS` **决定扰动落在哪一行**，
  旧的 `roll_moment:5` 在 `[Fx,Fy,Fz,Mx,My,Mz]` 下是 **Mz（偏航）**而非侧倾，
  会让侧倾指标求解偏航系统却返回格式完好的数；详见
  [raw/compliance-metrics-evidence.md](raw/compliance-metrics-evidence.md) §6。
  步骤 5 交付代码经第三次独立只读复审，**再发现并修复 3 个真实缺陷 + 6 项契约/命名/
  覆盖问题**，最严重的一条是缓存键上可选 `pattern_digest` 默认空串：稠密→稀疏的
  **真实模式变化**被记成 `structural_reuse`，理由写着"非零模式未变"，
  即设计 §5.3 明令必须失效的情形被记成其反面并发放"可复用符号分析"的许可；
  详见 [raw/response-cache-evidence.md](raw/response-cache-evidence.md) §7。
  **教训（第三次确认）**：自测蓄意破坏不足以覆盖全部盲区——该步自测 8 处破坏
  一个都没碰到这三项缺陷，因为它们都落在"我没想到要破坏"的语义维度上
  （可选字段的省略、第二条返回值路径、字段表示与同包既有契约的一致性）。
  独立复审是必要的第二道。
  步骤 6 交付物经第四次独立只读复审，**再发现 3 个真实缺陷 + 3 处过度声称**。
  最严重的是 **A1：差商用绝对步长 `h = 1e-7`**——载荷量级 ≳ `1e4` 后被浮点吸收，
  `loads = 1e8` 时相对误差达 **1.08e-02**，且**减小 `h` 会使其更差**
  （`h=1e-12` → 3.7e-05，`h=1e-13` → 4.0e-04）；A1b 更隐蔽：缩放基准必须是
  **整个载荷矩阵**而非被扰动那一行，否则 `loads=[[1e8,0],[-1,3]]` 中第 0 列精确、
  第 1 列误差 **6.3e-04**（我最初的诊断脚本只测第 0 列，据此误判已修好）。
  修后横跨 `1e-8..1e12`（20 个数量级）最差 **3.8e-11**。
  **教训（第四次确认，且这次更尖锐）**：A1 **躲过了我 18 项自设破坏**——那些破坏
  改的是结构与校验，而 A1 是**数值量级**问题，需要量级离谱的输入才暴露，
  我的夹具最大只到 `1e3`，距失效起点差一个数量级。
  **自设破坏的覆盖面受限于设计者自己的想象，只有独立复审能突破它。**
  详见 [raw/validation.md](raw/validation.md) §3.2–3.4。

## 步骤摘要

- 步骤 6 [DONE 2026-10-10] 独立 oracle、R7/R9 及 R8 边界锁定。
  新增 `tests/characteristics/oracle_equivalence.py`，以**四种算术**求同一响应算子
  `C = G A^{-1} B_w + D`：`direct`、`transpose`（`A^T Z = G^T` 再 `Z^T B_w + D`）、
  `unit_basis`（逐模式单位载荷——SPEC §57 字面点名的"独立全单位基投影"，
  **初版漏做，由复审发现**）、`finite_perturbation`（整行扰动差商）。
  非对称夹具实测 `transpose_gap = 2.78e-17`、`unit_basis_gap = 3.47e-18`、
  `perturbation_gap = 7.53e-10`，均在 P0 容差内。
  **两处语义纠正**：① 差商求的是 `dY/dP = C` 而非 `Y`，初版拿导数与响应相比
  得到 0.63 的**假分歧**，改为四条路线统一比较算子 `C`；
  ② 算子列轴是 `B_w` 的 modes 而非请求的载荷列数，按 `base` 尺寸分配会静默
  返回 `(measurements, 1)` 块——形状合理、内容错误。
  **杠杆效应**：四条路线没有一条是万能 oracle，价值来自覆盖并集——
  `transpose` 独有捕获"`A` 被转置"，`unit_basis` 与 `finite_perturbation`
  独有捕获"`B_w` 列被合并/错序"（direct 与 transpose 都消费 `B_w`，同错同过）。
  R7 台账：`static_jacobian`（`kernel_static_trim.cpp:358`）实测为**纯委托**，
  文件内无第二份组装算术（全文 404 行逐段核对）；状态记
  `migrated → verified`，**如实不记 `retired`**（P6 才收口全部消费者）。
  R8 精确片段锁定（本步唯一交付，迁移阶段为 P3/P4a）：`residual` 在
  `kernel_integrator_residual.cpp:28`、`constraint_jacobian` 调用 :142/:143；
  `fill_analytic_jacobian_columns` 在 `kernel_integrator_newton.cpp:24`，
  `body_effective_inertia_body` 同一表达式出现在 :64/:116（D8 抽共享单元候选）。
  **边界是结构性的而非注释**：实测 `mb_linearization` 传递头闭包 8 个中
  **0 命中** `solve_dynamic`/`integrator`，`WorkingSample` 全 `cpp` 唯一定义且
  5 字段无任何离散量，故物理响应**无法**读取离散系数。
  状态 `pending`（阶段未到，不越权声称已迁移）。
  蓄意破坏 **19/19 全部被捕获**，模块逐字节恢复 `clean`；
  同时判定**三个语义等价变异**并如实记录理由（把等价变异当漏检去修
  = 记录假阴性）。
  证据见 [raw/retirement-evidence.md](raw/retirement-evidence.md)、
  [raw/validation.md](raw/validation.md)。

- 步骤 1 [DONE 2026-10-10] 提取共享物理组装并更新 DAG。
  新增 C++ 模块 `mb_linearization`（`include/mb_linearization/types.hpp` +
  `src/linearization/physical.cpp`），成为"力律导数 + `constraint_jacobian` +
  `d(J^T lambda)` 预载几何项"的**唯一归属**；`static_jacobian` 改为向下委托。
  **关键实测**：设计 §19 字面点名的 `pose_candidate` / `static_gauge_coordinates` /
  `static_position_constraint_jacobian` 实际声明在 solver 模块（新模块**之上**），
  调用它们会造成 `mb_solve_static → mb_linearization → mb_solve_static` 环
  （初版编译即失败）。故改为**作为数据传入**（`WorkingSample.pose/gauge/
  position_constraint`）：共享模块只做组装算术，不重新推导调用方约定；
  新模块 include 集合内无 solver 头，环在结构上不可能。
  这正是 SPEC §36"物理 vs 策略"的落点：位姿参数化与"哪些行算位置约束"是**调用方约定**，
  两个 solver 可合法不同，放进共享模块等于把一方选择冻结成另一方契约。
  证据与过程见 [raw/module-dag.md](raw/module-dag.md)。

- 步骤 2 [DONE 2026-10-10] 可复用多 RHS 与真实转置接口。
  新增 `mb_linear/batch.hpp` + `src/linear/kernel_linear_batch.cpp`，交付
  `analyze_pattern` / `factorize_numeric` / `solve_many` / `solve_transpose_many`。
  **先核实再实现**：实测 8 个后端全部单 RHS、全部无转置。唯一转置记号
  `kNoTranspose='N'`（`factorization_types.hpp:339`）被**硬编码**进 `getrs`（:443）
  请求**前向**系统——它是"没有转置"的证据，不是"有"。两个诚实取舍：
  ① 字段命名 `caller_visible_pattern_step`，6 个后端全记 `false`：PARDISO 的符号
  分析（:476/:638/:652/:597）内嵌在首次 `factor` 中且**无公开 `analyze()`**，
  记 `true` 会让调用方报告一个它拿不到的规划步；
  ② 无后端支持转置，但拒绝转置会让接口无用，故 handle **保存原矩阵**、精确构造
  A^T 并对其分解，返回 `factorizations_used=2` + `last_transpose_rebuilt`，
  **不对称矩阵特判**（特判会让快慢路径数值不一致）。
  **反向验证**（见证据 §4）：用 A 冒充 A^T → 4 条 FAIL；谎报分解复用 → 1 条 FAIL。
  测试矩阵刻意非对称，否则转置缺陷会表现为通过。
  证据见 [raw/linear-batch-evidence.md](raw/linear-batch-evidence.md)。

- 步骤 3 [DONE 2026-10-10] 平衡模式响应与可辨识审计。
  新增 `results/equilibrium.py` + `tests/characteristics/test_equilibrium_response.py`。
  公式 `A X = B_w P`、`Y = G X + D P`；复用 P1 的 `audit_boundary`/
  `rank_and_null_space`（秩容差全仓库唯一，不另写一套判据）。
  **两次自我纠正，都由"蓄意破坏测试"暴露**：
  ① 初版写了逐模式**测量梯度**可辨识检查，禁用后 12 测试全过——追查发现它在
  方阵下**不可达**（`A` 非奇异 ⇒ `X` 唯一 ⇒ `X` 的任何线性函数都唯一可辨识），
  是死代码；真正需要逐模式判定的是**载荷侧**（某模式 `B_w P[:,j]` 为零时
  `A^{-1}0 = 0` 会返回格式完好的零，而全局审计看不见）。已改为判载荷列范数。
  ② 初版拒绝路径返回 `np.zeros`，与同文件原则冲突（零填充的拒绝与真实零响应
  数值上无区别）；改为返回 NaN。
  三种破坏全部被逮住：最小范数解回答秩亏系统 / 丢弃 `D P` / 禁用逐模式检查。
  证据见 [raw/equilibrium-response-evidence.md](raw/equilibrium-response-evidence.md)。

- 步骤 4 [DONE 2026-10-10] Wheel/Ride/Roll 与载荷角度纯算子。
  新增 `results/compliance.py` + `tests/characteristics/test_compliance_metrics.py`
  （**42 passed**）。本层每个函数只消费**已算出的响应**、自身不触发新求解：
  率是**倒数**，极小的位移响应会变成极大的率，这里是"貌似合理的错数"最容易产生处。
  Wheel Rate = 同相 heave 模式左右垂向位移响应的倒数；左右**分别**判定；
  **负值保留**（`abs` 只用于阈值，从不作用于输出——设计要求带稳定性诊断）。
  Ride Rate 两条路径分开命名：已分离用串联式，完整接地点响应取倒数且
  **不再套串联式**——关键断言是额外传入 `Ks/Kt` 后答案必须不变，
  否则再套一次会得到更软的数且不可察觉；半套串联被拒（设计 §456 禁止
  "关掉轮胎重算一个伪 Ks"，此处靠"必须成对传入"结构性实现）。
  Roll Rate = `ΔM/Δφ`；`roll_rate_per_degree` 是**独立函数而非开关**且校验输入单位，
  转换两次是静默 57 倍误差。
  载荷角度只做 `G_angle @ X + D_angle P`，签名**不接受任何位姿**，
  故设计点名的"把 `rx/ry/rz` 当 Toe/Camber"在接口上不可能发生（非靠注释禁止）。
  拒绝路径一律不带数字（`values=()`）带具名理由。
  **反向验证 12 处破坏全部被逮住**，其中一处（车轮两侧只判左侧）**首轮存活**，
  暴露非有限响应检查无任何断言覆盖——`1/inf == 0.0` 会静默产生有限零、
  读作"无限软"（与真意相反），补测试后才逮住。详见证据 §7。
  证据见 [raw/compliance-metrics-evidence.md](raw/compliance-metrics-evidence.md)。

- 步骤 5 [DONE 2026-10-10] 兼容请求共享缓存与计量。
  新增 `results/cache.py` + `tests/characteristics/test_response_cache.py`
  （**57 passed**）。缓存键覆盖设计 §8.1（515 行）的全部成分：模型/计划、样本、
  域/版本、边界 profile+version、冻结协议 protocol+version、活动模式。
  **四条关键取舍**：
  ① **非零模式一律由矩阵观测、键不接受调用方声明**。旧版键上可选 `pattern_digest`
  默认 `""`，于是稠密→稀疏（一个非零变零）时两者 `structural` 完全相同、
  `_find_structural` 命中、决策记成 `structural_reuse` 且理由写着"非零模式未变"——
  把设计 §5.3 明令必须失效的情形记成其反面，并发放"可复用符号分析"的许可。
  旧 42 用例全部漏检，因为唯独覆盖不到默认空 digest 的路径，而空 digest 恰是
  测试辅助函数的默认值：**断言了部署而非行为**。
  ② **结构复用与数值复用严格互斥且可分**：值变而模式不变 → `structural_reuse`；
  模式变 → `reassembled`；`_find_structural` 纳入 `sample_index`，
  故跨样本不会被记成"复用了上一点的分析"。
  ③ **响应缓存在上、分解缓存在下**：分解键不含指标/测量/请求 ID，
  故不同指标不阻断同一 `A` 的分解共享（设计 515 行）——已用装配计数器证明
  换指标后装配次数仍为 1；该测试同时是"把指标折进分解键"这一错误的唯一防线。
  ④ **`tangent` 与 `held_tangent` 均返回副本**。后者由 `factorize` 改名而来
  （本层从不分解，分解在响应层由 `numpy.linalg.solve` 完成、批量回代归 C++
  `solve_many`；以 "factorize" 之名交出矩阵会让调用方误以为持有分解句柄）。
  该路径的别名问题比 `tangent` 更隐蔽：`_Entry.matrix_digest` 不随之更新，
  篡改**永远不被察觉**，下次同值请求照样走数值复用分支。
  计量只记真实发生的工作：`factorization_count` 不再被复用路径自增
  （原实现数字随工作量下降而**上升**，与事实反向），`assemble=None` 时不再虚计装配，
  峰值内存 `merge` 取 max 而非求和。
  **蓄意破坏 14/14 全部被捕获**；其中 3 项对应复审发现的缺陷（D1/D2/D3），
  自测的 8 处破坏一个都没碰到。证据见 [raw/response-cache-evidence.md](raw/response-cache-evidence.md)。
  缺口：本模块尚无生产消费者（响应层未接线）；输出体积计量需序列化层参与。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
