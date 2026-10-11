# P2 步骤 6 退役台账：R7 / R8 / R9

> 授权：SPEC §52（步骤 6）· SPEC §59（验收条件）· 设计 §10.1 表（R7/R8/R9 行）·
> 设计 §10.3（台账字段与退出条件）· §11 表 P2 行。
>
> 台账字段按 §10.3 要求逐项记录：ID、精确旧文件/符号/代码段、新归属、已迁消费者、
> 保留公共入口及理由、状态、证据、退出条件。

## 0. 状态总览

| ID | 范围 | 本步状态 | 退出阶段 |
|---|---|---|---|
| R7 | static 切线组装副本 | **migrated → verified**（本步） | P2/P6 |
| R8 | 动态残差/解析 Jacobian 的物理调用组合 | **pending**（本步只锁定精确片段，不迁移） | P3/P4a/P6 |
| R9 | 新特性路径的重复求解/分解 | **migrated → verified**（本步，防重复约束） | P2/P4a/P6 |

R8 在本步**有意保持 pending**：设计 §555 行把它的阶段定为 P3/P4a/P6，本步的职责是
"P0/P2 迁移记录精确列出抽出函数/代码段"。本步交付的是那份精确清单（§3），不是迁移。

---

## 1. R7 —— static 切线组装

### 1.1 精确旧位置

| 项 | 内容 |
|---|---|
| 文件 | `packages/suspension_kernel/cpp/src/solve_static/kernel_static_trim.cpp` |
| 符号 | `static_jacobian`（当前 **358 行**）、`static_residual`（当前 **31 行**） |
| 被抽出的代码段 | 力律方向导数 + `constraint_jacobian` + `d(J^T lambda)` 预载几何项的组装算术 |

### 1.2 新归属

`mb_linearization`：

| 文件 | 符号 |
|---|---|
| `include/mb_linearization/types.hpp` | `struct WorkingSample`（`:37`）、`assemble_physical_tangent`（`:78`）、`assemble_pose_tangent`（`:92`）、`physical_tangent_dimension`（`:101`） |
| `src/linearization/physical.cpp` | 块序 `[pose | multiplier | contact compression]` 的组装实现 |

### 1.3 委托证据

`static_jacobian`（358-401 行）现在是**纯委托**：

```
WorkingSample working{ gauge, active_tires, load_scale, candidate,
                       static_position_constraint_jacobian(model, candidate) };
if (x.size() != physical_tangent_dimension(model, working)) return false;
return assemble_physical_tangent(model, sample, working, x, jacobian);
```

文件内**不存在第二份组装算术**——366-370 行明确记录了这一取舍："a duplicate would
agree today and drift the first time either side is edited"（注释块整体为 366-375 行）。

### 1.4 保留在 solver 的责任（设计 §554 行要求）

| 保留项 | 证据 |
|---|---|
| `load_scale` | `WorkingSample.load_scale`，由 solver 传入 |
| 活动集策略 | `active_tires` → `active_mask` / `static_compression`（43-50 行），在 solver 内构造 |
| 残量尺度 | `static_residual` 的 `force_residual` / `position_residual` 出参 |
| gauge / pin | `static_gauge_coordinates(model)`（377 行），在 solver 内求值 |
| 静态行选择 | `static_position_constraint_jacobian`（392 行），作为**数据**交给共享模块 |

### 1.5 关键结构决策：为什么 gauge 是数据而非调用

设计 §19 字面点名 `pose_candidate` / `static_gauge_coordinates` /
`static_position_constraint_jacobian` 应"共享"，但实测这三个符号声明在 solver 模块
（新模块**之上**）。直接调用会造成
`mb_solve_static → mb_linearization → mb_solve_static` 环——初版编译即失败
（`fatal error: mb_linearization/types.hpp: No such file or directory` 之后的第二个
真实阻塞）。

改为**作为数据传入**后，新模块 include 集合内无 solver 头，环在结构上不可能。
这同时是 SPEC §36"物理 vs 策略"的落点：位姿参数化与"哪些行算位置约束"是**调用方约定**，
两个 solver 可以合法不同；把它们放进共享模块等于把一方的选择冻结成另一方的契约。

### 1.6 状态

`migrated → verified`。验证：`check_module_layering.py --strict --final` 报
`OK: layering matches the recorded baseline`（无 `mb_linearization → mb_solve_static` 边）；
数值门三条全通过。**P6 才收口全部消费者**，本步不宣布 retired。

---

## 2. R9 —— 新特性路径的重复求解/分解

设计 §556 行：R9 治理"新特性生产路径中逐指标重复调用一次性 `solve_linear`、重复分解/
模式求解及本地指标或表布局副本"。§558 行明确：**R9 是实施中的防重复约束，不声称当前
已经存在新特性路径**。

### 2.1 台账

| 项 | 内容 |
|---|---|
| 旧位置 | 不存在——设计 §558：目前没有新特性路径，也没有可列为"已退役"的现有特性写表实现 |
| 新归属 | 响应合并与分解共享归 `results/cache.py`；指标归 `characteristics/`；表布局归 `io/characteristics.py`（P6） |
| 已迁消费者 | 本步交付的缓存机制**尚无生产消费者**（响应层未接线，见缺口） |
| 保留项 | 既有其他 `solve_linear` 调用**不在退役范围**（§556 末句）；`io/artifacts.py` 的 JSON/NPZ 读写继续保留（§558） |
| 状态 | **migrated → verified**（作为防重复约束已落实，不是"已退役某旧实现"） |

### 2.2 已落实的防重复性质

| 性质 | 实现位置 | 锁定测试 |
|---|---|---|
| 同一 `A` 只数值分解一次，跨指标共享 | `SampleTangentCache.tangent` | `test_a_second_metric_does_not_re_factor_the_same_matrix` |
| 分解键**不含**指标/测量/请求 ID，故不同指标不阻断共享 | `SampleTangentCache.response_key` 与 `FactorizationKey` 分离 | 同上 |
| 跨样本不复用数值分解 | `FactorizationKey.numeric` 含 `sample_index` | `test_a_different_sample_never_reuses_the_previous_factorization` |
| 跨域/边界/冻结协议版本不复用 | `FactorizationKey` 全字段 | `test_no_reuse_across_any_condition_boundary`（9 参数化） |
| 结构失效与数值失效分别判定 | `structural` vs `numeric` 身份 | `test_a_changed_value_at_the_same_pattern_reuses_the_structure_only` |
| 独立右端按实际数量计，不按提问次数计 | `CacheCounters.record_solve` | `test_independent_right_hand_sides_are_counted_not_questions` |
| 拒绝跨条件复用并逐项列出差异 | `assert_no_condition_reuse` | `test_condition_reuse_reports_which_factor_differed` |

### 2.3 为何这一条能判 verified

R9 的验收（§556）是"同 A 分解次数及独立右端符合预算，所有特性消费者使用统一响应/通道
描述，无临时并行写入器"。本步可判定的部分是**前两条的机制已存在且被测试锁定**；
"所有特性消费者"要等接线（P4a/P4b）。因此台账记 `verified` 的是**机制**，
`pending` 的是**消费者覆盖**——两者分开记，不以前者冒充后者。

---

## 3. R8 —— 精确函数与代码段锁定（本步唯一交付）

设计 §555 行要求 P0/P2 "迁移记录精确列出抽出函数/代码段"。以下为**实施时实测**的精确
清单，非设计文档的转述。

### 3.1 目标符号（实测行号）

| 文件 | 符号 | 行号 |
|---|---|---|
| `cpp/src/solve_dynamic/kernel_integrator_residual.cpp` | `residual(...)` | **28** |
| `cpp/src/solve_dynamic/kernel_integrator_newton.cpp` | `fill_analytic_jacobian_columns(...)` | **24** |

### 3.2 候选可共享的物理调用组合

`kernel_integrator_residual.cpp` 内，`residual` 已经调用的**物理原语**（这些是被抽出的
候选——它们是与 static 路径共有的物理调用，而非积分策略）：

| 行号 | 调用 | 性质 |
|---|---|---|
| 142 | `constraint_jacobian(model, next)` | 物理：约束雅可比，与 static `:41` **同一个原语** |
| 143 | `constraint_jacobian(model, evaluation)` | 同上，在 alpha 插值态上 |
| 133 | `ctx.performance->constraint_jacobian_calls.fetch_add(...)` | 计量，**留 solver** |

`kernel_integrator_newton.cpp` 内：

| 行号 | 调用 | 性质 |
|---|---|---|
| 64, 116 | `rotation * body_effective_inertia_body(model, bi) * transpose(rotation)` | 物理：刚体有效惯量在体/世界系间变换。**同一表达式出现两次**（两条评估方向），是 D8"第二次出现即抽共享单元"的候选 |

### 3.3 明确保留在 solver 的部分（设计 §555 行逐项）

| 保留项 | 实测位置 | 理由 |
|---|---|---|
| `ResidualContext` | `kernel_integrator_newton.cpp:782` `newton_step(const ResidualContext& ctx, ...)` | 积分语义类型；SPEC §36 禁止服务引用 |
| 积分状态混合 | `kernel_integrator_residual.cpp:83-116`（`tire_sx/sy/u/u_dot/v/v_dot/beta/beta_dot/phi` 的 `alpha_f` 插值） | alpha 系数是积分策略 |
| 步长 / alpha 系数 | `ctx.alpha_f` 全部出现处 | 离散化参数 |
| 离散约束行 | `kernel_integrator_residual.cpp` 内约束行组装 | 与 static 行选择不同的离散形式 |
| Newton 重试 | `kernel_integrator_newton.cpp:782` `newton_step` | 求解读取策略 |
| 缓存 | `NewtonLinearizationCache`（`factorization_types.hpp:2293`） | SPEC §36 与 §8.1 均禁止复用为物理切线 |

### 3.4 边界锁定结论：不得泄漏的项

设计 §555 行末句要求"无离散系数泄漏到物理响应"。以下为**结构性**保证，非注释约定：

1. `mb_linearization` 的 include 集合内**无** `solve_dynamic` 头（由
   `check_module_layering.py --strict --final` 的分层门约束）。
2. `WorkingSample` 的字段（`gauge` / `active_tires` / `load_scale` / `pose` /
   `position_constraint`）**不含任何时间离散量**——没有 `alpha_f`、没有步长、
   没有前一步状态。物理响应在结构上无法读取离散系数。
3. R8 迁移尚未发生，故当前不存在"双算"风险（设计 §576 行禁止"长期双算并把其中一份
   称为校验实现"）。本步只锁定清单，P3/P4a 迁移时旧位置必须委托。

### 3.5 状态

`pending`（字段齐全，阶段未到）。退出条件（§555 行）：P3/P4a/P6；"调用共享物理原语；
`ResidualContext`、积分状态混合、步长/alpha 系数、离散约束行、Newton 重试及缓存留
solver；无离散系数泄漏到物理响应，积分与事件基线不变"。

---

## 4. 退出条件复核（设计 §10.3）

| §10.3 要求 | 本步证据 |
|---|---|
| 每项记录 ID/精确旧符号/新归属/已迁消费者/保留入口及理由/状态/证据/退出阶段 | 本文档 §1–§3 逐项给出 |
| 不能以"新模块能运行"替代旧生产路径退出 | R7 记 migrated（原处已委托，非"能运行"）；R9 不声称有旧实现可退役；R8 明确 pending |
| 尚有生产消费者走旧算法则不能记 retired | R7 记 migrated→verified，**未记 retired**；P6 才收口 |
| 兼容入口保留而算法体退出可记 retired 但须注明保留符号 | 本步无 retired 条目 |
| R8 的抽出边界不得停留在泛称目录 | §3.1/§3.2 给出文件+符号+行号 |
| R7/R8 共享提取完成后旧位置必须委托，不能长期双算 | R7 已委托（§1.3）；R8 未提取故无双算 |
| 独立 oracle 只放测试范围 | `tests/characteristics/oracle_equivalence.py` 在 `tests/` 下，不在 `src/` |

---

## 5. 独立 oracle 与等价性证据（SPEC §57）

### 5.1 四条路线

`tests/characteristics/oracle_equivalence.py` 实现四条**算术上不同**的路线，各自独立
求同一个响应算子 `C = G A^{-1} B_w + D`：

| 路线 | 算术 | 独立性与其局限 |
|---|---|---|
| `direct` | `G @ solve(A, B_w) + D` | 基准 |
| `transpose` | `Z = solve(A^T, G^T)`，`Z^T @ B_w + D` | 用 `A^T` 与第二次分解；恒等式 `G A^{-1} B = (A^{-T} G^T)^T B` 使其成立 |
| `unit_basis` | 逐模式以单位载荷 `e_m` 求解后拼列 | 不用任何载荷向量（SPEC §57 字面点名）。捕获**列**级缺陷（`B_w` 列被合并/错序）；**不能**捕获转置的 `A`（它与 direct 同用 `solve(matrix, ...)`） |
| `finite_perturbation` | 整行扰动 → 差商 | 不显式形成 `A^{-1} B_w`；需按载荷量级缩放步长（见 §5.8） |

实测（非对称 `A`）：`transpose_gap = 2.8e-17`、`unit_basis_gap = 3.5e-18`、
`perturbation_gap = 7.5e-10`——三条独立路线均在机器精度/截断误差级别与直接路线一致。

### 5.1b 各路线捕获的缺陷类别（不夸大）

| 缺陷类别 | direct | transpose | unit_basis | finite_perturbation |
|---|---|---|---|---|
| `A` 被转置（`A^T` 当 `A`） | — | **✓** | — | — |
| `B_w` 列被合并/错序/漏乘 | — | — | **✓** | **✓** |
| `D` 丢失或错位 | — | **✓** | **✓** | **✓** |
| 步长/除数错误 | — | — | — | **✓** |

四条路线**没有哪一条是万能 oracle**；等价性检验的价值来自它们的覆盖并集互补。

### 5.2 为什么必须非对称

在对称 `A` 上 `A^T = A`，转置路线退化为直接路线的复述。若把 `A` 误当 `A^T` 使用，
对称夹具**测不出来**。因此所有夹具故意非对称，并由
`test_the_fixture_is_not_symmetric` 断言（而非注释约定），
`test_the_unit_basis_route_sees_a_transposed_tangent` 与
`test_substituting_the_tangent_for_its_transpose_is_detected` 另行证明该替换会改变答案。

### 5.3 有限扰动路线的语义澄清（本步实测纠正）

**关键**：差商求的是 `dY/dP = C`，**不是** `Y`。直接路线施加于某个载荷得到的是 `C P`。
把导数与响应相比会产生量级 `|C P|` 的差值，看起来像真实分歧。本步初版正是这样写错的
（首次运行 `perturbation_gap = 0.63`），修正为**四条路线统一比较算子 `C`**，载荷只
提供扰动方向。

另有形状陷阱：算子的列轴是 `B_w` 的 modes，**不是**请求的载荷列数。从 `base` 尺寸
分配结果会静默返回 `(measurements, 1)` 块——形状合理、内容错误。已修正为按
`load_map.shape[1]` 分配，并在载荷列不足时按名拒绝。

### 5.4 第一轮独立复审发现的真实缺陷

| # | 缺陷 | 触发 | 后果 | 修法 |
|---|---|---|---|---|
| D1 | `_checked` 不校验 `D` 行数 | `D` 行数 < `G` 行数 | `D @ P` 把单行**广播**到全部测量，静默返回量级正确的错误算子，全程无异常 | 新增 `_measurement()` 同时校验 `G` 列对切线阶、`G` 行对 `D` 行 |
| D2 | 空测量集真空真 | `G` 为 `(0, n)` | `gap()` 对空数组返回 0.0 → `agrees=True`，检查得最少的用例最容易通过 | 空测量集按名拒绝 |
| D3 | 文档声称需要"张成 modes 的载荷" | `P` 秩亏 | 实际无秩条件（`bumped - P = h·e_mode⊗1ᵀ`，取第 0 列合法） | 改写注释，明确只要求列数 ≥ modes |

D1 与 D2 均是"**静默给出可信但错误的数字**"类缺陷——正是本层最不能容忍的失败模式。
两者都被初版测试全部漏过，因为夹具的行数恰好正确、测量集恰好非空。

第二轮（针对修复后交付物）另发现 A1/A1b/A3，见 §5.8。

### 5.5 蓄意破坏验证

`tests/characteristics/sabotage_step6.py`：19 个真实缺陷注入，**19/19 全部被捕获**，
模块每次从内存副本恢复并逐字节校验（`module restored: clean`）。

其中两个用例（"步长为绝对值"、"步长按模式行缩放"）是 **A1/A1b 的回归用例**——
把 §5.8 的缺陷重新注入，确保它不会无声回归。

排查过程中发现**三个语义等价变异**（不是漏检，且已如实记录在脚本注释里，
防止后人"补回"当作覆盖率）：

1. `bumped[mode, :] += step` → `bumped[mode, 0] += step`：实现只读 `quotient[:, 0]`，
   整行扰动对第 0 列的改动与仅扰动第 0 列**逐位相同**（四个探针含病态缩放，实测 bit-identical）。
2. 除数用 `applied` 而非 `realised`：步长缩放后可表示，二者差在最后一位或两位——
   横跨 `1e-8..1e12` 实测最差 3.8e-11。`realised` 是诚实记账，非承重修复。
3. 读 `quotient[:, -1]` 而非 `[:, 0]`：差商各列承载同一导数，仅舍入级不同。

**破坏验证的边界**：这 19 项**没能发现 A1**，因为破坏改的是结构与校验，而 A1 是数值
量级问题，需要量级离谱的输入才暴露（我的夹具最大只到 `1e3`，距失效起点 `1e4` 差一个
数量级）。**破坏验证不能替代独立复审**——本任务已连续四次由复审发现自设破坏漏掉的
真实缺陷。

把等价变异当作漏检去"修"，等于记录一个假阴性。两者的正确处置是**记为等价并说明理由**。

### 5.6 复审同时确认的非问题（不得"修"）

- 转置恒等式对任意可逆非对称 `A` 严格成立（200 次随机 5×5 验证，最大差 3.3e-8），
  实现不是靠夹具巧合。
- `h = 1e-7` 本身安全（gap 7.5e-10），但**安全区间有下界**：实测 `h ≲ 1.45e-11` 后
  gap 随 h 减小而增大（`h=1e-12` → 3.7e-05，`h=1e-13` → 4.0e-04）。原先"`h ∈ [1e-1, 1e-11]`
  内无任何 h 超阈值"的说法**偏乐观**，已按实测改为安全下界 ≈1.45e-11。
  现行实现不再依赖调用方选对 h（见 §5.8）。
- `gap()` 的 `max(1.0, ...)` 下限不会制造假失败；但原先把归因写成"C 在 A、B 同比例缩放下
  解析不变"不准确——数值上 `max|C|` 随缩放漂移约 24 个数量级。结论成立，归因已删。
- `P` 秩亏、含零列、非方阵**均仍正确**（实测确认无秩条件）。
- 病态切线下转置路线**不是有效 oracle**——但原论据（"`cond(A)~4e14` 时 gap 恒为 0"）
  **方向相反**：30/30 随机病态试验的 gap 全非零，最大 0.125，`cond≈3.95e14` 时
  gap=7.5e-04 并触发 `agrees=False`。结论（不宜作病态矩阵的 oracle）成立，论据已改。

### 5.8 A1：差商步长的量级缺陷（第四轮复审发现，已修）

| 项 | 内容 |
|---|---|
| 缺陷 | 差商用**绝对**步长 `h = 1e-7` |
| 触发 | `loads` 量级 ≳ 1e4（吸收门槛 ≈ `2^52·h` ≈ 4.5e8） |
| 实测后果 | `loads=1e8` 时相对误差 **1.08e-02**；增量为 `1.0431e-07` 而非 `1e-07` |
| 反向性 | **减小 h 使其更差**：`h=1e-10` → 1.9e-07，`h=1e-12` → 3.7e-05，`h=1e-13` → 4.0e-04 |
| 修法 | 步长按 `max(step, max|P|·1e-6)` 缩放，并按实际落地的增量做除数 |
| 修后实测 | 横跨 `1e-8..1e12`（20 个数量级）最差 **3.8e-11** |

**关键子缺陷（A1b）**：缩放基准必须是**整个载荷矩阵**而非被扰动的那一行。`Y` 由矩阵中
最大载荷主导，故在 `loads=[[1e8,0],[-1,3]]` 中按各行缩放会使第 0 列精确、**第 1 列误差
6.3e-04**。这一条是复审从我的诊断脚本里指出来的——我只测了第 0 列，据此误判 `1e-6`
相对步长已足够。

**为何重要**：A1 使该路线在载荷与其量级同阶时不再可信，而它的存在意义正是充当独立
oracle。它同时躲过了本步的 19 项自设破坏（那些破坏改结构，A1 是数值量级问题，
需要量级离谱的输入才暴露）——**破坏验证不能替代独立复审**。

### 5.9 台账核验结果（原 §5.7，行号经复审逐条复核）

复审逐条打开 C++ 源文件核对，**8/8 行号全部 MATCH**，两条结构声明为真：

- `mb_linearization` 的**传递头闭包（实测 8 个）中 0 命中** `solve_dynamic`/`integrator` 头；
  `mb_linear/factorization_types.hpp:36` 只有 `struct ResidualContext;` 前向声明，不传递拉入离散类型。
  （早期稿写"26 个"是未实测的数字，已按实测更正。）
- `WorkingSample` 全 `cpp/` 唯一定义（`types.hpp:37`），字段恰为 5 个，唯一聚合初始化
  （`trim.cpp:387-393`）亦 5 项，无 `alpha_f`/步长/前一步状态。
- R7 的"文件内无第二份组装算术"经全文 404 行逐段核对为真：方向导数只在
  `physical.cpp:133/137`；`trim.cpp:41/345` 的 `constraint_jacobian` 只进入残差力平衡与
  行掩码置零，均不流入切线；`:73/:136` 的 `J*lambda` 是约束反力与量纲缩放（产物进
  `force_residual`），非 `d(J^Tλ)/d(pose)`。
