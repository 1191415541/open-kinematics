# P2 步骤 6 验证证据

> 步骤：SPEC §52 —— 完成独立 oracle、R7/R9 及 R8 边界锁定。
> 证据落点：本文档 + `raw/retirement-evidence.md`。
> 记录规则：只记**实测**数字与退出码；未跑的命令写"未跑"及原因，不写推测。

## 1. 命令与退出码

| 命令 | 结果 | 退出码 |
|---|---|---|
| `uv run --no-sync pytest .../tests/characteristics/test_route_equivalence.py -q` | **33 passed** | 0 |
| `uv run --no-sync pytest .../tests/characteristics -q` | **306 passed** | 0 |
| `uv run --no-sync pytest .../tests`（快速集，排除 adams/architecture/cases） | **1863 passed, 1 xfailed** | 0 |
| `uv run --no-sync pytest .../suspension_kernel/tests .../suspension_contracts/tests -q` | **176 passed** | 0 |
| `uv run --no-sync pytest .../tests/architecture/test_no_legacy_vehicle_models.py -q` | **5 passed** | 0 |
| `uv run --no-sync ruff check .` | All checks passed | 0 |
| `uv run --no-sync ty check .` | All checks passed | 0 |
| `legacy_surface_gate.py --check` | `OK: no unregistered Python boundary violation` | 0 |
| `check_module_layering.py --strict --final` | `OK: layering matches the recorded baseline` | 0 |
| `check_composable_release.py --skip-isolation` | `OK: 3 release checks passed` | 0 |
| `dynamic_hash_sentinel.py --check` | `OK: dynamic output matches the frozen baseline byte-for-byte` | 0 |
| `case_parity_check.py` | `OK: 8 families accepted` | 0 |
| `kc_perf_gate.py` | `OK: benchmarks are within the recorded budget` | 0 |
| `git diff --check` | 干净（仅 CRLF 警告） | 0 |
| `sabotage_step6.py` | **caught 19/19**，`module restored: clean` | 0 |

### 1.1 测试数量变化

| 时点 | 快速集 | characteristics |
|---|---|---|
| 步骤 5 收口 | 1830 passed / 1 xfailed | 273 |
| 步骤 6 本步 | **1863 passed / 1 xfailed** | **306** |

增量 +33 = `test_route_equivalence.py` 的 33 个用例。**skip 仍为 0、xfail 仍为 1**——未增长。

### 1.2 性能预算实测

| 基准 | 实测 | 基线 | 比值 |
|---|---|---|---|
| k-100 | 0.987 s | 1.0230 s | ×0.965 |
| c-66 | 1.3743 s | 1.4904 s | ×0.922 |

两条均在预算内；**未重录任何基线**。

## 2. SPEC §57 等价性证据

四条独立路线求同一算子 `C = G A^{-1} B_w + D`，非对称夹具 `TANGENT` 实测：

| 路线对 | gap（相对） |
|---|---|
| transpose vs direct | **2.78e-17** |
| unit_basis vs direct | **3.47e-18** |
| finite_perturbation vs direct | **7.53e-10** |
| `agrees` 汇总 | **True** |

P0 容差：K 类 `0.1 + 0.002·|ref|`（mm）、`0.02 + 0.005·|ref|`（deg）；
C 类分量 index<3 → `1e-6 + 1e-4·|ref|`，否则 `1e-8 + 1e-4·|ref|`。
观察到的最大 gap 7.53e-10 比最紧的 C 容差下限（1e-8）低约一个数量级，**在 P0 容差内**。

### 2.1 跨载荷量级稳定性（A1 缺陷修复后的实测）

有限扰动路线的步长按**整个问题**的载荷量级缩放（`max(step, max|P|·1e-6)`），
故精度不随载荷单位变化：

| `loads[0,0]` | 1e-8 | 1.0 | 1e3 | 1e4 | 1e6 | 1e8 | 1e12 |
|---|---|---|---|---|---|---|---|
| gap（相对） | 6.5e-12 | 6.5e-12 | 1.9e-11 | 3.8e-11 | 9.1e-12 | 3.1e-11 | 1.2e-11 |

**最差 3.8e-11，横跨 20 个数量级平坦**。修复前同一条路线在 `loads = 1e8` 时相对误差
达 **1.08e-02**，且**减小步长会使其更差**（`h=1e-12` → 3.7e-05，`h=1e-13` → 4.0e-04）。

### 2.1 非对称性由断言保证，不由注释保证

`test_the_fixture_is_not_symmetric` 断言 `not np.array_equal(TANGENT, TANGENT.T)`；
`test_a_symmetric_tangent_makes_the_transpose_route_vacuous` 另行**记录**对称夹具下转置
路线退化为真空真这一事实。两者合起来使"为什么必须非对称"成为可执行知识。

## 3. 独立复审（D2）发现与修复

第三次派发只读复审；结论：**核心数学正确，台账 8/8 行号与两条结构声明全部核验为真**，
但发现 1 个真实缺陷 + 2 个文档/契约问题。全部已修并加回归测试。

| # | 缺陷 | 触发输入 | 修复前行为（实测） | 修复后 | 回归测试 |
|---|---|---|---|---|---|
| D1 | `_checked` 不校验 `D` 行数 | `D = [[10, -5]]`（1 行）配 `G = eye(3)` | 静默返回 `(3,2)`，第 0 列 `[10.25, 9.99, 10.03]`，正确值 `[0.25, -0.012, 0.025]` | 按名拒绝 | `test_a_direct_term_with_too_few_rows_is_refused_not_broadcast` |
| D1b | 同上，行数过多 | `D` 4 行配 `G` 3 行 | 裸 `ValueError: operands could not be broadcast` | 按名拒绝 | `test_a_direct_term_with_too_many_rows_is_refused` |
| D2 | 空测量集真空真 | `G = (0, 3)` | `agrees=True`，gap 全 0.0 | 按名拒绝 | `test_an_empty_measurement_set_is_refused` |
| D3 | 注释声称需要"张成 modes 的载荷" | `P` 秩亏 | 无实际错误，但文档误导 | 改写为"仅要求列数 ≥ modes" | `test_a_rank_deficient_load_matrix_is_accepted` |

SPEC §57 字面缺口同样由复审发现：初版 `ROUTES` 只有三条，**缺"独立全单位基投影"**，
而 §57 逐字要求它。已补 `unit_basis` 路线及其 4 个测试。

### 3.2 第二轮独立复审：A1（差商步长的量级缺陷）

第四轮（针对修复后交付物）复审发现了**第一轮及我的 18 项自设破坏都没抓到**的真实缺陷：

| # | 缺陷 | 触发 | 修复前实测 | 修法 |
|---|---|---|---|---|
| A1 | 差商用**绝对**步长 `h = 1e-7` | `loads` 量级 ≳ 1e4 | `loads=1e8` 时相对误差 **1.08e-02**；`h=1e-7` 加到 `1e8` 上被量化，实际增量为 `1.0431e-07` 而非 `1e-07` | 步长按 `max(step, max|P|·1e-6)` 缩放 |
| A1b | 步长按**本模式行**缩放 | 大小模式混在同一个问题里 | `loads=[[1e8,0],[-1,3]]`：第 0 列精确，第 1 列误差 **6.3e-04** | 缩放基准改为**整个载荷矩阵** |
| A3 | 载荷非有限值未校验 | `loads` 含 `nan` | 静默返回 NaN | `_columns` 按名拒绝 |

**A1 的严重性**：它使"独立 oracle"这一角色失效——路线在载荷与其量级同阶时可信度不是
文档声称的"截断误差、随 h→0 收敛"。更关键的是**减小步长会使其更差**，因此后续若有人
"调小 h 提高精度"反而会破坏它。修复后横跨 20 个数量级最差 3.8e-11。

**A1b 是复审中"我的探针错了"的典型案例**：我最初的诊断脚本只测第 0 列，据此认为
`1e-6` 相对步长已足够；复审指出并实测出第 1 列仍差 6.3e-04。根因是 `Y` 由矩阵中
**最大**载荷主导，而非被扰动的那一行——这一条已写进 `_applied_step` 的 docstring。

### 3.3 复审同时纠正的三处**过度声称**

| 出处 | 原声称 | 实测 |
|---|---|---|
| 台账 §5.6 | `h ∈ [1e-1, 1e-11]` 内无任何 h 超阈值 | **偏乐观**：真实安全下界 ≈1.45e-11；`h=1e-7` 本身安全 |
| 台账 §5.6 | `cond(A)~4e14` 时 `transpose_gap` 恒为 0 | **反了**：30/30 随机病态试验全非零；"对病态矩阵不是有效 oracle"的结论方向对，论据错 |
| `unit_basis` docstring | 该路线能捕获转置的 `A` | **过度声称**：它调用 `solve(matrix, ...)`，与 direct 同矩阵；它捕获的是**列**缺陷。已改写 |

### 3.4 四次复审的累计教训

D1 与 D2 都属于"**静默给出可信但错误的数字**"。19 个初版测试全部漏过，原因是夹具恰好
行数正确、测量集恰好非空——**测试的夹具选择本身决定了缺陷能否暴露**。前两轮复审分别
发现 8 项与 6 项，本轮 3 项 + A1 系列 3 项；"自查充分"这一判断已被证伪四次。

A1 尤其值得记录：它躲过了**我的 18 项自设破坏**，因为我的破坏都改动结构（换矩阵、删校验），
而 A1 是**数值量级**问题——需要"用一个量级离谱的输入去跑"才能发现，而我的夹具最大只到
`1e3`，距失效起点 `1e4` 差一个数量级。教训：**自设破坏的覆盖面受限于设计者自己的想象**，
只有独立复审能突破。这已连续四次成立。

## 4. 蓄意破坏验证（19/19）

脚本：`tests/characteristics/sabotage_step6.py`。每个用例注入一个真实缺陷、重跑
`test_route_equivalence.py`、要求失败；每次从内存副本逐字节恢复并校验。

覆盖的缺陷类别：转置被替换为原矩阵（×1）、单位基路线转置（×1）、丢测量映射（×1）、
差商步长缩放错误（×3，含 **A1 与 A1b 两个回归用例**）、丢 `D`（×3）、丢行数校验（×3）、
丢 zero-coupling 拒绝（×3）、丢非有限校验（×1）、丢载荷行数校验（×1）、差商忽略载荷耦合（×1）。

### 4.1 三个语义等价变异（如实记录，非漏检）

排查中三个候选"漏检"经实测判定为**等价变异**，理由写进脚本注释以防后人补回：

1. `bumped[mode, :] += step` → `bumped[mode, 0] += step`：实现只读 `quotient[:, 0]`，
   整行扰动对第 0 列的改动与仅扰动第 0 列**逐位相同**（四个探针含 `1e3/1e-3` 病态缩放，
   实测 bit-identical，bug-vs-correct gap = 0.000e+00）。
2. 除数用 `applied` 而非 `realised`：步长缩放后是可表示数，二者差在最后一位或两位——
   横跨 `1e-8..1e12` 实测最差 **3.8e-11**。`realised` 是诚实的防御性记账（若将来重新引入
   吸收它会起作用），不是承重修复。
3. 读 `quotient[:, -1]` 而非 `[:, 0]`：差商各列承载同一导数，仅舍入级差异。

把等价变异当作漏检去"修"，等于记录一个假阴性。

### 4.2 破坏验证的边界（必须登记）

18 项（后 19 项）自设破坏**没能发现 A1**。原因见 §3.4：破坏改的是结构与校验，
A1 是数值量级问题，需要量级离谱的输入才暴露。**破坏验证不能替代独立复审**——
本任务已连续四次由复审发现自设破坏漏掉的真实缺陷。

## 5. 未跑项及原因

| 项 | 原因 |
|---|---|
| `just test-all`（全量 33 min） | 按 AGENTS.md §3，全量在 P6 收口时跑；本步已跑快速集 + 数值门 + 架构门 |
| `tests/adams/` | 需 Adams 对标环境，且本步未触及轮胎力律 |
| `tests/cases/` | 本步未改 `cases/` 或 `preparation` |
| R8 实际迁移 | 设计 §555 行将其阶段定为 P3/P4a/P6；本步只交付精确片段清单 |

## 6. 缺口（诚实登记）

1. **响应缓存与 oracle 均尚无生产消费者**：`results/cache.py` 未接线，设计 §8.1 的
   "启用该特性不改变积分/事件/接受-拒绝次数"需 P4a/P4b 接线后按预算验证。
2. **R9 的"所有特性消费者使用统一响应/通道描述"未达成**：机制已就位并被 57 个测试锁定，
   但消费者覆盖要等接线。台账据此把机制记 `verified`、消费者覆盖记 `pending`。
3. **病态切线下转置路线不是有效 oracle**：`cond(A)` ~ 4e14 时 gap 恒为 0。已登记为路线
   的已知局限；P3 若引入病态切线需另加校验手段。
4. **R8 的两个 `body_effective_inertia_body` 表达式（:64/:116）** 是 D8"第二次出现即抽
   共享单元"的候选项，本步只登记不实施——它属于 R8 迁移范围。
