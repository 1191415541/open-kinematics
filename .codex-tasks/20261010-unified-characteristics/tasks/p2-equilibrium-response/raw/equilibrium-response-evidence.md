# P2 步骤 3：平衡模式响应与可辨识审计

对应 SPEC §48（步骤 3）、§40（实现规则）、设计 §6.2 与 §6.6。
本文件只记实测。

## 0. 结论摘要

| 项 | 结果 |
|---|---|
| 新增实现 | `src/suspension_multibody/results/equilibrium.py` |
| 新增测试 | `tests/characteristics/test_equilibrium_response.py`（**16 passed**） |
| 公式 | `A X = B_w P`；`Y = G X + D P`（直接项不省略） |
| 批处理 | 多模式共享**一次**分解（实测 `audit["factorizations"] == 1`） |
| 拒绝路径 | 三处，各自具名、**都不返回数字** |
| 反向验证 | 自测破坏 3 处全部被逮住；**独立复审又发现 8 个真实缺陷**（见 §7） |
| 门 | 快速集 **1731 passed / 1 xfailed**（+16，xfail 未增长）；kernel+contracts 176；ruff/ty/三架构门 0；gate-numeric 三条 0 |

## 1. 复用 P1 既有审计，不另写一套（D6）

`characteristics/motion.py` 已有 `rank_and_null_space` / `audit_boundary` /
`is_identifiable` / `solve_motion` / `sample_status_for`。本步**直接复用**
`audit_boundary` 与 `rank_and_null_space`，没有新写秩判定或零行判定：

- `audit_boundary` 提供 `singular`（某行恒零，约束不了任何东西）与
  `unidentifiable`（行本身没问题但留下自由方向）的区分及其理由文本。
- `rank_and_null_space` 提供秩与零空间基。

**复用它们的理由不只是省代码**：秩容差与"零行"的判据必须全仓库只有一个，
否则同一个模型在不同路径上会被判成不同状态。P1 的 `RANK_TOLERANCE = 1e-10`
因此是本模块的默认容差。

## 2. 公式与两处不能省的东西

```
A = dR/dx      B_w = -dR/dw      G = dy/dx      D = dy/dw
A X = B_w P                    (机理怎么动)
Y = G X + D P                  (测量读到什么)
```

**`D P` 不能省。** 当测量直接读载荷本身（轮荷就是），`D` 非零而仅 `G X` 会
恰好差掉主要项。测试 `test_the_measurement_chain_adds_the_direct_term` 用一个
`G = 0`、`D = 3` 的构造让"漏掉直接项"变成必错：正确答案是 3，漏掉后是 0。

**`B_w` 的符号不在本模块再翻一次。** `B_w = -dR/dw` 的负号属于均衡约定，
由调用方带着；本模块若再取负，两次翻转互相抵消且不可见。

## 3. 三处拒绝路径，以及它们为什么必须不同

| 情形 | 状态 | 判据 | 为什么不能合并 |
|---|---|---|---|
| 某行恒零，约束不了任何东西 | `singular` | `audit["empty_rows"] > 0` | 模型/状态本身坏，修法与下者不同 |
| 行都好但留下自由方向 | `unidentifiable` | 秩亏或零空间非空 | 机理运动是一族，任何解都是任取一个 |
| 切向完美，但某模式`B_w P[:,j]` 数值为零 | `unidentifiable`（**仅该模式**） | 该列范数 ≤ 容差 | 全局审计看不见——切向没问题 |

第三处是本步最重要的实测发现，见下节。

## 4. 一次真实的自纠：我原来写的分支是死代码

初版 `audited_modes` 里有一个"逐模式测量梯度可辨识性"检查：把 `G` 的每一行
投影到 `A` 的零空间，非零就拒绝该模式。**蓄意禁用它（把条件改成 `if False`）
后 12 个测试全过**——即它没被任何东西覆盖。

追查原因，结论是**该分支在方阵情形下不可达**：若 `A` 是方阵且非奇异，则 `X` 唯一，
于是 `X` 的任何线性函数（包括 `G X`）都唯一可辨识。所以"测量侧逐模式检查"只在
`A` 已被拒绝时才会触发，而那时前面的全局短路已经返回了。**它是死代码。**

真正需要逐模式判定的是**载荷侧**：某模式的施加方向 `B_w P[:, j]` 若数值为零，
该模式什么都没问机理，而 `X = A^{-1} 0 = 0` 会给出一个**格式完好的零**——
这正是本模块存在的意义所要防的失败，且切向本身完全健康、全局审计看不见。

据此重写：删除死分支，改为检查该模式的施加列范数，并把测试
`test_a_mode_that_drives_nothing_is_refused_while_its_neighbour_answers` 指向
这条真实路径（切向取单位阵，即绝对良态，因此除逐模式检查外没有任何东西能产生拒绝）。
重写后同样禁用该检查 → **测试失败**（`assert 'valid' == 'unidentifiable'`）。

这一段保留在证据里，是为了防止有人把那条"看起来更严谨"的测量侧检查再加回来。

## 5. 拒绝必须不带数字（本步修掉的第二个自身缺陷）

初版拒绝路径返回 `np.zeros(...)`。这与我在同文件里写下的原则直接冲突：
"a refused mode that returned zeros would be indistinguishable from a mode whose
response really is zero"。**零填充的拒绝与真实的零响应在数值上完全一样**，
调用方一旦忘了读 `status` 就会发布一个貌似合理的错数。

改为返回 `np.full(..., np.nan)`。NaN 不可能被误认为读数，且本层的下游消费者
（`measurement_from_channel` / `statuses_for`）本来就按"非有限 = 不可用"处理。

这个缺陷之所以能被发现，是因为我按 §4 的方法做了蓄意破坏：把
`_solve_block` 的秩检查换成 `np.linalg.lstsq`（即"用最小范数解回答一族"），
**当时 12 个测试全过**——因为拒绝测试只断言了 `values is None`（`ModeResult` 层），
没断言返回数组本身。补上 `np.isnan(response).all()` 后同一破坏才被逮住。

## 6. 反向验证汇总（三种破坏，全部被逮住）

| # | 蓄意破坏 | 结果 |
|---|---|---|
| A | `_solve_block` 用 `lstsq` 最小范数解回答秩亏系统 | **FAIL**：`test_a_rank_deficient_tangent_returns_no_numbers`（返回了 `[[0.04,0.08],[0.08,0.16]]` 而非 NaN） |
| B | 丢弃 `D P` 直接项 | **FAIL**：`test_the_measurement_chain_adds_the_direct_term`（得 0，应为 3） |
| C | 禁用逐模式载荷方向检查 | **FAIL**：`assert 'valid' == 'unidentifiable'` |

（另有破坏 C 的**初版**：禁用测量侧检查 → 全过，正是 §4 的发现。）
三次破坏后均已从备份恢复并复核 `SABOTAGE present: False`。

## 7. 策略等价：转置是真转置

`test_the_transpose_is_a_real_transpose` 用一个**刻意非对称**的切向
`[[4,1],[0.5,3]]` 断言：前向解与转置解**必须不同**，且转置解满足 `A^T x = b`。
对称矩阵上任何转置缺陷都表现为通过，非对称才让"你到底转置了没有"可观测——
这与步骤 2 在 C++ 侧用同一手法（`linear_batch_selftest.cpp`）保持一致。

设计 §428 明文要求不得假定 `A` 对称（含速度项的衬套/摩擦不对称），本步遵守。

## 7. 交付后独立只读复审（D2）：发现并修复 8 个真实缺陷

本步交付后按 D2 派独立只读复审（覆盖步骤 2 的 C++ 与步骤 3 的 Python）。
**复审推翻了本文件初版的多条论断**，以下是逐条实测确认与修复。

### Python 侧（`equilibrium.py`）

| # | 缺陷 | 实测确认 | 修复 |
|---|---|---|---|
| D1 | **接受路径可产出 inf 并崩溃**：`audited_modes` 无有限性检查，溢出值到达 `statuses_for` 时**不带理由**而抛 `CharacteristicsError`，穿透给调用方 | 复现：`t=eye(2)`、`load[0,0]=2`、`g=[[1e308,1e308]]` → `equilibrium_response` 报 `valid` 但 `inf`，`audited_modes` 抛异常 | 在 `solution` 与 `response` 两处加有限性检查，转为带理由的 `unidentifiable` 拒绝 |
| D2 | **逐模式阈值是绝对量，误拒合法模式**：`scale = max(‖B_w‖_F, 1.0)` 使阈值在 ‖B_w‖<1 时退化为绝对 `1e-10` | 复现：`load = 1e-11 * I`（完全合法）→ **两个模式全被拒**，而 `audit` 仍报 `valid` | 改为与**本次请求中最大的施加列**比较（相对判据）；全零列时参考值 0，`0 <= 0` 仍正确拒绝 |
| D3 | **逐模式拒绝在 `audit` 中无痕迹**：只读 `audit` 的调用方会以为整体健康 | 复现：模式被拒时 `audit['status']=='valid'`、`reason==''` | 增加 `audit["refused_modes"]` / `audit["answered_modes"]`；`status` 仍只表达切向裁决（两个不同问题） |
| D4 | **形状只校验第 0 维**：2-D 输入通过校验，下游被当成 `(samples, n)` 向量消费；标量触发 `IndexError` 而非契约错误 | 确认 | 改为要求 `ndim == 1` 且长度等于模式数，标量也给 `CharacteristicsError` |
| D5 | **不校验 `mode_matrix` 列数**：列数与声明模式数不符时静默返回错列数的结果 | 确认 | 新增 `mode_count` 参数并校验列数 |

D2 的量纲批评尤其到点：拿**整块** `applied` 的 Frobenius 范数去缩放**单列**的阈值，
两者不是同一个量。已在代码注释中写明为何参考值取"本次请求的最大施加列"。

### C++ 侧（`kernel_linear_batch.cpp`）

| # | 缺陷 | 处理 |
|---|---|---|
| C1 | **`native_transpose == true` 分支是错的**：`transposed_handle = handle` 复制的是**前向**分解，会把 `A^T X = B` 当 `A X = B` 解并报 `factorizations_used=1`。今天无后端如此 ⇒ 既错又不可测 | **删除该分支**：无条件构造并分解 `A^T`。只有一条代码路径可推理；未来后端若真获得转置支持，selftest 的全表断言会先失败，强制重新决策。实测：把旧分支放回去，selftest **仍然通过**（证实不可达），故删除是对的 |
| C2 | `factorize_numeric` 无条件把 `backend` 写成 `"lu"`，使具名 handle 的能力查询失真 | 不再重置 `backend`（默认构造已是 `"lu"`，即本函数实际构建者），并在注释中说明该字段记录的是"调用方认为自己在用哪个后端" |

**复审还确认了一处非问题**（避免误"修"）：`solve_many` 的列主序索引在
两个函数中一致，`handle.last_transpose_rebuilt` 的写入经 `mutable` 正确落在调用方
对象上。我为此**新增了一条断言**（`last_transpose_rebuilt` 必须在调用方 handle 上可见，
且随后前向批量求解仍能复用原分解），并实测该断言有牙：蓄意不记录重建 → **2 条 FAIL**。

### 复审后的完整复跑

修复后全部门禁重新通过：

| 命令 | 结果 |
|---|---|
| `test_equilibrium_response.py` | **16 passed**（原 12 + 5 条复审回归，其中 D1/D2/D3/D5 各一条） |
| `pytest .../characteristics` | **174 passed** |
| 快速集 | **1731 passed / 1 xfailed**（xfailed 未增长） |
| kernel+contracts | **176 passed** |
| `mb_linear_batch_selftest.exe` | exit 0（新增 2 条断言） |
| ruff / ty | 0 / 0 |
| 三架构门 | 0 / 0 / 0 |
| gate-numeric 三条 | 0 / 0 / 0（perf ×1.108 c-66） |
| `git diff --check` | 0 |

**结论**：本步交付的初版有 8 个真实缺陷（其中 D1、D2、C1 会产出错误结果或崩溃），
全部由独立复审发现、逐条实测确认并修复，且每条都补了会失败的回归测试。
这也说明"自己写的蓄意破坏"不足以覆盖全部盲区——D1/D2/C1 都不是我原先设想的三处破坏
所能触及的，独立复审是必要的第二道。

## 8. 本步命令与退出码

| 命令 | 退出码 | 说明 |
|---|---|---|
| `pytest .../characteristics/test_equilibrium_response.py` | 0 | **16 passed** |
| `pytest .../characteristics` | 0 | **174 passed** |
| 快速集 | 0 | **1731 passed, 1 xfailed**（+16 = 本步新增；xfail 未增长） |
| kernel+contracts | 0 | **176 passed** |
| `mb_linear_batch_selftest.exe` | 0 | 含 2 条复审新增断言 |
| `ruff check .` / `ty check .` | 0 / 0 | All checks passed |
| 三架构门 | 0 / 0 / 0 | 见 §0 |
| `dynamic_hash_sentinel --check` | 0 | 逐位一致 |
| `case_parity_check` | 0 | 8 族接受 |
| `kc_perf_gate` | 0 | ×1.108（c-66），在预算内 |
| `git diff --check` | 0 | 干净 |

一次真实的自我纠正：初版测试
`test_the_response_is_the_solution_and_not_the_product` 期望 `-8/4 = +2`，
实测 `-2`。**是测试写错、代码是对的**，已改正并补上"既不能是乘积、也不能丢符号"
两条显式断言。

## 9. 缺口与未做

- **尚无生产调用方接真实切向**。本步用构造切向验证公式与审计；真实
  `constraint_jacobian` / 力律导数驱动的端到端响应属步骤 4/6。
- `measurement_load_gradient`（即 `D`）的**自动装配**未做：本步接受调用方传入
  `D` 并正确使用它，但没有从模型声明里推导 `D` 的通道。这是步骤 4/6 的范围。
- 逐采样（多 sample）响应未做：本步按"一个平衡态、多个载荷模式"组织。
  多采样需要针对每个样本分别审计与求解，属步骤 5（缓存与计量）与 P4a。
- 未验证：真实非保守切线的非对称显著性与转置收益，待步骤 6 的独立 oracle。
