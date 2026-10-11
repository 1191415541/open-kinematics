# P2 步骤 2：可复用多 RHS 与真实转置接口

对应 SPEC §48（步骤 2）、§28（产物）、§40（实现规则）。
本文件只记实测。

## 0. 结论摘要

| 项 | 结果 |
|---|---|
| 新增文件 | `include/mb_linear/batch.hpp` + `src/linear/kernel_linear_batch.cpp`（均在 `mb_linear` 内，**无新模块、无新边**） |
| 新能力 | `analyze_pattern` / `factorize_numeric` / `solve_many` / `solve_transpose_many` |
| 多 RHS | 真实共享**一次**分解；实测 `factorizations_used == 1`，逐列与单 RHS 解等值 |
| 转置 | **真正解 A^T**（显式构造转置并分解），实测 `factorizations_used == 2` 且 `last_transpose_rebuilt` |
| 后端能力 | 6 个后端**全部** `native_transpose=false`、`native_multi_rhs=false`，逐条给出 fallback 原因 |
| 测试 | 新增 `cpp/tests/linear_batch_selftest.cpp`（standalone executable），exit 0 |
| 反向验证 | **两种蓄意破坏都被测试逮住**（见 §4） |
| 门 | gate-numeric 三条全通过；快速集 1715 passed/1 xfailed；kernel+contracts 176 passed；ruff/ty/三架构门 0 |

## 1. 实测确定的现状：后端全是单 RHS、全无转置

写实现前先核实，结论与设计文档 §310 一致但更精确（含行号）：

| struct | 声明行 | `solve` 位置 | 多 RHS | 转置 | caller 可见符号分析 |
|---|---|---|---|---|---|
| `LuFactorization` | 38 | 252（返回 `void`） | 无 | **无** | 无 |
| `MklDenseFactorization` | 332 | 429 | 无（`nrhs` 写死 `1` @443） | **无**（`kNoTranspose='N'` @339 硬编码进 getrs @443） | 无 |
| `MklPardisoFactorization` | 455 | 665 | 无（成员 `nrhs{1}` @466 **从不被改写**） | **无** | **内部有**（`analyzed` @476、phase 12/22 @638/652、`same_pattern` @597），但**无公开 `analyze()`** |
| `SparseGmresFactorization` | 729 | 1015 | 无 | **无** | 无 |
| `SparseLuFactorization` | 1205 | 1318 | 无 | **无** | 无 |
| `BlockDiagonalMassFactorization` | 1372 | 1448（返回 `void`） | 无 | **无** | 无 |
| `ReducedNewtonFactorization` | 1483 | 2000 | 无 | **无** | 无 |
| `NewtonSystemFactorization` | 2234 | 2271 | 无 | **无** | 无 |

全 header 对 `solve_transpose|solve_transposed|solve_adjoint|'T'` 的检索**零匹配**。
唯一的转置相关记号是 `kNoTranspose = 'N'`，而它被**硬编码**用于请求**前向**系统——
它是"没有转置支持"的证据，不是"有"的证据。

**这一层的接口是鸭子类型，不是 C++ 抽象**：无基类、无 concept、无虚函数、无枚举；
`solve` 签名甚至不统一（`LuFactorization` @252 与 `BlockDiagonalMassFactorization`
@1448 返回 `void`，其余返回 `bool`）。因此"能力"只能由一张显式表说明一次。

## 2. 两个必须诚实处理的点（否则会写出会撒谎的接口）

### 2.1 `analyze_pattern` 不能声称 PARDISO 有调用方可见的模式步

PARDISO 确实复用符号分析，但那**内嵌在首次 `factor` 调用里**，调用方无法先规划。
若把它的 `caller_visible_pattern_step` 记为 `true`，调用方就会报告一个它做不到的
规划步。故**6 个后端全部记为 `false`**，`analyze_pattern` 是"有文档的 no-op"，
而不是一次从未发生的"缓存命中"。

字段名特意从 `separate_pattern_analysis` 改名为 `caller_visible_pattern_step`，
就是为了让这个区别写在类型里。

### 2.2 转置必须真解 A^T，不能假定对称（SPEC §40 明文禁止）

因为没有后端支持转置，"诚实的做法"本可以是**拒绝转置**。但那样这个接口就没有用。
采用的方案：`LinearFactorHandle` **保存原矩阵**，转置时**精确构造 A^T 并对其分解**。

- `solve_transpose_many` 返回 `factorizations_used = 2`（前向 1 + 转置 1），
  并在 handle 上置 `last_transpose_rebuilt`。
- **不对对称矩阵特判**：靠检测对称来省一次分解，会让快慢两条路径的数值不一致；
  调用方若想比较可以自己比。

`solve_many` 只计 **1**：`LuFactorization::solve` 是 `const`，一次分解确实服务了每一列。
计数是**出参**而非常量，因为"省了几次分解"正是这个批量接口要被计量的东西。

## 3. 新增文件不产生任何新的模块边

`batch.hpp` 只 include `mb_config/prelude.hpp`、`mb_numeric/vector.hpp`、
`mb_linear/factorization_types.hpp`——都在 `mb_linear` 自身及其下方。

实测 `check_module_layering.py --strict --final`：

```
header edges : 124      source edges : 116
mutual (reverse) edges     : 0
module cycles (SCC size>1) : 0
OK: layering matches the recorded baseline
```

124/116 **与步骤 1 结束时相同**，说明本步没有引入任何新边，因此**无需再改基线**。
（这是可以预期的：能力层属于既有 `mb_linear` 模块。）

## 4. 反向验证：测试确实能逮住它针对的缺陷

一个抓不到目标缺陷的测试等于没有测试。用两种蓄意破坏实测：

**破坏 1 —— 用 A 冒充 A^T**（正是 SPEC §40 点名禁止的错误）：

```
sabotage: transposed_handle.factorize_numeric(handle.matrix, n)   // 应为 transposed
→ SABOTAGED exit: 1
FAIL: each transposed batch column must satisfy A^T x = b
FAIL: each transposed batch column must satisfy A^T x = b
FAIL: each transposed batch column must satisfy A^T x = b
FAIL: on a non-symmetric matrix the forward and transposed solves must differ
4 check(s) failed
```

**破坏 2 —— 谎报分解复用**：

```
sabotage: factorizations_used = 1;   // 实际建了 2 次分解
→ SABOTAGED2 exit: 1
FAIL: a transpose solve must report its own factorization (2 total)
1 check(s) failed
```

两次破坏后均已从备份恢复，并复核：`SABOTAGE present: False`、`has real transpose
form: True`，恢复后 exit 0。

**测试矩阵是刻意非对称的**（`{4,1,2; 0.5,3,-1; 1.5,0.25,5}`）：对称矩阵上任何转置
缺陷都表现为"通过"，非对称才让"你到底转置了没有"变成可观测的。

## 5. 性能门的一次虚警（如实记录）

第一次跑 `kc_perf_gate.py` 得到：

```
FAIL
  k-100: 1.3000 s exceeds budget 1.2787 s (x1.271 of baseline, allowance x1.25)
```

**这不是本步造成的**。实测确认：新批量代码**不在任何求解路径上**——
`LinearFactorHandle`/`solve_many`/`solve_transpose_many` 的出现点只有
`batch.hpp`、`kernel_linear_batch.cpp` 与其 selftest，没有任何 solver 调用它们。

连续三次单跑 k-100 得 ×1.156 / ×1.098 / ×1.091，随后正式重跑得
**×1.139（k-100）/ ×1.167（c-66），OK**。即 ×1.271 是**运行间波动**，
不是回归。此处记录，避免下次有人把它当成真实退化去"修"。

（另：步骤 1 的提取本身带来约 ×1.1–1.2 的真实开销，见 `raw/module-dag.md` §3；
两个数字量级接近，混在一起时更要靠"是否可达"来区分。）

## 6. 本步命令与退出码

| 命令 | 退出码 | 说明 |
|---|---|---|
| `build_suspension_kernel.py` | 0 | 新 selftest 目标编译+链接通过 |
| `mb_linear_batch_selftest.exe` | 0 | `OK: mb_linear batch and transpose solves behave as declared` |
| （同上，破坏 1/2 后） | 1 / 1 | 证明测试有效 |
| `build_axle_native.py` | 0 | 同步 multibody 侧镜像 |
| `check_module_layering.py --strict --final` | 0 | 124/116 未变，0 环 |
| `dynamic_hash_sentinel.py --check` | 0 | 逐位一致 |
| `case_parity_check.py` | 0 | 8 族接受 |
| `kc_perf_gate.py` | 0 | ×1.139 / ×1.167 |
| `ruff check .` / `ty check .` | 0 / 0 | All checks passed |
| `legacy_surface_gate.py --check` | 0 | 0 findings |
| `check_composable_release.py --skip-isolation` | 0 | 3 release checks passed |
| 快速集 | 0 | **1715 passed, 1 xfailed** |
| kernel+contracts | 0 | **176 passed** |
| `git diff --check` | 0 | 干净 |

## 7. 缺口与未做

- **批量接口尚无生产调用方**。本步交付的是能力与能力声明；真正的消费者是
  步骤 3（平衡模式响应）、步骤 5（共享缓存与计量）与 P4a。此处不假装它已被使用。
- `matrix_rank` 仍只存在于 `solve_linear` 旁；本步未改它（属步骤 6 的边界锁定范围）。
- `BlockDiagonalMassFactorization` 被列入能力表以保持完整，但它的 `solve` 返回
  `void`，不满足 `solve_many` 所依赖的"可判成败"约定；未来若要把它接进批量接口，
  需要先给它一个可失败信号。此处如实登记，不在本步改它的签名。
- 未验证的非对称来源：本步只用了一个构造的非对称矩阵。真实非保守切线
  （含速度项的衬套、摩擦）的转置显著性待步骤 3/4 用真实切线复核。
