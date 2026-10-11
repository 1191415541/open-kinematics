# P3 步骤 6 退役台账：R8 迁移与旧位置委托

> 授权：SPEC §49（步骤 6）· SPEC §57（验收）· 设计 §555 行（R8 行）· 设计 §576 行（旧位置必须委托）
> · AGENTS.md §2（数值门触发条件）。承接 P2 步骤 6 的
> [R8 片段锁定](../../p2-equilibrium-response/raw/retirement-evidence.md)（该文件 §3 为本次实施的输入）。

## 1. R8 状态

| 项 | 状态 | 阶段 |
|---|---|---|
| R8 | **`pending` → `migrated`**（原处已委托，非"新模块能运行"） | P3 本步完成；P6 收口全部消费者 |

P2 步骤 6 有意保持 `pending`，理由记录为"设计 §555 行把阶段定为 P3/P4a/P6，本步职责是
精确锁定而非迁移"。本步执行该迁移。

**不记 `retired`**：与 R7 同一纪律（设计 §10.3 / §576）。`retired` 意味着消费者全部收口，
属 P6 范围；本步只做到"旧位置委托、不再自算"。

## 2. 迁移了什么：从 2 处扩展到 7 处（实测修正）

P2 锁定的 R8 片段（[P2 台账 §3.2](../../p2-equilibrium-response/raw/retirement-evidence.md)）
点名 `kernel_integrator_newton.cpp` 的 **64、116 两行**为"同一表达式出现两次"。

**实施时实测该表达式在全内核出现 7 次、跨 6 个翻译单元**，远超 P2 锁定的 2 处：

| # | 文件:行 | 上下文 |
|---|---|---|
| 1 | `force/layout.cpp:261` | 陀螺力矩 `cross(omega, Iw*omega)` |
| 2 | `joint/kernel_directional_constraint.cpp:416` | 方向约束质量逆 |
| 3 | `joint/kernel_directional_constraint.cpp:507` | `!directional_orientation_active` 分支 |
| 4 | `output/kernel_output.cpp:326` | 动能 `0.5*omega·(I*omega)` |
| 5 | `solve_dynamic/kernel_integrator.cpp:126` | 积分器惯量项 |
| 6 | `solve_dynamic/kernel_integrator_newton.cpp:63` | **R8 目标 1**（`evaluation` 态） |
| 7 | `solve_dynamic/kernel_integrator_newton.cpp:114` | **R8 目标 2**（`next` 态） |
| 8 | `solve_dynamic/kernel_integrator_residual.cpp:456` | 残差惯量项 |

表达式：`rotation * body_effective_inertia_body(model, bi) * transpose(rotation)`

> P2 锁定只覆盖 2 处，是因为当时的检索范围限于 R8 的设计行所指的两个函数体内。本步按 D8
> "第二处使用即抽共享单元"的完整口径做了全内核检索，才发现真实规模。**这不改变 P2 的结论**
> （其锁定无误），只是把 R8 的迁移面从 2 处修正为 7 处。

## 3. 抽出结果

**新增共享单元**（声明 `include/mb_model/functions.hpp:100`，定义
`src/model/kernel_model_accessors.cpp:101`）：

```cpp
// 声明
Mat3 body_world_inertia(const Model& model, int body, const Mat3& rotation);

// 定义
Mat3 body_world_inertia(const Model& model, int body, const Mat3& rotation) {
    return rotation * body_effective_inertia_body(model, body) * transpose(rotation);
}
```

**7 处调用点全部改为委托**（见 §2 表，行号已在改后重新实测）。

**计数核对：7 迁移 + 1 定义 = 8，全仓库无第 9 处。** 按形态
`body_effective_inertia_body(...) * transpose(...)` 全仓检索仅剩共享单元定义自身一处；
其余 23 处 `transpose(` 命中均为 `d_transpose`（DMat3 对偶数路径）或不同数学量
（`transpose(ra)`、`transpose(body_rotation)` 等）。

### 3.1 位置选择的依据

放在 `mb_model`（最低层模块）而非任何 solver 内，理由有二：

1. **语义**：世界系旋转是"模型 + 位姿"的性质，不是任何求解器离散化的性质。
2. **分层代价为零**：`mb_model → mb_numeric` 是**已登记**的 source edge
   （`layering_baseline.json`，`check_module_layering.py --strict --final` 复核 0 新增边、
   0 环）。若放进 `mb_linearization` 供 solver 共享，则需新增边并人工 review 基线——
   不必要。

## 4. 与 R8 约束的逐条对照

| 设计 §555 行要求 | 本步证据 |
|---|---|
| 抽出精确函数/代码段 | §2/§3 给出 7 处的文件+行号，非泛指目录 |
| 调用共享物理原语 | `body_world_inertia` 只组合 `body_effective_inertia_body`（已抽出访问器）与 `transpose` |
| **无离散系数泄漏到物理响应** | 见 §4.1 |
| 旧位置必须委托，不得长期双算 | §3 全部 7 处已委托；无双算 |

### 4.1 "无离散系数泄漏"的三重证据

1. **签名层面**：`body_world_inertia` 形参仅 `(model, body, rotation)`——不含步长、
   `alpha_m/alpha_f`、`beta`、`gamma`、前一步状态。物理响应在**结构上**无法读到离散量。
2. **文本层面**：`mb_model/functions.hpp` 与 `kernel_model_accessors.cpp` 内检索
   `alpha_f / alpha_m / beta / gamma / ctx.h / ResidualContext / step_size / previous.a`
   ——**0 命中**（已实测）。
3. **残留层面**：离散系数仍留在 solver 侧并乘在结果上，位置未变——
   `kernel_integrator_newton.cpp:67` 的 `-h²β`、`:72` 的 `(1-α_m)`；
   `kernel_model_accessors.cpp` 属 `model` 模块而非 solver 模块。

调用点已持有旋转矩阵，故**故意不提供四元数重载**——初版曾加，经复审指出**零调用者**
（9 处调用点全传 `Mat3`）后删除：未使用的重载是投机 API，与 D8"第二处真实使用才抽取"
相悖。此点已写入头文件注释，防止后人"补全"它。

## 5. 数值不变性证据

R8 迁移是**纯抽取**，因此第一要务是证明数值未动。

| 门禁 | 结果 |
|---|---|
| `dynamic_hash_sentinel.py --check` | **OK: dynamic output matches the frozen baseline byte-for-byte** |
| `case_parity_check.py` | **OK: 8 families accepted** |
| `kc_perf_gate.py` | **OK: within budget**（k-100 ×0.935、c-66 ×0.887） |
| `mb_linear_batch_selftest.exe` | **OK: mb_linear batch and transpose solves behave as declared**（exit 0） |

> **关于 sentinel 的非零 acceptance exit code**：该脚本自述（`dynamic_hash_sentinel.py:14-18`）
> "非零 acceptance exit code 是**预期**的：三个 road case 已知在 `fixture.force_z` 单一通道上
> 不通过 `time_convergence` 门（`docs/axle_dynamics_results.md`），本项目**如实记录**而非放宽
> 用例；故 sentinel 记录该退出码与失败清单，**只在漂移时失败**"。本步实测失败清单与基线
> 记录**一致**（9 个 case 名相同），未新增、未消失。sentinel 判定为 OK。
>
> 该报告同时显示 `adams_accuracy_status: BLOCKED`、
> `parameter_provenance: synthetic_labelled_not_for_formal_adams_accuracy`、
> `accuracy_claim_permitted: False`——即 Adams 参考数据缺失的**既有阻断**（EPIC §36），
> 与本步无关，且**不因本步缓解**。

## 6. 语义等价性的独立复核

D2 只读复审逐点核对了 7 个迁移点，结论 **7/7 语义等价**，其中特别核查了最危险的
静默互换风险：

- `kernel_integrator_newton.cpp:63` 仍取 `evaluation.q[bi]`（`evaluation` 态）；
- `kernel_integrator_newton.cpp:114` 仍取 `next.q[bi]`（`next` 态）。

两者是**不同的引用、不同的循环**，各自在自己作用域内取 `q[bi]`，未发生互换。
若互换，代码**照样编译**而物理错误——这是本次最需要盯的一类风险，已单独核验。

复审另确认：`Mat3` 与 `Quat` 均为无构造函数、无转换运算符的聚合体，两者互不可转换，
故**重载歧义不成立**（只可能编译失败，不可能静默选错）；`qmat` 形参为 `const Quat&`，
不存在"对已是旋转矩阵的值再取一次 `qmat`"的路径。

## 7. 残留缺口（不声称解决）

1. **`kernel_integrator_newton.cpp:114` 位于 `for(fi)` 内层 `for(row)` 循环体内**，
   每个 body 会被重算 `m` 次。这是**原代码既有形状**，迁移未引入、也未顺手修掉——
   属范围外，但记录以免被误认为本步的优化成果。
2. **性能无法由现有门禁排除小回归。** `dynamic_hash_sentinel` 逐位比对能证数值恒等，
   **证不了时间**；`kc_perf_gate` 只给两个比值且本次朝"变快"方向波动（×0.935/×0.887），
   说明测量噪声不可忽略。**若该门禁无方差界，任何 <10% 的回归都能藏进去**——这是门禁的
   固有局限，非本次改动缺陷。本步不声称"性能无回归"，只声称"在门禁分辨率内未检出"。
3. **R8 未记 `retired`**：消费者收口属 P6。
