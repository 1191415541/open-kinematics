# P4a 步骤 2 证据：坐标驱动的冻结态反力切线

> 验收条件（TODO.csv 行 2）：**功共轭反力与预载/参考体/端口导数完整，K=k 解析通过**。
> 本文件记录实际行为断言、命令退出码、误差/预算与缺口。命令通过/文件存在只作门禁，
> 不作为 DONE 依据。

## 1. 结论

步骤 2 **完成并通过行为验收**。交付物是"功共轭端口反力切线"——
给定冻结态的工作样本与装配好的物理切线 `A = dR/dx`，在**指定坐标**上施加单位位移，
解出自由坐标的响应，返回**为保持该位移所需的外部力** `Q_s`。

**本步只交付"坐标驱动的力切线"**。`frozen_load_compliance`（步骤 3，载荷驱动）尚未实现，
`B_w = -dR/dw` 仍为零实现。故 SPEC §57 的 `C = 1/k` 属步骤 3，本步证据**不覆盖**它。

`K = k` 的解析验收（SPEC §56）由本步交付，见 §3.1/§3.2。

## 2. 交付物

| 文件 | 性质 | 说明 |
|---|---|---|
| `cpp/include/mb_solve_static/functions.hpp` | 改 | 新增 `frozen_force_tangent_column` 声明 + 契约文档 |
| `cpp/src/solve_static/kernel_static_trim.cpp` | 改 | 新增 `frozen_force_tangent_column` 定义（文件末尾） |
| `cpp/tests/frozen_tangent_selftest.cpp` | **新增** | 独立自测可执行文件，8 组闭式断言 |
| `CMakeLists.txt` | 改 | 注册 `mb_frozen_tangent_selftest` |
| `layering_baseline.json` | 改 | 登记 `mb_solve_static -> mb_linearization` 头文件边 |
| `tests/test_registry_consistency.py` | 改 | 注册该自测，使断言真正被门禁执行 |

### 2.1 为什么放在 `mb_solve_static` 而不是 `mb_linearization`（分层决定）

第一版把声明放在 `mb_linearization/types.hpp`，**分层门立刻失败**：

```
[new_header_edge] mb_solve_static -> mb_linearization is not in the baseline header_edges
```

根因不是"忘了登记"，而是位置错了：端口求解需要 `mb_linear`（`LuFactorization`），
而 `mb_linearization -> mb_linear` **不是已登记边**，且 `mb_linear` 不在它之下。
`mb_linearization` 的契约明写它**永不向上调用**（`types.hpp:9`），
"哪个坐标被按住"是**求解器策略**而非物理装配——按"physics here, policy there"
的既定划分，它属于 `mb_solve_static`（该模块 `-> mb_linear` 与 `-> mb_linearization`
两条边**都已登记**）。

### 2.2 头文件边的登记（不静默基线）

移到 `mb_solve_static` 后仍新增了一条**头文件**边：`mb_solve_static/functions.hpp`
现在必须 include `mb_linearization/types.hpp` 才能命名 `WorkingSample`。

处理方式：**登记头文件边**，而不是改基线掩盖。实测 `reviewed_new_edges` 里
**已存在**同一 `edge` 的条目（R7 委托时留下），故本次只补 `header_edges`，
未新增/修改任何 ledger 条目：

```
old header_edges: 124   new header_edges: 125
added:   [['mb_solve_static', 'mb_linearization']]
removed: []
source_edges:       same=True
reviewed_new_edges: same=True
```

变更**仅 67 字节**，`source_edges` / `reviewed_new_edges` / `cycles` / `mutual_edges`
逐字节不变。方向未变（source 边早已存在），只是从"仅源码"变为"源码+头文件"。

## 3. 行为断言（8 组，全部闭式解）

自测文件 `cpp/tests/frozen_tangent_selftest.cpp`，退出码 0 打印
`OK: frozen force tangent self-test passed`。

### 3.1 K = k（SPEC §56 的解析验收）

| 用例 | 切线 `A` | 输入 | 期望 | 物理含义 |
|---|---|---|---|---|
| 单弹簧 | `[[500]]` | `ds=0.01` | `Q_s = +5.0` | **保持力**，非恢复力（符号约定见 §5） |
| 串联双弹簧 | `[[200,-200],[-200,500]]` | 按住坐标 0 | `Q_s = 120.0` | `k1*k2/(k1+k2) = 200*300/500` |
| 耦合端口 | `[[400,100],[100,900]]` | `ds={0.02,-0.03}` | `{5.0, -25.0}` | 含非对角项，非仅对角 |

**串联用例是"非平凡"的关键**：若实现错误地只取对角线，会返回 `200`；若忽略自由坐标
的响应，会返回 `200`。返回 `120` 才能证明**分区求解真的发生了**。
耦合用例同理：`Q_0 = 400*0.02 + 100*(-0.03) = 5.0` 而非 `400*0.02 = 8.0`。

### 3.2 拒绝语义（7 类，全部在写入前完成）

| 用例 | 断言 |
|---|---|
| 端口越界（`2`、`-1`） | 拒绝，且不写 `reaction` |
| 端口重复 | 拒绝 |
| `driven`/`prescribed` 长度不符（两个方向） | 拒绝 |
| 切线尺寸错误（过短 `{1.0,2.0}`、过长 3×3 对 2×2） | 拒绝，且不写 `reaction` |
| `prescribed` 非有限（NaN） | 拒绝 |
| 自由块奇异（`[[500,0],[0,0]]`；比例行 `[[4,6],[6,9]]`） | 拒绝，**不做最小二乘近似** |
| 乘子坐标留在自由块（`ndof=1, rows=1`） | 拒绝 |

### 3.3 零刚度是合法答案，不是失败信号

`A = [[0]]`、`ds = 0.01` ⇒ 返回 `true` 且 `Q_s = 0.0`。
文档明确写明"zero is a legitimate answer and must not double as the failure signal"。
反向确认：把一个非零反力改成拒绝的变异**被抓住**（见 §5）。

### 3.4 拒绝不写入

种子 `reaction = {7.0, 8.0, 9.0}`，每个拒绝用例后断言**逐字节未变**。
实现上所有拒绝测试都在 `reaction = std::move(result)` **之前**。

## 4. D2 review 结论与处置

对交付代码做了只读对抗性复审。**发现 1 个真实且严重的问题**，已修复：

### 4.1 严重：新自测没有任何调用者（覆盖率为零）

复审指出 `KERNEL_SELF_TESTS`（`accept_composable_architecture.py:117-121`）只登记了
`mb_contract_selftest` / `mb_cases_selftest` / `mb_tire_state_selftest` 三个。

**已独立复核**：全仓检索四个自测名，结果**只有 `CMakeLists.txt`**——
`mb_frozen_sample_selftest`、`mb_frozen_tangent_selftest`、`mb_linear_batch_selftest`
**全部无调用者**。这意味着即使自测**编译失败或断言失败**，日常门也毫无反应：
那 8 组断言买到的质量是 **0**。

**处置**：在 `tests/test_registry_consistency.py` 注册本步自测
（`_kernel_selftest` 模式，`check-fast` 路径即执行）。实测
kernel+contracts 由 **176 → 177 passed**，即断言真正跑起来了。

### 4.2 已知限制：`mb_frozen_sample_selftest`（步骤 1）不能在此注册

本步注册时**实测**：`mb_frozen_sample_selftest` 直接运行退出 0、
在**新建的 pytest 文件**（同目录）里也退出 0，但在
`test_registry_consistency.py` 进程内启动时返回 **0xC0000005**
（访问冲突，且**仍打印成功行**）。

这不是"环境噪声"，已排除以下假设：

| 假设 | 实测 | 结论 |
|---|---|---|
| 导入 numpy / 内核包 / `ctypes.CDLL` 导致 | 逐项加入最小复现文件，**全部退出 0** | 排除 |
| 前序三个自测"污染"进程 | 四个自测在同一 pytest 进程内顺序跑，**全部退出 0** | 排除 |
| `rglob` 命中多个陈旧 exe | 每个名字**仅 1 个**候选 | 排除 |
| 与 `ctypes` 测试（`:214`）的加载冲突 | 本文件 13 项全跑（含 ctypes 测试）**13 passed** | 排除 |
| 二进制陈旧 | 重新构建后行为不变 | 排除 |

**关键对照**：`mb_frozen_tangent_selftest` 处于**完全相同的位置**
（同一文件、`ctypes` 测试之后），连续 5 次全文件运行 **5/5 13 passed**。
故故障是**该可执行文件特有**，不是本模块进程的问题。

**处置**：**不注册**——把原因未明的崩溃放进日常门会制造假失败，
违背"不静默抽样/不掩盖"的原则。步骤 1 的证据仍以"直接运行退出 0"为准（已记录），
本限制**如实登记**而非粉饰。

### 4.3 采纳的其他复审点

- **§2.1 的分层纠正**：复审独立确认 `mb_linearization` 的"never calls back up"契约未被破坏，
  与我的实测一致。
- **Dirichlet 消元的符号与转置结构**：复审手工以 2 坐标串联弹簧逐位复算
  （`rhs = -(A[1,0]*1.0) = 200` ⇒ `dq_free = 0.4` ⇒ `Q = 200 - 200*0.4 = 120`），
  与实现及 `static_residual` 的 `A = dR/dx` 约定一致，**判定正确**。
- **`held`/`free` 不越界**：复审证明 `model.ndof <= dimension` 恒成立
  （`dimension = ndof + rows + active_tires.size()`，二者非负），故
  `driven` 校验 `< model.ndof` 足以保证 `held[coordinate]` 安全。**采纳**。

### 4.4 已登记但本步不修的复审点

复审指出：`factor` 成功而 `solve` 结果错误时，防线**只剩** `isfinite`
（`:489`）。本步的实现确实两者都做（`:484` 传 `false` + `:489` 逐分量 `isfinite`），
故当前**无缺陷**；但复审正确指出该冗余**没有注释说明**，属可改进项。
本步已把 §3.2/§3.3 的语义写进契约文档（零反力合法、奇异必须拒绝而非近似），
**未**改 `LuFactorization`（不属本步权限，且会动到 `mb_linear` 公共语义）。

## 5. 蓄意破坏验证（mutation testing）

脚本对 `kernel_static_trim.cpp` 做**字节级**变异 → 重建 → 运行，
要求自测**失败**。字节级读写 + `NEWLINE` 归一（本 checkout 为 CRLF，
文本模式会静默重写全部行尾）。每条变异先验模式唯一性（`count != 1` ⇒ SKIP/陈旧），
恢复后用 SHA256 复核，并**重跑基线确认绿**。

**结果：10/10 抓住，0 个未解释存活。**

| 变异 | 结果 |
|---|---|
| 反力丢掉保持力符号 | CAUGHT |
| 反力只取对角而非 `A*delta` | CAUGHT |
| held 坐标**未**从自由块移除 | CAUGHT（崩溃） |
| 指定增量**未**移到右端 | CAUGHT |
| 自由解被丢弃（`delta` 停在指定值） | CAUGHT |
| 重复端口被接受 | CAUGHT |
| 越界端口被接受 | CAUGHT（编译失败） |
| 长度不符被接受 | CAUGHT |
| 切线尺寸检查被放宽 | CAUGHT |
| 零反力被当成拒绝 | CAUGHT（崩溃） |

### 5.1 一个**语义等价**变异（如实登记，不计入"抓住"）

**"忽略 `factor()` 的失败"** —— 变异后自测**仍然通过**。

我没有把它当作"测试缺口"去凑夹具，而是**直接测出机制**：用探针打印该函数的真实返回值，
发现变异后**仍返回 `false`**。原因是 `LuFactorization::solve` 读取 `factor()`
留下的因子——`factor()` 失败时那些因子就是**输入矩阵**，于是 `solve()` 在未消元的数据上继续
求解并给出**非有限**值，被 `:489` 的 `isfinite` 拦下。

**结论**：两个防线在**所有可达输入**上拒绝同一批输入——不是设计上的冗余，
而是后端行为的巧合。故该变异**不可观测**，我**不声称**能抓住它。

处置：把它单列为 `EQUIVALENT` 集合，**每轮重新实测**；
若将来 `LuFactorization` 行为变化使等价失效，会显示为 "NO LONGER EQUIVALENT" 而非静默通过。
`factor()` 的检查仍然保留（它才是**有意**的拒绝，且不依赖后端巧合）。

## 6. 命令与退出码

| 命令 | 退出码 | 结论 |
|---|---|---|
| `cmake --build . --target mb_frozen_tangent_selftest` | 0 | 编译通过 |
| `mb_frozen_tangent_selftest.exe` | 0 | `OK: frozen force tangent self-test passed` |
| `mb_frozen_sample_selftest.exe` | 0 | 步骤 1 回归未破坏 |
| `ruff check .` | 0 | `All checks passed!` |
| `ty check .` | 0 | `All checks passed!` |
| `check_module_layering.py --strict --final` | 0 | `OK: layering matches the recorded baseline` |
| `legacy_surface_gate.py --check` | 0 | `OK: no unregistered Python boundary violation` |
| `check_composable_release.py --skip-isolation` | 0 | `OK: 3 release checks passed` |
| 快速测试集（除 adams/architecture/cases） | 0 | **1958 passed, 1 xfailed**（基线不变） |
| kernel+contracts | 0 | **177 passed**（基线 176 + 新增 1） |
| `dynamic_hash_sentinel.py --check` | 0 | 逐位不变 |
| `case_parity_check.py` | 0 | `OK: 8 families accepted` |
| `kc_perf_gate.py` | 0 | 性能预算内 |
| `git diff --check` | 0 | 干净 |

## 7. 一致性检查

- **无基线重录**：`dynamic_hash_sentinel` 逐位不变、`kc_perf_gate` 在预算内，
  未触碰任何冻结基线。
- **单一物理路径未被绕过**：本步只新增一个**内部** C++ 函数，
  **未**新增 Study/Case 族、未新增入口点、未改 `ResolvedSolvePlan`。
- **能力声明未被虚假填充**：`characteristics_response_kinds` 仍为**空数组**——
  本步**不导出 ABI**，故不声明任何 kind（读与答是两种能力，见 P0）。
- **测试基线未增长**：skip/xfail 未增长（仍 1 xfailed）。
- **`source_edges` 逐字节未变**：仅新增 1 条头文件边。

## 8. 登记缺口（如实）

1. **`B_w = -dR/w` 仍零实现**——步骤 3 的交付物，`frozen_load_compliance` 不可用。
2. **物理切线未跨 ABI 导出**——本步函数是**内部**能力；导出矩阵 + 声明
   `characteristics_response_kinds` 必须**同时**做（否则就是契约禁止的空壳能力）。
3. **动态样本 → 装配的适配器未实现**——本步函数接受已装配的 `tangent`，
   从 `AxleDynamicsResult` 样本到 `assemble_physical_tangent` 的接线尚缺。
4. **`mb_frozen_sample_selftest` 在 pytest 进程内崩溃原因未明**（§4.2）——
   已排除 5 类假设并记录对照实验，但未定位根因；故未注册。
5. **`factor`/`isfinite` 双防线冗余无注释**（§4.4）——当前无缺陷，属可改进项。
6. **`mb_linear_batch_selftest` 同样无调用者**——既有缺口，不属本步权限。
7. **`frozen_protocol`/`frozen_version` 缓存键已预留但未接线**——
   生产者（本步及步骤 3）尚未把结果送入缓存路径。
8. **有限扰动独立 oracle 未做**——属步骤 5。
