# P4a 步骤 1 证据：冻结协议与原样本残量审计

> 验收条件（TODO.csv 行 1）：**惯性wrench/world速度/材料点/乘子规则可验证，目标力律能力准确**。
> 本文件记录实际行为断言、命令退出码、误差/预算与缺口。命令通过/文件存在只作门禁，
> 不作为 DONE 依据。

## 1. 结论

步骤 1 **完成并通过行为验收**。交付物是"冻结样本构造器"——
把动态样本的既有状态描述成 `WorkingSample`，供后续步骤装配切线。

**本步只交付"描述状态"，不交付"装配切线"**：`frozen_force_tangent`（步骤 2）与
`frozen_load_compliance`（步骤 3）尚未实现，`B_w` 仍为零实现。本步的证据**不覆盖**SPEC §56–59
的 K=k / C=1/k 等验收（那属步骤 2/3）。

## 2. 交付物

| 文件 | 性质 | 说明 |
|---|---|---|
| `cpp/include/mb_linearization/types.hpp` | 改 | 新增 `frozen_working_sample` 声明（含契约文档） |
| `cpp/src/linearization/physical.cpp` | 改 | 新增 `frozen_working_sample` 定义 |
| `cpp/include/mb_model/functions.hpp` | 改 | 新增 `gauge_coordinates` 声明 |
| `cpp/src/model/kernel_model_accessors.cpp` | 改 | `gauge_coordinates` 定义 |
| `cpp/src/solve_static/kernel_static_trim.cpp` | 改 | `static_gauge_coordinates` 改为委托 |
| `cpp/tests/frozen_sample_selftest.cpp` | **新增** | 独立自测可执行文件，8 组断言 |
| `CMakeLists.txt` | 改 | 注册 `mb_frozen_sample_selftest` |

### 2.1 为什么抽出 `gauge_coordinates`（D8 + 分层）

`static_gauge_coordinates` 原是 `mb_solve_static` 的私有函数，但它是
**模型的纯函数**（只读 `static_gauge_body` / `static_gauge_dof_mask` / `body_to_free`），
而 `mb_linearization` 在求解器**之下**、契约明写"it never calls back up"
（`types.hpp:9`）。若从 `mb_linearization` 调上去就是分层反转 + 成环。

故按 R8/P3 已确立的"抽到正确的层"模式，把它下沉到 `mb_model`——
`mb_model → mb_numeric` 是既有边，且该函数只读模型自身字段，**不引入任何新边**。
旧位置改为委托（设计 §576：抽出后旧位置必须委托，不得双算）。

**分层门实测**：`module cycles (SCC size>1) : 0`，`OK: layering matches the recorded baseline`
——**0 环、无新边**。

### 2.2 冻结协议的字段来源（全部是模型级数据，无新策略概念）

| `WorkingSample` 字段 | 冻结态取值 | 依据 |
|---|---|---|
| `gauge` | `gauge_coordinates(model)` 展开 | 模型的纯函数 |
| `position_constraint` | `constraint_jacobian(model, pose)` | 静态侧 mask 恒全 1，见 §2.3 |
| `active_tires` | 调用方传入（按列序） | SPEC §34"活动分支" |
| `load_scale` | `1.0` | SPEC §35"未扰动输入" |
| `pose` | 调用方传入的冻结状态 | `copy_state` 列 0..12 |

### 2.3 "mask 恒全 1"的论证（诚实边界）

`static_position_constraint_mask`（`kernel_static_trim.cpp:318-330`）初值
`std::vector<unsigned char> mask(model.rows, 1)`，随后循环 `mask[coupler.row] = 1;`
——**循环只把已经是 1 的位置再写成 1，从不写 0**。

因此 `static_position_constraint_jacobian` 的"清零非 position 行"步骤**不生效**，
masked 矩阵 **就是** `constraint_jacobian` 本身。

**这条推理的要害不是"循环是空操作"，而是"没有任何路径把某行置 0"。**
已核对唯一写入路径：coupler 行只能以 1 进入 mask。故当前**确等价**。

**已登记的脆弱点**：若将来该 mask 长出"把某行置 0"的能力（例如给 coupler 加
非位置约束分类），本处的等价性会**静默失效**。已在 `types.hpp` 与 `physical.cpp`
两处注释中显式写明"这一行是 mask 变化时要一起改的地方"，避免靠一条无法失败的
推理支撑决策。

## 3. 行为断言（8 组，全部实测）

自测可执行 `mb_frozen_sample_selftest.exe` 覆盖：

| # | 断言 | 为什么需要它 |
|---|---|---|
| 1 | 报告的 `pose`/`v` **逐位等于**传入值 | 一个偷偷求解邻近状态的实现同样返回 `true`，且会让后续切线答非所问 |
| 2 | 无 gauge 的模型 → gauge **全 0** | 这是该模型的**诚实描述**，不是默认值 |
| 3 | gauge **恰好**是模型 mask 选中的坐标 | 全部 gauge 掉或 gauge 掉整个自由体都会"看起来合理" |
| 3b | mask `0x21` → 坐标 **0 与 5** | 与 #3 的 `0x9` 不同：把 mask 当作"坐标个数"或"从 0 起的连续区间"在 `0x9` 上会碰巧正确，只有非相邻位能区分 |
| 3c | 拒绝时 `working` **完全未被写入** | 见 §4 缺陷 2——部分填充的拒绝会让调用方拿着**上一状态**的矩阵，且尺寸仍匹配模型，装配器会接受 |
| 4 | `position_constraint` 是 `rows×ndof` 且非空 | **模型故意带 2 条约束行**：`rows=0` 时"正确计算"与"返回空"都是空，断言会假过 |
| 5 | `load_scale == 1.0`；`active_tires` **按序**透传 | 排序会静默重标接触列 |
| 6 | 体数不匹配 → 拒绝（多/少都测） | 交错的 `r` **不是**物理判定，是调用方递了别的模型的状态 |
| 7 | 非有限 `r`/`q`/`v`/`omega` → 拒绝 | 力律读 `v`/`omega`，NaN 会产出 NaN 切线而非拒绝 |

## 4. 独立复审发现并已修复的缺陷（D2）

对交付代码做了只读对抗性复审，发现 **4 个真实缺陷**，全部已修并补测：

1. **【已修】`false` 返回时 `working` 已被部分填充，与头文件契约矛盾。**
   原实现先写 `gauge`/`active_tires`/`load_scale`/`pose`，再在 `position_constraint`
   尺寸校验失败时返回 `false`。后果**不是拒绝而是静默复用陈旧数据**：旧矩阵尺寸
   恰好匹配模型时装配器会放行，用上一状态的约束 Jacobian 装配出"看似合理"的切线。
   **修法**：所有拒绝测试前移到任何写入之前；约束 Jacobian 先算进局部变量，
   尺寸通过后才 `std::move` 进 `working`。契约文档同步改为"拒绝时 `working` 原样不动"。

2. **【已修】`v`/`omega` 未做有限性检查**（只查了 `r`/`q`）。
   力律会读它们，NaN 会一路进装配。已补测（断言 #7 扩展）。

3. **【已修】`constraint_jacobian` 的前置条件未检查。**
   其内部以 `int` 算术算 `model.rows * model.ndof` 再转 `size_t`；负值或溢出会回绕成
   巨量分配并从本函数泄漏出 `length_error`。已加维度检查。
   （相对地，写入侧 ABI 有三条校验，故经合法路径不可达——这是**防御深度**问题，不是现存可达 bug。）

4. **【已修】原注释的论证对象错了。**
   原文说"循环把已置 1 的行再置 1，所以 mask 恒全 1"——**结论碰巧对，但论证的不是要害**。
   真正需要论证的是"有没有路径把某行置 0"。已按 §2.3 重写，并标出脆弱点。

**复审明确判定为正确的部分**（已独立核对，不采信为"看起来没问题"）：
搬移忠实逐项对应；分层无新边、无反向调用；`gauge_coordinates` 的
`body_to_free` 索引安全（由写入侧三条校验保证）；CMake 注册与既有 selftest 一致。

### 4.1 复审提出但**未采纳**的建议（记录理由，避免当作事实传播）

复审建议补一个有 `coordinate_couplers` 的模型来覆盖 `constraint_jacobian` 的 coupler 路径。
**未采纳**：本步交付的是"冻结样本构造器"的描述能力，coupler 路径是
`constraint_jacobian` **自身的**覆盖问题，与冻结协议无关；在合成模型上手搭 coupler
只能证明"我搭的那两条能过"，不能证明生产路径正确，反而会造出一条**看起来覆盖了**
实则与生产无关的证据。真正的覆盖由步骤 6 的 `case_parity_check.py`（8 个族、真实模型）承担。
**这是一条"测试不该假装覆盖"的判断，已登记而非静默略过。**

## 5. 投毒验证（sabotage）：12/12 全部被抓，0 存活

投毒脚本 `tmp_p4a_sabotage.py`（在 `$env:TEMP`，不入库）：逐条变异 →
重新构建 → 运行 → 必须失败 → 还原。
读取/写入**字节**并对齐 `NEWLINE`（本检出 CRLF），还原后校验 SHA256。

| 变异 | 结果 |
|---|---|
| gauge 被丢掉（从不置位） | CAUGHT |
| gauge 置位**所有**坐标 | CAUGHT |
| `load_scale` 不是 1.0 | CAUGHT |
| 活动轮胎被排序 | CAUGHT |
| 体数不匹配被接受 | CAUGHT |
| 非有限位姿被接受 | CAUGHT |
| `position_constraint` 留空 | CAUGHT |
| 冻结位姿被置零 | CAUGHT |
| `gauge_coordinates` 忽略 mask | CAUGHT |
| **拒绝前就写 sample（顺序回归）** | CAUGHT |
| 非有限速度被接受 | CAUGHT（BUILD-FAIL） |
| mask 被解释为连续区间而非位 | CAUGHT |

**一次无效变异已修正并记录**：首轮"顺序回归"变异被插在**所有拒绝测试之后**，
故语义等价、无从违反——这是我的变异构造错误，不是测试缺口。改为插在首个拒绝测试
**之前**后即被抓。**同类无效变异不得记作"测试通过"。**

**一次真实测试缺口已由投毒暴露并修复**：首轮"`position_constraint` 留空"**存活**，
原因是当时模型 `rows=0`，正确计算与留空**都是空矩阵**，形状断言假过。
给模型加 2 条真实约束行后即被抓。这正是投毒存在的意义。

## 6. 命令与退出码（全部实测）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `build_axle_native.py` | 0 | DLL 产出 |
| `cmake --build . --target mb_frozen_sample_selftest` | 0 | 编译通过（`-Wall -Wextra -Werror`） |
| `mb_frozen_sample_selftest.exe` | 0 | `OK: frozen working sample self-test passed` |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |
| `legacy_surface_gate.py --check` | 0 | 0 findings |
| `check_module_layering.py --strict --final` | 0 | **0 环、无新边** |
| `check_composable_release.py --skip-isolation` | 0 | 3 release checks passed |
| 快速集（`--ignore` adams/architecture/cases） | 0 | **1958 passed, 1 xfailed**（与基线一致） |
| kernel + contracts（**分开调用**） | 0 | **176 passed**（与基线一致） |
| `dynamic_hash_sentinel.py --check` | 0 | **逐位匹配冻结基线** |
| `case_parity_check.py` | 0 | 8 族接受 |
| `kc_perf_gate.py` | 0 | 预算内（k-100 ×1.118，c-66 ×1.089） |
| `git diff --check` | 0 | 干净 |

**未跑**：`just test-all`（33 min 全量，按 AGENTS.md 属 P6 收尾项）。
skip/xfail **未增长**（1 xfailed，与基线同）。

## 7. 与既有机制的一致性核对

- **无请求基线不变**：`dynamic_hash_sentinel` **逐位匹配**证明动态输出未变。
  本步改动落在静态切线与模型访问器上，且验证了 `static_gauge_coordinates` 的行为等价。
- **分层**：0 环、0 新边。
- **未重录任何冻结基线**。
- **未削弱任何断言**：全量测试数与基线一致。

## 8. 已登记缺口（不隐瞒）

1. **本步不交付切线**：`frozen_force_tangent` / `frozen_load_compliance` 未实现，
   `B_w` 仍零实现。SPEC §56 的 K=k / C=1/k 验收**属步骤 2/3**，本文件不冒认。
2. **`position_constraint` 等价性依赖 mask 不变**，脆弱点已注明（§2.3）。
3. **coupler 路径未被本步测试覆盖**（§4.1，刻意，理由已记）。
4. **`a`/`alpha` 不在 `pose` 必需字段内**：实测切线装配路径**不读**它们，
   故是"形状宽松"而非 bug；契约已写明"若将来某力律开始读，契约要一起长"。
5. **`constraint_jacobian` 的维度检查是防御深度**：经合法 ABI 路径不可达
   （写入侧三条校验），但本函数不依赖该外部保证。
6. **自测可执行文件未接入 pytest**：与既有 `mb_linear_batch_selftest` 同模式
   （独立可执行、证据中登记实测退出码）。**这是刻意的**：它是内部能力，
   走产品 ABI 会需要升 ABI 版本。
7. **未做 P2 步 1 / P3 步 1 的专项 D2 复审**（历史遗留，仍在缺口清单中）。
