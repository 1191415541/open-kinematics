# P4a 步骤 3 证据：具名载荷模式响应及可辨识诊断

> 验收条件（TODO.csv 行 3）：**C 响应不重配平，组合奇异正确，完整整车耦合保留**。
> 本文件记录实际行为断言、命令退出码、误差/预算与缺口。命令通过/文件存在只作门禁，
> 不作为 DONE 依据。

## 1. 结论

步骤 3 **完成并通过行为验收**。交付物是**载荷驱动边界**的第一处实现：
`B_w = -dR/dw` —— 机械残差对**一个单位 wrench 分量**的导数，此前全仓库零实现。

**本步交付的是载荷侧的"列生产者"与它的行为验收。** 消费层算子（`A X = B_w P`、
`Y = G X + D P`）在 P2 已完整实现且无生产者；本步补上了载荷侧输入，
使 SPEC §57 的 `C = 1/k` 第一次成为可算的量。

**本步不交付**：ABI 导出、`characteristics_response_kinds` 声明、
`local_response_v1` 结果通道接线（见 §8）。故本步证据**不覆盖** SPEC §30/§40 的
"结果通道记录模式 ID/测量 ID/有效性"，那属步骤 4。

## 2. 交付物

| 文件 | 性质 | 说明 |
|---|---|---|
| `cpp/include/mb_solve_static/functions.hpp` | 改 | 新增 `frozen_load_column` 声明 + 契约文档 |
| `cpp/src/solve_static/kernel_static_trim.cpp` | 改 | 新增 `frozen_load_column` 定义 + `frozen_load_gauge_coordinate` 共享谓词 |
| `cpp/tests/frozen_load_selftest.cpp` | **新增** | 独立自测可执行文件，7 组断言 |
| `CMakeLists.txt` | 改 | 注册 `mb_frozen_load_selftest` |
| `tests/test_registry_consistency.py` | 改 | 注册该自测，使断言真正被门禁执行 |

### 2.1 为什么用 `body_wrench` 而不是新造载荷通道（D6 复用）

已有的具名载荷通道就是 `SampleInput::body_wrench`（`mb_model/types.hpp:533`，
布局 `bodies*6`、每体 `[Fx,Fy,Fz,Mx,My,Mz]`），且**端到端已存在**：

- 施加：`force/layout.cpp:148-190`，`external_load_scale * body_wrench`；
- 力臂：`:172` `torque += cross(rotate(q, point), force)`，作用点由
  `model.body_wrench_point_local`（契约 `body_wrench_markers`）声明，
  **在求解位姿处解开**——正是 SPEC §35/§42 要求的几何项；
- 具名模式：`kc_quasi_static.cpp:545-597` 已按"具名 wrench 载荷 + 同相/反相镜像"
  组织载荷模式（SPEC §39 的"同相/反相等模式"）；
- Python 侧：`signal_bus.py:461`、`axle_channels.py:562-569`、`schema.py:1325` 均已接。

故本步**不新增任何载荷概念**，只补"对载荷求导"这一件缺失的事。

### 2.2 列的解析形式

`static_residual` 的力平衡行为 `out[col] = -force[col] - reaction[col]`（`:74`），
而 `force = +external_load_scale * body_wrench`。故

```
dR/dw[k]  = -1    在坐标 6*fi + slot(k) 上（load_scale = 1）
B_w       = -dR/dw = +1
slot(k)   = Fx,Fy,Fz → 0,1,2 ; Mx,My,Mz → 3,4,5
```

力在声明点还产生力矩 `cross(R*p, F)`，故

```
d/dF_k cross(R*p, F) = cross(R*p, e_k)
B_w[base+3..5]       = +cross(R*p, e_k)
```

**在给定 `pose` 处求值**，所以力臂随基点变化，不是常数。

## 3. 行为断言（7 组）

### 3.1 六个分量各自落位（不是只测 `Fx`）

对 `component = 0..5` **逐一**断言"仅目标位为 `-1`、其余全 0"。
只测 `Fx` 会让"把每个力都映射到第一个坐标"的实现通过；置换的实现只有逐个断言才抓得住。

### 3.2 符号

`-1` 是 `dR/dw` 的值（`B_w = +1`）。单独断言且用 `== -1.0` 精确比较，
使"丢符号"报成符号错而不是落位错。

### 3.3 力臂必须读位姿

标记 `p=(1,0,0)`、绕 z 四分之一转：`R*p=(0,1,0)`、
`cross((0,1,0),(1,0,0))=(0,0,-1)`，经 `-dR/dw` 取反为 `+M_z = +1`。

**对照**断言：同一标记在**单位位姿**下力臂为零。
忽略旋转的实现会在旋转用例上失败——这是该组的关键。

### 3.4 力矩模式不获得力臂

`dM_k/dM_j` 是恒等，`∂cross(R*p,F)/∂M_k = 0`。断言落在**转动块**而非平动块：
若力臂错误地对力矩模式生效，转动物理会多出 `cross(R*p, e_axis)`，
而平动块仍为零——只看平动块的测试会放过该缺陷。

### 3.5 不能动机构的载荷返回零列而非报错

固定体（`body_to_free = -1`）、越界 body/component 一律返回**尺寸正确的零列**。
契约明写这是刻意的：「该方向无作用」与「该方向不可能」是两个不同事实，
只有调用方的相容性审计能把它们分开（设计 §437 把区分责任放在审计侧：
不相容报 `singular`，测量不唯一报 `unidentifiable`）。在这里合并会把合法零响应变成拒绝。

**越界防护**用可观测的构造断言（见 §4.1）：`body_to_free = 100000` 的失控模型，
以及 `ndof` 被截断的模型。

### 3.6 gauge 坐标不得获得载荷列（**本步最严重的缺陷，见 §4.2**）

gauge 行不是力平衡行：切线把它替换成恒等行（`linearization/physical.cpp:144-147`），
静态残差把 gauge 值写在该行上（`kernel_static_trim.cpp:77-82`、`:108-109`）。
这些行**与载荷无关**，故真实 `dR/dw = 0`。

断言覆盖**三种**进入方式，且都带"夹具是活的"对照：

| 方式 | 断言 |
|---|---|
| 全局 gauge（`static_gauge_body` + mask）落在平动坐标 | 该坐标为 0；同一 body 的**未** gauge 坐标仍答 `-1` |
| 旋转 gauge（`static_rotation_gauges[].pivot`）落在转动坐标 | 该坐标为 0 |
| 力臂写入转动块时命中 pivot | pivot 坐标为 0，**同一模型的未 gauge 版本反而写 `+1`** |

第三行是关键：它把"守卫生效"与"测试根本没触发"分开。
若只断言"pivot 为 0"而夹具的力臂恰好不产生 Mx，则该断言恒真。

### 3.7 组合链：`C = 1/k`（SPEC §57）

`A X = B_w P` 用**载荷列作右端**真解一次，得到 `C = 1/k`。

**同一处断言两个边界不互为倒数替代**：坐标驱动路径对同一根弹簧报 `k`，
载荷驱动报 `1/k`。设计明确禁止用求逆或倒数切换物理边界（SPEC §37、设计 §422），
把两个数放在同一测试里使"把一条路径简化成另一条"必然失败。

## 4. D2 复审结论与处置

对交付代码做了只读对抗性复审。**发现 2 个 high 缺陷，均已修复并补回归测试。**

### 4.1 严重：gauge 行被写入 `-1`，凭空制造右端（**已修**）

**缺陷**：函数无条件把 `-1.0` 写进 `column[base+slot]`，不看该坐标是否为 gauge。
但同一坐标在切线 `A` 里是恒等行（`physical.cpp:144-147`），残差是纯位姿量
（`:77-82`），真实 `∂R/∂w = 0`。

**后果不是"一个错的向量元素"**：它给一个**不能承载右端的行**伪造了右端，
于是 `A X = B_w P` 解出 `X[gauge] = -1` 并向所有测量经 `Y = G X` 泄漏——
"载荷施加在模型刻意不允许移动的坐标上"被呈现为一个单位位移响应。

**已修**：提取共享谓词 `frozen_load_gauge_coordinate(model, coordinate)`，
同时识别全局 gauge mask 与 `static_rotation_gauge_for_pivot`
（后者已在 `mb_model/functions.hpp:72` 声明，无新边）。检查在**坐标**上做，
故覆盖平动块、转动块与力臂三项写入。

### 4.2 严重：缺 `load_scale` 因子（**已修，方式为明确契约而非加参数**）

**缺陷**：`static_residual` 有 `load_scale` 形参并作为 `external_load_scale`
乘进 `body_wrench`（`layout.cpp:151-158`），真实导数是 `-load_scale` 而非 `-1`；
且该参数**存在非 1 值的真实调用者**（`kernel_static_contact.cpp:270-271`
按载荷步从 `1/static_load_steps` 递增到 1）。

**处置**：**不加参数，改为明确契约**。理由是冻结协议自己就把
`working.load_scale` 钉成 `1.0`，并把原因写出来了
（`linearization/physical.cpp:350-353`："冻结响应是在未扰动载荷处提问……
在这里缩放载荷会回答另一个问题"）。加一个参数会让**只有一种合法取值**的旋钮
出现在签名里，反而制造误用面。

实现改为显式写明该列对应 `external_load_scale == 1`，并说明调用方若在别的尺度
装配切线则必须按比例缩放该列。原注释的"scaled by unity"读起来像在陈述
`layout.cpp` 的行为，**已重写**为对入参的约束。

### 4.3 复审发现的一个真实测试缺口（**已修**）

复审指出 §3.7 那组**几乎恒真**：它手写对角除法而非真正求解，
全部信息量等于"`column[0] == -1`"这一条已在别处断言过的数。
复审同时确认它**并非纯恒真**（载荷列若全零该断言会失败），判定准确。

**已修**：gauge 回归用例（§3.6）是当前唯一能抓住 §4.1 缺陷的构造，已加入。
此笔记为"复审证明我的自测在最重要的一处没有覆盖"，如实登记。

### 4.4 复审确认正确的点

- **映射**：`slot = is_force ? axis : 3 + axis` 逐个代入六种分量确实给出
  `[6*fi+0..5]`，无转置、无 off-by-one。
- **符号**：从第一原理推导 `∂R/∂w = -1`（`load_scale=1`），
  与 `B_w = +1` 及设计 §410 的定义一致，**未被翻两次**。
- **力臂符号**：`-=` 与 `B_w = +cross` 自洽（负号已由 `B_w = -∂R/∂w` 提供）。
- **越界**：五道守卫逐条完备，**不存在可达的越界读写**；
  `pose.q` 按**体**索引、`column` 按**自由坐标**索引，两个索引空间未被混淆。
- **与 `frozen_force_tangent_column` 无隐藏耦合**：无共享可变状态，
  `frozen_load_column` 不触碰 `active_element_wrench_sink()`。
- **零标记与显式 `(0,0,0)` 标记**判据与 `layout.cpp:171` 逐字一致。

## 5. 蓄意破坏验证（mutation testing）

字节级变异 → 重建 → 运行，要求自测**失败**。字节级读写 + `NEWLINE` 归一
（本 checkout 为 CRLF，文本模式会静默重写全部行尾）。每条变异先验模式唯一性，
恢复后 SHA256 复核，并**重跑基线确认绿**。

**结果：13/13 抓住，0 个未解释存活。**

| 变异 | 结果 |
|---|---|
| 力矩写到同轴的平动槽 | CAUGHT |
| 力写到转动槽 | CAUGHT |
| 载荷列丢符号 | CAUGHT |
| 力臂未旋转到 world | CAUGHT（编译失败） |
| 力臂力矩符号错误 | CAUGHT |
| 力臂**错误地**对力矩模式生效 | CAUGHT |
| 力臂**被丢掉**（力模式无力臂） | CAUGHT |
| 越界分量被接受 | CAUGHT（崩溃） |
| 越界基坐标检查被去掉 | CAUGHT（崩溃） |
| **gauge 坐标获得载荷列** | CAUGHT |
| 旋转 gauge 规则被去掉 | CAUGHT |
| 全局 gauge mask 被忽略 | CAUGHT |
| **力臂忽略 gauge 规则** | CAUGHT |

### 5.1 三条**语义等价**变异（如实登记，不计入"抓住"）

| 变异 | 为何不可观测 |
|---|---|
| 固定体返回载荷列而非零 | `free_body < 0` 被**两次**守卫（直接一次，`base = 6*free_body` 后 `base < 0` 又一次）；去掉前者，后者拒绝同一批输入 |
| 零标记被当作力臂 | `cross(rotate(q,{0,0,0}), e) = cross(0, e) = 0`，`if (true)` 形式加进去的就是被豁免跳过的那个零 |
| 负 body 被接受 | 去掉守卫使 `body_to_free[-1]` 成为**越界读**，值不确定；本构建返回一个恰好落在 `ndof` 内的垃圾索引，写入仍在界内，返回列两种情况都是全零 |

三条单列为 `EQUIVALENT` 并**每轮重测**（行为变化会显示 "NO LONGER EQUIVALENT"
而非静默通过）。第三条尤其要说明：**它能写成断言之外的原因正是这条守卫存在的理由**——
它是坏入参与未定义行为之间唯一的东西，而这一点无法由"对答案的断言"证明。

## 6. 命令与退出码

| 命令 | 退出码 | 结论 |
|---|---|---|
| `cmake --build . --target mb_frozen_load_selftest` | 0 | 编译通过 |
| `mb_frozen_load_selftest.exe` | 0 | `OK: frozen load column self-test passed` |
| `mb_frozen_tangent_selftest.exe` | 0 | 步骤 2 回归未破坏 |
| `ruff check .` | 0 | `All checks passed!` |
| `ty check .` | 0 | `All checks passed!` |
| `check_module_layering.py --strict --final` | 0 | `OK: layering matches the recorded baseline`（**0 新边**） |
| `legacy_surface_gate.py --check` | 0 | `OK: no unregistered Python boundary violation` |
| `check_composable_release.py --skip-isolation` | 0 | `OK: 3 release checks passed` |
| 快速测试集 | 0 | **1958 passed, 1 xfailed**（基线不变） |
| kernel+contracts | 0 | **178 passed**（177 基线 + 新增注册 1） |
| `dynamic_hash_sentinel.py --check` | 0 | **逐位不变** |
| `case_parity_check.py` | 0 | `OK: 8 families accepted` |
| `kc_perf_gate.py` | 0 | 性能预算内 |
| `git diff --check` | 0 | 干净 |

## 7. 一致性检查

- **无基线重录**：`dynamic_hash_sentinel` 逐位不变、`kc_perf_gate` 在预算内，
  未触碰任何冻结基线。**这是"新函数只增加能力、不改既有物理"的最强证据。**
- **分层零新增边**：`frozen_load_column` 只用已在 `mb_solve_static` 边内的
  `mb_model` 类型与函数（`static_rotation_gauge_for_pivot` 已在
  `mb_model/functions.hpp:72`），故 `layering_baseline.json` **未改**。
- **单一物理路径未被绕过**：未新增 Study/Case 族、未新增入口点、未改 `ResolvedSolvePlan`。
- **能力声明未被虚假填充**：`characteristics_response_kinds` 仍为**空数组**——
  本步**不导出 ABI**，故不声明任何 kind。
- **测试基线未增长**：skip/xfail 未增长（仍 1 xfailed）。
- **与步骤 2 的边界独立性**：两条路径无共享可变状态、互不调用，
  且 §3.7 在同一处断言 `k` 与 `1/k`，使"互相推导"必然失败。

## 8. 登记缺口（如实）

1. **载荷列未跨 ABI 导出**——本步函数是**内部**能力。导出矩阵 + 声明
   `characteristics_response_kinds` **必须同时做**；当前该数组仍为**空**。
2. **`local_response_v1` 结果通道未接线**——SPEC §30/§33 的模式 ID/测量 ID/
   分量单位/有效性记录属步骤 4；消费层 `equilibrium.py` 已具备该能力但未接生产者。
3. **`B_w` 的"载荷方向"仍只有 body-wrench 一种表达**——命名轮端模式
   （左/右、力/力矩轴、作用点）到 `body_wrench` 的**映射层**未实现。
   设计 §436 的 `adams_wheel_center_12_v1` 适配布局属可选项，独立登记。
4. **组合相容性/可辨识诊断未做**——设计 §437 的"组合有效而单列无效仍应给有效指标"
   需要 `B_w P` 的实际相容性判定；消费层 `audited_modes` 有该能力，未接。
5. **单位与 frame 换算未做**——设计 §435 的左右符号/frame 转换作用于
   相应测量与载荷映射，不能对整组输出乘常数；属步骤 4。
6. **转置/伴随求解路径未验证**——设计 §428 的 `A^T Z = G^T` 属步骤 5。
7. **有限扰动独立 oracle 未做**——属步骤 5。
8. **`mb_frozen_sample_selftest` 在 pytest 进程内崩溃原因未明**（步骤 1 遗留，见
   `raw/frozen-tangent-evidence.md` §4.2），未注册。
9. **`mb_linear_batch_selftest` 同样无调用者**——既有缺口，不属本步权限。
10. **`factor`/`isfinite` 双防线冗余无注释**（步骤 2 遗留），当前无缺陷。
11. **`frozen_protocol`/`frozen_version` 缓存键已预留但未接线**。
12. **P2 步骤 1 / P3 步骤 1 的专项 D2 复审仍未做**（历史遗留）。

## 2026-10-12 纠正（不改写上文，只在此追加）

本文档 §2/§3 的原表述"`B_w` 的条目是 `-1`"**已被实测推翻**。响应方程组是
`A X = B_w P`，`B_w = -dR/dw`：单位载荷分量在机械残差里带来 `-1`
（`out[col] = -force[col] - reaction[col]`），故 **`B_w` 在对应广义坐标上是 `+1`**，
力臂项是 **`+cross(R*p, e_k)`**（保物理符号）。原实现检查的符号恰好相反，
是一个"看起来像符号约定"的错答案。

判定依据不是推理而是**真实残量回归**：`cpp/tests/frozen_load_selftest.cpp` 新增
`test_load_column_is_negative_derivative_of_real_residual`，对真实 `static_residual`
的**六个分量**各做中心差分（`h=1e-4`），逐行断言 `B_w[row] == -(R(+h)-R(-h))/(2h)`（`1e-12`）。
该用例在旧符号下**先红**，修正 `kernel_static_trim.cpp`（`-1.0` → `1.0`、
力臂 `-=` → `+=`）后 `mb_frozen_load_selftest.exe` exit 0。
本文档 §3.2 的"单列断言 `== -1.0`"与 §4.3 的蓄意破坏表**都已随之重测**，
不再作为该结论的证据。

同轮的第二个真实缺陷（本文档 §8 未记录）：`frozen_load_column` 遇到力的平移 slot 是
gauge 时**整列 return**，把力臂力矩一并丢弃。gauge 只替换该坐标的方程，不改同体行块内的
物理力矩。现改为只抑制自身条目后继续进入力臂分支；新回归
`test_a_gauge_translation_still_carries_its_lever_arm_moment` 先红（`column[4]=0`）后绿，
并带无 gauge 的反向对照（`column[0]` 与 `column[4]` 同时为 `1.0`）。

本文档 §8 第 8 条"`mb_frozen_sample_selftest` 在 pytest 进程内崩溃原因未明"**本轮未再复现**：
该自测已注册进 `test_registry_consistency.py`，本轮在本模块内连跑 3 次均
**18 passed / exit 0**（每轮 1.0~1.2 s），`pytest packages/suspension_kernel/tests`
由 180 增至 **182 passed**。

**必须如实说明**：崩溃的**根因仍未定位**。我一度推断是旧夹具 `model.tires` 为空却被传入
`active_tires={1,0}`，导致 `assemble_physical_tangent` 里 `active_mask[tire_index] = 1`
越界写；**该推断已被证伪**——`git show HEAD:.../frozen_sample_selftest.cpp` 实测
`assemble_physical_tangent` 调用次数为 **0**，该自测从不进入装配路径，故不存在这条越界写。
当前可陈述的事实只有两条：①注册后本模块内不再复现；②同一位置既有对照自测
（`mb_frozen_tangent_selftest`）一直稳定。二者都不构成根因解释，**不声称已定位**。
本轮与该路径相关的确定改动是：夹具把 `model.rows` 由 2 改为 10 并 `model.tires.resize(2)`、
新增越界/重复 `active_tires` 的拒绝断言、`mb_frozen_load_selftest` 的链接组改为
`--start-group` 全链（原为算子子集，符号未定义行为可能由此而来，但**未验证**）。

本轮完整证据见 [frozen-inertial-load-evidence.md](frozen-inertial-load-evidence.md)。
