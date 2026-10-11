# P4a 步骤 4 证据：逐样本响应/指标共享与来源贯通

> 验收条件（TODO.csv 行 4）：**Wheel/Ride/Roll/Toe/Camber 实际序列；同 A 分解共享、来源可追溯**。
> 本文件记录实际行为断言、命令退出码、缺陷与缺口。命令通过/文件存在只作门禁，
> 不作为 DONE 依据。

## 1. 结论

步骤 4 **完成并通过行为验收**。交付物是**逐样本驱动层**：
把"某样本的物理切线 + 一组具名载荷模式"变成**按样本对齐的具名序列**，
并让"同一份 A 只分解一次"成为**可观测**的事实而非承诺。

**本步交付的是消费层的调度与簿记，不是新的物理算子。**
求解算子（`A X = B_w P`、`Y = G X + D P`）在 P2 已完整实现；
步骤 3 补上了载荷侧输入 `B_w`；本步把两者按样本串起来，并回答三个此前无人回答的问题：

1. **一个样本问多个问题，A 分解几次？** —— 一次。循环是"样本在外、问题在内"，
   切线按样本取；`PerSampleReport.counters` 让共享次数**可被读出**。
2. **一个样本解不出来时，其余样本怎么办？** —— 该样本记为**拒绝**（NaN + 非空原因），
   不抛异常、不丢弃邻居。
3. **两条序列凭什么说是同一来源？** —— `provenance` 记边界 profile/version、
   model/plan 指纹、冻结协议与版本；`SampleSequence` 公开 `mode_ids`/`units`/`sample_indices`
   与逐样本 identity，使列标签不从位置猜。

**本步不交付**：ABI 导出、`characteristics_response_kinds` 声明、
`local_response_v1` 结果通道（见 §8）。故本步证据**不覆盖** SPEC §30/§40 的
"结果通道记录模式 ID/测量 ID/有效性"，那属 P4a 与 ABI 的接线步骤。

**本步最重要的发现是一个设计缺陷，而不是一个功能。**
首轮交付通过全部 35 个自测、11 道静态门与 23/23 蓄意破坏验证，仍被第二份独立对抗性
复审找出 **6 处真实可复现缺陷**，其中 3 处由探针当场复现。它们的共同形态是
**"合法输入 ⇒ 静默的错误有限值"或"拒绝路径形状不一致"**，而不是崩溃。
这说明：**自测通过、门禁全绿、破坏测试全抓，都不足以证明一段代码正确**；
下面 §5 用 32 道突变固化这些修复。

## 2. 交付物

| 文件 | 性质 | 说明 |
|---|---|---|
| `results/per_sample.py` | **新增** | 逐样本驱动层：`SampleQuestion`/`SampleAnswer`/`SampleSequence`/`PerSampleReport`/`drive_sample_questions` |
| `results/__init__.py` | 改 | 再导出上述公共名 |
| `results/equilibrium.py` | 改 | `audited_modes` 接受调用方 `mode_matrix`；解除"测量行数 ∈ {1, 模式数}"限制 |
| `tests/characteristics/test_per_sample_sequences.py` | **新增** | 46 个用例（含 11 个第二轮复审回归用例） |

### 2.1 为什么复用 `audited_modes` 而不是新写一条求解路径（D6 复用）

设计 §6.6:414 已经把请求形状定死：`A X = B_w P`、`Y = G X + D P`，
`P[:,j] = ∂w/∂α_j` 由**调用方**给出（轮端、力/力矩轴、作用点、幅值单位都在 `P` 里）。
已有的实现就是 `equilibrium.audited_modes`（P2 交付），它已经提供本步唯一不能自造的东西：

- **逐模式判定**。`audited_modes` 是本仓库**唯一**"载荷模式自身无效"的权威：
  某模式的 `B_w P[:,j]` 数值为零时，它返回 `status="unidentifiable"` 且 `values=None`。
  其下的 `equilibrium_response` 是**全局**审计，会把这个情形发布为
  `X = A^{-1} 0 = 0` —— 一个格式良好的零，与"机构确实不响应这个载荷"**无法区分**。
  任何把调用点下沉到 `equilibrium_response` 的重构都会静默丢掉这条拒绝。
- **`G X + D P` 的直接项**。`D` 非零时测量读的是载荷本身（如轮荷），
  丢掉它会得到一个"差得刚好要命"的数。

因此本步**没有**新造求解器、没有平行实现，只补了调度、簿记与请求校验。

### 2.2 关键实现决策

| 决策 | 理由 |
|---|---|
| 切线是 `drive_sample_questions(tangents=[...])` 的**参数**，不是 `SampleQuestion` 的字段 | 切线属于**样本**。做成问题字段会让同一采样点的两个问题对"在问哪个机构"产生分歧，而答案照样看起来合理。 |
| 循环**样本在外、问题在内** | 问题在外会按样本重复分解，静默毁掉共享，且答案毫无异样。 |
| 拒绝是**逐样本记录**，不是抛出 | 长序列中一个奇异样本不该丢弃答得出来的样本。**请求形状**错误（模式数不符、维度不符）才抛，因为那是请求写错而非样本属性。 |
| `mode_ids` 从 `question.modes` 取，不从数组宽度反推 | 宽度反推会让"静默截断的多分量运动"看起来完全正常。 |

## 3. 行为断言（16 组）

| # | 断言 | 对应修复 |
|---|---|---|
| 1 | 每样本一条序列条目，按样本顺序排列 | — |
| 2 | 序列携带每个样本的 identity（`sample_index` ≠ 循环位置） | 复审 |
| 3 | 闭式柔度 `C = 1/k` 在每个样本成立（SPEC §57 标量 oracle） | — |
| 4 | 只要机构运动也产生具名序列 | — |
| 5 | **每样本一次分解，不是每问题一次**（`factorization_count == 样本数`、`numeric_reuse == 0`） | — |
| 6 | 第二问复用第一问的分解（`numeric_reuse` 增长） | — |
| 7 | 样本改变后**绝不**复用上一处分解 | — |
| 8 | `solve_count` 记录正确的独立右端数 | — |
| 9 | 奇异样本被拒绝，**且不丢弃邻居** | — |
| 10 | 被拒绝样本不携带有限数（NaN + 非空原因） | — |
| 11 | `accepted` 采样种类按名拒绝；`DRIVABLE_KINDS` 不含它 | — |
| 12 | 切线数 ≠ 选中样本数 / 问题 id 重复 / 模式矩阵列数不符 / 无模式 / 空问题表 —— 各按名拒绝 | — |
| 13 | 两问报同一测量名时两条序列**保持分离**（不合并列） | — |
| 14 | `provenance` 记录边界与 model/plan 指纹；冻结协议进入缓存键 | — |
| 15 | 宽度无法由"模式数 × 分量数"解释的序列被拒绝 | 复审 |
| 16 | **全部模式被拒 + 2 个测量仍能成序列**（宽度取自请求） | 复审 1a |
| 17 | **被拒样本与已答样本共享同一宽度**（否则 `np.stack` 失败） | 复审 1b |
| 18 | **具名测量缺 `G` 被拒绝**，不把机构运动当测量发布 | 复审 2 |
| 19 | 调用方 `mode_matrix` 真正被求解（×3 幅值 ⇒ ×3 响应） | 复审 3 |
| 20 | **一个模式可答两个测量**（设计 §6.7 的 Wheel Rate 形状） | 复审 4 |
| 21 | 逐模式拒绝使 `answered=False`，即使切线审计为 `valid` | 复审 5 |
| 22 | 含保留分隔符的问题 id 被拒绝（键必须可逆） | 复审 |
| 23 | 非 `(gradient, load_gradient)` 对的测量项按名拒绝，不逃逸为 `IndexError` | 复审 |
| 24 | 已答样本不得携带 NaN（反向不变式） | 复审 |
| 25 | 已发布数组不可写；字段存的是**转换后**的数组而非调用方原件 | 复审 |

### 3.1 三个由探针当场复现的缺陷（修复前后对照）

命令：`uv run --no-sync python %TEMP%\tmp_verify_fixes.py`（临时探针，用后回收）。

| 缺陷 | 修复前 | 修复后 |
|---|---|---|
| 1a 全部模式被拒 + 2 测量 | `IndexError: index 1 is out of bounds for axis 0 with size 1` | `values=[nan] valid=[False]` |
| 1b 拒绝样本与已答样本混排 | `ValueError: all input arrays must have the same shape` | `shape=(1,2) valid=[False]` |
| 2 测量 `gradient=None` | `'none\x1fa' = [[0.5]] valid=[True]`（机构坐标冒充测量） | `CharacteristicsError: measurement 'm' has no gradient; …` |
| 3 `mode_matrix` 装饰性 | 单位基与 ×3 幅值给出**同一**答案 | `[0.5, 0.0]` vs `[1.5, 0.0]` —— 幅值被真正使用 |
| 4 一模式两测量 | `CharacteristicsError`（限制行数 ∈ {1, 模式数}） | `{'h_left': [0.5], 'h_right': [0.0]}`，`factorization_count == 1` |
| 5 `answered` 读旧状态串 | 恒 `True`（`"underdetermined"` 从不出现） | `answered=[False]`，与 `refused_modes` 一致 |

**1a/1b 的根因是同一个**：响应块的行宽原先**从"哪个模式答出来了"反推**，
于是"全部拒绝"退化为 `(1, n_modes)`，而"拒绝"与"已答"两种样本宽度不同。
修复是把行宽改成**请求的属性**（有测量 ⇒ 测量数；无测量 ⇒ 切线维度），
由 `_answer_rows()` 计算。这与 §2.2 表末行同源：
**形状必须来自请求，不能来自答案。**

**缺陷 2 是本模块第三个"合法输入 ⇒ 静默错误有限值"**。前两个是
`_stack_load_terms` 因兄弟测量缺 `D` 而丢弃整块（使 `total` 读成 0.5 而非 5.5）、
以及 `audited_modes` 的逐模式拒绝缺失。三者的形态完全一致，这构成本模块的
主要失效模式，也是 §5 突变表的组织依据。

**缺陷 4 曾经拒绝设计自己的规范请求**：设计 §6.7:447 要求"一个同相模式，
一次求解，同时测 `h_left` 与 `h_right`"。原实现要求 `measurement_gradient` 行数
∈ {1, 模式数}，把**测量数**与**模式数**混为一谈；而 `G` 是 `(测量, ndof)`、
`X` 是 `(ndof, 模式)`，`G X` 对任意测量数都有定义。两个计数彼此独立，限制已解除。

## 4. D2 复审结论与处置

### 4.1 首轮复审（已处置）

D1–D7 七处缺陷全部修复：逐模式判定被丢弃、被拒模式当已答、
模式状态串误读、`usable = any(per_mode)`、单测量的 `D` 被丢、
兄弟缺失 `D` 丢弃整块、测量行按插入序堆叠。
另发现 `SampleAnswer.sample_index` 被写成循环位置（应为存储索引）。

### 4.2 第二轮独立对抗性复审（本轮处置）

复审者明确说明**无法运行代码**，因此"异常类型"与"`P` 行基约定"两类结论
由我**用探针实测**：1a/1b/2 三处当场复现（§3.1），确认不是误报。

| 严重度 | 发现 | 处置 |
|---|---|---|
| 严重 | 1a 全拒 + 多测量 ⇒ `IndexError` | **已修**（`_answer_rows`） |
| 严重 | 1b 拒绝/已答混排 ⇒ `ValueError` | **已修**（同一根因） |
| 严重 | 2 缺 `G` ⇒ 机构坐标冒充测量 | **已修**（`SampleQuestion` 拒绝） |
| 严重 | 3 `mode_matrix` 装饰性；`LoadMode.point` 从不被读 | **已修**（`mode_matrix` 贯通） |
| 严重 | 4 一模式两测量被拒（设计规范请求） | **已修**（解除行数限制） |
| 中 | 5 `SampleAnswer.answered` 测不存在的 `"underdetermined"` | **已修** |
| 中 | 8 序列数组未冻结、字段存调用方原件 | **已修** |
| 中 | 问题 id 含分隔符 ⇒ 键不可逆 | **已修**（请求期拒绝） |
| 中 | 测量项形状错误逃逸为 `IndexError` | **已修**（请求期拒绝） |
| 低 | 11 单位未随测量走到答案 | **部分**：`SampleQuestion.measurement_units` 已贯通；多测量单位不一致时返回空串（见 §8-G3） |
| 低 | 10/12/13/14 报告与结构的其余校验 | **部分**（见 §8） |

### 4.3 复审者**核实为正确**的部分（未被我的修复改动）

- 切线**按样本**取（非按"样本×问题"）与 `sample_index` 缓存键；
- 请求形状校验（重复 id、空问题表、`accepted`、切线数不符、模式矩阵列数）各按名抛；
- `_row` 的**模式主序**展平；
- `statuses_for` 关闭了"`valid=True` 却全 NaN"的门。

### 4.4 复审者自己指出的方法学局限（必须保留）

> **全局审计不能替代逐模式审计**（`equilibrium_response` vs `audited_modes`）；
> **要猎的是"看起来合理的有限数"，不是崩溃。**

本步的 6 处缺陷有 5 处符合第二条。这条教训在本项目已被独立确认两次。

## 5. 蓄意破坏验证（mutation testing）

命令：
```
uv run --no-sync python %TEMP%\tmp_persample_sabotage.py
```
结果：**caught 32/32, survived 0, skipped 0**，
`restore verified: True`、`sha256 match: True`、`baseline after restore: exit 0`。

| # | 突变 | 结果 |
|---|---|---|
| 1 | 逐模式判定被丢弃，全模式改标 `valid` | CAUGHT |
| 2 | 被拒模式当已答 | CAUGHT |
| 3 | 模式状态串误读为从不出现的值 | CAUGHT |
| 4 | 任一模答出即视为样本可用 | CAUGHT |
| 5 | 单测量的直接项被当"缺失"丢弃 | CAUGHT |
| 6 | 兄弟测量缺 `D` 丢弃整块 | CAUGHT |
| 7 | 测量行按插入序堆叠 | CAUGHT |
| 8 | 载荷项行按插入序堆叠 | CAUGHT |
| 9 | 结构性命中计为"未分解" | CAUGHT |
| 10 | 键分隔符改回 `:` | CAUGHT |
| 11 | 切线按问题重复取（共享丢失） | CAUGHT |
| 12 | 分解计数按问题记录 | CAUGHT |
| 13 | `accepted` 采样种类放行 | CAUGHT |
| 14 | 切线数不足被接受 | CAUGHT |
| 15 | 重复问题 id 被允许 | CAUGHT |
| 16 | `provenance` 丢掉边界 profile | CAUGHT |
| 17 | `provenance` 丢掉 model 指纹 | CAUGHT |
| 18 | 运动响应被截断为第一分量 | CAUGHT |
| 19 | 序列宽度检查被删除 | CAUGHT |
| 20 | 容器层允许被拒项携带有限值 | CAUGHT |
| 21 | 被拒列保留响应返回的任何内容 | CAUGHT |
| 22 | 歧义裸名解析到第一个匹配 | CAUGHT |
| 23 | identity 在读取前按位置重建 | CAUGHT |
| 24 | **响应宽度再次从答案反推** | CAUGHT |
| 25 | **具名测量缺 `G` 再次被接受** | CAUGHT |
| 26 | **调用方 `mode_matrix` 被忽略、改从模式反推** | CAUGHT |
| 27 | **`answered` 丢弃逐模式拒绝** | CAUGHT |
| 28 | 问题 id 分隔符守卫被删除 | CAUGHT |
| 29 | 形状错误的测量项被放行 | CAUGHT |
| 30 | 已答样本携带 NaN 被允许 | CAUGHT |
| 31 | 已发布数组以可写形式交出 | CAUGHT |
| 32 | 校验转换后的局部量、却存调用方原件 | CAUGHT |

**等价突变清单（`EQUIVALENT`）：空。**
上一版登记的"被拒列保留响应返回的任何内容"经**重新测量**已**不再等价**——
新的逐模式逻辑使该守卫承载语义，故从等价清单移入突变表（第 21 行），
这正是"等价声明必须随代码变化重测"的例子。

**第 32 行是本次新补的断言缺口**：首轮运行它以 SURVIVED 出现，
原因是所有用例都传 `np.ndarray`，而 `np.asarray(ndarray)` 返回同一对象，
"存原件"与"存转换量"不可区分。补上"传 list"的用例（断言字段是 `ndarray`、
`dtype` 已归一）后转为 CAUGHT。**这证明幸存突变确实指向断言缺口，而不是噪声。**

## 6. 命令与退出码

| 命令 | 结果 |
|---|---|
| `uv run --no-sync ruff check .` | **All checks passed!** (exit 0) |
| `uv run --no-sync ty check .` | **All checks passed!** (exit 0) |
| `legacy_surface_gate.py --check` | `OK: no unregistered Python boundary violation` (exit 0) |
| `check_module_layering.py --strict --final` | `module cycles (SCC size>1) : 0` + `OK: layering matches the recorded baseline` (exit 0) |
| `check_composable_release.py --skip-isolation` | `OK: 3 release checks passed` (exit 0) |
| `pytest tests/characteristics/test_per_sample_sequences.py` | **46 passed** (exit 0) |
| `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests` | **178 passed** (exit 0) |
| 快速集（忽略 adams/architecture/cases） | **2004 passed, 1 xfailed** (exit 0) |
| `dynamic_hash_sentinel.py --check` | `OK: dynamic output matches the frozen baseline byte-for-byte`，**exit 0** |
| 蓄意破坏验证 | caught 32/32, survived 0, skipped 0 |

### 6.1 门禁口径说明（避免误读）

- **快速集基线由 1993 升到 2004**，增量恰为本步新增的 11 个回归用例，
  **不是**产品行为变化；内核+契约保持 **178 passed** 不变。
- `dynamic_hash_sentinel` 输出中的
  `solver self-convergence: FAILED`、`adams accuracy: BLOCKED`、
  逐用例 `FAILED` 行，是 **Adams 2024.1 未安装导致的既有报告**，
  与本次改动无关；判据是末行 `OK` 与 **exit 0**（逐位一致）。
- 本轮**未重录任何冻结基线**；`layering_baseline.json` 未改动。

## 7. 一致性检查

| 检查 | 结果 |
|---|---|
| 新增分层边 | **0**（本步全在 Python 结果层，未触内核分层） |
| `layering_baseline.json` 大小 | 未变（167364 B） |
| 冻结数值基线 | 未重录；`dynamic_hash_sentinel --check` 逐位一致 |
| skip/xfail 增长 | 无（1 xfailed 与基线同） |
| 模块循环（SCC>1） | 0 |
| 遗留缺口 | 15 项（沿用步骤 3 登记，见 `PROGRESS.md`） |
| 临时文件 | 探针与突变脚本在 `%TEMP%`，用后回收（§9） |

## 8. 登记缺口（如实）

**G1（阻塞 ABI 接线）**：本步的 `load_jacobian`/`mode_matrix` 仍由**调用方**提供；
驱动层尚未从 `frozen_load_column` + `assemble_physical_tangent` **构造**它们，
也**没有**"具名轮端模式 → `body_wrench`"的映射层。故本步证据只覆盖"给定请求如何被答"，
**不覆盖**"请求如何从原生冻结态产生"。这是步骤 5/6 与 ABI 接线的前置。

**G2（未交付的能力声明）**：`characteristics_response_kinds` 仍是**刻意空数组**
（`kernel_contract_run.cpp:229-243`）。按本仓库"读请求与答请求是两种能力"的约定，
实现应答**必须同时**把该类型加入声明；本步在 Python 层实现，未动 ABI，故未加。

**G3（单位仅部分贯通）**：`SampleQuestion.measurement_units` 已把逐测量单位带入请求；
当**多个测量的单位不一致**时，`_units_of` 返回空串而非选一个。
理由：把"m"印在角度上比留空更糟；但**逐测量的单位目前没有出现在序列上**
（`SampleSequence.units` 是单串）。彻底修复需要序列携带逐列单位。

**G4（复审者提出、我本轮未修）**：`load_mode_matrix` 硬编码 `load_dimension=6`，
因此 `compliance.heave_mode()` 返回的 2 分量 `p=[left,right]` **不能**用于本路径
（`directions.shape[0]=6 != applied.shape[1]=2`）。步骤 4 的调用方传 `(6, n_modes)` 绕开了它，
但"轮端基 → 六分量基"的转换层仍缺（与 G1 同源）。

**G5**：`ModeResult`/`"underdetermined"` 残留在 `SampleAnswer.answered` 已清除；
但 `results/equilibrium.py` 内其他位置是否还有同类"测一个从不出现的状态串"，
**本步未做全层排查**（属 P6 退役收口）。

**G6（方法学局限，如实保留）**：复审者**无法运行代码**，其结论中"异常类型"与
"`P` 行基约定"两类由我探针实测确认。其余读推导结论（如 `LoadMode.point` 从不被读）
我已按代码核实，但**未**为每条都构造独立探针。

**G7**：P2 步骤 1 与 P3 步骤 1 至今**没有专门 D2 复审**（沿用步骤 3 登记）。

**G8**：`trajectory.py` 与 `differences.py` 仍**无生产消费者**；
本步也没有接上它们（本步指标序列只覆盖响应与宽度，见 G3）。

## 9. 清理

| 文件 | 处置 |
|---|---|
| `%TEMP%\tmp_verify_fixes.py` | 探针，用后回收 |
| `%TEMP%\tmp_persample_sabotage.py` | 突变脚本，用后回收 |
| `%TEMP%\tmp_fix_sep.py` / `tmp_fix_convention.py` / `tmp_fix_inline.py` / `tmp_fix_direct.py` / `tmp_patch_probe.py` | 上一轮一次性补丁脚本，回收 |
| `%TEMP%\tmp_probe_review.py` | 上一轮探针，回收 |
| `backups/per_sample.py.p4a-step4-d2round2.bak` | **保留**（改动前备份） |
| `backups/test_per_sample_sequences.py.p4a-step4-d2round2.bak` | **保留** |
