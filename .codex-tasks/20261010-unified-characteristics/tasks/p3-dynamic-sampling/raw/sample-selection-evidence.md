# P3 步骤 2 证据：三种采样选择与独立身份表

> SPEC §45（步骤 2）：接入三种采样选择与独立身份表；initial/accepted/output/indices
> 可追溯，拒绝步与不同网格不混淆。落点：`tests/characteristics/test_sample_selection.py`。
> 本步**不修改 native/C++**：步骤 1 的审计已证明逐接受步状态不存在，本步的职责是把
> 这个事实变成**可追溯的选择语义**，而不是伪造数据。

## 0. 结论

| 选择 | 状态 | 依据 |
|---|---|---|
| `output` | **可回答**，映射现有 `body_state` 行 | 采样器写的正是输出网格 |
| `initial` | **可回答**，每个 case 的首个存储样本 | 一致初始点在索引 0 处已写出 |
| `accepted` | **如实拒绝** | 逐接受步状态未持久化（步骤 1 实测） |

**实测的比例**：一次真实动态运行 **存储 21 个状态、接受 406 步**（≈20:1）。
把输出网格冒充 `accepted` 会把 21 个点标成 406 步，且每个下游速率都会变成
"每输出区间的速率"却报成"每接受步的速率"——这正是 SPEC §37/§38 禁止的。

## 1. 实测：接受步 vs 存储样本

用 `tests/analysis/test_axle_dynamic.py` 的同一 fixture 形态（`axle_dynamic` +
`provided_consistent_state`）跑三种网格：

| 输出样本数 | 端时 | 每样本 accepted_steps | 总接受步 | 存储状态 | 比例 |
|---|---|---|---|---|---|
| 3 | 0.02 | `[0, 24, 22]` | 46 | 3 | 15.3:1 |
| 9 | 0.08 | `[0, 24, 22, 20×6]` | 166 | 9 | 18.4:1 |
| 21 | 0.2 | `[0, 24, 22, 20×18]` | 406 | 21 | 19.3:1 |

**读法**：`accepted_steps` 是每个输出样本区间内累积的接受步**计数**（diagnostics 列 1），
不是状态。区间内被接受的步骤除最后一个外，其状态在写出时已被覆盖。
样本 0 恒为 0，因为它是初始一致点，区间尚未开始。

**索引 0 = initial 得到实测支持**：`kernel_abi.cpp:305` 在循环外写
`copy_state(state, model, output->body_state, 0)`，即起始一致状态。

## 2. 交付物

| 文件 | 内容 |
|---|---|
| `src/suspension_multibody/results/sampling.py` | **新增**，`SAMPLE_KINDS`/`RefusedSample`/`SampleSelection`/`select_samples`/`require_selection` |
| `src/suspension_multibody/characteristics/values.py` | `SampleIdentity` 新增 `sample_kind` 字段 + 校验；新增 `SAMPLE_KINDS` 单一真源 |
| `src/suspension_multibody/results/__init__.py` | 导出 |
| `tests/characteristics/test_sample_selection.py` | **新增**，19 个用例 |

**单一真源**：`SAMPLE_KINDS` 定义在 `values.py`，`sampling.py` 导入并转发，
`results/__init__.py` 从 `sampling` 导入。全仓库只有一份定义——
拒绝消息里的名字与校验用的名字不可能不一致。

## 3. 五个设计判断

### 3.1 `accepted` 为什么是拒绝而不是近似

近似会让**错标的数看起来是对的**。两种拒绝理由中，本文档采用可辨识性：
逐接受步状态根本不在任何输出数组里，且集合**无法枚举**（只有计数，没有身份）。
SPEC §84 已预警此点（"不能以全点 inconsistent_sample 结束任务"），
本步的答复是：该能力需要 native 侧新增持久化，属步骤 2 之外的范围，
故**如实拒绝**并给出定量短缺，而不是换一个集合顶上。

拒绝消息带**两个实际数字**（存储数、接受步数），因为笼统的"不支持"会诱导调用方
假设缺口很小或可以替代。400 与 21 摆在同一条消息里，替代方案就不成立。

### 3.2 `sample_kind` 为什么必须落在身份上

"第 0 个样本"在两种选择下是同一条记录（`index=0`）。没有 `sample_kind` 时，
消费方无法判断一个速率是取在一致点上还是取在采样器第一个输出上——两者都合理，
所以错了也不报错。`accepted_step_id` 已被声明但**全仓库无生产者**（实测：只在
`values.py` 定义处出现，无任何写入点），故它不能被当作接受步身份使用。

### 3.3 边界：拒绝步的身份

SPEC §54 的"拒绝试探步"场景需要身份。现有 diagnostics 只有标量
`rejected_attempts`（列 2），没有逐条记录，故拒绝步同样是**不可枚举**的。
本步**不声称**已覆盖该场景；已测得三次运行 `rejected_attempts` 全为 0，
因此现有 fixture 无法构造该场景，登记为缺口而非"已测试"。

## 4. 独立复审（D2）发现的缺陷与修复

第一轮实现交付后做只读对抗性复审，发现 **3 个真实缺陷**，全部已修并加回归。

### D-1（严重）ledger 行偏移漏加每 case 的尾部行

`_accepted_step_total` 用裸 `sample_offset` 读 diagnostics。但 ledger 的布局是
**每个 case 的两行尾部记录插在该 case 的样本之后**，不是全部堆在末尾。
C++ 写入端三处互证：

| 位置 | 内容 |
|---|---|
| `kernel_contract_run.cpp:856` | 基址 = `sample_offset*W + 2*cases_so_far*W` |
| `kernel_contract_run.cpp:910` | `base = sample_offset + 2 * case_spans.size()` |
| `kernel_contract_run.cpp:957` | `sample_offset += run.sample_count`（**只按样本数累加**） |

`raw.py:70`/`:81`/`:240` 三处既有读取也都显式加 `_DIAGNOSTIC_TAIL_ROWS * case_index`。
**我的代码与仓库既有惯例直接矛盾**。后果：多 case 时把前一个 case 的**尾部计数器
（线性求解数等）当成接受步加进去**——有限、可信、错误。修复：
`_ledger_rows_for_case(case_index, offset, count)` 集中做偏移，所有 ledger 读取共用。

**单 case 恰好正确**（`2*0=0`），而原测试只测单 case，所以这个缺陷**从未被看见**。

### D-2（中）`initial` 依赖两个独立 offset 来源恰好一致

原实现用 `sample_offset` 去匹配身份的 `index`。`sample_offset` 是写入端的**累计**
字段，身份的 `index` 来自其生产者的布局——两者相等只是当前巧合。一个
`sample_offset` 为 case 相对的合法 artifact 会走到"no stored identity matches"，
报出一个令人困惑的错，而不是给出初始点。修复：改为"每个 case 的第一个身份"，
不再做跨来源匹配。

### D-3（中）`refused` 的元组语义可颠倒且拒绝分支无覆盖

`refused: tuple[tuple[int, str], ...]` 的两个 int 无法区分 case 与 sample，
而 `require_selection` 用 `enumerate` 产生的位置当 sample 号——**位置不是身份**。
且 `refused` 无生产者，`require_selection` 的拒绝分支是防御性代码，
原测试只断言 dataclass 属性、**从未调用被测函数**。修复：改为具名
`RefusedSample(case_index, sample_index, reason)`；消息按名取值；
测试改为驱动真实的拒绝路径，并在 docstring 里如实标注该分支当前为防御性。

## 5. 破坏验证：3/3 全部被捕获

自设破坏（`tmp_sabotage_p3.py`，已移出仓库）在原始模块上逐个植入并跑测试，
模块从内存副本逐字节恢复并 SHA256 校验：

| 破坏 | 判定 | 说明 |
|---|---|---|
| ledger 行偏移忽略尾部行（D-1 本体） | **CAUGHT** | 回归用例直接命中 |
| ledger 求和整行而非 accepted 列 | **CAUGHT** | 见下 |
| `np.nansum` → `np.sum` | **CAUGHT** | 见下 |

**后两项第一轮是 MISSED，且是两件不同的事**——这个区分很重要：

- **整行求和**最初 MISSED，因为 fixture 除第 1 列全是 0，两种读法结果相同。
  我去测了**真实 ledger**：样本行的第 2..15 列**确实有非零值**
  （实测列 4/5/6/7/9/12/13 非零，如 `0.000625`、`0.0015`、`-0.000671`）。
  故这是**真实缺陷**，不是等价变异。修法是给 fixture 的第 2..15 列填噪声，
  现在能区分两种读法。**教训：fixture 若比真实数据"干净"，就测不出真实缺陷。**
- **`nansum` → `sum`** 最初 MISSED，因为我把 NaN 放在了尾部行（已入选区之外），
  选区里根本没有 NaN。修法是把 NaN 放进选区内部——那正是失败运行的形状
  （已写样本之间夹一个未写样本行）。这是**测试没测到该测的东西**，不是等价变异。

两个都不是"等价变异"，与 P2 记录的三个等价变异不同：这两个都是**测试的缺陷**，
修的是测试。

## 6. 验证结果（全部退出码 0）

| 检查 | 结果 |
|---|---|
| `test_sample_selection.py` | **19 passed** |
| characteristics 全目录 | **325 passed**（原 306，+19） |
| 快速集 | **1882 passed / 1 xfailed**（原 1863；skip 0、xfail 未增长） |
| kernel + contracts | **176 passed** |
| `ruff check .` / `ty check .` | 0 / 0 |
| 架构门：legacy / layering / release | 0 / 0 / 0 |
| `test_no_legacy_vehicle_models.py` | 5 passed |
| 破坏验证 | **caught 3/3**，`module restored: clean`，`final baseline: PASS` |
| `git diff --check` | 干净（仅 LF/CRLF 警告） |
| 冻结基线 | **未重录任何** |

未跑 `gate-numeric`：本步只改 Python 读取层，未触及 native/求解路径，
不满足 AGENTS.md §2 的触发条件。步骤 6 会跑。

## 7. 缺口与不声称

- **`accepted` 与拒绝步身份仍未提供**：需 native 侧逐接受步持久化，属新能力，
  不在本步范围。这是 SPEC §84 点名的风险，本步以**如实拒绝**回应，未缩小它。
- **`require_selection` 的拒绝分支当前不可达**：无生产者产生部分选择。
  保留为防御性代码并在 docstring 与测试中如实标注，不记为"已覆盖的生产行为"。
- **`_FakeResult` 与真实 `RawContractResult` 有一处保真度差异**：真实的
  `times_s` 是按 case 截断后拼接（`raw.py:122-134`），fixture 直接给全网格。
  当前测试的时间点数量恰好等于拼接结果，故测不出差异；已在复审中登记。
- **多 case 的真实端到端运行未构造**：本步的多 case 用例基于手工 ledger 布局，
  其偏移约定由 C++ 三处互证 + `raw.py` 三处既有读取确认，非端到端复现。
