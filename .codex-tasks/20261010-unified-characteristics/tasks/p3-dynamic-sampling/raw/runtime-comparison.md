# P3 步骤 5：状态不变性与性能预算

对应 SPEC §48 step 5、§56。证据落点：本文件。

**结论先行。** 步骤 5 的核心问题（"开启输出请求会不会改动原积分/事件/步序？"）在本机
rig 上**得到了否定但分层的答案**：

| 主张 | 实测结论 | 证据 |
|---|---|---|
| 无请求基线不变 | **成立（精确）** | `test_the_baseline_is_unchanged_when_nothing_is_requested` |
| 端点状态不随请求改变 | **成立（逐位）** | `test_the_requested_output_grid_does_not_move_the_endpoint` |
| 内部共享时刻状态不变 | **仅到舍入误差（1–4 ULP）** | `test_the_whole_trajectory_is_compared_not_only_its_end` |
| 接受步数不随请求改变 | **不成立** | `test_the_accepted_step_count_is_not_invariant_to_the_output_request` |
| 不静默抽样 | **成立** | `test_every_requested_sample_is_stored` 等 3 条 |
| 性能预算可读 | **成立，但通道与预期不同** | `test_the_ledger_is_the_authoritative_step_count` |

新增专项断言 16 条，落在
`packages/suspension_multibody/tests/characteristics/test_runtime_invariance.py`。

> **一句话结论**：§56 里"原积分"成立、"事件"本 rig 无事件可验、"步序"**不成立**。
> 该不成立项属 native 采样循环，本步如实登记并界定归属，未修改、未弱化、未掩盖。

---

## 1. 一个必须分层的结论：不是"不变"，也不是"变了"

SPEC §56 的措辞是"开启功能也不改变原积分/事件/步序"。实测把它**劈成了两类**：

### 1.1 物理量：可变网格下逐位/近逐位一致

同一 rig、同一初值，只改请求的输出网格：

```
网格        接受步数    存储状态数
2 点           44           2
3 点(a)        46           3
3 点(b)        46           3
5 点           46           5
9 点           50           9
```

`z(t) = 0.25 t` 是解析解（单刚体 + prismatic + 线性 prescribed motion + 常初速）。
各网格对解析解的最大偏差：

| 网格 | max\|z − 0.25t\| | 该量级的 ULP |
|---|---|---|
| 2 点 | 8.674e-19 | 1.0 |
| 5 点 | 2.602e-18 | 3.0 |
| 9 点 | 8.674e-19 | 1.0 |

**端点逐位相同**，但**内部共享时刻不是**。实测 `t = end/2`：

```
dense → 0.0025000000000000005
mid   → 0.0025000000000000014
```

差 1–4 ULP。这不是物理差异，是步点放置不同导致舍入累积路径不同。**因此步骤 5
不能声称"状态逐位不变"**——这正是本节存在的意义。

### 1.2 步数：确实随请求改变，且这是 placement 效应

接受步数 44 / 46 / 46 / 46 / 50 并非单调，也不是比例关系（9 倍采样点只多 13.6% 步数）。

**成因（读 native 源码得出，不是本测试证明的）：**

- `kernel_abi.cpp:347` 外层循环**按 sample 迭代**，每个 sample 是独立 target；
- `kernel_abi.cpp:382` `while(t < target - time_tolerance)`，步长
  `h = min(internal_step, remaining)` 被 `remaining` 截断；
- `kernel_input.cpp:169-184` `next_prescribed_input_breakpoint` 返回相邻 knot，
  `h` 再被它截断。

即：**请求的输出网格就是积分步长的硬上界**。请求点即同步点，密请求自然多切步。
这是采样架构的后果，不是物理不一致。

### 1.3 这不是 P3 引入的

P3 只新增纯读侧模块（`results/sampling.py`、`results/trajectory.py`、
`results/differences.py`），三者都不在 `simulate` 路径上。该结论用
**import 图可达性**证明（`test_the_p3_read_layers_are_not_reachable_from_simulate`），
并经 sabotage 反证有效（见 §4）。

> 两次被否决的更弱检查，记录以免重蹈：
> 1. **在 `api.py` + `compilation/resolved.py` 里做子串搜索**——真正的路径还经过
>    `simulation/runner.py` 与 `results/envelope.py`，这两个文件从未被搜；函数体内的
>    惰性 import 也完全逃逸。这是初版实现，已替换。
> 2. **运行后查 `sys.modules`**——**实测无效**：`results/__init__.py` 无条件 import
>    了这三个模块，它们永远在 `sys.modules` 里，与谁调用无关。

---

## 2. 性能预算：通道与预期不同

`raw.performance` 在本路径上**完全未填充**：

```
available = False
accepted_steps = 0
rejected_attempts = 0
newton_iterations = 0
```

而同一份结果的 diagnostics ledger 第一列求和是 **50**。也就是说预算数字只能从
ledger 取，不能从聚合 performance 取。`sampling.py` 的 `_accepted_step_total` 正是
直接读 ledger——这一点在步骤 2 就已确立，本步把它钉进断言。

**两条 ledger 视图的差异（sabotage 才发现）：**

- `raw.block("diagnostics")` → 原始块，**不受** `_DIAGNOSTIC_TAIL_ROWS` 影响；
- `raw.diagnostics`（property）→ 经 `_diagnostic_samples` 应用 tail 行算术。

只断言前者，把 `_DIAGNOSTIC_TAIL_ROWS` 改成 0 **测试照样通过**（已实测）。
故断言两条视图。

**范围限制（同样由实测确立，不夸大）：** 单 case 时该常数**不可观测**——
`_diagnostic_samples` 逐 case 切 `[offset : offset+count]`，单 case、offset=0 时
两种常数都给 `stored` 行。要覆盖它必须用多 case 计划，即步骤 2 抓到裸 ledger 偏移
缺陷的同一手法。**本文件不声称覆盖该常数。**

---

## 3. 不静默抽样

三条断言：

1. 存储状态数 == 请求采样点数（2 / 3 / 5 / 9 全对得上）；
2. 回读的 times 与请求网格逐位一致；
3. 状态是完整 19 列（`[position(3)|quaternion(4)|velocity(3)|omega(3)|额外 6]`），
   不是只留位姿的瘦身版——并且**用内容校验列位**（初速落在 7:10、角速度落在
   10:13 且为 0、四元数 3:7 单位长），不是只数长度。

`test_a_differenced_series_keeps_one_row_per_sample` 另证差分层也不抽稀：9 个样本
进、9 行出，不可用速率是原位报告而非丢行。

> 初版此处写 `state.shape[1] >= 13`，后被 review 判为半空断言并改为 `== 19`：
> 19→13 的退化恰好是本测试要抓的能力损失，用 `>=` 会静默放过。

---

## 4. Sabotage 结果（3 条，证明断言非空）

| 突变 | 目标测试 | 是否抓到 |
|---|---|---|
| 往 `results/envelope.py` 注入 `from .sampling import select_samples` | reachability | **抓到** |
| `body_state` 读时丢掉最后一个采样 | 不静默抽样 | **抓到** |
| `_DIAGNOSTIC_TAIL_ROWS` 2 → 0 | ledger 视图 | **未抓到（单 case 下等价）** |

第三条未抓到**不是 harness 陈旧**，是语义等价：单 case 下该常数不影响任何被断言
的形状（§2 已解释）。故不把它算作"断言失效"，而是记入 §2 的范围限制。harness 在
每次突变后按 SHA256 校验还原，三次 `restored: True`。

---

## 5. 独立复审的处置

本轮 D2 只读复审提出 8 项。逐项核验后处置：

| # | 复审意见 | 我的核验 | 处置 |
|---|---|---|---|
| 1 | 子串检查覆盖不到真正路径 | **成立** | 换 import 图可达性并加 sabotage 反证 |
| 2 | `assert ratio > 1.0` 近恒真 | **成立** | 改为对字面量 50/9 断言 |
| 3 | `< 2.0` 断言弱 | **成立** | 改为对上界 88 的显式断言 |
| 4 | `>= 13` 半空 | **成立** | 改 `== 19` |
| 5 | `+ 2` 应泛化为 `2 * cases` | **成立** | 已改 |
| 6 | 因果解释缺证据 | **成立** | 改为引 native 行号，并明确标注非本测试所证 |
| 7 | §56 前半句"无请求基线不变"无测试 | **成立** | 新增基线测试 2 条 |
| 8 | 直线 fixture 上"逐位不变"平凡 | **成立且更强** | 见下 |

**复审第 8 项我独立验证后发现比复审说得更严重**：复审认为端点比对"平凡但正确"。
实测显示，**内部共享时刻根本不逐位相同**（1–4 ULP），所以初版
`test_the_requested_output_grid_does_not_move_the_trajectory` 的名字与断言
（只比端点）**同时**低估和高估了事实：名字声称"轨迹"不变（过强），
断言只比端点（过弱）。两者都已修正，且 §1.1 的 ULP 事实已写入模块 docstring。

**复审还漏了一条**（我自查发现）：`SPARSE` 与 `DENSE` 在 `(0, end)` 内**没有任何共享
时刻**，因此初版的"整条轨迹"测试实际只验证了 2 个端点、其中 `t=0` 还是初值。新增
`MID` 网格（与 DENSE 共享 3 个内部时刻）后才非空；该测试首次运行即失败暴露此点。

---

## 6. 验证命令与结果

| 命令 | 结果 |
|---|---|
| `uv run --no-sync pytest .../tests/characteristics -q` | **401 passed** |
| 快速集（排除 adams/architecture/cases） | **1958 passed, 1 xfailed**（skip 0，xfail 未增长） |
| `pytest packages/suspension_kernel/tests packages/suspension_contracts/tests` | **176 passed** |
| `ruff check .` | 0 |
| `ty check .` | 0 |
| `legacy_surface_gate.py --check` | 0 |
| `check_module_layering.py --strict --final` | 0 |
| `check_composable_release.py --skip-isolation` | 0 |
| `git diff --check` | 干净 |

未触及求解路径（纯 Python 读侧 + 测试），故本步**未跑** `gate-numeric`；步骤 6
（R8 迁移）需要 native 重建，届时必跑。

未重录任何冻结基线。

---

## 7. 遗留与移交

1. **步数不随请求不变是真实缺陷面，但不在 P3 可达范围内。** 位于 native 采样循环
   （请求点即步长上界）。P3 是读侧任务，无权也无需修改。**若全局口径要求严格步序不
   变，应单独立项**，且需先与 §39"超全接受步预算报告，不降采样"的口径对齐。
2. **无事件场景未覆盖。** 本 rig 无接触事件（`contact_events` 为空、rejected 恒 0），
   故 §56 的"事件不变"与"步序不变"中"事件"一半**未被本步证据覆盖**。步骤 4 已记录
   "从事件时间映射到样本下标尚未实现"，两者是同一缺口。
3. **多 case 计划下的 tail 行常数未覆盖**（§2）。步骤 2 已证明该缺陷面存在。
4. **`raw.performance` 全零**是个独立可改进点：调用方若直接信任它会budget 成 0。
   当前 `sampling.py` 绕开了它，正确；但该陷阱未在任何地方拦截。建议后续在
   `performance["available"] is False` 时给出显式说明，而非静默返回 0。
