# p3-05 证据 (d)：测试同步与理由登记

## 1. 改动的测试文件

| 文件 | 状态 | 内容 |
|---|---|---|
| `packages/suspension_multibody/tests/metrics/test_outputs_match_legacy.py` | 修改（+73 行，0 删） | 新增独立的历史通道名单 `LEGACY_LOAD_CHANNELS`，新增 2 个用例；1 个既有用例的**注释**更新 |
| `packages/suspension_multibody/tests/metrics/test_placement_channels.py` | 新增 | 7 个用例：3 轴、改名实验、单轮、拒绝路径、4 轮算术与顺序 |
| `packages/suspension_multibody/tests/outputs/test_load_outputs.py` | 新增 | 5 个用例：声明层的名字/顺序/`reads`/改名实验/边界登记 |

`git diff --stat` 相关行：

```
 .../tests/metrics/test_outputs_match_legacy.py     |  73 +++++
```
（另两个文件是新增文件，不在 `git diff` 里；见 `git status`。）

## 2. 为什么 `test_outputs_match_legacy.py` 必须同步改（理由登记）

**改动前** `test_the_wheel_load_metrics_are_restated_value_for_value`（原 `:148-154`）：

```python
legacy = wheel_load_metrics(LOADS)
assert legacy, "the legacy reader should produce the aggregate keys"
for name, expected in legacy.items():
    _compare(name, values, expected)
```

这段本身**没有**被削弱：它迭代 `wheel_load_metrics(LOADS)` 的**全部**键，并要求每个键在 `BUILTIN.evaluate` 下得到同一个值。表从 11 个键变成 13 个键后，这条循环自动覆盖新增的两个 `normal_load_axle_*`——也就是说它仍然在跑、仍然是全覆盖，只是**覆盖面随表增长**，无法再单独回答「既有 11 个名字是否一个都没少」。

这正是兼容硬门（SPEC 判据 (b)）的关键问题：原用例的名字集合**来自被测函数自己**，函数少发一个名字它不会失败。所以补的是一条**独立的**名单对照，而不是改弱原断言。原用例只加了两行说明性注释（`:177-178`），断言一字未动。

## 3. 改动前后断言清单对照（未减弱）

### 3.1 既有用例：断言条数不减

| 用例 | 改动前断言 | 改动后断言 |
|---|---|---|
| `test_the_wheel_load_metrics_are_restated_value_for_value` | 1 条 `assert legacy` + 循环内 1 条 `_compare`（每个键一次，当时 11 次） | **完全相同**（现在 13 次，键多了） |
| `test_the_vehicle_metrics_are_restated_value_for_value` | 循环 10 个名字各 1 条 `_compare` | 完全相同 |
| 其余 10 个既有用例 | —— | 一字未改 |

没有任何 `assert` 被删除、放宽或用 `pytest.xfail`/`skip` 替代；本次改动**新增 0 个 skip、0 个 xfail**（`pytest -q` 输出里 `1182 passed, 1 xfailed`，那个 xfail 是既有的）。

### 3.2 新增用例：断言只增

`test_outputs_match_legacy.py` 新增：

| 用例 | 断言 |
|---|---|
| `test_the_wheel_load_metrics_still_publish_every_historical_channel` | `len(LEGACY_LOAD_CHANNELS) == 11`；11 个历史名字逐项值相等（独立名单，不取自被测函数）；两个新名与对应历史名同值；**整个键集合**等于「历史 11 + 新 2」 |
| `test_the_declared_load_outputs_are_exactly_the_report_channel_table` | 报表层键集（排序）== 声明层名字集（排序）；再逐项值相等 |

`test_placement_channels.py` 与 `test_load_outputs.py` 的断言见 `raw/three_axle_channels.md` 第 4 节。

## 4. 覆盖到的拒绝路径（原实现有、改后仍在）

| 场景 | 改动前 | 改动后 |
|---|---|---|
| 值非有限 | `ValueError("wheel loads must be finite")` | 同一条消息（`wheel_loads.py:78`） |
| 恰好四角之外 | `ValueError("wheel loads must contain exactly the four vehicle corners")` | **行为变化**：不再按角数拒绝（这是本任务的目的），改为「至少一个轮端 + 每个名字必须声明侧」。原消息在 `report/` 里不再出现；`outputs/builtin.py` 的 `_loads` 仍因只声明四角而在读不到输入时抛 `MissingOutputError` |
| 名字不合规 | 归入"角数不符" | `ValueError("wheel end 'axle_total' does not state a side; ...")`，按名报出 |

第 2 行是唯一的行为放宽，有测试覆盖：`test_a_name_that_states_no_side_is_refused_by_name`、`test_the_aggregates_follow_the_loads_rather_than_a_corner_count`。

## 5. 全跑结果

```
uv run --no-sync pytest packages/suspension_multibody/tests/metrics packages/suspension_multibody/tests/outputs -q -p no:cacheprovider
75 passed in 3.49s            (EXIT=0)
```

改动前该命令为 **61 passed**（开工前实测的基线），改动后 **75 passed**，+14 = 2（`test_outputs_match_legacy.py` 新增）+ 7（`test_placement_channels.py`）+ 5（`test_load_outputs.py`）；既有 61 个用例全部保留，无一替换或删除。
