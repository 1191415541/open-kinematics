# p3-05 证据 (a)：报表通道按安装角色动态注册

## 1. 生成代码位置（改造后）

真源：`packages/suspension_multibody/src/suspension_multibody/report/wheel_loads.py`（p3-05 唯一真源）

| 位置 | 符号 | 作用 |
|---|---|---|
| `wheel_loads.py:45` | `_SIDES = ("left", "right")` | 侧后缀，也是累加顺序 |
| `wheel_loads.py:48` | `wheel_end_side(wheel)` | 从轮端名读出侧；读不出即按名拒绝 |
| `wheel_loads.py:59` | `wheel_end_placement(wheel)` | 去掉 `_<side>` 后缀即放置名 |
| `wheel_loads.py:64` | `_entries(loads)` | 轮端表 → `placement -> {side: value}` |
| `wheel_loads.py:92` | `_axle_totals(table)` | 每个 placement 一个合计（左后右） |
| `wheel_loads.py:100` | `_side_totals(table)` | 每个侧一个合计（按 placement 顺序） |
| `wheel_loads.py:127` | `axle_loads(loads)` | 公开：placement → 合计 |
| `wheel_loads.py:132` | `side_loads(loads)` | 公开：side → 合计 |
| `wheel_loads.py:137` | `wheel_load_channels(loads)` | **发布通道表的唯一入口** |
| `wheel_loads.py:173` | `WheelLoadSummary` | 由 `axle_loads` / `side_loads` 支撑的汇总 |
| `wheel_loads.py:212` | `WheelLoadSummary.__getattr__` | `{placement}_axle`、`{side}_side`、`right_left_delta` 按实际放置解析 |
| `wheel_loads.py:255` | `summarize_wheel_loads(loads)` | 汇总入口；接受任意轮端数 |

通道名生成（`wheel_loads.py:145-170`，全部由 `placement` / `side` 驱动，**没有 front/rear 字面量**）：

```python
channels = {f"normal_load_{placement}_{side}": value ...}      # 每个轮端
channels["normal_load_total"] = _vehicle_total(axles)
for placement, total in axles.items():
    channels[f"normal_load_axle_{placement}"] = total          # 新增命名
    channels[f"normal_load_{placement}_axle"] = total          # 历史拼写，同一规则生成
for side, total in sides.items():
    channels[f"normal_load_{side}_side"] = total
if len(placements) > 1:
    channels[f"load_transfer_{placements[0]}_minus_{placements[-1]}"] = ...
if set(sides) == set(_SIDES):
    channels["load_transfer_right_minus_left"] = ...
```

`report/metrics/vehicle.py:28-30` 不再有通道表，`wheel_load_metrics` 只剩一句委派：

```python
def wheel_load_metrics(loads):
    """Return the wheel-load channel table for however many wheel ends there are."""
    return wheel_load_channels(loads)
```

## 2. 两处重复定义的真源归属

| 改动前 | 改动后 |
|---|---|
| `report/wheel_loads.py:17 _WHEELS` + `:26-31` 六个字段 + `:36-38` 恰好四角校验 | 该文件成为**唯一真源**：`wheel_load_channels` 生成通道表，`WheelLoadSummary` 由放置驱动 |
| `report/metrics/vehicle.py:18 _WHEELS` + `:21-43` 同一批 11 个字段，逐字重复 | 删除 `_WHEELS` 与整张表，改为 `from ..wheel_loads import wheel_load_channels` 并委派（`vehicle.py:23`、`:28-30`） |
| `outputs/builtin.py:334-359 load_scalar` + `:624-648 _load_outputs` 硬编码 11 个派生输出名 | **声明层**仍保留一份（原因见下），但改为从 `_WHEELS` 生成：`_load_channel_names()`（`:683`）+ `_load_channel_table()`（`:367`） |
| —— | 值级对照测试锁定两份不漂移：`tests/metrics/test_outputs_match_legacy.py::test_the_declared_load_outputs_are_exactly_the_report_channel_table` |

**为什么 `outputs/builtin.py` 不直接 import `report`**：`outputs/` 是「一次 run 产出了什么、从其中能导出什么」的层，`report/` 是**消费**这一层的层（`packages/suspension_multibody/src/suspension_multibody/outputs/__init__.py:3-9`）。派生输出的声明必须独立于任何报表模块，否则「报表从 run 的输出导出」这条边界就没有了。代价是一份受测试约束的重复，写在 `builtin.py:371-377` 的 docstring 里。

## 3. 生成结果清单（实测）

跑：见 `run_log.md` 的 C1/C2（`raw/channel_probe.py --emit` / `--compare`）。

### 3.1 4 轮（`front` / `rear` 两个放置）

| 通道 | 值（fixture 4 角 100/120/80/90） | 类别 |
|---|---|---|
| `normal_load_front_left` | `100.0` | 既有 |
| `normal_load_front_right` | `120.0` | 既有 |
| `normal_load_rear_left` | `80.0` | 既有 |
| `normal_load_rear_right` | `90.0` | 既有 |
| `normal_load_total` | `390.0` | 既有 |
| `normal_load_axle_front` | `220.0` | **新增** |
| `normal_load_front_axle` | `220.0` | 既有 |
| `normal_load_axle_rear` | `170.0` | **新增** |
| `normal_load_rear_axle` | `170.0` | 既有 |
| `normal_load_left_side` | `180.0` | 既有 |
| `normal_load_right_side` | `210.0` | 既有 |
| `load_transfer_front_minus_rear` | `50.0` | 既有 |
| `load_transfer_right_minus_left` | `30.0` | 既有 |

既有 11 个名字、顺序、值与改动前逐位一致（`raw/channel_probe_compare.txt`，`moved channels: 0`）。

### 3.2 3 轴（`front` / `middle` / `rear` 三个放置，6 轮端）

实测（`raw/four_wheel_after.json` 的 `assembly_three_axle`，装载 p3-04 三轴总成）：

```
normal_load_axle_front   5260177.728510661
normal_load_axle_middle  5260177.728510661
normal_load_axle_rear    5260177.728510661
normal_load_front_axle   5260177.728510661      (历史拼写，同一值)
normal_load_middle_axle  5260177.728510661      (历史拼写，同一规则生成)
normal_load_rear_axle    5260177.728510661
normal_load_left_side    7890266.592765992
normal_load_right_side   7890266.592765992
normal_load_total        15780533.185531983
load_transfer_front_minus_rear  0.0
load_transfer_right_minus_left  0.0
```

`WheelLoadSummary.axle_loads == {'front': ..., 'middle': ..., 'rear': ...}`，`summary.placements == ('front', 'middle', 'rear')`，`summary.middle_axle` 可读（原文见 `raw/three_axle_dump.txt` 的 `== three_axle` 段；`summary.single_axle` 则按名拒绝）。

### 3.3 单轮台架

```
normal_load_axle_single   12508758.86425533
normal_load_single_axle   12508758.86425533
normal_load_left_side     12508758.86425533
normal_load_total         12508758.86425533
```

无 `normal_load_right_side`、无 `load_transfer_right_minus_left`（该侧没有轮端，表里就没有这一项，不给 0）。`summary.right_side` / `right_left_delta` 抛 `AttributeError: this summary carries no 'right' side; it carries left`。

### 3.4 改动前的拒绝行为（对照）

改动前三轴与单轮都抛 `ValueError: wheel loads must contain exactly the four vehicle corners`（`raw/four_wheel_before.json`）。改造后（`raw/four_wheel_after.json`）两者都正常出表。"恰好四角"这条硬校验被替换为"至少一个轮端 + 每个名字必须声明侧 + 值有限"，错误消息仍含 `wheel loads must be finite`。

## 4. 累加顺序（4 轮逐位一致的原因）

- placement 合计 = 该 placement 的 `left + right`（`_axle_totals`，`wheel_loads.py:92-97`）；
- 侧合计 = 按 placement 顺序累加该侧的轮端（`_side_totals`，`:100-114`）；
- 整车合计 = 按 placement 顺序累加 placement 合计（`_vehicle_total`，`:117-119`）。

4 轮下即历史上的 `front+rear`、`fl+fr`、`fl+rl`，同样的数、同样的顺序。`tests/metrics/test_placement_channels.py::test_the_four_wheel_table_keeps_the_arithmetic_it_had` 把顺序写成了断言（比较对象是 `(100.0 + 120.0) + (80.0 + 90.0)` 这样的字面表达式，而不是等值数字）。

## 5. `report/` 分层

`wheel_loads.py` 只 import `collections.abc.Mapping`、`dataclasses`、`numpy`；`metrics/vehicle.py` 增加 `from ..wheel_loads import wheel_load_channels`。没有 native/kernel/solver/preparation 导入，也没有力律调用。门见 `run_log.md` V2（退出码 0，`findings: 0`）。

## 6. 未做（登记，不在本行写范围）

- `outputs/builtin.py` 的 `RIG_OUTPUTS`（`:220` 起）与 `minimum_unit_outputs`（`:1000` 附近）仍只声明四个 `wheel_load_<corner>` 输入。因此静态可声明的派生输出仍止于四角；三轴的 `normal_load_axle_middle` 在本行是**运行期**能力（`report/` 层已具备），**不是**声明层能力。该段属「非本行段」，按 SPEC 第 35 行回退裁决；已由 `tests/outputs/test_load_outputs.py::test_a_placement_the_declaration_does_not_declare_is_not_declared` 显式登记，供 p5-03 对接。
- `LEGACY_CLASSIFICATION` 中 `report.metrics.vehicle.wheel_load_metrics`（`builtin.py:828-834`）与 `_vehicle_dynamic_metrics`（`:772-777`）两行的通道名单仍列 11 个旧名，未含 `normal_load_axle_*`。该段属「分类表」段，不在本行授权的「派生输出声明」段内，故未改；差异已登记在 `PROGRESS.md`。
