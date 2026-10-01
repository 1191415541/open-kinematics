# p3-05 证据 (c)：3 轴情形生成按安装角色命名的三个通道

## 1. 判据

SPEC (c) 要求两件事，缺一不可：

1. 3 轴情形出现 `normal_load_axle_front` / `normal_load_axle_middle` / `normal_load_axle_rear`；
2. 一次**改名实验**（非 `front`/`middle`/`rear` 的放置名）证明生成逻辑由声明驱动，而不是把 `front/rear` 换成 `front/middle/rear` 三元组。

## 2. 3 轴实测

被测对象：p3-04 的三轴总成（`tests/physics/test_static_loads.py::_three_axle_assembly`，三个 `AxleEntry(placement=...)`：`front` / `middle` / `rear`，六个轮端），经 `compute_static_wheel_loads_for_assembly` 实解。

命令（见 `run_log.md` C3），原文落在 `raw/three_axle_dump.txt`：

```
== three_axle
  load front_left       2630088.8642553305
  load front_right      2630088.8642553305
  load middle_left      2630088.8642553305
  load middle_right     2630088.8642553305
  load rear_left        2630088.8642553305
  load rear_right       2630088.8642553305
  channels:
    normal_load_front_left           2630088.8642553305
    normal_load_front_right          2630088.8642553305
    normal_load_middle_left          2630088.8642553305
    normal_load_middle_right         2630088.8642553305
    normal_load_rear_left            2630088.8642553305
    normal_load_rear_right           2630088.8642553305
    normal_load_total                15780533.185531983
    normal_load_axle_front           5260177.728510661     <-- 新增命名
    normal_load_front_axle           5260177.728510661     <-- 历史拼写，同值
    normal_load_axle_middle          5260177.728510661     <-- 新增命名
    normal_load_middle_axle          5260177.728510661     <-- 历史拼写（同一规则生成）
    normal_load_axle_rear            5260177.728510661     <-- 新增命名
    normal_load_rear_axle            5260177.728510661     <-- 历史拼写，同值
    normal_load_left_side            7890266.592765992
    normal_load_right_side           7890266.592765992
    load_transfer_front_minus_rear   0.0
    load_transfer_right_minus_left   0.0
  summary.total          15780533.185531983
  summary.axle_loads     {'front': 5260177.728510661, 'middle': 5260177.728510661, 'rear': 5260177.728510661}
  summary.side_loads     {'left': 7890266.592765992, 'right': 7890266.592765992}
  summary.placements     ('front', 'middle', 'rear')
  summary.middle_axle    5260177.728510661
  summary.single_axle    AttributeError: this summary carries no 'single' placement; it carries front, middle, rear
```

三个新命名通道**都出现**，且各自的值等于该放置两个轮端之和的实测值（三轴对称布局下三者相等，均为 `5260177.728510661`；`5260177.728510661 * 3 == 15780533.185531983`，即 `normal_load_total`）。

对照：改造前三轴直接抛 `ValueError: wheel loads must contain exactly the four vehicle corners`，一个通道都拿不到（`raw/four_wheel_before.json` 的 `assembly_three_axle.metrics.error`）。

## 3. 改名实验（本条判据）

放置名不是来自任何常量，而是来自**轮端自己的名字**：`<placement>_<side>` 去掉 `_<side>` 后缀即放置名（`report/wheel_loads.py:59-61`；装配侧的同一约定见 `connections/policy.py:408-419 _ends_of`）。

实验一（`report/` 层，实测原文同上 `raw/three_axle_dump.txt` 末段）：

```
== rename experiment (bogie / trailer)
  输入 {bogie_left: 250, bogie_right: 150, trailer_left: 100, trailer_right: 100}
  normal_load_bogie_left            250.0
  normal_load_bogie_right           150.0
  normal_load_trailer_left          100.0
  normal_load_trailer_right         100.0
  normal_load_total                 600.0
  normal_load_axle_bogie            400.0
  normal_load_bogie_axle            400.0
  normal_load_axle_trailer          200.0
  normal_load_trailer_axle          200.0
  normal_load_left_side             350.0
  normal_load_right_side            250.0
  load_transfer_bogie_minus_trailer 200.0
  load_transfer_right_minus_left    -100.0
  summary.placements ('bogie', 'trailer')
  summary.bogie_axle 400.0
```

- 通道名跟着放置名走：`bogie` / `trailer`；
- 输出里**没有任何** `front` / `middle` / `rear` 字样（`normal_load_total`、`normal_load_left_side` 等聚合名不含放置名）；
- 差值通道名为 `load_transfer_bogie_minus_trailer`，即 `placements[0] - placements[-1]`，也由声明驱动。
- 反向对照：改造前同一组输入也抛 `ValueError: wheel loads must contain exactly the four vehicle corners`（`raw/four_wheel_before.json` 的 `rename_experiment.metrics.error`）。

实验二（`outputs/builtin.py` 声明层，`tests/outputs/test_load_outputs.py::test_the_declared_channels_follow_the_declared_wheel_ends`）：把声明层的 `_WHEELS` 换成 `bogie_*` / `trailer_*`，`_load_channel_names()` 产出的 13 个声明名整批改名（含 `load_transfer_bogie_minus_trailer`）。若声明是一张写死的名单，这条测试会失败。

## 4. 落到测试

`packages/suspension_multibody/tests/metrics/test_placement_channels.py`（新增，7 个用例）：

| 用例 | 断言 |
|---|---|
| `test_a_three_axle_vehicle_gets_one_channel_per_placement` | 三个 `normal_load_axle_{p}` 都在，且等于该放置两轮端之和；`load_transfer_front_minus_rear` 仍存在 |
| `test_a_three_axle_summary_answers_the_middle_axle` | `summary.placements == ('front','middle','rear')`；`middle_axle` 可读；`fourth_axle` 按名抛 `AttributeError` |
| `test_a_layout_whose_placements_are_not_front_middle_rear_is_named_after_them` | 改名实验：输出集合整表相等，且不含 `front`/`middle` |
| `test_the_aggregates_follow_the_loads_rather_than_a_corner_count` | 单轮只出 `_single` / `_left_side`，不出 `_right_side`；`right_side` 抛 `AttributeError` |
| `test_a_name_that_states_no_side_is_refused_by_name` | 无侧后缀的名字、`NaN`、空表分别按名拒绝 |
| `test_the_four_wheel_table_keeps_the_arithmetic_it_had` | 4 轮 13 个通道的**名字顺序**与逐项字面累加式 |
| `test_the_third_axle_of_a_six_wheel_run_is_a_real_placement_not_a_rounding` | 纵向加速度下 first/last 放置的通道前减后增、transfer 变负、总载荷仍守恒 |

`packages/suspension_multibody/tests/outputs/test_load_outputs.py`（新增，5 个用例）：声明层的名字、顺序、`reads`、改名实验，以及「`normal_load_axle_middle` 在 4 轮声明下**不**存在」这条边界登记。

## 5. 未被 3 轴覆盖到的部分（登记）

`outputs/builtin.py` 的 `RIG_OUTPUTS`（`:220` 起）与 `minimum_unit_outputs`（`wheel_loads` 分支）只声明四个 `wheel_load_<corner>` 输入，所以 3 轴在本行只是 `report/` 层的运行期能力，声明层仍止于四角。原因与边界见 `raw/dynamic_channel_registration.md` 第 6 节。
