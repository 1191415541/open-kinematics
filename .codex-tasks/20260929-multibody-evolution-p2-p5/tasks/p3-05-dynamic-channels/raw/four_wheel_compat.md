# p3-05 证据 (b)：4 轮向后兼容逐项一致（硬门）

## 1. 方法

- 改造**前**：`raw/channel_probe.py --emit raw/four_wheel_before.json`（EXIT=0）。
- 改造**后**：同一条命令写 `raw/four_wheel_after.json`（EXIT=0）。
- 比较：`raw/channel_probe.py --compare raw/four_wheel_before.json raw/four_wheel_after.json`（EXIT=0）。

探针（`raw/channel_probe.py`）把每个已发布数值写成 `repr`，因此比较是**逐位**的，不是近似；比较只对**改造前存在的**名字与属性做，新增名字不参与判定，但会检查「既有名字是否少了」与「既有相对顺序是否变了」。

两个用例都跑：
- `fake_four_wheel`：`front_left 100 / front_right 120 / rear_left 80 / rear_right 90`（`tests/metrics` 用的那组）；
- `assembly_four_wheel`：`tests/conftest.py::full_vehicle_model` 经 p3-04 的 `compute_static_wheel_loads` 实解出来的四个反力。

## 2. 结论

`raw/channel_probe_compare.txt` 末行：

```
moved channels: 0
OK: every pre-existing channel kept its name, order and value
```

逐项摘录（两段合计 42 行 `OK`：各 11 行 metrics + 8 行 summary + 1 行顺序 + 1 行缺失检查；无一行 `MOVE`）：

```
== fake_four_wheel
  OK   metrics.load_transfer_front_minus_rear before=50.0                       after=50.0
  OK   metrics.load_transfer_right_minus_left before=30.0                       after=30.0
  OK   metrics.normal_load_front_axle     before=220.0                      after=220.0
  OK   metrics.normal_load_front_left     before=100.0                      after=100.0
  OK   metrics.normal_load_front_right    before=120.0                      after=120.0
  OK   metrics.normal_load_left_side      before=180.0                      after=180.0
  OK   metrics.normal_load_rear_axle      before=170.0                      after=170.0
  OK   metrics.normal_load_rear_left      before=80.0                       after=80.0
  OK   metrics.normal_load_rear_right     before=90.0                       after=90.0
  OK   metrics.normal_load_right_side     before=210.0                      after=210.0
  OK   metrics.normal_load_total          before=390.0                      after=390.0
  OK   summary.front_axle                 before=220.0                      after=220.0
  OK   summary.front_rear_delta           before=50.0                       after=50.0
  OK   summary.left_side                  before=180.0                      after=180.0
  OK   summary.rear_axle                  before=170.0                      after=170.0
  OK   summary.right_left_delta           before=30.0                       after=30.0
  OK   summary.right_side                 before=210.0                      after=210.0
  OK   summary.total                      before=390.0                      after=390.0
  OK   summary.wheel_loads                before={'front_left': 100.0, ...} after={'front_left': 100.0, ...}
  OK   legacy channel order before=[...11 names...] after=[...same 11 names...]
  OK   legacy channels missing after: []
```

```
== assembly_four_wheel
  OK   metrics.load_transfer_front_minus_rear before=9.313225746154785e-10      after=9.313225746154785e-10
  OK   metrics.load_transfer_right_minus_left before=-9.313225746154785e-10     after=-9.313225746154785e-10
  OK   metrics.normal_load_front_axle     before=7222177.728510661          after=7222177.728510661
  OK   metrics.normal_load_front_left     before=3611088.8642553305         after=3611088.8642553305
  OK   metrics.normal_load_front_right    before=3611088.86425533           after=3611088.86425533
  OK   metrics.normal_load_left_side      before=7222177.728510661          after=7222177.728510661
  OK   metrics.normal_load_rear_axle      before=7222177.72851066           after=7222177.72851066
  OK   metrics.normal_load_rear_left      before=3611088.86425533           after=3611088.86425533
  OK   metrics.normal_load_rear_right     before=3611088.86425533           after=3611088.86425533
  OK   metrics.normal_load_right_side     before=7222177.72851066           after=7222177.72851066
  OK   metrics.normal_load_total          before=14444355.457021322         after=14444355.457021322
  OK   summary.front_axle                 before=7222177.728510661          after=7222177.728510661
  OK   summary.front_rear_delta           before=9.313225746154785e-10      after=9.313225746154785e-10
  OK   summary.left_side                  before=7222177.728510661          after=7222177.728510661
  OK   summary.rear_axle                  before=7222177.72851066           after=7222177.72851066
  OK   summary.right_left_delta           before=-9.313225746154785e-10     after=-9.313225746154785e-10
  OK   summary.right_side                 before=7222177.72851066           after=7222177.72851066
  OK   summary.total                      before=14444355.457021322         after=14444355.457021322
  OK   legacy channel order before=[...] after=[...same...]
  OK   legacy channels missing after: []
```

注意 `9.313225746154785e-10`（1/2 ULP 级差值）与 `3611088.8642553305` vs `3611088.86425533` 这类末位差异**改造前后完全相同**——说明求和顺序没有被改动（若把 `front+rear` 换成别的等价结合方式，这类值会变）。

## 3. 命名并存（不是只改名）

4 轮下 `normal_load_axle_front` 与 `normal_load_front_axle` **同时**发布，值相同；`_axle_rear` / `_rear_axle` 同理。列表见 `raw/dynamic_channel_registration.md` 第 3.1 节，实测原文见 `raw/channel_probe_emit_after.txt`。

## 4. 下游消费者

| 消费者（不在本行写范围） | 用到的名字 | 状态 |
|---|---|---|
| `vehicle/static_loads.py:121 summary` | `summarize_wheel_loads(self.wheel_loads)` | 未改；4 轮下 `.total` 等逐位一致 |
| `report/time_domain_physics.py:99` | `summarize_wheel_loads(loads)` | 同上 |
| `tests/physics/test_vehicle_physics.py` | `summary.total` / `.front_axle` / `.rear_axle` / `.left_side` / `.right_side` / `.right_left_delta` | 通过（V1 全量快速集） |
| `tests/physics/test_static_loads.py:385` | `summary.total` | 通过 |
| `report/__init__.py:47` | 再导出 `WheelLoadSummary` / `summarize_wheel_loads` | 名字未变 |

## 5. pytest

`tests/metrics` + `tests/outputs` 全跑：**75 passed**（EXIT=0，原文见 `run_log.md` V1）。

## 6. 报表字段名集合变化（D5 口径逐项登记）

| 文件 | 步骤 | 前 | 后 | 物理等价判据 |
|---|---|---|---|---|
| `report/wheel_loads.py` → `wheel_load_metrics` 表 | 4 轮 | 11 个通道 | 13 个通道（+2） | 新增两项是既有 `normal_load_front_axle` / `normal_load_rear_axle` 的**同值别名**，不改变任何既有读数；既有 11 项逐位不变（第 2 节） |
| `outputs/builtin.py` 派生输出声明 | 4 轮 | 11 个 `normal_load_*`/`load_transfer_*` 声明 | 13 个（+2 同值别名） | 同上 |
| `tests/data/kc_baseline/**`、`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json` | — | — | **逐字节未变** | 未重录；`dynamic_hash_sentinel.py --check` sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` 与冻结值一致（EXIT=0，见 `run_log.md` V3） |
