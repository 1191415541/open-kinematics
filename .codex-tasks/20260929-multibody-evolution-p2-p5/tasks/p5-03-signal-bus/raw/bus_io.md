# p5-03 判据 (a)(d)：读测点 + 写执行器，双向各有断言

总线模块：`packages/suspension_multibody/src/suspension_multibody/signal_bus.py`
测试：`packages/suspension_multibody/tests/api/test_signal_bus.py`（13 用例）

## (a) 读测点：轮速与车身加速度各 ≥1

| 通道 | 块 | 列 | 单位 | 声明 |
|---|---|---|---|---|
| `wheel_speed` | `body_state` | `10:13`（`omega` 三分量） | rad/s | `wheel_spin_rate` |
| `body_acceleration` | `body_state` | `13:16`（`a` 三分量） | mm/s^2 | `body_acceleration` |
| `tire_vertical_load` | `tire_output` | `4` | N | `tire_force` |

**列不是猜的**：`body_state` 的行布局来自内核自己的写入器
`packages/suspension_kernel/cpp/src/output/kernel_output.cpp:24-35`：
`position(0..2) / quaternion(3..6) / v(7..9) / omega(10..12) / a(13..15) / alpha(16..18)`。
轮胎列 `4` 与 `results/kc_state.py::_TIRE_LOAD_COLUMN` 同名同值。

## (a) 写执行器：可变阻尼与电机力矩各 ≥1

| 通道 | 路径 | 单位 |
|---|---|---|
| `variable_damping_L` | `("elements", "damper_L", "parameters", "compression_damping")` | N*s/mm |
| `motor_torque_FL` | `("body_wrench", "front_wheel_hub_L", "moment")` | N*mm |

写入**返回新文档、不改原文档**（断言在测试里）；路径不存在时按需创建中间映射。

## (d) 双向读写断言清单（不是同一条测试里互相抵消）

**读（3 条断言，各自对照结果文档）**
1. `test_a_wheel_speed_read_is_the_result_documents_own_number`：`np.array_equal(bus 读, body_state[-1, i, 10:13])`，且断言**和不为零**（`[0,12,0]`）
2. `test_a_body_acceleration_read_is_the_result_documents_own_number`：同上，取 `chassis` 的 `13:16`
3. `test_a_tire_load_read_is_the_tires_own_column`：`tire_output[-1, 0, 4]`

**写（2 条断言，各自检查写后文档与原文档）**
4. `test_a_write_edits_a_copy_and_leaves_the_original_alone`：写后 `42.0`、原文档仍 `1.0`、且不是同一对象
5. `test_the_motor_torque_write_lands_where_it_says_it_does`：`{"body_wrench": {"front_wheel_hub_L": {"moment": 1234.0}}}`

**拒绝路径（5 条）**：未点名实体 / 未知通道 / 非有限命令 / 路径穿过标量 / 未绑定 run 或文档。

## 实测输出

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/outputs packages/suspension_multibody/tests/results -q -p no:cacheprovider
133 passed in 5.90s
exit=0
```

探针原文（`raw/bus_probe_output.txt`）里可见三对 `identical: True`。
