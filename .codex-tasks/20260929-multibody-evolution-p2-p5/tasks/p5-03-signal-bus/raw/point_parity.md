# p5-03 判据 (b)：测点与结果文档逐项对照（不是新算一遍）

## 两侧原文（本行实跑，真实 run）

```
$ uv run --no-sync python .codex-tasks/.../p5-03-signal-bus/raw/bus_probe.py

wheel_speed(front_wheel_hub_L) via bus  = [ 0. 12.  0.]
                via block    = [ 0. 12.  0.]
identical: True

body_acceleration(chassis) via bus = [0. 0. 0.]
                          via block = [0. 0. 0.]
identical: True

tire_vertical_load(front_left) via bus = [0.], via block col4 = 0.0
identical: True
```

**轮速那一对是非零的**（`[0, 12, 0]`，来自 case 的 `initial_wheel_speeds=12 rad/s`），
所以这三对相等**不是**「两边都恰好为零」的假证据。

## 实现上为何不可能不一致

总线**不重算**：`MeasurementChannel.read` 的整个过程是

```python
block = _block_of(raw, self.block)          # raw.named_blocks[name]
row   = block[sample, index, :]             # 该实体该样本的行
return row[self.columns]                    # 该通道占的列
```

即**同一个 numpy 数组的同一段切片**。`np.array_equal` 恒真不是巧合，是构造使然——
与设计目标一致：总线的价值是「给这块数据起个名字」，不是「再算一遍」。

**没有第二条计算路径**：`signal_bus.py` 只 import `numpy` 与 stdlib；
它不 import `results/`、不 import solver、不 import native（可由本行实跑的分层门验证）。

## 换算

无。三个通道都**直接取原单位**（rad/s、mm/s^2、N），没有乘系数、没有重采样、没有符号翻转。
