# 接触frame修正的物理差异

- producer: `raw/vehicle_order_probe.py`；只读旧生产者与四个显式IR输入；不在生产入口使用旧算法。
- input: 现有 `tests.vehicle.test_native_vehicle._vehicle/_case`，0到1 ms、rack目标0到1 mm、无初始轮速、零重力；同一native二进制。
- result: `vehicle-order-probe.json`。
- 原始新文档与旧结果最大位置差 `2.531308496145357e-14 m`、四元数分量差 `8.016091474836635e-10`、角速度差 `3.177475227183601e-6 rad/s`、角加速度差 `0.0052045875222116456 rad/s²`。
- 单独对齐body顺序仍有差异；再对齐joint顺序仍有差异；仅把Tire接触frame恢复为旧hub后全部19列状态差为0。
- 原因：旧frame绑定自转hub，新frame声明在upright上；输入未施加轮速并不意味着求解中轴承没有相对spin。不能把整个状态要求逐位不变作为这个修正的验收。
- 旧frame仅出现在只读诊断oracle内；生产文档保持upright，未增加运行时兼容开关，未重录冻结基线，未调宽旧断言。
- 新spin场景必须继续运行其独立几何/力矩验收；其他同物理场景继续对原冻结数据比较。当前报告不表示完整生产切换的数值门已通过。
