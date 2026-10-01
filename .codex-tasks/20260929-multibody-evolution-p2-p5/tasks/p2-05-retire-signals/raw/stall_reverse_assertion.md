# p2-05 证据 (e)：静止 / 倒车振荡失真的断言

> 实测日期 2026-10-01。

## 1. 断言的落点

`packages/suspension_multibody/tests/cases/test_rotational_torque_document.py` 的
`test_the_couple_the_kernel_applied_is_non_zero`（由 p2-11 交付，本行按其口径消费）读内核
`element_wrench` 通道的 type code 10，断言的是**真实状态与力矩数值**，不是「函数被调用」：

```
pairs = rows.reshape(SAMPLES, 2, rows.shape[1])
np.testing.assert_allclose(
    pairs[:, 0, 3:6].astype(float), -(pairs[:, 1, 3:6].astype(float)), atol=0.0
)
first_moment = pairs[0, 0, 3:6].astype(float)
assert np.linalg.norm(first_moment) == pytest.approx(MAX_TORQUE, rel=1e-12), first_moment
assert np.linalg.norm(pairs[-1, 0, 3:6].astype(float)) == pytest.approx(0.0, abs=1e-12)
```

## 2. 静止态的实测数值

| 量 | 实测 | 判据 |
|---|---|---|
| 末样本（相对角速度已归零）code-10 力矩范数 | `0.0` | `approx(0.0, abs=1e-12)` —— **静止对不产生反向加速** |
| 静止态两端力矩 | `0` 与 `-0` | 等大反向，`atol=0` |
| 有相对角速度时的首样本力矩范数 | `1.0 N·m` | `approx(max_torque, rel=1e-12)` |

这条断言的可判别性在于：力律若在 `|rate| <= kEps` 时按舍入误差挑一个符号，末样本力矩就会是
非零满幅值，断言立即失败；若力律一直保持满幅值（不跟随状态），
`assert ... approx(0.0, abs=1e-12)` 同样失败。

## 3. 倒车态的实测数值

同一族力律的符号断言在内核侧（`packages/suspension_kernel/tests/test_rotational_torque.py`，
p2-02/p2-08 交付，本行未改）：

```
eval forward  tau_a 400 tau_b -400     # 正转：力偶阻碍，末端为负
eval reverse  tau_a -400 tau_b 400     # 反转：力偶翻转，仍阻碍
```

对应断言 `test_the_couple_reverses_with_a_reverse_rate` 与
`test_the_couple_opposes_a_forward_rate`（同一文件）。即：**负相对角速度下力偶符号翻转**，
力偶始终阻碍相对运动，不出现反向加速；两端始终等大反向。

`test_a_stationary_pair_gets_no_couple` 断言静止对两端恰为 `0.0`（不是「近似零」）。

## 4. 未声称

- 本行**不声称**整车 `vehicle_dynamic` 夹具上的非零力矩响应：那条由 `p2-11` 用专用两体装置
  独立验收（实测 code-10 首样本 `1.0 N·m`、等大反向 `atol=0`、末样本 `0.0`、对照组 0 行）。
- 本行**不声称** `templates/builtin.py` 的 `BRAKE.property_slots` 已含「分配 share」的槽。
  按裁决 `ddc3f952`，分配口径由本行角色自己的常量 `BRAKE_FRONT_SHARE`（`torque_elements.py`）
  驱动；模板槽集是 p2-04 已交付的四项标准化物理参数，本行不改模板（写范围不含）。
