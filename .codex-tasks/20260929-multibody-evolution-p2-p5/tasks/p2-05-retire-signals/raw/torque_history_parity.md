# p2-05 证据 (b)：力矩时程一致性

> 实测日期 2026-10-01。

## 1. 默认（`none`）路径：与 p2-01 快照逐项一致

同一模型、同一工况（`brake=0.25`、`drive=0`、`front_brake_bias=0.6`、2 个采样点）实测：

```
default brake @0.25: {'front_left': 0.75, 'front_right': 0.75, 'rear_left': 0.5, 'rear_right': 0.5}
default wheel_torque @0.25: {'front_left': 0.0, 'front_right': 0.0, 'rear_left': 0.0, 'rear_right': 0.0}
samples: 2
```

口径：`_build_wheel_torque_signals` 的 `share` 在该轴制轮间均分，故前轮得
`0.6/2 = 0.3`、后轮得 `0.4/2 = 0.2`，乘上制动压力与力矩上限的比得到上表数值。

**8 个冻结用例的逐位结论**（硬门，不是容差）：

```
$ uv run --no-sync python .../case_parity_check.py --family vehicle_dynamic
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
OK: 1 families accepted
```

```
$ uv run --no-sync python .../dynamic_hash_sentinel.py --check
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
```

差值 **0**（逐位一致，非容差内一致）。`vehicle_dynamics_baseline/sha256.json` 未被重录
（`git status --short -- packages/suspension_multibody/tests/data/` 为空）。

## 2. opt-in 路径：用 p2-11 的可收敛装置（本行不要求整车夹具收敛）

本行**不声称**整车夹具上的非零力矩响应：那条由 `p2-11` 用专用两体装置独立验收，实测
code-10 首样本范数 `1.0 N·m`（= `max_torque`）、逐样本严格等大反向（`atol=0`）、
末样本 `0.0`、对照组 0 行。见 `tasks/p2-11-torque-response/raw/non_zero_response.md`。

## 3. 结论

- `none` 路径：与冻结基线**逐位一致**（8 cases + combined sha256），无任何登记项。
- opt-in 路径：元素增益、需求表、文档 role 三者一一对应（见 `contract_tables.md`）。
- 无「超容差且无物理等价判据」的情形，故本行不需要按 D5 登记任何基线变化，也未使用
  物理等价判据通道。
